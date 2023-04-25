#include <Arduino.h>
#include "ESP32_FastPWM.h"
#include "pdm.h"
#include "virtualTimer.h"

#define SERIAL_DEBUG

#ifdef ARDUINO_ARCH_ESP32
#include "esp_can.h"
// TX pin and RX pin as input.
ESPCAN can_bus{};
#endif

// Initialize board
PDM pdm_board;

// Structure for handling timers
VirtualTimerGroup read_timer;

/// PWM ///

#define _PWM_LOGLEVEL_ 4
// Time between each step of turning on and turning off a device.
// Look at TurnOn and TurnOff functions.
#define PWM_INTERVAL 5

// Specify all the pins that will need to controlled with PWM.
// Not sure if all pins are PWM compatible.
uint32_t PWM_Pins[] = {
    pdm_board.AC_FAN_12V_CSENSE, pdm_board.LC_FAN_12V_CSENSE, pdm_board.LC_PUMP_12V_CSENSE,
    pdm_board.TWELVEV_HSD1_CSENSE, pdm_board.TWELVEV_HSD1_CSENSE};

// There are 16 PWM channels from 0 to 15.
// Not sure if it is necessary to have each pin operate on a separate channel.
// I think its fine as long as they are the same frequency, but I put each pin on a separate channel
// just in case.
uint32_t PWM_chan[] = {0, 1, 2, 3, 4};

#define NUM_OF_PINS (sizeof(PWM_Pins) / sizeof(uint32_t))

// Duty cycle basically defines the percentage a device is off vs on.
// It is a parameter of the square wave, for example 50% duty cycle means a device
// spends half the time on and the other half off.
// For our use of PWM, we will chance the duty cycle from 0 to 255 or 255 to 0.
// Increasing the duty cycle to 255 turns off a device and decreasing the duty cycle
// to 0 turns off a device.
// We have a separate duty cycle for each pin because they're at different
// states at the same time.
float dutyCycles[] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

// PWM Frequency is the count of PWM interval periods per second, expressed in Hertz (Hz).
// Mathematically, the frequency is equal to the inverse of the interval period's length (PWM_Frequency = 1 / PWM_Interval_Period).
// This is just another property of the square wave, different devices can handle different frequencies.
// For now, we assume that all devices can operate at the same frequency.
float frequency[] = {1000.0f, 1000.0f, 1000.0f, 1000.0f, 1000.0f};

// Max resolution is 20-bit
// Resolution 65536 (16-bit) for lower frequencies, OK @ 1K
// Resolution  4096 (12-bit) for lower frequencies, OK @ 10K
// Resolution  1024 (10-bit) for higher frequencies, OK @ 50K
// Resolution  256  ( 8-bit)for higher frequencies, OK @ 100K, 200K
// Resolution  128  ( 7-bit) for higher frequencies, OK @ 500K
// With a PWM resolution of 8 we can set the duty cycle between 0 and 255..
int PWM_resolution = 8;

// Create PWM instance for each pin.
ESP32_FAST_PWM *PWM_Instance[NUM_OF_PINS];

/// CAN Signals ///
CANSignal<float, 0, 8, CANTemplateConvertFloat(0.02), CANTemplateConvertFloat(0), false> v5_rail_signal{};
CANSignal<float, 8, 8, CANTemplateConvertFloat(0.02), CANTemplateConvertFloat(0), false> v12_rail_signal{};
CANSignal<float, 16, 8, CANTemplateConvertFloat(0.02), CANTemplateConvertFloat(0), false> vbat_rail_signal{};
CANSignal<float, 24, 16, CANTemplateConvertFloat(0.002), CANTemplateConvertFloat(0), true> vbat_input_current_signal{};
CANSignal<float, 32, 8, CANTemplateConvertFloat(0.05), CANTemplateConvertFloat(10), false> vbat_input_voltage_signal{};
CANTXMessage<5> tx_message_1{can_bus, pdm_board.kCANId1, 6, 100, read_timer, v5_rail_signal, v12_rail_signal, vbat_rail_signal, vbat_input_current_signal, vbat_input_voltage_signal};

CANSignal<float, 0, 8, CANTemplateConvertFloat(0.1), CANTemplateConvertFloat(0), false> air_fan_signal{};
CANSignal<float, 8, 8, CANTemplateConvertFloat(0.1), CANTemplateConvertFloat(0), false> liquid_fan_signal{};
CANSignal<float, 16, 8, CANTemplateConvertFloat(0.1), CANTemplateConvertFloat(0), false> liquid_pump_signal{};
CANSignal<float, 24, 8, CANTemplateConvertFloat(0.02), CANTemplateConvertFloat(0), false> hsd_1_signal{};
CANSignal<float, 32, 8, CANTemplateConvertFloat(0.02), CANTemplateConvertFloat(0), false> hsd_2_signal{};
CANTXMessage<5> tx_message_2{can_bus, pdm_board.kCANId2, 5, 100, read_timer, air_fan_signal, liquid_fan_signal, liquid_pump_signal, hsd_1_signal, hsd_2_signal};

/// Brake Pressure ///
// Receive Front Brake Pressure (psi)
CANSignal<float, 0, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> front_brake_press_rx_signal{};
CANRXMessage<1> rx_message_1{can_bus, pdm_board.kCANIdFB, front_brake_press_rx_signal};
// Receive Back Brake Pressure (psi)
CANSignal<float, 0, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> back_brake_press_rx_signal{};
CANRXMessage<1> rx_message_1{can_bus, pdm_board.kCANIdFB, back_brake_press_rx_signal};

/**
 * @brief Control VBat Rail, 12VRail, 5VRail.
 * These devices are on by default, ToggleDevice turns them on or off
 * based on the limit.
 * @return void
 */
void ToggleDevice(uint8_t pin, float current, uint8_t limit)
{
    // Turn off device.
    if (current > limit)
    {
        digitalWrite(pin, LOW);
    }
    // Turn on device
    else
    {
        digitalWrite(pin, HIGH);
    }
}

/**
 * @brief Slowly turn on device to specified percentage.
 * @param[in] index: index of pin in PWM_Pins[] array
 * @param[in] percentage: power of device (needs to be value between 0 and 1, for example 0.5 is 50% power)
 * @return void
 */
void RampUp(uint8_t index, u_int8_t percentage)
{
    // Turn on device.
    for (int dutyCycle = dutyCycles[index]; dutyCycle <= 255 * percentage; dutyCycle++)
    {
        PWM_Instance[index]->setPWM(PWM_Pins[index], frequency[index], dutyCycles[index]);
        delay(PWM_INTERVAL);
    }
}

/**
 * @brief Slowly turn off device.
 * @param[in] index: index of pin in PWM_Pins[] array
 * @return void
 */
void RampDown(uint8_t index)
{
    // Turn off device.
    for (int dutyCycle = dutyCycles[index]; dutyCycle >= 0; dutyCycle--)
    {
        PWM_Instance[index]->setPWM(PWM_Pins[index], frequency[index], dutyCycles[index]);
        delay(PWM_INTERVAL);
    }
}

/**
 * @brief Control when device is ramping up and down based on the current.
 * @param[in] pin: pin on ESP32 (all pins found in pdm.h so use pdm_board)
 * @param[in] index: index of pin in PWM_Pins[] array
 * @param[in] current: current of device in Amps
 * @param[in] limit: software disable limit in Amps
 * @param[in] percentage: power of device (needs to be value between 0 and 1, for example 0.5 is 50% power)
 * @return void
 */
void RampDevice(uint8_t pin, uint8_t index, float current, uint8_t limit, uint8_t percentage = 1)
{
    // Valid percentage.
    if (percentage >= 0 && percentage <= 1)
    {
        // Ramp up to 1 or specified percentage.
        if (current > limit)
        {
            RampUp(index, percentage);
        }
        // Devices are off by default.
        else
        {
            RampDown(index);
        }
    }
    // Not valid percentage just set device to full power.
    else
    {
        if (current > limit)
        {
            RampUp(index, 1);
        }
        // Devices are off by default.
        else
        {
            RampDown(index);
        }
    }
}

// Read currents.
void ReadCurrents()
{
    //  = FIVEV_Current && Enable/Disable
    v5_rail_signal = pdm_board.ReadCurrent(pdm_board.FIVEV_CSENSE, 0);
    ToggleDevice(pdm_board.FIVEV_CSENSE, v5_rail_signal, 5);
    // = TWELVEV_Current && Enable/Disable
    v12_rail_signal = pdm_board.ReadCurrent(pdm_board.TWELVEV_CSENSE, 0);
    ToggleDevice(pdm_board.TWELVEV_CSENSE, v12_rail_signal, 5);
    // = VBAT_RAIL_Current && Enable/Disable
    vbat_rail_signal = pdm_board.ReadCurrent(pdm_board.VBAT_RAIL_CSENSE, 0);
    ToggleDevice(pdm_board.VBAT_RAIL_CSENSE, vbat_rail_signal, 5);
    // = VBAT_Current
    vbat_input_current_signal = pdm_board.ReadCurrent(pdm_board.VBAT_CSENSE, 1);
    // = VBAT_Voltage
    vbat_input_voltage_signal = pdm_board.ReadVoltage(pdm_board.VBAT_VSENSE);

    // = AC_FAN_12V_Current && Enable/Disable
    air_fan_signal = pdm_board.ReadCurrent(pdm_board.AC_FAN_12V_CSENSE, 0);
    RampDevice(pdm_board.AC_FAN_12V_CSENSE, 0, air_fan_signal, 20);
    // = LC_FAN_12V_Current && Enable/Disable
    liquid_fan_signal = pdm_board.ReadCurrent(pdm_board.LC_FAN_12V_CSENSE, 0);
    RampDevice(pdm_board.LC_FAN_12V_CSENSE, 1, liquid_fan_signal, 20);
    // = LC_PUMP_12V_Current && Enable/Disable
    liquid_pump_signal = pdm_board.ReadCurrent(pdm_board.LC_PUMP_12V_CSENSE, 0);
    RampDevice(pdm_board.LC_PUMP_12V_CSENSE, 2, liquid_pump_signal, 20);
    // = TWELVEV_HSD1_Current && Enable/Disable
    hsd_1_signal = pdm_board.ReadCurrent(pdm_board.TWELVEV_HSD1_CSENSE, 0);
    RampDevice(pdm_board.LC_PUMP_12V_CSENSE, 3, hsd_1_signal, 1);
    // = TWELVEV_HSD2_Current && Enable/Disable
    hsd_2_signal = pdm_board.ReadCurrent(pdm_board.TWELVEV_HSD1_CSENSE, 0);
    RampDevice(pdm_board.LC_PUMP_12V_CSENSE, 4, hsd_1_signal, 1);
}

//  turn on and off the brake light by reading brake pressure off CAN? I think it's HSD1

void setup()
{

#ifdef SERIAL_DEBUG
    // Initialize serial output
    Serial.begin(9600);
#endif

    /// PWM ///

    // Specify settings for each PWM instance
    for (uint8_t index = 0; index < NUM_OF_PINS; index++)
    {
        // Assigns PWM frequency of 1.0 KHz and a duty cycle of 0%, channel, 8-bit resolution (0-255 value can inputted).
        PWM_Instance[index] = new ESP32_FAST_PWM(PWM_Pins[index], frequency[index], dutyCycles[index], PWM_chan[index],
                                                 PWM_resolution);

        if (PWM_Instance[index])
        {
            PWM_Instance[index]->setPWM();
        }
    }

    // Initialize CAN bus.
    can_bus.Initialize(ICAN::BaudRate::kBaud1M);

    // Initialize our timer(s) to read each sensor every
    // specified amount of time (ms).
    read_timer.AddTimer(100, ReadCurrents);
}

void loop()
{
    // can_bus.Tick();
    read_timer.Tick(millis());
}
