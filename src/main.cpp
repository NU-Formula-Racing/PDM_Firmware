#include <Arduino.h>
#include "ESP32_FastPWM.h"
#include "pdm.h"
#include "device.h"
#include "virtualTimer.h"
#include <array>

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
// Look at RampUp function, increase DutyCycle by this specified
// amount every time.
// #define PWM_INTERVAL 25

// Specify all the device pins that will need to controlled with PWM.
std::array<uint8_t, 5> PWM_Pins = {
    pdm_board.AC_FAN_12V_ENABLE, pdm_board.LC_FAN_12V_ENABLE, pdm_board.LC_PUMP_12V_ENABLE,
    pdm_board.TWELVEV_HSD1_ENABLE, pdm_board.TWELVEV_HSD1_ENABLE};

// There are 16 PWM channels from 0 to 15.
// Not sure if it is necessary to have each pin operate on a separate channel.
// I think its fine as long as they are the same frequency, but I put each pin on a separate channel
// just in case.
std::array<uint8_t, 5> PWM_Chan = {0, 1, 2, 3, 4};

// Initialize device class for each device that required PWM.
// Constructor requires passing the pwmPin and the channel.
// This allows us to retry to turn on the device after it has been
// shut off.
Device ac_fan_12v_device(PWM_Pins[0], PWM_Chan[0]);
Device lc_fan_12v_device(PWM_Pins[1], PWM_Chan[1]);
Device lc_pump_12v_device(PWM_Pins[2], PWM_Chan[2]);
Device hsd1_device(PWM_Pins[3], PWM_Chan[3]);
Device hsd2_device(PWM_Pins[4], PWM_Chan[4]);

// Array to store devices.
std::array<Device, 5> Devices = {
    ac_fan_12v_device, lc_fan_12v_device, lc_pump_12v_device,
    hsd1_device, hsd2_device};

// Duty cycle basically defines the percentage a device is off vs on.
// It is a parameter of the square wave, for example 50% duty cycle means a device
// spends half the time on and the other half off.
// For our use of PWM, we will chance the duty cycle from 0 to 255 or 255 to 0.
// Increasing the duty cycle to 255 turns on a device and decreasing the duty cycle
// to 0 turns off a device.
// We have a separate duty cycle for each pin because they're at different
// states at the same time.
// std::array<float, 5> dutyCycles = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

// PWM Frequency is the count of PWM interval periods per second, expressed in Hertz (Hz).
// Mathematically, the frequency is equal to the inverse of the interval period's length (PWM_Frequency = 1 / PWM_Interval_Period).
// This is just another property of the square wave, different devices can handle different frequencies.
// For now, we assume that all devices can operate at the same frequency.
// std::array<float, 5> frequency = {1000.0f, 1000.0f, 1000.0f, 1000.0f, 1000.0f};

// Max resolution is 20-bit
// Resolution 65536 (16-bit) for lower frequencies, OK @ 1K
// Resolution  4096 (12-bit) for lower frequencies, OK @ 10K
// Resolution  1024 (10-bit) for higher frequencies, OK @ 50K
// Resolution  256  ( 8-bit)for higher frequencies, OK @ 100K, 200K
// Resolution  128  ( 7-bit) for higher frequencies, OK @ 500K
// With a PWM resolution of 8 we can set the duty cycle between 0 and 255..
// int PWM_resolution = 8;

// Create PWM instance for each pin.
// ESP32_FAST_PWM *PWM_Instance[PWM_Pins.size()];

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
 * @brief Control when the brake light turns on.
 * The brake light is in PSI units and it can range from 0 to 3000 PSI.
 * @param[in] limit: Pressure value (PSI) to turn on brake light.
 * @return void
 */
void ControlBrakeLight(uint8_t limit)
{
    // Turn on device.
    if (front_brake_press_rx_signal > limit || back_brake_press_rx_signal > limit)
    {
        digitalWrite(pdm_board.TWELVEV_HSD1_ENABLE, HIGH);
    }
    // Turn off device
    else
    {
        digitalWrite(pdm_board.TWELVEV_HSD1_ENABLE, LOW);
    }
}

/**
 * @brief Turn on or off VBat Rail, 5V Rail, and 12V Rail if they are below or above the software disable limit.
 * These devices are on by default, ToggleDevice turns them on or off
 * based on the limit.
 * @param pin: pin on ESP32 (all pins found in pdm.h so use pdm_board)
 * @param current: current of device in Amps
 * @param limit: software disable limit in Amps
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

// /**
//  * @brief Slowly turn on device to specified percentage.
//  * @param index: index of pin in PWM_Pins[] array
//  * @param percentage: power of device (needs to be value between 0 and 1, for example 0.5 is 50% power)
//  * @return void
//  */
// void RampUp(uint8_t index, float percentage)
// {
//     // Turn on device.
//     // ramp for a specified amount of time
//     // make timer and stop when criteria is met
//     // for (float dutyCycle = dutyCycles[index]; dutyCycle <= 255 * percentage; dutyCycle++)
//     // {
//     //     PWM_Instance[index]->setPWM(PWM_Pins[index], frequency[index], dutyCycle);
//     //     delay(PWM_INTERVAL);
//     // }

//     // Turn on device.
//     if (dutyCycles[index] <= 255 * percentage)
//     {
//         PWM_Instance[index]->setPWM(PWM_Pins[index], frequency[index], dutyCycles[index] + PWM_INTERVAL);
//     }
// }

/**
 * @brief Turn on or off devices if they are below or above the software disable limit.
 * These devices are on by default, ToggleDevice turns them on or off
 * based on the limit.
 * @param index: index of pin in PWM_Pins[] array
 * @param current: current of device in Amps
 * @param limit: software disable limit in Amps
 * @param percentage: power of device (needs to be value between 0 and 1, for example 0.5 is 50% power)
 * @return void
 */
void RampDevice(uint8_t index, float current, uint8_t limit, float percentage = 1)
{
    // Turn off device.
    if (current > limit)
    {
        digitalWrite(PWM_Pins[index], LOW);

        // Change the stored state and start the timer.
        Devices[index].DeviceOff();
    }
    // Only attempt to restart if there is no current.
    if (current == 0)
    {

        // time to restart
        // 5 seconds stand off
        // abstracted device class
        // retry count
        // tried and failed multiple times
        // three distinct
        // if it hasnt gone away then restart

        // Restart device based on if the criteria.
        if (Devices[index].AttemptRestart())
        {
            // Turn on device
            // Invalid percentage.
            if (percentage < 0 || percentage > 1)
            {
                percentage = 1;
            }
            // Otherwise percentage is valid.
            // Ramp up device to percentage.
            // read_timer.AddTimer(1, RampUp(index, percentage), VirtualTimer::Type::kFiniteUse, (255 * percentage) / PWM_INTERVAL + 1);
        }
    }
}

/**
 * @brief Update times for all devices that are turned off.
 */
void UpdateTime()
{
    for (uint8_t index = 0; index < Devices.size(); index++)
    {
        Devices[index].UpdateTime();
    }
}

/**
 * @brief Slowly turn on all devices.
 * @param percentage: power of device (needs to be value between 0 and 1, for example 0.5 is 50% power)
 * @return void
 */
void StartUpRamp(float percentage = 1)
{
    // float percentage = 1;

    // Invalid percentage.
    if (percentage < 0 || percentage > 1)
    {
        percentage = 1;
    }

    // Otherwise percentage is valid.
    for (uint8_t index = 0; index < Devices.size(); index++)
    {
        read_timer.AddTimer(100, [index](){Devices[index].RampUp();});
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
    // RampDevice(0, air_fan_signal, 20);
    RampDevice(0, air_fan_signal, 20);
    // = LC_FAN_12V_Current && Enable/Disable
    liquid_fan_signal = pdm_board.ReadCurrent(pdm_board.LC_FAN_12V_CSENSE, 0);
    // RampDevice(1, liquid_fan_signal, 20);
    RampDevice(1, liquid_fan_signal, 20);
    // = LC_PUMP_12V_Current && Enable/Disable
    liquid_pump_signal = pdm_board.ReadCurrent(pdm_board.LC_PUMP_12V_CSENSE, 0);
    // RampDevice(2, liquid_pump_signal, 20);
    RampDevice(2, liquid_pump_signal, 20);
    // = TWELVEV_HSD1_Current && Enable/Disable
    hsd_1_signal = pdm_board.ReadCurrent(pdm_board.TWELVEV_HSD1_CSENSE, 0);
    // RampDevice(3, hsd_1_signal, 1);
    RampDevice(3, hsd_1_signal, 1);
    // = TWELVEV_HSD2_Current && Enable/Disable
    hsd_2_signal = pdm_board.ReadCurrent(pdm_board.TWELVEV_HSD1_CSENSE, 0);
    // RampDevice(4, hsd_2_signal, 1);
    RampDevice(4, hsd_2_signal, 1);
}

// Turn on brake lights based on the given limit (PSI), anything
// greater than the value will cause the brake lights to turn on.
void BrakeLight()
{
    ControlBrakeLight(100);
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
    // for (uint8_t index = 0; index < PWM_Pins.size(); index++)
    // {
    //     // Assigns PWM frequency of 1.0 KHz and a duty cycle of 0%, channel, 8-bit resolution (0-255 value can inputted).
    //     PWM_Instance[index] = new ESP32_FAST_PWM(PWM_Pins[index], frequency[index], dutyCycles[index], PWM_chan[index],
    //                                              PWM_resolution);

    //     if (PWM_Instance[index])
    //     {
    //         PWM_Instance[index]->setPWM();
    //     }
    // }

    // Turn on all devices slowly to specified percentage,
    // must be between 0 and 1.
    StartUpRamp(1);

    // Initialize CAN bus.
    can_bus.Initialize(ICAN::BaudRate::kBaud1M);

    // Initialize our timer(s) to read each sensor.
    // read_timer.AddTimer(100, StartUpRamp);
    read_timer.AddTimer(100, UpdateTime);
    read_timer.AddTimer(100, ReadCurrents);
    read_timer.AddTimer(100, BrakeLight);
}

void loop()
{
    can_bus.Tick();
    read_timer.Tick(millis());
}