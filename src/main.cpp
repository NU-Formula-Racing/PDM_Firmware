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

// Initialize PDM board.
PDM pdm_board;

// Initialize timer to call functions at
// specified frequencies.
VirtualTimerGroup read_timer;

/// PWM ///

// Useful Definitions

// Duty Cycle:
// Duty cycle basically defines the percentage a device is off vs on.
// It is a parameter of the square wave, for example 50% duty cycle means a device
// spends half the time on and the other half off.
// For our use of PWM, we will chance the duty cycle from 0 to 255.
// Increasing the duty cycle to 255 turns on a device and decreasing the duty cycle
// to 0 turns off a device.

// PWM Frequency:
// PWM Frequency is the count of PWM interval periods per second, expressed in Hertz (Hz).
// Mathematically, the frequency is equal to the inverse of the interval period's length
// (PWM_Frequency = 1 / PWM_Interval_Period).This is just another property of the square wave, different
// devices can handle different frequencies.

// PWM Resolution :
// Max resolution is 20-bit
// Resolution 65536 (16-bit) for lower frequencies, OK @ 1K
// Resolution  4096 (12-bit) for lower frequencies, OK @ 10K
// Resolution  1024 (10-bit) for higher frequencies, OK @ 50K
// Resolution  256  ( 8-bit)for higher frequencies, OK @ 100K, 200K
// Resolution  128  ( 7-bit) for higher frequencies, OK @ 500K
// With a PWM resolution of 8 we can set the duty cycle between 0 and 255.

// Specify all the device pins that will need to controlled with PWM.
std::array<uint8_t, 5> PWM_Pins = {
    pdm_board.AC_FAN_12V_ENABLE, pdm_board.LC_FAN_12V_ENABLE, pdm_board.LC_PUMP_12V_ENABLE,
    pdm_board.TWELVEV_HSD1_ENABLE, pdm_board.TWELVEV_HSD2_ENABLE};

// There are 16 PWM channels from 0 to 15.
// Not sure if it is necessary to have each pin operate on a separate channel.
// I think its fine as long as they are the same frequency, but I put each pin on a separate channel
// just in case.
std::array<uint8_t, 5> PWM_Chan = {0, 1, 2, 3, 4};

// The amount the duty cycle increases with each call to Device.RampUp().
std::array<uint8_t, 5> PWM_Int = {50, 50, 50, 50, 50};

// Track which device/s are trying to currently restart.
// This is necessary to not attempt another restart when
// the device is already restarting.
std::array<bool, 5> currentlyRestarting = {false, false, false, false, false};

// Initialize device class for each device that required PWM.
// Constructor requires passing the pwmPin, the channel, the percentage
// to run the device on (between 0 and 1), and the amount to increase the
// the duty cycle by with each call to Device.RampUp().
Device ac_fan_12v_device(PWM_Pins[0], PWM_Chan[0], 1, PWM_Int[0]);
Device lc_fan_12v_device(PWM_Pins[1], PWM_Chan[1], 1, PWM_Int[1]);
Device lc_pump_12v_device(PWM_Pins[2], PWM_Chan[2], 1, PWM_Int[2]);
Device hsd1_device(PWM_Pins[3], PWM_Chan[3], 1, PWM_Int[3]);
Device hsd2_device(PWM_Pins[4], PWM_Chan[4], 1, PWM_Int[4]);

// Array to store devices.
std::array<Device, 5> Devices = {
    ac_fan_12v_device, lc_fan_12v_device, lc_pump_12v_device,
    hsd1_device, hsd2_device};

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
// Receive Back Brake Pressure (psi)
CANSignal<float, 16, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> back_brake_press_rx_signal{};
CANRXMessage<2> rx_message_1{can_bus, pdm_board.kCANIdB, front_brake_press_rx_signal, back_brake_press_rx_signal};

// Brake Percentage
CANSignal<uint8_t, 8, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> brake_percentage_rx_signal{};
CANRXMessage<1> rx_message_2{can_bus, pdm_board.kCANIdP, brake_percentage_rx_signal};

////////////////////////////////

/// Functions ///

/**
 * @brief Control when the brake light turns on.
 * The brake pressure is in PSI units and it can range from 0 to 3000 PSI.
 * @return void
 */
void ControlBrakeLight()
{
    // Turn on device.
    if (front_brake_press_rx_signal > 50 || back_brake_press_rx_signal > 300 || brake_percentage_rx_signal > 5)
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

/**
 * @brief Turn on or off devices if they are below or above the software disable limit.
 * These devices are off by default.
 * @param index: index of pin in PWM_Pins[] array
 * @param current: current of device in Amps
 * @param limit: software disable limit in Amps
 * @return void
 */
void RampDevice(uint8_t index, float current, uint8_t limit)
{
    // Turn off device.
    if (current > limit)
    {
        digitalWrite(PWM_Pins[index], LOW);

        // Change the stored state and start the timer.
        Devices[index].RecordTime();
    }
    // Only attempt to restart if there is no current and
    // the device is not attempting to restart already.
    // Even thought the device is performing a ramp up the
    // current might still be 0.
    if (current == 0 && !currentlyRestarting[index])
    {
        // Set device state to off to start the timer.
        Devices[index].SetDeviceOff(true);

        // Restart device based on if the criteria.
        if (Devices[index].AttemptRestart())
        {
            currentlyRestarting[index] = true;
            read_timer.AddTimer(
                100,
                [index]()
                { Devices[index].RampUp(); },
                255 / PWM_Int[index] + 1);
        }
    }
    // If the device is actually ramping up
    // then there will be a small increase in current
    // so we can reset the currentlyRestarting variable.
    if (current >= 0.5)
    {
        currentlyRestarting[index] = false;
        Devices[index].SetDeviceOff(false);
    }
}

/**
 * @brief Update times for all devices that are turned off.
 * @return void
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
 * @return void
 */
void StartUpRamp()
{
    // Otherwise percentage is valid.
    for (uint8_t index = 0; index < Devices.size(); index++)
    {
        read_timer.AddTimer(
            100, [index]()
            { Devices[index].RampUp(); },
            255 / PWM_Int[index] + 1);
    }
}

/**
 * @brief Read currents and turn on/off devices.
 * @return void
 */
void ReadCurrents()
{
    // TODO: Check if v5 rail, v12 rail, and vbat rail turn off
    // with the specified pin.
    // I think its wrong because its a CSense pin but I doubt its
    // one of the enable pins.

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
    RampDevice(0, air_fan_signal, 20);

    Serial.println("AC FAN 12 V:");
    Serial.println(air_fan_signal);
    Serial.println("\n");

    // = LC_FAN_12V_Current && Enable/Disable
    liquid_fan_signal = pdm_board.ReadCurrent(pdm_board.LC_FAN_12V_CSENSE, 0);
    RampDevice(1, liquid_fan_signal, 20);
    // = LC_PUMP_12V_Current && Enable/Disable
    liquid_pump_signal = pdm_board.ReadCurrent(pdm_board.LC_PUMP_12V_CSENSE, 0);
    RampDevice(2, liquid_pump_signal, 20);
    // = TWELVEV_HSD1_Current && Enable/Disable
    hsd_1_signal = pdm_board.ReadCurrent(pdm_board.TWELVEV_HSD1_CSENSE, 0);
    RampDevice(3, hsd_1_signal, 1);
    // = TWELVEV_HSD2_Current && Enable/Disable
    hsd_2_signal = pdm_board.ReadCurrent(pdm_board.TWELVEV_HSD1_CSENSE, 0);
    RampDevice(4, hsd_2_signal, 1);
}

void setup()
{

#ifdef SERIAL_DEBUG
    // Initialize serial output
    Serial.begin(9600);
#endif

    // Turn on all devices.
    StartUpRamp();

    // Initialize CAN bus.
    can_bus.Initialize(ICAN::BaudRate::kBaud1M);

    // Initialize our timers.
    read_timer.AddTimer(100, UpdateTime);
    read_timer.AddTimer(100, ReadCurrents);
    read_timer.AddTimer(100, ControlBrakeLight);
}

void loop()
{
    can_bus.Tick();
    read_timer.Tick(millis());
    // digitalWrite(pdm_board.AC_FAN_12V_ENABLE, LOW);
}