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
std::array<uint8_t, 2> PWM_Pins = {
    pdm_board.LC_FAN_12V_ENABLE, pdm_board.LC_PUMP_12V_ENABLE};

// There are 16 PWM channels from 0 to 15.
// Not sure if it is necessary to have each pin operate on a separate channel.
// I think its fine as long as they are the same frequency, but I put each pin on a separate channel
// just in case.
std::array<uint8_t, 2> PWM_Chan = {0, 1};

// The amount the duty cycle increases with each call to Device.RampUp().
std::array<uint8_t, 2> PWM_Int = {3, 50};

// Track which device/s are trying to currently restart.
// This is necessary to not attempt another restart when
// the device is already restarting.
std::array<bool, 2> currentlyRestarting = {false, false};

// Initialize device class for each device that required PWM.
// Constructor requires passing the pwmPin, the channel, the percentage
// to run the device on (between 0 and 1), and the amount to increase the
// the duty cycle by with each call to Device.RampUp().
Device lc_fan_12v_device(PWM_Pins[0], PWM_Chan[0], 1, PWM_Int[0]);
Device lc_pump_12v_device(PWM_Pins[1], PWM_Chan[1], 1, PWM_Int[1]);

// Array to store devices.
std::array<Device, 2> Devices = {
    lc_fan_12v_device, lc_pump_12v_device};

/// CAN Signals ///

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
    read_timer.AddTimer(100, ControlBrakeLight);
}

void loop()
{
    can_bus.Tick();
    read_timer.Tick(millis());
}