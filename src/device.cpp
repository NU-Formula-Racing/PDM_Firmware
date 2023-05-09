#include <Arduino.h>
#include <device.h>
#include "ESP32_FastPWM.h"

/**
 * @brief Constructor for setup.
 */
Device::Device(uint8_t pwmPin, uint8_t channel)
{
    /// PWM specific parameters. ///
    PWM_PIN = pwmPin;
    percentage = 1;
    dutyCycle = 0.0f;
    frequency = 1000.0f;
    pwmResolution = 8;
    /// Handle Restarting Device ///
    PWM_INTERVAL = 25;
    standoffTime = 5000;
    numRestarts = 3;
    currentRestarts = 0;
    startTime = 0;
    elapsedTime = 0;
    deviceOff = false;

    // Creates PWM instance.
    ESP32_FAST_PWM *PWM_Instance;
    // Assigns PWM parameters.
    PWM_Instance = new ESP32_FAST_PWM(pwmPin, frequency, dutyCycle, channel, pwmResolution);
}

/**
 * @brief Update the time since the device has last attempted a
 * restart.
 */
void Device::UpdateTime()
{
    if (deviceOff)
    {
        elapsedTime = millis() - startTime;
    }
}

/**
 * @brief Update the bool that tracks the state of the device,
 * and record when the device is turned off.
 */
void Device::DeviceOff()
{
    deviceOff = true;
    startTime = millis();
}

/**
 * @brief Returns boolean to attempt a restart.
 */
bool Device::AttemptRestart()
{
    // Device is on, no restart required.
    if (!deviceOff)
    {
        return false;
    }
    // Check if we have already performed the
    // specified number of restarts.
    if (currentRestarts >= numRestarts)
    {
        return false;
    }
    // Perform a restart if the elapsed time
    // is equal to the standoff time.
    // We have to update the start_time
    // and increase num_restarts.
    if (elapsedTime >= standoffTime)
    {
        startTime = millis();
        numRestarts++;
        return true;
    }
    // Not enough time has elapsed.
    return false;
}

/**
 * @brief Slowly turn on device to specified percentage.
 * @param index: index of pin in PWM_Pins[] array
 * @param percentage: power of device (needs to be value between 0 and 1, for example 0.5 is 50% power)
 * @return void
 */
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