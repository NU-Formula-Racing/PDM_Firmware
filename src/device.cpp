#include <Arduino.h>
#include <device.h>
#include "ESP32_FastPWM.h"

/**
 * @brief Constructor for setup.
 */
Device::Device(uint8_t pwmPin, uint8_t channel, float percent, uint8_t pwmInterval)
{
    /// PWM specific parameters. ///
    PWM_PIN = pwmPin;
    // Make sure percent specified is valid.
    if (percent < 0 || percent > 1)
    {
        percent = 1;
    }
    percentage = percent;
    dutyCycle = 0.0f;
    frequency = 2000.0f;
    pwmResolution = 8;
    /// Handle Restarting Device ///
    // Check that pwmInterval makes sense.
    if (pwmInterval < 1 || pwmInterval > 255)
    {
        pwmInterval = 0;
    }
    PWM_INTERVAL = pwmInterval;
    standoffTime = 5000;
    numRestarts = 3;
    currentRestarts = 0;
    startTime = 0;
    elapsedTime = 0;
    deviceOff = false;

    // Assigns PWM parameters.
    PWM_Instance = new ESP32_FAST_PWM(pwmPin, frequency, dutyCycle, channel, pwmResolution);
}

/**
 * @brief Update the time since the device has last attempted a
 * restart if the device is off.
 */
void Device::UpdateTime()
{
    if (deviceOff)
    {
        elapsedTime = millis() - startTime;
    }
}

/**
 * @brief Record when the device is turned off.
 */
void Device::RecordTime()
{
    deviceOff = true;
    startTime = millis();
}

/**
 * @brief Utilized in deciding when to actually attempt a restart.
 * @return Returns boolean of whether or not to attempt a restart.
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
 */
void Device::RampUp()
{
    // Turn on device.
    if (dutyCycle + PWM_INTERVAL <= 100 * percentage)
    {
        PWM_Instance->setPWM(PWM_PIN, frequency, dutyCycle + PWM_INTERVAL);
        dutyCycle += PWM_INTERVAL;
        // Serial.println(dutyCycle);
    }
}

/**
 * @brief Change the tracking state of a device.
 * @param state: new tracking state
 */
void Device::SetDeviceOff(bool state)
{
    deviceOff = state;
}