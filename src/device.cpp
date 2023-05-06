#include <Arduino.h>
#include <device.h>

/**
 * @brief Constructor for setup.
 */
Device::Device()
{
    start_time = 0;
    elapsed_time = 0;
    restart_time = 0;
}

/**
 * @brief Update the time since the device has last attempted a
 * restart.
 */
void Device::UpdateTime()
{
    if (device_off)
    {
        elapsed_time = millis() - start_time;
    }
}

/**
 * @brief Update the bool that tracks the state of the device,
 * and record when the device is turned off.
 */
void Device::DeviceOff()
{
    device_off = true;
    start_time = millis();
}

/**
 * @brief Returns boolean to attempt a restart.
 */
bool Device::AttemptRestart()
{
    // Device is on, no restart required.
    if (!device_off)
    {
        return false;
    }
    // Check if we have already performed the
    // specified number of restarts.
    if (current_restarts >= num_restarts)
    {
        return false;
    }
    // Perform a restart if the elapsed time
    // is equal to the standoff time.
    // We have to update the start_time
    // and increase num_restarts.
    if (elapsed_time >= standoff_time)
    {
        start_time = millis();
        num_restarts++;
        return true;
    }
    // Not enough time has elapsed.
    return false;
}