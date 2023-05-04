/**
 *
 * @brief Definitions and constants for each PWM device of the PDM board.
 *
 */
class Device
{
public:
    // Constructor
    Device();

    // Time device turned off.
    unsigned long start_time;
    // Time since device has been off.
    unsigned long elapsed_time;

    // Boolean to track the state of the device.
    bool device_off = true;

    // Methods

private:
};