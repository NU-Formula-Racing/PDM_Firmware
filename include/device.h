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

    // Time between restarts (ms).
    unsigned long standoff_time = 5000;
    // Number of restarts to attempt.
    uint8_t num_restarts = 3;
    // Restarts performed.
    uint8_t current_restarts = 0;

    // Time device turned off.
    unsigned long start_time;
    // Time since device has been off.
    unsigned long elapsed_time;
    // Latest restart time.
    unsigned long restart_time;

    // Boolean to track the state of the device.
    bool device_off = false;

    // Methods
    void UpdateTime();
    void DeviceOff();
    bool AttemptRestart();

private:
};