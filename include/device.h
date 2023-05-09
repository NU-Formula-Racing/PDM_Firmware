/**
 *
 * @brief Definitions and constants for each PWM device of the PDM board.
 *
 */
class Device
{
public:
    /**
     * @brief Construct a new Device object.
     *
     * @param pwmPin: Device PWMpin that will control turning the device on or off.
     * @param channel: One of the 16 PWM channels from 0 to 15.
     */
    Device(uint8_t pwmPin, uint8_t channel);

    //// Methods ////

    /// PWM ///
    void RampUp();

    /// Handle Restarting Device ///
    void UpdateTime();
    void DeviceOff();
    bool AttemptRestart();

private:
    /// PWM specific parameters. ///

    // Specify device pin that will need to controlled with PWM.
    uint8_t PWM_PIN;

    // Percentage of power the device is running on.
    // (needs to be value between 0 and 1, for example 0.5 is 50% power)
    float percentage;

    // Duty cycle basically defines the percentage a device is off vs on.
    // It is a parameter of the square wave, for example 50% duty cycle means a device
    // spends half the time on and the other half off.
    // For our use of PWM, we will chance the duty cycle from 0 to 255 or 255 to 0.
    // Increasing the duty cycle to 255 turns on a device and decreasing the duty cycle
    // to 0 turns off a device.
    float dutyCycle;

    // PWM Frequency is the count of PWM interval periods per second, expressed in Hertz (Hz).
    // Mathematically, the frequency is equal to the inverse of the interval period's length (PWM_Frequency = 1 / PWM_Interval_Period).
    // This is just another property of the square wave, different devices can handle different frequencies.
    // For now, we assume that all devices can operate at the same frequency.
    float frequency;

    // Max resolution is 20-bit
    // Resolution 65536 (16-bit) for lower frequencies, OK @ 1K
    // Resolution  4096 (12-bit) for lower frequencies, OK @ 10K
    // Resolution  1024 (10-bit) for higher frequencies, OK @ 50K
    // Resolution  256  ( 8-bit)for higher frequencies, OK @ 100K, 200K
    // Resolution  128  ( 7-bit) for higher frequencies, OK @ 500K
    // With a PWM resolution of 8 we can set the duty cycle between 0 and 255..
    int pwmResolution;

    /// Handle Restarting Device ///

    // Time between each step of turning on and turning off a device.
    // Look at RampUp function, increase duty cycle by this specified
    // amount every time.
    float PWM_INTERVAL;

    // Time between restarts (ms).
    unsigned long standoffTime;
    // Number of restarts to attempt.
    uint8_t numRestarts;
    // Restarts performed.
    uint8_t currentRestarts;

    // Time device turned off or latest restart time.
    unsigned long startTime;
    // Time since device has been off.
    unsigned long elapsedTime;

    // Boolean to track the state of the device.
    bool deviceOff;
};