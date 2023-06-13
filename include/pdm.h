/**
 *
 * @brief Definitions and constants for the PDM board.
 *
 */
class PDM
{
public:
    // Constructor
    PDM();

    // Methods
    float ReadCurrent(uint8_t pin, uint8_t sens);
    float ReadVoltage(uint8_t pin);

    /// CAN Addresses ///
    // PDM
    const uint16_t kCANId1{0x500};
    const uint16_t kCANId2{0x501};
    // Brake Pressure
    const uint16_t kCANIdB{0x410};
    // Brake Percentage
    const uint16_t kCANIdP{0x300};

    /// Pins ///

    // Sensor Pin Defines

    // Enable Pin Defines
    static constexpr int TWELVEV_HSD1_ENABLE = 23;
    static constexpr int AC_FAN_12V_ENABLE = 18;  // now only has to be high
    static constexpr int LC_FAN_12V_ENABLE = 19;  // PWM
    static constexpr int LC_PUMP_12V_ENABLE = 21; // PWM
    static constexpr int KEEP_HIGH_PIN = 17;      // Has to be high because Jurgen is dumb
    static constexpr int FIVEV_ENABLE = 16;       // Has to be high
    static constexpr int VBAT_RAIL_ENABLE = 13;   // Has to be high

private:
};
