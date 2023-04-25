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
    const uint16_t kCANIdFB{0x410};
    const uint16_t kCANIdBB{0x411};

    /// Pins ///

    // Sensor Pin Defines
    static constexpr int FIVEV_CSENSE = 36;
    static constexpr int TWELVEV_CSENSE = 39;
    static constexpr int VBAT_RAIL_CSENSE = 34;
    static constexpr int AC_FAN_12V_CSENSE = 35;
    static constexpr int LC_FAN_12V_CSENSE = 32;
    static constexpr int LC_PUMP_12V_CSENSE = 33;
    static constexpr int TWELVEV_HSD1_CSENSE = 25;
    static constexpr int TWELVEV_HSD2_CSENSE = 26;
    static constexpr int VBAT_CSENSE = 27;
    static constexpr int VBAT_VSENSE = 14;

    // Enable Pin Defines
    static constexpr int TWELVEV_HSD1_CSENSE_ENABLE = 23;
    static constexpr int TWELVEV_HSD2_CSENSE_ENABLE = 22;
    static constexpr int AC_FAN_12V_ENABLE = 18;
    static constexpr int LC_FAN_12V_ENABLE = 19;
    static constexpr int LC_PUMP_12V_ENABLE = 21;

private:
};
