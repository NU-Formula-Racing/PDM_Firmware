#include <Arduino.h>
#include <pdm.h>

/**
 * @brief Constructor to setup pins and other configurations.
 */
PDM::PDM()
{
    // Enable interrupts.
    // sei();
    // Specify both pins as an input.
    pinMode(FIVEV_CSENSE, INPUT);
    pinMode(TWELVEV_CSENSE, INPUT);
    pinMode(VBAT_RAIL_CSENSE, INPUT);
    pinMode(AC_FAN_12V_CSENSE, INPUT);
    pinMode(LC_FAN_12V_CSENSE, INPUT);
    pinMode(LC_PUMP_12V_CSENSE, INPUT);
    pinMode(TWELVEV_HSD1_CSENSE, INPUT);
    pinMode(TWELVEV_HSD2_CSENSE, INPUT);
    pinMode(VBAT_CSENSE, INPUT);
    pinMode(VBAT_VSENSE, INPUT);
    pinMode(AC_FAN_12V_ENABLE, OUTPUT);
    pinMode(LC_FAN_12V_ENABLE, OUTPUT);
    pinMode(LC_PUMP_12V_ENABLE, OUTPUT);
    pinMode(TWELVEV_HSD1_ENABLE, OUTPUT);
    pinMode(TWELVEV_HSD2_ENABLE, OUTPUT);
}

/**
 * @brief Reads the current in Amps.
 *
 * @return float
 */
float PDM::ReadCurrent(uint8_t pin, uint8_t sens = 0)
{
    float curr;
    if (sens = 0)
    {
        float curr = (0.0008 * analogRead(pin) + 0.1372) / 0.2;
    }
    if (sens = 1)
    {
        float curr = (0.0008 * analogRead(pin) - 0.41128) / 0.05;
    }
    return curr;
}

/**
 * @brief Reads the voltage in mV.
 *
 * @return float
 */
float PDM::ReadVoltage(uint8_t pin)
{
    return analogRead(pin) / 4096 * 3.3;
}
