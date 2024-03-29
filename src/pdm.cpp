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
    pinMode(AC_FAN_12V_ENABLE, OUTPUT);
    pinMode(LC_FAN_12V_ENABLE, OUTPUT);
    pinMode(LC_PUMP_12V_ENABLE, OUTPUT);
    pinMode(TWELVEV_HSD1_ENABLE, OUTPUT);
    pinMode(FIVEV_ENABLE, OUTPUT);
    pinMode(KEEP_HIGH_PIN, OUTPUT);
    pinMode(VBAT_RAIL_ENABLE, OUTPUT);

    digitalWrite(AC_FAN_12V_ENABLE, HIGH);
    digitalWrite(TWELVEV_HSD1_ENABLE, HIGH);
    digitalWrite(FIVEV_ENABLE, HIGH);
    digitalWrite(KEEP_HIGH_PIN, HIGH);
    digitalWrite(VBAT_RAIL_ENABLE, HIGH);
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
