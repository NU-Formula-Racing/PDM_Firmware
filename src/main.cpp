#include <Arduino.h>
#include "ESP32_FastPWM.h"
#include "pdm.h"
#include "virtualTimer.h"

#define SERIAL_DEBUG

#ifdef ARDUINO_ARCH_ESP32
#include "esp_can.h"
// TX pin and RX pin as input.
ESPCAN can_bus{};
#endif

// Initialize board
PDM pdm_board;

// Structure for handling timers
VirtualTimerGroup read_timer;

/// CAN Signals ///
CANSignal<float, 0, 8, CANTemplateConvertFloat(0.02), CANTemplateConvertFloat(0), false> v5_rail_signal{};
CANSignal<float, 8, 8, CANTemplateConvertFloat(0.02), CANTemplateConvertFloat(0), false> v12_rail_signal{};
CANSignal<float, 16, 8, CANTemplateConvertFloat(0.02), CANTemplateConvertFloat(0), false> vbat_rail_signal{};
CANSignal<float, 24, 16, CANTemplateConvertFloat(0.002), CANTemplateConvertFloat(0), true> vbat_input_current_signal{};
CANSignal<float, 32, 8, CANTemplateConvertFloat(0.05), CANTemplateConvertFloat(10), false> vbat_input_voltage_signal{};
CANTXMessage<5> tx_message_1{can_bus, pdm_board.kCANId1, 6, 100, read_timer, v5_rail_signal, v12_rail_signal, vbat_rail_signal, vbat_input_current_signal, vbat_input_voltage_signal};

// /**
//  * @brief Shutdown or start device when the current (Amps (A)) is above the specified limit.
//  *
//  * @return void
//  */
// void check_current(float current, float limit, u_int8_t pin, uint8_t switch_drive, uint8_t default_drive)
// {
//   if (current > limit)
//   {
//     // set the switch drive
//   }
//   else
//   {
//     // set the default drive
//   }
// }

// void void loop()
// {
//   FIVEV_Current = read_current(FIVEV_CSENSE);
//   TWELVEV_Current = read_current(TWELVEV_CSENSE);
//   VBAT_RAIL_Current = read_current(VBAT_RAIL_CSENSE);
//   check_current(VBAT_RAIL_Current, 5, VBAT_RAIL_CSENSE, DRIVE_LOW, DRIVE_HIGH);
//   AC_FAN_12V_Current = read_current(AC_FAN_12V_CSENSE);
//   LC_FAN_12V_Current = read_current(LC_FAN_12V_CSENSE);
//   LC_PUMP_12V_Current = read_current(LC_PUMP_12V_CSENSE);
//   TWELVEV_HSD1_Current = read_current(TWELVEV_HSD1_CSENSE);
//   TWELVEV_HSD2_Current = read_current(TWELVEV_HSD2_CSENSE);
//   VBAT_Current = read_current(VBAT_CSENSE);
//   VBAT_Voltage = analogRead(VBAT_VSENSE) / 4096 * 3.3; // Give or take
// }