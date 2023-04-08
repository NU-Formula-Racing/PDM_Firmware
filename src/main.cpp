#include <Arduino.h>

//Sensor Pin Defines
#define FIVEV_CSENSE 36
#define TWELVEV_CSENSE 39
#define VBAT_RAIL_CSENSE 34
#define AC_FAN_12V_CSENSE 35
#define LC_FAN_12V_CSENSE 32
#define LC_PUMP_12V_CSENSE 33
#define TWELVEV_HSD1_CSENSE 25
#define TWELVEV_HSD2_CSENSE 26
#define VBAT_CSENSE 27
#define VBAT_VSENSE 14

//Enable Pin Defines
#define TWELVEV_HSD1_CSENSE_ENABLE 23
#define TWELVEV_HSD2_CSENSE_ENABLE 22
#define AC_FAN_12V_ENABLE 18
#define LC_FAN_12V_ENABLE 19
#define LC_PUMP_12V_ENABLE 21

float FIVEV_Current;
float TWELVEV_Current;
float VBAT_RAIL_Current;
float AC_FAN_12V_Current;
float LC_FAN_12V_Current;
float LC_PUMP_12V_Current;
float TWELVEV_HSD1_Current;
float TWELVEV_HSD2_Current;
float VBAT_Current;
float VBAT_Voltage;

void setup() {
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
  pinMode(TWELVEV_HSD1_CSENSE_ENABLE, OUTPUT);
  pinMode(TWELVEV_HSD2_CSENSE_ENABLE, OUTPUT);
}

float read_current(uint8_t pin, uint8_t sens = 0) {
  float ret;
  if (sens = 0) {
    float ret = (0.0008 * analogRead(pin) + 0.1372)/0.2;
  } else if (sens = 1) {
    float ret = (0.0008 * analogRead(pin) - 0.41128)/0.05;
  }
  return ret;
}

void loop() {
  FIVEV_Current = read_current(FIVEV_CSENSE);
  TWELVEV_Current = read_current(TWELVEV_CSENSE);
  VBAT_RAIL_Current = read_current(VBAT_RAIL_CSENSE);
  AC_FAN_12V_Current = read_current(AC_FAN_12V_CSENSE);
  LC_FAN_12V_Current = read_current(LC_FAN_12V_CSENSE);
  LC_PUMP_12V_Current = read_current(LC_PUMP_12V_CSENSE);
  TWELVEV_HSD1_Current = read_current(TWELVEV_HSD1_CSENSE);
  TWELVEV_HSD2_Current = read_current(TWELVEV_HSD2_CSENSE);
  VBAT_Current = read_current(VBAT_CSENSE);
  VBAT_Voltage = analogRead(VBAT_VSENSE) / 4096 * 3.3; //Give or take
}
