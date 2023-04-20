#include <Arduino.h>
#include "ESP32_FastPWM.h"

// Sensor Pin Defines
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

// Enable Pin Defines
#define TWELVEV_HSD1_CSENSE_ENABLE 23
#define TWELVEV_HSD2_CSENSE_ENABLE 22
#define AC_FAN_12V_ENABLE 18
#define LC_FAN_12V_ENABLE 19
#define LC_PUMP_12V_ENABLE 21

// Store Currents
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

void setup()
{
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

float read_current(uint8_t pin, uint8_t sens = 0)
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
 * @brief Shutdown or start device when the current (Amps (A)) is above the specified limit.
 *
 * @return void
 */
void check_current(float current, float limit, u_int8_t pin, uint8_t switch_drive, uint8_t default_drive)
{
  if (current > limit)
  {
    // set the switch drive
  }
  else
  {
    // set the default drive
  }
}

void void loop()
{
  FIVEV_Current = read_current(FIVEV_CSENSE);
  TWELVEV_Current = read_current(TWELVEV_CSENSE);
  VBAT_RAIL_Current = read_current(VBAT_RAIL_CSENSE);
  check_current(VBAT_RAIL_Current, 5, VBAT_RAIL_CSENSE, DRIVE_LOW, DRIVE_HIGH);
  AC_FAN_12V_Current = read_current(AC_FAN_12V_CSENSE);
  LC_FAN_12V_Current = read_current(LC_FAN_12V_CSENSE);
  LC_PUMP_12V_Current = read_current(LC_PUMP_12V_CSENSE);
  TWELVEV_HSD1_Current = read_current(TWELVEV_HSD1_CSENSE);
  TWELVEV_HSD2_Current = read_current(TWELVEV_HSD2_CSENSE);
  VBAT_Current = read_current(VBAT_CSENSE);
  VBAT_Voltage = analogRead(VBAT_VSENSE) / 4096 * 3.3; // Give or take
}

/// PWM ///

// Max resolution is 20-bit
// Resolution 65536 (16-bit) for lower frequencies, OK @ 1K
// Resolution  4096 (12-bit) for lower frequencies, OK @ 10K
// Resolution  1024 (10-bit) for higher frequencies, OK @ 50K
// Resolution  256  ( 8-bit)for higher frequencies, OK @ 100K, 200K
// Resolution  128  ( 7-bit) for higher frequencies, OK @ 500K
int PWM_resolution = 12;

uint32_t PWM_Pins[] = {FIVEV_CSENSE, TWELVEV_CSENSE, VBAT_RAIL_CSENSE, AC_FAN_12V_CSENSE, LC_FAN_12V_CSENSE, LC_PUMP_12V_CSENSE, TWELVEV_HSD1_CSENSE, TWELVEV_HSD2_CSENSE};

// Must use different group-timer for different frequency
// e.g. chan 0 (Group0-T0) and chan8(Group1-T0) OK for dif. freq, but not chan0(Group0-T0) and chan1(Group0-T0)
// timer = ((chan / 2) % 4)
// group = (chan / 8)
// For ESP32
// Group 0: channel 0-1 => timer0, channel 2-3 => timer1, channel 4-5 => timer2, channel 6-7 => timer3
// Group 1: channel 8-9 => timer0, channel 10-11 => timer1, channel 12-13 => timer2, channel 14-15 => timer3
uint32_t PWM_chan[] = {0, 2, 4, 6};

// Still OK if different freq
// uint32_t PWM_chan[]   = {  0,  8,  4,  6 };
// Not OK if different freq
// uint32_t PWM_chan[]   = {  0,  1,  4,  6 };

#define NUM_OF_PINS (sizeof(PWM_Pins) / sizeof(uint32_t))

float dutyCycle[] = {10.0f, 30.0f, 50.0f, 90.0f};

float frequency[] = {2000.0f, 3000.0f, 4000.0f, 8000.0f};

ESP32_FAST_PWM *PWM_Instance[NUM_OF_PINS];

char dashLine[] = "=====================================================================================";

void printPWMInfo(ESP32_FAST_PWM *PWM_Instance)
{
  Serial.println(dashLine);
  Serial.print("Actual data: pin = ");
  Serial.print(PWM_Instance->getPin());
  Serial.print(", PWM DC = ");
  Serial.print(PWM_Instance->getActualDutyCycle());
  Serial.print(", PWMPeriod = ");
  Serial.print(PWM_Instance->getPWMPeriod());
  Serial.print(", PWM Freq (Hz) = ");
  Serial.println(PWM_Instance->getActualFreq(), 4);
  Serial.println(dashLine);
}

void setup()
{
  Serial.begin(115200);

  while (!Serial && millis() < 5000)
    ;

  delay(500);

  Serial.print(F("\nStarting PWM_Multi on "));
  Serial.println(ARDUINO_BOARD);
  Serial.println(ESP32_FAST_PWM_VERSION);

  for (uint8_t index = 0; index < NUM_OF_PINS; index++)
  {
    PWM_Instance[index] = new ESP32_FAST_PWM(PWM_Pins[index], frequency[index], dutyCycle[index], PWM_chan[index],
                                             PWM_resolution);

    if (PWM_Instance[index])
    {
      PWM_Instance[index]->setPWM();
    }
  }

  Serial.println(dashLine);
  Serial.println("Index\tPin\tPWM_freq\tDutyCycle\tActual Freq");
  Serial.println(dashLine);

  for (uint8_t index = 0; index < NUM_OF_PINS; index++)
  {
    if (PWM_Instance[index])
    {
      Serial.print(index);
      Serial.print("\t");
      Serial.print(PWM_Pins[index]);
      Serial.print("\t");
      Serial.print(frequency[index]);
      Serial.print("\t\t");
      Serial.print(dutyCycle[index]);
      Serial.print("\t\t");
      Serial.println(PWM_Instance[index]->getActualFreq(), 4);
    }
    else
    {
      Serial.println();
    }
  }

  for (uint8_t index = 0; index < NUM_OF_PINS; index++)
  {
    printPWMInfo(PWM_Instance[index]);
  }
}

void loop()
{
  // Long delay has no effect on the operation of hardware-based PWM channels
  delay(1000000);
}
