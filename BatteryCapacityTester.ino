//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// ARDUINO BATTERY capacity TESTER
//Version-2.2
//by Karol Hirsz // The code is taken from Hesam Moshiri and deba168,INDIA ( https://www.pcbway.com/blog/technology/Battery_capacity_measurement_using_Arduino.html )
//Dated : 31/07/2022

#include <JC_Button.h>
#include <Adafruit_SSD1306.h>
#include "songs.h"

//#define DEBUG

#define SERIAL_SPEED 38400

#define RESISTANCE   1.00 // Value of power resistor in ohms, check with multimeter

// Calculate Vref scale constant (in mV); 1.1*1023*1000 = 1125300
// For better results calibrate it using equasion below
// VREF_SCALE_CONSTANT = (1.1 * Vcc1 (per voltmeter) / Vcc2 (per getVcc() function)) * 1023 * 1000
#define VREF_SCALE_CONSTANT 1125300

#define VBAT_THD_DEFUALT 2.80 // Default threshold for stopping test in Volts
#define VBAT_THD_STEP 0.1
#define VBAT_MAX_LEVEL 5.2

/// OLED definitions ///

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

/// PIN definitions ///

#define PWM_PIN 10
#define BUZZER_PIN 9
#define BAT_PIN A0

Button Up_Button(2, 25, false, true);
Button Down_Button(3, 25, false, true);

/// Misc definitions ///

#define HELLO_TIME 2000
#define MEAS_COUNT 25
#define LONG_PRESS_MS 1000
#define MAX_STR_VAL_LEN 10
#define DISPLAY_REFRESH_DELAY 300
#define MEAS_LOOP_MAIN_DELAY 1000
#define MEAS_LOOP_SHORT_DELAY 100
#define ADC_MAX_VALUE 1023

//Desired Current steps with a 3R load (R7)
#define PWM_ARRAY_SIZE 12
const int Current[PWM_ARRAY_SIZE] = {0, 50, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000};
const int PWM[PWM_ARRAY_SIZE] = {0, 2, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50};

float VBatThd = VBAT_THD_DEFUALT;
float Vcc = 5.00; // Voltage of Arduino 5V pin ( Measured during setup using internal reference )
float VBat = 0;
int PWMIndex = 0;       // Index into PWM array during test

void setup()
{
  Serial.begin(SERIAL_SPEED);

#ifdef DEBUG
  Serial.println("\nSetup!");
#endif

  pinMode(BAT_PIN, INPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  analogWrite(PWM_PIN, 0);
  Up_Button.begin();
  Down_Button.begin();

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextColor(WHITE);

  displayLogo();

  playSong1(BUZZER_PIN);

#ifdef DEBUG
  Serial.println("\nStart of calculations");
#endif

  //Read voltage of internal referance and use it to calculate actual Vcc
  Vcc = getVcc();

#ifdef DEBUG
  // Serial.println("Measured Vcc Voltage: " + String(Vcc) + " volts");
  Serial.print(F("Threshold: "));
  Serial.print(VBatThd, 3);
  Serial.println(F(" V"));
  Serial.print(F("Resistance: "));
  Serial.print(RESISTANCE, 3);
  Serial.println(F(" ohms"));
  Serial.print(F("Measured Vcc: "));
  Serial.print(Vcc, 4);
  Serial.println(F(" V"));

  Serial.println(F("Index Actual(mA) PWM"));
  int curr, pwm;
  for (int i = 0; i < PWM_ARRAY_SIZE; i++)
  {
    getCurrAndPWM(i, &curr, &pwm);

    // Serial.println( "["+String(i)+"]="+String(Current[i])+" mA PWM["+String(i)+"]="+String(PWM[i]) );
    Serial.print("[");
    Serial.print(i);
    Serial.print("]");
    Serial.print(" ");
    Serial.print(curr, DEC);
    Serial.print(" ");
    Serial.print(pwm, DEC);
    Serial.println(" ");
  }
#endif
} //Setup

//************************* End of Setup function *******************************

bool upDown = false;
unsigned long previousMillis = 0UL;

void loop()
{
  int curr, pwm;
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis > DISPLAY_REFRESH_DELAY)
  {
    Vcc = getVcc();
    VBat = getVBat();

    if (!upDown)
    {
      displayAdjCurr();
    }
    else
    {
      displayUpDown(curr, pwm);
    }

    previousMillis = currentMillis;
  }

  Up_Button.read();
  Down_Button.read();

  if (!upDown && Up_Button.pressedFor(LONG_PRESS_MS))
  {
#ifdef DEBUG
    Serial.println(F("Long UP action!"));
#endif
    VBatThd += VBAT_THD_STEP;
    if (VBatThd > VBAT_MAX_LEVEL)
    {
      VBatThd = VBAT_MAX_LEVEL;
    }
    shortBeep();
    consumeButtonPress(&Up_Button);
  }
  else if (Up_Button.wasReleased() && PWMIndex < (PWM_ARRAY_SIZE - 1))
  {
    upDown = true;
#ifdef DEBUG
    Serial.println(F("UP action!"));
#endif
    getCurrAndPWM(++PWMIndex, &curr, &pwm);
  }

  if (!upDown && Down_Button.pressedFor(LONG_PRESS_MS))
  {
#ifdef DEBUG
    Serial.println(F("Long Down action!"));
#endif
    VBatThd -= VBAT_THD_STEP;
    if (VBatThd < 0)
    {
      VBatThd = 0;
    }
    shortBeep();
    consumeButtonPress(&Down_Button);
  }
  else if (Down_Button.wasReleased() && PWMIndex > 0)
  {
    upDown = true;
#ifdef DEBUG
    Serial.println(F("DOWN action!"));
#endif
    getCurrAndPWM(--PWMIndex, &curr, &pwm);
  }

  if (upDown && (Up_Button.pressedFor(LONG_PRESS_MS) || Down_Button.pressedFor(LONG_PRESS_MS)))
  {
#ifdef DEBUG
    Serial.println(F("Start discharge!"));
#endif

    shortBeep();
    display.clearDisplay();
    measLoop();
  }
}

//************************* End of Loop function *******************************

void getCurrAndPWM(int idx, int *pCurrent, int *pPWM)
{
  int iTemp;

  if (pPWM == NULL || pCurrent == NULL)
  {
    return;
  }

  if (idx >= PWM_ARRAY_SIZE)
  {
    idx = 0;
  }

  // Convert desired current levels to PWM using actual Vcc
  // Convert PWM values back to actual current levels
  // While measuring actual current I discovered that the actual draw is
  // (PWM + 1)/256 rather than PWM/255 as indicated in Arduino documentation
  iTemp = int(RESISTANCE * Current[idx] * 256.0 / Vcc / 1000.0 - 1.0 + 0.5); // desired current to nearest PWM
  if (iTemp < 0)
  {
    iTemp = 0;
  }
  else if (iTemp > 255)
  {
    iTemp = 255;
  }

  *pCurrent = ((iTemp + 1) * Vcc / 256.0 / RESISTANCE * 1000.0); // actual current for PWM
  *pPWM = iTemp;
}

void measLoop()
{
  int hour = 0, minute = 0, second = 0;
  unsigned long capacity;
  float fCapacity;
  int curr, pwm;

  previousMillis = millis();

  while (true)
  {
    unsigned long currentMillis = millis();

    // Update time
    if (currentMillis - previousMillis < MEAS_LOOP_MAIN_DELAY)
    {
      delay(MEAS_LOOP_SHORT_DELAY);
      continue;
    }

    previousMillis += MEAS_LOOP_MAIN_DELAY;

    // Update time
    second++;

    if (second == 60)
    {
      second = 0;
      minute++;
      if (minute == 60)
      {
        minute = 0;
        hour++;
      }
    }    

    //************ Measuring Battery Voltage and Vcc ***********
    Vcc = getVcc();
    VBat = getVBat();

    //************ Update PWM values ***********
    getCurrAndPWM(PWMIndex, &curr, &pwm);
    analogWrite(PWM_PIN, pwm);

    //************ Calculate capacity ***********
    capacity = ((unsigned long)hour * 3600) + ((unsigned long)minute * 60) + (unsigned long)second;
    fCapacity = ((float)capacity * curr) / 3600.0;


    displayMeas(second, minute, hour, curr, fCapacity);

#ifdef DEBUG
    Serial.print(hour);
    Serial.print(":");
    Serial.print(minute);
    Serial.print(":");
    Serial.print(second);
    Serial.print(" ");
    Serial.print(VBat, 3);
    Serial.print("v ");
    Serial.print(fCapacity, 1);
    Serial.println("mAh");
#endif

    if (VBat < VBatThd)
    {
#ifdef DEBUG
      Serial.println(F("\nCurrent_Value PWM_Value"));
      Serial.print(curr);
      Serial.print(F(" "));
      Serial.println(pwm);
      Serial.println(F("\nHour minute second PWMIndex"));
      Serial.print(hour);
      Serial.print(F(" "));
      Serial.print(minute);
      Serial.print(F(" "));
      Serial.print(second);
      Serial.print(F(" "));
      Serial.println(PWMIndex);
      Serial.println(F("capacity HMS"));
      Serial.println(capacity);
      Serial.println(F("capacity HMS*PWM"));
      Serial.println(fCapacity, 1);
#endif

      analogWrite(PWM_PIN, 0);

      while (true)
      { 
        displayCapacity(fCapacity);
        playSong2(BUZZER_PIN);
        delay(DISPLAY_REFRESH_DELAY);
      }
    }
  }
} // measLoop

void shortBeep()
{
  tone(BUZZER_PIN, 1000);
  delay(100);
  noTone(BUZZER_PIN);
}

void consumeButtonPress(Button *button)
{
  for (int i = 0; i < 5; i++)
  {
    delay(50);
    button->read();
  }
}

void displayLogo()
{
  display.setCursor(10, 25);
  display.print(F("Bat Cap Tester V2.2"));
  display.display();
  delay(HELLO_TIME);
  display.clearDisplay();
}

void displayAdjCurr()
{
  char str[MAX_STR_VAL_LEN] = "";

  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print(F("Adj Curr"));
  display.setCursor(0, 18);
  display.print(F("UP/Down"));

  display.setTextSize(1);
  display.setCursor(0, 48);
  display.print(F("Long press to chg thd"));

  //Include Threshold and Vcc
  display.setCursor(0, 57);
  display.print("Thd=");
  dtostrf(VBatThd, 5, 3, str);
  display.print(str);
  display.print(", Vcc=");
  dtostrf(Vcc, 5, 3, str);
  display.print(str);

  display.display();
}

void displayUpDown(int curr, int pwm)
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("Thd=");
  display.print(VBatThd, 3);
  display.print(", VBat=");
  display.print(VBat, 3);

  display.setTextSize(2);
  display.setCursor(0, 18);
  display.print("Curr=");
  display.print(curr);
  display.print("mA");
  display.setCursor(0, 35);
  display.print("PWM=");
  display.print(pwm);

  display.setTextSize(1);
  display.setCursor(0, 56);
  display.print(F("Long press to start"));

  display.display();
}

void displayMeas(int second, int minute, int hour, int curr, float fCapacity)
{
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(32, 0);
    // display.print(String(hour) + ":" + String(minute) + ":" + String(second));
    display.print(hour);
    display.print(":");
    display.print(minute);
    display.print(":");
    display.print(second);

    display.setTextSize(1);
    display.setCursor(0, 25);
    display.print(F("I="));
    // display.print(String(Current[PWMIndex])+"mA");
    display.print(curr);
    display.print("mA");
    display.print(F(", Vcc="));
    display.print(Vcc, 3);

    display.setCursor(2, 40);
    // display.print("Bat Volt:" + String(VBat)+"V" );
    display.print(F("Vbat="));
    display.print(VBat, 3);
    display.print(F(", Thd="));
    display.print(VBatThd, 3);

    display.setCursor(2, 55);
    // display.print("capacity:" + String(capacity) + "mAh");
    display.print(F("CAPACITY="));
    display.print(fCapacity, 1);
    display.print("mAh");

    display.display();
}

void displayCapacity(float fCapacity)
{
  char str[MAX_STR_VAL_LEN] = "";

  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(2, 0);
  display.print(F("Capacity:"));
  display.setCursor(2, 23);
  // display.print(String(capacity) + "mAh");
  display.print(fCapacity, 1);
  display.print(F("mAh"));

  //Include Threshold and VBat
  display.setTextSize(1);
  display.setCursor(0, 57);
  display.print("VBat=");
  dtostrf(VBat, 5, 3, str);
  display.print(str);
  display.print(", Thd=");
  dtostrf(VBatThd, 5, 3, str);
  display.print(str);

  display.display();
}

float getVBat()
{
  long result = 0;

  analogReference(DEFAULT);
  for (int i = 0; i < MEAS_COUNT; i++)
  {
    delay(2);
    result += analogRead(BAT_PIN); //read the Battery voltage
  }
  result /= MEAS_COUNT;

  if (result >= ADC_MAX_VALUE)
  {
    // Overflow, return max VBat
    return VBAT_MAX_LEVEL;
  }

  return ((float)result) * Vcc / ((float)ADC_MAX_VALUE);
}

float getVcc()
{
// Read 1.1V reference against AVcc
// set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif

  long result = 0;
  for (int i = 0; i < MEAS_COUNT; i++)
  {
    delay(2);            // Wait for Vref to settle
    ADCSRA |= _BV(ADSC); // Start conversion
    while (bit_is_set(ADCSRA, ADSC)); // measuring

    uint8_t low = ADCL;  // must read ADCL first - it then locks ADCH
    uint8_t high = ADCH; // unlocks both

    result += ((high << 8) | low);
  }

  result /= MEAS_COUNT;
  result = VREF_SCALE_CONSTANT / result; // Calculate Vcc

  return ((float)result) / 1000.0; // Vcc in millivolts to float
}