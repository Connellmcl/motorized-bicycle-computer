
#include <FS.h>
#include "SPIFFS.h"
#include <TFT_eSPI.h>
#include <Adafruit_NeoPixel.h>
#include "Wire.h"
#include "SD.h"
#include <Arduino.h>
#include "SPI.h"
#include <Adafruit_GFX.h>
#include <TinyGPSPlus.h>
#include <Adafruit_Sensor.h>
#include "MAX6675.h"

int CHTthermoCS = 13;
int EGTthermoCS = 12;
const int thermoDataPin = 26;
const int thermoClockPin = 27;

#define SEALEVELPRESSURE_HPA (1013.25)

// Display Definitions
//  This is the file name used to store the calibration data
//  The SPIFFS file name must start with "/".
#define CALIBRATION_FILE "/TouchCalData2"
#define REPEAT_CAL false

#define RXD2 16
#define TXD2 17
#define RPM_PIN 36

#define PIXEL_PIN 32

#define NUMPIXELS 8
#define BRIGHTNESS 50 // Set BRIGHTNESS to about 1/5 (max = 255)

unsigned long ticks = 0;
unsigned long rpm = 0;
unsigned long last_tick_check_threshold = 50;
unsigned long last = 0;
unsigned long last_tick_check = 0;
int screen_clear = 0;
int gps_fix = 0;
bool ledOn = false;

double oldLat = 0;
double oldLng = 0;
double newLat = 0;
double newLng = 0;
double distanceTravelled = 0;

int mode_pin = 12;
int screen_width = 320;
int screen_height = 240;
int rpm_gauge_height = 20;
int engine_was_off = 0;
int engine_was_on = 0;
int demo_rpm = 0;
int max_rpm = 6000;
bool normal_mode = true;
int engine_off_red = 0;

static unsigned long last_interrupt_time = 0;

TinyGPSPlus gps;

TFT_eSPI tft = TFT_eSPI(); // Invoke custom library
#define TFT_GREY 0x5AEB

hw_timer_t *timer = NULL;

Adafruit_NeoPixel strip(NUMPIXELS, PIXEL_PIN, NEO_GRB + NEO_KHZ800);
MAX6675 EGTThermoCouple;
MAX6675 CHTThermoCouple;

void touch_calibrate();
void LEDStrip();

void IRAM_ATTR pickrpm()
// This is the interrupt subroutine that increments ticks counts for each HES response.
{
  unsigned long interrupt_time = micros();
  // If interrupts come faster than 200ms, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time > 2000)
  {
    ticks++;
  }
  last_interrupt_time = interrupt_time;
}

void setup()
{
  timer = timerBegin(0, 80, true);
  Serial.begin(9600);

  // start GPS
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  EGTThermoCouple.begin(thermoClockPin, EGTthermoCS, thermoDataPin);
  CHTThermoCouple.begin(thermoClockPin, CHTthermoCS, thermoDataPin);
  // Initialise the TFT screen
  tft.init();

  // Set the rotation before we calibrate
  tft.setRotation(1);

  // Calibrate the touch screen and retrieve the scaling factors
  touch_calibrate();

  // Clear the screen
  tft.fillScreen(TFT_BLACK);

  pinMode(RPM_PIN, INPUT);
  pinMode(mode_pin, INPUT_PULLDOWN);

  attachInterrupt(RPM_PIN, pickrpm, RISING);



  strip.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();  // Turn OFF all pixels ASAP
  strip.setBrightness(BRIGHTNESS);
}

void loop(void)
{

  while (Serial2.available() > 0)
    gps.encode(Serial2.read());
  /*    for (int i = 0; i < NUMPIXELS; i++)
   { // For each pixel...

     // pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
     // Here we're using a moderately bright green color:
     strip.setPixelColor(i, strip.Color(255, 255, 255));

     strip.show(); // Send the updated pixel colors to the hardware.
   }
  */
  if ((millis() - last_tick_check) > last_tick_check_threshold)
  {
    rpm = (ticks * 60000) / ((millis() - last_tick_check));
    ticks = 0;
    last_tick_check = millis();
  }
  uint16_t x = tft.width() / 2;
  uint16_t y = 0;
  int rpm_bar_pixels = tft.width() / max_rpm;
  int w = tft.width();
  int rpm_bar_width = rpm * rpm_bar_width;
  tft.drawRect(0, y, w, 50, TFT_GREY);
  tft.fillRect(2, y, rpm_bar_width, 50 - 4, TFT_WHITE);
  tft.fillRect(rpm_bar_width, y, w - rpm_bar_width, 50 - 4, TFT_BLACK);

  tft.setTextDatum(TC_DATUM);

  tft.println();
  tft.setFreeFont(&FreeSansBold24pt7b); // Select the free font

  tft.setTextColor(TFT_WHITE, TFT_WHITE);
  y += 50;
  if (rpm < 10)
  {
    int padding = tft.textWidth("ENGINE OFF", 4); // get the width of the text in pixels
    tft.setTextPadding(padding);
    tft.setTextColor(TFT_RED, TFT_RED);
    tft.drawString("ENGINE OFF", x, y);
  }
  else
  {
    int padding = tft.textWidth("ENGINE OFF", 4); // get the width of the text in pixels
    tft.setTextPadding(padding);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawNumber(rpm, x, y); // Print the font name onto the TFT screen
  }
  y += 50;
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  int padding = tft.textWidth("99.9999", 4); // get the width of the text in pixels
  tft.setTextPadding(padding);
  if (gps.charsProcessed() < 10)
      { tft.setTextColor(TFT_RED, TFT_RED);

    tft.drawString("GPS: FAIL", x, y); // Print the font name onto the TFT screen}
      }
  else if (gps.satellites.value() < 4)
  {
    if (gps_fix == 1){
       tft.setTextColor(TFT_RED,TFT_RED);
       gps_fix = 0;
    }
    else if(gps_fix == 0){
    tft.setTextColor(TFT_YELLOW, TFT_YELLOW);
    gps_fix = 1;
    }
    tft.drawString("GPS ACQ. SIV:", x, y); // Print the font name onto the TFT screen
    tft.drawNumber(int(gps.satellites.value()), x +180,y);
  }
  else if (gps.speed.isUpdated())
  {    
      tft.drawFloat(float(gps.speed.mph()), 5, x, y); // Print the font name onto the TFT screen
  }
   tft.setTextColor(TFT_WHITE, TFT_WHITE);
  y += 50;
  EGTThermoCouple.read();
  CHTThermoCouple.read();

/* 
  if (CHTThermoCouple.getTemperature() < 0 ||  CHTThermoCouple.getTemperature() > 500 || CHTThermoCouple.getTemperature() == 0.000){
   tft.setTextColor(TFT_RED, TFT_RED);
   tft.drawString("CHT: FAIL", x, y);
}
else{ */
   tft.setTextColor(TFT_WHITE, TFT_WHITE);

  tft.drawFloat(CHTThermoCouple.getTemperature(), 1, x, y); // Print the font name onto the TFT screen
//}

  y += 50;
/* if (EGTThermoCouple.getTemperature() < 0 ||  EGTThermoCouple.getTemperature() > 500 || EGTThermoCouple.getTemperature() == 0.000){
   tft.setTextColor(TFT_RED, TFT_RED);
   tft.drawString("EGT: FAIL", x, y);
}
else{ */
   tft.setTextColor(TFT_WHITE, TFT_WHITE);

  tft.drawFloat(EGTThermoCouple.getTemperature(), 1, x, y); // Print the font name onto the TFT screen
//}

  y += 50;
  // tft.drawFloat(CHTThermoCouple.getTemperature(), 1, x, y); // Print the font name onto the TFT screen
if(!ledOn){tft.fillRoundRect(360, 240, 128, 78, 5, TFT_RED);}
if(ledOn) {tft.fillRoundRect(360, 240, 128, 78, 5, TFT_GREEN);}
   tft.setTextColor(TFT_WHITE);

    tft.drawString("LED", 420, 270, 4);

  Serial.print(gps.speed.mph());
  last = millis();
  Serial.println();
   uint16_t touch_x;
  uint16_t touch_y;
   tft.getTouch(&touch_x, &touch_y);
  if (touch_x > 400 && touch_y > 240){
      Serial.println("Touch!");
      ledOn = !ledOn;
  }

  LEDStrip();
  delay(100);
  Serial.println(analogRead(RPM_PIN));

  //}
}

void LEDStrip()
{
  if (ledOn){
  for (int i = 0; i < NUMPIXELS; i++)
  { // For each pixel...

    // pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
    // Here we're using a moderately bright green color:
    strip.setPixelColor(i, strip.Color(255, 255, 255));

    strip.show(); // Send the updated pixel colors to the hardware.
  }
  }
  else {
     for (int i = 0; i < NUMPIXELS; i++){
    // For each pixel...

    // pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
    // Here we're using a moderately bright green color:
    strip.setPixelColor(i, strip.Color(0, 0, 0));

    strip.show(); // Send the updated pixel colors to the hardware.
     }
  }
}

void touch_calibrate()
{
  uint16_t calData[5];
  uint8_t calDataOK = 0;

  // check file system exists
  if (!SPIFFS.begin())
  {
    Serial.println("Formating file system");
    SPIFFS.format();
    SPIFFS.begin();
  }

  // check if calibration file exists and size is correct
  if (SPIFFS.exists(CALIBRATION_FILE))
  {
    if (REPEAT_CAL)
    {
      // Delete if we want to re-calibrate
      SPIFFS.remove(CALIBRATION_FILE);
    }
    else
    {
      File f = SPIFFS.open(CALIBRATION_FILE, "r");
      if (f)
      {
        if (f.readBytes((char *)calData, 14) == 14)
          calDataOK = 1;
        f.close();
      }
    }
  }

  if (calDataOK && !REPEAT_CAL)
  {
    // calibration data valid
    tft.setTouch(calData);
  }
  else
  {
    // data not valid so recalibrate
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(20, 0);
    tft.setTextFont(2);
    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);

    tft.println("Touch corners as indicated");

    tft.setTextFont(1);
    tft.println();

    if (REPEAT_CAL)
    {
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.println("Set REPEAT_CAL to false to stop this running again!");
    }

    tft.calibrateTouch(calData, TFT_MAGENTA, TFT_BLACK, 15);

    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.println("Calibration complete!");

    // store data
    File f = SPIFFS.open(CALIBRATION_FILE, "w");
    if (f)
    {
      f.write((const unsigned char *)calData, 14);
      f.close();
    }
  }
}
