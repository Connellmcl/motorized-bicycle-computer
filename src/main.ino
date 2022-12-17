
#include <Arduino.h>

#include "SPI.h"
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <TinyGPSPlus.h>

#include <Ucglib.h>
#include <TMP36.h>

#define TFT_DC 22
#define TFT_CS 13

#define RXD2 16
#define TXD2 17
#define RPM_PIN 4

unsigned long ticks = 0;
unsigned long rpm = 0;
unsigned long last_tick_check_threshold = 50;
unsigned long last = 0;
unsigned long last_tick_check = 0;
int screen_clear = 0;
int gps_fix = 0;

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

bool normal_mode = true;
int engine_off_red = 0;

static unsigned long last_interrupt_time = 0;

// Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, 23, 18, 5, 19);
// Ucglib_ILI9341_18x240x320_HWSPI ucg(/*cd=*/ 9, /*cs=*/ 10, /*reset=*/ 8);
Ucglib_ILI9341_18x240x320_HWSPI ucg(/*cd=*/22, /*cs=*/13, /*reset=*/5);

TMP36 Ambient(32, 3.3);
TMP36 CHT(33, 3.3);
TMP36 EGT(34, 3.3);

TinyGPSPlus gps;

hw_timer_t *timer = NULL;


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

void IRAM_ATTR modeSwitch()
// This is the interrupt subroutine that increments ticks counts for each HES response.
{

  normal_mode = !normal_mode;
}

void setup()
{
  timer = timerBegin(0, 80, true); 
  Serial.begin(9600);

  // start GPS
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  ucg.begin(UCG_FONT_MODE_SOLID);
  ucg.clearScreen();

  pinMode(RPM_PIN, INPUT_PULLDOWN);
  pinMode(mode_pin, INPUT_PULLDOWN);

  attachInterrupt(RPM_PIN, pickrpm, FALLING);
  attachInterrupt(mode_pin, modeSwitch, HIGH);
}

void loop(void)
{
  while (Serial2.available() > 0)
    gps.encode(Serial2.read());

  ucg.setRotate90();

  unsigned long start = millis();
  ucg_int_t y = 20;
  ucg_int_t h = 30;
  ucg_int_t second_row_y = 0;

  if (normal_mode)
  {

    gps_fix = 1;

    // Distance Travelled

    if (oldLat < 1)
    {
      oldLat = gps.location.lat();
      oldLng = gps.location.lng();
    }
    else if (gps.speed.mph() > 0.20)
    {
      distanceTravelled += gps.distanceBetween(oldLat, oldLng, gps.location.lat(),
                                               gps.location.lng());
      oldLat = gps.location.lat();
      oldLng = gps.location.lng();
    }

    if (screen_clear == 0)
    {
      ucg.clearScreen();
      screen_clear = 1;
    }

    if (last_tick_check < 10)
    {
      last_tick_check = micros();
      rpm = 1;
    }

    if ((millis() - last_tick_check) > last_tick_check_threshold)
    {
      rpm = (ticks * 60000) / ((millis() - last_tick_check));
      ticks = 0;
      last_tick_check = millis();
    }

    // Draw RPM Gauge

    ucg.setColor(0, 0, 0);
    if (rpm < 2)
    {
 
      ucg.setColor(255, 0, 0);
     ucg.drawBox((rpm / 25) + 1, y, screen_width - 3 - (rpm / 25), rpm_gauge_height); // red rpm blanking bar
    }
    else
    {
      ucg.drawBox((rpm / 25) + 1, y, screen_width - 3 - (rpm / 25), rpm_gauge_height); // black rpm blanking bar
    
      
      if (rpm < 2500)
      { 
        ucg.setColor(0,255,0);
      }
      if (rpm <4500 && rpm > 2500)
      {
        ucg.setColor(255,180,0);
      }
      if(rpm > 4000)
      {
        ucg.setColor(255,0,0);
      }
    

ucg.drawBox(0, y, (rpm / 25) + 1, rpm_gauge_height);
    }
     
    ucg.drawFrame(0, y - 2, screen_width - 1, rpm_gauge_height + 4); // rpm gauge box
    y += rpm_gauge_height * 2 + 15;

    // Draw RPM raw
    ucg.setColor(1, 0, 0, 0);
    ucg.setFont(ucg_font_inr30_mr);

    if (rpm < 2)
    {
      engine_was_off = 1;

      if (engine_was_on)
      {

        ucg.setColor(0, 0, 0);
        ucg.drawBox(0, 40, screen_width, 40);
        engine_was_on = 0;
      }

        ucg.setColor(255, 0, 0);
        engine_off_red++;
     
      ucg.setPrintPos(40, y);
      ucg.print("ENGINE OFF");
    }
    else
    {
      engine_was_on = 1;
      if (engine_was_off)
      {

         ucg.setColor(0, 0, 0);
        ucg.drawBox(0, 40, screen_width, 40); 
        engine_was_off = 0;
      }
      ucg.setPrintPos(90, y);

      ucg.setColor(255, 255, 255);
      ucg.print(rpm);
      ucg.setFont(ucg_font_inr16_mr);

      ucg.print(" rpm");
    }
    y += h + 10;

    // Draw Speed Gauge
    ucg.setFont(ucg_font_inr30_mr);
     ucg.setColor(1, 0, 0, 0);
    ucg.setColor(255, 255, 255);
    ucg.setPrintPos(60, y);
if(gps.satellites.value() > 4){
    ucg.println(gps.speed.mph());
    ucg.print("mph ");
}
else{
   ucg.setColor(255, 0, 0);
ucg.print("GPS ACQ ");

  

}
    ucg.setColor(255, 255, 255);
     ucg.setFont(ucg_font_profont12_mf);
     ucg.print("SIV:");
   ucg.print(gps.satellites.value());

 y += h + 10;

    // Draw Divider Lines

    ucg.drawFrame(0, 130, 160, 110);
    ucg.drawFrame(160, 130, 160, 110);
    // Draw Temperature Box
    ucg.setFont(ucg_font_profont17_mf);

    second_row_y = y;

    ucg.setPrintPos(4, y);

    ucg.print("OAT:");

    ucg.print(Ambient.getTempC());
    ucg.print("C "); // extra spaces

    y += h;
    ucg.setPrintPos(4, y);

    ucg.print("CHT:");

    ucg.print(CHT.getTempC());
    ucg.print("C"); // extra spaces
    y += h;
    ucg.setPrintPos(4, y);

    ucg.print("EGT:");

    ucg.print(EGT.getTempC());
    ucg.print("C"); // extra spaces

    ucg.setPrintPos(164, second_row_y);

    ucg.print("DT:");

    ucg.print(distanceTravelled);
    ucg.print("m  "); // extra spaces

    second_row_y += h;
    ucg.setPrintPos(164, second_row_y);
    ucg.print("COG:");

    ucg.print(gps.course.value());
    ucg.print(" "); // extra spaces

    second_row_y += h;
    ucg.setPrintPos(180, second_row_y);

    if (gps.time.hour() < 10)
      ucg.print(F("0"));
    ucg.print(gps.time.hour());
    ucg.print(F(":"));
    if (gps.time.minute() < 10)
      ucg.print(F("0"));
    ucg.print(gps.time.minute());
    ucg.print(F(":"));
    if (gps.time.second() < 10)
      ucg.print(F("0"));
    ucg.print(gps.time.second());
  }
  else
  {
    ucg.clearScreen();
  }

  second_row_y += h / 2;
  ucg.setPrintPos(164, second_row_y);
  ucg.setFont(ucg_font_profont12_mf);
  last = millis() - start;
  //ucg.print(last);
  ucg.print(timerReadSeconds(timer));
}