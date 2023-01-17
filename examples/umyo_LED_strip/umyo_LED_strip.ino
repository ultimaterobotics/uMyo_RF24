/*
Obtaining muscle data from uMyo via nRF24 module and showing 
activity level on a connected WS2812 LED strip
*/

#include <uMyo_RF24.h>
#include <FastLED.h>

#define NUM_LEDS 16
#define DATA_PIN 13

int rf_cen = 10; //nRF24 chip enable pin
int rf_cs = 9; //nRF24 CS pin

CRGB leds[NUM_LEDS];

void setup() {
  Serial.begin(115200);
  uMyo.begin(rf_cs, rf_cen);
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
  for(int l = 0; l < NUM_LEDS; l++)
    leds[l] = CRGB(0, 0, 0);
  FastLED.show();
}

void loop() 
{
  uMyo.run(); //need to call this really often, therefore
  //no long delays can be used in the code
  int dev_count = uMyo.getDeviceCount(); //if more than one device is present, show all of them
  float muscle_level = 0;
  if(dev_count > 0) muscle_level = uMyo.getAverageMuscleLevel(0); //take data from the 1st connected device if more than 1 is connected
  float max_level = 600; //defines sensitivity - the lower is maximum, the less effort is needed to fill all LEDs
  int active_leds = NUM_LEDS * muscle_level / max_level;
  int brightness = 150;
  for(int l = 0; l < NUM_LEDS; l++)
  {
    if(l < active_leds) leds[l] = CRGB(l * brightness / NUM_LEDS, 0, brightness/2); //blue -> purple red color scale
    else leds[l] = CRGB(0, 0, 0);
  }
  FastLED.show();
  delay(1);
}

