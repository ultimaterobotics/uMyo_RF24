/*
Obtaining muscle data from uMyo via nRF24 module
Usage: attach nRF24 module using any tutorial
 - open Serial Plotter (or Serial Monitor) at 115200 speed
 - turn on uMyo device
 - switch it into RF24 mode if necessary (to switch, 
press button once or twice, depending on current mode until you see 3 pink blinks)
 - you should see muscle activity chart on a plot (or as serial output)
*/

#include <uMyo_RF24.h>


void setup() {
  Serial.begin(115200);
  uMyo.begin();
}

uint32_t last_print_ms = 0;

void loop() 
{
  uMyo.run(); //need to call this really often, therefore
  //no delays can be used in the code
  int dev_count = uMyo.getDeviceCount(); //if more than one device is present, show all of them
  if(millis() - last_print_ms > 30)
  {
    for(int d = 0; d < dev_count; d++)
    {
      Serial.print(uMyo.getMuscleLevel(d));
      if(d < dev_count-1) Serial.print(' ');
      else Serial.println();
    }
    last_print_ms = millis();
  }
}
