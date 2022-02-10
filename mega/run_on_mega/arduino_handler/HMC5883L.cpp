/*
    Authors: Theodor Giles
    Created: 10/29/21
    Last Edited 2/9/22
    Description:
    class for thruster hardpoint control
*/
#include "HMC5883L.h"
#include <Wire.h>
#define HMCaddr 0x1E

HMC5883L::HMC5883L() {
  compass.begin();
}

float HMC5883L::pollHeading(){
  sensors_event_t event;
  compass.getEvent(&event);

  headingRads = atan2(event.magnetic.y, event.magnetic.x);
  headingRads += declinationAngle;
  if(headingRads < 0) headingRads += 2*PI;
  if(headingRads > 2*PI) headingRads -= 2*PI;

  headingDegs = headingRads * 180/M_PI;
  return headingDegs;
}

float HMC5883L::getHeading(){
  return headingDegs;
}
void HMC5883L::Calibrate(){
  //serial.println(": Calibrated.");
}
