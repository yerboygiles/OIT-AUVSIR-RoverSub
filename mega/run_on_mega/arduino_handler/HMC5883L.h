/*
    Authors: Theodor Giles
    Created: 2/10/22
    Last Edited 2/10/22
    Description:
    class for HMC compass data collection
*/
#ifndef HMC5883L_h
#define HMC5883L_h

#include "Arduino.h"
#include <Adafruit_HMC5883_U.h>

class HMC5883L
{
  private:
  const float declinationAngle = 0.2443461; // kfalls magnetic declination
  float headingRads;                        // compass zangle rad
  float headingDegs;                        // compass zangle deg
  String Name;                              // compass ID
  int Offset;                               // simple offset for finicky data
  Adafruit_HMC5883_Unified compass;         // compass object


  public:
    float ThrusterSignal;

    HMC5883L();
    float pollHeading();
    float getHead();  //i need top
    float getHeading();
    void Calibrate();
};

#endif
