/*
    Authors: Theodor Giles
    Created: 10/29/21
    Last Edited 2/9/22
    Description:
    class for thruster hardpoint control
*/
#ifndef THRUSTERDRIVER_h
#define THRUSTERDRIVER_h

#include "Servo.h"
#include "Arduino.h"

class ThrusterDriver
{
  private:
  Servo Motor;      // motor driver
  String Name;      // thruster ID
  int Offset;       // offset for finicky escs

  // Servo Angles

  public:
    float ThrusterSignal;
    
    ThrusterDriver();
    ThrusterDriver(Servo motor, String ID);
    ThrusterDriver(Servo motor, String ID, int offset);
    void Drive(int power);
    void Calibrate();
};

#endif 
