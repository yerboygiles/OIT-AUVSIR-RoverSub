/*
    Authors: Theodor Giles
    Created: 10/29/21
    Last Edited 10/29/21
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

  // Servo Angles
  float ThrusterSignal;

  public:
    ThrusterDriver();
    ThrusterDriver(Servo motor, String ID);
    void Drive(int power);
    void Calibrate();
};

#endif 
