/*
    Authors: Theodor Giles
    Created: 10/29/21
    Last Edited 10/29/21
    Description:
    class for thruster hardpoint control
*/
#include "ThrusterDriver.h"

ThrusterDriver::ThrusterDriver(Servo motor, String ID) {
  Motor = motor;
  Name = ID;
}
void ThrusterDriver::Drive(int power){ // -100 to 100
  Motor.writeMicroseconds(1500+(power*4));
}
void ThrusterDriver::Calibrate(){
  Motor.writeMicroseconds(1500);
  Serial.print("Yep. Calibrated.");
}
