/*
    Authors: Theodor Giles
    Created: 10/29/21
    Last Edited 2/9/22
    Description:
    class for thruster hardpoint control
*/
#include "ThrusterDriver.h"

ThrusterDriver::ThrusterDriver() {
}
ThrusterDriver::ThrusterDriver(Servo motor, String ID) {
  Motor = motor;
  Name = ID;
  Offset = 0;
}
ThrusterDriver::ThrusterDriver(Servo motor, String ID, int offset) {
  Motor = motor;
  Name = ID;
  Offset = offset;
}
void ThrusterDriver::Drive(int power){ // -100 to 100
  ThrusterSignal = 1500+int(power*4);
  Motor.writeMicroseconds(ThrusterSignal);
}
void ThrusterDriver::Calibrate(){
  Motor.writeMicroseconds(1500);
  delay(7000);
  Serial.print(Name);
  Serial.println(": Calibrated.");
}
