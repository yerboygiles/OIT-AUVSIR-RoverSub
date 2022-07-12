/*
    Author: Theodor Giles
    Created: 10/29/21
    Last Edited 1/30/22
    Description:
    thruster test code
*/
#include <Servo.h>
// #include <Wire.h>
// #include <JY901.h>


byte LBpin = 2; //left back

Servo LBthruster;





void setup() {
  // put your setup code here, to run once:
  LBthruster.attach(LBpin);

  
  delay(7000);
  LBthruster.writeMicroseconds(1500);
  delay(7000);

}

void loop() {
  // put your main code here, to run repeatedly:
  LBthruster.writeMicroseconds(1900);
}
