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
byte LFpin = 3; //left front
byte RBpin = 4; //right back
byte RFpin = 5; //right front
byte BLpin = 6; //back left
byte BRpin = 7; //back right
byte FLpin = 8; //front left
byte FRpin = 9; //front right

Servo LBthruster;
Servo LFthruster;
Servo RBthruster;
Servo RFthruster;
Servo BLthruster;
Servo BRthruster;
Servo FLthruster;
Servo FRthruster;




void setup() {
  // put your setup code here, to run once:

  LBthruster.attach(LBpin);
  LFthruster.attach(LFpin);
  RBthruster.attach(RBpin);
  RFthruster.attach(RFpin);
  BLthruster.attach(BLpin);
  BRthruster.attach(BRpin);
  FLthruster.attach(FLpin);
  FRthruster.attach(FRpin);

  
  LBthruster.writeMicroseconds(1500);
  LFthruster.writeMicroseconds(1500);
  RBthruster.writeMicroseconds(1500);
  RFthruster.writeMicroseconds(1500);
  BLthruster.writeMicroseconds(1500);
  BRthruster.writeMicroseconds(1500);
  FLthruster.writeMicroseconds(1500);
  FRthruster.writeMicroseconds(1500);
  delay(7000);
  //print("Done setting up.");

}

void loop() {
  // put your main code here, to run repeatedly:
  int Signal = 1700;
  LBthruster.writeMicroseconds(Signal);
  LFthruster.writeMicroseconds(Signal);
  RBthruster.writeMicroseconds(Signal);
  RFthruster.writeMicroseconds(Signal);
  BLthruster.writeMicroseconds(Signal);
  BRthruster.writeMicroseconds(Signal);
  FLthruster.writeMicroseconds(Signal);
  FRthruster.writeMicroseconds(Signal);
  delay(1000);
  Signal = 1300;
  LBthruster.writeMicroseconds(Signal);
  LFthruster.writeMicroseconds(Signal);
  RBthruster.writeMicroseconds(Signal);
  RFthruster.writeMicroseconds(Signal);
  BLthruster.writeMicroseconds(Signal);
  BRthruster.writeMicroseconds(Signal);
  FLthruster.writeMicroseconds(Signal);
  FRthruster.writeMicroseconds(Signal);
  delay(1000);
}
