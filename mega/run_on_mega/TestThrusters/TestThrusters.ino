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


byte pin2 = 2; //left back
byte pin3 = 3; //left back
byte pin4 = 4; //left back
byte pin5 = 5; //left back
byte pin6 = 6; //left back
byte pin7 = 7; //left back
byte pin8 = 8; //left back
byte pin9 = 9; //left back

Servo RFthruster;
Servo RBthruster;
Servo LFthruster;
Servo LBthruster;

Servo FRthruster;
Servo FLthruster;
Servo BLthruster;
Servo BRthruster;





void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  // ventral
  RFthruster.attach(pin2);
  RBthruster.attach(pin3);
  LFthruster.attach(pin4);
  LBthruster.attach(pin5);

  // lateral
  FRthruster.attach(pin6);
  FLthruster.attach(pin7);
  BLthruster.attach(pin8);
  BRthruster.attach(pin9);
  
  RFthruster.writeMicroseconds(1500);
  RBthruster.writeMicroseconds(1500);
  LFthruster.writeMicroseconds(1500);
  LBthruster.writeMicroseconds(1500);
  
  FRthruster.writeMicroseconds(1500);
  FLthruster.writeMicroseconds(1500);
  BLthruster.writeMicroseconds(1500);
  BRthruster.writeMicroseconds(1500);
  delay(7000);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(2000);
  Serial.println("RF");
  BRthruster.writeMicroseconds(1500);
  
  RFthruster.writeMicroseconds(1600);
  delay(2000);
  Serial.println("RB");
  RFthruster.writeMicroseconds(1500);
  
  RBthruster.writeMicroseconds(1600);
  delay(2000);
  Serial.println("LF");
  RBthruster.writeMicroseconds(1500);
  
  LFthruster.writeMicroseconds(1600);
  delay(2000);
  Serial.println("LB");
  LFthruster.writeMicroseconds(1500);
  
  LBthruster.writeMicroseconds(1600);
  delay(2000);
  Serial.println("FR");
  LBthruster.writeMicroseconds(1500);
  
  FRthruster.writeMicroseconds(1600);
  delay(2000);
  Serial.println("FL");
  FRthruster.writeMicroseconds(1500);
  
  FLthruster.writeMicroseconds(1600);
  delay(2000);
  Serial.println("BL");
  FLthruster.writeMicroseconds(1500);
  
  BLthruster.writeMicroseconds(1600);
  delay(2000);
  Serial.println("BR");
  BLthruster.writeMicroseconds(1500);
  
  BRthruster.writeMicroseconds(1600);
}
