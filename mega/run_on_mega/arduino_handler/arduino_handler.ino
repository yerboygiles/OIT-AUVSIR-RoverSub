/*
    Author: Theodor Giles
    Created: 1/30/22
    Last Edited 2/9/22
    Description:
    arduino mega controller
*/
#include <Servo.h>
#include "JY901.h"
#include "ThrusterDriver.h"
#include "kArmDriver.h"
// #include <Wire.h>


byte LBpin = 2; //left back
byte LFpin = 3; //left front
byte RBpin = 4; //right back
byte RFpin = 5; //right front
byte BLpin = 6; //back left
byte BRpin = 7; //back right
byte FLpin = 8; //front left
byte FRpin = 9; //front right

ThrusterDriver LB_Thruster;
ThrusterDriver LF_Thruster;
ThrusterDriver RB_Thruster;
ThrusterDriver RF_Thruster;
ThrusterDriver BL_Thruster;
ThrusterDriver BR_Thruster;
ThrusterDriver FL_Thruster;
ThrusterDriver FR_Thruster;
int thrusterpower[8];

void setup() {
  Servo LBsig;
  Servo LFsig;
  Servo RBsig;
  Servo RFsig;
  Servo BLsig;
  Servo BRsig;
  Servo FLsig;
  Servo FRsig;
  // put your setup code here, to run once:
  Serial.begin(9600);

  LBsig.attach(LBpin);
  LFsig.attach(LFpin);
  RBsig.attach(RBpin);
  RFsig.attach(RFpin);
  BLsig.attach(BLpin);
  BRsig.attach(BRpin);
  FLsig.attach(FLpin);
  FRsig.attach(FRpin);
  
  LB_Thruster = ThrusterDriver(LBsig, "LB");
  LF_Thruster = ThrusterDriver(LFsig, "LF");
  RB_Thruster = ThrusterDriver(RBsig, "RB");
  RF_Thruster = ThrusterDriver(RFsig, "RF");
  BL_Thruster = ThrusterDriver(BLsig, "BL");
  BR_Thruster = ThrusterDriver(BRsig, "BR");
  FL_Thruster = ThrusterDriver(FLsig, "FL");
  FR_Thruster = ThrusterDriver(FRsig, "FR");
  
  LB_Thruster.Calibrate();
  LF_Thruster.Calibrate();
  RB_Thruster.Calibrate();
  RF_Thruster.Calibrate();
  BL_Thruster.Calibrate();
  BR_Thruster.Calibrate();
  FL_Thruster.Calibrate();
  FR_Thruster.Calibrate();
  delay(7000);
  //print("Done setting up.");

}

void loop() {
  // put your main code here, to run repeatedly:
  LB_Thruster.Drive(15);
  Serial.println("Thruster at port 2:");
  Serial.println(LB_Thruster.ThrusterSignal);
  delay(1000);
  LB_Thruster.Drive(0);
  
  LF_Thruster.Drive(15);
  Serial.println("Thruster at port 3.");
  Serial.println(LF_Thruster.ThrusterSignal);
  delay(1000);
  LF_Thruster.Drive(0);
  
  RB_Thruster.Drive(15);
  Serial.println("Thruster at port 4.");
  Serial.println(RB_Thruster.ThrusterSignal);
  delay(1000);
  RB_Thruster.Drive(0);
  
  RF_Thruster.Drive(15);
  Serial.println("Thruster at port 5.");
  Serial.println(RF_Thruster.ThrusterSignal);
  delay(1000);
  RF_Thruster.Drive(0);
  
  BL_Thruster.Drive(15);
  Serial.println("Thruster at port 6.");
  Serial.println(BL_Thruster.ThrusterSignal);
  delay(1000);
  BL_Thruster.Drive(0);
  
  BR_Thruster.Drive(15);
  Serial.println("Thruster at port 7.");
  Serial.println(BR_Thruster.ThrusterSignal);
  delay(1000);
  BR_Thruster.Drive(0);
  
  FL_Thruster.Drive(15);
  Serial.println("Thruster at port 8.");
  Serial.println(FL_Thruster.ThrusterSignal);
  delay(1000);
  FL_Thruster.Drive(0);
  
  FR_Thruster.Drive(15);
  Serial.println("Thruster at port 9.");
  Serial.println(FR_Thruster.ThrusterSignal);
  delay(1000);
  FR_Thruster.Drive(0);
}

String getSerialCommands(){
    if (Serial.available() > 0)
    {
        //String JetsonCommand = Serial.readStringUntil(TERMINATOR);
        
        
    }
}

String getValue(String data, char separator, int index){
    int found = 0;
    int strIndex[] = {0, -1};
    int maxIndex = data.length()-1;

    for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
    }
    return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}
