/*
    Author: Theodor Giles
    Created: 1/30/22
    Last Edited 5/4/22
    Description:
    arduino mega controller
*/
#include <Servo.h>
#include "JY901.h"
#include "ThrusterDriver.h"
#include "kArmDriver.h"
#include "HMC5883L.h"
// #include <Wire.h>


const byte RESET_PIN = 2;
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

Servo LBsig;
Servo LFsig;
Servo RBsig;
Servo RFsig;
Servo BLsig;
Servo BRsig;
Servo FLsig;
Servo FRsig;
  
int thrusterpower[8];
String JetsonCommand; // serial string

void setup() {
  // put your setup code here, to run once:
  
  // reset pin
  digitalWrite(RESET_PIN,HIGH);
  pinMode(RESET_PIN, OUTPUT);
  
  Serial.begin(9600);
  Serial.println("Configuring...");
  
  Serial1.begin(115200);

  //Serial1.onReceive(ParseCommands)

  LBsig.attach(LBpin);
  LFsig.attach(LFpin);
  RBsig.attach(RBpin);
  RFsig.attach(RFpin);
  BLsig.attach(BLpin);
  BRsig.attach(BRpin);
  FLsig.attach(FLpin);
  FRsig.attach(FRpin);

  //ventral
  LB_Thruster = ThrusterDriver(LBsig, "LB");
  LF_Thruster = ThrusterDriver(LFsig, "LF");
  RB_Thruster = ThrusterDriver(RBsig, "RB");
  RF_Thruster = ThrusterDriver(RFsig, "RF");

  //lateral
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
  delay(1000);
  Serial.println("Configured.");
  //print("Done setting up.");
}

void loop() {
    // put your main code here, to run repeatedly:
    //Serial1.println("Hello!!");
    if(JetsonCommand!=NULL){
      Serial.print("Command: ");
      Serial.println(JetsonCommand);
      commandRead();
    }
    Serial.println("In loop...");
    JetsonCommand = "";
    
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
void serialEvent1(){
//  Serial.println("Info received.");

  // moves serial value into a string
//  while (goodcomm){
//    char inChar = (char)Serial1.read();
//    if (inChar != NULL){
//      JetsonCommand += inChar;
//    }
//    else{
//      goodcomm = false;
//    }
//  }
  JetsonCommand = Serial1.readString();
//  break;
  return;
}

void commandRead(){
  String str_array[9]; // array of substrings 
  int n_str = 0; // number of subsrtings
  int startIndex=0;
  int endIndex=0;
  int cmdIndex = JetsonCommand.indexOf(',');
  String encodedCMD = JetsonCommand.substring(0,cmdIndex);
  String value;
  int j=0;
  int i=0;
  startIndex = cmdIndex;
  for (int i = 0; i < 10; i++)
  {
    endIndex = JetsonCommand.indexOf(',', endIndex+1);
    value = JetsonCommand.substring(startIndex+1,endIndex);
    if(value.compareTo(",")!=0){
      str_array[i] = value;
    }
    startIndex = endIndex;
  }
  switch (encodedCMD.charAt(0))
  {
    case 't': // thrusters
      // test second char in string 0
      switch (encodedCMD.charAt(1))
      {
        case 'a': // all
//          Serial1.flush();
//          Serial1.println("ATD");
          LB_Thruster.Drive(str_array[1].toInt());
          LF_Thruster.Drive(str_array[2].toInt());
          RB_Thruster.Drive(str_array[3].toInt());
          RF_Thruster.Drive(str_array[4].toInt());
          BL_Thruster.Drive(str_array[5].toInt());
          FL_Thruster.Drive(str_array[6].toInt());
          BR_Thruster.Drive(str_array[7].toInt());
          FR_Thruster.Drive(str_array[8].toInt());
//          Serial.println("Drove all thrusters.");
        case 'v': // ventral
          switch (encodedCMD.charAt(1)){
            
          }
          LB_Thruster.Drive(str_array[1].toInt());
          LF_Thruster.Drive(str_array[2].toInt());
          RB_Thruster.Drive(str_array[3].toInt());
          RF_Thruster.Drive(str_array[4].toInt());
        case 'l': // lateral
          BL_Thruster.Drive(str_array[1].toInt());
          FL_Thruster.Drive(str_array[2].toInt());
          BR_Thruster.Drive(str_array[3].toInt());
          FR_Thruster.Drive(str_array[4].toInt());
      }
//    case 'g': // gyros
//      switch (str_array[0][1])
//      {
//        case '1':
//          Serial.println("fortnite");
//      }
      // add move cases below when needed
  }
//  Serial1.flush();
//  Serial.println("Done reading command.");
  return;
//  break;
}

void beAuto(){
    LB_Thruster.Drive(10);
    LF_Thruster.Drive(10);
    RB_Thruster.Drive(10);
    RF_Thruster.Drive(10);
    BL_Thruster.Drive(15);
    BR_Thruster.Drive(15);
    FL_Thruster.Drive(15);
    FR_Thruster.Drive(15);
    delay(3000);
    LB_Thruster.Drive(10);
    LF_Thruster.Drive(10);
    RB_Thruster.Drive(10);
    RF_Thruster.Drive(10);
    BL_Thruster.Drive(-15);
    BR_Thruster.Drive(15);
    FL_Thruster.Drive(-15);
    FR_Thruster.Drive(15);
    delay(3000);
    LB_Thruster.Drive(10);
    LF_Thruster.Drive(10);
    RB_Thruster.Drive(10);
    RF_Thruster.Drive(10);
    BL_Thruster.Drive(15);
    BR_Thruster.Drive(-15);
    FL_Thruster.Drive(15);
    FR_Thruster.Drive(-15);
    delay(3000);
    LB_Thruster.Drive(-10);
    LF_Thruster.Drive(-10);
    RB_Thruster.Drive(-10);
    RF_Thruster.Drive(-10);
    BL_Thruster.Drive(15);
    BR_Thruster.Drive(15);
    FL_Thruster.Drive(15);
    FR_Thruster.Drive(15);
    delay(3000);
}
void Test(){
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
