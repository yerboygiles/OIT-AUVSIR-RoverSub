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

void setup() {
  // put your setup code here, to run once:
  
  // reset pin
  digitalWrite(RESET_PIN,HIGH);
  pinMode(RESET_PIN, OUTPUT);
  
  Serial.begin(9600);
  Serial.println("Configuring...");
  
  Serial1.begin(9600);

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
  delay(7000);
  Serial.println("Configured.");
  //print("Done setting up.");
}

void loop() {
    // put your main code here, to run repeatedly:
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
    Serial.println("Info received.");
    // variable declarations
    String JetsonCommand; // serial string
    String str_array[9]; // array of substrings 
    int n_str = 0; // number of subsrtings
  
    // moves serial value into a string
    while (Serial1.available())
    {
      // JetsonCommand = Serial1.readString();
      char inChar = (char)Serial1.read();
      JetsonCommand += inChar;
    }
    Serial.print("Command: ");
    Serial.println(JetsonCommand);
    // creates c string from JetsonCommand
    char str[JetsonCommand.length() + 1];
    for (int i = 0; i < JetsonCommand.length(); ++i)
    {
      Serial.println("In for loop.");
      str[i] = JetsonCommand[i];
    }
    str[JetsonCommand.length()] = '\0';

    // sepaerates JetsonCommand into array of substrings
    String sub_str = strtok(str, ","); // create initial substring
    while (sub_str != NULL) // while not empty
    {
      Serial.println("In str!=NULL loop.");
      n_str++; // increments number of current substring
      str_array[n_str - 1] = sub_str; // sets value of string in array
      sub_str = strtok(str, ","); // get next substring
    }

    // test the first char in string 0
    Serial.print("Checking indexes for command parse.");
    switch (str_array[0][0])
    {
      case 't': // thrusters
        // test second char in string 0
        switch (str_array[0][1])
        {
          case 'a': // all
            LB_Thruster.Drive(str_array[1].toInt());
            LF_Thruster.Drive(str_array[2].toInt());
            RB_Thruster.Drive(str_array[3].toInt());
            RF_Thruster.Drive(str_array[4].toInt());
            BL_Thruster.Drive(str_array[5].toInt());
            FL_Thruster.Drive(str_array[6].toInt());
            BR_Thruster.Drive(str_array[7].toInt());
            FR_Thruster.Drive(str_array[8].toInt());
            Serial1.println("ATD");
            break;
          case 'v': // ventral
            switch (str_array[1][0]){
              
            }
            LB_Thruster.Drive(str_array[1].toInt());
            LF_Thruster.Drive(str_array[2].toInt());
            RB_Thruster.Drive(str_array[3].toInt());
            RF_Thruster.Drive(str_array[4].toInt());
            break;
          case 'l': // lateral
            BL_Thruster.Drive(str_array[1].toInt());
            FL_Thruster.Drive(str_array[2].toInt());
            BR_Thruster.Drive(str_array[3].toInt());
            FR_Thruster.Drive(str_array[4].toInt());
            break;
        }
        break;
        // add move cases below when needed
    }
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
