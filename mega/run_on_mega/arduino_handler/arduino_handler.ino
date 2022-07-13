/*
    Author: Theodor Giles
    Created: 1/30/22
    Last Edited 5/18/22
    Description:
    arduino mega controller
*/
#include <Servo.h>
#include "JY901_Serial.h"
#include "ThrusterDriver.h"
#include "kArmDriver.h"
#include "HMC5883L.h"
// #include <Wire.h>

//const byte RESET_PIN = 2;
byte LBpin = 5; //left back
byte LFpin = 4; //left front
byte RBpin = 3; //right back
byte RFpin = 2; //right front

byte BLpin = 8; //back left
byte FLpin = 7; //front left
byte BRpin = 9; //back right
byte FRpin = 6; //front right

byte ledPin = 13; //pin for SOS Leak Board
byte leakPin = 10; //Leak Signal Pin
byte OffPin = 11; //pin to turn off board
int leak = 0; // 0 = dry, 1 = leak

// old, ventral and lateral flipped?.. too tired to swap wires
//byte LBpin = 8; //left back
//byte LFpin = 7; //left front
//byte RBpin = 9; //right back
//byte RFpin = 6; //right front
//
//byte BLpin = 5; //back left
//byte FLpin = 4; //front left
//byte BRpin = 3; //back right
//byte FRpin = 2; //front right

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
String JetsonCommand = ""; // serial string
bool stringComplete = false;  // whether the string is complete

int commandComplete = 0;
String str_array[10]; // array of substrings
int startIndex = 0;
int endIndex = 0;
int cmdIndex;
String encodedCMD;
String value;

//gyro data
//roll
float Xangle_R;
//pitch
float Yangle_R;
//yaw
float Zangle_R;

float Xangle_F;
float Yangle_F;
float Zangle_F;

//gyro data offsets for calib/heading reset
float Xangle_offset_R=0;
float Yangle_offset_R=0;
float Zangle_offset_R=0;

float Xangle_offset_F=0;
float Yangle_offset_F=0;
float Zangle_offset_F=0;

//accel data
float Xaccel;
float Yaccel;
float Zaccel;

//velocity integration
float Xveloc=0;
float Xposit=0;

float Yveloc=0;
float Yposit=0;

float Zveloc=0;
float Zposit=0;

long newTime;
long lastTime=0;

void setup() {
  // put your setup code here, to run once:

  // reset pin
//  digitalWrite(RESET_PIN, HIGH);
//  pinMode(RESET_PIN, OUTPUT);

  pinMode(ledPin, OUTPUT);
  pinMode(leakPin, INPUT);
  pinMode(OffPin, OUTPUT);

  Serial.begin(9600);
  Serial.println("Configuring...");

  Serial1.begin(115200);
  JetsonCommand.reserve(200);
  value.reserve(5);
  
  Serial2.begin(115200);
  JY901_F.attach(Serial2);

  Serial3.begin(115200);
  JY901_R.attach(Serial3);

  JY901_F.receiveSerialData();
  JY901_R.receiveSerialData();

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
  delay(5000);
  return;
}


void loop() {
//  while (1){
//    Test();
//  }

  // put your main code here, to run repeatedly:
  //Serial1.println("Hello!!");
  if (stringComplete) {
    Serial1.flush();
//    Serial.print("Command: ");
//    Serial.println(JetsonCommand);
    
//    Serial.println("Done with readCommand().");
//    Serial.println(commandComplete);
    switch (readCommand()) {
      case 0:
        Serial.println("No command run.");
        break;
      case 1:
        Serial.println("Thrusters driven.");
        break;
      case 2:
        Serial.println("Thrusters calibrated.");
        break;
      default:
        Serial.println("Default...");
        break;
    }
    JetsonCommand = "";
    stringComplete = false;
  }
//    Serial.println("In loop...");
  if (Serial1.available()) {
    // get the new byte:
    char inChar = (char)Serial1.read();
    // add it to the inputString:
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
    if (stringComplete == false) {
      JetsonCommand += inChar;
    }
  }

  leak = digitalRead(leakPin); // Read Leak Sensor Pin
  digitalWrite(ledPin, leak); // Set the LED to the sensor's value

  if(leak == 1)
  {
    digitalWrite(OffPin, HIGH); //Turn off board
  }
  return;
}

int readCommand() {
  commandComplete = 0;
  endIndex = 0;
  cmdIndex = JetsonCommand.indexOf(',');
  encodedCMD = JetsonCommand.substring(0, cmdIndex);
//  int i = 0;
  startIndex = cmdIndex;
  for (int i = 0; i <= 10; i++)
  {
    endIndex = JetsonCommand.indexOf(',', endIndex + 1);
    value = JetsonCommand.substring(startIndex + 1, endIndex);
    if (value.compareTo(",") != 0) {
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
//          Serial.print("BL:");
//          Serial.print(str_array[1]);
//          Serial.print(",FL:");
//          Serial.print(str_array[2]);
//          Serial.print(",BR:");
//          Serial.print(str_array[3]);
//          Serial.print(",FR:");
//          Serial.print(str_array[4]);
//          Serial.print(",LB:");
//          Serial.print(str_array[5]);
//          Serial.print(",LF:");
//          Serial.print(str_array[6]);
//          Serial.print(",RB:");
//          Serial.print(str_array[7]);
//          Serial.print(",RF:");
//          Serial.println(str_array[8]);
          BL_Thruster.Drive(str_array[1].toInt());
          FL_Thruster.Drive(str_array[2].toInt());
          BR_Thruster.Drive(str_array[3].toInt());
          FR_Thruster.Drive(str_array[4].toInt());
          
          LB_Thruster.Drive(str_array[5].toInt());
          LF_Thruster.Drive(str_array[6].toInt());
          RB_Thruster.Drive(str_array[7].toInt());
          RF_Thruster.Drive(str_array[8].toInt());
//          Serial.println("Drove all thrusters.");
          commandComplete = 1;
          break;
        case 'v': // ventral
          LB_Thruster.Drive(str_array[3].toInt());
          LF_Thruster.Drive(str_array[4].toInt());
          RB_Thruster.Drive(str_array[5].toInt());
          RF_Thruster.Drive(str_array[6].toInt());
          commandComplete = 1;
          break;
        case 'l': // lateral
          BL_Thruster.Drive(str_array[3].toInt());
          FL_Thruster.Drive(str_array[4].toInt());
          BR_Thruster.Drive(str_array[5].toInt());
          FR_Thruster.Drive(str_array[6].toInt());
          commandComplete = 1;
          break;
        case 'c': // calibrate
          LB_Thruster.Calibrate();
          LF_Thruster.Calibrate();
          RB_Thruster.Calibrate();
          RF_Thruster.Calibrate();
          BL_Thruster.Calibrate();
          BR_Thruster.Calibrate();
          FL_Thruster.Calibrate();
          FR_Thruster.Calibrate();
//          delay(7000);
          commandComplete = 2;
          break;
        default: // default
          break;
      }
      break;
    default:
      break;
    case 'g': // gyros
      switch (str_array[0][1])
      {
        case 'c':
          switch(str_array[0][2]){
            case 'a':
            JY901_F.receiveSerialData();
            JY901_R.receiveSerialData();
            String toprint = "C:";
            toprint += JY901_F.getAccX();
            toprint += ",";
            toprint += JY901_R.getAccX();
            toprint += ":";
            toprint += JY901_F.getAccY();
            toprint += ",";
            toprint += JY901_R.getAccY();
            toprint += ":";
            toprint += JY901_F.getAccZ();
            toprint += ",";
            toprint += JY901_R.getAccZ();
            toprint += "\n";
            Serial1.print(toprint);
          }
        case 'f':
          JY901_F.receiveSerialData();
          switch (str_array[0][2]){
            case 'h':
              Zangle_offset_F = JY901_F.getYaw();
              Serial1.print("fh\n");
              break;
            case 'c':
              Xangle_offset_F = JY901_F.getRoll();
              Yangle_offset_F = JY901_F.getPitch();
              Zangle_offset_F = JY901_F.getYaw();
              JY901_F.autoCaliGyro(1);
              Serial1.print("fc\n");
              break;
            case 'a':
              Serial1.print("A:");
              Serial1.print(JY901_F.getYaw());
              Serial1.print(":");
              Serial1.print(JY901_F.getPitch());
              Serial1.print(":");
              Serial1.print(JY901_F.getRoll());
              Serial1.print("\n");
              break;
          }
          break;
        case 'r':
          JY901_R.receiveSerialData();
          switch (str_array[0][2]){
            case 'h':
              Zangle_offset_R = JY901_R.getYaw();
              Serial1.print("rh\n");
              break;
            case 'c':
              Xangle_offset_R = JY901_R.getRoll();
              Yangle_offset_R = JY901_R.getPitch();
              Zangle_offset_R = JY901_R.getYaw();
              JY901_R.autoCaliGyro(1);
              Serial1.print("rc\n");
              break;
            case 'a':
              Serial1.print("A:");
              Serial1.print(JY901_R.getYaw());
              Serial1.print(":");
              Serial1.print(JY901_R.getPitch());
              Serial1.print(":");
              Serial1.print(JY901_R.getRoll());
              Serial1.print("\n");
              break;
          }
          break;
        // all gyros
        case 'a':
          JY901_F.receiveSerialData();
          JY901_R.receiveSerialData();
          switch (str_array[0][2]){
            case 'h':
              Zangle_offset_R = JY901_R.getYaw();
              Serial1.print("rh\n");
              break;
            case 'c':
              Xangle_offset_R = JY901_R.getRoll();
              Yangle_offset_R = JY901_R.getPitch();
              Zangle_offset_R = JY901_R.getYaw();
              Xangle_offset_F = JY901_F.getRoll();
              Yangle_offset_F = JY901_F.getPitch();
              Zangle_offset_F = JY901_F.getYaw();
              JY901_R.autoCaliGyro(1);
              JY901_F.autoCaliGyro(1);
              Serial1.print("rc\n");
              break;
            case 'a':
              String toprint = "A:";
              toprint += JY901_F.getYaw();
              toprint += ",";
              toprint += JY901_R.getYaw();
              toprint += ":";
              toprint += JY901_F.getPitch();
              toprint += ",";
              toprint += JY901_R.getPitch();
              toprint += ":";
              toprint += JY901_F.getRoll();
              toprint += ",";
              toprint += JY901_R.getRoll();
              toprint += "\n";
              Serial1.print(toprint);
              break;
          }
          break;
      }
      // add move cases below when needed
  }
  //  Serial1.flush();
//  Serial.println("Done reading command.");
  return commandComplete;
  //  break;
}

void updateAccel(){
  Xaccel = (JY901_R.getAccX() + JY901_F.getAccX())/2;
  Yaccel = (JY901_R.getAccY() + JY901_F.getAccY())/2;
  Zaccel = (JY901_R.getAccZ() + JY901_F.getAccZ())/2;
}
float integrateXaccel(){
  newTime = millis();
  
  Xveloc = Xveloc + Xaccel * (newTime - lastTime)/1000;
  
  Xposit = Xposit + Xveloc * (newTime - lastTime)/1000;
  
  lastTime=newTime;
}
float integrateYaccel(){
  newTime = millis();
  
  Yveloc = Yveloc + Yaccel * (newTime - lastTime)/1000;
  
  Yposit = Yposit + Yveloc * (newTime - lastTime)/1000;
  
  lastTime=newTime;
}
float integrateZaccel(){
  newTime = millis();
  
  Zveloc = Zveloc + Zaccel * (newTime - lastTime)/1000;
  
  Zposit = Zposit + Zveloc * (newTime - lastTime)/1000;
  
  lastTime=newTime;
}

void beAuto() {
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
void Test() {
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


//void serialEvent2(){
//  while (Serial1.available()) {
//    // get the new byte:
//    char inChar = (char)Serial1.read();
//    // add it to the inputString:
//    if(stringComplete == false){
//      JetsonCommand += inChar;
//    }
//    // if the incoming character is a newline, set a flag so the main loop can
//    // do something about it:
//    if (inChar == '}') {
//      stringComplete = true;
////      parsingCommand = true;
//      break;
//    }
//  }

//  JetsonCommand = Serial1.readString();
//  break;
//  return;
//}
