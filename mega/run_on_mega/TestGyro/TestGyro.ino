#include "JY901_Serial.h"

#include <time.h>

float Xangle = 0;
float Yangle = 0;
float Zangle = 0;
bool Cali = false;
int i = 0;

void setup()
{ 
  Serial.begin(9600);
  
  Serial.println("Testing...");
  
  Serial2.begin(115200);
  JY901_F.attach(Serial1);
  Serial3.begin(115200);
  JY901_R.attach(Serial1);
  // put your setup code here, to run once:
  
  delay(7000);

}

void loop()
{
  JY901_F.receiveSerialData();
  JY901_R.receiveSerialData();
  
  // print the gyro values
  String toprint = "Serial 2 Angles:";
  toprint += JY901_F.getYaw();
  toprint += ",";
  toprint += JY901_F.getPitch();
  toprint += ",";
  toprint += JY901_F.getRoll();
  toprint += "\n";
  Serial.print(toprint);
  toprint = "Serial 3 Angles:";
  toprint += JY901_R.getYaw();
  toprint += ",";
  toprint += JY901_R.getPitch();
  toprint += ",";
  toprint += JY901_R.getRoll();
  toprint += "\n";
  Serial.print(toprint);
  delay(2000);

//  if(i == 10)
//  {
//    if(Cali == false)
//    {
//      JY901_F.autoCaliGyro(1);
//      Cali = true;
//    }
//  }
  
  i++;
}
