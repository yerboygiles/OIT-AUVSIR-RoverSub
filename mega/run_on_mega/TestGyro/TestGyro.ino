#include "JY901_Serial.h"

<<<<<<< HEAD
#include <time.h>

float Xangle = 0;
float Yangle = 0;
float Zangle = 0;
=======
float Xangle_F = 0;
float Yangle_F = 0;
float Zangle_F = 0;
float Xangle_R = 0;
float Yangle_R = 0;
float Zangle_R = 0;
>>>>>>> bfa835fa0fdc5d6c11d6ec40191da9f577d5f85f
bool Cali = false;
int i = 0;

void setup()
{ 
  Serial.begin(9600);
  
  Serial.println("Testing...");
  
  Serial2.begin(115200);
  JY901_F.attach(Serial1);
<<<<<<< HEAD
  Serial3.begin(115200);
  JY901_R.attach(Serial1);
=======

  Serial2.begin(115200);
  JY901_R.attach(Serial2);
>>>>>>> bfa835fa0fdc5d6c11d6ec40191da9f577d5f85f
  // put your setup code here, to run once:
  
  delay(7000);

}

void loop()
{
  JY901_F.receiveSerialData();
<<<<<<< HEAD
  JY901_R.receiveSerialData();
  
  // print the gyro values
  String toprint = "Serial 2 Angles:";
=======
  // front gyro
  Xangle_F = JY901_F.getRoll();
  Yangle_F = JY901_F.getPitch();
  Zangle_F = JY901_F.getYaw();
  
  // rear gyro
  Xangle_R = JY901_R.getRoll();
  Yangle_R = JY901_R.getPitch();
  Zangle_R = JY901_R.getYaw();
  
  // print the gyro values
  String toprint = "Sensor 1:";
>>>>>>> bfa835fa0fdc5d6c11d6ec40191da9f577d5f85f
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
  String toprint2 = "Sensor 2:";
  toprint2 += JY901_R.getYaw();
  toprint2 += ",";
  toprint2 += JY901_R.getPitch();
  toprint2 += ":";
  toprint2 += JY901_R.getRoll();
  toprint2 += ",";
  toprint2 += "\n";
  Serial.print(toprint2);
  
  delay(2000);

//  if(i == 10)
//  {
//    if(Cali == false)
//    {
//      JY901_F.autoCaliGyro(1);
//      Cali = true;
//    }
//  }
<<<<<<< HEAD
  
  i++;
=======
//  
//  i++;
>>>>>>> bfa835fa0fdc5d6c11d6ec40191da9f577d5f85f
}
