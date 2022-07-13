#include "JY901_Serial.h"

float Xangle = 0;
float Yangle = 0;
float Zangle = 0;
bool Cali = false;
int i = 0;

void setup()
{ 
  Serial.begin(9600);
  Serial.println("Testing...");
  
  Serial1.begin(115200);
  JY901_F.attach(Serial1);
  // put your setup code here, to run once:

}

void loop()
{
  JY901_F.receiveSerialData();
  // front gyro
  Xangle = JY901_F.getRoll();
  Yangle = JY901_F.getPitch();
  Zangle = JY901_F.getYaw();
  
  // print the gyro values
  String toprint = "A:";
  toprint += JY901_F.getYaw();
  toprint += ",";
  toprint += JY901_F.getPitch();
  toprint += ":";
  toprint += JY901_F.getRoll();
  toprint += ",";
  toprint += "\n";
  Serial.print(toprint);
  delay(2000);

  if(i == 10)
  {
    if(Cali == false)
    {
      JY901_F.autoCaliGyro(1);
      Cali = true;
    }
  }
  
  i++;
}
