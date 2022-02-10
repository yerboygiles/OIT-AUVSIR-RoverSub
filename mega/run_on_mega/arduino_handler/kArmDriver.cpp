/*
    Authors: Theodor Giles, Austin Morris
    Created: 10/29/21
    Last Edited 2/9/22
    Description:
    class for arm hardpoint kinematic control
*/
#include "Arduino.h"
#include "kArmDriver.h"

kArmDriver::kArmDriver (Servo sh, Servo el, Servo gr)
{
  ServoSH = sh;      // Shoulder joint
  ServoEL = el;      // Elbow joint
  ServoGR = gr;      // Gripper
}
void kArmDriver::SetArmConfig(float upper_arm, float lower_arm){
        a = upper_arm;
        b = lower_arm;
    }
void kArmDriver::PointTo(float x_input, float y_input){
    x = x_input + X_CorrectionFactor;
    y = y_input + Y_CorrectionFactor;

    c = sqrt( sq(x) + sq(y) );                                            // pythagorean theorem
    B = (acos( (sq(b) - sq(a) - sq(c))/(-2*a*c) )) * (180/pi);            // Law of cosines: Angle opposite upper arm section
    C = (acos( (sq(c) - sq(b) - sq(a))/(-2*a*b) )) * (180/pi);            // Law of cosines: Angle opposite hypotenuse
    theta = (asin( y / c )) * (180/pi);                                   // Solve for theta to correct for lower joint's impact on upper joint's angle
    ServoSH_Angle = B + theta + SSHCorrectionFactor;                    // Find necessary angle. Add Correction
    ServoEL_Angle = C + ELCorrectionFactor;                            // Find neceesary angle. Add Correction

    ServoSH.write(ServoSH_Angle);
    ServoEL.write(ServoEL_Angle);
}
void kArmDriver::Reset()
{
    x = 5;   //not sure what to have as the default x/y
    y = -7;

    c = sqrt( sq(x) + sq(y) );                                            // pythagorean theorem
    B = (acos( (sq(b) - sq(a) - sq(c))/(-2*a*c) )) * (180/pi);            // Law of cosines: Angle opposite upper arm section
    C = (acos( (sq(c) - sq(b) - sq(a))/(-2*a*b) )) * (180/pi);            // Law of cosines: Angle opposite hypotenuse
    theta = (asin( y / c )) * (180/pi);                                   // Solve for theta to correct for lower joint's impact on upper joint's angle
    ServoSH_Angle = B + theta + SSHCorrectionFactor;                    // Find necessary angle. Add Correction
    ServoEL_Angle = C + ELCorrectionFactor;                            // Find neceesary angle. Add Correction

    ServoSH.write(ServoSH_Angle);
    ServoEL.write(ServoEL_Angle);
}
//class kArmDriver(){
//  Servo ServoSH;      // Shoulder joint
//  Servo ServoEL;      // Elbow joint
//  Servo ServoGR;      // Gripper
//  
//  // Servo Angles
//  float ServoSH_Angle = 90;
//  float ServoEL_Angle = 90;
//  float ServoGR_Angle = 90;
//  
//  float a = 18.2;      // Upper arm lenth (cm)
//  float b = 22.8;      // Forearm length (cm)
//  
//  // Correction factors to align servo values with their respective axis
//  const float SSHCorrectionFactor = 0;     // Align arm "a" with the horizontal when at 0 degrees
//  const float ELCorrectionFactor = 0;     // Align arm "b" with arm "a" when at 0 degrees
//  
//  // Correction factor to shift origin out to edge of the mount
//  const float X_CorrectionFactor  = 0;       // X direction correction factor (cm)
//  const float Y_CorrectionFactor  = 0;       // Y direction correction factor (cm)
//  
//  float A ;            //Angle opposite side a (between b and c)
//  float B ;            //Angle opposite side b
//  float C ;            //Angle opposite side c
//  float theta;        //Angle formed between line from origin to (x,y) and the horizontal
//  float x;            // x position (cm)
//  float y;            // y position (cm)
//  float c;            // Hypotenuse length in cm
//  const float pi = M_PI;
//  int GOpen = 100;    // Servo angle for open gripper
//  int GClose = 10;    // Servo angle for closed gripper
//  public:
//    kArmDriver (Servo sh, Servo el, Servo gr){
//      ServoSH = sh;
//      ServoEL = el;
//      ServoGR = gr;
//    }
//    void SetArmConfig(float upper_arm, float lower_arm){
//        a = upper_arm
//    }
//    void PointTo(float x_input, float y_input){
//        x = x_input + X_CorrectionFactor;
//        y = y_input + Y_CorrectionFactor;
//    
//        c = sqrt( sq(x) + sq(y) );                                            // pythagorean theorem
//        B = (acos( (sq(b) - sq(a) - sq(c))/(-2*a*c) )) * (180/pi);            // Law of cosines: Angle opposite upper arm section
//        C = (acos( (sq(c) - sq(b) - sq(a))/(-2*a*b) )) * (180/pi);            // Law of cosines: Angle opposite hypotenuse
//        theta = (asin( y / c )) * (180/pi);                                   // Solve for theta to correct for lower joint's impact on upper joint's angle
//        ServoSH_Angle = B + theta + SSHCorrectionFactor;                    // Find necessary angle. Add Correction
//        ServoEL_Angle = C + ELCorrectionFactor;                            // Find neceesary angle. Add Correction
//    
//        ServoSH.write(ServoSH_Angle);
//        ServoEL.write(ServoEL_Angle);
//    }
//    void Reset()
//        x = 5;   //not sure what to have as the default x/y
//        y = -7;
//
//        c = sqrt( sq(x) + sq(y) );                                            // pythagorean theorem
//        B = (acos( (sq(b) - sq(a) - sq(c))/(-2*a*c) )) * (180/pi);            // Law of cosines: Angle opposite upper arm section
//        C = (acos( (sq(c) - sq(b) - sq(a))/(-2*a*b) )) * (180/pi);            // Law of cosines: Angle opposite hypotenuse
//        theta = (asin( y / c )) * (180/pi);                                   // Solve for theta to correct for lower joint's impact on upper joint's angle
//        ServoSH_Angle = B + theta + SSHCorrectionFactor;                    // Find necessary angle. Add Correction
//        ServoEL_Angle = C + ELCorrectionFactor;                            // Find neceesary angle. Add Correction
//
//        ServoSH.write(ServoSH_Angle);
//        ServoEL.write(ServoEL_Angle);
//
//}

#define arren( x )  ( sizeof( x ) / sizeof( *x ) )
