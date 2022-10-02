/*
    Authors: Theodor Giles, Austin Morris
    Created: 10/29/21
    Last Edited 2/9/22
    Description:
    class for arm hardpoint kinematic control
*/

#ifndef KARMDRIVER_H
#define KARMDRIVER_H

#include "Servo.h"
#include "Arduino.h"

class kArmDriver{
    private:
        // Servo Angles
        float ServoSH_Angle;
        float ServoEL_Angle;
        float ServoGR_Angle;

        float a;      // Upper arm lenth (cm)
        float b;      // Forearm length (cm)

        // Correction factors to align servo values with their respective axis
        float SSHCorrectionFactor;     // Align arm "a" with the horizontal when at 0 degrees
        float ELCorrectionFactor;     // Align arm "b" with arm "a" when at 0 degrees

        // Correction factor to shift origin out to edge of the mount
        const float X_CorrectionFactor = 0;       // X direction correction factor (cm)
        const float Y_CorrectionFactor = 0;       // Y direction correction factor (cm)

        float A ;            //Angle oppposite side a (between b and c)
        float B ;            //Angle oppposite side b
        float C ;            //Angle oppposite side c
        float theta;        //Angle formed between line from origin to (x,y) and the horizontal
        float x;            // x position (cm)
        float y;            // y position (cm)
        float c;            // Hypotenuse legngth in cm
        const float pi;
        int GOpen;    // Servo angle for open gripper
        int GClose;    // Servo angle for closed gripper
    public:
        kArmDriver();
        kArmDriver(Servo sh, Servo el, Servo gr);
        void Arm();
        void PointTo(float x_input, float y_input);
        void SetArmConfig(float upper_arm, float lower_arm);
        void Reset();
        Servo ServoSH;      // Shoulder joint
        Servo ServoEL;      // Elbow joint
        Servo ServoGR;      // Gripper


};

#endif 
