/*
    Authors: Theodor Giles, Austin Morris
    Created: 10/29/21
    Last Edited 10/29/21
    Description:
    class and controller of arm hardpoint kinematics
*/
class arm(){
  Servo ServoSH;      // Shoulder joint
  Servo ServoEL;      // Elbow joint
  Servo ServoGR;      // Gripper
  
  // Servo Angles
  float ServoSH_Angle = 90;
  float ServoEL_Angle = 90;
  float ServoGR_Angle = 90;
  
  const float a = 18.2;      // Upper arm lenth (cm)
  const float b = 22.8;      // Forearm length (cm)
  
  // Correction factors to align servo values with their respective axis
  const float SSHCorrectionFactor = 0;     // Align arm "a" with the horizontal when at 0 degrees
  const float ELCorrectionFactor = 0;     // Align arm "b" with arm "a" when at 0 degrees
  
  // Correction factor to shift origin out to edge of the mount
  const float X_CorrectionFactor  = 0;       // X direction correction factor (cm)
  const float Y_CorrectionFactor  = 0;       // Y direction correction factor (cm)
  
  float A ;            //Angle oppposite side a (between b and c)
  float B ;            //Angle oppposite side b
  float C ;            //Angle oppposite side c
  float theta;        //Angle formed between line from origin to (x,y) and the horizontal
  float x;            // x position (cm)
  float y;            // y position (cm)
  float c;            // Hypotenuse legngth in cm
  const float pi = M_PI;
  int GOpen = 100;    // Servo angle for open gripper
  int GClose = 10;    // Servo angle for closed gripper
  public:
    LED (Servo sh, Servo el, Servo gr){
      ServoSH = sh;
      ServoEL = el;
      ServoGR = gr;
    }
    void PointArm(float x_input, float y_input){
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

}

#define arren( x )  ( sizeof( x ) / sizeof( *x ) )
