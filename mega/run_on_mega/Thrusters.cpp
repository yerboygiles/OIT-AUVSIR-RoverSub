/*
    Authors: Theodor Giles
    Created: 10/29/21
    Last Edited 10/29/21
    Description:
    class for thruster hardpoint control
*/
class Thruster(){
  Servo motor;      // motor driver
  
  // Servo Angles
  float ThrusterSignal = 0

  public:
    Thruster (Servo motor){
      Motor = motor;
      ServoEL = el;
      ServoGR = gr;
    }
    void Drive(int power){ // -100 to 100
        Motor.writeMicroseconds(1500+(power*4));
    }
    void Calibrate(){
        print("Yep. Calibrated.")
    }
}

