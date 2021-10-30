/*!python3
    Author: Theodor Giles
    Created: 7/15/21
    Last Edited 7/28/21
    Description:
    File to be sent to the arduino mega.
    Manages all arduino functions, including
    communication, arm/thruster driving,
    and gyroscope communication
*/

#include <Servo.h>
#include <Wire.h>
#include <JY901.h>

bool initimu = false;
bool usingimu = false;

byte LBpin = 2; //left back
byte LFpin = 3; //left front
byte RBpin = 4; //right back
byte RFpin = 5; //right front
byte BLpin = 6; //back left
byte BRpin = 7; //back right
byte FLpin = 8; //front left
byte FRpin = 9; //front right

Servo LBthruster;
Servo LFthruster;
Servo RBthruster;
Servo RFthruster;
Servo BLthruster;
Servo BRthruster;
Servo FLthruster;
Servo FRthruster;

bool thrustersValid;
bool secondSet;
bool stopped = true;
int thrusterpower[8];

int maxpower;
int lowerlim;
int higherlim;
int dutycycle;

String strings[8];

char *ptr = NULL;
int index;
int dataindex;

char input[40];

String strinput;
String thrusternamestr;
String thrusterpowerstr;


Servo ServoSH_R;      // Shoulder joint
Servo ServoEL_R;      // Elbow joint
Servo ServoGR_R;      // Gripper

// Servo Angles
float ServoSH_R_Angle = 90;
float ServoEL_R_Angle = 90;
float ServoGR_R_Angle = 90;

const float a_R = 18.2;      // Upper arm lenth (cm)
const float b_R = 22.8;      // Forearm length (cm)

// Correction factors to align servo values with their respective axis
const float SSHCorrectionFactor_R = 0;     // Align arm "a" with the horizontal when at 0 degrees
const float ELCorrectionFactor_R = 0;     // Align arm "b" with arm "a" when at 0 degrees

// Correction factor to shift origin out to edge of the mount
const float X_CorrectionFactor_R = 0;       // X direction correction factor (cm)
const float Y_CorrectionFactor_R = 0;       // Y direction correction factor (cm)

float A_R;            //Angle oppposite side a (between b and c)
float B_R;            //Angle oppposite side b
float C_R;            //Angle oppposite side c
float theta_R;        //Angle formed between line from origin to (x,y) and the horizontal
float x_R;            // x position (cm)
float y_R;            // y position (cm)
float c_R;            // Hypotenuse legngth in cm
const float pi = M_PI;
int GOpen_R = 100;    // Servo angle for open gripper
int GClose_R = 10;    // Servo angle for closed gripper

Servo ServoSH_L;      // Shoulder joint
Servo ServoEL_L;      // Elbow joint
Servo ServoGR_L;      // Gripper

// Servo Angles
float ServoSH_L_Angle = 90;
float ServoEL_L_Angle = 90;
float ServoGR_L_Angle = 90;

const float a_L = 18.2;      // Upper arm lenth (cm)
const float b_L = 22.8;      // Forearm length (cm)

// Correction factors to align servo values with their respective axis
const float SSHCorrectionFactor_L = 0;     // Align arm "a" with the horizontal when at 0 degrees
const float ELCorrectionFactor_L = 0;     // Align arm "b" with arm "a" when at 0 degrees

// Correction factor to shift origin out to edge of the mount
const float X_CorrectionFactor_L = 0;       // X direction correction factor (cm)
const float Y_CorrectionFactor_L = 0;       // Y direction correction factor (cm)

float A_L;            //Angle oppposite side a (between b and c)
float B_L;            //Angle oppposite side b
float C_L;            //Angle oppposite side c
float theta_L;        //Angle formed between line from origin to (x,y) and the horizontal
float x_L;            // x position (cm)
float y_L;            // y position (cm)
float c_L;            // Hypotenuse legngth in cm
int GOpen_L = 100;    // Servo angle for open gripper
int GClose_L = 10;    // Servo angle for closed gripper

#define arr_Ren( x )  ( sizeof( x ) / sizeof( *x ) )

void updateThrusters(void) {
    LBthruster.writeMicroseconds(thrusterpower[0]); // sending driving values to arduino
    LFthruster.writeMicroseconds(thrusterpower[1]);
    RBthruster.writeMicroseconds(thrusterpower[2]);
    RFthruster.writeMicroseconds(thrusterpower[3]);
    BLthruster.writeMicroseconds(thrusterpower[4]);
    BRthruster.writeMicroseconds(thrusterpower[5]);
    FLthruster.writeMicroseconds(thrusterpower[6]);
    FRthruster.writeMicroseconds(thrusterpower[7]);
}
void updateThrusters(int singleval) {
    for(int i=0; i<=7; i++){
        thrusterpower[i] = singleval;
    }
    LBthruster.writeMicroseconds(thrusterpower[0]); // sending driving values to arduino
    LFthruster.writeMicroseconds(thrusterpower[1]);
    RBthruster.writeMicroseconds(thrusterpower[2]);
    RFthruster.writeMicroseconds(thrusterpower[3]);
    BLthruster.writeMicroseconds(thrusterpower[4]);
    BRthruster.writeMicroseconds(thrusterpower[5]);
    FLthruster.writeMicroseconds(thrusterpower[6]);
    FRthruster.writeMicroseconds(thrusterpower[7]);
}

void setup() {
    Serial.begin(9600);
    Serial1.begin(115200);
    //JY901.StartIIC();
    Serial2.begin(9600);
    // put your setup code here, to run once:
    /*
    ServoSH_R.attach(9);
    ServoEL_R.attach(10);
    ServoGR_R.attach(11);

    ServoSH_L.attach(9);
    ServoEL_L.attach(10);
    ServoGR_L.attach(11);
    */
    LBthruster.attach(2);
    LFthruster.attach(3);
    RBthruster.attach(4);
    RFthruster.attach(5);
    BLthruster.attach(6);
    BRthruster.attach(7);
    FLthruster.attach(8);
    FRthruster.attach(9);
    updateThrusters(0);  // send "stop"/voltage off signal to ESC.
    Serial.println("IMU or NOIMU?"); Serial.println("");
    /* Initialise the sensor */
    while(!initimu){
        if(Serial.available() > 0){
            strinput = Serial.readStringUntil('\n');
            Serial.println(strinput.compareTo("IMU"));
            if (strinput.compareTo("IMU")==0){
                initimu = true;
                usingimu = true;
                //Serial1.println("Using IMU.");
                Serial.println("Using IMU.");
            }
            if (strinput.compareTo("NOIMU")==0){
                initimu = true;
                usingimu = false;
                //Serial1.println("Not using IMU.");
                Serial.println("Not using IMU.");
            }
        }
        strinput = "";
    }
    if(usingimu){
        //Serial1.println("Arming thrusters.");
        Serial.println("Arming thrusters.");
        updateThrusters(1500); // send "arm" signal to ESCs
        delay(5000);
        //Serial1.println("Warming up.");
        Serial.println("Warming up.");
        updateThrusters(1550); // low forwards signal to ESCs
        delay(1000);
        updateThrusters(1500); // send "arm" signal to ESCs, there's an issue with prolonged
        delay(3000);
        //Serial1.println("Killing thrusters.");
        Serial.println("Killing thrusters.");
        updateThrusters(0); // send "dead" signal to ESCs, theres an issue after prolonged time
                          // with random high movement here
    }
    Serial.println("Init over. Running main sys...");
    secondSet = false;
}



// https://stackoverflow.com/questions/9072320/split-string-into-string-array
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

void PointArm_L(float x_input, float y_input){
    x_L = x_input + X_CorrectionFactor_L;
    y_L = y_input + Y_CorrectionFactor_L;

    c_L = sqrt( sq(x_L) + sq(y_L) );                                            // pythagorean theorem
    B_L = (acos( (sq(b_L) - sq(a_L) - sq(c_L))/(-2*a_L*c_L) )) * (180/pi);            // Law of cosines: Angle opposite upper arm section
    C_L = (acos( (sq(c_L) - sq(b_L) - sq(a_L))/(-2*a_L*b_L) )) * (180/pi);            // Law of cosines: Angle opposite hypotenuse
    theta_L = (asin( y_L / c_L )) * (180/pi);                                   // Solve for theta to correct for lower joint's impact on upper joint's angle
    ServoSH_L_Angle = B_L + theta_L + SSHCorrectionFactor_L;                    // Find necessary angle. Add Correction
    ServoEL_L_Angle = C_L + ELCorrectionFactor_L;                            // Find neceesary angle. Add Correction

    ServoSH_L.write(ServoSH_L_Angle);
    ServoEL_L.write(ServoEL_L_Angle);
}
void PointArm_R(float x_input, float y_input){
    x_R = x_input + X_CorrectionFactor_R;
    y_R = y_input + Y_CorrectionFactor_R;

    c_R = sqrt( sq(x_R) + sq(y_R) );                                            // pythagorean theorem
    B_R = (acos( (sq(b_R) - sq(a_R) - sq(c_R))/(-2*a_R*c_R) )) * (180/pi);            // Law of cosines: Angle opposite upper arm section
    C_R = (acos( (sq(c_R) - sq(b_R) - sq(a_R))/(-2*a_R*b_R) )) * (180/pi);            // Law of cosines: Angle opposite hypotenuse
    theta_R = (asin( y_R / c_R )) * (180/pi);                                   // Solve for theta to correct for lower joint's impact on upper joint's angle
    ServoSH_R_Angle = B_R + theta_R + SSHCorrectionFactor_R;                    // Find necessary angle. Add Correction
    ServoEL_R_Angle = C_R + ELCorrectionFactor_R;                            // Find neceesary angle. Add Correction

    ServoSH_R.write(ServoSH_R_Angle);
    ServoEL_R.write(ServoEL_R_Angle);
}

void printWT61PGyro(Stream &serial) { 
  serial.print("Angle:");serial.print((float)JY901.stcAngle.Angle[0]/32768*180);serial.print(":");serial.print((float)JY901.stcAngle.Angle[1]/32768*180);serial.print(":");serial.println((float)JY901.stcAngle.Angle[2]/32768*180);
  serial.println("");
}
void printWT61PAccel(Stream &serial) { 
  serial.print("Acc:");serial.print((float)JY901.stcAcc.a[0]/32768*16);serial.print(" ");serial.print((float)JY901.stcAcc.a[1]/32768*16);serial.print(" ");serial.println((float)JY901.stcAcc.a[2]/32768*16);
  serial.println("");
}

void loop() {
    // put your main code here, to run repeatedly:
    int j = 0;
    if(Serial.available() > 0){
        strinput = Serial.readStringUntil('\n');
        if((strinput.compareTo("STOP")==0)) {
            Serial.println("THRUSTERS DISARMED <-");
            stopped = true;
            updateThrusters(0);
            delay(3000);
        }
        else if((getValue(strinput, ':', 0).compareTo("L ARM")==0)) {
            PointArm_L(getValue(strinput, ':', 1).toFloat(), getValue(strinput, ':', 2).toFloat());
        }
        else if((getValue(strinput, ':', 0).compareTo("R ARM")==0)) {
            PointArm_R(getValue(strinput, ':', 1).toFloat(), getValue(strinput, ':', 2).toFloat());
        }
        else if((strinput.compareTo("START")==0)) {
            Serial.println("THRUSTERS ARMED <-");
            stopped = false;
            updateThrusters(1500);
            delay(1000);
        }
        else if((getValue(strinput, ':', 0).compareTo("MAXPOWER")==0)) {
            Serial.println("POW SET <-");
            maxpower = getValue(strinput, ':', 1).toFloat();
            lowerlim = (1500-(400)*(maxpower/100));
            higherlim = (1500+(400)*(maxpower/100));
        }
        else if((strinput.compareTo("GYRO")==0)) {
            Serial.println("IMU ANG->");
            printWT61PGyro(Serial);
        }
        else if((strinput.compareTo("POSITION")==0)) {
            Serial.println("IMU POS ->");
            printWT61PPosition(Serial);
        } else {

            index = 0;
            dataindex = -1;
            while(getValue(strinput, ',', index) != NULL) {
                strings[index] = getValue(strinput, ',', index);
                thrusternamestr = getValue(strings[index], ':', 0);
                //         Serial.print("Name: ");
                //         Serial.print(thrusternamestr);
                //         Serial.print(", Power: ");
                //         Serial.println(thrusterpowerstr);
                dutycycle = thrusterpowerstr.toFloat();
                map(dutycycle, 1100, 1900, lowerlim, higherlim);
                if (thrusternamestr.compareTo("FR")==0){
                    thrusterpowerstr = getValue(strings[index], ':', 1);
                    thrusterpower[7] = dutycycle;
                }
                else if (thrusternamestr.compareTo("FL")==0){
                    thrusterpowerstr = getValue(strings[index], ':', 1);
                    thrusterpower[6] = dutycycle;
                }
                else if (thrusternamestr.compareTo("BR")==0){
                    thrusterpowerstr = getValue(strings[index], ':', 1);
                    thrusterpower[5] = dutycycle;
                }
                else if (thrusternamestr.compareTo("BL")==0){
                    thrusterpowerstr = getValue(strings[index], ':', 1);
                    thrusterpower[4] = dutycycle;
                }
                else if (thrusternamestr.compareTo("RF")==0){
                    thrusterpowerstr = getValue(strings[index], ':', 1);
                    thrusterpower[3] = dutycycle;
                }
                else if (thrusternamestr.compareTo("RB")==0){
                    thrusterpowerstr = getValue(strings[index], ':', 1);
                    thrusterpower[2] = dutycycle;
                }
                else if (thrusternamestr.compareTo("LF")==0){
                    thrusterpowerstr = getValue(strings[index], ':', 1);
                    thrusterpower[1] = dutycycle;
                }
                else if (thrusternamestr.compareTo("LB")==0){
                    thrusterpowerstr = getValue(strings[index], ':', 1);
                    thrusterpower[0] = dutycycle;
                }
                index++;
                //ptr = strtok(NULL, ":");  // takes a list of delimiters
            }
        }
        Serial.println("");
    }
    // dont write motors when stopped
    if(!stopped){
    updateThrusters();
    }
    while (Serial1.available()) 
    {
      JY901.CopeSerialData(Serial1.read()); //Call JY901 data cope function
    }
}
