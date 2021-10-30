/*!python3
    Author: Theodor Giles
    Created: 7/15/20
    Last Edited 5/14/21
    Description:
    File to be sent to the arduino mega.
    Manages all arduino functions, including
    communication, servo/thruster driving,
    and gyroscope communication
*/

#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.
   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.
   You should also assign a unique ID to this sensor for use with
   the Adafruit Sensor API so that you can identify this particular
   sensor in any data logs, etc.  To assign a unique ID, simply
   provide an appropriate value in the constructor below (12345
   is used by default in this example).
   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3-5V DC
   Connect GROUND to common ground
   History
   =======
   2015/MAR/03  - First release (KTOWN)
   2021/APR/16  - Integration into robosub (OTUS)
*/

double xPos = 0, yPos = 0, zPos = 0, headingVel = 0;
double xOffset, yOffset, zOffset;

int SystemCalib, GyroCalib, AccelCalib = 0;

uint16_t BNO055_SAMPLERATE_DELAY_MS = 100; //how often to read data from the board
uint16_t PRINT_DELAY_MS = 500; // how often to print the data
uint16_t printCount = 0; //counter to avoid printing every 10MS sample

double ACCEL_VEL_TRANSITION =  (double)(BNO055_SAMPLERATE_DELAY_MS) / 1000.0;
double ACCEL_POS_TRANSITION = 0.5 * ACCEL_VEL_TRANSITION * ACCEL_VEL_TRANSITION;
double DEG_2_RAD = 0.01745329251; //trig functions require radians, BNO055 outputs degrees

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

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

#define arr_len( x )  ( sizeof( x ) / sizeof( *x ) )

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

void displaySensorDetails(void) {
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}
void setup() {
  Serial1.begin(115200);
  Serial.begin(115200);
  // put your setup code here, to run once:
  LBthruster.attach(2);
  LFthruster.attach(3);
  RBthruster.attach(4);
  RFthruster.attach(5);
  BLthruster.attach(6);
  BRthruster.attach(7);
  FLthruster.attach(8);
  FRthruster.attach(9);
  updateThrusters(1500); // send "arm" signal to ESC. Also necessary to arm the ESC.
  delay(7000); // delay to allow the ESC to recognize the stopped signal.
  Serial1.println("Thrusters armed, resetting to stop.");
  updateThrusters(0);  // send "stop"/voltage off signal to ESC.
  Serial.println("Orientation Sensor Test."); Serial.println("");
  /* Initialise the sensor */
  if(!bno.begin()){
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial1.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  Serial.println("Calibration post start:");
  displayCalStatus();
  
  /* Use external crystal for better accuracy */
  bno.setExtCrystalUse(true);
   
  /* Display some basic information on this sensor */
  displaySensorDetails();
  secondSet = false;
  

}

void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
 
  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }
 
  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  SystemCalib = system;
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  GyroCalib = gyro;
  Serial.print(" A:");
  Serial.print(accel, DEC);
  AccelCalib = accel;
  Serial.print(" M:");
  Serial.println(mag, DEC);
}

void printEvent(sensors_event_t* event, Stream &serial) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ORIENTATION) {
    serial.print("Orient");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
    serial.print(":x:");
    if(x > 180) {
      serial.print((x - 180));
    } else if (x <= 180) {
      serial.print(-(x + 180));
    }
    serial.print(":y:");
    serial.print(y - yOffset);
    serial.print(":z:");
    serial.println(z - zOffset);
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
//    serial.print("Linear");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
    serial.print(":x:");
    serial.print(x + 180);
    serial.print(":y:");
    serial.print(y - yOffset);
    serial.print(":z:");
    serial.println(z - zOffset);
  }
  else {
    serial.print("Unk:");
  }
}

// https://stackoverflow.com/questions/9072320/split-string-into-string-array
String getValue(String data, char separator, int index)
{
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

void loop() {
  // put your main code here, to run repeatedly:
  
  sensors_event_t orientationData , angVelData , linearAccelData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

  xPos = xPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.x;
  yPos = yPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.y;
  zPos = zPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.z;
  
  headingVel = ACCEL_VEL_TRANSITION * linearAccelData.acceleration.x / 
               cos(DEG_2_RAD * orientationData.orientation.x);
  thrustersValid = true;
  //printEvent(&orientationData, Serial);
  //printEvent(&linearAccelData, Serial);
  //displayCalStatus();
  
  int j = 0;
  while(GyroCalib < 3){
    delay(1000);
    displayCalStatus();
    Serial.print("Calibrating... On run: ");
    Serial.print(j);
    Serial.println(". Wait 1.");
    j = j + 1;
    printEvent(&orientationData, Serial);
    xOffset = orientationData.orientation.x;
    yOffset = orientationData.orientation.y;
    zOffset = orientationData.orientation.z;
  if (!(GyroCalib < 3)){
    updateThrusters(1500); // send "arm" signal to ESCs
    delay(3000);
    updateThrusters(1600); // low forwards signal to ESCs
    delay(2000);
    updateThrusters(1500); // send "arm" signal to ESCs, there's an issue with prolonged 
    delay(3000);
    updateThrusters(0); // send "dead" signal to ESCs, theres an issue after prolonged time 
                        // with random high movement here
    }
  }
  if(Serial1.available() > 0){ 
     
    strinput = Serial1.readStringUntil('\n');
//    if((strinput.compareTo("INIT")==0)) {
//      Serial.println("Init. Wait 3.");
//      updateThrusters(0);
//      printEvent(&orientationData, Serial1);
//      printEvent(&linearAccelData, Serial1);
//      delay(3000);
//    }
    if((strinput.compareTo("STOP")==0)) {
      Serial.println("Thrusters disengaging. Wait 3.");
      stopped = true;
      updateThrusters(0);
      delay(3000);
    }
    else if((strinput.compareTo("START")==0)) {
      Serial.println("Thrusters engaging. Wait 3.");
      stopped = false;
      updateThrusters(1500);
      delay(3000);
    } 
    else if((getValue(strinput, ':', 0).compareTo("MAXPOWER")==0)) {
      Serial.println("Setting max power. Wait 1.");
      maxpower = getValue(strinput, ':', 1).toFloat();
      lowerlim = (1500-(400)*(maxpower/100));
      higherlim = (1500+(400)*(maxpower/100));
      delay(1000);
    } 
    else if((strinput.compareTo("GYRO")==0)) {
      Serial.println("Sending Gyro Data...");
      printEvent(&orientationData, Serial1);
    } 
    index = 0;
    dataindex = -1;
    while(getValue(strinput, ',', index) != NULL)
      {
        strings[index] = getValue(strinput, ',', index);
        thrusternamestr = getValue(strings[index], ':', 0);
        Serial.print("Name: ");
        Serial.print(thrusternamestr);
        Serial.print(", Power: ");
        Serial.println(thrusterpowerstr);
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
      Serial.println("");
    }
    // dont write motors when stopped
    if(!stopped){
      updateThrusters();
    }
  }
