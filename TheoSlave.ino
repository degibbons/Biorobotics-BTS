#include <SPI.h>                //include the pinout library
#include <SD.h>                 //include the sd interface library
#include <stdio.h>
#include <ax12.h>               //include base library for DYNAMIXELs
#include <BioloidController.h>  //include bioloid libary for poses/movements
#include "poses.h"              //home and center poses for the robot
#include <Wire.h>
#include <Arduino.h>
#include <string.h>
#include <math.h>

BioloidController bioloid = BioloidController(1000000); //start the BIOLOID library at 1mbps. 

#define PIN_SPI_SS    (5)
#define PIN_SPI_MOSI  (6)
#define PIN_SPI_MISO  (7)
#define PIN_SPI_SCK   (8)
#define SLAVE_ADDRESS 0x13

const int SERVOCOUNT = 16;  //number of servos in this robot
int id;                    //temperary id for movement
char temp[32];
char *ptr;
long int inNum;
int stopVal = 0;
int stopPin = 21; int contPin = 22;
int resPin = 23; int endPin = 20;
int startPin = 18;
int pinstate;
int speeds[21][16];
int breakCondition = 0;
int torq1; int volt1; int curr1; 
int torq2; int volt2; int curr2; 
int torq3; int volt3; int curr3; 
int torq4; int volt4; int curr4; 
int torq5; int volt5; int curr5; 
int torq6; int volt6; int curr6; 
int torq7; int volt7; int curr7; 
int torq8; int volt8; int curr8; 
int torq9;  int volt9; int curr9;
int torq10; int volt10; int curr10;
int torq11; int volt11; int curr11; 
int torq12; int volt12; int curr12;
int torq13; int volt13; int curr13; 
int torq14; int volt14; int curr14; 
int torq15; int volt15; int curr15; 
int torq16;  int volt16; int curr16;
int row; int col;
int w = 2; // index corresponds to 10 in percPosVals
int x = 17; // index corresponds to 85 in percPosVals
int y = 16; // index corresponds to 80 in percPosVals
int z = 0; // index corresponds to 0 in percPosVals
int phaseA = 0; int phaseB = 0;
int phaseC = 0; int phaseD = 0;
int indicator = 0;
float m1, m2, m3, m4, m5, m6, m7, m8, m9, m10, m11, m12, m13, m14, m15, m16;
float E1, E2, E3, E4, E5, E6, E7, E8, E9, E10, E11, E12, E13, E14, E15, E16;
float E17, E18, E19, E20, E21, E22, E23, E24, E25, E26, E27, E28, E29, E30, E31, E32;
float s1, s2, s3, s4, s5, s6, s7, s8, s9, s10, s11, s12, s13, s14, s15, s16;
int c1 = 0; int c2 = 1; int c3 = 2; int c4 = 3;
int c5 = 0; int c6 = 1; int c7 = 2; int c8 = 3;
int c9 = 8; int c10 = 9; int c11 = 10; int c12 = 11;
int c13 = 8; int c14 = 9; int c15 = 10; int c16 = 11;
int Acheck, Bcheck, Ccheck, Dcheck;
int percPosVals[21] = {0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100};

void receiveData(int HowMany){
    for (int i = 0; i < HowMany; i++) {
    temp[i] = Wire.read();
    temp[i + 1] = '\0'; //add null after ea. char
  }
  //RPi first byte is cmd byte so shift everything to the left 1 pos so temp contains our string
  for (int i = 0; i < HowMany; ++i) {
    temp[i] = temp[i + 1];
  }
  if (indicator == 0)
  {
    if (0 == strcmp(temp,"speeds")){
    indicator = 1;
    }
  }
  else if (indicator == 1)
  {
    inNum = strtol(temp, &ptr, 10);
    if (inNum > 0){
      speeds[row][col] = inNum;
      row = row + 1;
      if (row > 21){
        row = 0;
        col = col + 1;
      }
    }
    else if (0 == strcmp(temp,"fin")){
      indicator = 2;
      breakCondition = 1;
    }
  }
  }

void setup() {
  Serial.begin(9600);
  //for the stop input
  pinMode(stopPin, INPUT);
  pinMode(contPin, INPUT);
  pinMode(resPin, INPUT);
  pinMode(startPin, INPUT);
  pinMode(endPin, INPUT);

while (!Serial) {
    continue; // wait for serial port to connect. Needed for native USB port only
  }
// Do we need this?? ^^^


Wire.begin(SLAVE_ADDRESS);
Wire.onReceive(receiveData);

m1 = ((6.817877) * pow(10,-5) * pow(percPosVals[w],4) + ((-0.162070) * pow(percPosVals[w],3)) + (1.267515) * pow(percPosVals[w],2) + -39.010520 * percPosVals[w] + 2506.230418);
m2 = ((3.809397) * pow(10,-5) * pow(percPosVals[w],4) + ((-0.006441) * pow(percPosVals[w],3)) +  (0.252559) * pow(percPosVals[w],2) + -0.740270 * percPosVals[w] + 1995.735805);
m3 = ((-3.207349) * pow(10,-5) * pow(percPosVals[w],4) + ((0.00598476) * pow(percPosVals[w],3)) + -0.505305 * pow(percPosVals[w],2) + 15.340435 * percPosVals[w] + 2305.756793);
m4 = ((1.749753) * pow(10,-4) * pow(percPosVals[w],4) + ((-0.0340130) * pow(percPosVals[w],3)) + 2.123775 * pow(percPosVals[w],2) + (-51.211922) * percPosVals[w] + 2304.650519);
m5 = ((-4.769704) * pow(10,-5) * (pow(percPosVals[x],4)) +(0.00730001) * (pow(percPosVals[x],3)) + (-0.159904) * pow(percPosVals[x],2) +  (-5.320965) * percPosVals[x] + 1920.992153);
m6 = ((1.398389) * pow(10,-5) * (pow(percPosVals[x],4)) + (-0.00139954) * (pow(percPosVals[x],3)) + (-0.0141368) * pow(percPosVals[x],2) + (3.999583) * percPosVals[x] + 1777.0442639);
m7 = ((-7.017184) * pow(10,-5) * (pow(percPosVals[x],4)) + (0.00979741) * (pow(percPosVals[x],3)) + (-0.0211076) * pow(percPosVals[x],2) + (-19.963550) * percPosVals[x] + 1722.781680);
m8 = ((8.721547) * pow(10,-6) * (pow(percPosVals[x],4)) + (-0.00618508) * (pow(percPosVals[x],3)) + (0.651992) * pow(percPosVals[x],2) + (-10.356281) * percPosVals[x] + 1866.605480);
m9 = ((2.938367) * pow(10,-5) * pow(percPosVals[y],4) + (-0.00347977) * pow(percPosVals[y],3) + (-0.00569714) * pow(percPosVals[y],2) + (-4.534130) * percPosVals[y] + 2662.454749);
m10 = ((1.469778) * pow(10,-5) * pow(percPosVals[y],4) + (-0.00388263) * pow(percPosVals[y],3) + (0.341401) * pow(percPosVals[y],2) + (-12.819534) * percPosVals[y] + 2314.296035);
m11 = ((4.922065) * pow(10,-5) * pow(percPosVals[y],4) + (-0.00855236) * pow(percPosVals[y],3) + (0.508577) * pow(percPosVals[y],2) + (-18.483145) * percPosVals[y] + 2115.654455);
m12 = ((-7.623962) * pow(10,-5) * pow(percPosVals[y],4) + (0.0162755) * pow(percPosVals[y],3) + (-0.850935) * pow(percPosVals[y],2) + (-4.333270) * percPosVals[y] + 2671.475180);
m13 = ((-1.863341) * pow(10,-5) * pow(percPosVals[z],4) + (0.0305724) * pow(percPosVals[z],3) + (-1.050135) * pow(percPosVals[z],2) + (-4.891468) * percPosVals[z] + 1592.759820);
m14 = ((3.713302) * pow(10,-5) * pow(percPosVals[z],4) + (-0.00249816) * pow(percPosVals[z],3) + (-0.161114) * pow(percPosVals[z],2) + (8.069242) * percPosVals[z] + 2038.914946);
m15 = ((-3.967701) * pow(10,-5) * pow(percPosVals[z],4) + (0.00739745) * pow(percPosVals[z],3) + (-0.128276) * pow(percPosVals[z],2) + (-15.152598) * percPosVals[z] + 1545.974812);
m16 = ((-1.042285) * pow(10,-5) * pow(percPosVals[z],4) + (0.0263401) * pow(percPosVals[z],3) + (-2.175832) * pow(percPosVals[z],2) + (61.241992) * percPosVals[z] + 2310.794899);

w = w + 1;
x = x + 1;
y = y + 1;
z = z + 1;

dxlSetGoalPosition(1, m1); dxlSetGoalPosition(2, m2); dxlSetGoalPosition(3, m3);
dxlSetGoalPosition(4, m4); dxlSetGoalPosition(5, m1); dxlSetGoalPosition(6, m2);
dxlSetGoalPosition(7, m3); dxlSetGoalPosition(8, m4);
dxlSetGoalPosition(9,  m9); dxlSetGoalPosition(10, m10); dxlSetGoalPosition(11, m11);
dxlSetGoalPosition(12, m12); dxlSetGoalPosition(13, m9); dxlSetGoalPosition(14, m10);
dxlSetGoalPosition(15, m11); dxlSetGoalPosition(16, m12);
dxlSetGoalPosition(17, 2048); dxlSetGoalPosition(18, 2048); dxlSetGoalPosition(19,2048);
dxlSetGoalPosition(20,2048); dxlSetGoalPosition(21,2048); dxlSetGoalPosition(22,2048);
dxlSetGoalPosition(23,2048); dxlSetGoalPosition(24,2048);

Serial.println("Beginning Start Up Sequence.");

//Initialize the sd to be able to interface, read/write
Serial.print("\nInitializing SD card...");

while(1) {
  if (!SD.begin(4)) {
    Serial.println("No SD Card Present. Initialization Failed.");
    continue;
} else if (SD.begin(4)) {
  delay(1000);
  Serial.println("SD Card Detected. Initialization Success.");
  break;
  }
}

while (1) {
        if (breakCondition == 0){
                Serial.println("Have not received speeds.");
                Serial.println("Continuing wait loop.");
        }
        else if (breakCondition == 1){
                Serial.println("Speeds received.");
                Serial.println("Moving to Standby Mode.");
                break;
        }
}

File myFile;

while (1){
    int startState =   digitalRead(startPin);
    if (startState != 0){
      stopVal = 0;
      myFile = SD.open("test.txt", FILE_WRITE);

      // if the file opened okay, write to it:
//      if (myFile) {
        Serial.print("Writing to test.txt...");
        //Set the speed to get to the first position
        break; 
      }
}

while (stopVal == 0){
//All Stance Phase Equations
E1 = ((6.817877) * pow(10,-5) * pow(percPosVals[w],4) + ((-0.162070) * pow(percPosVals[w],3)) + (1.267515) * pow(percPosVals[w],2) + -39.010520 * percPosVals[w] + 2506.230418);
E2 = ((3.809397) * pow(10,-5) * pow(percPosVals[w],4) + ((-0.006441) * pow(percPosVals[w],3)) +  (0.252559) * pow(percPosVals[w],2) + -0.740270 * percPosVals[w] + 1995.735805);
E3 = ((-3.207349) * pow(10,-5) * pow(percPosVals[w],4) + ((0.00598476) * pow(percPosVals[w],3)) + -0.505305 * pow(percPosVals[w],2) + 15.340435 * percPosVals[w] + 2305.756793);
E4 = ((1.749753) * pow(10,-4) * pow(percPosVals[w],4) + ((-0.0340130) * pow(percPosVals[w],3)) + 2.123775 * pow(percPosVals[w],2) + (-51.211922) * percPosVals[w] + 2304.650519);
E5 = ((6.817877) * pow(10,-5) * pow(percPosVals[x],4) + ((-0.162070) * pow(percPosVals[x],3)) + (1.267515) * pow(percPosVals[x],2) + -39.010520 * percPosVals[x] + 2506.230418);
E6 = ((3.809397) * pow(10,-5) * pow(percPosVals[x],4) + ((-0.006441) * pow(percPosVals[x],3)) +  (0.252559) * pow(percPosVals[x],2) + -0.740270 * percPosVals[x] + 1995.735805);
E7 = ((-3.207349) * pow(10,-5) * pow(percPosVals[x],4) + ((0.00598476) * pow(percPosVals[x],3)) + -0.505305 * pow(percPosVals[x],2) + 15.340435 * percPosVals[x] + 2305.756793);
E8 = ((1.749753) * pow(10,-4) * pow(percPosVals[x],4) + ((-0.0340130) * pow(percPosVals[x],3)) + 2.123775 * pow(percPosVals[x],2) + (-51.211922) * percPosVals[x] + 2304.650519);
E9 = ((2.938367) * pow(10,-5) * pow(percPosVals[y],4) + (-0.00347977) * pow(percPosVals[y],3) + (-0.00569714) * pow(percPosVals[y],2) + (-4.534130) * percPosVals[y] + 2662.454749);
E10 = ((1.469778) * pow(10,-5) * pow(percPosVals[y],4) + (-0.00388263) * pow(percPosVals[y],3) + (0.341401) * pow(percPosVals[y],2) + (-12.819534) * percPosVals[y] + 2314.296035);
E11 = ((4.922065) * pow(10,-5) * pow(percPosVals[y],4) + (-0.00855236) * pow(percPosVals[y],3) + (0.508577) * pow(percPosVals[y],2) + (-18.483145) * percPosVals[y] + 2115.654455);
E12 = ((-7.623962) * pow(10,-5) * pow(percPosVals[y],4) + (0.0162755) * pow(percPosVals[y],3) + (-0.850935) * pow(percPosVals[y],2) + (-4.333270) * percPosVals[y] + 2671.475180);
E13 = ((2.938367) * pow(10,-5) * pow(percPosVals[z],4) + (-0.00347977) * pow(percPosVals[z],3) + (-0.00569714) * pow(percPosVals[z],2) + (-4.534130) * percPosVals[z] + 2662.454749);
E14 = ((1.469778) * pow(10,-5) * pow(percPosVals[z],4) + (-0.00388263) * pow(percPosVals[z],3) + (0.341401) * pow(percPosVals[z],2) + (-12.819534) * percPosVals[z] + 2314.296035);
E15 = ((4.922065) * pow(10,-5) * pow(percPosVals[z],4) + (-0.00855236) * pow(percPosVals[z],3) + (0.508577) * pow(percPosVals[z],2) + (-18.483145) * percPosVals[z] + 2115.654455);
E16 = ((-7.623962) * pow(10,-5) * pow(percPosVals[z],4) + (0.0162755) * pow(percPosVals[z],3) + (-0.850935) * pow(percPosVals[z],2) + (-4.333270) * percPosVals[z] + 2671.475180);

E17 = ((-4.769704) * pow(10,-5) * (pow(percPosVals[w],4)) +(0.00730001) * (pow(percPosVals[w],3)) + (-0.159904) * pow(percPosVals[w],2) +  (-5.320965) * percPosVals[w] + 1920.992153);
E18 = ((1.398389) * pow(10,-5) * (pow(percPosVals[w],4)) + (-0.00139954) * (pow(percPosVals[w],3)) + (-0.0141368) * pow(percPosVals[w],2) + (3.999583) * percPosVals[w] + 1777.0442639);
E19 = ((-7.017184) * pow(10,-5) * (pow(percPosVals[w],4)) + (0.00979741) * (pow(percPosVals[w],3)) + (-0.0211076) * pow(percPosVals[w],2) + (-19.963550) * percPosVals[w] + 1722.781680);
E20 = ((8.721547) * pow(10,-6) * (pow(percPosVals[w],4)) + (-0.00618508) * (pow(percPosVals[w],3)) + (0.651992) * pow(percPosVals[w],2) + (-10.356281) * percPosVals[w] + 1866.605480);
E21 = ((-4.769704) * pow(10,-5) * (pow(percPosVals[x],4)) +(0.00730001) * (pow(percPosVals[x],3)) + (-0.159904) * pow(percPosVals[x],2) +  (-5.320965) * percPosVals[x] + 1920.992153);
E22 = ((1.398389) * pow(10,-5) * (pow(percPosVals[x],4)) + (-0.00139954) * (pow(percPosVals[x],3)) + (-0.0141368) * pow(percPosVals[x],2) + (3.999583) * percPosVals[x] + 1777.0442639);
E23 = ((-7.017184) * pow(10,-5) * (pow(percPosVals[x],4)) + (0.00979741) * (pow(percPosVals[x],3)) + (-0.0211076) * pow(percPosVals[x],2) + (-19.963550) * percPosVals[x] + 1722.781680);
E24 = ((8.721547) * pow(10,-6) * (pow(percPosVals[x],4)) + (-0.00618508) * (pow(percPosVals[x],3)) + (0.651992) * pow(percPosVals[x],2) + (-10.356281) * percPosVals[x] + 1866.605480);
E25 = ((-1.863341) * pow(10,-5) * pow(percPosVals[y],4) + (0.0305724) * pow(percPosVals[y],3) + (-1.050135) * pow(percPosVals[y],2) + (-4.891468) * percPosVals[y] + 1592.759820);
E26 = ((3.713302) * pow(10,-5) * pow(percPosVals[y],4) + (-0.00249816) * pow(percPosVals[y],3) + (-0.161114) * pow(percPosVals[y],2) + (8.069242) * percPosVals[y] + 2038.914946);
E27 = ((-3.967701) * pow(10,-5) * pow(percPosVals[y],4) + (0.00739745) * pow(percPosVals[y],3) + (-0.128276) * pow(percPosVals[y],2) + (-15.152598) * percPosVals[y] + 1545.974812);
E28 = ((-1.042285) * pow(10,-5) * pow(percPosVals[y],4) + (0.0263401) * pow(percPosVals[y],3) + (-2.175832) * pow(percPosVals[y],2) + (61.241992) * percPosVals[y] + 2310.794899);
E29 = ((-1.863341) * pow(10,-5) * pow(percPosVals[z],4) + (0.0305724) * pow(percPosVals[z],3) + (-1.050135) * pow(percPosVals[z],2) + (-4.891468) * percPosVals[z] + 1592.759820);
E30 = ((3.713302) * pow(10,-5) * pow(percPosVals[z],4) + (-0.00249816) * pow(percPosVals[z],3) + (-0.161114) * pow(percPosVals[z],2) + (8.069242) * percPosVals[z] + 2038.914946);
E31 = ((-3.967701) * pow(10,-5) * pow(percPosVals[z],4) + (0.00739745) * pow(percPosVals[z],3) + (-0.128276) * pow(percPosVals[z],2) + (-15.152598) * percPosVals[z] + 1545.974812);
E32 = ((-1.042285) * pow(10,-5) * pow(percPosVals[z],4) + (0.0263401) * pow(percPosVals[z],3) + (-2.175832) * pow(percPosVals[z],2) + (61.241992) * percPosVals[z] + 2310.794899);

if (phaseA == 0){
        s1 = E1;
        s2 = E2;
        s3 = E3;
        s4 = E4;
        }
        else if (phaseA == 1){
        s1 = E17;
        s2 = E18;
        s3 = E19;
        s4 = E20;
        }

        if (phaseB == 0){
        s5 = E5;
        s6 = E6;
        s7 = E7;
        s8 = E8;
        }
        else if (phaseB == 1){
        s5 = E21;
        s6 = E22;
        s7 = E23;
        s8 = E24;  
        }

        if (phaseC == 0){
        s9 = E9;
        s10 = E10;
        s11 = E11;
        s12 = E12;
        }
        else if (phaseC == 1){
        s9 = E25;
        s10 = E26;
        s11 = E27;
        s12 = E28; 
        }

        if (phaseD == 0){
        s13 = E13;
        s14 = E14;
        s15 = E15;
        s16 = E16;
        }
        else if (phaseD == 1){
        s13 = E29;
        s14 = E30;
        s15 = E31;
        s16 = E32;  
        }

torq1 = dxlGetTorque(1); volt1 = dxlGetVoltage(1); curr1 = mxGetCurrent(1); 
torq2 = dxlGetTorque(2); volt2 = dxlGetVoltage(2); curr2 = mxGetCurrent(2); 
torq3 = dxlGetTorque(3); volt3 = dxlGetVoltage(3); curr3 = mxGetCurrent(3); 
torq4 = dxlGetTorque(4); volt4 = dxlGetVoltage(4); curr4 = mxGetCurrent(4); 
torq5 = dxlGetTorque(5); volt5 = dxlGetVoltage(5); curr5 = mxGetCurrent(5); 
torq6 = dxlGetTorque(6); volt6 = dxlGetVoltage(6); curr6 = mxGetCurrent(6); 
torq7 = dxlGetTorque(7); volt7 = dxlGetVoltage(7); curr7 = mxGetCurrent(7); 
torq8 = dxlGetTorque(8); volt8 = dxlGetVoltage(8); curr8 = mxGetCurrent(8); 
torq9 = dxlGetTorque(9); volt9 = dxlGetVoltage(9); curr9 = mxGetCurrent(9); 
torq10 = dxlGetTorque(10); volt10 = dxlGetVoltage(10); curr10 = mxGetCurrent(10); 
torq11 = dxlGetTorque(11); volt11 = dxlGetVoltage(11); curr11 = mxGetCurrent(11); 
torq12 = dxlGetTorque(12); volt12 = dxlGetVoltage(12); curr12 = mxGetCurrent(12); 
torq13 = dxlGetTorque(13); volt13 = dxlGetVoltage(13); curr13 = mxGetCurrent(13); 
torq14 = dxlGetTorque(14); volt14 = dxlGetVoltage(14); curr14 = mxGetCurrent(14); 
torq15 = dxlGetTorque(15); volt15 = dxlGetVoltage(15); curr15 = mxGetCurrent(15); 
torq16 = dxlGetTorque(16); volt16 = dxlGetVoltage(16); curr16 = mxGetCurrent(16); 



myFile.println("SERVO_1: ");
String torqValue1 = "Torque: "; torqValue1 += torq1; myFile.println(torqValue1);
String voltValue1 = "Voltage: "; voltValue1 += volt1; myFile.println(voltValue1);
String currValue1 = "Current: "; currValue1 += curr1; myFile.println(currValue1);
String nx = "Move#: "; nx += x; myFile.println(nx);
myFile.println("\n");
            
myFile.println("SERVO_2: ");
String torqValue2 = "Torque: "; torqValue2 += torq2; myFile.println(torqValue2);
String voltValue2 = "Voltage: "; voltValue2 += volt2; myFile.println(voltValue2);
String currValue2 = "Current: "; currValue2 += curr2; myFile.println(currValue2);
myFile.println(nx);
myFile.println("\n");

myFile.println("SERVO_3: ");
String torqValue3 = "Torque: "; torqValue3 += torq3; myFile.println(torqValue3);
String voltValue3 = "Voltage: "; voltValue3 += volt3; myFile.println(voltValue3);
String currValue3 = "Current: "; currValue3 += curr3; myFile.println(currValue3);
myFile.println(nx);
myFile.println("\n");

myFile.println("SERVO_4: ");
String torqValue4 = "Torque: "; torqValue4 += torq4; myFile.println(torqValue4);
String voltValue4 = "Voltage: "; voltValue4 += volt4; myFile.println(voltValue4);
String currValue4 = "Current: "; currValue4 += curr4; myFile.println(currValue4);
myFile.println(nx);
myFile.println("\n");

myFile.println("SERVO_5: ");
String torqValue5 = "Torque: "; torqValue5 += torq5; myFile.println(torqValue5);
String voltValue5 = "Voltage: "; voltValue5 += volt5; myFile.println(voltValue5);
String currValue5 = "Current: "; currValue5 += curr5; myFile.println(currValue5);
myFile.println(nx); 
myFile.println("\n");

myFile.println("SERVO_6: ");
String torqValue6 = "Torque: "; torqValue6 += torq6; myFile.println(torqValue6);
String voltValue6 = "Voltage: "; voltValue6 += volt6; myFile.println(voltValue6);
String currValue6 = "Current: "; currValue6 += curr6; myFile.println(currValue6);
myFile.println(nx); 
myFile.println("\n");

myFile.println("SERVO_7: ");
String torqValue7 = "Torque: "; torqValue7 += torq7; myFile.println(torqValue7);
String voltValue7 = "Voltage: "; voltValue7 += volt7; myFile.println(voltValue7);
String currValue7 = "Current: "; currValue7 += curr7; myFile.println(currValue7);
myFile.println(nx); 
myFile.println("\n");

myFile.println("SERVO_8: ");
String torqValue8 = "Torque: "; torqValue8 += torq8; myFile.println(torqValue8);
String voltValue8 = "Voltage: "; voltValue8 += volt8; myFile.println(voltValue8);
String currValue8 = "Current: "; currValue8 += curr8; myFile.println(currValue8); 
myFile.println(nx); 
myFile.println("\n");

myFile.println("SERVO_9: ");
String torqValue9 ="Torque: "; torqValue9 += torq9; myFile.println(torqValue9);
String voltValue9 = "Voltage: "; voltValue9 += volt9; myFile.println(voltValue9);
String currValue9 = "Current: "; currValue9 += curr9; myFile.println(currValue9);
myFile.println(nx);
myFile.println("\n");

myFile.println("SERVO_10: ");
String torqValue10 ="Torque: "; torqValue10 += torq10; myFile.println(torqValue10);
String voltValue10 = "Voltage: "; voltValue10 += volt10; myFile.println(voltValue10);
String currValue10 = "Current: "; currValue10 += curr10; myFile.println(currValue10);
myFile.println(nx);
myFile.println("\n");

myFile.println("SERVO_11: ");
String torqValue11 ="Torque: "; torqValue11 += torq11; myFile.println(torqValue11);
String voltValue11 = "Voltage: "; voltValue11 += volt11; myFile.println(voltValue11);
String currValue11 = "Current: "; currValue11 += curr11; myFile.println(currValue11);
myFile.println(nx);
myFile.println("\n");

myFile.println("SERVO_12: ");
String torqValue12 ="Torque: "; torqValue12 += torq12; myFile.println(torqValue12);
String voltValue12 = "Voltage: "; voltValue12 += volt12; myFile.println(voltValue12);
String currValue12 = "Current: "; currValue12 += curr12; myFile.println(currValue12);
myFile.println(nx);
myFile.println("\n");

myFile.println("SERVO_13: ");
String torqValue13 ="Torque: "; torqValue13 += torq13; myFile.println(torqValue13);
String voltValue13 = "Voltage: "; voltValue13 += volt13; myFile.println(voltValue13);
String currValue13 = "Current: "; currValue13 += curr13; myFile.println(currValue13);
myFile.println(nx); 
myFile.println("\n");

myFile.println("SERVO_14: ");
String torqValue14 ="Torque: "; torqValue14 += torq14; myFile.println(torqValue14);
String voltValue14 = "Voltage: "; voltValue14 += volt14; myFile.println(voltValue14);
String currValue14 = "Current: "; currValue14 += curr14; myFile.println(currValue14);
myFile.println(nx);  
myFile.println("\n");

myFile.println("SERVO_15: ");
String torqValue15 ="Torque: "; torqValue15 += torq15; myFile.println(torqValue15);
String voltValue15 = "Voltage: "; voltValue15 += volt15; myFile.println(voltValue15);
String currValue15 = "Current: "; currValue15 += curr15; myFile.println(currValue15);
myFile.println(nx); 
myFile.println("\n");

myFile.println("SERVO_16: ");
String torqValue16 ="Torque: "; torqValue16 += torq16; myFile.println(torqValue16);
String voltValue16 = "Voltage: "; voltValue16 += volt16; myFile.println(voltValue16);
String currValue16 = "Current: "; currValue16 += curr16; myFile.println(currValue16);
myFile.println(nx); 
myFile.println("\n");

dxlSetGoalSpeed(1,speeds[w][c1]);
dxlSetGoalSpeed(2,speeds[w][c2]);
dxlSetGoalSpeed(3,speeds[w][c3]);
dxlSetGoalSpeed(4,speeds[w][c4]);
dxlSetGoalSpeed(5,speeds[x][c5]);
dxlSetGoalSpeed(6,speeds[x][c6]);
dxlSetGoalSpeed(7,speeds[x][c7]);
dxlSetGoalSpeed(8,speeds[x][c8]);
dxlSetGoalSpeed(9,speeds[y][c9]);
dxlSetGoalSpeed(10,speeds[y][c10]);
dxlSetGoalSpeed(11,speeds[y][c11]);
dxlSetGoalSpeed(12,speeds[y][c12]);
dxlSetGoalSpeed(13,speeds[z][c13]);
dxlSetGoalSpeed(14,speeds[z][c14]);
dxlSetGoalSpeed(15,speeds[z][c15]);
dxlSetGoalSpeed(16,speeds[z][c16]);

dxlSetGoalPosition(1,s1); dxlSetGoalPosition(2,s2);
dxlSetGoalPosition(3,s3); dxlSetGoalPosition(4,s4);
dxlSetGoalPosition(5,s5); dxlSetGoalPosition(6,s6);
dxlSetGoalPosition(7,s7); dxlSetGoalPosition(8,s8);
dxlSetGoalPosition(9,s9); dxlSetGoalPosition(10,s10);
dxlSetGoalPosition(11,s11); dxlSetGoalPosition(12,s12);
dxlSetGoalPosition(13,s13); dxlSetGoalPosition(14,s14);
dxlSetGoalPosition(15,s15); dxlSetGoalPosition(16,s16);

Serial.println("Move!");
Serial.println(s1);
Serial.println(s2);
Serial.println(s3);
Serial.println(s4);
Serial.println("\n");

Serial.println(s9);
Serial.println(s10);
Serial.println(s11);
Serial.println(s12);
Serial.println("\n");


    w = w + 1;
    x = x + 1;
    y = y + 1;
    z = z + 1;

    if (w > 20){
            w = 0;
        }
        if (x > 20){
            x = 0;
        }
        if (y > 20){
            y = 0;
        }
        if (z > 20){
            z = 0;
        }

        Acheck = percPosVals[w];
        Bcheck = percPosVals[x];
        Ccheck = percPosVals[y];
        Dcheck = percPosVals[z];

        if (Acheck == 0){
            if (phaseA == 0){
                phaseA = 1;
            }
            else if (phaseA == 1)
                phaseA = 0;
        }
        else if (Acheck == 5){
            if (phaseA == 0){
                c1 = c1 - 4;
                c2 = c2 - 4;
                c3 = c3 - 4;
                c4 = c4 - 4;
            }
            else if (phaseA == 1){
                c1 = c1 + 4;
                c2 = c2 + 4;
                c3 = c3 + 4;
                c4 = c4 + 4;
            }
        }

        if (Bcheck == 0){
            if (phaseB == 0){
                phaseB = 1;
            }
            else if (phaseB == 1)
                phaseB = 0;
        }
        else if (Bcheck == 5){
            if (phaseB == 0){
                c5 = c5 - 4;
                c6 = c6 - 4;
                c7 = c7 - 4;
                c8 = c8 - 4;
            }
            else if (phaseB == 1){
                c5 = c5 + 4;
                c6 = c6 + 4;
                c7 = c7 + 4;
                c8 = c8 + 4;
            }
        }

        if (Ccheck == 0){
            if (phaseC == 0){
                phaseC = 1;
            }
            else if (phaseC == 1)
                phaseC = 0;
        }
        else if (Ccheck == 5){
            if (phaseC == 0){
                c9 = c9 - 4;
                c10 = c10 - 4;
                c11 = c11 - 4;
                c12 = c12 - 4;
            }
            else if (phaseC == 1){
                c9 = c9 + 4;
                c10 = c10 + 4;
                c11 = c11 + 4;
                c12 = c12 + 4;
            }
        }

        if (Dcheck == 0){
            if (phaseD == 0){
                phaseD = 1;
            }
            else if (phaseD == 1)
                phaseD = 0;
        }
        else if (Dcheck == 5){
            if (phaseD == 0){
                c13 = c13 - 4;
                c14 = c14 - 4;
                c15 = c15 - 4;
                c16 = c16 - 4;
            }
            else if (phaseD == 1){
                c13 = c13 + 4;
                c14 = c14 + 4;
                c15 = c15 + 4;
                c16 = c16 + 4;
            }
        }
int pinstate = digitalRead(stopPin);
Serial.println("Running Stance");
if (pinstate != 0){
          while (1){
            int contPinState = digitalRead(contPin);
            int resPinState = digitalRead(resPin);
            int stopPinState = digitalRead(stopPin);
            int endPinState = digitalRead(endPin);
            if (contPinState == 1 && resPinState == 0 && endPinState == 0){
              break;
            }
            else if (contPinState == 0 && resPinState == 1 && endPinState == 0){
              dxlSetGoalPosition(1, 2048); dxlSetGoalPosition(2, 2048); dxlSetGoalPosition(3, 2048);
              dxlSetGoalPosition(4, 2048); dxlSetGoalPosition(5, 2048); dxlSetGoalPosition(6, 2048);
              dxlSetGoalPosition(7, 2048); dxlSetGoalPosition(8, 2048);
              dxlSetGoalPosition(9, 2048); dxlSetGoalPosition(10, 2048); dxlSetGoalPosition(11, 2048);
              dxlSetGoalPosition(12, 2048); dxlSetGoalPosition(13, 2048); dxlSetGoalPosition(14, 2048);
              dxlSetGoalPosition(15, 2048); dxlSetGoalPosition(16, 2048);
            }
            else if (contPinState == 0 && resPinState == 0 && endPinState == 1){
              stopVal = 1;
              break;
            }
            else if (contPinState == 0 && resPinState == 0 && endPinState == 0){
              continue;
            }
        }
        if (stopVal == 1){
          myFile.close();
          Serial.println("done.");
          break;
        }
}
}
}

void loop(){}
