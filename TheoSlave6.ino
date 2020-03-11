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

int id;                    //temperary id for movement
char temp[32]; 
char *ptr;
long int inNum;
int stopVal = 0;
const int PROGMEM stopPin = 21; const int PROGMEM contPin  =  22;
const int PROGMEM resPin = 23; const int PROGMEM endPin = 20;
const int PROGMEM startPin = 18;
int pinstate;
int speeds[11][16];
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
int row = 0; int col = 0;
int w = 1; // index corresponds to 10 in percPosVals
int x = 9; // index corresponds to 85 in percPosVals
int y = 8; // index corresponds to 80 in percPosVals
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
const int percPosVals[11] = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100};

int counterTest = 1;

int FL_ST_R_1[11] = {1680, 1838, 2001, 1998, 1990, 1981, 1975, 2008, 2127, 2177, 2197};
int FL_ST_R_2[11] = {2115, 2081, 2045, 2048, 2058, 2078, 2109, 2191, 2315, 2305, 2295};
int FL_ST_R_3[11] = {2338, 2401, 2467, 2451, 2407, 2345, 2277, 2204, 2068, 1811, 1653};
int FL_ST_R_4[11] = {2153, 2031, 1894, 1880, 1877, 1876, 1869, 1731, 1424, 1632, 1818};
int HL_ST_R_1[11] = {1443, 1480, 1541, 1652, 1786, 1901, 2136, 2234, 2422, 2494, 2499};
int HL_ST_R_2[11] = {1802, 1867, 1883, 1998, 2009, 1891, 1997, 1991, 2009, 2043, 2046};
int HL_ST_R_3[11] = {2121, 1971, 1899, 1819, 1767, 1680, 1676, 1544, 1503, 1594, 1661};
int HL_ST_R_4[11] = {2687, 2551, 2377, 2131, 1981, 1941, 1839, 1924, 2096, 2279, 2369};
int FL_ST_L_1[11] = {2416, 2258, 2095, 2098, 2106, 2115, 2121, 2088, 1969, 1919, 1899};
int FL_ST_L_2[11] = {2115, 2081, 2045, 2048, 2058, 2078, 2109, 2191, 2315, 2305, 2295};
int FL_ST_L_3[11] = {1758, 1695, 1629, 1645, 1689, 1751, 1819, 1892, 2028, 2285, 2443};
int FL_ST_L_4[11] = {2153, 2031, 1894, 1880, 1877, 1876, 1869, 1731, 1424, 1632, 1818};
int HL_ST_L_1[11] = {2653, 2616, 2555, 2444, 2310, 2195, 1960, 1862, 1674, 1602, 1597};
int HL_ST_L_2[11] = {1802, 1867, 1883, 1998, 2009, 1891, 1997, 1991, 2009, 2043, 2046};
int HL_ST_L_3[11] = {1975, 2125, 2197, 2277, 2329, 2416, 2420, 2552, 2593, 2502, 2435};
int HL_ST_L_4[11] = {2687, 2551, 2377, 2131, 1981, 1941, 1839, 1924, 2096, 2279, 2369};
int FL_SW_R_1[11] = {2197, 2227, 2288, 2329, 2307, 2220, 2092, 1949, 1816, 1718, 1680};
int FL_SW_R_2[11] = {2295, 2293, 2265, 2230, 2220, 2238, 2263, 2272, 2226, 2153, 2115};
int FL_SW_R_3[11] = {1653, 1563, 1392, 1282, 1323, 1468, 1675, 1906, 2120, 2277, 2338};
int FL_SW_R_4[11] = {1818, 1839, 1897, 1980, 2076, 2235, 2411, 2474, 2377, 2229, 2153};
int HL_SW_R_1[11] = {2500, 2638, 2806, 2893, 2893, 2759, 2407, 1913, 1593, 1438, 1427};
int HL_SW_R_2[11] = {2045, 1989, 1975, 2022, 2077, 2077, 2137, 2389, 2223, 1885, 1855};
int HL_SW_R_3[11] = {1660, 1297, 1171, 1225, 1234, 1136, 1136, 1378, 1737, 2039, 1978};
int HL_SW_R_4[11] = {2380, 2736, 2835, 2791, 2785, 2547, 2546, 2467, 2519, 2573, 2620};
int FL_SW_L_1[11] = {1899, 1869, 1808, 1767, 1789, 1876, 2004, 2147, 2280, 2378, 2416};
int FL_SW_L_2[11] = {2295, 2293, 2265, 2230, 2220, 2238, 2263, 2272, 2226, 2153, 2115};
int FL_SW_L_3[11] = {2443, 2533, 2704, 2814, 2773, 2628, 2421, 2190, 1976, 1819, 1758};
int FL_SW_L_4[11] = {1818, 1839, 1897, 1980, 2076, 2235, 2411, 2474, 2377, 2229, 2153};
int HL_SW_L_1[11] = {1596, 1458, 1290, 1203, 1203, 1337, 1689, 2183, 2503, 2658, 2669};
int HL_SW_L_2[11] = {2045, 1989, 1975, 2022, 2077, 2077, 2137, 2389, 2223, 1885, 1855};
int HL_SW_L_3[11] = {2436, 2799, 2925, 2871, 2862, 2960, 2960, 2718, 2359, 2057, 2118};
int HL_SW_L_4[11] = {2380, 2736, 2835, 2791, 2785, 2547, 2456, 2467, 2519, 2573, 2620};


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
//      Serial.println(F("Sending"));
      speeds[row][col] = inNum;
      row = row + 1;
//      Serial.println(inNum);
      if (row > 10){
        row = 0;
        col = col + 1;
      }
    }
    else if (0 == strcmp(temp,"fin")){
//      Serial.println(F("Finished Sending"));
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


Wire.begin(SLAVE_ADDRESS);
Wire.onReceive(receiveData);

m1 = FL_ST_R_1[w];
m2 = FL_ST_R_2[w];
m3 = FL_ST_R_3[w];
m4 = FL_ST_R_4[w];
m5 = FL_ST_L_1[x];
m6 = FL_ST_L_2[x];
m7 = FL_ST_L_3[x];
m8 = FL_ST_L_4[x];
m9 = HL_ST_R_1[y];
m10 = HL_ST_R_2[y];
m11 = HL_ST_R_3[y];
m12 = HL_ST_R_4[y];
m13 = HL_ST_L_1[z];
m14 = HL_ST_L_2[z];
m15 = HL_ST_L_3[z];
m16 = HL_ST_L_4[z]; 

//Serial.println("Servo 1 first position: ");
//Serial.println(m1);
//Serial.println("Servo 2 first position: ");
//Serial.println(m2);
//Serial.println("Servo 3 first position: ");
//Serial.println(m3);
//Serial.println("Servo 4 first position: ");
//Serial.println(m4);
//Serial.println("Servo 5 first position: ");
//Serial.println(m5);
//Serial.println("Servo 6 first position: ");
//Serial.println(m6);
//Serial.println("Servo 7 first position: ");
//Serial.println(m7);
//Serial.println("Servo 8 first position: ");
//Serial.println(m8);
//Serial.println("Servo 9 first position: ");
//Serial.println(m9);
//Serial.println("Servo 10 first position: ");
//Serial.println(m10);
//Serial.println("Servo 11 first position: ");
//Serial.println(m11);
//Serial.println("Servo 12 first position: ");
//Serial.println(m12);
//Serial.println("Servo 13 first position: ");
//Serial.println(m13);
//Serial.println("Servo 14 first position: ");
//Serial.println(m14);
//Serial.println("Servo 15 first position: ");
//Serial.println(m15);
//Serial.println("Servo 16 first position: ");
//Serial.println(m16);



w = w + 1;
x = x + 1;
y = y + 1;
z = z + 1;


dxlSetGoalSpeed(1,65);
dxlSetGoalSpeed(2,65);
dxlSetGoalSpeed(3,65);
dxlSetGoalSpeed(4,65);
dxlSetGoalSpeed(5,65);
dxlSetGoalSpeed(6,65);
dxlSetGoalSpeed(7,65);
dxlSetGoalSpeed(8,65);
dxlSetGoalSpeed(9,65);
dxlSetGoalSpeed(10,65);
dxlSetGoalSpeed(11,65);
dxlSetGoalSpeed(12,65);
dxlSetGoalSpeed(13,65);
dxlSetGoalSpeed(14,65);
dxlSetGoalSpeed(15,65);
dxlSetGoalSpeed(16,65);
dxlSetGoalSpeed(17,65);
dxlSetGoalSpeed(18,65);
dxlSetGoalSpeed(19,65);
dxlSetGoalSpeed(20,65);
dxlSetGoalSpeed(21,65);
dxlSetGoalSpeed(22,65);
dxlSetGoalSpeed(23,65);
dxlSetGoalSpeed(24,65);


dxlSetGoalPosition(1, m1); 
delay(10);
dxlSetGoalPosition(2, m2);
delay(10);
dxlSetGoalPosition(3, m3);
delay(10);
dxlSetGoalPosition(4, m4); 
delay(10);
dxlSetGoalPosition(5, m5); 
delay(10);
dxlSetGoalPosition(6, m6);
delay(10);
dxlSetGoalPosition(7, m7); 
delay(10);
dxlSetGoalPosition(8, m8);
delay(10);
dxlSetGoalPosition(9,  m9); 
delay(10);
dxlSetGoalPosition(10, m10); 
delay(10);
dxlSetGoalPosition(11, m11);
delay(10);
dxlSetGoalPosition(12, m12); 
delay(10);
dxlSetGoalPosition(13, m13); 
delay(10);
dxlSetGoalPosition(14, m14);
delay(10);
dxlSetGoalPosition(15, m15); 
delay(10);
dxlSetGoalPosition(16, m16);
delay(10);
dxlSetGoalPosition(17, 2048); 
delay(10);
dxlSetGoalPosition(18, 2048); 
delay(10);
dxlSetGoalPosition(19,2048);
delay(10);
dxlSetGoalPosition(20,2048); 
delay(10);
dxlSetGoalPosition(21,2048); 
delay(10);
dxlSetGoalPosition(22,2048);
delay(10);
dxlSetGoalPosition(23,2048); 
delay(10);
dxlSetGoalPosition(24,2048);

//Initialize the sd to be able to interface, read/write
//Serial.print("\nInitializing SD card...");

//while(1) {
//  if (!SD.begin(4)) {
//    Serial.println(F("No SD Card Present. Initialization Failed."));
//    continue;
//} else if (SD.begin(4)) {
//  delay(1000);
//  Serial.println(F("SD Card Detected. Initialization Success."));
//  break;
//  }
//}

while (1) {
        if (breakCondition == 0){
                Serial.println(F("Have not received speeds."));
                Serial.println(F("Continuing wait loop."));
        }
        else if (breakCondition == 1){
                Serial.println(F("Speeds received."));
                Serial.println(F("Moving to Standby Mode."));
                break;
        }
}

//File myFile;

while (1){
    int startState =   digitalRead(startPin);
    if (startState != 0){
      stopVal = 0;
//      myFile = SD.open("test.txt", FILE_WRITE);

      // if the file opened okay, write to it:
//      if (myFile) {
//        Serial.print("Writing to test.txt...");
        //Set the speed to get to the first position
        break; 
      }
}

while (1){

        if (phaseA == 0){
        s1 = FL_ST_R_1[w];
        s2 = FL_ST_R_2[w];
        s3 = FL_ST_R_3[w];
        s4 = FL_ST_R_4[w];
        }
        else if (phaseA == 1){
        s1 = FL_SW_R_1[w]; 
        s2 = FL_SW_R_2[w];
        s3 = FL_SW_R_3[w];
        s4 = FL_SW_R_4[w];
        }

        if (phaseB == 0){
        s5 = FL_ST_L_1[x];
        s6 = FL_ST_L_2[x];
        s7 = FL_ST_L_3[x];
        s8 = FL_ST_L_4[x];
        }
        else if (phaseB == 1){
        s5 = FL_SW_L_1[x];
        s6 = FL_SW_L_2[x];
        s7 = FL_SW_L_3[x];
        s8 = FL_SW_L_4[x];  
        }

        if (phaseC == 0){
        s9 = HL_ST_R_1[y];
        s10 = HL_ST_R_2[y];
        s11 = HL_ST_R_3[y];
        s12 = HL_ST_R_4[y];
        }
        else if (phaseC == 1){
        s9 = HL_SW_R_1[y];
        s10 = HL_SW_R_2[y];
        s11 = HL_SW_R_3[y];
        s12 = HL_SW_R_4[y]; 
        }

        if (phaseD == 0){
        s13 = HL_ST_L_1[z];
        s14 = HL_ST_L_2[z];
        s15 = HL_ST_L_3[z];
        s16 = HL_ST_L_4[z];
        }
        else if (phaseD == 1){
        s13 = HL_SW_L_1[z];
        s14 = HL_SW_L_2[z];
        s15 = HL_SW_L_3[z];
        s16 = HL_SW_L_4[z];  
        }

//torq1 = dxlGetTorque(1); volt1 = dxlGetVoltage(1); curr1 = mxGetCurrent(1); 
//torq2 = dxlGetTorque(2); volt2 = dxlGetVoltage(2); curr2 = mxGetCurrent(2); 
//torq3 = dxlGetTorque(3); volt3 = dxlGetVoltage(3); curr3 = mxGetCurrent(3); 
//torq4 = dxlGetTorque(4); volt4 = dxlGetVoltage(4); curr4 = mxGetCurrent(4); 
//torq5 = dxlGetTorque(5); volt5 = dxlGetVoltage(5); curr5 = mxGetCurrent(5); 
//torq6 = dxlGetTorque(6); volt6 = dxlGetVoltage(6); curr6 = mxGetCurrent(6); 
//torq7 = dxlGetTorque(7); volt7 = dxlGetVoltage(7); curr7 = mxGetCurrent(7); 
//torq8 = dxlGetTorque(8); volt8 = dxlGetVoltage(8); curr8 = mxGetCurrent(8); 
//torq9 = dxlGetTorque(9); volt9 = dxlGetVoltage(9); curr9 = mxGetCurrent(9); 
//torq10 = dxlGetTorque(10); volt10 = dxlGetVoltage(10); curr10 = mxGetCurrent(10); 
//torq11 = dxlGetTorque(11); volt11 = dxlGetVoltage(11); curr11 = mxGetCurrent(11); 
//torq12 = dxlGetTorque(12); volt12 = dxlGetVoltage(12); curr12 = mxGetCurrent(12); 
//torq13 = dxlGetTorque(13); volt13 = dxlGetVoltage(13); curr13 = mxGetCurrent(13); 
//torq14 = dxlGetTorque(14); volt14 = dxlGetVoltage(14); curr14 = mxGetCurrent(14); 
//torq15 = dxlGetTorque(15); volt15 = dxlGetVoltage(15); curr15 = mxGetCurrent(15); 
//torq16 = dxlGetTorque(16); volt16 = dxlGetVoltage(16); curr16 = mxGetCurrent(16); 
//


//myFile.println("SERVO_1: ");  
//String torqValue1 = "Torque: "; torqValue1 += torq1; myFile.println(torqValue1);
//String voltValue1 = "Voltage: "; voltValue1 += volt1; myFile.println(voltValue1);
//String currValue1 = "Current: "; currValue1 += curr1; myFile.println(currValue1);
//String nx = "Move#: "; nx += x; myFile.println(nx);
//myFile.println("\n");
//            
//myFile.println("SERVO_2: ");
//String torqValue2 = "Torque: "; torqValue2 += torq2; myFile.println(torqValue2);
//String voltValue2 = "Voltage: "; voltValue2 += volt2; myFile.println(voltValue2);
//String currValue2 = "Current: "; currValue2 += curr2; myFile.println(currValue2);
//myFile.println(nx);
//myFile.println("\n");
//
//myFile.println("SERVO_3: ");
//String torqValue3 = "Torque: "; torqValue3 += torq3; myFile.println(torqValue3);
//String voltValue3 = "Voltage: "; voltValue3 += volt3; myFile.println(voltValue3);
//String currValue3 = "Current: "; currValue3 += curr3; myFile.println(currValue3);
//myFile.println(nx);
//myFile.println("\n");
//
//myFile.println("SERVO_4: ");
//String torqValue4 = "Torque: "; torqValue4 += torq4; myFile.println(torqValue4);
//String voltValue4 = "Voltage: "; voltValue4 += volt4; myFile.println(voltValue4);
//String currValue4 = "Current: "; currValue4 += curr4; myFile.println(currValue4);
//myFile.println(nx);
//myFile.println("\n");
//
//myFile.println("SERVO_5: ");
//String torqValue5 = "Torque: "; torqValue5 += torq5; myFile.println(torqValue5);
//String voltValue5 = "Voltage: "; voltValue5 += volt5; myFile.println(voltValue5);
//String currValue5 = "Current: "; currValue5 += curr5; myFile.println(currValue5);
//myFile.println(nx); 
//myFile.println("\n");
//
//myFile.println("SERVO_6: ");
//String torqValue6 = "Torque: "; torqValue6 += torq6; myFile.println(torqValue6);
//String voltValue6 = "Voltage: "; voltValue6 += volt6; myFile.println(voltValue6);
//String currValue6 = "Current: "; currValue6 += curr6; myFile.println(currValue6);
//myFile.println(nx); 
//myFile.println("\n");
//
//myFile.println("SERVO_7: ");
//String torqValue7 = "Torque: "; torqValue7 += torq7; myFile.println(torqValue7);
//String voltValue7 = "Voltage: "; voltValue7 += volt7; myFile.println(voltValue7);
//String currValue7 = "Current: "; currValue7 += curr7; myFile.println(currValue7);
//myFile.println(nx); 
//myFile.println("\n");
//
//myFile.println("SERVO_8: ");
//String torqValue8 = "Torque: "; torqValue8 += torq8; myFile.println(torqValue8);
//String voltValue8 = "Voltage: "; voltValue8 += volt8; myFile.println(voltValue8);
//String currValue8 = "Current: "; currValue8 += curr8; myFile.println(currValue8); 
//myFile.println(nx); 
//myFile.println("\n");
//
//myFile.println("SERVO_9: ");
//String torqValue9 ="Torque: "; torqValue9 += torq9; myFile.println(torqValue9);
//String voltValue9 = "Voltage: "; voltValue9 += volt9; myFile.println(voltValue9);
//String currValue9 = "Current: "; currValue9 += curr9; myFile.println(currValue9);
//myFile.println(nx);
//myFile.println("\n");
//
//myFile.println("SERVO_10: ");
//String torqValue10 ="Torque: "; torqValue10 += torq10; myFile.println(torqValue10);
//String voltValue10 = "Voltage: "; voltValue10 += volt10; myFile.println(voltValue10);
//String currValue10 = "Current: "; currValue10 += curr10; myFile.println(currValue10);
//myFile.println(nx);
//myFile.println("\n");
//
//myFile.println("SERVO_11: ");
//String torqValue11 ="Torque: "; torqValue11 += torq11; myFile.println(torqValue11);
//String voltValue11 = "Voltage: "; voltValue11 += volt11; myFile.println(voltValue11);
//String currValue11 = "Current: "; currValue11 += curr11; myFile.println(currValue11);
//myFile.println(nx);
//myFile.println("\n");
//
//myFile.println("SERVO_12: ");
//String torqValue12 ="Torque: "; torqValue12 += torq12; myFile.println(torqValue12);
//String voltValue12 = "Voltage: "; voltValue12 += volt12; myFile.println(voltValue12);
//String currValue12 = "Current: "; currValue12 += curr12; myFile.println(currValue12);
//myFile.println(nx);
//myFile.println("\n");
//
//myFile.println("SERVO_13: ");
//String torqValue13 ="Torque: "; torqValue13 += torq13; myFile.println(torqValue13);
//String voltValue13 = "Voltage: "; voltValue13 += volt13; myFile.println(voltValue13);
//String currValue13 = "Current: "; currValue13 += curr13; myFile.println(currValue13);
//myFile.println(nx); 
//myFile.println("\n");
//
//myFile.println("SERVO_14: ");
//String torqValue14 ="Torque: "; torqValue14 += torq14; myFile.println(torqValue14);
//String voltValue14 = "Voltage: "; voltValue14 += volt14; myFile.println(voltValue14);
//String currValue14 = "Current: "; currValue14 += curr14; myFile.println(currValue14);
//myFile.println(nx);  
//myFile.println("\n");
//
//myFile.println("SERVO_15: ");
//String torqValue15 ="Torque: "; torqValue15 += torq15; myFile.println(torqValue15);
//String voltValue15 = "Voltage: "; voltValue15 += volt15; myFile.println(voltValue15);
//String currValue15 = "Current: "; currValue15 += curr15; myFile.println(currValue15);
//myFile.println(nx); 
//myFile.println("\n");
//
//myFile.println("SERVO_16: ");
//String torqValue16 ="Torque: "; torqValue16 += torq16; myFile.println(torqValue16);
//String voltValue16 = "Voltage: "; voltValue16 += volt16; myFile.println(voltValue16);
//String currValue16 = "Current: "; currValue16 += curr16; myFile.println(currValue16);
//myFile.println(nx); 
//myFile.println("\n");

//dxlSetGoalSpeed(1,speeds[w][c1]);
//dxlSetGoalSpeed(2,speeds[w][c2]);
//dxlSetGoalSpeed(3,speeds[w][c3]);
//dxlSetGoalSpeed(4,speeds[w][c4]);
//dxlSetGoalSpeed(5,speeds[x][c5]);
//dxlSetGoalSpeed(6,speeds[x][c6]);
//dxlSetGoalSpeed(7,speeds[x][c7]);
//dxlSetGoalSpeed(8,speeds[x][c8]);
//dxlSetGoalSpeed(9,speeds[y][c9]);
//dxlSetGoalSpeed(10,speeds[y][c10]);
//dxlSetGoalSpeed(11,speeds[y][c11]);
//dxlSetGoalSpeed(12,speeds[y][c12]);
//dxlSetGoalSpeed(13,speeds[z][c13]);
//dxlSetGoalSpeed(14,speeds[z][c14]);
//dxlSetGoalSpeed(15,speeds[z][c15]);
//dxlSetGoalSpeed(16,speeds[z][c16]);

dxlSetGoalPosition(1,s1); 
////delay(50);
dxlSetGoalPosition(2,s2);
////delay(50);
dxlSetGoalPosition(3,s3); 
////delay(50);
dxlSetGoalPosition(4,s4);
//delay(50);
dxlSetGoalPosition(5,s5); 
//delay(50);
dxlSetGoalPosition(6,s6);
//delay(50);
dxlSetGoalPosition(7,s7); 
//delay(50);
dxlSetGoalPosition(8,s8);
//delay(50);
dxlSetGoalPosition(9,s9); 
////delay(50);
dxlSetGoalPosition(10,s10);
////delay(50);
dxlSetGoalPosition(11,s11); 
////delay(50);
dxlSetGoalPosition(12,s12);
////delay(50);
dxlSetGoalPosition(13,s13); 
////delay(50);
dxlSetGoalPosition(14,s14);
////delay(50);
dxlSetGoalPosition(15,s15); 
////delay(50);
dxlSetGoalPosition(16,s16);
////delay(50);

delay(500);

Serial.println("Move!");
Serial.println(counterTest);
Serial.println(s5);
Serial.println(s6);
Serial.println(s7);
Serial.println(s8);
Serial.println("\n");
//
//Serial.println(s9);
//Serial.println(s10);
//Serial.println(s11);
//Serial.println(s12);
//Serial.println("\n");


w = w + 1;
x = x + 1;
y = y + 1;
z = z + 1;

if (w > 10){
  w = 0;
}
if (x > 10){
  x = 0;
}
if (y > 10){
  y = 0;
}
if (z > 10){
  z = 0;
}

Acheck = percPosVals[w];
Bcheck = percPosVals[x];
Ccheck = percPosVals[y];
Dcheck = percPosVals[z];

        if (Acheck == 0){
            if (phaseA == 0){
                phaseA = 1;
                Serial.println("Changing A to Swing");

            }
            else if (phaseA == 1){
                phaseA = 0;
                Serial.println("Changing A to Stance");
            }
        }
        else if (Acheck == 10){
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
                Serial.println("Changing B to Swing");
            }
            else if (phaseB == 1){
                phaseB = 0;
                Serial.println("Changing B to Stance");
            }
        }
        else if (Bcheck == 10){
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
                Serial.println("Changing C to Swing");
            }
            else if (phaseC == 1){
                phaseC = 0;
                Serial.println("Changing C to Stance");

            }
        }
        else if (Ccheck == 10){
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
                Serial.println("Changing D to Swing");
            }
            else if (phaseD == 1){
                phaseD = 0;
                Serial.println("Changing D to Stance");

                }
        }
        else if (Dcheck == 10){
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

counterTest = counterTest + 1;
int pinstate = digitalRead(stopPin);
Serial.println(F("End of Movement Loop"));
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
//          myFile.close();
          Serial.println(F("done."));
          break;
        }
}
} // While StopVal == 0
} // void Setup()

void loop(){}
