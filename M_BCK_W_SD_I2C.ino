#include <SPI.h>                //include the pinout library
#include <SD.h>                 //include the sd interface library
#include <stdio.h>
#include <ax12.h>               //include base library for DYNAMIXELs
#include <BioloidController.h>  //include bioloid libary for poses/movements
#include "poses.h"              //home and center poses for the robot
#include <Wire.h>
#include <Arduino.h>
#include <string.h>
//below, set the baud rate programmed on the servos
BioloidController bioloid = BioloidController(1000000); //start the BIOLOID library at 1mbps. 
//This will run the dxlInit() function internally, so we don't need to call it
//Select thepinout for the relative board, for the arbotix board, 
//utilize the pinout of the atmega chip to select correct data pins and define them
#define PIN_SPI_SS    (5)
#define PIN_SPI_MOSI  (6)
#define PIN_SPI_MISO  (7)
#define PIN_SPI_SCK   (8)
#define SLAVE_ADDRESS 0x14
const int SERVOCOUNT = 8;  //number of servos in this robot
int id;                    //temperary id for movement
int GoSignal = 0;
int talk = 0;
int GreenLight = 0;
//volatile boolean receiveFlag = false;
char temp[32];
char *ptr;
long int srvodist;
int soloSrvo = 1;
int stopVal = 0;
int firstHalfStride = 63; // Default equal values
int secondHalfStride = 63;
int firstAssigned = 0;
int running = 1;
int stopPin = 21;
int contPin = 22;
int pinstate;    

//Function to send data to master
void sendData() {
  if (GreenLight == 1) {
    byte bytdata = (byte)1;
    Wire.write(bytdata);
  }
  else if (GreenLight == 2){
    byte bytdata = (byte)2;
    Wire.write(bytdata);
  }
  else if (GreenLight == 3){
    byte bytdata = (byte)3;
    Wire.write(bytdata);
  }
}

//Call the file placeholder for the sd to later write to
File myFile;

//Function to receive data from master
void receiveData(int HowMany) {
  for (int i = 0; i < HowMany; i++) {
    temp[i] = Wire.read();
    temp[i + 1] = '\0'; //add null after ea. char
  }
  //RPi first byte is cmd byte so shift everything to the left 1 pos so temp contains our string
  for (int i = 0; i < HowMany; ++i) {
    temp[i] = temp[i + 1];
  }
  String valReceived = "Received: ";
  valReceived += temp;
  Serial.println(valReceived);
  srvodist = strtol(temp, &ptr, 10);
  Serial.println(srvodist);
  
  if (srvodist > 0) {
    if (srvodist >= 1  && srvodist <= 16){
      soloSrvo = srvodist;
    }
    else if (srvodist >= 17 && srvodist < 198){
      srvodist = srvodist - 17;
      dxlSetGoalPosition(soloSrvo, srvodist);
    }
    else if (srvodist >= 198 && firstAssigned == 0){
      srvodist = srvodist - 197;
      firstHalfStride = srvodist;
      firstAssigned = 1;
    }
    else if (srvodist >= 198 && firstAssigned == 1){
      srvodist = srvodist - 197;
      secondHalfStride = srvodist;
      firstAssigned = 0;
    }
  }
  else if (srvodist <= 0){
    if (0 == strcmp(temp,"back_com_check")) {
    Serial.println("Communication check achieved.");
      GreenLight = 1;
      sendData();
      Serial.println("Writing 'Communication is Present' Back to Pi");
    //  receiveFlag = false;
    //  break;
    }
    else if (0 == strcmp(temp,"back_sd_present")) {
      GoSignal = 1;
      GreenLight = 2;
      sendData();
      Serial.println("Received Go Signal. Continuing");
  //  receiveFlag = false;
    //  break
    }
  else if (0 == strcmp(temp,"back_board_start")) {
      stopVal = 0;
      myFile = SD.open("test.txt", FILE_WRITE);

      // if the file opened okay, write to it:
      if (myFile) { 
        Serial.print("Writing to test.txt..."); 
    //Set the speed constant for the servos
        dxlSetGoalSpeed(9, 46); dxlSetGoalSpeed(10, 46); dxlSetGoalSpeed(11, 46);
        dxlSetGoalSpeed(12, 46); dxlSetGoalSpeed(13, 46); dxlSetGoalSpeed(14, 46);
        dxlSetGoalSpeed(15, 46); dxlSetGoalSpeed(16, 46);                  
    while (stopVal == 0){
    //Loop for first movement
    for (int x = 0; x < firstHalfStride; x = x + 1)  {
    //Define all the variables utilzed in loop
    int torq9; int sped9; int volt9; int curr9; int temp9; int posi9;
    int torq10; int sped10; int volt10; int curr10; int temp10; int posi10;
    int torq11; int sped11; int volt11; int curr11; int temp11; int posi11;
    int torq12; int sped12; int volt12; int curr12; int temp12; int posi12;
    int torq13; int sped13; int volt13; int curr13; int temp13; int posi13;
    int torq14; int sped14; int volt14; int curr14; int temp14; int posi14;
    int torq15; int sped15; int volt15; int curr15; int temp15; int posi15;
    int torq16; int sped16; int volt16; int curr16; int temp16; int posi16;

    //Define polynomials utilized for motion
    int m12 = (((-6.885579823) * (0.01) * (x * x)) + (4.268922536 * x) + (1859.013452));
    int m11 = (((1.524691917) * (0.01) * (x * x)) - (6.568722054 * x) + (2143.11934));
    int m10 = (((-1.284510229) * (0.1) * (x * x)) + (5.260906283 * x) + (2214.538775));
    int m9 = (((-2.599952146) * (0.01) * (x * x)) + (11.31370255 * x) + (1614.662444));

    //Define request commands for each respective servo
    torq9 = dxlGetTorque(9); sped9 = dxlGetGoalSpeed(9); volt9 = dxlGetVoltage(9);
    temp9 = dxlGetTemperature(9); curr9 = mxGetCurrent(9); posi9 = dxlGetPosition(9);

    torq10 = dxlGetTorque(10); sped10 = dxlGetGoalSpeed(10); volt10 = dxlGetVoltage(10);
    temp10 = dxlGetTemperature(10); curr10 = mxGetCurrent(10); posi10 = dxlGetPosition(10);

    torq11 = dxlGetTorque(11); sped11 = dxlGetGoalSpeed(11); volt11 = dxlGetVoltage(11);
    temp11 = dxlGetTemperature(11); curr11 = mxGetCurrent(11); posi11 = dxlGetPosition(11);

    torq12 = dxlGetTorque(12); sped12 = dxlGetGoalSpeed(12); volt12 = dxlGetVoltage(12);
    temp12 = dxlGetTemperature(12); curr12 = mxGetCurrent(12); posi12 = dxlGetPosition(12);

    torq13 = dxlGetTorque(13); sped13 = dxlGetGoalSpeed(13); volt13 = dxlGetVoltage(13);
    temp13 = dxlGetTemperature(13); curr13 = mxGetCurrent(13); posi13 = dxlGetPosition(13);

    torq14 = dxlGetTorque(14); sped14 = dxlGetGoalSpeed(14); volt14 = dxlGetVoltage(14);
    temp14 = dxlGetTemperature(14); curr14 = mxGetCurrent(14); posi14 = dxlGetPosition(14);

    torq15 = dxlGetTorque(15); sped15 = dxlGetGoalSpeed(15); volt15 = dxlGetVoltage(15);
    temp15 = dxlGetTemperature(15); curr15 = mxGetCurrent(15); posi15 = dxlGetPosition(15);

    torq16 = dxlGetTorque(16); sped16 = dxlGetGoalSpeed(16); volt16 = dxlGetVoltage(16);
    temp16 = dxlGetTemperature(16); curr16 = mxGetCurrent(16); posi16 = dxlGetPosition(16);

    //Actually move servos relative to value in polynomial
    dxlSetGoalPosition(9,  m9); dxlSetGoalPosition(10, m10); dxlSetGoalPosition(11, m11);
    dxlSetGoalPosition(12, m12); dxlSetGoalPosition(13, m9); dxlSetGoalPosition(14, m10);
    dxlSetGoalPosition(15, m11); dxlSetGoalPosition(16, m12);

    //WRITING TO SD:

    myFile.println("SERVO_9: ");
    String torqValue9 ="Torque: ";
    torqValue9 += torq9;
    myFile.println(torqValue9);
    String spedValue9 = "Speed: ";
    spedValue9 += sped9;
    myFile.println(spedValue9);
    String voltValue9 = "Voltage:";
    voltValue9 += volt9;
    myFile.println(voltValue9);
    String tempValue9 = "Temperature: ";
    tempValue9 += temp9;
    myFile.println(tempValue9);
    String currValue9 = "Current: ";
    currValue9 += curr9;
    myFile.println(currValue9);
    String posValue9 = "Position: ";
    posValue9 += posi9;
    myFile.println(posValue9);
    String nx = "Move#: ";
    nx += x;
    myFile.println(nx);
    String polynum9 = "Polynum: ";
    polynum9 += m9;
    myFile.println(polynum9);
    myFile.println("\n");

    myFile.println("SERVO_10: ");
    String torqValue10 ="Torque: ";
    torqValue10 += torq10;
    myFile.println(torqValue10);
    String spedValue10 = "Speed: ";
    spedValue10 += sped10;
    myFile.println(spedValue10);
    String voltValue10 = "Voltage:";
    voltValue10 += volt10;
    myFile.println(voltValue10);
    String tempValue10 = "Temperature: ";
    tempValue10 += temp10;
    myFile.println(tempValue10);
    String currValue10 = "Current: ";
    currValue10 += curr10;
    myFile.println(currValue10);
    String posValue10 = "Position: ";
    posValue10 += posi10;
    myFile.println(posValue10);
    myFile.println(nx);
    String polynum10 = "Polynum: ";
    polynum10 += m10;
    myFile.println(polynum10);               
    myFile.println("\n");

    myFile.println("SERVO_11: ");
    String torqValue11 ="Torque: ";
    torqValue11 += torq11;
    myFile.println(torqValue11);
    String spedValue11 = "Speed: ";
    spedValue11 += sped11;
    myFile.println(spedValue11);
    String voltValue11 = "Voltage:";
    voltValue11 += volt11;
    myFile.println(voltValue11);
    String tempValue11 = "Temperature: ";
    tempValue11 += temp11;
    myFile.println(tempValue11);
    String currValue11 = "Current: ";
    currValue11 += curr11;
    myFile.println(currValue11);
    String posValue11 = "Position: ";
    posValue11 += posi11;
    myFile.println(posValue11);
    myFile.println(nx);
    String polynum11 = "Polynum: ";
    polynum11 += m11;
    myFile.println(polynum11);               
    myFile.println("\n");

    myFile.println("SERVO_12: ");
    String torqValue12 ="Torque: ";
    torqValue12 += torq12;
    myFile.println(torqValue12);
    String spedValue12 = "Speed: ";
    spedValue12 += sped12;
    myFile.println(spedValue12);
    String voltValue12 = "Voltage:";
    voltValue12 += volt12;
    myFile.println(voltValue12);
    String tempValue12 = "Temperature: ";
    tempValue12 += temp12;
    myFile.println(tempValue12);
    String currValue12 = "Current: ";
    currValue12 += curr12;
    myFile.println(currValue12);
    String posValue12 = "Position: ";
    posValue12 += posi12;
    myFile.println(posValue12);
    myFile.println(nx);
    String polynum12 = "Polynum: ";
    polynum12 += m12;
    myFile.println(polynum12);
    myFile.println("\n");

    myFile.println("SERVO_13: ");
    String torqValue13 ="Torque: ";
    torqValue13 += torq13;
    myFile.println(torqValue13);
    String spedValue13 = "Speed: ";
    spedValue13 += sped13;
    myFile.println(spedValue13);
    String voltValue13 = "Voltage:";
    voltValue13 += volt13;
    myFile.println(voltValue13);
    String tempValue13 = "Temperature: ";
    tempValue13 += temp13;
    myFile.println(tempValue13);
    String currValue13 = "Current: ";
    currValue13 += curr13;
    myFile.println(currValue13);
    String posValue13 = "Position: ";
    posValue13 += posi13;
    myFile.println(posValue13);
    myFile.println(nx);
    myFile.println(polynum9);  
    myFile.println("\n");

    myFile.println("SERVO_14: ");
    String torqValue14 ="Torque: ";
    torqValue14 += torq14;
    myFile.println(torqValue14);
    String spedValue14 = "Speed: ";
    spedValue14 += sped14;
    myFile.println(spedValue14);
    String voltValue14 = "Voltage:";
    voltValue14 += volt14;
    myFile.println(voltValue14);
    String tempValue14 = "Temperature: ";
    tempValue14 += temp14;
    myFile.println(tempValue14);
    String currValue14 = "Current: ";
    currValue14 += curr14;
    myFile.println(currValue14);
    String posValue14 = "Position: ";
    posValue14 += posi14;
    myFile.println(posValue14);
    myFile.println(nx);
    myFile.println(polynum10);               
    myFile.println("\n");

    myFile.println("SERVO_15: ");
    String torqValue15 ="Torque: ";
    torqValue15 += torq15;
    myFile.println(torqValue15);
    String spedValue15 = "Speed: ";
    spedValue15 += sped15;
    myFile.println(spedValue15);
    String voltValue15 = "Voltage:";
    voltValue15 += volt15;
    myFile.println(voltValue15);
    String tempValue15 = "Temperature: ";
    tempValue15 += temp15;
    myFile.println(tempValue15);
    String currValue15 = "Current: ";
    currValue15 += curr15;
    myFile.println(currValue15);
    String posValue15 = "Position: ";
    posValue15 += posi15;
    myFile.println(posValue15);
    myFile.println(nx);
    myFile.println(polynum11);
    myFile.println("\n");

    myFile.println("SERVO_16: ");
    String torqValue16 ="Torque: ";
    torqValue16 += torq16;
    myFile.println(torqValue16);
    String spedValue16 = "Speed: ";
    spedValue16 += sped16;
    myFile.println(spedValue16);
    String voltValue16 = "Voltage:";
    voltValue16 += volt16;
    myFile.println(voltValue16);
    String tempValue16 = "Temperature: ";
    tempValue16 += temp16;
    myFile.println(tempValue16);
    String currValue16 = "Current: ";
    currValue16 += curr16;
    myFile.println(currValue16);
    String posValue16 = "Position: ";
    posValue16 += posi16;
    myFile.println(posValue16);
    myFile.println(nx);
    myFile.println(polynum12);               
    myFile.println("\n");

  int pinstate = digitalRead(stopPin);
  Serial.println("runningFront");
    delay(100);
  if (pinstate != 0){            
    while (1){
            int contPinState = digitalRead(contPin);
            if (contPinState == 1){
              break;
            }
            else if (contPinState == 0){
              continue;
            }
          }
        } 
    }


  //Return loop
    for (int x = secondHalfStride; x > 0; x = x - 1)  {

    int torq9; int sped9; int volt9; int curr9; int temp9; int posi9;
    int torq10; int sped10; int volt10; int curr10; int temp10; int posi10;
    int torq11; int sped11; int volt11; int curr11; int temp11; int posi11;
    int torq12; int sped12; int volt12; int curr12; int temp12; int posi12;
    int torq13; int sped13; int volt13; int curr13; int temp13; int posi13;
    int torq14; int sped14; int volt14; int curr14; int temp14; int posi14;
    int torq15; int sped15; int volt15; int curr15; int temp15; int posi15;
    int torq16; int sped16; int volt16; int curr16; int temp16; int posi16;

    torq9 = dxlGetTorque(9); sped9 = dxlGetGoalSpeed(9); volt9 = dxlGetVoltage(9);
    temp9 = dxlGetTemperature(9); curr9 = mxGetCurrent(9); posi9 = dxlGetPosition(9);
                                       
    torq10 = dxlGetTorque(10); sped10 = dxlGetGoalSpeed(10); volt10 = dxlGetVoltage(10);
    temp10 = dxlGetTemperature(10); curr10 = mxGetCurrent(10); posi10 = dxlGetPosition(10);
                                       
    torq11 = dxlGetTorque(11); sped11 = dxlGetGoalSpeed(11); volt11 = dxlGetVoltage(11);
    temp11 = dxlGetTemperature(11); curr11 = mxGetCurrent(11); posi11 = dxlGetPosition(11);
    
    torq12 = dxlGetTorque(12); sped12 = dxlGetGoalSpeed(12); volt12 = dxlGetVoltage(12);
    temp12 = dxlGetTemperature(12); curr12 = mxGetCurrent(12); posi12 = dxlGetPosition(12);

    torq13 = dxlGetTorque(13); sped13 = dxlGetGoalSpeed(13); volt13 = dxlGetVoltage(13);
    temp13 = dxlGetTemperature(13); curr13 = mxGetCurrent(13); posi13 = dxlGetPosition(13);
                                                               
    torq14 = dxlGetTorque(14); sped14 = dxlGetGoalSpeed(14); volt14 = dxlGetVoltage(14);
    temp14 = dxlGetTemperature(14); curr14 = mxGetCurrent(14); posi14 = dxlGetPosition(14);
                      
    torq15 = dxlGetTorque(15); sped15 = dxlGetGoalSpeed(15); volt15 = dxlGetVoltage(15);
    temp15 = dxlGetTemperature(15); curr15 = mxGetCurrent(15); posi15 = dxlGetPosition(15);

    torq16 = dxlGetTorque(16); sped16 = dxlGetGoalSpeed(16); volt16 = dxlGetVoltage(16); 
    temp16 = dxlGetTemperature(16); curr16 = mxGetCurrent(16); posi16 = dxlGetPosition(16);

    int m12 = (((-6.885579823) * (0.01) * (x * x)) + (4.268922536 * x) + (1859.013452));
    int m11 = (((1.524691917) * (0.01) * (x * x)) - (6.568722054 * x) + (2143.11934));
    int m10 = (((-1.284510229) * (0.1) * (x * x)) + (5.260906283 * x) + (2214.538775));
    int m9 = (((-2.599952146) * (0.01) * (x * x)) + (11.31370255 * x) + (1614.662444));
    
    dxlSetGoalPosition(9, m9); dxlSetGoalPosition(10, m10); dxlSetGoalPosition(11, m11);
    dxlSetGoalPosition(12, m12); dxlSetGoalPosition(13, m9); dxlSetGoalPosition(14, m10);
    dxlSetGoalPosition(15, m11); dxlSetGoalPosition(16, m12);                

    myFile.println("SERVO_9_Return: ");
    String torqValue9 ="Torque: ";
    torqValue9 += torq9;
    myFile.println(torqValue9);
    String spedValue9 = "Speed: ";
    spedValue9 += sped9;
    myFile.println(spedValue9);
    String voltValue9 = "Voltage:";
    voltValue9 += volt9;
    myFile.println(voltValue9);
    String tempValue9 = "Temperature: ";
    tempValue9 += temp9;
    myFile.println(tempValue9);
    String currValue9 = "Current: ";
    currValue9 += curr9;
    myFile.println(currValue9);
    String posValue9 = "Position: ";
    posValue9 += posi9;
    myFile.println(posValue9);
    String nx = "Move#: ";
    nx += x;
    myFile.println(nx);
    String polynum9 = "Polynum: ";
    polynum9 += m9;
    myFile.println(polynum9);              
    myFile.println("\n");

    myFile.println("SERVO_10_Return: ");
    String torqValue10 ="Torque: ";
    torqValue10 += torq10;
    myFile.println(torqValue10);
    String spedValue10 = "Speed: ";
    spedValue10 += sped10;
    myFile.println(spedValue10);
    String voltValue10 = "Voltage:";
    voltValue10 += volt10;
    myFile.println(voltValue10);
    String tempValue10 = "Temperature: ";
    tempValue10 += temp10;
    myFile.println(tempValue10);
    String currValue10 = "Current: ";
    currValue10 += curr10;
    myFile.println(currValue10);
    String posValue10 = "Position: ";
    posValue10 += posi10;
    myFile.println(posValue10);               
    myFile.println(nx);
    String polynum10 = "Polynum: ";
    polynum10 += m10;
    myFile.println(polynum10); 
    myFile.println("\n");

    myFile.println("SERVO_11_Return: ");
    String torqValue11 ="Torque: ";
    torqValue11 += torq11;
    myFile.println(torqValue11);
    String spedValue11 = "Speed: ";
    spedValue11 += sped11;
    myFile.println(spedValue11);
    String voltValue11 = "Voltage:";
    voltValue11 += volt11;
    myFile.println(voltValue11);
    String tempValue11 = "Temperature: ";
    tempValue11 += temp11;
    myFile.println(tempValue11);
    String currValue11 = "Current: ";
    currValue11 += curr11;
    myFile.println(currValue11);
    String posValue11 = "Position: ";
    posValue11 += posi11;
    myFile.println(posValue11);
    myFile.println(nx);
    String polynum11 = "Polynum: ";
    polynum11 += m11;
    myFile.println(polynum11);             
    myFile.println("\n");

    myFile.println("SERVO_12_Return: ");
    String torqValue12 ="Torque: ";
    torqValue12 += torq12;
    myFile.println(torqValue12);
    String spedValue12 = "Speed: ";
    spedValue12 += sped12;
    myFile.println(spedValue12);
    String voltValue12 = "Voltage:";
    voltValue12 += volt12;
    myFile.println(voltValue12);
    String tempValue12 = "Temperature: ";
    tempValue12 += temp12;
    myFile.println(tempValue12);
    String currValue12 = "Current: ";
    currValue12 += curr12;
    myFile.println(currValue12);
    String posValue12 = "Position: ";
    posValue12 += posi12;
    myFile.println(posValue12);
    myFile.println(nx);
    String polynum12 = "Polynum: ";
    polynum12 += m12;
    myFile.println(polynum12);
    myFile.println("\n");

    myFile.println("SERVO_13_Return: ");
    String torqValue13 ="Torque: ";
    torqValue13 += torq13;
    myFile.println(torqValue13);
    String spedValue13 = "Speed: ";
    spedValue13 += sped13;
    myFile.println(spedValue13);
    String voltValue13 = "Voltage:";
    voltValue13 += volt13;
    myFile.println(voltValue13);
    String tempValue13 = "Temperature: ";
    tempValue13 += temp13;
    myFile.println(tempValue13);
    String currValue13 = "Current: ";
    currValue13 += curr13;
    myFile.println(currValue13);
    String posValue13 = "Position: ";
    posValue13 += posi13;
    myFile.println(posValue13);
    myFile.println(nx);
    myFile.println(polynum9); 
    myFile.println("\n");

    myFile.println("SERVO_14_Return: ");
    String torqValue14 ="Torque: ";
    torqValue14 += torq14;
    myFile.println(torqValue14);
    String spedValue14 = "Speed: ";
    spedValue14 += sped14;
    myFile.println(spedValue14);
    String voltValue14 = "Voltage:";
    voltValue14 += volt14;
    myFile.println(voltValue14);
    String tempValue14 = "Temperature: ";
    tempValue14 += temp14;
    myFile.println(tempValue14);
    String currValue14 = "Current: ";
    currValue14 += curr14;
    myFile.println(currValue14);
    String posValue14 = "Position: ";
    posValue14 += posi14;
    myFile.println(posValue14);
    myFile.println(nx);
    myFile.println(polynum10);
    myFile.println("\n");

    myFile.println("SERVO_15_Return: ");
    String torqValue15 ="Torque: ";
    torqValue15 += torq15;
    myFile.println(torqValue15);
    String spedValue15 = "Speed: ";
    spedValue15 += sped15;
    myFile.println(spedValue15);
    String voltValue15 = "Voltage:";
    voltValue15 += volt15;
    myFile.println(voltValue15);
    String tempValue15 = "Temperature: ";
    tempValue15 += temp15;
    myFile.println(tempValue15);
    String currValue15 = "Current: ";
    currValue15 += curr15;
    myFile.println(currValue15);
    String posValue15 = "Position: ";
    posValue15 += posi15;
    myFile.println(posValue15);
    myFile.println(nx);
    myFile.println(polynum11);
    myFile.println("\n");

    myFile.println("SERVO_16_Return: ");
    String torqValue16 ="Torque: ";
    torqValue16 += torq16;
    myFile.println(torqValue16);
    String spedValue16 = "Speed: ";
    spedValue16 += sped16;
    myFile.println(spedValue16);
    String voltValue16 = "Voltage:";
    voltValue16 += volt16;
    myFile.println(voltValue16);
    String tempValue16 = "Temperature: ";
    tempValue16 += temp16;
    myFile.println(tempValue16);
    String currValue16 = "Current: ";
    currValue16 += curr16;
    myFile.println(currValue16);
    String posValue16 = "Position: ";
    posValue16 += posi16;
    myFile.println(posValue16);
    myFile.println(nx);
    myFile.println(polynum12); 
    myFile.println("\n");
                 
  int pinstate = digitalRead(stopPin);
  Serial.println("RunningBack");
  
        delay(100);
        if (pinstate != 0){
          while (1){
            int contPinState = digitalRead(contPin);
            if (contPinState == 1){
              break;
            }
            else if (contPinState == 0){
              continue;
            }
          }
          }
        }
      }
      }
    }
    
  else if (0 == strcmp(temp,"back_board_reset")) {
      //Return to home position
      delay(500);
      dxlSetGoalPosition(9, 2048); dxlSetGoalPosition(10, 2048); dxlSetGoalPosition(11, 2048);
      dxlSetGoalPosition(12, 2048); dxlSetGoalPosition(13, 2048); dxlSetGoalPosition(14, 2048);
      dxlSetGoalPosition(15, 2048); dxlSetGoalPosition(16, 2048);
    }
    else if (0 == strcmp(temp,"back_board_end")) {
      running = 0;
      // open the file. Note that only one file can be open at a time,
      // so you have to close this one before opening another.
      myFile.close();
      Serial.println("done.");
    }
    else {
      Serial.println("No options selected.");
    }
  }
   
   
  
  
  Serial.println("End of communication check.");
}
                     


//Main initialization and movement code here:
void setup() {

  // Open serial communications and wait for port to open:
  Serial.begin(9600);

  //for the stop input
  pinMode(stopPin, INPUT);
  
  while (!Serial) {
    continue; // wait for serial port to connect. Needed for native USB port only
  }

//Set to home position(stance)
  dxlSetGoalPosition(9, 2048); dxlSetGoalPosition(10, 2048); dxlSetGoalPosition(11, 2048);
  dxlSetGoalPosition(12, 2048); dxlSetGoalPosition(13, 2048); dxlSetGoalPosition(14, 2048);                                        
  dxlSetGoalPosition(15, 2048); dxlSetGoalPosition(16, 2048);                                                                
                 
  delay(3000);

//Inititate rudementary i2c comms
Wire.begin(SLAVE_ADDRESS);
Wire.onReceive(receiveData);
Wire.onRequest(sendData);

Serial.println("Beginning Start Up Sequence.");
Serial.println("Checking for I2C Communication Channels.");

//Check to confirm proper communcation between Arbotix(Slave) and RPi(Master)  
while(1) {
  if (GreenLight == 1){
    Serial.println("Channel Present. Continuing to SD Check");
    break;
  }
  else {
  Serial.println("Channel NOT Present, looping again");
  delay(1000);
  continue;
  }
}

//Initialize the sd to be able to interface, read/write
Serial.print("\nInitializing SD card...");

//Detecting presence of SD card, responding to master
while(1) {
  if (!SD.begin(4)) {
    Serial.println("No SD Card Present. Initialization Failed.");
    continue;
} else if (SD.begin(4)) {
  delay(1000);
  Serial.println("SD Card Detected. Initialization Success.");
  delay(2000);
  Serial.println("Sending Confirmation to Pi Master");
  GreenLight=2;
  sendData();
  Serial.println("Continuing to Wait Stage");
  break;
  }
}

//Signalling if all SD cards are present, writing response from master
while (1) {
  if (GoSignal == 1)  {
    Serial.println("All SD Cards Confirmed. Commencing Standby Mode!");
    break;
    } else  {
      Serial.println("Not all confirmed, Looping");
      delay(1000);
      continue;
      }
}

while (running == 1) {
}
//Re-open the file for reading:
  myFile = SD.open("test.txt");
  if (myFile) {
    Serial.println("test.txt:");

    //Read from the file until there's nothing else in it:
    while(myFile.available()) {
      Serial.write(myFile.read());
    }
    // close the file:
  myFile.close();
  } else  {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
}

void loop() {
  // nothing happens after setup
  // Entire process happens with setup
}
