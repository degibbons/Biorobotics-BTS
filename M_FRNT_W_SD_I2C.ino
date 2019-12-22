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
#define SLAVE_ADDRESS 0x13
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
    if (0 == strcmp(temp,"front_com_check")) {
      Serial.println("Communication check achieved.");
      GreenLight = 1;
      sendData();
      Serial.println("Writing 'Communication is Present' Back to Pi");
    //  receiveFlag = false;
    //  break;
    }
    else if (0 == strcmp(temp,"front_sd_present")) {
      GoSignal = 1;
      GreenLight = 2;
      sendData();
      Serial.println("Received Go Signal. Continuing");
    //  receiveFlag = false;
    //  break
    }
    else if (0 == strcmp(temp,"front_board_start")) {
      stopVal = 0;
      myFile = SD.open("test.txt", FILE_WRITE);

      // if the file opened okay, write to it:
      if (myFile) {
        Serial.print("Writing to test.txt...");
        //Set the speed constant for the servos
        dxlSetGoalSpeed(1, 46); dxlSetGoalSpeed(2, 46); dxlSetGoalSpeed(3, 46);
        dxlSetGoalSpeed(4, 46); dxlSetGoalSpeed(5, 46); dxlSetGoalSpeed(6, 46);
        dxlSetGoalSpeed(7, 46); dxlSetGoalSpeed(8, 46);
        while (stopVal == 0){
        //Loop for first movement
        for (int x = 0; x < firstHalfStride; x = x + 1) {
        //Define all the variables utilzed in loop
        int torq1; int sped1; int volt1; int curr1; int temp1; int posi1;
        int torq2; int sped2; int volt2; int curr2; int temp2; int posi2;
        int torq3; int sped3; int volt3; int curr3; int temp3; int posi3;
        int torq4; int sped4; int volt4; int curr4; int temp4; int posi4;
        int torq5; int sped5; int volt5; int curr5; int temp5; int posi5;
        int torq6; int sped6; int volt6; int curr6; int temp6; int posi6;
        int torq7; int sped7; int volt7; int curr7; int temp7; int posi7;
        int torq8; int sped8; int volt8; int curr8; int temp8; int posi8;

      //Define polynomials utilized for motion
        int m4 = (((-6.885579823) * (0.01) * (x * x)) + (4.268922536 * x) + (1859.013452));
        int m3 = (((1.524691917) * (0.01) * (x * x)) - (6.568722054 * x) + (2143.11934));
        int m2 = (((-1.284510229) * (0.1) * (x * x)) + (5.260906283 * x) + (2214.538775));
        int m1 = (((-2.599952146) * (0.01) * (x * x)) + (11.31370255 * x) + (1614.662444));

      //Define request commands for each respective servo
        torq1 = dxlGetTorque(1); sped1 = dxlGetGoalSpeed(1); volt1 = dxlGetVoltage(1);
        temp1 = dxlGetTemperature(1); curr1 = mxGetCurrent(1); posi1 = dxlGetPosition(1);

        torq2 = dxlGetTorque(2); sped2 = dxlGetGoalSpeed(2); volt2 = dxlGetVoltage(2);
        temp2 = dxlGetTemperature(2); curr2 = mxGetCurrent(2); posi2 = dxlGetPosition(2);

        torq3 = dxlGetTorque(3); sped3 = dxlGetGoalSpeed(3); volt3 = dxlGetVoltage(3);
        temp3 = dxlGetTemperature(3); curr3 = mxGetCurrent(3); posi3 = dxlGetPosition(3);

        torq4 = dxlGetTorque(4); sped4 = dxlGetGoalSpeed(4); volt4 = dxlGetVoltage(4);
        temp4 = dxlGetTemperature(4); curr4 = mxGetCurrent(4); posi4 = dxlGetPosition(4);

        torq5 = dxlGetTorque(5); sped5 = dxlGetGoalSpeed(5); volt5 = dxlGetVoltage(5);
        temp5 = dxlGetTemperature(5); curr5 = mxGetCurrent(5); posi5 = dxlGetPosition(5);

        torq6 = dxlGetTorque(6); sped6 = dxlGetGoalSpeed(6); volt6 = dxlGetVoltage(6);
        temp6 = dxlGetTemperature(6); curr6 = mxGetCurrent(6); posi6 = dxlGetPosition(6);

        torq7 = dxlGetTorque(7); sped7 = dxlGetGoalSpeed(7); volt7 = dxlGetVoltage(7);
        temp7 = dxlGetTemperature(7); curr7 = mxGetCurrent(7); posi7 = dxlGetPosition(7);

        torq8 = dxlGetTorque(8); sped8 = dxlGetGoalSpeed(8); volt8 = dxlGetVoltage(8);
        temp8 = dxlGetTemperature(8); curr8 = mxGetCurrent(8); posi8 = dxlGetPosition(8);

      //Actually move servos relative to value in polynomial
        dxlSetGoalPosition(1, m1); dxlSetGoalPosition(2, m2); dxlSetGoalPosition(3, m3);
        dxlSetGoalPosition(4, m4); dxlSetGoalPosition(5, m1); dxlSetGoalPosition(6, m2);
        dxlSetGoalPosition(7, m3); dxlSetGoalPosition(8, m4);

      //WRITING TO SD:

        myFile.println("SERVO_1: ");
        String torqValue1 = "Torque: ";
        torqValue1 += torq1;
        myFile.println(torqValue1);
        String spedValue1 = "Speed: ";
        spedValue1 += sped1;
        myFile.println(spedValue1);
        String voltValue1 = "Voltage:";
        voltValue1 += volt1;
        myFile.println(voltValue1);
        String tempValue1 = "Temperature: ";
        tempValue1 += temp1;
        myFile.println(tempValue1);
        String currValue1 = "Current: ";
        currValue1 += curr1;
        myFile.println(currValue1);
        String posValue1 = "Position: ";
        posValue1 += posi1;
        myFile.println(posValue1);
        String nx = "Move#: ";
        nx += x;
        myFile.println(nx);
        String polynum1 = "Polynum: ";
        polynum1 += m1;
        myFile.println(polynum1);
        myFile.println("\n");

        myFile.println("SERVO_2: ");
        String torqValue2 = "Torque: ";
        torqValue2 += torq2;
        myFile.println(torqValue2);
        String spedValue2 = "Speed: ";
        spedValue2 += sped2;
        myFile.println(spedValue2);
        String voltValue2 = "Voltage:";
        voltValue2 += volt2;
        myFile.println(voltValue2);
        String tempValue2 = "Temperature: ";
        tempValue2 += temp2;
        myFile.println(tempValue2);
        String currValue2 = "Current: ";
        currValue2 += curr2;
        myFile.println(currValue2);
        String posValue2 = "Position: ";
        posValue2 += posi2;
        myFile.println(posValue2);
        myFile.println(nx);
        String polynum2 = "Polynum: ";
        polynum2 += m2;
        myFile.println(polynum2);
        myFile.println("\n");

        myFile.println("SERVO_3: ");
        String torqValue3 = "Torque: ";
        torqValue3 += torq3;
        myFile.println(torqValue3);
        String spedValue3 = "Speed: ";
        spedValue3 += sped3;
        myFile.println(spedValue3);
        String voltValue3 = "Voltage:";
        voltValue3 += volt3;
        myFile.println(voltValue3);
        String tempValue3 = "Temperature: ";
        tempValue3 += temp3;
        myFile.println(tempValue3);
        String currValue3 = "Current: ";
        currValue3 += curr3;
        myFile.println(currValue3);
        String posValue3 = "Position: ";
        posValue3 += posi3;
        myFile.println(posValue3);
        myFile.println(nx);
        String polynum3 = "Polynum: ";
        polynum3 += m3;
        myFile.println(polynum3);
        myFile.println("\n");

        myFile.println("SERVO_4: ");
        String torqValue4 = "Torque: ";
        torqValue4 += torq4;
        myFile.println(torqValue4);
        String spedValue4 = "Speed: ";
        spedValue4 += sped4;
        myFile.println(spedValue4);
        String voltValue4 = "Voltage:";
        voltValue4 += volt4;
        myFile.println(voltValue4);
        String tempValue4 = "Temperature: ";
        tempValue4 += temp4;
        myFile.println(tempValue4);
        String currValue4 = "Current: ";
        currValue4 += curr4;
        myFile.println(currValue4);
        String posValue4 = "Position: ";
        posValue4 += posi4;
        myFile.println(posValue4);
        myFile.println(nx);
        String polynum4 = "Polynum: ";
        polynum4 += m4;
        myFile.println(polynum4);
        myFile.println("\n");

        myFile.println("SERVO_5: ");
        String torqValue5 = "Torque: ";
        torqValue5 += torq5;
        myFile.println(torqValue5);
        String spedValue5 = "Speed: ";
        spedValue5 += sped5;
        myFile.println(spedValue5);
        String voltValue5 = "Voltage:";
        voltValue5 += volt5;
        myFile.println(voltValue5);
        String tempValue5 = "Temperature: ";
        tempValue5 += temp5;
        myFile.println(tempValue5);
        String currValue5 = "Current: ";
        currValue5 += curr5;
        myFile.println(currValue5);
        String posValue5 = "Position: ";
        posValue5 += posi5;
        myFile.println(posValue5);
        myFile.println(nx);
        myFile.println(polynum1);
        myFile.println("\n");

        myFile.println("SERVO_6: ");
        String torqValue6 = "Torque: ";
        torqValue6 += torq6;
        myFile.println(torqValue6);
        String spedValue6 = "Speed: ";
        spedValue6 += sped6;
        myFile.println(spedValue6);
        String voltValue6 = "Voltage:";
        voltValue6 += volt6;
        myFile.println(voltValue6);
        String tempValue6 = "Temperature: ";
        tempValue6 += temp6;
        myFile.println(tempValue6);
        String currValue6 = "Current: ";
        currValue6 += curr6;
        myFile.println(currValue6);
        String posValue6 = "Position: ";
        posValue6 += posi6;
        myFile.println(posValue6);
        myFile.println(nx);
        myFile.println(polynum2);
        myFile.println("\n");

        myFile.println("SERVO_7: ");
        String torqValue7 = "Torque: ";
        torqValue7 += torq7;
        myFile.println(torqValue7);
        String spedValue7 = "Speed: ";
        spedValue7 += sped7;
        myFile.println(spedValue7);
        String voltValue7 = "Voltage:";
        voltValue7 += volt7;
        myFile.println(voltValue7);
        String tempValue7 = "Temperature: ";
        tempValue7 += temp7;
        myFile.println(tempValue7);
        String currValue7 = "Current: ";
        currValue7 += curr7;
        myFile.println(currValue7);
        String posValue7 = "Position: ";
        posValue7 += posi7;
        myFile.println(posValue7);
        myFile.println(nx);
        myFile.println(polynum3);
        myFile.println("\n");

        myFile.println("SERVO_8: ");
        String torqValue8 = "Torque: ";
        torqValue8 += torq8;
        myFile.println(torqValue8);
        String spedValue8 = "Speed: ";
        spedValue8 += sped8;
        myFile.println(spedValue8);
        String voltValue8 = "Voltage:";
        voltValue8 += volt8;
        myFile.println(voltValue8);
        String tempValue8 = "Temperature: ";
        tempValue8 += temp8;
        myFile.println(tempValue8);
        String currValue8 = "Current: ";
        currValue8 += curr8;
        myFile.println(currValue8);
        String posValue8 = "Position: ";
        posValue8 += posi8;
        myFile.println(posValue8);
        myFile.println(nx);
        myFile.println(polynum4);
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

          int torq1; int sped1; int volt1; int curr1; int temp1; int posi1;
          int torq2; int sped2; int volt2; int curr2; int temp2; int posi2;
          int torq3; int sped3; int volt3; int curr3; int temp3; int posi3;
          int torq4; int sped4; int volt4; int curr4; int temp4; int posi4;
          int torq5; int sped5; int volt5; int curr5; int temp5; int posi5;
          int torq6; int sped6; int volt6; int curr6; int temp6; int posi6;
          int torq7; int sped7; int volt7; int curr7; int temp7; int posi7;
          int torq8; int sped8; int volt8; int curr8; int temp8; int posi8;

          torq1 = dxlGetTorque(1); sped1 = dxlGetGoalSpeed(1); volt1 = dxlGetVoltage(1);
          temp1 = dxlGetTemperature(1); curr1 = mxGetCurrent(1); posi1 = dxlGetPosition(1);                                     

          torq2 = dxlGetTorque(2); sped2 = dxlGetGoalSpeed(2); volt2 = dxlGetVoltage(2);
          temp2 = dxlGetTemperature(2); curr2 = mxGetCurrent(2); posi2 = dxlGetPosition(2);                                         

          torq3 = dxlGetTorque(3); sped3 = dxlGetGoalSpeed(3); volt3 = dxlGetVoltage(3);
          temp3 = dxlGetTemperature(3); curr3 = mxGetCurrent(3); posi3 = dxlGetPosition(3);
                                            
          torq4 = dxlGetTorque(4); sped4 = dxlGetGoalSpeed(4); volt4 = dxlGetVoltage(4);
          temp4 = dxlGetTemperature(4); curr4 = mxGetCurrent(4); posi4 = dxlGetPosition(4);

          torq5 = dxlGetTorque(5); sped5 = dxlGetGoalSpeed(5); volt5 = dxlGetVoltage(5);
          temp5 = dxlGetTemperature(5); curr5 = mxGetCurrent(5); posi5 = dxlGetPosition(5);

          torq6 = dxlGetTorque(6); sped6 = dxlGetGoalSpeed(6); volt6 = dxlGetVoltage(6);
          temp6 = dxlGetTemperature(6); curr6 = mxGetCurrent(6); posi6 = dxlGetPosition(6);

          torq7 = dxlGetTorque(7); sped7 = dxlGetGoalSpeed(7); volt7 = dxlGetVoltage(7);
          temp7 = dxlGetTemperature(7); curr7 = mxGetCurrent(7); posi7 = dxlGetPosition(7);
  
          torq8 = dxlGetTorque(8); sped8 = dxlGetGoalSpeed(8); volt8 = dxlGetVoltage(8);
          temp8 = dxlGetTemperature(8); curr8 = mxGetCurrent(8); posi8 = dxlGetPosition(8);
        
    int m1 = (((-2.599952146) * (0.01) * (x * x)) + (11.31370255 * x) + (1614.662444));
    int m2 = (((-1.284510229) * (0.1) * (x * x)) + (5.260906283 * x) + (2214.538775));
    int m3 = (((1.524691917) * (0.01) * (x * x)) - (6.568722054 * x) + (2143.11934));
        int m4 = (((-6.885579823) * (0.01) * (x * x)) + (4.268922536 * x) + (1859.013452));
        
        dxlSetGoalPosition(1, m1); dxlSetGoalPosition(2, m2); dxlSetGoalPosition(3, m3);
        dxlSetGoalPosition(4, m4); dxlSetGoalPosition(5, m1); dxlSetGoalPosition(6, m2);
        dxlSetGoalPosition(7, m3); dxlSetGoalPosition(8, m4);
            
          myFile.println("SERVO_1_Return: ");
          String torqValue1 = "Torque: ";
          torqValue1 += torq1;
          myFile.println(torqValue1);
          String spedValue1 = "Speed: ";
          spedValue1 += sped1;
          myFile.println(spedValue1);
          String voltValue1 = "Voltage:";
          voltValue1 += volt1;
          myFile.println(voltValue1);
          String tempValue1 = "Temperature: ";
          tempValue1 += temp1;
          myFile.println(tempValue1);
          String currValue1 = "Current: ";
          currValue1 += curr1;
          myFile.println(currValue1);
          String posValue1 = "Position: ";
          posValue1 += posi1;
          myFile.println(posValue1);
          String nx = "Move#: ";
          nx += x;
          myFile.println(nx);
          String polynum1 = "Polynum: ";
          polynum1 += m1;
          myFile.println(polynum1);
          myFile.println("\n");

          myFile.println("SERVO_2_Return: ");
          String torqValue2 = "Torque: ";
          torqValue2 += torq2;
          myFile.println(torqValue2);
          String spedValue2 = "Speed: ";
          spedValue2 += sped2;
          myFile.println(spedValue2);
          String voltValue2 = "Voltage:";
          voltValue2 += volt2;
          myFile.println(voltValue2);
          String tempValue2 = "Temperature: ";
          tempValue2 += temp2;
          myFile.println(tempValue2);
          String currValue2 = "Current: ";
          currValue2 += curr2;
          myFile.println(currValue2);
          String posValue2 = "Position: ";
          posValue2 += posi2;
          myFile.println(posValue2);
          myFile.println(nx);
          String polynum2 = "Polynum: ";
          polynum2 += m2;
          myFile.println(polynum2);
          myFile.println("\n");

          myFile.println("SERVO_3_Return: ");
          String torqValue3 = "Torque: ";
          torqValue3 += torq3;
          myFile.println(torqValue3);
          String spedValue3 = "Speed: ";
          spedValue3 += sped3;
          myFile.println(spedValue3);
          String voltValue3 = "Voltage:";
          voltValue3 += volt3;
          myFile.println(voltValue3);
          String tempValue3 = "Temperature: ";
          tempValue3 += temp3;
          myFile.println(tempValue3);
          String currValue3 = "Current: ";
          currValue3 += curr3;
          myFile.println(currValue3);
          String posValue3 = "Position: ";
          posValue3 += posi3;
          myFile.println(posValue3);
          myFile.println(nx);
          String polynum3 = "Polynum: ";
          polynum3 += m3;
          myFile.println(polynum3);
          myFile.println("\n");

          myFile.println("SERVO_4_Return: ");
          String torqValue4 = "Torque: ";
          torqValue4 += torq4;
          myFile.println(torqValue4);
          String spedValue4 = "Speed: ";
          spedValue4 += sped4;
          myFile.println(spedValue4);
          String voltValue4 = "Voltage:";
          voltValue4 += volt4;
          myFile.println(voltValue4);
          String tempValue4 = "Temperature: ";
          tempValue4 += temp4;
          myFile.println(tempValue4);
          String currValue4 = "Current: ";
          currValue4 += curr4;
          myFile.println(currValue4);
          String posValue4 = "Position: ";
          posValue4 += posi4;
          myFile.println(posValue4);
          myFile.println(nx);
          String polynum4 = "Polynum: ";
          polynum4 += m4;
          myFile.println(polynum4);
          myFile.println("\n");

          myFile.println("SERVO_5_Return: ");
          String torqValue5 = "Torque: ";
          torqValue5 += torq5;
          myFile.println(torqValue5);
          String spedValue5 = "Speed: ";
          spedValue5 += sped5;
          myFile.println(spedValue5);
          String voltValue5 = "Voltage:";
          voltValue5 += volt5;
          myFile.println(voltValue5);
          String tempValue5 = "Temperature: ";
          tempValue5 += temp5;
          myFile.println(tempValue5);
          String currValue5 = "Current: ";
          currValue5 += curr5;
          myFile.println(currValue5);
          String posValue5 = "Position: ";
          posValue5 += posi5;
          myFile.println(posValue5);
          myFile.println(nx);
          myFile.println(polynum1);
          myFile.println("\n");

          myFile.println("SERVO_6_Return: ");
          String torqValue6 = "Torque: ";
          torqValue6 += torq6;
          myFile.println(torqValue6);
          String spedValue6 = "Speed: ";
          spedValue6 += sped6;
          myFile.println(spedValue6);
          String voltValue6 = "Voltage:";
          voltValue6 += volt6;
          myFile.println(voltValue6);
          String tempValue6 = "Temperature: ";
          tempValue6 += temp6;
          myFile.println(tempValue6);
          String currValue6 = "Current: ";
          currValue6 += curr6;
          myFile.println(currValue6);
          String posValue6 = "Position: ";
          posValue6 += posi6;
          myFile.println(posValue6);
          myFile.println(nx);
          myFile.println(polynum2);
          myFile.println("\n");

          myFile.println("SERVO_7_Return: ");
          String torqValue7 = "Torque: ";
          torqValue7 += torq7;
          myFile.println(torqValue7);
          String spedValue7 = "Speed: ";
          spedValue7 += sped7;
          myFile.println(spedValue7);
          String voltValue7 = "Voltage:";
          voltValue7 += volt7;
          myFile.println(voltValue7);
          String tempValue7 = "Temperature: ";
          tempValue7 += temp7;
          myFile.println(tempValue7);
          String currValue7 = "Current: ";
          currValue7 += curr7;
          myFile.println(currValue7);
          String posValue7 = "Position: ";
          posValue7 += posi7;
          myFile.println(posValue7);
          myFile.println(nx);
          myFile.println(polynum3);
          myFile.println("\n");

          myFile.println("SERVO_8_Return: ");
          String torqValue8 = "Torque: ";
          torqValue8 += torq8;
          myFile.println(torqValue8);
          String spedValue8 = "Speed: ";
          spedValue8 += sped8;
          myFile.println(spedValue8);
          String voltValue8 = "Voltage:";
          voltValue8 += volt8;
          myFile.println(voltValue8);
          String tempValue8 = "Temperature: ";
          tempValue8 += temp8;
          myFile.println(tempValue8);
          String currValue8 = "Current: ";
          currValue8 += curr8;
          myFile.println(currValue8);
          String posValue8 = "Position: ";
          posValue8 += posi8;
          myFile.println(posValue8);
          myFile.println(nx);
          myFile.println(polynum4);
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

    else if (0 == strcmp(temp,"front_board_reset")) {
      //Return to home position
      delay(500);
      dxlSetGoalPosition(1, 2048); dxlSetGoalPosition(2, 2048); dxlSetGoalPosition(3, 2048);
      dxlSetGoalPosition(4, 2048); dxlSetGoalPosition(5, 2048); dxlSetGoalPosition(6, 2048);
      dxlSetGoalPosition(7, 2048); dxlSetGoalPosition(8, 2048);
    }
    else if (0 == strcmp(temp, "front_board_end")) {
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
  dxlSetGoalPosition(1, 2048); dxlSetGoalPosition(2, 2048); dxlSetGoalPosition(3, 2048);
  dxlSetGoalPosition(4, 2048); dxlSetGoalPosition(5, 2048); dxlSetGoalPosition(6, 2048);             
  dxlSetGoalPosition(7, 2048); dxlSetGoalPosition(8, 2048);
      
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
