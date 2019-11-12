#include <stdio.h>
#include <ax12.h>               //include base library for DYNAMIXELs
#include <BioloidController.h>  //include bioloid libary for poses/movements
#include "poses.h"              //home and center poses for the robot

BioloidController bioloid = BioloidController(1000000); //start the BIOLOID library at 1mbps. This will run the dxlInit() function internally, so we don't need to call it

const int SERVOCOUNT = 4;  //number of servos in this robot
int id;                    //temperary id for movement
int pos;                   //temporary position for movement
boolean runCheck = false;  //flag to see if we're running, so that we don't print out the menu options unnecessarily

String polyValue;
String loopValue;
String torqValue;
String spedValue;
String voltValue;
String tempValue;
String currValue;
String posiValue;

void setup()
{
  pinMode(USER_LED, OUTPUT); //user led as an output
  digitalWrite(USER_LED, HIGH); //set LED high to show that the test has started

  Serial.begin(9600); //open serial port

  Serial.println("######################################################");
  Serial.println("Serial Communication Established.");
  Serial.println("Starting PhantomX Turret Test.");

  dxlVoltageReport(SERVOCOUNT);  //serial report for the system voltage
  dxlServoReport(SERVOCOUNT);    //Scan Servos, return position and error (if there are any)



  runCheck = true;  //any function calls will show the menu after this
  dxlSetGoalPosition(1, 1825.75984);
  dxlSetGoalPosition(2, 2214.538775);
  dxlSetGoalPosition(3, 2478.157503);
  dxlSetGoalPosition(4, 2016.952725);

for (int x = 0; x < 63; x = x + 1) {
  
}

  //int
  for (int x = 0; x < 63; x = x + 1) {
    //    int m1;
    int torq;
    int sped;
    int volt;
    int curr;
    int temp;
    int posi;

    int m1 = (((-1.437728279) * (0.1) * (x * x)) + (18.696767 * x) + (1825.75984));
    int m2 = (((-1.284510229) * (0.1) * (x * x)) + (5.260906283 * x) + (2214.538775));
    int m3 = (((1.621709744) * (0.1) * (x * x)) - (27.68891768 * x) + (2478.157503));
    int m4 = (((-5.259269244) * (0.1) * (x * x)) + (25.01440985 * x) + (2016.952725));


    dxlSetGoalPosition(1, m1);
    dxlSetGoalPosition(2, m2);
    dxlSetGoalPosition(3, m3);
    dxlSetGoalPosition(4, m4);


    torq = dxlGetTorque(4);
    sped = dxlGetSpeed(4);
    volt = dxlGetVoltage(4);
    temp = dxlGetTemperature(4);
    curr = mxGetCurrent(4);
    posi = dxlGetPosition(4);

    polyValue = String("The Polynomial Value is ");
    loopValue = String("The Loop Value is ");
    torqValue = String("The Torque Value is ");
    spedValue = String("The Speed Value is ");
    voltValue = String("The Voltage Value is ");
    tempValue = String("The Temperature Value is ");
    currValue = String("The Current Value is ");
    posiValue = String("The Position Value is ");

    // dxlGetTorque(2);
    // dxlGetTorque(3);
    // dxlGetTorque(4);

    Serial.print(polyValue);
    Serial.println(m4);
    Serial.print(loopValue);
    Serial.println(x);
    Serial.print(torqValue);
    Serial.println(torq);
    Serial.print(spedValue);
    Serial.println(sped);
    Serial.print(voltValue);
    Serial.println(volt);
    Serial.print(tempValue);
    Serial.println(temp);
    Serial.print(currValue);
    Serial.println(curr);
    Serial.print(posiValue);
    Serial.println(posi);
    Serial.println("\n");


    delay(100);

  }

  for (int x = 63; x > 0; x = x - 1) {

    int m1 = (((-1.437728279) * (0.1) * (x * x)) + (18.696767 * x) + (1825.75984));
    int m2 = (((-1.284510229) * (0.1) * (x * x)) + (5.260906283 * x) + (2214.538775));
    int m3 = (((1.621709744) * (0.1) * (x * x)) - (27.68891768 * x) + (2478.157503));
    int m4 = (((-5.259269244) * (0.1) * (x * x)) + (25.01440985 * x) + (2016.952725));

    dxlSetGoalPosition(1, m1);
    dxlSetGoalPosition(2, m2);
    dxlSetGoalPosition(3, m3);
    dxlSetGoalPosition(4, m4);


    delay(100);

  }
  //dxlSetGoalPosition(1, 1825.75984);
  //dxlSetGoalPosition(2, 2214.538775);
  //dxlSetGoalPosition(3, 2478.157503);
  //dxlSetGoalPosition(4, 2016.952725);
}




void loop() {}
