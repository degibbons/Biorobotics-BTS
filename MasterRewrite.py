from smbus import SMBus
import numpy as np
import pandas as pd
import math
import time
import copy as cp
import sys
import RPi.GPIO as GPIO

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)
GPIO.setup(27, GPIO.OUT)
GPIO.setup(22, GPIO.OUT)
GPIO.setup(15, GPIO.OUT)
GPIO.setup(21, GPIO.OUT)
GPIO.setup(23, GPIO.OUT)
boardAddress = 0x13

bus = SMBus(1)
second = 1

def StringToBytes(val):
    retVal = []
    for c in val:
        retVal.append(ord(c))
    return retVal

def writeData(address,value):
    byteValue = StringToBytes(value)    
    bus.write_i2c_block_data(address,0x00,byteValue) #first byte is 0=command byte
    return -1

df = pd.read_csv('KinematicsTiming3.csv')

df = df.values[:][:]
cf = cp.copy(df)

tspan = float(input("What timespan in seconds do you want one stride to take?\n"))

h_stance = 4.892994
h_swing = 3.347006
f_stance = 5.211718
f_swing = 3.028282
h_st_per = h_stance / 8.24
h_sw_per = h_swing / 8.24
f_st_per = f_stance / 8.24
f_sw_per = f_swing / 8.24
b = cf.shape
cLength = b[0]
cWidth = b[1]
percents = np.linspace(1,cLength-1,cLength-1)
joints = list(range(1,cWidth+1))
percents = percents.astype(int).tolist()

for i in percents:
    for j in joints:
        if (j == 1 or j == 2 or j == 3 or j == 4 ):
            T0 = abs(cf[i][j-1]-cf[i-1][j-1])/4096
            T2 = (tspan*f_st_per/cLength)/60
            T3 = (T0 / T2) / 0.114
            T4 = round(T3,5)
            cf[i-1][j-1] = T4
        elif (j == 5 or j == 6 or j == 7 or j == 8): 
            T0 = abs(cf[i][j-1]-cf[i-1][j-1])/4096
            T2 = (tspan*f_sw_per/cLength)/60
            T3 = (T0 / T2) / 0.114
            T4 = round(T3,5)
            cf[i-1][j-1] = T4
        elif (j == 9 or j == 10 or j == 11 or j == 12 ):
            T0 = abs(cf[i][j-1]-cf[i-1][j-1])/4096
            T2 = (tspan*h_st_per/cLength)/60
            T3 = (T0 / T2) / 0.114
            T4 = round(T3,5)
            cf[i-1][j-1] = T4
        elif (j == 13 or j == 14 or j == 15 or j == 16 ):
            T0 = abs(cf[i][j-1]-cf[i-1][j-1])/4096
            T2 = (tspan*h_sw_per/cLength)/60
            T3 = (T0 / T2) / 0.114
            T4 = round(T3,5)
            cf[i-1][j-1] = T4
cf = np.round(cf)
joints = list(range(0,cWidth))
for j in joints:
    if (j == 0 or j == 1 or j == 2 or j == 3):
        R1 = abs(df[-1][j] - df[0][j+4])/4096
        R2 = (tspan * f_st_per / cLength) / 60
        R3 = (R1 / R2) / 0.114
        cf[-1][j] = round(R3)
    elif (j == 4 or j == 5 or j == 6 or j == 7):
        R1 = abs(df[-1][j] - df[0][j-4])/4096
        R2 = (tspan * f_sw_per / cLength) / 60
        R3 = (R1 / R2) / 0.114
        cf[-1][j] = round(R3)
    elif (j == 8 or j == 9 or j == 10 or j == 11):
        R1 = abs(df[-1][j] - df[0][j+4])/4096
        R2 = (tspan * h_st_per / cLength) / 60
        R3 = (R1 / R2) / 0.114
        cf[-1][j] = round(R3)
    elif (j == 12 or j == 13 or j == 14 or j == 15):
        R1 = abs(df[-1][j] - df[0][j-4])/4096
        R2 = (tspan * h_sw_per / cLength) / 60
        R3 = (R1 / R2) / 0.114
        cf[-1][j] = round(R3)
cf[cf==0]=1
rows = cf.shape[0]
cols = cf.shape[1]
for x in range(0, rows):
    for y in range(0, cols):
        if (cf[x][y] > 1023):
            cf[x][y] = 1023

print('\nSending Speeds to Arbotix Board...')
writeData(boardAddress,'speeds') #Prepare to receive Main Speeds
time.sleep(1)
for m in list(range(0,cols-1)):
    for n in list(range(0,rows-1)):
        writeData(boardAddress,str(cf[n][m]))
#         time.sleep(.05)
time.sleep(1)
writeData(boardAddress,'fin') #Finished Sending Data

while(second):
    print('\n\n=== In Standby Mode ===\n')
    print('Type "Start" to start the robot moving.')
    print('Type "Stop" to stop Theo at any specified moment.')
    print('Note: While stopped, type "Continue" to resume movement.')
    print('Type "Reset" at any point to return to the home position and this menu.')
    print('Type "End" to end movement and stop the robot entirely.\n')
    while (second):
        usrcmd = input("Command: ")
        if (usrcmd.lower() == "start"):
            print("Starting Movement.\n")
            GPIO.output(21, 1)
            time.sleep(1)
            GPIO.output(21, 0)
            continue
        elif (usrcmd.lower() == "stop"):
            GPIO.output(27, 1)
            time.sleep(2)
            GPIO.output(27, 0)
            continue
        elif (usrcmd.lower() == "continue"):
            GPIO.output(22, 1)
            time.sleep(2)
            GPIO.output(22, 0)
            continue
        elif (usrcmd.lower() == "reset"):
            GPIO.output(15, 1)
            time.sleep(2)
            GPIO.output(15, 0)
            continue
        elif (usrcmd.lower() == "end"):
            GPIO.output(23, 1)
            time.sleep(2)
            GPIO.output(23, 0)
            first = 0