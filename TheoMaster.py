from smbus import SMBus
import numpy as np
import pandas as pd
import math
import time
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
first = 1
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

def get_pos_num(num):
    if (num != 0):
        if ((math.floor((num % .01) * 1000))==5):
            roundNum = 1
        elif ((math.floor((num % .01) * 1000))!=5):
            roundNum = 0
    return roundNum



df = pd.read_csv('KinematicsTiming.csv')
cf = df
tspan = float(input("What timespan in seconds do you want one stride to take?\n"))
h_stance = 4.892994
h_swing = 3.347006
f_stance = 5.211718
f_swing = 3.028282
h_st_per = h_stance / 8.24
h_sw_per = h_swing / 8.24
f_st_per = f_stance / 8.24
f_sw_per = f_swing / 8.24
b = df.shape
cLength = b[0]
cWidth = b[1]
percents = np.linspace(1,cLength-1,cLength-1)
my_dict = {1:'Joint 1_st',2:'Joint 2_st',3:'Joint 3_st',4:'Joint 4_st',5:'Joint 1_sw',
            6:'Joint 2_sw',7:'Joint 3_sw',8:'Joint 4_sw',9:'Joint 5_st',10:'Joint 6_st',
            11:'Joint 7_st',12:'Joint 8_st',13:'Joint 5_sw',14:'Joint 6_sw',15:'Joint 7_sw',
            16:'Joint 8_sw'}
joints = list(range(1,cWidth+1))
for i in percents:
    for j in joints:
        if (j == 1 or j == 2 or j == 3 or j == 4 ):
            cf.loc[i-1,my_dict[j]]= round(((abs(df.loc[i,my_dict[j]]-df.loc[i-1,my_dict[j]])/4096) / ((tspan*f_st_per/cLength)/60))/0.114,5) # Not cLength - 1 because extra one included for transition phase points
            roundOrNot = get_pos_num(cf.loc[i-1,my_dict[j]])
        elif (j == 5 or j == 6 or j == 7 or j == 8): 
            cf.loc[i-1,my_dict[j]]= round(((abs(df.loc[i,my_dict[j]]-df.loc[i-1,my_dict[j]])/4096) / ((tspan*f_sw_per/cLength)/60))/0.114,5)
            roundOrNot = get_pos_num(cf.loc[i-1,my_dict[j]])
        elif (j == 9 or j == 10 or j == 11 or j == 12 ):
            cf.loc[i-1,my_dict[j]]= round(((abs(df.loc[i,my_dict[j]]-df.loc[i-1,my_dict[j]])/4096) / ((tspan*h_st_per/cLength)/60))/0.114,5)
            roundOrNot = get_pos_num(cf.loc[i-1,my_dict[j]])
        elif (j == 13 or j == 14 or j == 15 or j == 16 ):
            cf.loc[i-1,my_dict[j]]= round(((abs(df.loc[i,my_dict[j]]-df.loc[i-1,my_dict[j]])/4096) / ((tspan*h_sw_per/cLength)/60))/0.114,5)  
            roundOrNot = get_pos_num(cf.loc[i-1,my_dict[j]])
        if (roundOrNot == 1):
            if ((math.floor(cf.loc[i-1,my_dict[j]])%2)==1):
                cf.loc[i-1,my_dict[j]] = math.ceil(cf.loc[i-1,my_dict[j]])
                cf.loc[i-1,my_dict[j]] = int(cf.loc[i-1,my_dict[j]])
            elif ((math.floor(cf.loc[i-1,my_dict[j]])%2)==0):
                cf.loc[i-1,my_dict[j]] = math.floor(cf.loc[i-1,my_dict[j]])
                cf.loc[i-1,my_dict[j]] = int(cf.loc[i-1,my_dict[j]])
        else:
            cf.loc[i-1,my_dict[j]] = int(round(cf.loc[i-1,my_dict[j]]))
            # Even down
            # Odd Up


mf = cf.values
np.delete(mf,-1,0)

tranSpeeds = np.zeros((2,int(cWidth/2)))
lastLine = df.tail(1)
for k in [0,1]:
    for l in list(range(0,int(cWidth/2))): 
        if (l == 1 or l == 2 or l == 3 or l == 4 ):
            if (k == 0): #Stance to Swing
                tranSpeeds[k][l] = round(((abs(lastLine.loc[cLength-1,my_dict[l]] - df.loc[0,my_dict[l+4]])/4096)/ ((tspan*f_st_per/cLength)/60))/0.114,5)
                roundOrNot = get_pos_num(tranSpeeds[k][l])
            elif (k == 1): #Swing to Stance
                tranSpeeds[k][l] = round(((abs(lastLine.loc[cLength-1,my_dict[l+4]] - df.loc[0,my_dict[l]])/4096)/ ((tspan*f_sw_per/cLength)/60))/0.114,5)
                roundOrNot = get_pos_num(tranSpeeds[k][l])
        elif (l == 5 or l == 6 or l == 7  or l == 8 ):
            r = l + 4
            if (k == 0):
                tranSpeeds[k][l] = round(((abs(lastLine.loc[cLength-1,my_dict[r]] - df.loc[0,my_dict[r+4]])/4096)/ ((tspan*h_st_per/cLength)/60))/0.114,5)
                roundOrNot = get_pos_num(tranSpeeds[k][l])
            elif (k == 1):
                tranSpeeds[k][l] = round(((abs(lastLine.loc[cLength-1,my_dict[r+4]] - df.loc[0,my_dict[r]])/4096)/ ((tspan*h_sw_per/cLength)/60))/0.114,5)
                roundOrNot = get_pos_num(tranSpeeds[k][l])
        if (roundOrNot == 1):
            if ((math.floor(tranSpeeds[k][l])%2)==1):
                tranSpeeds[k][l] = math.ceil(tranSpeeds[k][l])
                tranSpeeds[k][l] = int(tranSpeeds[k][l])
            elif ((math.floor(tranSpeeds[k][l])%2)==0):
                tranSpeeds[k][l] = math.floor(tranSpeeds[k][l])
                tranSpeeds[k][l] = int(tranSpeeds[k][l])
        else:
            tranSpeeds[k][l] = int(round(tranSpeeds[k][l]))


(e,f) = np.array_split(tranSpeeds,2,axis=1)
e = e.flatten('C')
f = f.flatten('C')
g = np.concatenate((e,f),axis = 0)
h = np.expand_dims(g, axis=0)
mf = np.concatenate((mf,h))

rows = mf.shape[0]
cols = mf.shape[1]
for x in range(0, rows):
    for y in range(0, cols):
        if (mf[x][y] > 1023):
            mf[x][y] = 1023
            

print('\nSending Speeds to Arbotix Board...')
writeData(boardAddress,'speeds') #Prepare to receive Main Speeds
time.sleep(1)
sp1 = mf.shape
for m in list(range(0,sp1[1]-1)):
    for n in list(range(0,sp1[0]-1)):
        writeData(boardAddress,str(mf[n][m]))
        time.sleep(.05)
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