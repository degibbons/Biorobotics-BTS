from smbus import SMBus
import time
import sys
import RPi.GPIO as GPIO

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)

bus = SMBus(1)
slaveAddress1 = 0x13
slaveAddress2 = 0x14
cont = 1
scnd = 1

GPIO.output(17, 1)
time.sleep(1)
GPIO.output(17, 0)

print("Reset Arbotix-M Successfully")

x=[10, 9, 8, 7, 6, 5, 4, 3, 2, 1]
print("Beginning Countdown:\n ")
for i in x:
    print(i)
    time.sleep(1.75)
print(" ")

while(cont):
    readyGo = 0
    try:
        bus.write_byte(slaveAddress1,1)
        print("Sending communication check to Arbotix-m 1.")
        bus.write_byte(slaveAddress2,1)
        print("Sending communication check to Arbotix-m 2.")
        print(" ")
        time.sleep(2)
        readyArd1 = bus.read_byte(slaveAddress1)
        print("Receiving Confirmation from Arbotix-m 1.")
        print(readyArd1)
        readyArd2 = bus.read_byte(slaveAddress2)
        print("Receiving Confirmation from Arbotix-m 2.")
        print(readyArd2)
        print(" ")
        if (readyArd1 == 1):
            readyGo = 1
            print("Arbotix-m 1 ready.")
            if (readyArd2 == 1):
                print("Arbotix-m 2 ready.\n")
                print("Now check for SD Card.")
                readyGo = 2
            else:
                print("Arbotix-m 1 ready, Arbotix-m 2 NOT ready.") 
    except KeyboardInterrupt:
        print("Aborting. Try Code Again")
        sys.exit()
    except:
        print("Something went wrong. Go back and fix it.")
        sys.exit()
    if (readyGo == 2):
        print("Successful Arbotix-m Check, Progressing!\n")
        cont = 0
    else:
        print("Error, Try again")
        pass


while(scnd):
    print("Entering SD Card Check Protocol.\n")
    commence = 0
    time.sleep(2)
    try:
        x = bus.read_byte(slaveAddress1)
        print("Arbotix-m 1 value read.")
        print(x)
        y = bus.read_byte(slaveAddress2)
        print("Arbotix-m 2 value read.")
        print(y)
        print(" ")
        if (x == 4):
            print("Arbotix-m 1 has SD card.")
            time.sleep(2)
            if (y == 4):
                print("Arbotix-m 2 has SD card.\n")
                bus.write_byte(slaveAddress1,2)
                time.sleep(1)
                bus.write_byte(slaveAddress2,2)
                commence = 1
        elif (x == 0):
            print("No SD Card Detected, Checking Again.")
            time.sleep(2)
    except EOFError as error:
        print(error) 
    if (commence == 1):
        scnd=0
        print("All Checks Finished. Beginning Movement\n\n\n")
    else:
        print("Beginning SD Card Check Again.")
        pass
