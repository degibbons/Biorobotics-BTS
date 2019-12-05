from smbus import SMBus
import time
import sys
import RPi.GPIO as GPIO
bus = SMBus(1)
slaveAddress1 = 0x13
slaveAddress2 = 0x14
cont = 1
scnd = 1
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)
GPIO.output(17, 1)
time.sleep(1)
GPIO.output(17, 0)
print("Reset Arbotix-M succesefully")
time.sleep(12)
while(cont):
    readyGo = 0
    try:
        bus.write_byte(slaveAddress1,1)
        print("1 sent to Arduino 1.")
        bus.write_byte(slaveAddress2,1)
        print("1 sent to Arduino 2.")
        time.sleep(2)
        readyArd1 = bus.read_byte(slaveAddress1)
        print("Reading From Arduino 1.")
        print(readyArd1)
        readyArd2 = bus.read_byte(slaveAddress2)
        print("Reading From Arduino 2.")
        print(readyArd2)
        if (readyArd1 == 1):
            readyGo = 1
            print("Ard 1 ready.")
            if (readyArd2 == 1):
                print("Ard 2 ready. Now check for SD.")
                readyGo = 2
            else:
                print("Ard 1 ready, Ard 2 NOT ready.") 
    except KeyboardInterrupt:
        print("SOS, ABANDON SHIP!")
        sys.exit()
    except:
        print("Something went wrong. Go back and fix it!")
        sys.exit()
    if (readyGo == 2):
        print("Successful Arduino Check, Progressing!")
        cont = 0
    else:
        print("That's a no go, Try again")
        pass


while(scnd):
    print("Second loop Entered")
    commence = 0
    time.sleep(2)
    try:
        x = bus.read_byte(slaveAddress1)
        print("Arduino 1 value written.")
        print(x)
        y = bus.read_byte(slaveAddress2)
        print("Arduino 2 value written.")
        print(y)
        if (x == 4):
            print("Arduino 1 has SD card.")
            time.sleep(2)
            if (y == 4):
                print("Arduino 2 has SD card. Green Light!")
                bus.write_byte(slaveAddress1,2)
                time.sleep(1)
                bus.write_byte(slaveAddress2,2)
                commence = 1
        elif (x == 0):
            print("NO SD CARD YET!")
            time.sleep(2)
    except EOFError as error:
        print(error) 
    if (commence == 1):
        scnd=0
        print("FINISHED!")
    else:
        print("Check for SD cards AGAIN!")
        pass
