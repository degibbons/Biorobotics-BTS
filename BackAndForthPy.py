from smbus import SMBus
import time
bus = SMBus(1)
slaveAddress1 = 0x13
slaveAddress2 = 0x14

while(1):
    try:
        readyGo = 0
        bus.write_byte(slaveAddress1,1)
        readyArd1 = bus.read_byte(slaveAddress1)
        bus.write_byte(slaveAddress2,1)
        readyArd2 = bus.read_byte(slaveAddress2)
        if (readyArd1 = = 1):
            readyGo = 1
            print("Ard 1 ready.")
            if (readyArd2 == 1):
                print("Ard 2 ready. Now check for SD.")
                readyGo == 2
            else :
                print("Ard 1 ready, Ard 2 NOT ready.")
    if (readyGo == 2):
        break
    else:
        pass


while(1):
    commence = 0;
    try:
        x = bus.read_byte(slaveAddress1)
        y = bus.read_byte(slaveAddress2)
        if (x == 2):
            print("Arduino 1 has SD card.")
            if (y == 2):
                print("Arduino 2 has SD card. Green Light!")
                bus.write_byte(slaveAddress1,2)
                bus.write_byte(slaveAddress2,2)
                commence = 1
                break
        else if (x == 0):
            print("NO SD CARD YET!")
    except EOFError as error:
        print(error) 
    if (commence == 1):
        break
    else:
        pass