from smbus import SMBus
import time
import sys
import RPi.GPIO as GPIO

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)
GPIO.setup(27, GPIO.OUT)
GPIO.setup(22, GPIO.OUT)

bus = SMBus(1)
slaveAddress1 = 0x13
slaveAddress2 = 0x14
first = 1
scnd = 1
thrd = 1

GPIO.output(17, 1)
time.sleep(1)
GPIO.output(17, 0)

# Need to alter these for MECHANICAL (or biological) limits 
maxes = {1:180,2:180,3:180,4:180,5:180,6:180,7:180,8:180,9:180,10:180,11:180,12:180,13:180,14:180,15:180,16:180}
mins = {1:1,2:0,3:0,4:0,5:0,6:0,7:0,8:0,9:0,10:0,11:0,12:0,13:0,14:0,15:0,16:0}

def StringToBytes(val):
    retVal = []
    for c in val:
        retVal.append(ord(c))
    return retVal
    
def writeData(address,value):
    byteValue = StringToBytes(value)    
    bus.write_i2c_block_data(address,0x00,byteValue) #first byte is 0=command byte.. just is.
    return -1

print("Reset Arbotix-M Successfully")

x=[10, 9, 8, 7, 6, 5, 4, 3, 2, 1]
print("Beginning Countdown:\n ")
for i in x:
    print(i)
    time.sleep(1.75)
print(" ")

while(first):
    readyGo = 0
    try:
        #bus.write_byte(slaveAddress1,1)
        writeData(slaveAddress1,"front_com_check")
        print("Sending communication check to Arbotix-m Front.")
        #bus.write_byte(slaveAddress2,1)
        writeData(slaveAddress2,"back_com_check")
        print("Sending communication check to Arbotix-m Back.")
        print(" ")
        time.sleep(2)
        readyArd1 = bus.read_byte(slaveAddress1)
        print("Receiving Confirmation from Arbotix-m Front.")
        print(readyArd1)
        readyArd2 = bus.read_byte(slaveAddress2)
        print("Receiving Confirmation from Arbotix-m Back.")
        print(readyArd2)
        print(" ")
        if (readyArd1 == 1):
            readyGo = 1
            print("Arbotix-m Front ready.")
            if (readyArd2 == 1):
                print("Arbotix-m Back ready.\n")
                readyGo = 2
            else:
                print("Arbotix-m Front ready, Arbotix-m Back NOT ready.") 
    except KeyboardInterrupt:
        print("\tAborting. Try Code Again")
        sys.exit()
    except:
        print("Something went wrong. Go back and fix it.")
        sys.exit()
    if (readyGo == 2):
        print("Successful Arbotix-m Check, Progressing!")
        print("Now check for SD Card.\n")
        first = 0
    else:
        print("Error, Try again")
        pass


while(scnd):
    print("Entering SD Card Check Protocol.\n")
    commence = 0
    time.sleep(2)
    try:
        x = bus.read_byte(slaveAddress1)
        print("Arbotix-m Front value read.")
        print(x)
        y = bus.read_byte(slaveAddress2)
        print("Arbotix-m Back value read.")
        print(y)
        print(" ")
        if (x == 2):
            print("Arbotix-m Front has SD card.")
            time.sleep(2)
            if (y == 2):
                print("Arbotix-m Back has SD card.\n")
                writeData(slaveAddress1,"front_sd_present")
                time.sleep(1)
                writeData(slaveAddress2,"back_sd_present")
                commence = 1
        elif (x == 0):
            print("No SD Card Detected, Checking Again.")
            time.sleep(2)
    except EOFError as error:
        print(error) 
    if (commence == 1):
        scnd=0
        print("All Checks Finished.\n\n")
    else:
        print("Beginning SD Card Check Again.")
        pass

#Standby Mode
while (thrd):
    print('\n\n=== In Standby Mode ===\n')
    print('Type "Start" to start the robot moving.')
    print('Type "Move" to move a specified servo a desired distance.')
    print('Type "Stop" to stop Theo at any specified moment.')
    print('Note: While stopped, type "Continue" to resume movement.')
    print('Type "Reset" at any point to return to the home position and this menu.')
    print('Type "End" to end movement and stop the robot entirely.')
    while(thrd):
        endofstride = 0
        usrcmd = input("Command: ")
        if (usrcmd.lower() == "start"):
            print("\n\n=== In Stride Mode ===\n")
            #Do movements, then loop, waiting for a stop or a finish return
            print("Starting Movement.\n")
            firstHalfofStride = input("How many points should the first half of the stride consist of?: ")
            firstHalfofStride = int(firstHalfofStride)
            if (firstHalfofStride <0):
                print("Theo needs a number zero or greater. Try again.")
                break
            secondHalfofStride = input("And how many points should the second half of the stride consist of?: ")
            secondHalfofStride = int(secondHalfofStride)
            if (firstHalfofStride <0):
                print("Theo needs a number zero or greater. Try again.")
                break
            firstHalfofStride = firstHalfofStride + 197
            secondHalfofStride = secondHalfofStride + 197
            firstHalfofStride = str(firstHalfofStride)
            secondHalfofStride = str(secondHalfofStride)
            writeData(slaveAddress1,firstHalfofStride)
            writeData(slaveAddress2,firstHalfofStride)
            writeData(slaveAddress1,secondHalfofStride)
            writeData(slaveAddress2,secondHalfofStride)
            time.sleep(0.1)
            writeData(slaveAddress1,"front_board_start")
            writeData(slaveAddress2,"back_board_start")
        elif (usrcmd.lower() == "move"):
            print("\n\n=== In Single Servo Mode ===\n")
            writeData
            #Ask for specific servo and either percentage or specific pin location
            srvo = input("Which Servo would you like to move? (Number 1 - 16): ")
            try:
                value = int(srvo)
                if 1 <= value <= 16:
                    servMax = maxes[value]
                    servMin = mins[value]
                    print("The minimum and maximum of this servo is:")    
                    print("Minimum: " + servMin + "\t Maximum: " + servMax)
                    dist = input("What distance would you like to move servo number " + srvo + "?")
                    if (servMin <= dist <= servMax):
                        if (1<= value <= 8):
                            srvo = str(srvo)
                            dist = dist + 17
                            dist = str(dist)
                            writeData(slaveAddress1,srvo) # Write Servo
                            time.sleep(1)
                            writeData(slaveAddress1,dist) # Then Write Distance
                        elif (9 <= value <= 16):
                            srvo = str(srvo)
                            dist = dist + 17
                            dist = str(dist)
                            writeData(slaveAddress2,srvo) # Write Servo
                            time.sleep(1)
                            writeData(slaveAddress2,dist) # Then Write Distance
                    else:
                        print("Distance does not fall within the given bounds. Try again.")
                        break
                else:
                    print("Sorry, Servo # is not between 1 and 16. Try again.")
                    break
            except ValueError:
                print("Sorry, that's not an integer value. Try again.")
                continue
        elif (usrcmd.lower() == "stop"):
           # writeData(slaveAddress1,"front_board_stop")
           # writeData(slaveAddress2,"back_board_stop")
            GPIO.output(27, 1)
            time.sleep(2)
            GPIO.output(27, 0)

            continue
        elif (usrcmd.lower() == "continue"):
            GPIO.output(22, 1)
            time.sleep(2)
            GPIO.output(22, 0)

           # writeData(slaveAddress1,"front_board_continue")
           # writeData(slaveAddress2,"back_board_continue")
            continue
        elif (usrcmd.lower() == "reset"):
            # Reset to home position, then break out of loop and 
            writeData(slaveAddress1,"front_board_reset")
            time.sleep(1)
            writeData(slaveAddress2,"back_board_reset")
            endofstride = 0
            break
        elif (usrcmd.lower() == "end"):
            # End the entire movement. Will need to run entire code again to move
            rstpos = 1
            while (rstpos):
                rst = input("Reset to home position? (Y/n): ")
                if (rst.lower() == "y"):
                    # Reset to home position, then break out
                    writeData(slaveAddress1,"front_board_reset")
                    time.sleep(1)
                    writeData(slaveAddress2,"back_board_reset")
                    rstpos = 0
                    print("Thanks for using Theo!")
                    thrd = 0
                    break
                elif (rst.lower() == "n"):
                    # Just break out
                    rstpos = 0
                    print("Thanks for using Theo!")
                    thrd = 0
                else:
                    print("Not a recognized command. Try again.")
                    continue
                writeData(slaveAddress1,"front_board_end")
                time.sleep(1)
                writeData(slaveAddress2,"back_board_end")
                endofstride = 1
            else:
                print("Not a recognized command. Try again.") 
                continue
    else:
        pass

# Make all variables make sense!
# Also, COMMENT WITHIN CODE TO EXPLAIN STEPS!
