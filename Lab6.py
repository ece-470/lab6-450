import RPi.GPIO as  GPIO
import time

# These are the pins we will be using.
# ENA-6, ENB-26, IN1-12, IN2-13, IN3-20, IN4-21
# CS-5, DataOut-23, Address-24, Clock-25
ENA = 6
ENB = 26
IN1 = 12
IN2 = 13
IN3 = 20
IN4 = 21
CS = 5
DataOut = 23
Address = 24
Clock = 25


numSensors = 5
gvalue = [0,0,0,0,0,0] #array to hold values of the rake sensor

#Functions to carry out movement of the robot
# 0 = GPIO.LOW  1 = GPIO.HIGH
def moveBackward():
    # print("Moving Backward...")
    GPIO.output(IN1,GPIO.HIGH)
    GPIO.output(IN2,GPIO.LOW)
    GPIO.output(IN3,GPIO.LOW)
    GPIO.output(IN4,GPIO.HIGH)

def stop():
    # print("Stopping...")
    GPIO.output(IN1,GPIO.LOW)
    GPIO.output(IN2,GPIO.LOW)
    GPIO.output(IN3,GPIO.LOW)
    GPIO.output(IN4,GPIO.LOW)

def moveForward():
    # print("Moving Forward...")
    GPIO.output(IN1,GPIO.LOW)
    GPIO.output(IN2,GPIO.HIGH)
    GPIO.output(IN3,GPIO.HIGH)
    GPIO.output(IN4,GPIO.LOW)


def pivotRight():
    # print("Left Pivot...")
    GPIO.output(IN1,GPIO.HIGH)
    GPIO.output(IN2,GPIO.LOW)
    GPIO.output(IN3,GPIO.HIGH)
    GPIO.output(IN4,GPIO.LOW)

def pivotLeft():
    # print("Right Pivot...")
    GPIO.output(IN1,GPIO.LOW)
    GPIO.output(IN2,GPIO.HIGH)
    GPIO.output(IN3,GPIO.LOW)
    GPIO.output(IN4,GPIO.HIGH)

def turnRight():
    # print("Turn Right...")
    GPIO.output(IN1,GPIO.LOW)
    GPIO.output(IN2,GPIO.LOW)
    GPIO.output(IN3,GPIO.HIGH)
    GPIO.output(IN4,GPIO.LOW)


def turnLeft():
    # print("Turn Left...")
    GPIO.output(IN1,GPIO.LOW)
    GPIO.output(IN2,GPIO.HIGH)
    GPIO.output(IN3,GPIO.LOW)
    GPIO.output(IN4,GPIO.LOW)




# Reads in the values from the sensors, returns the values
def readAnalog():
    value = [0,0,0,0,0,0] #array to hold values of the rake sensor
    for j in range(0,6):
        GPIO.output(CS, GPIO.LOW)
        for i in range(0,4):
            #sent 4-bit Address
            if(((j) >> (3 - i)) & 0x01):
                GPIO.output(Address,GPIO.HIGH)
            else:
                GPIO.output(Address,GPIO.LOW)
            #read MSB 4-bit data
            value[j] <<= 1
            if(GPIO.input(DataOut)):
                value[j] |= 0x01
            GPIO.output(Clock,GPIO.HIGH)
            GPIO.output(Clock,GPIO.LOW)
        for i in range(0,6):
            #read LSB 8-bit data
            value[j] <<= 1
            if(GPIO.input(DataOut)):
                value[j] |= 0x01
            GPIO.output(Clock,GPIO.HIGH)
            GPIO.output(Clock,GPIO.LOW)
        #no mean ,just delay
        for i in range(0,6):
            GPIO.output(Clock,GPIO.HIGH)
            GPIO.output(Clock,GPIO.LOW)
#       time.sleep(0.0001)
        GPIO.output(CS,GPIO.HIGH)

    return value[1:]


# Sets the desired pin numbering system to BCM
GPIO.setmode(GPIO.BCM)

# Disables warnings in case the RPI.GPIO detects that a pin has been configured
# to something other than the default (input)
GPIO.setwarnings(False)

# Sets all the pins stated above as outputs
chan_list = [ENA,ENB,IN1,IN2,IN3,IN4,Address,CS,Clock]
GPIO.setup(chan_list,GPIO.OUT)
GPIO.setup(DataOut,GPIO.IN,GPIO.PUD_UP)

# creates objects "p1" and "p2", sets ena and enb to 50 Hz
p1 = GPIO.PWM(ENA,50)
p2 = GPIO.PWM(ENB,50)

maximum = 35
oldKp = 0
Kp = 0
Ki = 0
Kd = 0

# Launch.
# Loops through until the track is completed.
while True:
    
# Stops the robot and sleeps for some time, in order to read from the sensors easier
    time.sleep(.1)

# Read values from sensors, store into s1 - s5
    gvalue = readAnalog()
    s1 = gvalue[0]
    s2 = gvalue[1]
    s3 = gvalue[2]
    s4 = gvalue[3]
    s5 = gvalue[4]

# Print statements used for debugging the code and robot position.
    for i in range(1,6):
        print("Sensor %d: ") % i
        print(gvalue[i-1])
    print("\n")
    
    # left2 pos = 2180
    # left pos = 2025
    # middle pos = 1975
    # right pos = 1770
    # right2 pos = 1625
    pos = (0*s1 + 1000*s2 + 2000*s3 + 3000*s4 + 4000*s5) / (s1 + s2 + s3 + s4 + s5)
    
    print("Position: ", pos, "\n")
    
    # value of Kp
    Kp = pos - 1975
    
    # value of Ki
    Ki = Ki + Kp
    
    # value of Kd
    Kd = Kp - oldKp
    
    oldKp = Kp
    
    diffa = Kp/25
    diffb = Kp/25 + Kd/100
    diffc = Kp/25 + Kd/100 + Ki/1000
    
    # diffa for P Controller
    # diffb for PD Controller
    # diffc for PID Controller
    if(diffa > maximum):
        diffa = maximum
    if(diffa < - maximum):
        diffa = - maximum
    if(diffa < 0):
        p1.ChangeDutyCycle(maximum + diffa)
        p2.ChangeDutyCycle(maximum)
    else:
        p1.ChangeDutyCycle(maximum)
        p2.ChangeDutyCycle(maximum - diffa)

# Stops both the PWM outputs
p1.stop()
p2.stop()

# Cleans up the used resources
GPIO.cleanup()