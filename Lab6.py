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

# creates objects "p1" and "p2", sets ena and enb to 500 Hz
p1 = GPIO.PWM(ENA,500)
p2 = GPIO.PWM(ENB,500)

# coefficients to control the power difference
Kp = .25
Ki = .0001
Kd = .001
# inital values to start with
maximum = 35
oldProportional = 0
pos = 2000
p1.start(maximum)
p2.start(maximum)
moveForward()

# Launch.
# Loops through until the track is completed.
while True:
    
# Stops the robot and sleeps for some time, in order to read from the sensors easier
    #time.sleep(.1)

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
    
    if(s1 > 400):
        s1 = 1000
    else:
        s1 = 0

    if(s2 > 400):
        s2 = 1000
    else:
        s2 = 0

    if(s3 > 400):
        s3 = 1000
    else:
        s3 = 0

    if(s4 > 400):
        s4 = 1000
    else:
        s4 = 0

    if(s5 > 400):
        s5 = 1000
    else:
        s5 = 0

    # if all sensors are zero then we don't want to allow a division by 0
    # most likely it's not near a black line and has wondered away.
    # left2 pos = 2500
    # left pos = 2250
    # middle pos = 2000
    # right pos = 1750
    # right2 pos = 1500
    if(s1 or s2 or s3 or s4 or s5):
        pos = (0*s1 + 1000*s2 + 2000*s3 + 3000*s4 + 4000*s5) / (s1 + s2 + s3 + s4 + s5)
    print("Position: ", pos, "\n")
    
    # This process calculates the gain, integral and deriviate for the PID controller
    # value for P part of controller
    proportional = pos - 2000
    print("Proportional: ", proportional, "\n")
    # value for I part of controller
    integral += proportional
    print("Integral: ", integral, "\n")
    # value for D part of controller
    derivative = proportional - oldProportional
    print("Derivative: ", derivative, "\n")
    
    oldProportional = proportional
    
    diffa = Kp*proportional
    #print("diffa: ", diffa, "\n")    
    diffb = Kp*proportional + Kd*derivative
    print("diffb: ", diffb, "\n")    
    diffc = Kp*proportional + Kd*derivative + Ki*integral
    #print("diffc: ", diffc, "\n")
    print("maximum: ", maximum, "\n\n")
    
    # diffa for P Controller
    # diffb for PD Controller
    # diffc for PID Controller
    if(diffb > maximum):
        diffb = maximum
    if(diffb < - maximum):
        diffb = - maximum
    if(diffb < 0):
        p1.ChangeDutyCycle(maximum + diffb)
        p2.ChangeDutyCycle(maximum)
    else:
        p1.ChangeDutyCycle(maximum)
        p2.ChangeDutyCycle(maximum - diffb)
