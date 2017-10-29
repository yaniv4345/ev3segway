#!/usr/bin/env python3
import time
import sys
from collections import deque
import ev3dev.ev3 as ev3
import parameters
import importlib
import socket
import select
import threading
from itertools import cycle
from multiprocessing import Queue

## Socket setup
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
address = ('127.0.0.1', 12000)
sock.bind(address)
sock.listen(16)
command = 0

## Thread setup
exitFlag=0
class myThread(threading.Thread):
    def __init__(self, threadID, name, q):
        super(myThread, self).__init__()
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.q = q
        self._stop_event = threading.Event()
    def stop(self):
        self._stop_event.set()
    def stopped(self):
        return self._stop_event.is_set()
    def run(self):
        #print("Starting %s" % self.name)
        process_data(self.name, self.q)
        #print("Exiting %s" % self.name)

def process_data(threadName, q):
  while not exitFlag:
    connection, client_address = sock.accept()
    try:
      data = connection.recv(6)
      if data:
        #print("%s data: %s" % (threadName, data))
        queueLock.acquire(timeout=1)
        q.put(data)
        queueLock.release()
        connection.sendall(data)
    finally:
      connection.close()
  connection.close()

## Settings for queue and thread
queueLock = threading.Lock()
workQueue = Queue(10)
threadID=1
thread = myThread(threadID, 'socket_listen', workQueue)
thread.daemon = True
thread.start()


########################################################################
## File I/O functions
########################################################################

# Function for fast reading from sensor files
def FastRead(infile):
    infile.seek(0)    
    return(int(infile.read().decode().strip()))

# Function for fast writing to motor files    
def FastWrite(outfile,value):
    outfile.truncate(0)
    outfile.write(str(int(value)))
    outfile.flush()    

# Debug print (https://stackoverflow.com/a/14981125)
def eprint(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)

# Function to set the duty cycle of the motors
def SetDuty(motorDutyFileHandle, duty):
    # Compansate for nominal voltage and round the input
    dutyInt = int(round(duty*voltageCompensation))
    # Add or subtract offset and clamp the value between -100 and 100
    if dutyInt > 0:
        dutyInt = min(100, dutyInt + frictionOffset)
    elif dutyInt < 0:
        dutyInt = max(-100, dutyInt - frictionOffset)
    # Apply the signal to the motor
    FastWrite(motorDutyFileHandle, dutyInt)

########################################################################
## One-time Hardware setup
########################################################################

# EV3 Brick
powerSupply = ev3.PowerSupply()
buttons = ev3.Button()

# EV3 Leds
leftLed=ev3.Leds.LEFT
rightLed=ev3.Leds.RIGHT

# Gyro Sensor setup
gyroSensor          = ev3.GyroSensor()
gyroSensor.mode     = gyroSensor.MODE_GYRO_RATE
gyroSensorValueRaw  = open(gyroSensor._path + "/value0", "rb")   

# Configure the motors
motorLeft  = ev3.LargeMotor('outD')
motorRight = ev3.LargeMotor('outA')    

#############################################################################
## Outer Loop (Uses Touch Sensor to easily start and stop "Balancing Loop")
#############################################################################

#while not buttons.any():

########################################################################
## Hardware (Re-)Config
########################################################################

# Reset the motors
motorLeft.reset()                   # Reset the encoder
motorRight.reset()
motorLeft.run_direct()              # Set to run direct mode
motorRight.run_direct() 

# Open sensor files for (fast) reading
motorEncoderLeft    = open(motorLeft._path + "/position", "rb")    
motorEncoderRight   = open(motorRight._path + "/position", "rb")           

# Open motor files for (fast) writing
motorDutyCycleLeft = open(motorLeft._path + "/duty_cycle_sp", "w")
motorDutyCycleRight= open(motorRight._path + "/duty_cycle_sp", "w")

########################################################################
## Definitions and Initialization variables
########################################################################    
		
#Math constants
radiansPerDegree               = 3.14159/180                                   # The number of radians in a degree.

#Platform specific constants and conversions
degPerSecondPerRawGyroUnit     = 1                                             # For the LEGO EV3 Gyro in Rate mode, 1 unit = 1 deg/s
radiansPerSecondPerRawGyroUnit = degPerSecondPerRawGyroUnit*radiansPerDegree   # Express the above as the rate in rad/s per gyro unit
degPerRawMotorUnit             = 1                                             # For the LEGO EV3 Large Motor 1 unit = 1 deg
radiansPerRawMotorUnit         = degPerRawMotorUnit*radiansPerDegree           # Express the above as the angle in rad per motor unit
RPMperPerPercentSpeed          = 1.7                                           # On the EV3, "1% speed" corresponds to 1.7 RPM (if speed control were enabled)
degPerSecPerPercentSpeed       = RPMperPerPercentSpeed*360/60                  # Convert this number to the speed in deg/s per "percent speed"
radPerSecPerPercentSpeed       = degPerSecPerPercentSpeed * radiansPerDegree   # Convert this number to the speed in rad/s per "percent speed"

# Variables representing physical signals (more info on these in the docs)
motorAngleRaw              = 0 # The angle of "the motor", measured in raw units (degrees for the EV3). We will take the average of both motor positions as "the motor" angle, wich is essentially how far the middle of the robot has traveled.
motorAngle                 = 0 # The angle of the motor, converted to radians (2*pi radians equals 360 degrees).
motorAngleReference        = 0 # The reference angle of the motor. The robot will attempt to drive forward or backward, such that its measured position equals this reference (or close enough).
motorAngleError            = 0 # The error: the deviation of the measured motor angle from the reference. The robot attempts to make this zero, by driving toward the reference.
motorAngleErrorAccumulated = 0 # We add up all of the motor angle error in time. If this value gets out of hand, we can use it to drive the robot back to the reference position a bit quicker.
motorAngularSpeed          = 0 # The motor speed, estimated by how far the motor has turned in a given amount of time
motorAngularSpeedReference = 0 # The reference speed during manouvers: how fast we would like to drive, measured in radians per second.
motorAngularSpeedError     = 0 # The error: the deviation of the motor speed from the reference speed.
motorDutyCycle             = 0 # The 'voltage' signal we send to the motor. We calulate a new value each time, just right to keep the robot upright.
gyroRateRaw                = 0 # The raw value from the gyro sensor in rate mode.
gyroRate                   = 0 # The angular rate of the robot (how fast it is falling forward or backward), measured in radians per second.
gyroEstimatedAngle         = 0 # The gyro doesn't measure the angle of the robot, but we can estimate this angle by keeping track of the gyroRate value in time
gyroOffset                 = 0 # Over time, the gyro rate value can drift. This causes the sensor to think it is moving even when it is perfectly still. We keep track of this offset.

eprint("Hold robot upright. Wait for beep to start. Or any button to exit.")

########################################################################
## Read/reload Parameters
########################################################################    

# Reload parameters class
importlib.reload(parameters)

powerParameters = parameters.Power()
gains           = parameters.Gains()
timing          = parameters.Timing()

# Read battery voltage
voltageIdle = powerSupply.measured_volts
voltageCompensation = powerParameters.voltageNominal/voltageIdle

# Offset to limit friction deadlock
frictionOffset = int(round(powerParameters.frictionOffsetNominal*voltageCompensation))

#Timing settings for the program
loopTimeSec             = timing.loopTimeMiliSec/1000  # Time of each loop, measured in seconds.
loopCount               = 0                     # Loop counter, starting at 0

# A deque (a fifo array) which we'll use to keep track of previous motor positions, which we can use to calculate the rate of change (speed)
motorAngleHistory = deque([0],timing.motorAngleHistoryLength)

# The rate at which we'll update the gyro offset (precise definition given in docs)
gyroDriftCompensationRate      = timing.gyroDriftCompensationFactor*loopTimeSec*radiansPerSecondPerRawGyroUnit


########################################################################
## Calibrate Gyro
########################################################################    

ev3.Leds.set_color(leftLed, (1,0))
ev3.Leds.set_color(rightLed, (1,0))
eprint("-----------------------------------")      
eprint("Calibrating...")

#As you hold the robot still, determine the average sensor value of 100 samples
gyroRateCalibrateCount = 100
for i in range(gyroRateCalibrateCount):
  gyroOffset = gyroOffset + FastRead(gyroSensorValueRaw)
  time.sleep(0.01)
gyroOffset = gyroOffset/gyroRateCalibrateCount 

# Print the result   
eprint("GyroOffset: ",gyroOffset)   
eprint("-----------------------------------")    
eprint("GO!") 
eprint("-----------------------------------") 
ev3.Sound.beep()
########################################################################
## Balancing Loop
########################################################################    

# Remember start time
tProgramStart = time.time()

    # Keep looping until Touch Sensor is pressed again   
while not buttons.any():
        ## Try get Queue
        if not workQueue.empty():
          queueLock.acquire(timeout=1)
          command = workQueue.get()
          queueLock.release()
          command=command.decode("utf-8")
          #print("This is command = %s" % command)
        lastCommand = command

        ###############################################################
        ##  Loop info
        ###############################################################
        loopCount = loopCount + 1
        tLoopStart = time.time()  

        ###############################################################
        ##
        ##  Driving and Steering. Modify this section as you like to
        ##  make your segway go anywhere!
        ##
        ##  To begin, uncomment one of the examples below, and modify 
        ##  from there
        ##
        ###############################################################
    
        # Example 1: Doing nothing: just balance in place:
        speed    = 0
        steering = 0

        # Example 2: Control speed and steering based on the IR Remote
        speed_max = 20
        steer_max_right = 8

        if(command == 'UP'):
            speed    =  18
            steering =  0
        elif (command == 'LEFT'):
            speed    =  0
            steering = 9
        elif (command == 'RIGHT'):
            speed    =  0
            steering = -9            
        elif (command == 'DOWN'):
            speed    = -18
            steering = 0
        elif (command == 'CLR'):
            speed = 0
            steering = 0
        else:
            speed    = 0
            steering = 0
        


###############################################################
        ##  Reading the Gyro.
        ###############################################################
        gyroRateRaw = FastRead( gyroSensorValueRaw)
        gyroRate = (gyroRateRaw - gyroOffset)*radiansPerSecondPerRawGyroUnit

        ###############################################################
        ##  Reading the Motor Position
        ###############################################################

        motorAngleRaw = (FastRead(motorEncoderLeft) + FastRead(motorEncoderRight))/2
        motorAngle = motorAngleRaw*radiansPerRawMotorUnit

        motorAngularSpeedReference = speed*radPerSecPerPercentSpeed
        motorAngleReference = motorAngleReference + motorAngularSpeedReference*loopTimeSec

        motorAngleError = motorAngle - motorAngleReference    
        
        ###############################################################
        ##  Computing Motor Speed
        ###############################################################
        
        motorAngularSpeed = (motorAngle - motorAngleHistory[0])/(timing.motorAngleHistoryLength*loopTimeSec)
        motorAngularSpeedError = motorAngularSpeed# - motorAngularSpeedReference # Uncommenting this leads to a raher abrubt change in speed when using the remote. So I'll leave it commented until I add some code that gradually increases ths reference when a button is pressed or depressed 
        motorAngleHistory.append(motorAngle)

        ###############################################################
        ##  Computing the motor duty cycle value
        ###############################################################

        motorDutyCycle =( gains.GyroAngle  * gyroEstimatedAngle
                        + gains.GyroRate   * gyroRate
                        + gains.MotorAngle * motorAngleError
                        + gains.MotorAngularSpeed * motorAngularSpeedError
                        + gains.MotorAngleErrorAccumulated * motorAngleErrorAccumulated)    
        
        ###############################################################
        ##  Apply the signal to the motor, and add steering
        ###############################################################

        SetDuty(motorDutyCycleRight, motorDutyCycle + steering)
        SetDuty(motorDutyCycleLeft , motorDutyCycle - steering)

        ###############################################################
        ##  Update angle estimate and Gyro Offset Estimate
        ###############################################################

        gyroEstimatedAngle = gyroEstimatedAngle + gyroRate*loopTimeSec
        gyroOffset = (1-gyroDriftCompensationRate)*gyroOffset+gyroDriftCompensationRate*gyroRateRaw

        ###############################################################
        ##  Update Accumulated Motor Error
        ###############################################################

        motorAngleErrorAccumulated = motorAngleErrorAccumulated + motorAngleError*loopTimeSec

        ###############################################################
        ##  Read the touch sensor (the kill switch)
        ###############################################################

        #touchSensorPressed = FastRead(touchSensorValueRaw) 

        ###############################################################
        ##  Busy wait for the loop to complete
        ###############################################################
    
        while(time.time() - tLoopStart <  loopTimeSec):
            time.sleep(0.0001) 
        
    ########################################################################
    ##
    ## Closing down & Cleaning up
    ##
    ######################################################################## 

# Loop end time, for stats
tProgramEnd = time.time()  

# Turn off the motors    
FastWrite(motorDutyCycleLeft ,0)
FastWrite(motorDutyCycleRight,0)

# Calculate loop time
tLoop = (tProgramEnd - tProgramStart)/loopCount
total = tProgramEnd - tProgramStart
eprint("Loop time:", tLoop*1000,"ms")
eprint("Total time:", total,"sec")

ev3.Leds.set_color(leftLed, (0,1))
ev3.Leds.set_color(rightLed, (0,1))
# Print a stop message
eprint("-----------------------------------")   
eprint("STOP")
eprint("-----------------------------------")     
