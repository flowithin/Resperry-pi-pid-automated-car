import time
import numpy as np
import RPi.GPIO as GPIO
from gpiozero_extended import Motor, PID
#try this code on the car
GPIO.setmode(GPIO.BCM)
class EchoSensor:
    def init(self, trigPin = 15, echoPin = 13):
        self.trigPin = trigPin
        self.echoPin = echoPin
        t0 = time.perf_counter()
        while(t1 < t0 + 3):
            t1 = time.perf_counter()
            if self.get_dist() < 3:
                self.hand = True
                break
            else:
                self.hand = False
    def get_dist(self):
        while True:
            # setup the trigger pin
            GPIO.output(self.trigPin,0)
            time.sleep(2E-6)
            GPIO.output(self.trigPin,1)
            time.sleep(10E-6)
            GPIO.output(self.trigPin,0)
            # measure with the echo pin
            while GPIO.input(self.echoPin)==0:
                pass
            echoStartTime = time.time()
            while GPIO.input(self.echoPin)==1:
                pass
            echoStopTime = time.time()
            pingTravelTime = echoStopTime - echoStartTime
            # speed of sound is 343 m/s, 34300 cm/s at 20 degree celcius
            echoTravelDistance = pingTravelTime*34300
            print(round(self.distance,1),' cm')
            time.sleep(0.2)
            return echoTravelDistance/2
#put the configuration pins in here
echoL = EchoSensor(trigPin =7, echoPin = 8) 
echoR = EchoSensor(trigPin =6, echoPin = 5 )
maxDist_diff = 40
def turnLeft():
    tstart = time.time()
    tstop = tstart
    while tstop < tstart + 3:
        tstop = time.time()
        mymotorL.set_output(0)
        mymotorR.set_output(1)

def turnRight():
    tstart = time.time()
    tstop = tstart
    while tstop < tstart + 3:
        tstop = time.time()
        mymotorL.set_output(1)
        mymotorR.set_output(0)

# Setting general parameters
tstop = 2  # Execution duration (s)
tsample = 0.01  # Sampling period (s)
wsp = 1  # Motor speed set point (rad/s)
wmax = 100  # Maximum motor speed (rad/s) for scaling
tau = 0.1  # Speed low-pass filter response time (s)
distance = 0
ena = 16
in1 = 20
in2 = 21
enb = 13
in3 = 26
in4 = 19
# Creating PID controller object
kp = 0.15
ki = 0.35
kd = 0.01
taupid = 0.01
pid = PID(tsample, kp, ki, kd, umin=0, umax=1, tau=taupid)  # Ensuring umax is set to 1

# Creating motor object using GPIO pins 16, 17, and 18
#without encoder
mymotorL = Motor(enable1=enb, pwm1=in3, pwm2=in4 )
mymotorR = Motor(enable1=ena, pwm1=in1, pwm2=in2 )
#mymotor.reset_angle()

# Initializing previous and current values
ucurr = 0  # x[n] (step input)
Dist_diff_prev = 0  # y[n-1]
Dist_diff_curr = 0  # y[n]

# Initializing variables and starting clock
thetaprev = 0
tprev = 0
tcurr = 0
tstart = time.perf_counter()

# Running execution loop
print('Running code for', tstop, 'seconds ...')
try:
    while True:
        #get distance of the sensor
        # Pausing for `tsample` to give CPU time to process encoder signal
        distL = echoL.get_dist()
        distR = echoR.get_dist()
        if(distL > 30 and distR > 30):
            if(echoL.hand and echoR.hand):
                turnLeft()
            else:
                turnRight()
        time.sleep(tsample)
        # Getting current time (s)
        tcurr = time.perf_counter() - tstart
        # Getting motor shaft angular position: I/O (data in)
        #thetacurr = mymotor.get_angle()
        # Calculating motor speed (rad/s)
        #wcurr = np.pi / 180 * (thetacurr - thetaprev) / (tcurr - tprev)
        # Filtering motor speed signal
        #wfcurr = tau / (tau + tsample) * wfprev + tsample / (tau + tsample) * wcurr
        Dist_diff_curr = distL - distR 
        Dist_diff_prev = Dist_diff_curr
        # Scaling speed measurement to [0, 1]
        #wfcurr_scaled = wfcurr / wmax
        #wsp_scaled = wsp / wmax
        # Calculating closed-loop output
        ucurr = pid.control(0, Dist_diff_curr)
        ucurr_scaled = ucurr / maxDist_diff
        # Assigning motor output: I/O (data out)
        mymotorL.set_output(ucurr_scaled + wsp)
        mymotorR.set_output(-ucurr_scaled + wsp)
        # Updating previous values
        #thetaprev = thetacurr
        tprev = tcurr
except Exception as e:
    print(f"An error occurred: {e}")
finally:
    # Stopping motor and releasing GPIO pins
    mymotorR.set_output(0, brake=True)
    mymotorL.set_output(0, brake=True)
    del mymotorR
    del mymotorL
    print('Done.')
