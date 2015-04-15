import serial
import time
import datetime
import math
from itg3200 import SensorITG3200
from adxl345 import SensorADXL345
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(17,GPIO.IN)
GPIO.setup(22,GPIO.IN)
GPIO.setup(23,GPIO.IN)
GPIO.setup(27,GPIO.IN)

State = "pause"
state = "stop"
state_1="forward"
prev_state = "forward"

turn_dir = "left"
turnNum = 0

prev_input1 = 0
prev_input2 = 0
prev_input3 = 0
prev_input4 = 0
switch=[0,0,0,0]

s1=0
s2=0
s3=0
s4=0
#sf_start_time = datetime.datetime.now()
#sf_time_diff = datetime.datetime.now()
sf_start_time = time.time()
sf_time_diff = time.time()
forward_time=time.time()

prev_gx=0
prev_gy=0
prev_gz=0
offset = 0
scaleW = .512
w = 0;
angle = 0
#last_time = datetime.datetime.now()
#curr_time = datetime.datetime.now()
last_time = time.time()
curr_time = time.time()
time_diff = 0
dt = .2
enable = 0
gz = 0
angle_Initial = 0
flag = 0
short = 0
thre=0.1
sign=0
back = 0
back_flag = 0


CONST_SF_TIME_DELAY = 2.5
CONST_LEFT_90 = 80.0
CONST_RIGHT_90 = -82.0


arduinoSerialPort = serial.Serial('/dev/ttyACM0', 115200)
switch_input = GPIO.input(17)
#sensor = SensorITG3200(1, 0x68) # update with your bus number and address
#sensor.default_init()
sensor1 = SensorADXL345(1, 0x53)
sensor1.default_init()
time.sleep(2)

def getAngle():
    #time.sleep(0.1)
    
   # gx,gy,gz = sensor.read_data()
    time.sleep(0.1)
    ax, ay, az = sensor1.read_data()
    sensor1.standby()
    
    
   # gz/=14.375
	
    ax/=256.0
    ay/=256.0
    az/=256.0
    #print ax, ay, az
    #print gx, gy, gz
    A=math.sqrt( ax*ax + ay*ay+ az*az)
    while (A == 0):
        #sensor.default_init()
        #sensor1.default_init()
        #gx,gy,gz = sensor.read_data()
        time.sleep(0.1)
        ax, ay, az = sensor1.read_data()
        sensor1.standby()    
        #gz/=14.375
	ax/=256.0
        ay/=256.0
        az/=256.0
        A=math.sqrt( ax*ax + ay*ay+ az*az)
        print A
    Ax=ax/A
    Ay=ay/A
    Az=az/A
    Angle_xy = math.atan2(Ay, Ax)    
  #  if abs(gz)>0.5:
        #Angle_xy = Angle_xy + gz*dt*math.pi/180.0
    if (Angle_xy>math.pi):
        Angle_xy = math.pi
    if (Angle_xy<-math.pi):
        Angle_xy = - math.pi
    #print Angle_xy
    return Angle_xy

def Switch():
    switch_input1 = GPIO.input(17)
    switch_input2 = GPIO.input(22)
    switch_input3 = GPIO.input(27)
    switch_input4 = GPIO.input(23)
    #time.sleep(0.1)
    if ((not prev_input1) and switch_input1):
        sw_1 = 1;
	s1=sw_1
    else :
        sw_1 = 0;  
    prev_input1 = switch_input1

    if ((not prev_input2) and switch_input2):
        sw_2 = 1;
	#s1=sw_1
    else :
        sw_2 = 0;  
    prev_input2 = switch_input2

    if ((not prev_input3) and switch_input3):
        sw_3 = 1;
	#s1=sw_1
    else :
        sw_3 = 0;  
    prev_input3 = switch_input3

    if ((not prev_input4) and switch_input4):
        sw_4 = 1;
	#s1=sw_1
    else :
        sw_4 = 0;  
    prev_input4 = switch_input4

    switch = [sw_1,sw_2,sw_3,sw_4]
    return switch
        
while(True):
    
    
    # update time
    curr_time = time.time()
    time_diff=curr_time-last_time

    # Get angle from IMU
    if (time_diff > dt):
        Angle_xy=getAngle()
        last_time = curr_time
        
    # check switch state
    switch = Switch()
    s1=switch[0]
    s2=switch[1] # s2 is the start/pause button

    if (s2==1 and State == "pause"): # initial state is "stop", initial prev_state is "forward",
                                     # When robot is paused, press the start/pause button, state= prev_state
        State="start"
        state = prev_state
    if (s2==1 and State == "start"):
        State = "pause"
        prev_state = state
        state = "stop"
        
    
    if (flag == 0 and State == "start"):  
        state = "initial"
    # initial flag, find the upright direction to start
    if (state == "initial"):
        if (Angle_xy-angle_Initial<-thre):
            arduinoSerialPort.write("3") # turn right
        if (Angle_xy-angle_Initial>thre):
            arduinoSerialPort.write("4") # turn left
        if (-thre<Angle_xy-angle_Initial<thre):
            arduinoSerialPort.write("0")
            state = "forward"
            flag=1
            print "Go"
            time.sleep(1)
    
    
        
       
    print state,state_1,Angle_xy,sign
    
###############################################
	
    if (state == "forward"):
        state_1 =state
        short = 0
		# Once triggering the limit switch, first move backward for a while 
        if (s1 == 1 and back == 0):
	    state = "backward"
            s1=0 # reset limit switch
            break
        
			
		# Maintain the direction when robot move upward
        if(2*thre < math.fabs (Angle_xy) < 0.5 * math.pi):
            if (Angle_xy > 0):
                state_1="left"
                    
            else:
                    
                state_1="right"
                    
        
		# Maintain the direction when robot move downward
        if( 0.5*math.pi<math.fabs(Angle_xy)<math.pi-2*thre):
            sign = math.copysign(1.0, Angle_xy)
            if (0<Angle_xy-sign*math.pi):
                state_1="left"
            else:
                state_1="right"
				
		#  Deviation to right, need to move left
        if (state_1 == "left"):
            arduinoSerialPort.write("6") # forward left
            if (0 < Angle_xy < thre or 0 < math.pi-math.fabs(Angle_xy) < thre ):
                state_1 = "forward"
		# Deviation to left, need to move right
        if (state_1 == "right"):
            arduinoSerialPort.write("5") # forward right
            if (-thre < Angle_xy < 0 or 0<math.pi - math.fabs(Angle_xy) < thre):
                state_1 = "forward"
		# Go straight
        if (state_1 == "forward"):
            arduinoSerialPort.write("1")
			
###############################################

    if (state == "short_forward"):
        
        arduinoSerialPort.write("1")
        short = 1
        back_flag = 0    # reset backward flag
	back = 0
        
        sf_time_diff = curr_time - sf_start_time
        
        if(sf_time_diff > CONST_SF_TIME_DELAY):
            if(turn_dir == "left"):
                
                state = "turn_left"
                turn_dir = "right"
            else: 
                state = "turn_right"
                turn_dir = "left"
				
#############################################

    if (state == "turn_left"):
        arduinoSerialPort.write("4")
        if (math.fabs(Angle_xy)>(0.5*math.pi+0.5 *math.pi*short)-thre):
            
            if (turnNum == 1):
                turnNum = 0
                state = "forward"
            else:
                turnNum = 1
                sf_start_time = curr_time
                state = "short_forward"

##############################################

    if (state == "turn_right"):
        arduinoSerialPort.write("3")
        if (math.fabs(Angle_xy)<(0.5*math.pi-0.5 *math.pi*short)+thre):
            
            if (turnNum == 1):
                turnNum = 0
                state = "forward"
		#state = "stop"
            else:
                turnNum = 1
                sf_start_time = curr_time
                state = "short_forward" 
				
##############################################

    if (state=="stop"):
	
	arduinoSerialPort.write("0")    

##############################################
	
    if (state == "backward"):
	if (back_flag == 0):
            arduinoSerialPort.write("2")
            back_start = curr_time
            back_flag = 1
        
        back_tdiff = curr_time - back_start 
        if (back_tdiff>1):
            back = 1
            arduinoSerialPort.write("0")
            if (turn_dir == "left"):
                state = "turn_left"                        
	    elif(turn_dir == "right"):
                state = "turn_right"
		
		
		
        
    


