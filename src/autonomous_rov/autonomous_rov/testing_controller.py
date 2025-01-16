#!/usr/bin/env python3
import math
from os import kill
import string
import numpy as np
from yaml import FlowEntryToken

import rospy
import tf
from std_msgs.msg import Int16
from std_msgs.msg import Float64
from std_msgs.msg import Float64
from std_msgs.msg import Empty
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from mavros_msgs.msg import OverrideRCIn
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Imu
#from waterlinked_a50_ros_driver.msg import DVL
#from waterlinked_a50_ros_driver.msg import DVLBeam
from sensor_msgs.msg import FluidPressure
from sensor_msgs.msg import LaserScan
from mavros_msgs.srv import CommandLong
from geometry_msgs.msg import Twist

import time
import sys
import argparse

# ---------- Global Variables ---------------

#----------global variables for control----------
global int_error #for integral 
global dt 
global prev_error

prev_error = [0,0,0]    # pitch yaw depth
dt = [0,0,0]            # pitch yaw depth
int_error = [0,0,0]     # pitch yaw depth 

global kp
global ki
global kd
kp = [0,0, 20]           # pitch yaw depth
kd = [0,0, 0.2]        # pitch yaw depth
ki = [0,0, 0.4]          # pitch yaw depth

global control_state #the state we want our system to go
global control_mode # the mode at which we want to control
global floatability

floatability = 15/4
control_state = [0,3.14/2,0.5]   # pitch yaw depth
control_mode = [3,3,3]      #0 for not controlling, 1 for P, 2 for PI, 3 for PID


### yaw control
global yaw_control_enabled, yaw_target, yaw_int_error, yaw_prev_error, yaw_last_time, control_signal_mapped
global current_angle
control_signal_mapped = 1500

yaw_control_enabled = False
yaw_target = 0.0
yaw_int_error = 0.0
yaw_prev_error = 0.0
yaw_last_time = None

KP_YAW = 0.1
KI_YAW = 0.1
KD_YAW = 0.0


# controller
global expoloration_speed, running_speed
expoloration_speed = 1530
running_speed = 1540

global control_depth, control_yaw , go_forward, explore , obstacle_avoidance
control_depth = True
control_yaw = False
obstacle_avoidance = True

go_forward = True
explore = False



# Alpha-beta filter parameters
# ALPHA = 0.1  # Tune this value
# BETA = 0.005
def trajectory_gen( t_final=5, mission_time=30, state_init = 0, state_final=1, dt=1/60):
    '''
    t_final: the time at which we want our robot to reach final state
    mission_time: the time upto which the robot should work
    state_init : initial state of the robot
    state_final: final state after control
    dt: depends on the sensor reading

    '''
    t = np.linspace(0, t_final, int(t_final/dt))
    t_steady = np.linspace(t_final, mission_time, int((mission_time-t_final)/dt))
    
    a2 = 3*(state_final- state_init)/(t_final**2)
    a3 = -2*(state_final-state_init)/(t_final**3)
    
    traj = state_init + (a2 * (t*2)) + (a3 *(t*3)) 
    
    derivative_traj = state_init + 2*(a2*t) + 3*(a3 (t*2))
    
    N = t_steady.shape[0] 

    state_steady = state_final* np.ones((1, N))
    state_steady = state_steady.squeeze()
    
    steady_state_der = state_final* np.zeros((1, N))
    steady_state_der = steady_state_der.squeeze()
 
    return np.concatenate((traj, state_steady), axis =0), np.concatenate((derivative_traj, steady_state_der), axis =0) 


def generate_trajectories():
    global depth_p0
    global angle_pitch_a0
    global angle_yaw_a0

    global control_state
    t_final = 5
    mission_time = 600
    traj_pitch,der_pitch = trajectory_gen(t_final=t_final, mission_time=mission_time, state_init=angle_pitch_a0, state_final=control_state[0])  
    traj_yaw, der_yaw = trajectory_gen(t_final=t_final, mission_time=mission_time, state_init=angle_yaw_a0, state_final=control_state[1])
    traj_depth, der_depth = trajectory_gen(t_final=t_final, mission_time=mission_time, state_init=depth_p0, state_final=control_state[2])
    
    global traj
    global der_traj
    traj = [traj_pitch, traj_yaw, traj_depth]
    der_traj = [der_pitch, der_yaw, der_depth]

    rospy.loginfo(f"Trajectory generated successfully")
    rospy.loginfo(f"initial_states:{angle_pitch_a0, angle_yaw_a0, depth_p0}")
    rospy.loginfo(f"final_states: {control_state}")
    rospy.loginfo(f"t_final: {t_final}, mission_time:{mission_time}")

    

#---previous global variables that were given---
global arming
global set_mode

set_mode = [0]*3
set_mode[0] = True   # Mode manual
set_mode[1] = False  # Mode automatic without correction
set_mode[2] = False  # Mode with correction

#Conditions
init_a0 = True
init_p0 = True
arming = False

angle_wrt_startup = [0]*3
angle_roll_a0 = 0.0
angle_pitch_a0 = 0.0
angle_yaw_a0 = 0.0
depth_wrt_startup = 0
depth_p0 = 0

enable_depth = False 
enable_ping = True 
pinger_confidence = 0
pinger_distance = 0

Vmax_mot = 1900
Vmin_mot = 1100




### trajectory
global counter
counter = 0 #to index the trajectory 
#### mapping of the thrusters. Use this before sending the singlas to the motors
def get_mapping(x, battery = 12):
    if battery==16:
        #pos
        m1 = 6.660955443377915 
        c1 = 1563.8915056025257
        #neg
        m2 = 8.556474591368378 
        c2 = 1437.5105488646345
    
    elif battery==14:
        # pos--
        m1,c1 = 7.664152101775231, 1566.2199559949108
        # neg
        m2,c2 =  9.86611712277398, 1435.8849828900563

    elif battery==12:
        # pos--
        m1,c1 = 9.26409545359083, 1530.0
        m2,c2 = 11.876213274212256, 1470.0        # neg--

    if -0.00002<x<0.0002:
        y = 1500
    elif x>0:
        y = m1*x+c1
    elif x<0:
        y = m2*x + c2

    if y>=1900:
        y = 1900
    elif y<=1100:
        y=1100

    return int(y)



# Linear/angular velocity 
u = 0               # linear surge velocity 
v = 0               # linear sway velocity
w = 0               # linear heave velocity 

p = 0               # angular roll velocity
q = 0               # angular pitch velocity 
r = 0               # angular heave velocity 

# ---------- Functions---------------
#alpha beta filtering
class AlphaBetaFilter:
    def _init_(self,initial_depth,a=0.1,b=0.005,dt=1/58):
        self.a = a
        self.b = b
        self.dt = dt
        self.vk=0
        self.xk=0
        self.xk_0=initial_depth
        self.vk_0=0

    def filter_step(self, xm):
        self.xk = self.xk_0 + self.vk_0*self.dt
        self.vk = self.vk_0

        self.rk = xm - self.xk
        self.xk +=self.a*self.rk
        self.vk += (self.b*self.rk)/self.dt

        self.xk_0 = self.xk
        self.vk_0 = self.vk 
        
        return self.vk_0

def PID_control(kp=kp, ki=ki, kd=kd, state=[5,5,5], destination=[3,3,3], floatability=floatability):
    global int_error
    global dt
    global counter
    global der_traj
    global alpha_beta_filter
    global enable
    global imu_data
    
    #calculating the error in states
    # destination[2]=0.5
    error = np.subtract(np.array(destination), np.array(state))
    P = np.multiply(kp, error)
    # print(f"error{error}")
    # print(f"des:{state}")
    #
    int_error+=np.multiply(error, dt)
    I = np.multiply(ki,int_error)
    # print(int_error)
    
    destination_der = [der_traj[0][counter], der_traj[1][counter], der_traj[2][counter]]
    
    der_depth = alpha_beta_filter.filter_step(state[2])
    state_der = [imu_data['pitch_rate'], imu_data['yaw_rate'], der_depth]
    # print(state_der)
    der_error = np.subtract(destination_der, state_der)
    # print(der_error)
    D = np.multiply(kd, der_error)

    DD =-( P+I+D +[0,0,floatability])
    # AA = np.multiply(DD,np.array([0,1,0]))
    return DD 


def joyCallback(data):
    global arming
    global set_mode
    global init_a0
    global init_p0
    global Sum_Errors_Vel
    global Sum_Errors_angle_yaw
    global Sum_Errors_depth
    global yaw_target
    # Joystick buttons
    btn_arm = data.buttons[7]  # Start button
    btn_disarm = data.buttons[6]  # Back button
    btn_manual_mode = data.buttons[3]  # Y button
    btn_automatic_mode = data.buttons[2]  # X button
    btn_corrected_mode = data.buttons[0]  # A button

    # Disarming when Back button is pressed
    if (btn_disarm == 1 and arming == True):
        arming = False
        armDisarm(arming)

    # Arming when Start button is pressed
    if (btn_arm == 1 and arming == False):
        arming = True
        armDisarm(arming)

    # Switch manual, auto and correction mode
    if (btn_manual_mode and not set_mode[0]):
        set_mode[0] = True
        set_mode[1] = False
        set_mode[2] = False		
        rospy.loginfo("Mode manual")
    if (btn_automatic_mode and not set_mode[1]):
        set_mode[0] = False
        set_mode[1] = True
        set_mode[2] = False		
        rospy.loginfo("Mode automatic")
    if (btn_corrected_mode and not set_mode[2]):
        init_a0 = True
        init_p0 = True
        
        set_mode[0] = False
        set_mode[1] = False
        set_mode[2] = True
        rospy.loginfo("Mode correction")
        generate_trajectories()
        enable_yaw_control(yaw_target)

def armDisarm(armed):
    # This functions sends a long command service with 400 code to arm or disarm motors
    if (armed):
        rospy.wait_for_service('mavros/cmd/command')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/command', CommandLong)
            armService(0, 400, 0, 1, 0, 0, 0, 0, 0, 0)
            rospy.loginfo("Arming Succeeded")
        except rospy.ServiceException as e:
            rospy.loginfo("Except arming")
    else:
        rospy.wait_for_service('mavros/cmd/command')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/command', CommandLong)
            armService(0, 400, 0, 0, 0, 0, 0, 0, 0, 0)
            rospy.loginfo("Disarming Succeeded")
        except rospy.ServiceException as e:
            rospy.loginfo("Except disarming")


def velCallback(cmd_vel):
    global set_mode

    # Only continue if manual_mode is enabled
    if (set_mode[1] or set_mode[2]):
        return

    # Extract cmd_vel message
    roll_left_right 	= mapValueScalSat(cmd_vel.angular.x)
    yaw_left_right 		= mapValueScalSat(-cmd_vel.angular.z)
    ascend_descend 		= mapValueScalSat(cmd_vel.linear.z)
    forward_reverse 	= mapValueScalSat(cmd_vel.linear.x)
    lateral_left_right 	= mapValueScalSat(-cmd_vel.linear.y) #ask V about negative
    pitch_left_right 	= mapValueScalSat(cmd_vel.angular.y)

    setOverrideRCIN(pitch_left_right, roll_left_right, ascend_descend, yaw_left_right, forward_reverse, lateral_left_right)

def pingerCallback(data):
    global pinger_confidence
    global pinger_distance
    global go_forward, explore, control_yaw , obstacle_avoidance , current_angle
    pinger_distance = data.data[0]
    pinger_confidence = data.data[1]
    dead_zone = 500
    obstacle_zone = 700

    print("pinger distance", pinger_distance, "pinger confidence", pinger_confidence)
    if obstacle_avoidance:
        if dead_zone < pinger_distance < obstacle_zone:
            print("I am avoiding the obstacle")
            go_forward=False
            explore=True
            control_yaw= False
        else:
            go_forward=True
            explore=False
            control_yaw=True
            enable_yaw_control(current_angle)

def enable_yaw_control(target_yaw):
    global yaw_control_enabled, yaw_target, yaw_int_error, yaw_prev_error, yaw_last_time
    yaw_control_enabled = True
    yaw_target = target_yaw
    yaw_int_error = 0.0
    yaw_prev_error = 0.0
    yaw_last_time = rospy.get_time()
def OdoCallback(data):
    global angle_roll_a0
    global angle_pitch_a0
    global angle_yaw_a0
    global angle_wrt_startup
    global init_a0
    global p
    global q
    global r
    global arming
    global set_mode
    global imu_data #to store angles and their rate
    global yaw_control_enabled, yaw_target, yaw_int_error, yaw_prev_error, yaw_last_time, control_yaw
    global control_signal_mapped
    global current_angle
    orientation = data.orientation
    angular_velocity = data.angular_velocity

    # extraction of yaw angle
    q = [orientation.x, orientation.y, orientation.z, orientation.w]
    euler = tf.transformations.euler_from_quaternion(q)
    angle_roll = euler[0]
    angle_pitch = euler[1]
    angle_yaw = euler[2]
    
    

    if (init_a0):
        # at 1st execution, init
        angle_roll_a0 = angle_roll
        angle_pitch_a0 = angle_pitch
        angle_yaw_a0 = angle_yaw
        init_a0 = False

    angle_wrt_startup[0] = ((angle_roll - angle_roll_a0 + 3.0*math.pi)%(2.0*math.pi) - math.pi) * 180/math.pi #ask V
    angle_wrt_startup[1] = ((angle_pitch - angle_pitch_a0 + 3.0*math.pi)%(2.0*math.pi) - math.pi) * 180/math.pi
    angle_wrt_startup[2] = ((angle_yaw - angle_yaw_a0 + 3.0*math.pi)%(2.0*math.pi) - math.pi) * 180/math.pi
    current_angle= angle_wrt_startup[2]


    angle = Twist()
    angle.angular.x = angle_wrt_startup[0]
    angle.angular.y = angle_wrt_startup[1]
    angle.angular.z = angle_wrt_startup[2]

    pub_angle_degre.publish(angle)

    # Extraction of angular velocity
    p = angular_velocity.x
    q = angular_velocity.y
    r = angular_velocity.z
    
    imu_data = {
        'roll':angle_roll,
        'pitch':angle_pitch,
        'yaw':angle_yaw,
        'roll_rate':p,
        'pitch_rate':q,
        'yaw_rate':r
    }
    if yaw_control_enabled:
        current_time = rospy.get_time()
        dt = current_time - yaw_last_time if yaw_last_time is not None else 0
        error = yaw_target - angle_wrt_startup[2]

        # Integral term
        yaw_int_error += error * dt

        derivative = (error - yaw_prev_error) / dt if dt > 0 else 0
        # filtered_derivative = ALPHA * derivative + (1 - ALPHA) * (BETA / dt)

        control_signal = KP_YAW * error + KI_YAW * yaw_int_error
        # print(control_signal)
        control_signal_mapped = get_mapping(control_signal)
        if not control_yaw:
            control_signal_mapped = 1500
        # if control_yaw:
            # print(get_mapping(0.000001) , control_signal)
            # print("dt", dt, "error", error, "target", yaw_target, "current angle", angle_wrt_startup[2],"control_signal_mapped", control_signal_mapped)

        # print(control_signal_mapped)
        yaw_prev_error = error
        yaw_last_time = current_time

    # Only continue if manual_mode is disabled
    if (set_mode[0]):
        return

    
def DvlCallback(data):
    global set_mode
    global u
    global v
    global w

    u = data.velocity.x  # Linear surge velocity
    v = data.velocity.y  # Linear sway velocity
    w = data.velocity.z  # Linear heave velocity

    Vel = Twist()
    Vel.linear.x = u
    Vel.linear.y = v
    Vel.linear.z = w
    pub_linear_velocity.publish(Vel)

def go_forward_or_stop():
    global go_forward, running_speed
    if go_forward:
        return running_speed
    else:
        return 1500

def exploraion():
    global explore, expoloration_speed
    if explore:
        return expoloration_speed
    else:
        return 1500
def PressureCallback(data):
    global depth_p0
    global depth_wrt_startup
    global init_p0


    rho = 1000.0 # 1025.0 for sea water
    g = 9.80665

    # Only continue if manual_mode is disabled
    if (set_mode[0]):
        return
    elif (set_mode[1]):
        # Only continue if automatic_mode is enabled
        # Define an arbitrary velocity command and observe robot's velocity
        setOverrideRCIN(1500, 1500, 1500, 1500, 1500, 1500)
        return

    pressure = data.fluid_pressure

    if (init_p0):
        # 1st execution, init
        depth_p0 = (pressure - 101300)/(rho*g)
        global alpha_beta_filter
        alpha_beta_filter = AlphaBetaFilter(initial_depth=depth_p0) #filtering
        init_p0 = False

    depth_wrt_startup = (pressure - 101300)/(rho*g) - depth_p0

    # setup depth servo control here
    # ...

    #params for control
    global control_state #the state we want our system to go
    global control_mode # the mode at which we want to control
    global floatability
    global kp
    global ki
    global kd
    global counter
    global imu_data
    global traj
    global control_depth
    current_state = [imu_data['pitch'], imu_data['yaw'], depth_wrt_startup]

    if sum(control_mode)>0 and set_mode[2] and arming:
        controller_signal = PID_control(
                        kp=kp,ki=ki,kd=kd,
                        state=current_state,
                        destination =[traj[0][counter],traj[1][counter] ,traj[2][counter]],
                        floatability=floatability
                        )
        counter+=1     #p control for yaw
    Correction_depth = 1500
    Correction_pitch = 1500
    Correction_yaw = 1500

    # Send PWM commands to motors

    if sum(control_mode)>0:
        Correction_pitch = int(get_mapping(controller_signal[0]))
        Correction_yaw = int(get_mapping(controller_signal[1]))
        Correction_depth = int(get_mapping(controller_signal[2]))
    if not control_depth:
        Correction_depth = 1500

    control_signal_forward= go_forward_or_stop()
    explore_signal = exploraion()
    if explore:
        control_signal_yaw = explore_signal
    else:
        control_signal_yaw = control_signal_mapped
        control_signal_yaw=1500
    print("depth", Correction_depth, "yaw", control_signal_yaw, "forward", control_signal_forward)

    setOverrideRCIN(1500, 1500, Correction_depth, control_signal_yaw, control_signal_forward, 1500)

    control_cmd = Twist()
    control_cmd.linear.z = Correction_depth
    pub_cmd_vel.publish(control_cmd)



def mapValueScalSat(value):  # ask V 
    # Correction_Vel and joy between -1 et 1
    # scaling for publishing with setOverrideRCIN values between 1100 and 1900
    # neutral point is 1500
    pulse_width = value * 400 + 1500

    # Saturation
    if pulse_width > 1900:
        pulse_width = 1900
    if pulse_width < 1100:
        pulse_width = 1100

    return int(pulse_width)


def setOverrideRCIN(channel_pitch, channel_roll, channel_throttle, channel_yaw, channel_forward, channel_lateral):
    # This function replaces setservo for motor commands.
    # It overrides Rc channels inputs and simulates motor controls.
    # In this case, each channel manages a group of motors not individually as servo set

    msg_override = OverrideRCIn()
    msg_override.channels[0] = np.uint(channel_pitch)       # pulseCmd[4]--> pitch
    msg_override.channels[1] = np.uint(channel_roll)        # pulseCmd[3]--> roll
    msg_override.channels[2] = np.uint(channel_throttle)    # pulseCmd[2]--> heave
    msg_override.channels[3] = np.uint(channel_yaw)        # pulseCmd[5]--> yaw
    msg_override.channels[4] = np.uint(channel_forward)     # pulseCmd[0]--> surge
    msg_override.channels[5] = np.uint(channel_lateral)     # pulseCmd[1]--> sway
    msg_override.channels[6] = 1500
    msg_override.channels[7] = 1500
    pub_msg_override.publish(msg_override)


def subscriber():
    global counter
    rospy.Subscriber("joy", Joy, joyCallback)
    rospy.Subscriber("cmd_vel", Twist, velCallback)
    rospy.Subscriber("mavros/imu/data", Imu, OdoCallback)
    rospy.Subscriber("mavros/imu/water_pressure", FluidPressure, PressureCallback)
    #rospy.Subscriber("/dvl/data", DVL, DvlCallback)
    rospy.Subscriber("distance_sonar", Float64MultiArray, pingerCallback)
    counter+=1
    rospy.spin()

if _name_ == '_main_':
    armDisarm(False)  # Not automatically disarmed at startup
    # arming=True
    rospy.init_node('autonomous_MIR', anonymous=False)

    pub_msg_override = rospy.Publisher("mavros/rc/override", OverrideRCIn, queue_size = 10, tcp_nodelay = True)
    pub_angle_degre = rospy.Publisher('angle_degree', Twist, queue_size = 10, tcp_nodelay = True)
    pub_depth = rospy.Publisher('depth/state', Float64, queue_size = 10, tcp_nodelay = True)

    pub_angular_velocity = rospy.Publisher('angular_velocity', Twist, queue_size = 10, tcp_nodelay = True)
    pub_linear_velocity = rospy.Publisher('linear_velocity', Twist, queue_size = 10, tcp_nodelay = True)    
    pub_cmd_vel = rospy.Publisher('/br2/cmd_vel',Twist, queue_size = 10, tcp_nodelay = True)

    subscriber()