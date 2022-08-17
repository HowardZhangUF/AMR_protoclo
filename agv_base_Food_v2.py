##### Note: This file "Neglect" the CRC16 checksum which sould be applied in the Modbus Potocol. #####


#!/usr/bin/env python2

import rospy
import sys
import time
import math
import tf
import tf2_ros
import Queue
import copy
import threading
#
import json
#
import serial
import crcmod # CRC16 generation and checking
import struct # For packing/unpacking the byte array

import socket

from sensor_msgs.msg import Imu #, JointState
from tf.transformations import quaternion_from_euler
#
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import (
    Float32,
    Float64,
    Bool,
    Empty,
    String
)
from nav_msgs.msg import Odometry
# from agv.msg import(
#     io_out_feedback,
#     power_state,
#     alarm_feedback
# )


#
# from std_msgs.msg import Float32MultiArray

# Settings
is_broadcasting_tf = True

#TCP ethernet
tcp_host = '192.168.1.1' # Product Car
#tcp_host = '192.168.52.239' # Test Car
tcp_port = 502
#tcp_host = '127.0.0.1'
#tcp_port = 7000
tcp_socket = None

value_x = 0
value_y = 0

# Serial port
serial_device_name = "/dev/base_USB"
#serial_device_name = "/dev/ttyUSB0"
serial_baud_rate = 115200 #  57600 # 115200
# Buffer for serial communication with base
rec_buff = ""
# CRC16
crc16_key = 0x11021 # int
#crc16_key = 0x18005 # int
crc16 = crcmod.mkCrcFun(crc16_key,  rev=False)
# Special chars
head_char = b'\x01' # 0x01
end_chars = b'\x0d\x0a' # \r\n
#
serialDev = None
is_serial_device_available = False



# Publishers
serialOnlinePub = rospy.Publisher('/base/online', Bool, queue_size = 2, latch=True) # Latched

# Note: While using Cartographer's odom, the odom topic below should not be published. 
#odom_pub = rospy.Publisher('/odom', Odometry, queue_size = 1)

# lift_height_pub = rospy.Publisher('/base/status/lift_height', Float64, queue_size = 1, latch=True)
# base_IO_pub = rospy.Publisher('/base/status/IO', kenmec_base_io_out, queue_size = 1, latch=True)
pause_move_pub = rospy.Publisher('/base/lock', Bool, queue_size = 1, latch=True) # This topic is mainly send to base_serial
pause_move_status_pub = rospy.Publisher('/base/status/pause_move', String, queue_size = 1, latch=True) # This topic is mainly send to base_serial
# io_out_feedback_pub = rospy.Publisher('/base/status/io_out_feedback', io_out_feedback, queue_size = 1, latch=True)
# power_state_pub = rospy.Publisher('/base/status/power', power_state, queue_size = 1, latch=True)
# alarm_feedback_pub = rospy.Publisher('/base/status/alarm_feedback', alarm_feedback, queue_size = 1, latch=True)
# joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size = 5, latch=True)
# tf broadcaster
odom_broadcaster =  tf2_ros.TransformBroadcaster()

# Messages

# Time stamps
last_send_time = None
last_receiving_time = None

# send-data buffer
reset_list          = [0.0, 0.0, 0.0, 0.0]
cmd_vel_list        = [1.0, 0.0, 0.0, 0.0] # vel_x, vel_y, omega_z


# Variables
#------------------------#
x = 0.0
y = 0.0
th = 0.0

left_rpm = 0.0
right_rpm = 0.0

#
is_lock = False # Initialized to false
#
fork_plate_height = 0.0
#------------------------#

# last_vel_x = 0.0
# last_vel_y = 0.0
# last_vel_th = 0.0
# last_odom_time = time.time()
# ODOM_DELTA_TIME = 0.2

# Constants
#------------------------#
deg2rad = 0.017453292519943295 # pi/180.0
rad2deg = 57.29577951308232 # 180.0/pi
#------------------------#

# Program start time
stamp_prog_start = time.time()
stamp_rec_previous = time.time()


# odom and tf
#------------------------------------------------------#
def proc_odometry(vel_x, vel_y, omega_z):
    global current_time
    global last_time
    global x,y,th
    # global last_vel_x, last_vel_y, last_vel_th

    #
    current_time = rospy.Time.now()
    dt = (current_time - last_time).to_sec()
    last_time = current_time
    #
    delta_x = (vel_x * math.cos(th) - vel_y * math.sin(th) ) * dt
    delta_y = (vel_x * math.sin(th) + vel_y * math.cos(th) ) * dt
    delta_th = omega_z * dt

    x += delta_x
    y += delta_y
    th += delta_th
    #
    pubodom(vel_x, vel_y, omega_z)
    # pubodom(last_vel_x, last_vel_y, last_vel_th)
    if is_broadcasting_tf:
        broadcastodom()
    # print "theta =", (th*57.2957795), "deg"

def broadcastodom():
    global x,y,th
    global odom_broadcaster
    # odom_broadcaster =  tf2_ros.TransformBroadcaster()
    odom_quat = tf.transformations.quaternion_from_euler(0,0,th)
    odom_trans = TransformStamped()

    odom_trans.header.stamp = current_time
    odom_trans.header.frame_id = "odom"
    odom_trans.child_frame_id = "base_footprint" #"rugby_base"
    odom_trans.transform.translation.x = x
    odom_trans.transform.translation.y = y
    odom_trans.transform.translation.z = 0.0
    odom_trans.transform.rotation.x = odom_quat[0]
    odom_trans.transform.rotation.y = odom_quat[1]
    odom_trans.transform.rotation.z = odom_quat[2]
    odom_trans.transform.rotation.w = odom_quat[3]
    # print("[base] broadcast the odom-->base_footprint tf")
    odom_broadcaster.sendTransform(odom_trans)

def pubodom(vel_x, vel_y, omega_z):
    global x,y,th
    global odom_pub

    # odom_pub = rospy.Publisher('/odom', Odometry, queue_size = 50)
    odom_quat = tf.transformations.quaternion_from_euler(0,0,th)
    odom = Odometry()

    odom.header.stamp = current_time
    odom.header.frame_id = "odom"
    odom.pose.pose.position.x = x
    odom.pose.pose.position.y = y
    odom.pose.pose.position.z = 0.0
    odom.pose.pose.orientation.x =  odom_quat[0]
    odom.pose.pose.orientation.y =  odom_quat[1]
    odom.pose.pose.orientation.z =  odom_quat[2]
    odom.pose.pose.orientation.w =  odom_quat[3]

    odom.pose.covariance[0] = 1
    odom.pose.covariance[7] = 1
    odom.pose.covariance[14] = 1
    odom.pose.covariance[21] = 1
    odom.pose.covariance[28] = 1
    odom.pose.covariance[35] = 1

    odom.child_frame_id = "base_footprint" #"rugby_base"
    odom.twist.twist.linear.x = vel_x
    odom.twist.twist.linear.y = vel_y
    odom.twist.twist.angular.z = omega_z

    odom_pub.publish(odom)
#------------------------------------------------------#



# Definition of pubMsgGroup
# Using string as key
#-----------------------------------#
"""
motion
query
turnSig
alarm
pauseButton
engagement
errorState
IOChange
rangeSense
IOIn
IOOut
power
radar_layer
"""
pubMsgGroup_func_dict = dict()
#-----------------------------------#


# joint_state_pub_rate = 10.0 # Hz
# def joint_state_sending_worker():
#     """
#     This is the worker for the thread of periodically sending the joint state of the Lift
#     """
#     global fork_plate_height
#     rate = rospy.Rate(joint_state_pub_rate)
#     print("[base] joint_state_sending_worker thread started.")
#     while not rospy.is_shutdown():
#         send_single_joint_state("fork_plat_joint", fork_plate_height)
#         rate.sleep()
#     print("[base] joint_state_sending_worker thread ended.")

# def send_single_joint_state(name, position, velocity=0.0, effort=0.0, stamp=None):
#     """
#     """
#     global joint_state_pub
#     # Broadcast joint state of the lift
#     #-------------------------------------------------#
#     _js = JointState()
#     _js.header.stamp = rospy.get_rostime() if stamp is None else stamp
#     _js.name.append(name)
#     _js.position.append(position)
#     _js.velocity.append(velocity)
#     _js.effort.append(effort)
#     joint_state_pub.publish(_js)
#     #-------------------------------------------------#

# Managing the pause move signal state (ultimately it will be connected /base/lock)
#-------------------------------------#
pause_move_status_dict = dict()
required_to_pause = False
def reset_pause_move():
    """
    Note: This function is not thread safe with update_pause_move(), only for testing.
    """
    global pause_move_status_dict
    global required_to_pause
    for _key in pause_move_status_dict:
        pause_move_status_dict[_key] = False
    required_to_pause = False
    pause_move_pub.publish(required_to_pause)
    rospy.loginfo("[base] reset pause_move, required_to_pause = %s" % str(required_to_pause) )

# def publish_pause_move(value_list):
#     """
#     value_list: any one of the element is True will result in pause moving.
#     """
#     global required_to_pause
#     # Scan
#     required_to_pause_new = False
#     triggered_idx_list = []
#     for _i, _v in enumerate(value_list):
#         required_to_pause_new |= _v
#         if _v:
#             triggered_idx_list.append(_i)
#     # Publish
#     if required_to_pause != required_to_pause_new:
#         pause_move_pub.publish(required_to_pause_new)
#         required_to_pause = required_to_pause_new
#         print("[base] required_to_pause = %s, triggered_idx_list = %s" % (str(required_to_pause), str(triggered_idx_list) ) )
#     return required_to_pause

def update_pause_move(new_status_dict):
    """
    new_status_dict:
    - key: the control item
    - value: the value of the related control item, True: pause, False: safe

    Any one of the element being True results in base locking.
    """
    global pause_move_status_dict
    global required_to_pause
    # Update
    pause_move_status_dict.update(new_status_dict)
    triggered_key_list = []
    required_to_pause_new = False
    should_send_log = False
    for _key, _value in pause_move_status_dict.items():
        if _value: # Only enter this clause if the value is True
            required_to_pause_new = True
            # Immediate publish the message (early stopping, but keep collecting the keys with value True)
            if required_to_pause != required_to_pause_new:
                pause_move_pub.publish(required_to_pause_new)
                required_to_pause = required_to_pause_new # Note: This prevent the entering of this clause at next key with value True.
                should_send_log = True
            # Collecting the keys with value True
            triggered_key_list.append(_key)
    # Publish when the value from True --> False (no early stopping --> all False)
    if required_to_pause != required_to_pause_new:
        pause_move_pub.publish(required_to_pause_new)
        required_to_pause = required_to_pause_new
        should_send_log = True
    # log
    if should_send_log:
        rospy.loginfo("[base] required_to_pause = %s, triggered_key_list = %s" % (str(required_to_pause), str(triggered_key_list) ) )
    return required_to_pause

#-------------------------------------#




# Receiving-data processing { NEED TO MODIFIED!!!!!!!!!!!!!!!!!!!!!!!!!!! }
#-------------------------------------------------#

def pubMsgGroup_command(ticket, rec_data_list):
    print("send command")


def pubMsgGroup_motion(ticket, rec_data_list):
    #
    # vel_x = rec_data_list[3]
    # vel_y = rec_data_list[4]
    # omega_z = rec_data_list[5]
    #print("<%s>(vel_x, vel_y, omega_z) = %s" % (ticket["pubMsgGroup"], str((vel_x, vel_y, omega_z))) )
    # proc_odometry(vel_x, vel_y, omega_z)
    pass

def pubMsgGroup_query(ticket, rec_data_list):
    #
    # vel_x = rec_data_list[3]
    # vel_y = rec_data_list[4]
    # omega_z = rec_data_list[5]
    #print("<%s>(vel_x, vel_y, omega_z) = %s" % (ticket["pubMsgGroup"], str((vel_x, vel_y, omega_z))) )
    # proc_odometry(vel_x, vel_y, omega_z)
    pass

# def pubMsgGroup_lift(ticket, rec_data_list):
#     """
#     output
#     - height_now (unit: m): Note: the numeric in data frame from base in the unit of mm
#     """
#     global fork_plate_height
#     height_now = rec_data_list[4] * 0.001
#     print("<%s>height_now = %f" % (ticket["pubMsgGroup"], height_now))
#     lift_height_pub.publish(height_now)
#     # Broadcast joint state of the lift
#     #-------------------------------------------------#
#     fork_plate_height = copy.copy(height_now)
#     send_single_joint_state("fork_plat_joint", fork_plate_height)
#     #-------------------------------------------------#


def pubMsgGroup_alarm():
    print("send alarm")
    pass

def pubMsgGroup_io_out(ticket, rec_data_list):
    # status = rec_data_list[4]
    # bit_str = get_bit_str_from_uint16[status]
    # auto_charge = get_bit_from_bit_str(bit_str, 0)
    # left_light = get_bit_from_bit_str(bit_str, 1)
    # right_light = get_bit_from_bit_str(bit_str, 2)
    # red_light = get_bit_from_bit_str(bit_str, 3)
    # green_light = get_bit_from_bit_str(bit_str, 4)
    # yellow_light = get_bit_from_bit_str(bit_str, 5)
    # buzzer = get_bit_from_bit_str(bit_str, 6)
    # speaker = get_bit_from_bit_str(bit_str, 7)
    # msg = io_out_feedback()
    # msg.auto_charge = auto_charge
    # msg.left_light = left_light
    # msg.right_light = right_light
    # msg.red_light = red_light
    # msg.green_light = green_light
    # msg.yellow_light = yellow_light
    # msg.buzzer = buzzer
    # msg.speakerA = speakerA # On Duty Speaker
    # msg.speakerB = speakerB # Deceleration Speaker
    # msg.speakerC = speakerC # Charging Speaker
    # io_out_feedback_pub.publish(msg)
    # print("auto_charge = " + auto_charge)
    # print("left_light = " + left_light)
    # print("right_light = " + right_light)
    # print("red_light = " + red_light)
    # print("green_light = " + green_light)
    # print("yellow_light = " + yellow_light)
    # print("buzzer = " + buzzer)
    # print("on duty speaker = " + speakerA)
    # print("deceleration speaker = " + speakerB)
    # print("charge speaker = " + speakerC)
    pass


def pubMsgGroup_turn_sign():
    print("send turn signal")
    pass


def pubMsgGroup_power(ticket, rec_data_list):
    #v_bat = rec_data_list[3]
    #i_bat = rec_data_list[4]
    print("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% v " + str(rec_data_list[3]))
    print("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% i " + str(rec_data_list[4]))
    soc = rec_data_list[5]
    print("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% soc " + str(rec_data_list[5]))
    soc = rec_data_list[5]
    #print("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%<%s>(v_bat, i_vat, SoC) = %s" % (ticket["pubMsgGroup"], str((v_bat, i_bat, soc))))
    msg = power_state()
    msg.v_bat = 0
    msg.i_bat = 0
    msg.soc = soc
    power_state_pub.publish(msg)

def pubMsgGroup_radar_layer(ticket, rec_data_list):
    print("send turn radar_layer")
    pass

def pubMsgGroup_io_in(ticket, rec_data_list):
    print("send io_in")
    pass

#-------------------------------------------------#

# Register the functions for pubMsgGroup
#-------------------------------------------------#
pubMsgGroup_func_dict["motion"] = pubMsgGroup_motion #pubMsgGroup_command
pubMsgGroup_func_dict["query"] = pubMsgGroup_query
# pubMsgGroup_func_dict["lift"]   = pubMsgGroup_lift
pubMsgGroup_func_dict["alarm"] = pubMsgGroup_alarm
pubMsgGroup_func_dict["IOOut"] = pubMsgGroup_io_out
pubMsgGroup_func_dict["turnSig"] = pubMsgGroup_turn_sign
pubMsgGroup_func_dict["power"] = pubMsgGroup_power
pubMsgGroup_func_dict["radar_layer"] = pubMsgGroup_radar_layer
pubMsgGroup_func_dict["IOIn"] = pubMsgGroup_io_in
#-------------------------------------------------#


def get_bit_str_from_uint16(num_uint16_in):
    """
    Example
    num_uint16_in = 0x25 (dec: 37)
    bit_str_out = "0000000000100101"
           ** Note: MSB is at the left. Index of the MSB is 0 **
    """
    return ( '{:016b}'.format(num_uint16_in) )

def get_bit_from_bit_str(bit_str, bit_idx=0):
    """
    "bit" string: a len-16 string with each char being '0' (string) or '1' (string)
                  e.g. "0001", bit #0 is 1 (True), where bit #3 is 0 (False)

    return True/False (1/0)
    """
    return (bit_str[-(1+bit_idx)] == '1') # Index from the back of the bit_str

def get_bit_list_from_bit_str(bit_str):
    """
    "bit" string: a len-16 string with each char being '0' (string) or '1' (string)
                  e.g. "0001", bit #0 is 1 (True), where bit #3 is 0 (False)

    return [True/False (1/0)] (a list)
    """
    bit_list = list()
    for bit_idx in range(len(bit_str)):
        bit_list.append( (bit_str[-(1+bit_idx)] == '1') ) # Index from the back of the bit_str
    return bit_list
#----------------------------------------#
# def process_event_0x33(data):
#     """
#     """
#     # Convert to "bit" string - a len-16 string with each char being '0' (string) or '1' (string)
#     # bit_str_data_1_2 = '{:016b}'.format(data)
#     bit_str_data_1_2 = get_bit_str_from_uint16( data )
#     bit_str_data_1 = bit_str_data_1_2[:8]
#     bit_str_data_2 = bit_str_data_1_2[8:]
#     print("bit_str_data_1 = [%s]" % bit_str_data_1)
#     print("bit_str_data_2 = [%s]" % bit_str_data_2)
#     #
#     bit_list_data_1 = get_bit_list_from_bit_str(bit_str_data_1)
#     bit_list_data_2 = get_bit_list_from_bit_str(bit_str_data_2)
#     # Assign to each element
#     # touch_L, touch_R, approx_L, approx_R, bumper, reset_button, TIM320_L_out1, TIM320_L_out2 = bit_list_data_1
#     # TIM320_L_out3, TIM320_L_out4, TIM320_R_out1, TIM320_R_out2, TIM320_R_out3, TIM320_R_out4, range_check_L, range_check_R = bit_list_data_2
#     touch_L, touch_R, approx_L, approx_R, bumper, reset_button, range_check_L, range_check_R = bit_list_data_1
#     TIM320_L_out1, TIM320_L_out2, TIM320_L_out3, TIM320_L_out4, TIM320_R_out1, TIM320_R_out2, TIM320_R_out3, TIM320_R_out4 = bit_list_data_2
#     _io_msg = kenmec_base_io_out()
#     _io_msg.touch_L = touch_L
#     _io_msg.touch_R = touch_R
#     _io_msg.approx_L = approx_L
#     _io_msg.approx_R = approx_R
#     _io_msg.bumper = bumper
#     _io_msg.reset_button = reset_button
#     _io_msg.TIM320_L_out1 = TIM320_L_out1
#     _io_msg.TIM320_L_out2 = TIM320_L_out2
#     _io_msg.TIM320_L_out3 = TIM320_L_out3
#     _io_msg.TIM320_L_out4 = TIM320_L_out4
#     _io_msg.TIM320_R_out1 = TIM320_R_out1
#     _io_msg.TIM320_R_out2 = TIM320_R_out2
#     _io_msg.TIM320_R_out3 = TIM320_R_out3
#     _io_msg.TIM320_R_out4 = TIM320_R_out4
#     _io_msg.range_check_L = range_check_L
#     _io_msg.range_check_R = range_check_R
#     print(str(_io_msg))
#     base_IO_pub.publish(_io_msg)
#     # Test, base lock directly issued by bumper
#     # pause_move_pub.publish( bumper)
#     # publish_pause_move( (bumper, TIM320_L_out1, TIM320_R_out1) )
#     new_status_dict = dict()
#     new_status_dict["bumper"] = bumper
#     new_status_dict["TIM320_L_out1"] = TIM320_L_out1
#     new_status_dict["TIM320_R_out1"] = TIM320_R_out1
#     update_pause_move(new_status_dict)
#     #

#----------------------------------------#
def process_event(valid_data_frame):
    """
    This is the function for parsing the event and pubilish it.
    """
    print("Event: valid_data_frame = 0x{}".format( valid_data_frame.encode('hex')))
    
    # rec_data_list = struct.unpack('>BBHH', valid_data_frame[:-2]) # Big-endian, remove the CRC16 part
    
    print("Event: rec_data_list = {}".format(rec_data_list))
    # TODO: Publish the event according to the CMD address
    if rec_data_list[2] == int("33", 16):
        process_event_0x33(rec_data_list[3])
    else:
        pass
    #
    return rec_data_list
#----------------------------------------#

# New data receiving process
#-----------------------------------------------------#
# data_frame_format for motion feedback '>BBHfff'
def _serialRead(data_frame_length=6, data_frame_format='>BBHH', read_once=False):
    """
    Wait for the specified response frame and event from serial port.
    Inputs:
    data_frame_length: The length of the specified data frame (Note: CRC16 bytes are not included)
    data_frame_format: The format of the data frame (Note: the CRC16 bytes are not included)
    Note: Default arguments are set for the event frame.
    """
    global stamp_prog_start, stamp_rec_previous
    global rec_buff, is_serial_device_available
    global crc16, head_char, end_chars

    # Return if the serial is not available
    if not is_serial_device_available:
        return

    # Parameter for receiveing data
    MAX_BUF_SIZE = 1000 # 2500 # For a package being 50 bytes, 50 Hz transmition rate, this is a 1 sec. buffer
    RECEIVED_TIMEOUT = 0.15 # 0.2 # 5.0 # sec.

    # Read serial
    is_frame_completed = False
    count_no_data = 0
    count_loop = 0
    is_first_time = True
    #
    
    # data_frame_length_CRC = data_frame_length + 2 # The CRC16 contains 2 bytes
    # data_frame_length_target = data_frame_length_CRC
    
    # if not read_once:
    #     print("data_frame_length = %d" % data_frame_length)
    #     print("data_frame_length_CRC = %d" % data_frame_length_CRC)
    #     print("data_frame_length_target = %d" % data_frame_length_target)
    is_event = False
    stamp_start = time.time()
    #
    rec_data_list = None

    delta_time = 0.0
    while not is_frame_completed:
        count_loop += 1
        # Timeout check
        delta_prog_time = time.time() - stamp_prog_start
        delta_time = time.time() - stamp_start
        if delta_time > RECEIVED_TIMEOUT:
            print("[WARN]Receive-timeout: %f > %f" % (delta_time, RECEIVED_TIMEOUT))
            break

        # Sleep for a while (1ms) when looping
        if is_first_time:
            is_first_time = False
        else:
            if read_once:
                break
            time.sleep(0.001)
            # time.sleep(0.1)

        # Roger: If the request will not be response, set frame completed
        if data_frame_length == 0 and data_frame_format == '>XXXX':
            is_frame_completed = True
            break

        # Read from buffer
        try:
            rec_delta = serialDev.read(serialDev.inWaiting())
        except:
            #is_serial_device_available = False
            break

        # if not read_once:
        #     print("delta_time = %f" % delta_time)
        #     print(rec_buff)
        #     print("rec_delta = 0x{}".format( rec_delta.encode('hex')))
        #     print("rec_buff = 0x{}".format( rec_buff.encode('hex')))
        # if read_once:
        #     print("delta_time = %f" % delta_time)
        #     print(rec_buff)
        #     print("rec_delta = 0x{}".format( rec_delta.encode('hex')))
        #     print("rec_buff = 0x{}".format( rec_buff.encode('hex')))


        # See if we got data
        # if no data received this time, bypass the parsing this time if it's not read_once for efficiency
        #----------------------#
        if len(rec_delta) == 0 and (not read_once):
            count_no_data += 1
            # print("no data this time")
            continue
        else:
            count_no_data = 0
            if not read_once:
                # print("rec_buff (before appending) = 0x{}".format( rec_buff.encode('hex')))
                print("--Received data from serial port, time elapsed = %f sec., count_loop = %d" % (delta_time, count_loop))
                print("--rec_delta = 0x{}".format( rec_delta.encode('hex')))
        #----------------------#


        # Add to the buffer
        rec_buff += rec_delta
        size_rec_buf = len(rec_buff)
        if not read_once:
            print("delta_prog_time = %f" % delta_prog_time)
            print(rec_buff)
            print("rec_delta = 0x{}".format( rec_delta.encode('hex')))
            print("rec_buff = 0x{}".format( rec_buff.encode('hex')))

        idx_head = rec_buff.find(head_char)
        # if not read_once:
        #     print("rec_buff = 0x{}".format( rec_buff.encode('hex')))
        #     print("idx_head = %d" % idx_head)
        if idx_head == -1:
            # "head" not found,
            # continue
            # Clean data when the buffer is too long
            if size_rec_buf >= MAX_BUF_SIZE:
                rec_buff = ""
            continue


        # Check the length for least valid data (empty)
        if (size_rec_buf-idx_head) < 4:
            # Nomatter which function we used, the length of data frame is at least 4 bytes (id|func|...|CRC-H|CRC-L)
            # It's not posssible to find a valid data frame within this short data,
            # keep waiting
            continue

        # Found the plausible head. Further check
        # Check if it's event (Note: the indexing is valid because the length of the data is guarenteed to be more than 4 bytes)
        if rec_buff[idx_head+1] == b'\x08': # Event
            data_frame_length_target = 8 # Temporary modify the task to receive the event
            is_event = True
            print("<--(jump) Waiting for event")
        else:
            data_frame_length_target = data_frame_length_CRC # Return to the original target
            is_event = False

        # Check the length for the require data frame
        if (size_rec_buf - idx_head) < data_frame_length_target:
            # Frame not completed, keep waiting
            continue

        # We got the full data frame, check if it's valid (CRC)
        idx_end = idx_head + data_frame_length_target # Note: idx_end points to teh char that is not in the frame
        print("--rec_buff (found a plaussible data segment with sufficient length and correct head)\n\t\t= 0x{}".format( rec_buff.encode('hex')))
        print("--data_frame_length_target = %d" % data_frame_length_target)
        print("--(idx_head, idx_end) = (%d, %d)" % (idx_head, idx_end))



        # 2. CRC16 checksum
        # Generate the crc16 checksum, should be the same as the one the message carried
        # buff_crc_int = crc16( rec_buff[idx_head:(idx_end-2)] )
        # print("buff_crc = {}".format( hex(buff_crc_int) ) )
        # Checksum, the inverse calculation includes the crc16 code
        buff_checksum_int = crc16( rec_buff[idx_head:idx_end] )
        print("--buff_checksum_int = %d" %  buff_checksum_int)
        # Should be exactly zero
        if buff_checksum_int != 0:
            print("[WARN]Checksum test failed!!")
            # Remove the current head (It's not sufficient to show that the rest char are invalid data)
            rec_buff = rec_buff[(idx_head+1):]
            continue

        # Parse data
        valid_data_frame = rec_buff[idx_head:idx_end]
        # Trim frame
        rec_buff = rec_buff[idx_end:]

        #
        state_rec_current_time = time.time()
        stamp_rec_delta = state_rec_current_time - stamp_rec_previous
        stamp_rec_previous = state_rec_current_time
        print("--Found valid data frame with CRC check passed, time elapsed = %f sec., count_loop = %d" % (delta_time, count_loop))
        print("--valid_data_frame = 0x{}".format( valid_data_frame.encode('hex')))
        print("--Delta time from previous receiving = %f" % stamp_rec_delta)
        #

        # Different processing routing for event and required response
        if is_event:
            # Parse and send the event
            process_event(valid_data_frame)
            print("-->(return) Finish the evet processing, keep waiting for the required response.")
            # Continue for the main receiving task
            continue
        else:
            # Parse the data in the given format
            try:
                rec_data_list = struct.unpack(data_frame_format, valid_data_frame[:-2]) # Remove the CRC16 part
                print(">>>(Got) rec_data_list = {}".format(rec_data_list))
            except:
                print("[WARN]Received valid non-event frame does not match the given data frame, break.")
                break
            # # Note: the following code is only for demo.
            # # Represented by dict()
            # rec_data_dict = dict()
            # rec_data_dict['id'] = rec_data_list[0]
            # rec_data_dict['rw'] = rec_data_list[1]
            # rec_data_dict['msg_type'] = rec_data_list[2]
            # rec_data_dict['x'] = rec_data_list[3]
            # rec_data_dict['y'] = rec_data_list[4]
            # rec_data_dict['yaw'] = rec_data_list[5]
            # print("rec_data_dict = %s" % json.dumps(rec_data_dict, indent=4))

            # End of the receiving task, leave
            is_frame_completed = True
            break # break the while
    # end while

    if is_frame_completed:
        print(">>>(Got) Data received, time elapsed = %f sec., count_loop = %d" % (delta_time, count_loop))
    else: # May be tiemout, data not received
        if not read_once:
            print("Receiving task was not completed, timeout/invalid-frame.")
        #

    return rec_data_list
#-----------------------------------------------------#

# New data sending process
#-----------------------------------------------------#
def _serialSend(send_data_list, data_frame_format='>BBHfff'):
    """
    Input:
    - send_data_list: list of data which are ready to be serialized as string (Note: CRC16 is not included.)
    - data_frame_format: The format for the data frame. (Note: CRC16 is not included, default: motion cmd.)
    """
    global last_send_time
    global crc16, end_chars

    if len(send_data_list) != (len(data_frame_format)-1):
        print("send_data_list (cell:%d) does not match with data_frame_format (cell:%d), no sending" % (len(send_data_list), (len(data_frame_format)-1)) )
        return

    # # Generate data_string. This should be "bytes" (str)
    # data_string = struct.pack(data_frame_format, 1, 3, int(cmd_group), \
    #                                 cmd_list[1], \
    #                                 cmd_list[2], \
    #                                 cmd_list[3])

    data_string = struct.pack(data_frame_format, *send_data_list)
    # Test
    # send_buff = "01030001bdcccccdbe4ccccdbe99999a".decode("hex")

    # CRC16 checksum
    # Generate the crc16 checksum, should be the same as the one the message carried
    buff_crc_int = crc16( data_string )
    buff_crc_str = struct.pack('>H', buff_crc_int)
    data_string += buff_crc_str
    # end_chars
    # data_string += end_chars

    # Try to send down
    try:
        print("send start......")
    	tcp_socket.send(data_string)
        print("send end......")
        indata = tcp_socket.recv(1024)
        #serialDev.write(data_string)
        # serialDev.write("Hello")
    except:
        is_serial_device_available = False

    print("-"*20)
    print("<<<(send)buff_crc_int = %d" % buff_crc_int)
    print("<<<(send)buff_crc = 0x{}".format( buff_crc_str.encode('hex') ) )
    print("<<<(send)data_string = 0x{}".format(data_string.encode('hex')))
    print("-"*20)

    """
    tcp_indata = tcp_socket.recv(256)
    if len(tcp_indata) == 0: # connection closed
		tcp_socket.close()
		print("tcp socket recv error.........")
		is_serial_device_available = False

    print("recv: 0x{}".format(tcp_indata.encode('hex')))
    """
#-----------------------------------------------------#








# Communication engine (request-response type)
#------------------------------------------------#
base_req_Q =  Queue.Queue() # Unlimited length
ticket_dropping_threshold = 0.2 # sec.

def book_request(pubMsgGroup,
                send_data_list,
                send_data_frame_format='>BBHfff',
                rec_data_frame_length=6,
                rec_data_frame_format='>BBHfff'):
    """
    This is the function for booking a request.
    """
    global base_req_Q
    ticket = dict()
    ticket["pubMsgGroup"] = pubMsgGroup
    ticket["send_data_list"] = send_data_list
    ticket["send_data_frame_format"] = send_data_frame_format
    ticket["rec_data_frame_length"] = rec_data_frame_length
    ticket["rec_data_frame_format"] = rec_data_frame_format
    ticket["stamp"] = time.time()
    base_req_Q.put(ticket)
    print("@(booking) A ticket booked for <%s>" % ticket["pubMsgGroup"] )

def communication_engine():
    """
    A routine process for handling request, if any
    """
    global base_req_Q
    global ticket_dropping_threshold
    # global last_vel_x, last_vel_y, last_vel_th, last_odom_time

    while ( (not base_req_Q.empty()) and (not rospy.is_shutdown()) ):
        # while base_req_Q.qsize() > 3.0:
        #     base_req_Q.get()
        # Non-empty
        ticket = base_req_Q.get()
        # Flushing out old commands
        _current_stamp = time.time()
        if ticket_dropping_threshold is not None:
            _dropout_stamp = _current_stamp - ticket_dropping_threshold
            while ticket["stamp"] < _dropout_stamp and ticket["pubMsgGroup"] == "motion":
                print("!! !!(handling) The ticket of <%s> got from the queue is too old, <%f> sec. ago. Drop out.." % (ticket["pubMsgGroup"], (_current_stamp - ticket["stamp"]) ) )
                try:
                    ticket = base_req_Q.get(block=False)
                    print("D: %f ms" % (1000.0*(time.time() - _current_stamp)) )
                except:
                    ticket = None
                    break
            if ticket is None:
                break
        #
        print("!!(handling) A ticket of <%s> got from the queue. Number of remained ticket(s) in the queue = %d" % (ticket["pubMsgGroup"], base_req_Q.qsize()) )
        rec_data_list = _communication_core(
                                    ticket["send_data_list"],
                                    ticket["send_data_frame_format"],
                                    ticket["rec_data_frame_length"],
                                    ticket["rec_data_frame_format"]
                                    )
        # Processing the data
        print(">>>(got) rec_data_list from serialRead = %s" % str(rec_data_list))
        if rec_data_list is not None:
            pubMsgGroup_func_dict[ ticket["pubMsgGroup"] ](ticket, rec_data_list)
            # try:
            #     pubMsgGroup_func_dict[ ticket["pubMsgGroup"] ](ticket, rec_data_list)
            # except:
            #     print("No function registered for [%s]" % ticket["pubMsgGroup"])
            # Sleep for a while in case that the base cannot process the request right after the previous request
            time.sleep(0.001)
        print("-"*20)
        print("\n")
        
        # # check odom
        # odom_delta_time = time.time() - last_odom_time
        # if odom_delta_time > ODOM_DELTA_TIME:
        #     last_vel_x = 0.0
        #     last_vel_y = 0.0
        #     last_vel_th = 0.0
        #     pubodom(last_vel_x, last_vel_y, last_vel_th)
        #     last_odom_time = time.time()

    # Still, do a polling once
    # Clean-up read (parse and send event if there is any)
    _serialRead(read_once=True)



def _communication_core(send_data_list,
                        send_data_frame_format='>BBHfff',
                        rec_data_frame_length=6,
                        rec_data_frame_format='>BBHfff'):
    """
    This is the engine to perform the request-response type of protocal
    as well as the event trigger type protocal.
    The engine also include a clean-up action for only receive the event and clean otherwise.
    """
    # Clean-up read (parse and send event if there is any)
    _serialRead(read_once=True)
    # send
    _serialSend(send_data_list, send_data_frame_format)
    # read & parse (While it's waiting, parse and send event if there is any)
    rec_data_list = _serialRead(rec_data_frame_length, rec_data_frame_format, read_once=False)
    # rec_data_list = None
    # return parsed data list in the given data frame
    return rec_data_list
#------------------------------------------------#





# Multiple type of serial-request functions
#---------------------------------------------#
def SerialReq_motion( vel_x, vel_y, omega_z ):
    """
    Request to set the motion command.
    input:
    """

    print "vel_x = ", vel_x # m/s
    print "vel_y = ", vel_y 
    print "omega_z = ", omega_z # rad/s

    # Calibrate the porportion of the wheel motor ( 0 for 0 m/min, 8000 for 50 m/min ) 
    value_y = vel_x / 50 * 60 * 8000
    
    # yaw_rate = (Rwheelspeed - Lwheelspeed) / d where d is the track width between two differential wheels
    # for Food AMR, the maximun value of value_x = 4000 equals to the difference speed 22.4 m/min between two wheels.
    if value_y == 0:
        value_x = omega_z / 22.4 * 60 * 0.55 * 4000 * 2 # +/- correcting the control direction 
    else:
        value_x = omega_z / 22.4 * 60 * 0.55 * 4000 # +/- correcting the control direction

    if value_y >= 8000:
	    value_y = 8000
    elif value_y <= -8000:
	    value_y = -8000

    if value_x >= 4000:
	    value_x = 4000
    elif value_x <= -4000:
	    value_x = -4000

    #send_data_list = (0x01, 0x06, 0x0262, 1000)
    #send_data_list = (0x0001, 0x0000, 0x0006, 0x01, 0x06, 610, 100)
    #send_data_list = (0x0001, 0x0000, 0x0006, 0x01, 0x06, 606, 1) #start
    #send_data_list = (0x0001, 0x0000, 0x0006, 0x01, 0x06, 606, 2) #stop
    #send_data_list = (0x0001, 0x0000, 0x0006, 0x01, 0x06, 610, 500) #send x, angle -4000 ~ 4000
    #send_data_list = (0x0001, 0x0000, 0x0006, 0x01, 0x06, 610, -500) #send x, angle -4000 ~ 4000
    #send_data_list = (0x0001, 0x0000, 0x0006, 0x01, 0x06, 611, 1000) #send y, speed -8000 ~ 8000
    #send_data_list = (0x0001, 0x0000, 0x0006, 0x01, 0x06, 611, -1000) #send y, speed -8000 ~ 8000

#---------------------------------------------------------------------------------#

    send_data_list = (0x0001, 0x0000, 0x0006, 0x01, 0x03, 509, 12)
    serial_request("voltage",
                send_data_list,
                send_data_frame_format='>HHHBBHh',
                rec_data_frame_length=11
                rec_data_frame_format='>HHHBBHh')

    send_data_list = (0x0001, 0x0000, 0x0006, 0x01, 0x03, 510, 100)
    serial_request("X_axis_offset",
                send_data_list,
                send_data_frame_format='>HHHBBHh',
                rec_data_frame_length=11
                rec_data_frame_format='>HHHBBHh')

    send_data_list = (0x0001, 0x0000, 0x0006, 0x01, 0x03, 511, 100)
    serial_request("Y_axis_offset",
                send_data_list,
                send_data_frame_format='>HHHBBHh',
                rec_data_frame_length=11
                rec_data_frame_format='>HHHBBHh')

    send_data_list = (0x0001, 0x0000, 0x0006, 0x01, 0x03, 512, 120)
    serial_request("FL_RPM",
                send_data_list,
                send_data_frame_format='>HHHBBHh',
                rec_data_frame_length=12
                rec_data_frame_format='>HHHBBHh')

    send_data_list = (0x0001, 0x0000, 0x0006, 0x01, 0x03, 513, 120)
    serial_request("FR_RPM",
                send_data_list,
                send_data_frame_format='>HHHBBHh',
                rec_data_frame_length=12
                rec_data_frame_format='>HHHBBHh')

    send_data_list = (0x0001, 0x0000, 0x0006, 0x01, 0x03, 520, 100)
    serial_request("FL_error",
                send_data_list,
                send_data_frame_format='>HHHBBHh',
                rec_data_frame_length=11
                rec_data_frame_format='>HHHBBHh')
                
    send_data_list = (0x0001, 0x0000, 0x0006, 0x01, 0x03, 521, 100)
    serial_request("FR_error",
                send_data_list,
                send_data_frame_format='>HHHBBHh',
                rec_data_frame_length=11
                rec_data_frame_format='>HHHBBHh')

    send_data_list = (0x0001, 0x0000, 0x0006, 0x01, 0x06, 602, 0)
    serial_request("GM/NM",
                send_data_list,
                send_data_frame_format='>HHHBBHh',
                rec_data_frame_length=11
                rec_data_frame_format='>HHHBBHh')

    send_data_list = (0x0001, 0x0000, 0x0006, 0x01, 0x06, 606, 1)
    serial_request("stop/start",
                send_data_list,
                send_data_frame_format='>HHHBBHh',
                rec_data_frame_length=11
                rec_data_frame_format='>HHHBBHh')

    send_data_list = (0x0001, 0x0000, 0x0006, 0x01, 0x06, 607, 1)
    serial_request("CW/CCW",
                send_data_list,
                send_data_frame_format='>HHHBBHh',
                rec_data_frame_length=11
                rec_data_frame_format='>HHHBBHh')

    send_data_list = (0x0001, 0x0000, 0x0006, 0x01, 0x06, 612, 150)
    serial_request("wheel_dia",
                send_data_list,
                send_data_frame_format='>HHHBBHh',
                rec_data_frame_length=12
                rec_data_frame_format='>HHHBBHh')

    send_data_list = (0x0001, 0x0000, 0x0006, 0x01, 0x06, 613, 0.5)
    serial_request("reduction_ratio",
                send_data_list,
                send_data_frame_format='>HHHBBHh',
                rec_data_frame_length=11
                rec_data_frame_format='>HHHBBHh')

    send_data_list = (0x0001, 0x0000, 0x0006, 0x01, 0x06, 614, 15)
    serial_request("max_vel",
                send_data_list,
                send_data_frame_format='>HHHBBHh',
                rec_data_frame_length=11
                rec_data_frame_format='>HHHBBHh')

    send_data_list = (0x0001, 0x0000, 0x0006, 0x01, 0x06, 615, 1000)
    serial_request("KP",
                send_data_list,
                send_data_frame_format='>HHHBBHh',
                rec_data_frame_length=12
                rec_data_frame_format='>HHHBBHh')

    send_data_list = (0x0001, 0x0000, 0x0006, 0x01, 0x06, 616, 0.1)
    serial_request("KI",
                send_data_list,
                send_data_frame_format='>HHHBBHh',
                rec_data_frame_length=12
                rec_data_frame_format='>HHHBBHh')

    send_data_list = (0x0001, 0x0000, 0x0006, 0x01, 0x06, 617, 5)
    serial_request("KD",
                send_data_list,
                send_data_frame_format='>HHHBBHh',
                rec_data_frame_length=12
                rec_data_frame_format='>HHHBBHh')

    send_data_list = (0x0001, 0x0000, 0x0006, 0x01, 0x06, 618, 100)
    serial_request("acc_time",
                send_data_list,
                send_data_frame_format='>HHHBBHh',
                rec_data_frame_length=12
                rec_data_frame_format='>HHHBBHh')

    send_data_list = (0x0001, 0x0000, 0x0006, 0x01, 0x06, 619, 100)
    serial_request("dec_time",
                send_data_list,
                send_data_frame_format='>HHHBBHh',
                rec_data_frame_length=12
                rec_data_frame_format='>HHHBBHh')
#---------------------------------------------------------------------------------#

    send_data_list = (0x0001, 0x0000, 0x0006, 0x01, 0x06, 611, value_y)
    book_request("motion",
                send_data_list,
                send_data_frame_format='>HHHBBHh',
                rec_data_frame_length=12
                rec_data_frame_format='>HHHBBHh')

    send_data_list = (0x0001, 0x0000, 0x0006, 0x01, 0x06, 610, value_x)
    book_request("motion",
                send_data_list,
                send_data_frame_format='>HHHBBHh',
                rec_data_frame_length=12,
                rec_data_frame_format='>HHHBBHh')

    # After booking, it can leave
    
    """
    if vel_x >= 0:
        send_data_list = (1,0x16,3,6,0x12, vel_x, vel_y, -omega_z )
        proc_odometry(vel_x, vel_y, -omega_z)
    else:
        send_data_list = (1,0x16,3,6,0x12, vel_x, vel_y, omega_z )
        proc_odometry(vel_x, vel_y, omega_z)
    
        
    book_request("motion",
                send_data_list,
                send_data_frame_format='>BBHHBfff',
                rec_data_frame_length=6,
                rec_data_frame_format='>BBHH')
    # After booking, it can leave
    """

# Query the rpm feedback of left(Address: 512) and right(Address: 513) wheel motors
def SerialQue_rpm( ):
    
    global left_rpm
    global right_rpm
    global rec_data_list

    """
    Query the current rpm
    """
    #the last value 2 means to read 2 bytes start from address 512 to address 513
    send_data_list = (0x0001, 0x0000, 0x0006, 0x01, 0x03, 512, 2) 
    book_request("query",
                send_data_list,
                send_data_frame_format='>HHHBBHh',
                # rec example: 00 01 00 00 00 07 01 03 04 00 ea 00 ed
                rec_data_frame_length=13, 
                rec_data_frame_format='>HHHBBBhh')
    
    #left_rpm = rec_data_list[0]
    
    #right_rpm = rec_data_list[0]

    #print(rec_data_list)
    

    


# def SerialReq_lift( height_cmd, is_query=False ):
#     """
#     Request to set the motion command and receive the motion feedback.
#     input:
#     - height_cmd (unit: m) NOTE: the numeric in serial data frame is in the unit of mm
#     """
#     send_data_list = (1,6,2, (0 if is_query else 1), float(height_cmd)*1000.0 )
#     book_request("lift",
#                 send_data_list,
#                 send_data_frame_format='>BBHBI',
#                 rec_data_frame_length=9,
#                 rec_data_frame_format='>BBHBI')

def SerialReq_alarm(state):
    send_data_list = (1, 6, 0x58, int(state))
    book_request("alarm",
                 send_data_list,
                 send_data_frame_format='>BBHH',
                 rec_data_frame_length=0,
                 rec_data_frame_format='>XXXX')

def SerialReq_io_output_feedback():
    send_data_list = (1, 3, 0x34, 2)
    book_request("IOOut",
                 send_data_list,
                 send_data_frame_format='>BBHH',
                 rec_data_frame_length=9,
                 rec_data_frame_format='>BBBBB')

def SerialReq_turn_signal(state):
    send_data_list = (1, 6, 0x50, int(state))
    book_request("turnSig",
                 send_data_list,
                 send_data_frame_format='>BBHH',
                 rec_data_frame_length=0,
                 rec_data_frame_format='>XXXX')


def SerialReq_power_state():
    send_data_list = (1, 3, 0x36, 7)
    book_request("power",
                 send_data_list,
                 send_data_frame_format='>BBHH',
                 rec_data_frame_length=10,
                 rec_data_frame_format='>BBBfHB')


def SerialReq_radar_layer(state):
    send_data_list = (1, 6, 0x3, 0, 0, int(state, 2))
    book_request("radar_layer",
                 send_data_list,
                 send_data_frame_format='>BBHBBB',
                 rec_data_frame_length=6,
                 rec_data_frame_format='>BBBBBB')
    # After booking, it can leave

def SerialReq_06(datas):
    send_data_list = (1, 6, 0x34, int(datas[0], 2), int(datas[1], 2), int(datas[2], 2))
    book_request("IOIn",
                 send_data_list,
                 send_data_frame_format='>BBHBBB',
                 rec_data_frame_length=6,
                 rec_data_frame_format='>BBBBBB')
#---------------------------------------------#
# end Multiple type of request functions


#------------------------------------------------#
def _base_lock_CB(msg):
    """
    This is the callback function of /base/lock.
    """
    global is_lock
    #
    if msg.data:
        is_lock = True
        #-----------------------------#
        SerialReq_motion(0.0, 0.0, 0.0) # Stop
    else:
        is_lock = False

    # Log for lock Status
    rospy.loginfo("[Base] Base is %s." % ("locked" if is_lock else "unlocked") )


def cmd_vel_CB(twist_aux):
    global cmd_vel_list
    global is_lock
    # global last_vel_x, last_vel_y, last_vel_th, last_time

    # Masking
    if is_lock:
        #-----------------------------#
        SerialReq_motion(0.0, 0.0, 0.0) # Stop
        return
    # else:
    vel_x = twist_aux.linear.x
    vel_y = twist_aux.linear.y
    vel_th = twist_aux.angular.z
    #-----------------------------#
    SerialReq_motion(vel_x, vel_y, vel_th) # Move
    
    # if vel_x != 0 or vel_y != 0 or vel_th != 0:
    #     # if vel_th > 0.1 or vel_th < -0.1:
    #     #     vel_x = 0.0
    # #-----------------------------#
    #     SerialReq_motion(vel_x, vel_y, vel_th) # Move
        
    #     # last_vel_x = vel_x
    #     # last_vel_y = vel_y
    #     # last_vel_th = vel_th
    #     # last_odom_time = time.time()

    #     # pubodom(vel_x, vel_y, vel_th)

    # return
    


# def set_lift_height_CB(msg):
#     """
#     The callback function for setting the height of the lift
#     """
#     SerialReq_lift(msg.data, is_query=False) # Move

# def get_lift_height_CB(msg):
#     """
#     The callback function for setting the height of the lift
#     """
#     SerialReq_lift(0.0, is_query=True) # Move


def update_pause_move_CB(msg):
    """
    The callback function for updating the pause_move status.
    """
    new_status_dict = None
    try:
        new_status_dict = json.loads(msg.data)
    except Exception as e: # Note: the string input might not be a json
        rospy.logerr(repr(e))
        rospy.logerr("[Base] String with Wrong json format sent to /base/update_pause_move topic, no update.")
        rospy.logerr("[Base] msg.data = %s" % msg.data)
    else:
        update_pause_move(new_status_dict)

def reset_pause_move_CB(msg):
    """
    The callback function for reseting the pause_move status.
    """
    reset_pause_move()


def set_alarm_lamp_CB(msg):
    """
    The callback function for setting the alarm lamp status.
    """
    print("set_alarm_lamp" + msg.data)
    SerialReq_alarm(msg.data)


def get_io_output_feedback_CB():
    """
    The callback function for getting io output feedback
    """
    print("get_io_output_feedback")
    SerialReq_io_output_feedback()


def set_turn_signal_CB(msg):
    """
    The callback function for getting io output feedback
    """
    print("set_turn_signal" + msg.data)
    SerialReq_turn_signal(msg.data)


def get_power_state_CB(msg):
    """
    The callback function for getting power state
    """
    print("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%get_power_state")
    SerialReq_power_state()

def set_radar_layer_CB(msg):
    """
        The callback function for setting radar layer
        """
    print("=============================================================set_rad_layer")
    if msg.data == "1":
        state = "10001111"
    elif msg.data == "0":
        state = "11111111"
    SerialReq_radar_layer(state)

def set_io_CB(msg):
    print("===============================================================set_io_CB ", msg.data )
    data_s = msg.data.split(" ")
    SerialReq_06(data_s)

#------------------------------------------------#

def try_connecting_tcp_device():
    """
    This function is for constructing the tcp communication.
    """
    global tcp_host, tcp_port
    global tcp_socket
    global is_serial_device_available

    # Try to close the device
    if not tcp_socket is None:

        send_data_list = (0x0001, 0x0000, 0x0006, 0x01, 0x06, 606, 2) #stop
        book_request("motion",
                send_data_list,
                send_data_frame_format='>HHHBBHh',
                rec_data_frame_length=6,
                rec_data_frame_format='>BBHH')
        try:
            tcp_socket.close()
            rospy.loginfo("[Base] tcp_socket closed.")
        except:
            pass
    # Try to construct the tcp connection
    is_serial_device_available = False

    if (not is_serial_device_available) and (not rospy.is_shutdown()):
        try:
            tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            tcp_socket.connect((tcp_host, tcp_port))            
            is_serial_device_available = True
            rospy.loginfo("[Base] === TCP socket started ===")
        except:
            rospy.loginfo("[Base] TCP socket is not available, retrying...")                    
    #

def try_connecting_serial_device():
    """
    This function is for constructing the serial communication with some persistance tries.
    """
    global serial_device_name, serial_baud_rate
    global serialDev
    global is_serial_device_available

    # Try to close the device
    if not serialDev is None:
        try:
            serialDev.close()
            rospy.loginfo("[Base] Serial closed.")
        except:
            pass
    # Try to construct the serial connection
    is_serial_device_available = False
    count_serial_connection = 0
    while (not is_serial_device_available) and (not rospy.is_shutdown()) and (count_serial_connection < 100):
        try:
            serialDev = serial.Serial(serial_device_name, serial_baud_rate)
            is_serial_device_available = True
            rospy.loginfo("[Base] === Serial port started ===")
        except:
            rospy.loginfo("[Base] Serial port is not available, retrying...")
            count_serial_connection += 1;
            # is_serial_device_available = False
            rospy.sleep(0.2) # Sleep 0.2 seconds for waiting the parameters loading
    #
def reset_MCU():
    """
    This function reset the MCU by sequencially sending serial commands.

    This function takes for about 1.2 sec.
    """
    # Reset the ST-NUCLEO board

    # Sleep 0.2 seconds for waiting commands to be sent
    rospy.sleep(0.2)



def construct_connection_and_reset_MCU():
    """
    This function try to construct the serial connection and try to reset MCU.

    !!! Note: This function is a blocking one for continuously trying constructing a connection.
              For the reset part, it takes for about 1.2 sec.
    """
    # Try to construct the serial connection
    #try_connecting_serial_device()
    # Try to construct the tcp connection
    try_connecting_tcp_device()
    # Reset MCU by serial commands
    reset_MCU()

    if is_serial_device_available:
        """
        send_data_list = (0x0001, 0x0000, 0x0006, 0x01, 0x06, 611, 0) #send y, speed -8000 ~ 8000
        book_request("motion",
                send_data_list,
                send_data_frame_format='>HHHBBHh',
                rec_data_frame_length=6,
                rec_data_frame_format='>BBHH')

        send_data_list = (0x0001, 0x0000, 0x0006, 0x01, 0x06, 610, 0) #send x, angle -4000 ~ 4000
        book_request("motion",
                send_data_list,
                send_data_frame_format='>HHHBBHh',
                rec_data_frame_length=6,
                rec_data_frame_format='>BBHH')
        """
        # rospy.sleep(2)
        
        # send_data_list = (0x0001, 0x0000, 0x0006, 0x01, 0x06, 606, 2) #stop
        # book_request("motion",
        #         send_data_list,
        #         send_data_frame_format='>HHHBBHh',
        #         rec_data_frame_length=6,
        #         rec_data_frame_format='>BBHH')

        # rospy.sleep(10)        

        send_data_list = (0x0001, 0x0000, 0x0006, 0x01, 0x06, 606, 1) #start
        book_request("motion",
                send_data_list,
                send_data_frame_format='>HHHBBHh',
                rec_data_frame_length=12,
                rec_data_frame_format='>HHHBBHh')






#------------------------------------#

if __name__ == '__main__':

    # global current_time
    # global last_time
    # global serialDev
    # global is_serial_device_available

    """
    default_wheelSeparation = 0.412
    default_wheelRadius = 0.075
    default_ADin2volt_correctionRatio = 1.0
    """
    default_rate = 20 #1000 # 100 Hz

    # Initialize the node
    rospy.init_node("agv_base_serial",anonymous = False)

    # Time stamps
    current_time = rospy.Time.now()
    last_time = rospy.Time.now()

    # Load parameters
    # Private parameters
    loop_rate = rospy.get_param('~rate', default_rate)
    is_broadcasting_tf = rospy.get_param('~is_broadcasting_tf', is_broadcasting_tf)

    # Logs for displaying parameters
    #---------------------------------#
    print("is_broadcasting_tf = %s" % str(is_broadcasting_tf))
    #---------------------------------#


    # Deadzone on encoder datas
    is_using_omega_threshold = False
    omega_threshold = 0.0
    try:
        omega_threshold = rospy.get_param("~omega_threshold")
        if omega_threshold > 0.0:
            is_using_omega_threshold = True
        else:
            omega_threshold = 0.0
            # is_using_omega_threshold = False
        rospy.loginfo("[Base] omega_threshold from param = %f", omega_threshold)
    except:
        # is_using_omega_threshold = False
        rospy.loginfo("[Base] No param: omega_threshold")
    rospy.loginfo( "[Base] %s omega_threshold" % ("Enable" if is_using_omega_threshold else "Disable") )

    # Processing parameters
    #-------------------------------#
    #-------------------------------#


    # Update the internal joint state once, no matter the base is ready or not
    #----------------------------------------------------------#
    # send_single_joint_state("fork_plat_joint", 0.0)
    #----------------------------------------------------------#


    # Initializations of MCU
    #----------------------------------------------------------#
    construct_connection_and_reset_MCU()
    # Publish the status of serial port
    serialOnlinePub.publish(is_serial_device_available)
    #----------------------------------------------------------#
    # end Initialization of MCU

    # Pause move (output)
    pause_move_pub.publish(False)

    # Force stopping (pause), (input)
    is_lock = False
    # Log for lock Status
    rospy.loginfo("[Base] Base is %s." % ("locked" if is_lock else "unlocked") )

    # Initialize the fork plat
    #-------------------------------#
    # set_lift_height_CB(Float64(0.5))
    # get_lift_height_CB(Empty())
    # Start the thread for periodically sending the joint_states of the Lift
    # _t_joint_state = threading.Thread(target=joint_state_sending_worker)
    # _t_joint_state.daemon = True
    # _t_joint_state.start()
    #-------------------------------#

    # ROS interfaces
    #---------------------#
    # Subscribers
    rospy.Subscriber("/base/lock", Bool, _base_lock_CB) # This topic is mainly send to base_serial
    rospy.Subscriber('/cmd_vel', Twist, cmd_vel_CB) # Command velocity
    # rospy.Subscriber('/base/set_lift_height', Float64, set_lift_height_CB)
    # rospy.Subscriber('/base/get_lift_height', Empty, get_lift_height_CB)
    # Pause move
    rospy.Subscriber('/base/update_pause_move', String, update_pause_move_CB)
    rospy.Subscriber('/base/reset_pause_move', Empty, reset_pause_move_CB)
    rospy.Subscriber('/base/alarm_lamp', String, set_alarm_lamp_CB)
    rospy.Subscriber('/base/io_output_feedback', Empty, get_io_output_feedback_CB)
    rospy.Subscriber('/base/turn_sig', String, set_turn_signal_CB)
    rospy.Subscriber('/base/power_state', Empty, get_power_state_CB)
    rospy.Subscriber("/base/set_radar_layer", String, set_radar_layer_CB)
    rospy.Subscriber("/base/io_cmd", String, set_io_CB)
    #---------------------#




    # Sleep, relese the resources
    rate = rospy.Rate(loop_rate) # 100hz
    #
    health_check_counter = 0
    while not rospy.is_shutdown():
        
        #SerialQue_rpm()
        communication_engine()
        
        # rospy.loginfo('serail reading')

        # Health check
        health_check_counter += 1
        if health_check_counter >= int(1.0*loop_rate): # 1.0 sec.
            health_check_counter = 0
            # Publish the status of pause_move
            pause_move_status_pub.publish(json.dumps(pause_move_status_dict))
            # health_check()
            # Try to reconstruct the connection
            if not is_serial_device_available:
                construct_connection_and_reset_MCU()
                # health_check again
                # health_check()
            #
        # Sleep, relese the resources
        rate.sleep()


    send_data_list = (0x0001, 0x0000, 0x0006, 0x01, 0x06, 606, 2) #stop
    book_request("motion",
                send_data_list,
                send_data_frame_format='>HHHBBHh',
                rec_data_frame_length=12,
                rec_data_frame_format='>HHHBBHh')

    try:
        serialDev.close()
        tcp_socket.close()
        rospy.loginfo("serial and tcp socket closed")
    except:
        pass
