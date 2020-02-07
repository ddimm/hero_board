#!/usr/bin/env python3
import rospy
import threading
import queue
import serial
from hero_board.msg import MotorVal
from utils.protocol import var_len_proto_recv
import struct
import random

motor_signals = queue.Queue(10)
should_terminate = threading.Event()
def hero_recv():
    ser =None
    try:

        ser  = serial.Serial('/dev/ttyUSB0', 115200, timeout=1) 
    except Exception as e:
        print(e)
        rospy.signal_shutdown("couldn't connect to serial")
        return
    while not rospy.is_shutdown():
        motor_vals = ser.read(ser.inWaiting())
        motor_signals.put(var_len_proto_recv(motor_vals))
    ser.close()

def dummy_recv():
    while not rospy.is_shutdown():
        motor_vals = struct.pack('1i', [random.randint(0,32)])
        motor_vals = var_len_proto_send(motor_vals)
        motor_signals.put(motor_vals)
    

def motor_pub():
    pub = rospy.Publisher('motor_volts', MotorVal, queue_size=10)
    rospy.init_node('motor_volts', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if motor_signals.empty():
            continue
        data = motor_signals.get()
        rospy.loginfo(data)
        pub.publish(MotorVal(data))
        rate.sleep()


if __name__=="__main__":
    recv_thread = threading.Thread(target=hero_recv)
    recv_thread.start()
    # dummy_thread = threading.Thread(target=dummy_recv)
    # dummy_thread.start()
    try:
        motor_pub()
    except rospy.ROSInterruptException as e:
        print(e)
        exit(1)
    except Exception as e:
        print(e)
        exit(1)