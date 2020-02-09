#!/usr/bin/env python3
import rospy
import threading
import queue
import serial
from hero_board.msg import MotorVal
from utils.protocol import var_len_proto_recv
import struct
import random
import traceback
motor_signals = queue.Queue()
should_terminate = threading.Event()

def hero_recv():
    ser =None
    try:
        ser  = serial.Serial('/dev/ttyUSB0', 115200, timeout=1) 
    except Exception as e:
        print(e)
        traceback.print_exc()
        rospy.signal_shutdown("couldn't connect to serial")
        exit(-1)
        
    while not rospy.is_shutdown():
        motor_vals = ser.read(ser.inWaiting())
        to_send = var_len_proto_recv(motor_vals)
        for x in to_send:
            motor_signals.put(list(x))

    ser.close()

def dummy_recv():
    while not rospy.is_shutdown():
        motor_vals = struct.pack('1i', [random.randint(0,32)])
        
        motor_vals = var_len_proto_send(motor_vals)
        motor_signals.put(motor_vals)
    

def motor_pub():
    pub = rospy.Publisher('motor/current', MotorVal, queue_size=1)
    rospy.init_node('motor_volts', anonymous=True)
    while not rospy.is_shutdown():
        if motor_signals.empty():
            continue
        data = motor_signals.get()
        rospy.loginfo(data)
        pub.publish(MotorVal(data))


if __name__=="__main__":
    recv_thread = threading.Thread(target=hero_recv)
    recv_thread.daemon = True
    recv_thread.start()
    try:
        motor_pub()
    except rospy.ROSInterruptException as e:
        print(e)
        traceback.print_exc()
        recv_thread.join()
        exit(1)
    except Exception as e:
        print(e)
        traceback.print_exc()
        recv_thread.join()
        exit(1)
