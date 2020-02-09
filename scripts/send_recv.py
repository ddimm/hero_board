#!/usr/bin/env python3


import rospy
import queue
import threading

import serial
import traceback
from hero_board.msg import MotorVal
from utils.protocol import var_len_proto_recv, var_len_proto_send

MOTOR_REC_NAME = "motor_commands"
MOTOR_COMMAND_PUB = "/motor/output"
MOTOR_PUB_NAME = "motor_volts"
MOTOR_VOLT_NAME = "motor/current"

motor_signals = queue.Queue(5)

try:
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=None)
except Exception as e:
    traceback.print_exc()
    exit(-1)


def hero_recv():
    '''
        runs in a thread to read data from the serial port
    '''
    while not rospy.is_shutdown():
        motor_vals = ser.read(ser.inWaiting())
        to_send = var_len_proto_recv(motor_vals)
        for x in to_send:
            motor_signals.put(list(x))
            



def process_motor_values(motor_vals):
    '''takes in the MotorVal message as a parameter and sends the bytes
    to serial'''
    m_val = motor_vals.motorval
    rospy.loginfo("motor value: %s",m_val)
    ser.write(var_len_proto_send(m_val))


def motor_listener():
    '''
        subscribes to the motor command publisher and passes the MotorVal
        message to the callback
    '''
    rospy.init_node(MOTOR_REC_NAME, anonymous=True)
    rospy.Subscriber(MOTOR_COMMAND_PUB, MotorVal, process_motor_values)

if __name__=="__main__":
    try:
        motor_listener()
        recv_thread = threading.Thread(target=hero_recv)
        recv_thread.daemon=True
        recv_thread.start()

        pub = rospy.Publisher(MOTOR_VOLT_NAME, MotorVal)
        print("starting publisher")
        while not rospy.is_shutdown():
            if motor_signals.empty():
                continue
            data = motor_signals.get()
            rospy.loginfo("current value: %s",data)
            pub.publish(MotorVal(data))

    except KeyboardInterrupt as k:
        traceback.print_exc()
    except Exception as e:
        traceback.print_exc()
    finally:
        ser.close()
        exit(-1)
