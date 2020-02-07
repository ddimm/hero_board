#!/usr/bin/env python3
import rospy
import threading
import queue
import serial
from hero_board.msg import MotorVal
from utils.protocol import var_len_proto_send, var_len_proto_recv


#name of the receiver node
MOTOR_REC_NAME = "moto_commands"
MOTOR_COMMAND_PUB = "motor_pub"
QUEUE_SIZE = 10

try:

    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
except Exception as e:
    print(e)
    exit(-1)




def process_motor_values(motor_vals):
    '''takes in the MotorVal message as a parameter and sends the bytes
    to serial'''
    m_val = motor_vals.motorval
    rospy.loginfo(m_val)
    ser.write(var_len_proto_send(m_val))




def motor_listener():
    '''
        subscribes to the motor command publisher and passes the MotorVal
        message to the callback
    '''
    rospy.init_node(MOTOR_REC_NAME, anonymous=True)
    rospy.Subscriber(MOTOR_COMMAND_PUB, MotorVal, process_motor_values)
    rospy.spin()


if __name__ == "__main__":
    try:
        motor_listener()
    except KeyboardInterrupt as k:
        print(k)
        ser.close()
        exit(-1)
    except Exception:
        ser.close()
        exit(-1)
