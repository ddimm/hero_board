#!/usr/bin/env python3
import rospy
import serial
import traceback
from hero_board.msg import MotorVal
from utils.protocol import var_len_proto_recv, var_len_proto_send

MOTOR_REC_NAME = "motor_commands"
MOTOR_COMMAND_PUB = "/motor/output"
MOTOR_VOLT_NAME = "/motor/current"

try:
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=None)
except Exception as e:
    traceback.print_exc()
    exit(-1)

def process_motor_values(motor_vals):
    '''
    takes in the MotorVal message as a parameter and sends the bytes
    to serial
    '''
    m_val = motor_vals.motorval
    rospy.loginfo("motor value: %s", m_val)
    ser.write(var_len_proto_send(m_val))


if __name__ == "__main__":
    try:
        '''
        subscribes to the motor command publisher and passes the MotorVal
        message to the callback
        '''
        rospy.init_node(MOTOR_REC_NAME, anonymous=True)
        rospy.Subscriber(MOTOR_COMMAND_PUB, MotorVal, process_motor_values)

        print("starting publisher")
        # no need to used a thread here
        # as per testing, main thread works fine
        # ros publisher queue can be used to limit the number of messages
        pub = rospy.Publisher(MOTOR_VOLT_NAME, MotorVal, queue_size=5)
        while not rospy.is_shutdown():
            motor_vals = ser.read(ser.inWaiting())
            to_send = var_len_proto_recv(motor_vals)
            for x in to_send:
                pub.publish(MotorVal(list(x)))
        
    except KeyboardInterrupt as k:
        traceback.print_exc()
    except Exception as e:
        traceback.print_exc()
    finally:
        ser.close()
        exit(-1)
