#!/usr/bin/env python3
import rospy
import serial
import traceback
from hero_board.msg import MotorVal
from hero_board.srv import ControlServer, ControlServerResponse, GetState, GetStateResponse
from utils.protocol import var_len_proto_recv, var_len_proto_send

MOTOR_REC_NAME = "motor_commands"
MOTOR_COMMAND_PUB = "/motor/output"
MOTOR_VOLT_NAME = "/motor/current"
AI_MOTOR_CONTROL = "/motor/cmd_vel"
manual_sub=None
auto_sub =None
current_state = 'manual'
try:
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=None)
except Exception as e:
    traceback.print_exc()
    exit(-1)
def handle_state_request(req):
    global current_state
    return GetStateResponse(current_state)
def process_motor_values(motor_vals):
    '''
    takes in the MotorVal message as a parameter and sends the bytes
    to serial
    '''
    m_val = motor_vals.motorval
    rospy.loginfo("motor value: %s", m_val)
    ser.write(var_len_proto_send(m_val))

def process_auto_motor_values(motor_vals):
    m_val = motor_vals.motorval
    rospy.loginfo('motor value: %s', m_val)
    ser.write(var_len_proto_send(m_val))

def change_control(req):
    global auto_sub
    global manual_sub
    global current_state
    if req.changeControl == 'manual':
        print('changing to manual control')
        if auto_sub:
            auto_sub.unregister()
        manual_sub = rospy.Subscriber(MOTOR_COMMAND_PUB, MotorVal, process_motor_values)
        current_state = 'manual'
        return ControlServerResponse('changing to manual control')

    elif req.changeControl == 'automatic':
        print('changing to automatic')
        current_state='automatic'
        if manual_sub:
            manual_sub.unregister()
        
        auto_sub = rospy.Subscriber(AI_MOTOR_CONTROL, MotorVal, process_auto_motor_values)
        return ControlServerResponse('changing to automatic control')



def control_server():
    serv = rospy.Service("change_control", ControlServer, change_control)
    print("server started")
    state_serv = rospy.Service('current_state', GetState, handle_state_request)
    print('started state service')

if __name__ == "__main__":
    try:
        '''
        subscribes to the motor command publisher and passes the MotorVal
        message to the callback
        '''
        control_server()
        rospy.init_node(MOTOR_REC_NAME, anonymous=True)
        manual_sub =rospy.Subscriber(MOTOR_COMMAND_PUB, MotorVal, process_motor_values)
        
        print("starting publisher")
        # no need to used a thread here. As per testing, main thread works fine
        # ros publisher queue can be used to limit the number of messages
        pub = rospy.Publisher(MOTOR_VOLT_NAME, MotorVal, queue_size=5)
        while not rospy.is_shutdown():
            motor_vals = ser.read(ser.inWaiting())
            to_send = var_len_proto_recv(motor_vals)
            for x in to_send:
                pub.publish(MotorVal(list(x)))
        
    except KeyboardInterrupt as k:
        traceback.print_exc()
    finally:
        ser.close()
        exit(-1)
