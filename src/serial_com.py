#! /usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

def state_comm(state):	

    if state.data:
    	print('envio el mesanje')
    	comando = bytearray([0x3E, 0x02, 0x08, 0xA4, 0x00, 0xF4, 0x01, 0xA0, 0x8C, 0x00, 0x00])
    	ser.write(comando)
    	time.sleep(1) 
    	respuesta = ser.read_all()
    	print("Respuesta recibida:", respuesta)


def state_motor():

	rospy.init_node('state_comm', anonymous=True)
	rospy.Subscriber("/motor_rmd", Float32, state_comm)
	rospy.spin()

if __name__ == '__main__':
    try:
        state_motor()
    except rospy.ROSInterruptException:
        pass
    finally:
        ser.close() 
    

