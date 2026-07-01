#!/usr/bin/env python3

import rospy
import serial
import struct
import yaml
import threading
import time
from geometry_msgs.msg import Twist
from motor_rmd.srv import SetPosition, SetPositionResponse
from motor_rmd.srv import SetVelocity, SetVelocityResponse
from motor_rmd.srv import SetTorque, SetTorqueResponse
from motor_rmd.srv import StopMotor, StopMotorResponse
from motor_rmd.srv import ReadHomePosition, ReadHomePositionResponse
from motor_rmd.srv import ReadPosition, ReadPositionResponse
from motor_rmd.srv import StatusError, StatusErrorResponse
from motor_rmd.srv import StatusMotor, StatusMotorResponse


ERROR_STATE_DICT = {
    0x0002: "Motor stall",
    0x0004: "Low pressure",
    0x0008: "Overvoltage",
    0x0010: "Overcurrent",
    0x0040: "Power overrun",
    0x0080: "Calibration parameter writing error",
    0x0100: "Speeding",
    0x1000: "Motor temperature over temperature",
    0x2000: "Encoder calibration error"
}

# Función para calcular el CRC16
def crc16(data):
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for _ in range(8):
            if (crc & 0x0001) != 0:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc

class RMDControl:
    def __init__(self):
        rospy.init_node('rmd_control_node')
        
        self.port = rospy.get_param('~port', '/dev/ttyUSB0')
        self.baudrate = rospy.get_param('~baudrate', 115200)
        self.id = rospy.get_param('~id', 1)
        self.max_speed = 420
        self.latest_velocity = 0  # last velocity received 
        self.lock = threading.Lock() 
        self.serial_connection = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0.1
        )
        
        self.commands = self.load_commands(rospy.get_param('~commands_file'))
        #start thread
        self.running = True
        self.thread = threading.Thread(target=self.process_velocity)
        self.thread.daemon = True
        self.thread.start()
        
        rospy.Service('/rmd_motor/cmd_vel', SetVelocity, self.handle_set_velocity)
        rospy.Service('/rmd_motor/set_position', SetPosition, self.handle_set_position)
        rospy.Service('/rmd_motor/set_torque', SetTorque, self.handle_set_torque)
        rospy.Service('/rmd_motor/stop_motor', StopMotor, self.handle_stop_motor)
        rospy.Service('/rmd_motor/read_home_position', ReadHomePosition, self.handle_read_home_position)
        rospy.Service('/rmd_motor/read_position', ReadPosition, self.handle_read_position)
        rospy.Service('/rmd_motor/status_error', StatusError, self.handle_status_error)
        rospy.Service('/rmd_motor/status_motor', StatusMotor, self.handle_status_motor)

    def load_commands(self, file_path):
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)

    def send_command(self, command):
        self.serial_connection.reset_input_buffer()
        self.serial_connection.reset_output_buffer()
        packet = struct.pack(
            'BBB8B',
            command['header'],
            self.id,  # Suponiendo que el ID del motor es 1
            command['length'],
            *command['data']
        )
        crc = crc16(packet)
        crc_l = crc & 0xFF
        crc_h = (crc >> 8) & 0xFF
        packet += struct.pack('BB', crc_l, crc_h)
        self.serial_connection.write(packet)
        response = self.serial_connection.read(14)  
        rospy.loginfo("Sending command: {}".format(packet.hex()))
        return response

    def handle_set_velocity(self, req):
        """ Servicio para recibir comandos de velocidad, pero solo guarda el último. """
        with self.lock:
            self.latest_velocity = req.vel  # Solo se guarda el último comando recibido
            return SetVelocityResponse(success=True)
        return SetVelocityResponse(success=False)

    def process_velocity(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.running:
            with self.lock:
                speed = max(-self.max_speed, min(self.max_speed, self.latest_velocity))

            command = self.commands['set_speed'].copy()
            
            speed_units = speed * 100 #by 100 times according to the protocol,
            speed_bytes = speed_units.to_bytes(4, byteorder='little', signed=True)
            command['data'][4] = speed_bytes[0]
            command['data'][5] = speed_bytes[1]
            command['data'][6] = speed_bytes[2]
            command['data'][7] = speed_bytes[3]
            response = self.send_command(command)
            rospy.loginfo("velocity send: {}".format(speed))
            rate.sleep()


    def handle_set_position(self, req):
        command = self.commands['set_position'].copy()
        position = req.position*100  # Position in degrees by 100 times according to 0.01degree/LSB
        position_bytes = position.to_bytes(4, byteorder='little', signed=True)
        speed_bytes = self.max_speed.to_bytes(2, byteorder='little', signed=True)
        command['data'][2] = speed_bytes[0]
        command['data'][3] = speed_bytes[1]
        command['data'][4] = position_bytes[0]
        command['data'][5] = position_bytes[1]
        command['data'][6] = position_bytes[2]
        command['data'][7] = position_bytes[3]
        response = self.send_command(command)
        rospy.loginfo("Position command response: {}".format(response.hex()))
        return SetPositionResponse(success=True)

    def handle_set_torque(self, req):
        #send current
        command = self.commands['set_torque'].copy()
        torque = int(req.torque * 100)  # Ajusta la escala si es necesario
        torque_bytes = torque.to_bytes(2, byteorder='little', signed=True)
        command['data'][4] = torque_bytes[0]
        command['data'][5] = torque_bytes[1]
        response = self.send_command(command)
        rospy.loginfo("Torque command response: {}".format(response))
        return SetTorqueResponse(success=True)

    def handle_stop_motor(self, req):
        command = self.commands['stop_motor'].copy()
        response = self.send_command(command)
        return StopMotorResponse(success=True)

    def handle_read_home_position(self, req):
        command = self.commands['read_home_position'].copy()
        response = self.send_command(command)
        rospy.loginfo("Received position response: {}".format(response.hex()))
        pos = struct.unpack('i', response[7:11])[0]
        return ReadHomePositionResponse(position=pos)

    def handle_read_position(self, req):
        command = self.commands['read_position'].copy()
        response = self.send_command(command)
        position = struct.unpack('i', response[7:11])[0]  # Ajusta según el formato de la respuesta
        rospy.loginfo("Received position response: {}".format(response.hex()))
        return ReadPositionResponse(position=position)

    def handle_status_error(self, req):
        command = self.commands['status_error'].copy()
        response = self.send_command(command)
        temperature = struct.unpack('b', response[4:5])[0]  # int8_t
        brake = response[6]  # uint8_t
        voltage = struct.unpack('h', response[7:9])[0] * 0.1  # uint16_t, en voltios
        error_state = struct.unpack('h', response[9:11])[0]  # uint16_t

        # Obtener la descripción del error directamente del diccionario
        error_description_str = ERROR_STATE_DICT.get(error_state, "Unknown error" if error_state != 0 else "No errors")
        return StatusErrorResponse(
            temperature = temperature,
            brake = brake,
            voltage = voltage,
            error_state = error_description_str
        )

        
    def handle_status_motor(self, req):
        command = self.commands['status_motor'].copy()
        response = self.send_command(command)
        rospy.loginfo("Received status motor response: {}".format(response.hex()))
        temperature = struct.unpack('b', response[4:5])[0]  # int8_t
        current = struct.unpack('H', response[5:7])[0]  * 0.01
        motor_speed = struct.unpack('h', response[7:9])[0]  
        motor_position = struct.unpack('h', response[9:11])[0] 

        return StatusMotorResponse(
            temperature = temperature,
            current = current,
            speed = motor_speed,
            position = motor_position
        )
    def stop(self):
        self.running = False
        self.thread.join()
        self.serial_connection.close()

    def run(self):
        rospy.spin()
        self.stop()

if __name__ == '__main__':
    rmd_control = RMDControl()
    rmd_control.run()