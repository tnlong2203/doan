#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray, Float32MultiArray, Int64MultiArray
from math import *
from pymodbus.client import ModbusSerialClient
import concurrent.futures
import time


class MotorController:
    def __init__(self):
        # self.CHASSIS_RADIUS = 0.258
        self.CHASSIS_RADIUS = 0.2673
        self.WHEEL_RADIUS = 0.1015
        self.jog = [0, 0, 0]
        self.prev_alarm_data = None
        self.client = self.initialize_modbus_client()

        rospy.init_node('cmd_vel_subscriber_node', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        self.encoder_pub = rospy.Publisher('encoder_pub', Int64MultiArray, queue_size=10)
        self.speed_pub = rospy.Publisher('speed_pub', Float32MultiArray, queue_size=10)
        self.alarm_pub = rospy.Publisher('alarm_pub', Int32MultiArray, queue_size=10)

        rospy.on_shutdown(self.clean_shutdown)
        self.run()
        rospy.spin()


    def initialize_modbus_client(self):
        return ModbusSerialClient(method="rtu", port="/dev/ttyS3", baudrate=115200, stopbits=1, parity="N", timeout=0.2)

    def connect_to_modbus(self):
        if not self.client.connect():
            rospy.logerr("Failed to connect to Modbus device")
            return False
        return True

    def setup_motors(self):
        for unit in [1, 2, 3]:
            self.client.write_register(0x7C, 0xD8, unit=unit)                       # stop jogging
            self.client.write_registers(0x7C, [0x0098, 0x0000, 0x0000], unit=unit)  # reset encoder
            self.client.write_registers(338, [0x0000, 0x010E], unit=unit)           # motor dec
            self.client.write_registers(340, [0x0000, 0x010E], unit=unit)           # motor acc
            self.client.write_register(0x7C, 0x9F, unit=unit)                       # motor enable     
            self.client.write_registers(342, [0x0000, 0x0000], unit=unit)           # set speed is zero
            self.client.write_register(0x7C, 0x96, unit=unit)                       # start jogging

        rospy.loginfo("Motors ready!")

    # subscribe topic cmd_vel
    def cmd_vel_callback(self, data):                       
        linear_x, linear_y, angular_z = data.linear.x, data.linear.y, data.angular.z

        wheels = [((-sqrt(3) / 2) * linear_x + linear_y / 2 + self.CHASSIS_RADIUS * angular_z) / -self.WHEEL_RADIUS,
                (0 * linear_x - linear_y + self.CHASSIS_RADIUS * angular_z) / -self.WHEEL_RADIUS,
                ((sqrt(3) / 2) * linear_x + linear_y / 2 + self.CHASSIS_RADIUS * angular_z) / -self.WHEEL_RADIUS]

        self.jog = [int(round((wheels[i] * 240 * 20 * 2.941176471) / (pi * 2), 0)) for i in range(3)]

    def write_velocity_for_unit(self, unit, int_value):
            self.client.write_registers(342, [int_value >> 16, int_value & 0xFFFF], unit=unit)


    def write_velocity(self):
        hex_values = [f'0x{value & 0xFFFFFFFF:08X}' for value in self.jog]
        int_values = [int(hex_value, 16) for hex_value in hex_values]
        for unit, int_value in enumerate(int_values, start=1):
            self.client.write_registers(342, [int_value >> 16, int_value & 0xFFFF], unit=unit)

    def publish_speed(self):
        speed_data = Float32MultiArray() 
        speed_data.data = []  

        for unit in [1, 2, 3]:
            speed = self.client.read_input_registers(16, 1, unit=unit)
            if speed.registers[0] <= 32767:
                speed = round(speed.registers[0] * pi * self.WHEEL_RADIUS * 2 / (240 * 20 * 2.941176471), 2)
            else:
                speed = round((speed.registers[0] - 2 ** 16) * pi * self.WHEEL_RADIUS * 2 / (240 * 20 * 2.941176471), 2)
            speed_data.data.append(speed)

        self.speed_pub.publish(speed_data)

    def publish_encoders(self):
        encoder_data = Int64MultiArray() 
        encoder_data.data = [] 

        for unit in [1, 2, 3]:
            encoder = self.client.read_input_registers(10, 2, unit=unit)
            combined_hex = f"0x{encoder.registers[0]:04X}{encoder.registers[1]:04X}"
            combined_decimal = int(combined_hex, 16)

            if combined_decimal > ((2**32)/2 - 1):
                combined_decimal = combined_decimal - 2**32

            encoder_data.data.append(combined_decimal)
        self.encoder_pub.publish(encoder_data)

    def publish_alarm(self):
        alarm_data = Int32MultiArray() 
        alarm_data.data = []
        error_flag = False

        for unit in [1, 2, 3]:
            alarm = self.client.read_input_registers(0, 2, unit=unit)
 
            alarm_data_value = int(f"{alarm.registers[0]:04X}{alarm.registers[1]:04X}", 16)
            if alarm_data_value != 0 and error_flag != True:
                alarm_result = int(log2(alarm_data_value))
                rospy.logwarn(f"Driver {unit} encountered an error!")
                error_flag = True
            else:
                alarm_result = alarm_data_value

            alarm_data.data.append(alarm_result)
           

        if self.prev_alarm_data != alarm_data.data:
            self.alarm_pub.publish(alarm_data)
            self.prev_alarm_data = alarm_data.data
            

        if error_flag == True:
            for unit in [1, 2, 3]:
                alarm = self.client.read_input_registers(0, 2, unit=unit)
                alarm_data_value = int(f"{alarm.registers[0]:04X}{alarm.registers[1]:04X}", 16)
                if alarm_data_value == 0:
                    rospy.loginfo(f"No error on driver {unit}")
                    error_flag = False
                else:
                    rospy.logerr("Error still remains")                 

    def run(self):
        if not self.connect_to_modbus():
            return

        try:
            rospy.loginfo("Connected to Modbus device")
            self.setup_motors()
            rate = rospy.Rate(50)
            
            while not rospy.is_shutdown():
                self.publish_alarm()
                self.publish_encoders() 
                self.write_velocity()
                self.publish_speed()
                rate.sleep()
                

        except Exception as e:
            rospy.logerr(f"An error occurred: {e}")
        except KeyboardInterrupt:
            self.client.close()
            
    def clean_shutdown(self):
        rospy.loginfo("System is shutting down. Stopping robot...")
        
        time.sleep(1)  # Adjust the delay as needed

        for unit in [1, 2, 3]:
                self.client.write_registers(342, [0x0000, 0x0000], unit=unit)
                self.client.write_register(0x7C, 0xD8, unit=unit)                   # stop jogging 

         # Close the connection to the Modbus device
        if self.client.is_socket_open():
            self.client.close()

    rospy.loginfo("Robot stopped. Shutdown complete.")

if __name__ == '__main__':
    motor_controller = MotorController()
    