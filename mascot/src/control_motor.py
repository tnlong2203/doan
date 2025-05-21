#!/usr/bin/env python3

import rospy
from pymodbus.client import ModbusSerialClient
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from math import sqrt, pi
from std_msgs.msg import Int16
class MotorControlNode:
    def __init__(self):
        rospy.init_node('cmd_vel_subscriber_node', anonymous=True)

        self.CHASSIS_RADIUS = 0.4
        self.WHEEL_RADIUS = 0.1
        self.jog1 = self.jog2 = self.jog3 = 0

        self.client = ModbusSerialClient(
            method="rtu",
            port="/dev/ttyUSB0",
            baudrate=19200,
            stopbits=1,
            parity="N",
            timeout=0.2
        )

        self.cmd_vel_subscriber = rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)

    def control_MBDV_Motor(self, ID, Velocity):
        if not self.client.connect():
            print("Failed to connect to Modbus device")
            return

        hex_value = hex(Velocity & 0xFFFF).upper()
        if len(hex_value) < 6:
            hex_value = '0x' + '0' * (6 - len(hex_value)) + hex_value[2:]

        self.client.write_register(0x7C, 0x9F, slave=ID)
        try:
            if Velocity > 0:
                print("motor clockwise")
                self.client.write_registers(342, [0x0000, int(hex_value, 16)], slave=ID)
            elif Velocity < 0:
                print("motor anticlockwise")
                self.client.write_registers(342, [0xFFFF, int(hex_value, 16)], slave=ID)
            elif Velocity == 0:
                self.client.write_registers(342, [0x0000, 0x0000], slave=ID)
            self.client.write_register(0x7C, 0x96, slave=ID)
        except Exception as e:
            print(f"An error occurred: {e}")

    def cmd_vel_callback(self, data):
        linear_velocity_x = data.linear.x
        linear_velocity_y = data.linear.y
        angular_velocity = data.angular.z

        wheel1 = ((-sqrt(3)/2) * linear_velocity_x + linear_velocity_y/2 + self.CHASSIS_RADIUS * angular_velocity) / self.WHEEL_RADIUS
        wheel2 = ( 0 * linear_velocity_x - linear_velocity_y + self.CHASSIS_RADIUS*  angular_velocity) / self.WHEEL_RADIUS
        wheel3 = ((sqrt(3)/2) * linear_velocity_x + linear_velocity_y/2 + self.CHASSIS_RADIUS * angular_velocity) / self.WHEEL_RADIUS

        self.jog1 = int(round((wheel1 * 240) / (pi/50), 0))
        self.jog2 = int(round((wheel2 * 240) / (pi/50), 0))
        self.jog3 = int(round((wheel3 * 240) / (pi/50), 0))
        print("v1 = ", self.jog1)
        print("v2 = ", self.jog2)
        print("v3 = ", self.jog3)
        print(" ")
        self.control_MBDV_Motor(1, self.jog1)
        self.control_MBDV_Motor(2, self.jog2)
        self.control_MBDV_Motor(3, self.jog3)


    def main(self):
        if not self.client.connect():
            print("Failed to connect to Modbus device")
            return
        print("Connected to Modbus device")
        # try:
        #     while not rospy.is_shutdown():
        #         self.control_MBDV_Motor(1, self.jog1)
        #         self.control_MBDV_Motor(2, self.jog2)
        #         self.control_MBDV_Motor(3, self.jog3)

        # except Exception as e:
        #     print(f"An error occurred: {e}")
        # except KeyboardInterrupt:
        #     print("KeyboardInterrupt: Turn off the motor!")
        #     self.client.write_registers(342, [0x0000, 0x0000], slave=0x01)
        #     self.client.write_registers(342, [0x0000, 0x0000], slave=0x02)
        #     self.client.write_registers(342, [0x0000, 0x0000], slave=0x03)
        #     self.client.write_register(0x7C, 0xD8, slave=0x01)
        #     self.client.write_register(0x7C, 0xD8, slave=0x02)
        #     self.client.write_register(0x7C, 0xD8, slave=0x03)
        #     self.client.close()

if __name__ == '__main__':
    node = MotorControlNode()
    node.main()
