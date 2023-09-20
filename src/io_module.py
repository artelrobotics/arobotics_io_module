#!/usr/bin/env python3

import rospy
import serial
import re
from arobotics_io_module.msg import ReadInputs, SetOutput, SetOutputs

class IOModule:
    def __init__(self, port, baudrate = 9600, freq = 10) -> None:
        self.port = port
        self.baudrate = baudrate
        self.freq = freq
        self.rate = rospy.Rate(self.freq)
        self.serial_device = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=1)
        self.inputs_pub = rospy.Publisher("~inputs_state", ReadInputs, queue_size=10)
        rospy.Subscriber("~set_outpt", SetOutput, self.set_output_clb)
        rospy.Subscriber("~set_outpts", SetOutputs, self.set_outputs_clb)
        rospy.loginfo(f"IO module port:{self.port} baud:{self.baudrate}")

    def set_output_clb(self, msg):
        if self.serial_device.is_open:
            data = f"#S{msg.pin}{'H' if msg.level else 'L'}\n"
            self.serial_device.write(data.encode('utf-8'))
        else:
            rospy.logwarn(f'Unable to set pin {msg.pin} to {msg.level}')

    def set_outputs_clb(self, msg):
        if self.serial_device.is_open:
            data = f"#SA{'H' if msg.level else 'L'}\n"
            self.serial_device.write(data.encode('utf-8'))
        else:
            rospy.logwarn(f'Unable to set pins to {msg.level}')

    def read_inputs(self):
        if self.serial_device.is_open:
            data = "#RA\n"
            self.serial_device.write(data.encode('utf-8'))
            line = self.serial_device.readline().decode('utf-8')
            if line:
                key_value_pairs = re.findall(r'(\d+): ([A-Z]+)', line)
                result_list = [value == 'LOW' for key, value in key_value_pairs]
                return result_list
            else:
                return None
        else:
            return None

    def publish_inputs(self):
        while not rospy.is_shutdown():
            pub_msg = ReadInputs()
            pins_state = self.read_inputs()
            if pins_state is None:
                rospy.logwarn(f'Unable to read inputs state')
            else:
                pub_msg.pin_1 = pins_state[0]
                pub_msg.pin_2 = pins_state[1]
                pub_msg.pin_3 = pins_state[2]
                pub_msg.pin_4 = pins_state[3]
                pub_msg.pins  = pins_state
                self.inputs_pub.publish(pub_msg)
                self.rate.sleep()

    def shutdown(self):
        if self.serial_device.is_open:
            data = f"#SAL\n"
            self.serial_device.write(data.encode('utf-8'))
            rospy.logwarn("Set all outputs to False")
            self.serial_device.close()