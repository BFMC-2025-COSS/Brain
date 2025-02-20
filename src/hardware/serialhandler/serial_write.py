#!/usr/bin/env python3

import rospy
import serial
from std_msgs.msg import String
import json
from Brain.src.hardware.serialhandler.threads.messageconverter import MessageConverter

class SerialWrite:
    def __init__(self, port="/dev/ttyACM0", baudrate=115200):
        rospy.init_node("serial_write_node", anonymous=True)
        rospy.loginfo("serial write Node Started")
        self.serial_port = serial.Serial(port, baudrate, timeout=1)
        self.converter = MessageConverter()
        rospy.Subscriber("/serial/command", String, self.write_serial)

    def write_serial(self, msg):
        """ROS 토픽에서 받은 데이터를 Serial로 송신"""
        try:
            dict_msg = json.loads(str(msg.data))
            converted_msg = self.converter.get_command(**dict_msg)
            if converted_msg != "error":
                rospy.loginfo(f"Sending to Serial: {converted_msg.strip()}")
                self.serial_port.write(converted_msg.encode("ascii"))
            else:
                rospy.logerr("Invalid command format.")
        except Exception as e:
            rospy.logerr(f"Error processing command: {e}")

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    writer = SerialWrite()
    try:
        writer.run()
    except rospy.ROSInterruptException:
        pass

