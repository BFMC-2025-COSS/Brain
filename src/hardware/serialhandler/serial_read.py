import rospy
import time
import math
import serial
from std_msgs.msg import Float32, Float64
from bfmc.msg import bfmc_imu
import numpy as np


class SerialReadNode:
    def __init__(self, port="/dev/ttyACM0", baudrate=115200):
        rospy.init_node("serial_read_node", anonymous=True)
        rospy.loginfo("Serial Read Node Started")

        self.serial_port = serial.Serial(port, baudrate, timeout=0.1)

        self.speed_pub = rospy.Publisher("/sensor/speed",Float32,queue_size=10)
        self.seral_speed_pub = rospy.Publisher("/speed",Float32,queue_size=10)
        self.imu_pub = rospy.Publisher("/BFMC_imu",bfmc_imu,queue_size=10)
        self.Q = 0.7
        self.R = 1.3
        self.P = 2.0
        self.K = 0.0

        self.x_est_last = 0.0
        self.x_est = 0.0

        self.count = 0
        self.speed_avg = 0.0
        self.start = time.time()
    def KalmanF(self,speed):
        self.P = self.P+self.Q
        self.K = self.P / (self.P + self.R)
        self.x_est = self.x_est_last + self.K * (speed - self.x_est_last)
        self.P = (1 - self.K) * self.P
        self.x_est_last = self.x_est
        return self.x_est



    def sendqueue(self,buff):
        try:
            action, value = buff.split(":") # @action:value;;
            action = action[1:]
            value = value[:-2]
            if action == "speedSensor":
                speed = value.split(",")[0]
                F_speed = self.KalmanF(float(speed))
                F_speed = (F_speed/20)*(math.pi*6.5)/70

                #speed = (float(speed)/20)*(math.pi*6.1)/60 
                #rospy.loginfo(f"Received speed: {speed}")
                self.speed_pub.publish(F_speed)
            elif action == "speed":
                serial_speed = value.split(",")[0]
                if self.isFloat(serial_speed):
                    #print("seral:",serial_speed)
                    self.seral_speed_pub.publish(float(serial_speed))
            elif action == "imu":
                splittedValue = value.split(";")
                if(len(buff)>20):
                    data = {
                        "roll": splittedValue[0],
                        "pitch": splittedValue[1],
                        "yaw": splittedValue[2],
                        "accelx": splittedValue[3],
                        "accely": splittedValue[4],
                        "accelz": splittedValue[5],
                    }
                imu_msg = bfmc_imu()
                imu_msg.header.stamp = rospy.Time.now()
                imu_msg.header.frame_id = "imu_link"
                imu_msg.roll = float(data["roll"])
                imu_msg.pitch = float(data["pitch"])
                imu_msg.yaw = float(data["yaw"])
                imu_msg.accelx = float(data["accelx"])
                imu_msg.accely = float(data["accely"])
                imu_msg.accelz = float(data["accelz"])

                self.imu_pub.publish(imu_msg)
        except Exception as e:
            rospy.logerr(f"Error processing serial data: {e}")

    def run(self):
        buff =""
        isResponse = False
        self.start = time.time()

        while not rospy.is_shutdown():
            read_chr = self.serial_port.read()
            try:
                read_chr = read_chr.decode("ascii")
                if read_chr == "@":
                    isResponse = True
                    buff = ""
                elif read_chr == "\r":
                    isResponse = False
                    if len(buff) != 0:
                        self.sendqueue(buff)
                if isResponse:
                    buff += read_chr
            except Exception as e:
                print(e)

    def isFloat(self, string):
        try: 
            float(string)
        except ValueError:
            return False
        
        return True
            
if __name__ == "__main__":
    node = SerialReadNode()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
