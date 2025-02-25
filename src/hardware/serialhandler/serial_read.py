import rospy
import math
import serial
from std_msgs.msg import Float32, Float64
import matplotlib.pyplot as plt
import numpy as np


class SerialReadNode:
    def __init__(self, port="/dev/ttyACM0", baudrate=115200):
        rospy.init_node("serial_read_node", anonymous=True)
        rospy.loginfo("Serial Read Node Started")

        self.serial_port = serial.Serial(port, baudrate, timeout=0.1)

        self.speed_pub = rospy.Publisher("/sensor/speed",Float32,queue_size=10)
        self.seral_speed_pub = rospy.Publisher("/speed",Float32,queue_size=10)
        self.imu_pub = rospy.Publisher("/BFMC_yaw",Float64,queue_size=10)
        self.Q = 0.7
        self.R = 1.3
        self.P = 2.0
        self.K = 0.0

        self.x_est_last = 0.0
        self.x_est = 0.0
        
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(6, 4))
        self.ax.set_xlabel('Sample Index')
        self.ax.set_ylabel('Speed (some unit)')
        self.ax.set_title('Real-time Speed vs Filtered Speed')

        # speed, F_speed를 저장할 리스트
        self.raw_speed_data = []
        self.filtered_speed_data = []

        # 두 개의 라인 객체를 미리 만들어 둔다.
        (self.line_raw,) = self.ax.plot([], [], 'r-', label='Raw Speed')
        (self.line_filtered,) = self.ax.plot([], [], 'b-', label='Filtered Speed')
        self.ax.legend()

    def update_plot(self):
        """
        Matplotlib 라인에 데이터가 들어온 후 호출하여
        그래프를 갱신하는 함수
        """
        # x축 인덱스(샘플 번호)
        x_vals = np.arange(len(self.raw_speed_data))

        # 라인 데이터 갱신
        self.line_raw.set_xdata(x_vals)
        self.line_raw.set_ydata(self.raw_speed_data)
        self.line_filtered.set_xdata(x_vals)
        self.line_filtered.set_ydata(self.filtered_speed_data)

        # 축 범위 자동 설정
        self.ax.relim()
        self.ax.autoscale_view()

        plt.draw()
        # 너무 크게 잡으면 반응이 느리므로 아주 짧게 pause
        plt.pause(0.001)

    def isFloat(self, val):
        try:
            float(val)
            return True
        except:
            return False

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

                self.raw_speed_data.append(float(speed)/120)
                self.filtered_speed_data.append(float(F_speed)/120)
                print("speed:",float(speed)/120,"F:",float(F_speed)/120)
                speed = (float(speed)/20)*(math.pi*6.1)/60 
                #rospy.loginfo(f"Received speed: {speed}")
                self.speed_pub.publish(speed)

                self.update_plot()

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
                self.imu_pub.publish(float(data["yaw"]))
        except Exception as e:
            rospy.logerr(f"Error processing serial data: {e}")

    def run(self):
        buff =""
        isResponse = False

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
