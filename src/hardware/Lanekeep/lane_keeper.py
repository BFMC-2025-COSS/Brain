#!/usr/bin/env python3

import rospy
import cv2
import torch
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float32, String
from cv_bridge import CvBridge
from multiprocessing import Process
import json

from Brain.src.hardware.Lanekeep.threads.utils import OptimizedLaneNet
from Brain.src.utils.lantracker_pi.tracker import LaneTracker
from Brain.src.utils.lantracker_pi.perspective import flatten_perspective

class LaneKeeper(Process):
    def __init__(self):
        super(LaneKeeper, self).__init__()
        rospy.init_node('lane_keeper_node', anonymous=True)

        # ROS Subscribers & Publishers
        self.bridge = CvBridge()
        #self.image_sub = rospy.Subscriber('/camera/color/image_raw', CompressedImage, self.callback)
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.callback)
        self.steering_pub = rospy.Publisher('/lane/steering', Float32, queue_size=10)
        self.speed_pub = rospy.Publisher('/lane/speed', Float32, queue_size=10)
        self.serial_pub = rospy.Publisher('serial/command',String,queue_size=10)

        # Lane Keeping 모델 로드
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        model_path = "/home/seame/model_pt/best_finetuned_model.pt"

        # 모델 로드
        self.model = OptimizedLaneNet().to(self.device)
        self.model.load_state_dict(torch.load(model_path, map_location=self.device))
        self.model.eval()

    def callback(self, msg):
        """ROS 토픽에서 수신한 카메라 데이터를 차선 감지에 활용"""
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        frame = cv2.resize(frame, (480, 270))

        # 모델 입력 전처리
        input_tensor = torch.tensor(frame / 255.0, dtype=torch.float32).permute(2, 0, 1).unsqueeze(0).to(self.device)

        with torch.no_grad():
            output = self.model(input_tensor).squeeze().cpu().numpy()

        # 차선 감지 후처리
        mask = (output > 0.2).astype(np.uint8) * 255
        BEV_mask, unwrap_matrix = flatten_perspective(mask)
        lane_tracker = LaneTracker(frame, BEV_mask)
        processed_frame, offset, curvature = lane_tracker.process(frame, BEV_mask, unwrap_matrix, True, True)

        # 조향값 및 속도 계산
        steering_angle = self.calculate_steering_angle(offset, curvature)
        speed = self.calculate_speed(steering_angle)

        # ROS 토픽으로 조향 및 속도 전송
        self.steering_pub.publish(steering_angle)
        self.speed_pub.publish(speed)

        speed_command = {"action": "speed", "speed":int(speed)}
        speed_str = json.dumps(speed_command)
        self.serial_pub.publish(speed_str)

        steer_command = {"action": "steer", "steerAngle":int(steering_angle)}
        steer_str = json.dumps(steer_command)
        self.serial_pub.publish(steer_str)


        #cv2.imshow("Detected", mask)
        #cv2.waitKey(1)

    def run(self):
        """멀티프로세스로 실행"""
        rospy.spin()

    def calculate_steering_angle(self, offset,curvature):
        return self.map_linear(offset)


    def calculate_speed(self, steering_angle, max_speed=300, min_speed=100):
        angle_abs = abs(steering_angle)
        if angle_abs > 200:
            return min_speed
        speed = max_speed - ((max_speed - min_speed) * (angle_abs / 50))
        return int(max(min_speed, min(max_speed, speed)))
    

    
    def map_linear(self,offset, max_offset= 15, max_angle=250):
         steering = (offset/max_offset) * max_angle *(-1)
         return np.clip(steering, -max_angle, max_angle)

    # nonLinear Mapping function
    def map_nonlinear(self, offset, max_angle=250, alpha=5.0):
        steering_angle = np.tanh(alpha * offset) * max_angle *(-1)
        return steering_angle

    def map_curvature(self,offset,curvature,k1=2.9087, k2=0.1189):
        steering_angle_deg = k1 * offset + k2 * (1/curvature)
        steering_angle=steering_angle_deg*7*(-1) # vehicle wheel's deg: -25~25, servo value: -250~250

        return steering_angle

if __name__ == '__main__':
    lane_keeper_process = LaneKeeper()
    lane_keeper_process.start()
    lane_keeper_process.join()
