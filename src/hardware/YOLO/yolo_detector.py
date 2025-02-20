#!/usr/bin/env python3

import rospy
import cv2
import torch
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge

class YoloPublisher:
    def __init__(self):
        rospy.init_node('yolo_publisher_node', anonymous=True)
        
        # ROS Subscribers & Publishers
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.callback)
        self.aeb_pub = rospy.Publisher('/yolo/aeb', Float32, queue_size=10)

        # YOLO 모델 로드
        if torch.cuda.is_available():
            self.device = "cuda:0"
        else:
            self.device = "cpu"
        # self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")  # Jetson에서 GPU 사용 가능
        model_path = "/home/seame/model_pt/best.pt"
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path, device=self.device)
        self.model.conf = 0.25  # Confidence threshold
        self.model.iou = 0.45  # IoU threshold

    def callback(self, msg):
        """ROS 토픽에서 카메라 데이터를 받아 YOLO로 객체 감지"""
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        results = self.model(frame, size=256)
        detections = results.xyxy[0]
        annotated_frame = frame.copy()

        detected = False
        for det in detections:
            x1, y1, x2, y2, conf, cls = det
            area = (x2 - x1) * (y2 - y1)
            cv2.rectangle(annotated_frame,(int(x1),int(y1)),(int(x2),int(y2)),(0,255,0),2)

            if int(cls) == 0:  # 보행자 (class 0)
                print(f"cls:{int(cls)}, Area: {area:.2f}")
                if area > 5000 and conf > 0.4:  # 크기 및 신뢰도 임계값
                    detected = True
            else:
                detected = False
        self.aeb_pub.publish(1.0 if detected else 0.0)  # AEB 신호 전송
        cv2.imshow("Detected",annotated_frame)
        cv2.waitKey(1)  

if __name__ == '__main__':
    yolo_node = YoloPublisher()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
