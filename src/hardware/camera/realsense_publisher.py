#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import tf.transformations as transformations
import time
from bfmc.msg import realsense_imu

# RealSense 사용 시 활성화
import pyrealsense2 as rs 

class Realsense:
    def __init__(self):
        rospy.init_node('realsense_publish_node', anonymous=True)
        rospy.loginfo("Realsense Publisher Node Started")

        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=1)
        self.imu_pub = rospy.Publisher('/realsense_imu',realsense_imu, queue_size=10)


        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.gyro)
        config.enable_stream(rs.stream.accel)
        self.pipeline.start(config)
        

        self.yaw = 0.0
        self.prev_time = None
        self.rate = rospy.Rate(30)

        self.gyro_samples = []
        self.gyro_bias_y = 0.0
        self.bias_init = False

    def calibrate_gyro_bias(self,gyro_y):
        if len(self.gyro_samples) < 100:
            self.gyro_samples.append(gyro_y)
        else:
            self.gyro_bias_y = np.mean(self.gyro_samples)
            self.bias_init = True
            rospy.loginfo(f"Gyro Bias(y): {self.gyro_bias_y}")



    def get_quaternion_from_yaw(self,yaw):
        return transformations.quaternion_from_euler(0,0,yaw)


    def publish_data(self):
        while not rospy.is_shutdown():
            frames = self.pipeline.wait_for_frames()

            color_frame = frames.get_color_frame()
            if color_frame:
                frame = np.asanyarray(color_frame.get_data())
            else:
                ret, frame = self.cap.read()
                if not ret:
                    rospy.logwarn("Failed to capture frame from webcam.")
                    continue

            image_msg = self.bridge.cv2_to_imgmsg(frame,encoding = "bgr8")
            self.image_pub.publish(image_msg)

            gyro_frame = frames.first_or_default(rs.stream.gyro)
            accel_frame = frames.first_or_default(rs.stream.accel)

            if gyro_frame and accel_frame:
                gyro_data = gyro_frame.as_motion_frame().get_motion_data()
                timestamp = gyro_frame.get_timestamp() / 1000.0 # ms -> s

                # ---- [bias calibration] ----
                if not self.bias_init:
                    self.calibrate_gyro_bias(gyro_data.y)
                    continue    #No update yaw value during initialize
                corrected_gyro_y = gyro_data.y - self.gyro_bias_y


                if self.prev_time is not None:
                    dt = timestamp - self.prev_time
                    self.yaw += -corrected_gyro_y * dt
                self.prev_time = timestamp

                accel_data = accel_frame.as_motion_frame().get_motion_data()

                imu_msg = realsense_imu()
                imu_msg.header.stamp = rospy.Time.now()
                imu_msg.header.frame_id = "imu_link"

                imu_msg.angular_velocity.x = gyro_data.x
                imu_msg.angular_velocity.y = gyro_data.y
                imu_msg.angular_velocity.z = gyro_data.z

                imu_msg.linear_acceleration.x = accel_data.x
                imu_msg.linear_acceleration.y = accel_data.y
                imu_msg.linear_acceleration.z = accel_data.z

                quaternion = self.get_quaternion_from_yaw(self.yaw)
                imu_msg.orientation.x = quaternion[0]
                imu_msg.orientation.y = quaternion[1]
                imu_msg.orientation.z = quaternion[2]
                imu_msg.orientation.w = quaternion[3]

                imu_msg.yaw = self.yaw
                
                self.imu_pub.publish(imu_msg)
            self.rate.sleep()

if __name__ == '__main__':
    node = Realsense()
    try:
        node.publish_data()
    except rospy.ROSInterruptException:
        pass
