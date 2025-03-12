import numpy as np
import cv2
import pyrealsense2 as rs

import time
import base64
from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (mainCamera)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.allMessages import LaneKeeping, LaneSpeed
from src.utils.lantracker_pi.tracker import LaneTracker
from src.utils.lantracker_pi.perspective import flatten_perspective
from src.hardware.Lanekeep.threads.utils import OptimizedLaneNet
#from picamera2 import Picamera2
import torch

class threadLanekeep(ThreadWithStop):
    """This thread handles Lanekeep.
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    def __init__(self, queueList, logging, debugging=False):
        super(threadLanekeep, self).__init__()
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        self.lanekeepingSender = messageHandlerSender(self.queuesList, LaneKeeping)
        self.lanespeedSender = messageHandlerSender(self.queuesList, LaneSpeed)
        self.cameraSubscriber = messageHandlerSubscriber(self.queuesList, mainCamera,"fifo",True)
        self.model = self._load_model()
        self.pipeline = self._init_camera()
        self.priority = 0
        self.threshold = 3000
    #def map_f(self,x,in_min,in_max,out_min,out_max):
    #    a= (x-in_min)*(out_max-out_min)*(-1) / (in_max-in_min)+out_min
    #    return a
    
    def _load_model(self):
        start = time.time()
        model = OptimizedLaneNet()
        model_path = "src/hardware/Lanekeep/threads/best_finetuned_model.pt"
        model.load_state_dict(torch.load(model_path))
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        model.to(device)
        model.eval()
        
        end = time.time()
        print(f"Model Load Time: {end-start:.2f}s")
        return model

     #Linear Mapping function
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

    def _init_camera(self):
        """Camera Initialization"""
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 480, 270, rs.format.bgr8, 30)
        pipeline.start(config)
        return pipeline
    
    def _get_frame(self):
        """Frame capture with camera"""
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return None
        frame = np.asanyarray(color_frame.get_data())
        return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    def detect_stop_line(self, masked_image):
        """Detect stop line"""
        height, width = masked_image.shape
        roi = masked_image[int(height * 0.85):, int(width * 0.3):int(width * 0.7)]
        pixels = cv2.countNonZero(roi)
        print(f"pixels: {pixels}")
        if pixels > self.threshold:
            self.turn_left()
        else:
            pass

    def turn_left(self):
        for i in range(3):
            print("stop")
            self.lanespeedSender.send(float(0))
            time.sleep(0.5)
        for i in range(30):
            print("go")
            try:
                self.lanekeepingSender.send(float(-20))
                self.lanespeedSender.send(float(100))
            except:
                print("ecvept")
            time.sleep(0.1)
        for i in range(40):
            print("turn left", i)
            self.lanekeepingSender.send(float(-180))
            self.lanespeedSender.send(float(200))
            time.sleep(0.1)
            self.priority = 1
            self.threshold = 4000
        pass

    def run(self):
        frame = self._get_frame()

        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        input_image = torch.tensor(frame / 255.0, dtype=torch.float32).permute(2, 0, 1).unsqueeze(0).to(device)
        start = time.time()
        with torch.no_grad():
            output = self.model(input_image)
            output = output.squeeze().cpu().numpy()
        print("lane: ",time.time() - start)
        mask = (output > 0.2).astype(np.uint8) * 255
        BEV_mask, unwrap_matrix= flatten_perspective(mask)
        
        lane_track = LaneTracker(np.asanyarray(frame),BEV_mask)

        #use_camera = True
        count = 0
        while self._running:
            try:
                frame = self._get_frame()
                if frame is None:
                    continue
                #frame = cv2.resize(frame, (480,270))
                device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
                input_image = torch.tensor(frame / 255.0, dtype=torch.float32).permute(2, 0, 1).unsqueeze(0).to(device)
                start = time.time()
                with torch.no_grad():
                    output = self.model(input_image)
                    output = output.squeeze().cpu().numpy()
                #print("lane: ",time.time() - start)
                mask = (output > 0.2).astype(np.uint8) * 255
                BEV_frame = flatten_perspective(frame)
                BEV_mask, unwrap_matrix = flatten_perspective(mask)
                self.detect_stop_line(BEV_mask)
                # save one frame
                #cv2.imwrite(f"/home/seame/LaneNet_dataset/mask{count}.jpg", frame)
                #count+=1
                #cv2.imwrite(f"/home/seame/LaneNet_dataset/mask{count}.jpg", frame)
                
                # frame = np.asanyarray(frame.get_data())
                processed_frame, offset, curvature = lane_track.process(frame, BEV_mask,self.priority, unwrap_matrix, True, True)

                #cv2.imshow("Processed frame", processed_frame)
                #cv2.imshow("BEV_frame", BEV_frame[0])
                #cv2.imshow("BEV_mask", BEV_mask)
                #cv2.imshow("frame", frame)
                # cv2.imshow("mask", mask)
                if cv2.waitKey(10) & 0xFF == ord('q'):
                    break
                # frame = np.asanyarray(frame.get_data())
                # start = time.time()
                # frames = self.pipeline.wait_for_frames()
                # frame = frames.get_color_frame()
                # if not frame:
                #     continue

                # frame = np.asanyarray(frame.get_data())
                # processed_frame, offset, curvature = lane_track.process(frame, True, True)
                steering_angle = self.calculate_steering_angle(offset, curvature)
                speed = self.calculate_speed(steering_angle)

                print("angle:", steering_angle, "speed:", speed)
                self.lanekeepingSender.send(float(steering_angle))
                self.lanespeedSender.send(float(speed))
                # print(time.time() - start)
                time.sleep(0.1)
            except Exception as e:
                if self.debugging:
                    self.logging.error(f"Error in lane tracking: {e}")

        self.pipeline.stop()
        cv2.destroyAllWindows()
        

            
                

    
    def calculate_steering_angle(self, offset,curvature):

        #return self.map_f(offset,-0.254,0.1725,-100,100)
        offest_angle =  self.map_linear(offset)
        if curvature != 0:
            curvature_factor = 1/curvature
        else:
            curvature_factor = 0
        curvature_factor = 0
        steering_angle = offest_angle + (curvature_factor*20)
        #print("offset: ", offset, "curvature: ", curvature, "steering_angle: ", steering_angle)
        return np.clip(steering_angle, -250, 250)
        #return self.map_curvature(offset,curvature)

    def calculate_speed(self, steering_angle, max_speed=300, min_speed=100):

        angle_abs = abs(steering_angle)

        if angle_abs > 200:
            return min_speed


        speed = max_speed - ((max_speed - min_speed) * (angle_abs / 50))
        return int(max(min_speed, min(max_speed, speed)))



