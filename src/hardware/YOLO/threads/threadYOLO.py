from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.allMessages import AEB
#from picamera2 import Picamera2
import cv2
import torch
import pyrealsense2 as rs
import numpy as np
import time
from models.yolo import Model
from models.common import DetectMultiBackend
# from utils.general import non_max_suppression
# from utils.plots import Annotator, colors

class threadYOLO(ThreadWithStop):
    def __init__(self, queueList, logging, debugging=False, imgsz=256, fps=5):
        super(threadYOLO, self).__init__()
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        self.frame_count = 0
        self.not_detected_count = 0
        self.threshold_area = 14000  # BBOX Size Threshold
        self.threshold_conf = 0.4   # Confidence Threshold
        self.imgsz = imgsz
        self.fps = fps
        self.model = self._load_model()
        self.pipeline = self._init_camera()
        self.AEBSender = messageHandlerSender(self.queuesList, AEB)
        

    def _load_model(self):
        """YOLO Model Load"""
        device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu") # For Jetson
        #device = torch.device("cpu")
        
        model_path="/home/seame/Brain/pt/640_ped+human.pt"
        
        model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path, device = device)        
        # model = DetectMultiBackend(model_path = "/home/seame/yolov5/models/yolov5m.yaml", weight="/home/seame/Brain/pt/640_ped+human.pt", device = device)
        model.to(device).eval()
        
        model.conf = 0.25
        model.iou = 0.45
        return model

    def _init_camera(self):
        """Camera Initialization"""
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
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

    def run(self):
        while self._running:
            frame = self._get_frame()
            if frame is None:
                continue
            start = time.time()
            results = self.model(frame, size=self.imgsz)
            detections = results.xyxy[0]
            annotated_frame = frame.copy()
            end = time.time()
            print(f"YOLO: {end-start:.2f}s")
            detected = False
            for det in detections:
                x1, y1, x2, y2, conf, cls = det
                area = (x2 - x1) * (y2 - y1)
                cv2.rectangle(annotated_frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)

                if cls == 0:  # 0: BFMC_pedestrian
                    print(f"cls:{int(cls)}, Area: {area:.2f}")
                    detected = True
                    # if area > self.threshold_area and conf > self.threshold_conf:
                    #     detected = True
                else:
                    detected = False
            # Visualize BBOX Image
            cv2.imshow("Detected", annotated_frame)
            cv2.waitKey(1)        

            if detected:  # Condition of stopping Vehicle
                self.frame_count += 1
                self.not_detected_count = 0
                if self.frame_count >= 2:
                    self.AEBSender.send(1.0)
                    print("AEB Start")
            else:
                if self.frame_count >= 2:  # Already AEB situation
                    self.not_detected_count += 1
                    if self.not_detected_count >= 3:  # AEB Finish signal
                        self.AEBSender.send(0.0)
                        print("AEB Stop - Vehicle Moving")
                        self.frame_count = 0
            

                    
