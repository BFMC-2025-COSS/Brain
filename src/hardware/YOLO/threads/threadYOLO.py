from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.allMessages import AEB
from picamera2 import Picamera2
import cv2
import torch
# from utils.general import non_max_suppression
# from utils.plots import Annotator, colors

class threadYOLO(ThreadWithStop):
    def __init__(self, queueList, logging, debugging=False, imgsz=256, fps=5):
        super(threadYOLO, self).__init__()
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        self.frame_count = 0
        self.threshold_area = 28000  # BBOX Size Threshold
        self.threshold_conf = 0.4   # Confidence Threshold
        self.imgsz = imgsz
        self.fps = fps
        self.model = self._load_model()
        self.camera = self._init_camera()
        self.brakeSender = messageHandlerSender(self.queuesList, AEB)

    def _load_model(self):
        """YOLO Model Load"""
        #device = torch.device("cuda" if torch.cuda.is_available() else "cpu") # For Jetson
        device = torch.device("cpu") # For RPI
        model_path="/home/pi/Brain/pt/640_ped+human.pt"
        model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path, device = device)
        model.conf = 0.25
        model.iou = 0.45
        return model

    def _init_camera(self):
        """Camera Initialization"""
        picam2 = Picamera2()
        config = picam2.create_preview_configuration(main={"size": (480, 640), "format": "RGB888"}) # Camera Configuration
        picam2.configure(config)
        picam2.set_controls({"FrameRate": self.fps}) # Setting FPS
        picam2.start()
        return picam2

    def _get_frame(self):
        """Frame capture with camera"""
        frame = self.camera.capture_array()
        return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    def run(self):
        while self._running:
            frame = self._get_frame()
            results = self.model(frame, size=self.imgsz)
            detections = results.xyxy[0]
            annotated_frame = frame.copy()
            
            detected = False
            for det in detections:
                x1, y1, x2, y2, conf, cls = det
                area = (x2 - x1) * (y2 - y1)
                cv2.rectangle(annotated_frame,(int(x1),int(y1)),(int(x2),int(y2)),(0,255,0),2)

                if cls ==0: # 0: BFMC_pedestrian
                    print(f"cls:{int(cls)}, Area: {area:.2f}")
                    if area > self.threshold_area and conf > self.threshold_conf:
                        detected = True
            # Visualize BBOX Image
            cv2.imshow("Detected",annotated_frame)
            cv2.waitKey(1)        

            if detected and cls ==0: #Condition of stopping Vehicle
                self.frame_count += 1
                if self.frame_count >= 1:
                    self.brakeSender.send(1.0)
                    print("AEB Start")
            else:
                self.frame_count = 0
