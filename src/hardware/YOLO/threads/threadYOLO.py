from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.allMessages import AEB, HighwaySignal, LaneKeeping, LaneSpeed
import cv2
import torch
import pyrealsense2 as rs
import numpy as np
import time
from models.yolo import Model
from models.common import DetectMultiBackend
from utils.general import check_img_size, non_max_suppression

from src.utils.lantracker_pi.tracker import LaneTracker
from src.utils.lantracker_pi.perspective import flatten_perspective
from src.hardware.Lanekeep.threads.utils import OptimizedLaneNet


class threadYOLO(ThreadWithStop):
    def __init__(self, queueList, logging, debugging=False, imgsz=256, fps=5):
        super(threadYOLO, self).__init__()
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        self.frame_count = 0
        self.not_pedestrian_detected_count = 0
        self.threshold_area = 14000  # BBOX Size Threshold
        self.threshold_conf = 0.4   # Confidence Threshold
        self.imgsz = imgsz
        self.fps = fps

        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        self.weights = "/home/seame/Brain/pt/ColorAug_BalancedCls_256.pt"
        #self.weights = "/home/seame/Brain/pt/best.pt"
        self.pipeline = self._init_camera()
        self.model_yolo = self._load_yolo_model()
        self.model_lanekeep = self._load_lanekeep_model()
        self.priority = 0
        self.threshold = 3000

        #self.pipeline = self._init_camera()
        self.AEBSender = messageHandlerSender(self.queuesList, AEB)
        self.highwaySender = messageHandlerSender(self.queuesList, HighwaySignal)
        self.lanekeepingSender = messageHandlerSender(self.queuesList, LaneKeeping)
        self.lanespeedSender = messageHandlerSender(self.queuesList, LaneSpeed)
        self.lastSteer = 0


    def _load_yolo_model(self):
        """YOLO Model Load"""
        print("start")
        start = time.time()
        model = DetectMultiBackend(self.weights, device=self.device)
        self.stride, self.names, self.pt = model.stride, model.names, model.pt

        self.imgsz = check_img_size(self.imgsz, s=self.stride)
        print(self.imgsz)
        if isinstance(self.imgsz, int): self.imgsz = (256,192)

        model.to(self.device).eval()
        model.warmup(imgsz = (1,3,*self.imgsz))

        end = time.time()
        print(f"YOLO Model Load Time: {end-start:.2f}s")
        return model

    def _load_lanekeep_model(self):
        """LaneKeep Model Load"""
        start = time.time()
        model = OptimizedLaneNet()
        model_path = "src/hardware/Lanekeep/threads/best_finetuned_model.pt"
        model.load_state_dict(torch.load(model_path))
        model.to(self.device).eval()
        end = time.time()
        print(f"LaneKeep Model Load Time: {end-start:.2f}s")
        return model
    
    
    
    def _init_camera(self):
        """Camera Initialization"""
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 480, 270, rs.format.rgb8, 30)
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
    
    def calculate_steering_angle(self, offset,curvature):

        offest_angle =  self.map_linear(offset)
        if curvature != 0:
            curvature_factor = 1/curvature
        else:
            curvature_factor = 0
        curvature_factor = 0
        steering_angle = offest_angle + (curvature_factor*20)
        #print("offset: ", offset, "curvature: ", curvature, "steering_angle: ", steering_angle)
        steer_final = np.clip(steering_angle, -250, 250)
        if self.priority == 1:
            steer_final += 20
            if steer_final > 220:
                self.priority = 0
                print("Intersection Mode Off")
        self.lastSteer = steer_final
        return steer_final
    
    def map_linear(self,offset, max_offset= 15, max_angle=250):
         steering = (offset/max_offset) * max_angle *(-1)
         return np.clip(steering, -max_angle, max_angle)

    def calculate_speed(self, steering_angle, max_speed=300, min_speed=150):
        angle_abs = abs(steering_angle)

        if angle_abs > 200:
            return min_speed
        if self.priority == 1:
            return 60

        speed = max_speed - ((max_speed - min_speed) * (angle_abs / 50))
        return int(max(min_speed, min(max_speed, speed)))

    def detect_stop_line(self, masked_image,steer):
        height, width = masked_image.shape
        roi = masked_image[int(height * 0.8):, int(width * 0.3):int(width * 0.7)]
        pixels = cv2.countNonZero(roi)
        #print(f"pixels: {pixels}")
        if pixels > self.threshold:
            self.turn_left(steer)
            return True
        else:
            return False
    def turn_left(self,steer):
        print("[InterSection Mode On]")
        for i in range(5):
            self.lanespeedSender.send(float(0))
            time.sleep(0.2)
        while steer > 0:
            steer -= 20
            self.lanekeepingSender.send(float(steer))
            time.sleep(0.3)
        self.lanespeedSender.send(float(100))
        time.sleep(0.4)
        for i in range(2):
            try:
                self.lanekeepingSender.send(float(-24))
            except:
                print("ecvept")
            time.sleep(0.4)
        self.lanespeedSender.send(float(200))
        time.sleep(0.1)
        for i in range(45):
            #print("turn left", i)
            self.lanekeepingSender.send(float(max(-50-i*3.3,-240)))
            time.sleep(0.1)
            self.priority = 1
            #print("Intersection Mode On")
            self.threshold = 400000
        pass

    def turn_left_after(self):
        for i in range(1):
            #print("stop")
            self.lanespeedSender.send(float(0))
            time.sleep(0.5)
        self.lanespeedSender.send(float(100))
        for i in range(23):
            #print("go")
            try:
                self.lanekeepingSender.send(float(-25))
            except:
                print("ecvept")
            time.sleep(0.1)
        self.lanespeedSender.send(float(200))
        for i in range(35):
            #print("turn left", i)
            self.lanekeepingSender.send(float(max(-70 - i * 4,-190)))
            time.sleep(0.1)
            self.priority = 0
            self.threshold = 400000
        pass



    def run(self):
        frame = self._get_frame()

        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        frame = cv2.resize(frame,(480,270))
        input_image = torch.tensor(frame / 255.0, dtype=torch.float32).permute(2, 0, 1).unsqueeze(0).to(device)
        with torch.no_grad():
            output = self.model_lanekeep(input_image)
            output = output.squeeze().cpu().numpy()
        mask = (output > 0.2).astype(np.uint8) * 255
        BEV_mask, unwrap_matrix= flatten_perspective(mask)
        
        lane_track = LaneTracker(np.asanyarray(frame),BEV_mask)
        highway_state = False
        highwayon_flag=True
        highwayoff_flag=True
        pedestrian_detected = False
        steering_angle = 0.0
        offcount = 0
        while self._running:
            try:
                start = time.time()
                frame = self._get_frame()
                if frame is None:
                    continue
                frame = cv2.resize(frame,self.imgsz, interpolation = cv2.INTER_LINEAR)

                im = torch.from_numpy(frame).to(self.device)
                im = im.permute(2, 0, 1).unsqueeze(0)  # HWC → CHW, Batch 차원 추가
                im = im.half() if self.model_yolo.fp16 else im.float()  # uint8 → FP16/FP32 변환
                im /= 255.0  # Normalize to [0,1]
                if len(im.shape) == 3:
                    im = im[None]  # Batch 차원 추가

                with torch.no_grad():
                    pred = self.model_yolo(im)  # YOLO 모델 추론
                detections = non_max_suppression(pred, self.threshold_conf, 0.4, classes=None, agnostic=False)

                annotated_frame = frame.copy()
                pedestrian_detected = False
                for det in detections:
                    if det is None or len(det) == 0:
                        continue
                    det = det.squeeze(0)
                    x1, y1, x2, y2, conf, cls = det
                    area = (x2 - x1) * (y2 - y1)
                    cv2.rectangle(annotated_frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                    if cls == 0:  # 0: BFMC_pedestrian
                        print(f"Pedestrian Detected")
                        pedestrian_detected = True
                        # if area > self.threshold_area and conf > self.threshold_conf:
                        #     detected = True
                    elif cls == 1:
                        print(f"Person Detected")
                    elif cls == 2:
                        print(f"Priority Detected")
                    elif cls == 3:
                        print(f"Crosswalk Detected")
                    elif cls == 4:
                        print(f"Stop Detected")
                    elif cls == 5:
                        print(f"park Detected")
                    elif cls == 6:
                        if highwayon_flag:
                            print(f"[Highway Mode on]")
                            highwayon_flag = False
                        highway_state = True
                        offcount = 0
                        
                    elif cls == 7: # 7: Highway Exit sign
                        if offcount > 3:
                            if highwayoff_flag and highway_state:
                                print(f"[Highway Mode off]")
                                highwayoff_flag = False
                                offcount = 0
                        offcount += 1
                        highway_state = False
                    elif cls == 9:
                        print(f"Round-about Detected")
               # Visualize BBOX Image
                cv2.imshow("Detected", annotated_frame)
                cv2.waitKey(1) 

                if pedestrian_detected:
                    self.frame_count += 1
                    self.not_pedestrian_detected_count = 0
                    if self.frame_count >= 1:
                        self.AEBSender.send(1.0)
                        print("[AEB Mode On]")
                        continue
                else:
                    if self.frame_count >= 2:
                        self.not_pedestrian_detected_count += 1
                        if self.not_pedestrian_detected_count >= 20:
                            self.AEBSender.send(0.0)
                            
                            print("[AEB Mode Off]")
                            self.turn_left_after()
                            self.frame_count = 0

                if highway_state:
                    self.highwaySender.send(1.0)
                else:
                    self.highwaySender.send(0.0)


                # LaneKeep Model Processing
                input_image = torch.tensor(frame / 255.0, dtype=torch.float32).permute(2, 0, 1).unsqueeze(0).to(self.device)
                with torch.no_grad():
                    output = self.model_lanekeep(input_image)
                    output = output.squeeze().cpu().numpy()
                
                mask = (output > 0.2).astype(np.uint8) * 255
                BEV_frame = flatten_perspective(frame)
                BEV_mask, unwrap_matrix = flatten_perspective(mask)
                processed_frame, offset, curvature = lane_track.process(frame, BEV_mask, self.priority, unwrap_matrix, True, True)
                if self.detect_stop_line(BEV_mask,steering_angle):
                    continue
                steering_angle = self.calculate_steering_angle(offset, curvature)
                speed = self.calculate_speed(steering_angle)

                print("\t\t\t angle:", int(steering_angle) ,"\tspeed:", speed +(100 if highway_state else 0))
                self.lanekeepingSender.send(float(steering_angle))
                self.lanespeedSender.send(float(speed))
                #time.sleep(0.2-time.time() + start)

            except Exception as e:
                if self.debugging:
                    self.logging.error(f"Error in processing: {e}")

        self.pipeline.stop()
        cv2.destroyAllWindows()
            