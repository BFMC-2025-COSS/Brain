import numpy as np
import cv2
import time
import base64
from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (mainCamera)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.allMessages import LaneKeeping, LaneSpeed
from src.utils.lantracker_pi.tracker import LaneTracker
from picamera2 import Picamera2


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

    #def map_f(self,x,in_min,in_max,out_min,out_max):
    #    a= (x-in_min)*(out_max-out_min)*(-1) / (in_max-in_min)+out_min
    #    return a
    
     #Linear Mapping function
    def map_linear(self,offset, max_offset= 17.5, max_angle=200):
         steering = (offset/max_offset) * max_angle *(-1)
         return np.clip(steering, -max_angle, max_angle)

    # nonLinear Mapping function
    def map_nonlinear(self, offset, max_angle=200, alpha=5.0):
        steering_angle = np.tanh(alpha * offset) * max_angle *(-1)
        return steering_angle

    def map_curvature(self,offset,curvature,k1=2.9087, k2=0.1189):
        steering_angle_deg = k1 * offset + k2 * (1/curvature)
        steering_angle=steering_angle_deg*7*(-1) # vehicle wheel's deg: -25~25, servo value: -250~250

        return steering_angle


    def run(self):
        picam2 = Picamera2()
        config = picam2.create_preview_configuration(main={"size": (960,540)})
        picam2.configure(config)
        picam2.start()

        frame =picam2.capture_array()
        lane_track = LaneTracker(frame)
        # lane_track = None
        # picam2=None

        use_camera = True
        

        # if use_camera:
            # picam2 = Picamera2()
            # config = picam2.create_preview_configuration()
            # config = picam2.create_preview_configuration(main={"size": (960, 540)})
            # picam2.configure(config)
            # picam2.start()

        while self._running:
            try:
                start = time.time()
                if use_camera:
                    frame = picam2.capture_array()
                    #frame = cv2.resize(frame, (640,480))
                else:
                    camera_data = self.cameraSubscriber.receive()
                    if not camera_data:
                        continue
                    frame_data = np.frombuffer(base64.b64decode(camera_data), dtype=np.uint8)
                    frame = cv2.imdecode(frame_data, cv2.IMREAD_GRAYSCALE)

                if use_camera:
                    # lane_track = LaneTracker(frame)                    
                    processed_frame, offset, curvature = lane_track.process(frame,True,True)
                    #cv2.imshow("Lane Tracking", frame)
                    #key =cv2.waitKey(1)
                else:
                    lane_track = LaneTracker(frame)
                    processed_frame, offset, curvature = lane_track.process(frame,False,False)
                steering_angle = self.calculate_steering_angle(offset,curvature)
                speed = self.calculate_speed(steering_angle)

                print("angle:",steering_angle,"speed:",speed)
                self.lanekeepingSender.send(float(steering_angle))
                self.lanespeedSender.send(float(speed))
                print(time.time()-start)

            except Exception as e:
                if self.debugging:
                    self.logging.error(f"Error in lane tracking: {e}")
        if use_camera:
            picam2.stop()
            cv2.destroyAllWindows()
        

                # if camera_data:
                    
                    
                #     start = time.time()
                    
                #     end = time.time()
                    
                    # if checksum:
                    #     self.lanekeepingSender.send(float(150.0))
                    #     checksum=checksum-1
                    #     print(150)
                    # elif checksum==0:
                    #     self.lanekeepingSender.send(float(-150.0))
                    #     checksum=checksum+1
                    #     print(-150)
                    

                    # self.lanekeepingSender.send(float(steering_angle))
                    # self.lanespeedSender.send(float(speed))

                # Simulate receiving frame data (replace with actual frame capture)
                ###frame =picam2.capture_array()
                #lane_track = LaneTracker(frame)
                ###processed_frame, offset, curvature = lane_track.process(frame)
                # Visualization
                # cv2.imshow("Lane Tracking", processed_frame)
                # key =cv2.waitKey(1)
                # if key == 27:
                #    pass
                ###print("offset:", offset)
               # Calculate steering angle
                ###steering_angle = self.calculate_steering_angle(offset)
                ###speed = self.calculate_speed(steering_angle)
                
                #self.lanekeepingSender.send(float(steering_angle))
                
                # self.lanespeedSender.send(float(speed))
                ###print("angle:",steering_angle,"speed:",speed)
                # time.sleep(1)

            
                

    
    def calculate_steering_angle(self, offset,curvature):

        """
        Calculates the steering angle based on lane offset and curvature.

        Parameters
        ----------
        offset : float
            The lateral offset of the vehicle from the lane center.
        curvature : float
            The curvature of the detected lane.

        Returns
        -------
        steering_angle : float
            The calculated steering angle for lane keeping.
        """
        #print("here")
        #return self.map_f(offset,-0.254,0.1725,-100,100)
        return self.map_linear(offset)
        #return self.map_curvature(offset,curvature)

    def calculate_speed(self, steering_angle, max_speed=300, min_speed=100):
        """
        Calculates speed based on the steering angle.

        Parameters
        ----------
        steering_angle : float
            The current steering angle of the vehicle.
        max_speed : int
            The maximum speed of the vehicle (for straight roads).
        min_speed : int
            The minimum speed of the vehicle (for sharp turns).

        Returns
        -------
        speed : int
            Calculated speed based on the steering angle.
        """
        angle_abs = abs(steering_angle)

        if angle_abs > 200:
            return min_speed


        speed = max_speed - ((max_speed - min_speed) * (angle_abs / 50))
        return int(max(min_speed, min(max_speed, speed)))



