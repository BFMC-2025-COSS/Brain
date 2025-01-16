import numpy as np
import cv2
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

    def map_f(self,x,in_min,in_max,out_min,out_max):
        a= (x-in_min)*(out_max-out_min) / (in_max-in_min)+out_min
        return a

    def run(self):
        picam2 = Picamera2()
        config = picam2.create_preview_configuration()
        picam2.configure(config)
        picam2.start()
        while self._running:
            try:
                # Simulate receiving frame data (replace with actual frame capture)
                frame =picam2.capture_array()
                lane_track = LaneTracker(frame)
                processed_frame, offset, curvature = lane_track.process(frame)
                # Visualization
                #cv2.imshow("Lane Tracking", processed_frame)
                #key =cv2.waitKey(1)
                #if key == 27:
                #    pass
                # Calculate steering angle
                steering_angle = self.calculate_steering_angle(offset, curvature)
                speed = self.calculate_speed(steering_angle)
                self.lanekeepingSender.send(flaot(steering_angle))
                #print(steering_angle,speed)
                self.lanespeedSender.send(speed)

            except Exception as e:
                if self.debugging:
                    self.logging.error(f"Error in lane tracking: {e}")
                

    
    def calculate_steering_angle(self, offset, curvature):
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
        return self.map_f(offset,-0.254,0.1725,-100,100)

    def calculate_speed(self, steering_angle, max_speed=100, min_speed=50):
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

        if angle_abs > 50:
            return min_speed


        speed = max_speed - ((max_speed - min_speed) * (angle_abs / 50))
        return int(max(min_speed, min(max_speed, speed)))



