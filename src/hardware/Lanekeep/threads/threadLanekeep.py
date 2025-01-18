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
        a= (x-in_min)*(out_max-out_min)*(-1) / (in_max-in_min)+out_min
        return a
    
    # Linear Mapping function
    # def map_linear(offset, max_offset= 0.4 / 2, max_angle=200):
    #     steering_angle = (offset / max_offset) * max_angle
    #     return np.clip(steering_angle, -max_angle, max_angle)

    # nonLinear Mapping function
    # def map_nonlinear(offset, max_angle=200, alpha=5.0):
    #     steering_angle = np.tanh(alpha * offset) * max_angle
    #     return steering_angle

    def run(self):
        picam2 = Picamera2()
        config = picam2.create_preview_configuration(main={"size": (720,240)})
        picam2.configure(config)
        picam2.start()

        frame =picam2.capture_array()
        lane_track = LaneTracker(frame)
        
        while self._running:
            try:
                # Simulate receiving frame data (replace with actual frame capture)
                frame =picam2.capture_array()
                processed_frame, offset, curvature = lane_track.process(frame)
                # Visualization
                #cv2.imshow("Lane Tracking", processed_frame)
                #key =cv2.waitKey(1)
                #if key == 27:
                #    pass
                print("offset:", offset)
               # Calculate steering angle
                steering_angle = self.calculate_steering_angle(offset, curvature)
                speed = self.calculate_speed(steering_angle)
                self.lanekeepingSender.send(float(steering_angle))
                print(float(steering_angle),float(speed))
                self.lanespeedSender.send(float(speed))

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
        #return self.map_linear(offset)
        #return self.map_nonlinear(offset)

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



