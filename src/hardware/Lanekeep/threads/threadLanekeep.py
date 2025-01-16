import numpy as np
import cv2
from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (mainCamera)
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.allMessages import LaneKeeping
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
        

    def run(self):

        picam2 = Picamera2()

        config = picam2.preview_configuration
        picam2.configure(config)

        picam2.start()
        print("00")
        while self._running:
            try:
                # Simulate receiving frame data (replace with actual frame capture)
                frame =picam2.capture_array()
                lane_track = LaneTracker(frame)
                processed_frame, offset, curvature = lane_track.process(frame)
                cv2.imshow("Lane Tracking", processed_frame)
                key =cv2.waitKey(1)
                if key == 27:
                    pass
                
                steering_angle = self.calculate_steering_angle(offset, curvature)
                print(steering_angle)

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
        if curvature == 0:  # Prevent division by zero
            return 0
        return offset * 0.1 + (1 / curvature) * 0.5
