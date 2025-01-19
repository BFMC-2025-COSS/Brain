import cv2
from picamera2 import Picamera2
from tracker import LaneTracker

import cv2

picam2 = Picamera2()

config = picam2.preview_configuration
picam2.configure(config)

picam2.start()

while True:
    frame = picam2.capture_array()
    #cv2.imshow("Camera", frame)

    bird_img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    lane_tracker = LaneTracker(bird_img)
    processed_frame, offset, cur = lane_tracker.process(bird_img)
    print(offset,cur)


    cv2.imshow("Lane Tracking", processed_frame)

    key =cv2.waitKey(1)
    
    if key == 27:
        pass