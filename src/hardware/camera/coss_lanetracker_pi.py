import cv2
import rospy
import base64
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from tracker import LaneTracker
from threading import Thread
from multiprocessing import Queue
from src.hardware.camera.threads.threadCamera import threadCamera

class LaneTrackerNode:
    def __init__(self, queueList):
        self.bridge = CvBridge()
        self.lane_tracker = None
        self.i = 1
        self.queueList = queueList

        # 큐에서 이미지를 계속해서 가져와 처리하는 스레드를 생성합니다.
        self.processing_thread = Thread(target=self.process_images)
        self.processing_thread.daemon = True
        self.processing_thread.start()

    def process_images(self):
        # 큐에서 이미지가 들어올 때마다 처리하는 부분입니다.
        while True:
            if not self.queueList["General"].empty():
                img_data = self.queueList["General"].get()
                
                if "msgValue" in img_data:
                    # base64로 인코딩된 이미지를 디코딩합니다.
                    image_data = base64.b64decode(img_data["msgValue"])
                    img = np.frombuffer(image_data, dtype=np.uint8)
                    cv_image = cv2.imdecode(img, cv2.IMREAD_COLOR)

                    if self.lane_tracker is None:
                        # 첫 번째 이미지로 LaneTracker 초기화
                        self.lane_tracker = LaneTracker(cv_image)
                        rospy.loginfo("LaneTracker initialized with the first frame.")
                    
                    # Lane tracking을 수행합니다.
                    processed_frame = self.lane_tracker.process(cv_image)

                    # 결과 이미지 출력
                    cv2.imshow("Lane Tracking", processed_frame)

                    # 'ESC' 키를 누르면 종료
                    key = cv2.waitKey(1)
                    if key == 27:  # ESC 키를 누르면 종료
                        rospy.signal_shutdown("ESC key pressed. Shutting down...")

                    self.i += 1

    def run(self):
        # ROS spin을 호출하지 않고, 이미지를 계속해서 처리합니다.
        while True:
            pass

if __name__ == '__main__':
    # 카메라 스레드를 생성하고 큐를 전달합니다.
    queueList = {
        "Critical": Queue(),
        "Warning": Queue(),
        "General": Queue(),
        "Config": Queue(),
    }
    
    cameraThread = threadCamera(queueList, None, debugger=True)
    cameraThread.start()

    try:
        # LaneTrackerNode를 시작하고 큐를 전달합니다.
        lanetracker_nod = LaneTrackerNode(queueList)
        lanetracker_nod.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        # 종료 시 카메라 스레드를 멈추고 OpenCV 창을 닫습니다.
        cameraThread.stop()
        cv2.destroyAllWindows()
