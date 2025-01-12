import cv2
import threading
import base64
import time

from src.utils.messages.allMessages import (
    mainCamera,
    serialCamera,
    Recording,
    Record,
    Brightness,
    Contrast,
)
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.templates.threadwithstop import ThreadWithStop


class threadCamera(ThreadWithStop):
    def __init__(self, queuesList, logger, debugger):
        super(threadCamera, self).__init__()
        self.queuesList = queuesList
        self.logger = logger
        self.debugger = debugger
        self.frame_rate = 5
        self.recording = False

        self.video_writer = None

        self.recordingSender = messageHandlerSender(self.queuesList, Recording)
        self.mainCameraSender = messageHandlerSender(self.queuesList, mainCamera)
        self.serialCameraSender = messageHandlerSender(self.queuesList, serialCamera)

        self.subscribe()
        self._init_camera()
        self.Queue_Sending()
        self.Configs()

    def subscribe(self):
        self.recordSubscriber = messageHandlerSubscriber(self.queuesList, Record, "lastOnly", True)
        self.brightnessSubscriber = messageHandlerSubscriber(self.queuesList, Brightness, "lastOnly", True)
        self.contrastSubscriber = messageHandlerSubscriber(self.queuesList, Contrast, "lastOnly", True)

    def Queue_Sending(self):
        self.recordingSender.send(self.recording)
        threading.Timer(1, self.Queue_Sending).start()

    def stop(self):
        if self.recording and self.video_writer is not None:
            self.video_writer.release()
        if self.camera is not None:
            self.camera.release()
        super(threadCamera, self).stop()

    def Configs(self):
        # OpenCV는 밝기/대비를 지원하지만, 장치별로 동작이 다를 수 있습니다.
        if self.brightnessSubscriber.isDataInPipe():
            message = self.brightnessSubscriber.receive()
            if self.debugger:
                self.logger.info(str(message))
            self.camera.set(cv2.CAP_PROP_BRIGHTNESS, float(message))
        if self.contrastSubscriber.isDataInPipe():
            message = self.contrastSubscriber.receive()
            if self.debugger:
                self.logger.info(str(message))
            self.camera.set(cv2.CAP_PROP_CONTRAST, float(message))
        threading.Timer(1, self.Configs).start()

    def run(self):
        send = True
        while self._running:
            ret, frame = self.camera.read()
            if not ret:
                continue

            if self.recording:
                if self.video_writer is None:
                    fourcc = cv2.VideoWriter_fourcc(*"XVID")
                    self.video_writer = cv2.VideoWriter(
                        "output_video" + str(time.time()) + ".avi",
                        fourcc,
                        self.frame_rate,
                        (frame.shape[1], frame.shape[0]),
                    )
                self.video_writer.write(frame)

            if send:
                # 전송을 위한 데이터 변환
                _, mainEncodedImg = cv2.imencode(".jpg", frame)
                mainEncodedImageData = base64.b64encode(mainEncodedImg).decode("utf-8")

                # 메시지 전송
                self.mainCameraSender.send(mainEncodedImageData)

            send = not send

    def start(self):
        super(threadCamera, self).start()

    def _init_camera(self):
        # OpenCV VideoCapture 초기화
        self.camera = cv2.VideoCapture(0)  # 0은 기본 카메라 장치
        if not self.camera.isOpened():
            raise RuntimeError("카메라를 열 수 없습니다.")
        
        # 원하는 해상도 설정 (예: 2048x1080)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 2048)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
