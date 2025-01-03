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
    """Thread which will handle camera functionalities using OpenCV."""

    # ================================ INIT ===============================================
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
        """Subscribe function. In this function we make all the required subscribe to process gateway."""

        self.recordSubscriber = messageHandlerSubscriber(self.queuesList, Record, "lastOnly", True)
        self.brightnessSubscriber = messageHandlerSubscriber(self.queuesList, Brightness, "lastOnly", True)
        self.contrastSubscriber = messageHandlerSubscriber(self.queuesList, Contrast, "lastOnly", True)

    def Queue_Sending(self):
        """Callback function for recording flag."""
        self.recordingSender.send(self.recording)
        threading.Timer(1, self.Queue_Sending).start()

    # =============================== STOP ================================================
    def stop(self):
        if self.recording and self.video_writer is not None:
            self.video_writer.release()
        self.camera.release()
        super(threadCamera, self).stop()

    # =============================== CONFIG ==============================================
    def Configs(self):
        """Callback function for receiving configs on the pipe."""

        if self.brightnessSubscriber.isDataInPipe():
            message = self.brightnessSubscriber.receive()
            if self.debugger:
                self.logger.info(str(message))
            # Brightness adjustment is not directly supported by OpenCV; you may handle it in post-processing.

        if self.contrastSubscriber.isDataInPipe():
            message = self.contrastSubscriber.receive()
            if self.debugger:
                self.logger.info(str(message))
            # Contrast adjustment is not directly supported by OpenCV; handle it in post-processing.

        threading.Timer(1, self.Configs).start()

    # ================================ RUN ================================================
    def run(self):
        """This function will run while the running flag is True. It captures the image from camera and processes it."""

        send = True
        while self._running:
            try:
                recordRecv = self.recordSubscriber.receive()
                if recordRecv is not None:
                    self.recording = bool(recordRecv)
                    if not self.recording and self.video_writer is not None:
                        self.video_writer.release()
                        self.video_writer = None
                    elif self.recording and self.video_writer is None:
                        fourcc = cv2.VideoWriter_fourcc(*"XVID")
                        self.video_writer = cv2.VideoWriter(
                            "output_video" + str(time.time()) + ".avi",
                            fourcc,
                            self.frame_rate,
                            (self.frame_width, self.frame_height),
                        )

            except Exception as e:
                print(e)

            ret, frame = self.camera.read()
            if not ret:
                continue

            # Resize frame for low-resolution
            lores_frame = cv2.resize(frame, (512, 270))

            # Save frame if recording
            if self.recording and self.video_writer is not None:
                self.video_writer.write(frame)

            # Encode frames for sending
            _, mainEncodedImg = cv2.imencode(".jpg", frame)
            _, serialEncodedImg = cv2.imencode(".jpg", lores_frame)

            mainEncodedImageData = base64.b64encode(mainEncodedImg).decode("utf-8")
            serialEncodedImageData = base64.b64encode(serialEncodedImg).decode("utf-8")

            self.mainCameraSender.send(mainEncodedImageData)
            self.serialCameraSender.send(serialEncodedImageData)

            send = not send

    # =============================== START ===============================================
    def start(self):
        super(threadCamera, self).start()

    # ================================ INIT CAMERA ========================================
    def _init_camera(self):
        """This function will initialize the OpenCV camera object."""
        self.camera = cv2.VideoCapture(0, cv2.CAP_V4L2)  # Use default camera
        if not self.camera.isOpened():
            raise RuntimeError("Could not open camera.")

        self.frame_width = int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
