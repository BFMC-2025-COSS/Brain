import sys

sys.path.append(".")

import cv2
import base64
import numpy as np
from multiprocessing import Queue
from src.hardware.camera.threads.threadCamera import threadCamera

# Queue 초기화
queueList = {
    "Critical": Queue(),
    "Warning": Queue(),
    "General": Queue(),
    "Config": Queue(),
}

# 카메라 쓰레드 실행
cameraThread = threadCamera(queueList, None, debugger=True)
cameraThread.start()

try:
    while True:
        # General Queue에서 이미지를 수신
        if not queueList["General"].empty():
            img_data = queueList["General"].get()
            if "msgValue" in img_data:
                image_data = base64.b64decode(img_data["msgValue"])
                img_array = np.frombuffer(image_data, dtype=np.uint8)
                image = cv2.imdecode(img_array, cv2.IMREAD_COLOR)

                # OpenCV 창에 이미지 표시
                cv2.imshow("Camera Feed", image)

                # 'q'를 누르면 종료
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
finally:
    # 자원 해제
    cameraThread.stop()
    cv2.destroyAllWindows()
