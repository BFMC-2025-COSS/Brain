import pyrealsense2 as rs
import numpy as np
import time

yaw = 0.0
prev_time = None
pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.gyro)

pipeline.start(config)

try:
    while True:
        frames = pipeline.wait_for_frames()
        gyro_frame = frames.first_or_default(rs.stream.gyro)
        if gyro_frame:
            gyro_data = gyro_frame.as_motion_frame().get_motion_data()
            timestamp = gyro_frame.get_timestamp() / 1000.0

            if prev_time is not None:
                dt = timestamp - prev_time
                yaw += gyro_data.z * dt
            prev_time = timestamp
            print(f"yaw:{np.degrees(yaw):.2f}")
        time.sleep(0.01)
except KeyboardInterrupt:
    print("key")

finally:
    pipeline.stop()