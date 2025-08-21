import utils.stream as stream
from picamera2 import Picamera2
import cv2
import numpy as np
import time

picam2 = Picamera2()

picam2.configure(picam2.create_video_configuration(
    main={"format": "RGB888", "size": (1536, 864)}
))
picam2.set_controls({"AfMode": 2})  # Continuous autofocus


picam2.start()
stream.init()
time.sleep(1)

try:
    while True:
        frame = picam2.capture_array()
        stream.show("frame", frame)

except KeyboardInterrupt:
    pass
finally:
    picam2.stop()
