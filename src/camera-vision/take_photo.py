from picamera2 import Picamera2
import time

import cv2
# Initialize camera
picam2 = Picamera2()

# Optional: Configure preview resolution (can adjust as needed)
config = picam2.create_preview_configuration(main={"size": (4608, 2592)})
picam2.configure(config)

# Start camera
picam2.start()
time.sleep(2)  # Let the camera warm up

# Capture image
image = picam2.capture_array()
image=cv2.flip(image, -1)

# Save to file
from datetime import datetime
filename = f"photo_color_cal.jpg"

cv2.imwrite(filename, image)

print(f"Photo saved as {filename}")
