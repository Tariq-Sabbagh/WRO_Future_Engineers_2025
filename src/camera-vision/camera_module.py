# camera_module.py
import cv2
import numpy as np
from picamera2 import Picamera2

class Camera:
    def __init__(self, resolution=(1280, 720), flip=True, brightness=7, contrast=1):
        self.resolution = resolution
        self.flip = flip
        self.brightness = brightness
        self.contrast = contrast
        
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(
            main={"size": self.resolution},
            raw={"size": (2304, 1296)}
        )
        self.picam2.configure(config)
        self.picam2.set_controls({"ExposureTime": 10000})
        self.picam2.start()

    def set_brightness_contrast(self, brightness, contrast):
        self.brightness = brightness
        self.contrast = contrast

    def capture_frame(self):
        """Capture a frame and apply initial processing"""
        frame = self.picam2.capture_array()
        
        if self.flip:
            frame = cv2.flip(frame, -1)  # Flip both vertically and horizontally
            
        # Apply global brightness and contrast adjustments
        frame = cv2.addWeighted(frame, self.contrast, 
                              np.zeros(frame.shape, frame.dtype), 
                              0, self.brightness)
        
        # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Convert to RGB color space
        return frame

    def stop(self):
        self.picam2.stop()