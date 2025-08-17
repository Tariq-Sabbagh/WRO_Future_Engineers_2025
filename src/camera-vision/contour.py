import serial
import time
import cv2
import numpy as np
import struct
from picamera2 import Picamera2
from flask import Flask, Response, app



app = Flask(__name__)


def detect_color_in_lab(lab_frame, lower_bound, upper_bound):
    mask = cv2.inRange(lab_frame, lower_bound, upper_bound)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return mask

def initialize_camera():
    global picam2
    picam2 = Picamera2()
    # --- CHANGE: Updated to user-specified resolution ---
    # WARNING: High resolution can cause significant performance issues.
    # Consider (1920, 1080) or (1280, 720) for better real-time performance.
    config = picam2.create_preview_configuration(main={"size": (4608, 2592)})
    picam2.configure(config)
    picam2.start()
    print("Camera initialized.")

def find_largest_contour(mask, min_area=200):
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if not contours: return None
    largest_contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest_contour) > min_area: return largest_contour
    return None

def generate_frames():
    global picam2

    lab_green_lower = np.array([83, 0, 0])
    lab_green_upper = np.array([113, 113, 255])

    lab_red_lower = np.array([0, 131, 0])
    lab_red_upper = np.array([142, 255, 134])

    while True:
        try:
            frame = picam2.capture_array()
            frame = cv2.flip(frame, -1)

            frame_lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)

            red_mask = detect_color_in_lab(frame_lab, lab_red_lower, lab_red_upper)
            green_mask = detect_color_in_lab(frame_lab, lab_green_lower, lab_green_upper)

            red_contour = find_largest_contour(red_mask)
            green_contour = find_largest_contour(green_mask)

            if red_contour is not None:
                cv2.drawContours(frame, [red_contour], -1, (0, 0, 255), 3)
                M = cv2.moments(red_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    cv2.circle(frame, (cx, cy), 7, (0, 0, 255), -1)
                    cv2.putText(frame, "Red", (cx - 20, cy - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

            if green_contour is not None:
                cv2.drawContours(frame, [green_contour], -1, (0, 255, 0), 3)
                M = cv2.moments(green_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    cv2.circle(frame, (cx, cy), 7, (0, 255, 0), -1)
                    cv2.putText(frame, "Green", (cx - 20, cy - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            ret, buffer = cv2.imencode('.jpg', frame)
            if not ret:
                continue
            frame_bytes = buffer.tobytes()

            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        except Exception as e:
            print(f"[ERROR] Frame generation error: {e}")
            continue

        


@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    # Use f-string to dynamically set the image size in the HTML
    w, h = (4608, 2592) # Default if camera not ready
    if picam2 and 'PixelArraySize' in picam2.camera_properties:
       w, h = picam2.camera_properties['PixelArraySize']
    
    # Scale down for display
    display_w = 1280
    display_h = int(display_w * (h/w))

    return f"""
    <html><head><title>WRO Obstacle Detection Stream</title>
    <style>body{{font-family:sans-serif;text-align:center;background-color:#282c34;color:white;}}img{{border:2px solid #61dafb;margin-top:20px;}}</style>
    </head><body><h2>WRO Obstacle Detection (LAB)</h2>
    <img src="/video_feed" width="{display_w}" height="{display_h}"></body></html>
    """

if __name__ == '__main__':
    initialize_camera()
    app.run(host='0.0.0.0', port=8080, threaded=True)
