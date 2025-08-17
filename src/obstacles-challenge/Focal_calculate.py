import cv2
import numpy as np
from picamera2 import Picamera2
from flask import Flask, Response

# --- CONFIGURATION ---

# 1. SET THE REAL-WORLD HEIGHT OF YOUR CALIBRATION OBJECT
KNOWN_OBJECT_HEIGHT_CM = 10.0

# 2. MEASURE AND SET THE DISTANCE FROM THE CAMERA TO THE OBJECT
KNOWN_DISTANCE_CM = 30.0 # <-- CHANGE THIS to your measured distance

# --- Flask App Initialization ---
app = Flask(__name__)

# --- Global Camera Variable ---
picam2 = None

# --- Function Definitions ---

def initialize_camera():
    """Initializes and configures the PiCamera2."""
    global picam2
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"size": (4608 , 2592 )})
    picam2.configure(config)
    picam2.start()
    print("Camera initialized.")

def detect_color_in_lab(lab_frame, lower_bound, upper_bound):
    """Creates a mask for a given color range in the LAB colorspace."""
    mask = cv2.inRange(lab_frame, lower_bound, upper_bound)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return mask

def find_largest_contour(mask, min_area=200):
    """Finds and returns the largest contour in a mask if it's above a min area."""
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    largest_contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest_contour) > min_area:
        return largest_contour
    return None

def generate_frames():
    """The main generator function that processes camera frames and yields them for streaming."""
    global picam2
    
    # Define the LAB color range for your calibration object
    lab_red_lower = np.array([0, 150, 0])
    lab_red_upper = np.array([254, 255, 102])

    print("\n--- Focal Length Calibration ---")
    print(f"Known Object Height: {KNOWN_OBJECT_HEIGHT_CM} cm")
    print(f"Measured Distance to Object: {KNOWN_DISTANCE_CM} cm")
    print("----------------------------------")
    print("Streaming to web browser. Check terminal for focal length values.")
    
    while True:
        frame = picam2.capture_array()
        frame = cv2.flip(frame, -1)  # Flip if your camera is upside down

        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame_lab = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2LAB)

        mask = detect_color_in_lab(frame_lab, lab_red_lower, lab_red_upper)
        contour = find_largest_contour(mask)

        if contour is not None:
            x, y, w, h = cv2.boundingRect(contour)
            pixel_height = h
            
            focal_length = (pixel_height * KNOWN_DISTANCE_CM) / KNOWN_OBJECT_HEIGHT_CM

            # Draw info on the screen
            cv2.rectangle(frame_rgb, (x, y), (x + w, y + h), (0, 255, 0), 2)
            info_text = f"Pixel Height: {pixel_height}px"
            cv2.putText(frame_rgb, info_text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Print the calculated focal length to the terminal
            print(f"Pixel Height: {pixel_height:4d}px  |  Calculated Focal Length: {focal_length:.2f}")
        else:
            print("Object not found in frame...")

        # Encode the frame in JPEG format
        (flag, encodedImage) = cv2.imencode(".jpg", frame_rgb)
        if not flag:
            continue

        # Yield the output frame in the byte format
        yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' +
               bytearray(encodedImage) + b'\r\n')

# --- Flask Routes ---

@app.route('/video_feed')
def video_feed():
    """Video streaming route."""
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    """Home page that displays the video stream."""
    return """
    <html>
      <head>
        <title>Focal Length Calibration</title>
        <style>
            body { font-family: sans-serif; text-align: center; background-color: #282c34; color: white; }
            img { border: 2px solid #61dafb; margin-top: 20px; }
        </style>
      </head>
      <body>
        <h2>Focal Length Calibration Stream</h2>
        <img src="/video_feed" width="1280" height="720">
      </body>
    </html>
    """

if __name__ == '__main__':
    initialize_camera()
    app.run(host='0.0.0.0', port=8080, threaded=True)
