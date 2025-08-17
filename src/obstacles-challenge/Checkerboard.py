import cv2
import numpy as np
from picamera2 import Picamera2
from flask import Flask, Response
import threading
import time

# --- Configuration ---
KNOWN_DISTANCE_CM = 50.0  # Cube is placed at 50cm from camera
KNOWN_CUBE_HEIGHT_CM = 10.0  # Actual height of your cube in cm
SERIAL_PORT = '/dev/ttyUSB0' 
BAUDRATE = 115200
OPTIMIZED_RESOLUTION = (1280, 720)  # Better performance

# --- Global Variables ---
picam2 = None
arduino = None
focal_length = None
calibration_done = False
latest_frame = None
frame_lock = threading.Lock()
display_text = "Place cube at 50cm and press 'Calibrate'"

# --- Function Definitions ---
def initialize_camera():
    global picam2
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": OPTIMIZED_RESOLUTION},
        raw={"size": (1920, 1080)}  # Half-resolution binning
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(2)  # Camera warm-up
    print("Camera initialized.")

def initialize_serial():
    global arduino
    try:
        arduino = serial.Serial(port=SERIAL_PORT, baudrate=BAUDRATE, timeout=0.1)
        print(f"Serial connection established on {SERIAL_PORT}.")
    except Exception as e:
        print(f"Serial connection failed: {e}")
        arduino = None

def detect_color_in_lab(lab_frame, lower_bound, upper_bound):
    mask = cv2.inRange(lab_frame, lower_bound, upper_bound)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return mask

def find_largest_contour(mask, min_area=200):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours: 
        return None
    largest_contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest_contour) > min_area: 
        return largest_contour
    return None

def calculate_distance(pixel_height):
    """Calculate distance using focal length and object height"""
    global focal_length
    if pixel_height == 0 or focal_length is None:
        return 0
    return (KNOWN_CUBE_HEIGHT_CM * focal_length) / pixel_height

def calculate_focal_length(pixel_height):
    """Calculate focal length using known distance and object height"""
    return (pixel_height * KNOWN_DISTANCE_CM) / KNOWN_CUBE_HEIGHT_CM

# --- Video Frame Generator ---
def generate_frames():
    global focal_length, calibration_done, latest_frame, display_text
    
    # LAB color ranges for your cubes
    lab_green_lower = np.array([83, 0, 0])
    lab_green_upper = np.array([113, 113, 255])
    lab_red_lower = np.array([0, 131, 0])
    lab_red_upper = np.array([142, 255, 134])

    while True:
        frame = picam2.capture_array()
        frame = cv2.flip(frame, -1)
        
        # Store latest frame for calibration
        with frame_lock:
            latest_frame = frame.copy()
        
        # Convert to LAB color space
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame_lab = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2LAB)

        # Detect red cube
        red_mask = detect_color_in_lab(frame_lab, lab_red_lower, lab_red_upper)
        red_contour = find_largest_contour(red_mask)
        
        # Add calibration info to frame
        cv2.putText(frame_rgb, display_text, (20, 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        
        if red_contour is not None:
            x, y, w, h = cv2.boundingRect(red_contour)
            cv2.rectangle(frame_rgb, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            if focal_length:
                distance = calculate_distance(h)
                distance_text = f"Distance: {distance:.1f}cm"
                cv2.putText(frame_rgb, distance_text, (x, y - 40), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Show height info
            height_text = f"Height: {h}px"
            cv2.putText(frame_rgb, height_text, (x, y - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Encode frame for streaming
        (flag, encodedImage) = cv2.imencode(".jpg", frame_rgb)
        if flag:
            yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + 
                   bytearray(encodedImage) + b'\r\n')

# --- Flask App ---
app = Flask(__name__)

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/calibrate')
def calibrate_focal_length():
    global focal_length, calibration_done, display_text
    
    with frame_lock:
        if latest_frame is None:
            return "No frame available", 400
            
        frame = latest_frame.copy()
    
    # Process frame to find cube
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    frame_lab = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2LAB)
    
    lab_red_lower = np.array([0, 131, 0])
    lab_red_upper = np.array([142, 255, 134])
    red_mask = detect_color_in_lab(frame_lab, lab_red_lower, lab_red_upper)
    red_contour = find_largest_contour(red_mask)
    
    if red_contour is None:
        display_text = "Calibration failed: Cube not found"
        return "Cube not detected in frame", 400
    
    # Calculate focal length
    _, _, _, h = cv2.boundingRect(red_contour)
    focal_length = calculate_focal_length(h)
    calibration_done = True
    
    display_text = f"Calibrated! Focal Length: {focal_length:.2f}px"
    
    return f"""
    <html>
    <head>
        <title>Calibration Complete</title>
        <meta http-equiv="refresh" content="2; url=/" />
    </head>
    <body>
        <h2>Calibration Successful!</h2>
        <p>Focal Length: {focal_length:.2f} pixels</p>
        <p>Redirecting to main page in 2 seconds...</p>
    </body>
    </html>
    """

@app.route('/')
def index():
    global focal_length
    
    # Calculate display dimensions
    display_w = 1280
    display_h = int(display_w * (OPTIMIZED_RESOLUTION[1] / OPTIMIZED_RESOLUTION[0]))
    
    # Show focal length status
    focal_status = f"Focal Length: {focal_length:.2f}px" if focal_length else "Focal Length: Not Calibrated"
    
    return f"""
    <html>
    <head>
        <title>Camera Focal Length Calibration</title>
        <style>
            body {{
                font-family: sans-serif;
                text-align: center;
                background-color: #282c34;
                color: white;
                margin: 0;
                padding: 20px;
            }}
            .container {{
                max-width: 1300px;
                margin: 0 auto;
            }}
            h1 {{
                color: #61dafb;
                margin-bottom: 10px;
            }}
            .status {{
                font-size: 1.2em;
                margin: 15px 0;
                padding: 10px;
                background-color: #3c404d;
                border-radius: 5px;
                display: inline-block;
            }}
            .calibrate-btn {{
                background-color: #61dafb;
                color: #282c34;
                border: none;
                padding: 12px 24px;
                font-size: 1.1em;
                border-radius: 5px;
                cursor: pointer;
                margin: 20px 0;
                transition: background-color 0.3s;
            }}
            .calibrate-btn:hover {{
                background-color: #4fa3d1;
            }}
            .video-container {{
                margin: 20px 0;
            }}
            img {{
                border: 2px solid #61dafb;
                border-radius: 5px;
                max-width: 100%;
                height: auto;
            }}
            .instructions {{
                background-color: #3c404d;
                padding: 15px;
                border-radius: 5px;
                text-align: left;
                max-width: 800px;
                margin: 20px auto;
            }}
            .instructions h3 {{
                color: #61dafb;
                margin-top: 0;
            }}
            .instructions ol {{
                padding-left: 20px;
            }}
        </style>
    </head>
    <body>
        <div class="container">
            <h1>Camera Focal Length Calibration</h1>
            
            <div class="status">{focal_status}</div>
            <br>
            
            <a href="/calibrate">
                <button class="calibrate-btn">Calibrate Focal Length</button>
            </a>
            
            <div class="instructions">
                <h3>Calibration Instructions:</h3>
                <ol>
                    <li>Place a cube exactly {KNOWN_DISTANCE_CM}cm from the camera</li>
                    <li>Ensure the cube is clearly visible in the frame</li>
                    <li>Press the "Calibrate Focal Length" button above</li>
                    <li>System will calculate focal length based on cube height</li>
                    <li>Once calibrated, distance measurements will be displayed</li>
                </ol>
            </div>
            
            <div class="video-container">
                <img src="/video_feed" width="{display_w}" height="{display_h}">
            </div>
        </div>
    </body>
    </html>
    """

if __name__ == '__main__':
    initialize_camera()
    # Start serial in a separate thread to avoid blocking
    serial_thread = threading.Thread(target=initialize_serial, daemon=True)
    serial_thread.start()
    
    app.run(host='0.0.0.0', port=8080, threaded=True)