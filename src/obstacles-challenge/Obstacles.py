import serial
import time
import cv2
import numpy as np
import struct
from picamera2 import Picamera2
from flask import Flask, Response


# --- Configuration ---
KNOWN_OBSTACLE_HEIGHT_CM = 10.0
SERIAL_PORT = '/dev/ttyUSB0' 
BAUDRATE = 115200
CALIBRATION_FILE = "calibration_data.npz"  # Calibration data file
OPTIMIZED_RESOLUTION = (1280, 720)  # Better performance

# --- Load Calibration Data ---
calibration_data = np.load(CALIBRATION_FILE)
mtx = calibration_data['mtx']
dist = calibration_data['dist']

# Scale camera matrix for new resolution
SCALE_X = OPTIMIZED_RESOLUTION[0] / 4608
SCALE_Y = OPTIMIZED_RESOLUTION[1] / 2592
mtx_scaled = mtx.copy()
mtx_scaled[0, 0] *= SCALE_X  # fx
mtx_scaled[1, 1] *= SCALE_Y  # fy
mtx_scaled[0, 2] *= SCALE_X  # cx
mtx_scaled[1, 2] *= SCALE_Y  # cy
FOCAL_LENGTH = 570.0  # Use calibrated focal length

# Precompute undistortion maps for performance
map1, map2 = cv2.initUndistortRectifyMap(
    mtx_scaled, dist, None, mtx_scaled, 
    (OPTIMIZED_RESOLUTION[0], OPTIMIZED_RESOLUTION[1]), 
    cv2.CV_16SC2
)
# --- Flask App Initialization ---
app = Flask(__name__)

# --- Global Variables for Camera and Serial ---
picam2 = None
arduino = None # Start with no connection

# --- Function Definitions (Unchanged) ---
def initialize_camera():
    global picam2
    picam2 = Picamera2()
    # --- CHANGE: Updated to user-specified resolution ---
    # WARNING: High resolution can cause significant performance issues.
    # Consider (1920, 1080) or (1280, 720) for better real-time performance.
    config = picam2.create_preview_configuration(
        main={"size": OPTIMIZED_RESOLUTION},
        raw={"size": (2304, 1296)}  # Half-resolution binning
    )
    picam2.configure(config)
    picam2.start()
    print("Camera initialized.")

def initialize_serial(port, baudrate):
    global arduino
    try:
        arduino = serial.Serial(port=port, baudrate=baudrate, timeout=0.1)
        print(f"Serial connection established on {port}.")
    except serial.SerialException:
        arduino = None

def detect_color_in_lab(lab_frame, lower_bound, upper_bound):
    mask = cv2.inRange(lab_frame, lower_bound, upper_bound)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return mask

def find_largest_contour(mask, min_area=200):
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if not contours: return None
    largest_contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest_contour) > min_area: return largest_contour
    return None

def calculate_distance(focal_length, real_height, pixel_height):
    if pixel_height == 0: return 0
    return (real_height * focal_length) / pixel_height

def calculate_maneuver(frame_shape, bounding_rect, distance_cm):
    x, y, w, h = bounding_rect
    obj_center_x = x + w / 2
    frame_center_x = frame_shape[1] / 2
    pixel_offset = obj_center_x - frame_center_x
    cm_per_pixel = KNOWN_OBSTACLE_HEIGHT_CM / h
    offset_x_cm = pixel_offset * cm_per_pixel + 15
    tendon = np.sqrt(offset_x_cm**2 + distance_cm**2)
    angle = np.arctan2(offset_x_cm,distance_cm)
    turn_angle = angle * 180 / np.arccos(-1)
    return tendon , turn_angle

# --- Main Generator with Error Handling ---
def generate_frames():
    """Processes frames and handles serial communication errors gracefully."""
    global arduino, picam2
    
    lab_green_lower = np.array([69, 0, 150])
    lab_green_upper = np.array([255, 114, 255])
    
    lab_red_lower = np.array([0, 135, 0])
    lab_red_upper = np.array([134, 255, 108])

    while True:
        if arduino is None:
            initialize_serial(SERIAL_PORT, BAUDRATE)
            if arduino is None:
                # Create a black frame for the warning message
                # Use the configured resolution
                w, h = picam2.camera_properties['PixelArraySize']
                frame = np.zeros((h, w, 3), dtype=np.uint8)
                cv2.putText(frame, "ESP32 Disconnected", (50, int(h/2)), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 3)
                (flag, encodedImage) = cv2.imencode(".jpg", frame)
                if flag:
                    yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + bytearray(encodedImage) + b'\r\n')
                time.sleep(1)
                continue
        
        frame = picam2.capture_array()
        frame = cv2.flip(frame, -1)
        
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame_lab = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2LAB)

        red_mask = detect_color_in_lab(frame_lab, lab_red_lower, lab_red_upper)
        red_contour = find_largest_contour(red_mask)
        
        if red_contour is not None:
            X_contour, Y_contour, W_contour, H_contour = cv2.boundingRect(red_contour)
            cv2.rectangle(frame_rgb, (X_contour, Y_contour), (X_contour + W_contour, Y_contour + H_contour), (0, 255, 0), 2)
            distance = calculate_distance(FOCAL_LENGTH, KNOWN_OBSTACLE_HEIGHT_CM, H_contour)
            travel_dist, turn_angle = calculate_maneuver(frame_rgb.shape, (X_contour, Y_contour, W_contour, H_contour), distance)
            # print(f"x: {X_contour:.1f}")
            # print(f"W: {W_contour:.1f}")
            info_text = f"Dist: {travel_dist:.1f}cm , Angle: {turn_angle:.1f}deg"
            cv2.putText(frame_rgb, info_text, (X_contour, Y_contour - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            # print("travel_dist" + travel_dist + " turn_angle")
            
            try:
                
                arduino.write()

            except (serial.SerialException, OSError) as e:
                print(f"ERROR: Write failed. ESP32 disconnected? {e}")
                if arduino:
                    arduino.close()
                arduino = None
            except struct.error as e:
                print(f"CRITICAL ERROR: Struct packing failed. {e}. Values: Dist={dist_int}, Angle={angle_int}")

        
        (flag, encodedImage) = cv2.imencode(".jpg", frame_rgb)
        if flag:
            yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + bytearray(encodedImage) + b'\r\n')

# --- Flask Routes (Unchanged) ---
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
