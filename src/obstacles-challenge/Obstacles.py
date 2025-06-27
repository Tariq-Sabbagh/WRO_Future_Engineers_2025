import serial
import time
import cv2
import numpy as np
import struct
from picamera2 import Picamera2
from flask import Flask, Response

# --- Configuration ---
KNOWN_OBSTACLE_HEIGHT_CM = 10.0
FOCAL_LENGTH = 1900.00
SERIAL_PORT = '/dev/ttyUSB0' 
BAUDRATE = 115200

# --- Flask App Initialization ---
app = Flask(__name__)

# --- Global Variables for Camera and Serial ---
picam2 = None
arduino = None # Start with no connection

# --- Function Definitions (Unchanged) ---
def initialize_camera():
    global picam2
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"size": (4608 , 2592 )})
    picam2.configure(config)
    picam2.start()
    print("Camera initialized.")

def initialize_serial(port, baudrate):
    global arduino
    try:
        arduino = serial.Serial(port=port, baudrate=baudrate, timeout=0.1)
        print(f"Serial connection established on {port}.")
    except serial.SerialException:
        # Don't print an error here, as we will be retrying
        arduino = None

def detect_color_in_lab(lab_frame, lower_bound, upper_bound):
    mask = cv2.inRange(lab_frame, lower_bound, upper_bound)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return mask

def find_largest_contour(mask, min_area=200):
    # --- CHANGE: Lowered min_area from 1000 to 200 ---
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
    offset_x_cm = pixel_offset * cm_per_pixel
    if distance_cm**2 > offset_x_cm**2:
        offset_y_cm = np.sqrt(distance_cm**2 - offset_x_cm**2)
    else:
        offset_y_cm = 0
    travel_distance = np.sqrt(offset_y_cm**2 + (offset_x_cm + 15)**2)
    if offset_y_cm == 0:
        turn_angle_rad = np.pi / 2
    else:
        turn_angle_rad = np.arctan((offset_x_cm + 15) / offset_y_cm)
    turn_angle_deg = np.degrees(turn_angle_rad)
    return travel_distance, turn_angle_deg

# --- Main Generator with Error Handling ---
def generate_frames():
    """Processes frames and handles serial communication errors gracefully."""
    global arduino, picam2
    
    lab_green_lower = np.array([0, 0, 132])
    lab_green_upper = np.array([255, 120, 255])
    lab_red_lower = np.array([0, 0, 0])
    lab_red_upper = np.array([81, 255, 102])

    while True:
        # --- NEW: Check for connection and try to reconnect if lost ---
        if arduino is None:
            initialize_serial(SERIAL_PORT, BAUDRATE)
            if arduino is None: # Still no connection
                # Draw a warning on the frame
                frame = np.zeros((720, 1280, 3), dtype=np.uint8)
                cv2.putText(frame, "ESP32 Disconnected", (50, 360), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 3)
                (flag, encodedImage) = cv2.imencode(".jpg", frame)
                if flag:
                    yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + bytearray(encodedImage) + b'\r\n')
                time.sleep(1) # Wait before retrying connection
                continue # Skip the rest of the loop
        
        # --- Normal processing ---
        frame = picam2.capture_array()
        frame = cv2.flip(frame, -1)
        
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame_lab = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2LAB)

        red_mask = detect_color_in_lab(frame_lab, lab_red_lower, lab_red_upper)
        # green_mask = detect_color_in_lab(frame_lab, lab_green_lower, lab_green_upper)
        red_contour = find_largest_contour(red_mask)
        # green_contour = find_largest_contour(green_mask)
        
        # --- NEW LOGIC: Choose the CLOSEST (Taller) object, not the LARGEST ---
        primary_contour = None
        if red_contour is not None :
            # Get the height of the bounding box for each contour
            _, _, _, h_red = cv2.boundingRect(red_contour)
            # _, _, _, h_green = cv2.boundingRect(green_contour)
            # The contour with the greater height is closer to the camera
            primary_contour = red_contour 
        elif red_contour is not None:
            primary_contour = red_contour
        # elif green_contour is not None:
        #     primary_contour = green_contour
        # --- END OF NEW LOGIC ---

        if primary_contour is not None: # Changed from largest_contour
            x, y, w, h = cv2.boundingRect(primary_contour)
            cv2.rectangle(frame_rgb, (x, y), (x + w, y + h), (0, 255, 0), 2)
            distance = calculate_distance(FOCAL_LENGTH, KNOWN_OBSTACLE_HEIGHT_CM, h)
            travel_dist, turn_angle = calculate_maneuver(frame_rgb.shape, (x, y, w, h), distance)
            info_text = f"Dist: {travel_dist:.1f}cm, Angle: {turn_angle:.1f}deg"
            cv2.putText(frame_rgb, info_text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # --- Try-except block for writing data ---
            try:
                dist_int = int(travel_dist * 10)
                angle_int = int(turn_angle * 10)
                packed_data = struct.pack('<HH', dist_int, angle_int)
                arduino.write(packed_data)
                print(f"Sent bytes for: Dist={dist_int}, Angle={angle_int}")
                time.sleep(1)
            except (serial.SerialException, OSError) as e:
                print(f"ERROR: Write failed. ESP32 disconnected? {e}")
                if arduino:
                    arduino.close()
                arduino = None # Set to None to trigger reconnection attempt
        
        (flag, encodedImage) = cv2.imencode(".jpg", frame_rgb)
        if flag:
            yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + bytearray(encodedImage) + b'\r\n')

# --- Flask Routes (Unchanged) ---
@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    return """
    <html><head><title>WRO Obstacle Detection Stream</title>
    <style>body{font-family:sans-serif;text-align:center;background-color:#282c34;color:white;}img{border:2px solid #61dafb;margin-top:20px;}</style>
    </head><body><h2>WRO Obstacle Detection (LAB)</h2>
    <img src="/video_feed" width="1280" height="720"></body></html>
    """

if __name__ == '__main__':
    initialize_camera()
    # We no longer initialize serial here, the loop will handle it
    app.run(host='0.0.0.0', port=8080, threaded=True)
