from flask import Flask, Response, render_template_string, request
import cv2
import numpy as np
import time
from picamera2 import Picamera2

app = Flask(__name__)

# Constants (Match first code)
CALIBRATION_FILE = "calibration_data.npz"
OPTIMIZED_RESOLUTION = (1280, 720)
KNOWN_OBSTACLE_HEIGHT_CM = 10.0
FOCAL_LENGTH = 570.0

# --- Camera Initialization (Fixed) ---
def initialize_camera():
    global picam2, map1, map2
    try:
        # Match first code's camera config
        picam2 = Picamera2()
        config = picam2.create_preview_configuration(
            main={"size": OPTIMIZED_RESOLUTION},
            raw={"size": (2304, 1296)}  # Match raw resolution
        )
        picam2.configure(config)
        picam2.set_controls({"ExposureTime": 9000})  # Critical setting
        picam2.start()
        print("Camera initialized.")
        
        # Load calibration AFTER camera init
        load_calibration()
        return True
    except Exception as e:
        print(f"Camera init error: {str(e)}")
        return False

# --- Calibration Loading (Fixed) ---
def load_calibration():
    global map1, map2
    try:
        calibration_data = np.load(CALIBRATION_FILE)
        mtx = calibration_data['mtx']
        dist = calibration_data['dist']

        # Proper scaling (match first code)
        SCALE_X = OPTIMIZED_RESOLUTION[0] / 4608
        SCALE_Y = OPTIMIZED_RESOLUTION[1] / 2592
        mtx_scaled = mtx.copy()
        mtx_scaled[0, 0] *= SCALE_X
        mtx_scaled[1, 1] *= SCALE_Y
        mtx_scaled[0, 2] *= SCALE_X
        mtx_scaled[1, 2] *= SCALE_Y

        # Correct undistortion maps
        map1, map2 = cv2.initUndistortRectifyMap(
            mtx_scaled, dist, None, mtx_scaled,
            OPTIMIZED_RESOLUTION,
            cv2.CV_16SC2
        )
        print("Calibration loaded.")
    except Exception as e:
        print(f"Calibration error: {str(e)}")
        map1, map2 = None, None

# --- Frame Capture (Fixed) ---
def capture_frame():
    global picam2
    if picam2 is None:
        return np.zeros((720, 1280, 3), dtype=np.uint8)  # Fallback
    
    # Match first code's capture process:
    frame = picam2.capture_array()
    frame = cv2.flip(frame, -1)  # Match flip
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # Critical conversion
    return frame

# Initialize on startup
if not initialize_camera():
    print("Failed to initialize camera")

# ... rest of code unchanged ...

@app.route('/video_feed')
def video_feed():
    def generate():
        while True:
        
            frame = capture_frame()  # Now returns RGB
            
            # Apply undistortion (if available)
            if map1 is not None and map2 is not None:
                frame = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)
            
            # Convert to LAB correctly: RGB → LAB
            lab = cv2.cvtColor(frame, cv2.COLOR_RGB2LAB)  # FIXED CONVERSION
            
            # ... rest of processing ...
            
            # Final output remains RGB
            _, buffer = cv2.imencode('.jpg', frame)
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

# --- HTML Interface ---
HTML_PAGE = '''
<!DOCTYPE html>
<html>
<head>
    <title>LAB Color Filter Tool</title>
    <style>
        body { background: #111; color: #eee; font-family: Arial, sans-serif; text-align: center; }
        .slider-container { width: 300px; margin: 0 auto; padding: 20px; text-align: left; }
        label { display: block; margin: 10px 0 4px; }
        input[type=range] { width: 100%; }
        #video-frame { margin-top: 20px; border: 2px solid #444; }
        .value-display { display: inline-block; width: 40px; text-align: right; }
    </style>
</head>
<body>
    <h1>LAB Color Range Filter</h1>
    <div class="slider-container">
        <form id="labForm">
            <label>L min: <span class="value-display" id="l_min_val">{{ l_min }}</span></label>
            <input type="range" name="l_min" min="0" max="255" value="{{ l_min }}" oninput="updateVal(this)">
            <label>L max: <span class="value-display" id="l_max_val">{{ l_max }}</span></label>
            <input type="range" name="l_max" min="0" max="255" value="{{ l_max }}" oninput="updateVal(this)">

            <label>A min: <span class="value-display" id="a_min_val">{{ a_min }}</span></label>
            <input type="range" name="a_min" min="0" max="255" value="{{ a_min }}" oninput="updateVal(this)">
            <label>A max: <span class="value-display" id="a_max_val">{{ a_max }}</span></label>
            <input type="range" name="a_max" min="0" max="255" value="{{ a_max }}" oninput="updateVal(this)">

            <label>B min: <span class="value-display" id="b_min_val">{{ b_min }}</span></label>
            <input type="range" name="b_min" min="0" max="255" value="{{ b_min }}" oninput="updateVal(this)">
            <label>B max: <span class="value-display" id="b_max_val">{{ b_max }}</span></label>
            <input type="range" name="b_max" min="0" max="255" value="{{ b_max }}" oninput="updateVal(this)">
        </form>
    </div>

    <img id="video-frame" src="/video_feed" width="640" height="480" />

    <script>
        function updateVal(slider) {
            document.getElementById(slider.name + '_val').textContent = slider.value;

            const formData = new FormData(document.getElementById('labForm'));
            fetch('/set_lab', {
                method: 'POST',
                body: formData
            });
        }
    </script>
</body>
</html>
'''

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8080, threaded=True)