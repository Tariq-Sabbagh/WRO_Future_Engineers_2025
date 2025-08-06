from flask import Flask, Response, render_template_string, request
import cv2
import numpy as np
import time
from picamera2 import Picamera2



# --- Configuration ---
CALIBRATION_FILE = "calibration_data.npz"  # Calibration data file
OPTIMIZED_RESOLUTION = (1280, 720)

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
FOCAL_LENGTH = mtx_scaled[1, 1]  # Use calibrated focal length

# Precompute undistortion maps for performance
map1, map2 = cv2.initUndistortRectifyMap(
    mtx_scaled, dist, None, mtx_scaled, 
    (OPTIMIZED_RESOLUTION[0], OPTIMIZED_RESOLUTION[1]), 
    cv2.CV_16SC2
)
app = Flask(__name__)

# Initialize Pi Camera
picam2 = Picamera2()
config = picam2.create_preview_configuration(
        main={"size": OPTIMIZED_RESOLUTION},
        raw={"size": (2304, 1296)}  # Half-resolution binning
    )   
picam2.configure(config)
picam2.set_controls({"ExposureTime": 10000})
picam2.start()
picam2.set_controls({"AeEnable": True})
time.sleep(2)  # Camera warm-up

# Global LAB range dictionary
lab_range = {
    "l_min": 0,   "l_max": 255,
    "a_min": 0,   "a_max": 255,
    "b_min": 0,   "b_max": 255
}

@app.route('/')
def index():
    return render_template_string(HTML_PAGE, **lab_range)

@app.route('/set_lab', methods=['POST'])
def set_lab():
    for key in lab_range:
        if key in request.form:
            lab_range[key] = int(request.form[key])
    return ('', 204)

@app.route('/video_feed')
def video_feed():
    def generate():
        while True:
            frame = picam2.capture_array()
            frame = cv2.flip(frame, -1)
            lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
            lab = cv2.GaussianBlur(lab, (7, 7), 0)
            l, a, b = cv2.split(lab)

            # Apply CLAHE to L channel to normalize lighting
            clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
            cl = clahe.apply(l)

            # Merge back to LAB image
            lab_clahe = cv2.merge((cl, a, b))

            lower = (
                lab_range["l_min"],
                lab_range["a_min"],
                lab_range["b_min"]
            )
            upper = (
                lab_range["l_max"],
                lab_range["a_max"],
                lab_range["b_max"]
            )

            # LAB mask
            lab_mask = cv2.inRange(lab_clahe, lower, upper)
            
            kernel = np.ones((5, 5), np.uint8)
            lab_mask = cv2.morphologyEx(lab_mask, cv2.MORPH_OPEN, kernel)
            lab_mask = cv2.morphologyEx(lab_mask, cv2.MORPH_CLOSE, kernel)

            combined_mask = cv2.bitwise_and(lab_mask, lab_mask, mask=lab_mask)

            # Apply the combined mask
            result = cv2.bitwise_and(frame, frame, mask=combined_mask)

            _, buffer = cv2.imencode('.jpg', result)
            frame_bytes = buffer.tobytes()

            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

# Inline HTML Template
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
    </style>
</head>
<body>
    <h1>LAB Color Range Filter</h1>
    <div class="slider-container">
        <form id="labForm">
            <label>L min: <span id="l_min_val">{{ l_min }}</span></label>
            <input type="range" name="l_min" min="0" max="255" value="{{ l_min }}" oninput="updateVal(this)">
            <label>L max: <span id="l_max_val">{{ l_max }}</span></label>
            <input type="range" name="l_max" min="0" max="255" value="{{ l_max }}" oninput="updateVal(this)">

            <label>A min: <span id="a_min_val">{{ a_min }}</span></label>
            <input type="range" name="a_min" min="0" max="255" value="{{ a_min }}" oninput="updateVal(this)">
            <label>A max: <span id="a_max_val">{{ a_max }}</span></label>
            <input type="range" name="a_max" min="0" max="255" value="{{ a_max }}" oninput="updateVal(this)">

            <label>B min: <span id="b_min_val">{{ b_min }}</span></label>
            <input type="range" name="b_min" min="0" max="255" value="{{ b_min }}" oninput="updateVal(this)">
            <label>B max: <span id="b_max_val">{{ b_max }}</span></label>
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
