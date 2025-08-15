from flask import Flask, Response, render_template_string, request, jsonify
import cv2
import numpy as np
import json
import os
from picamera2 import Picamera2

# ==============================
# Flask App
# ==============================
app = Flask(__name__)

# --- Camera Settings ---
OPTIMIZED_RESOLUTION = (1280, 720)

# Initialize Pi Camera
picam2 = Picamera2()
config = picam2.create_preview_configuration(
    main={"size": OPTIMIZED_RESOLUTION},
    raw={"size": (2304, 1296)}
)
picam2.configure(config)
picam2.set_controls({"ExposureTime": 10000})
picam2.start()

# --- LAB range defaults ---
lab_range = {
    "l_min": 0, "l_max": 255,
    "a_min": 0, "a_max": 255,
    "b_min": 0, "b_max": 255
}

# --- Color profiles dictionary ---
color_profiles = {
    "red": {"lower": [0, 0, 0], "upper": [255, 175, 69], "offset_adjust": 20},
    "green": {"lower": [0, 0, 135], "upper": [255, 113, 255], "offset_adjust": -20},
    "orange": {"lower": [0, 120, 81], "upper": [255, 149, 100], "offset_adjust": 90},
    "blue": {"lower": [0, 128, 138], "upper": [119, 160, 252], "offset_adjust": -90}
}

selected_color = "red"  # default color

# ==============================
# Routes
# ==============================


@app.route('/')
def index():
    return render_template_string(HTML_PAGE,
                                  colors=list(color_profiles.keys()),
                                  selected_color=selected_color,
                                  **lab_range)


@app.route('/set_lab', methods=['POST'])
def set_lab():
    """Update LAB values from sliders"""
    global lab_range
    for key in lab_range:
        if key in request.form:
            lab_range[key] = int(request.form[key])
    return ('', 204)


@app.route('/set_color', methods=['POST'])
def set_color():
    """Switch active color"""
    global selected_color, lab_range
    selected_color = request.form.get("color", selected_color)

    # Load LAB range from profile if available
    lower = color_profiles[selected_color]["lower"]
    upper = color_profiles[selected_color]["upper"]
    lab_range = {
        "l_min": lower[0], "a_min": lower[1], "b_min": lower[2],
        "l_max": upper[0], "a_max": upper[1], "b_max": upper[2]
    }
    return ('', 204)


@app.route('/save_profile', methods=['POST'])
def save_profile():
    """Save current LAB settings to file"""
    filename = request.form.get("filename", "color_profiles.json")
    filename = filename if filename.endswith(".json") else filename + ".json"

    color_profiles[selected_color]["lower"] = [
        lab_range["l_min"], lab_range["a_min"], lab_range["b_min"]
    ]
    color_profiles[selected_color]["upper"] = [
        lab_range["l_max"], lab_range["a_max"], lab_range["b_max"]
    ]

    with open(filename, "w") as f:
        json.dump(color_profiles, f, indent=4)

    return jsonify({"status": "saved", "file": filename})


@app.route('/video_feed')
def video_feed():
    def generate():
        while True:
            frame = picam2.capture_array()
            frame = cv2.flip(frame, -1)
            brightness, contrast = 3, 1.4
            frame = cv2.addWeighted(frame, contrast, np.zeros(
                frame.shape, frame.dtype), 0, brightness)

            # Convert to LAB and filter
            lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
            lab = cv2.GaussianBlur(lab, (7, 7), 0)
            l, a, b = cv2.split(lab)

            clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
            cl = clahe.apply(l)
            lab_clahe = cv2.merge((cl, a, b))

            lower = (lab_range["l_min"],
                     lab_range["a_min"], lab_range["b_min"])
            upper = (lab_range["l_max"],
                     lab_range["a_max"], lab_range["b_max"])

            mask = cv2.inRange(lab_clahe, lower, upper)
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            result = cv2.bitwise_and(frame, frame, mask=mask)
            _, buffer = cv2.imencode('.jpg', result)
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')


# ==============================
# HTML Template
# ==============================
HTML_PAGE = '''
<!DOCTYPE html>
<html>
<head>
    <title>Color Calibration Tool</title>
    <style>
        body { background: #111; color: #eee; font-family: Arial, sans-serif; text-align: center; }
        .container { display: flex; justify-content: center; gap: 50px; }
        .sliders { width: 300px; text-align: left; }
        label { display: block; margin-top: 10px; }
        input[type=range] { width: 100%; }
        button { margin-top: 10px; padding: 5px 10px; }
    </style>
</head>
<body>
    <h1>Interactive LAB Color Calibration</h1>

    <div>
        <label>Select Color:</label>
        <select id="colorSelect" onchange="changeColor()">
            {% for c in colors %}
            <option value="{{c}}" {% if c == selected_color %}selected{% endif %}>{{c}}</option>
            {% endfor %}
        </select>
    </div>

    <div class="container">
        <div class="sliders">
            <form id="labForm">
                {% for key in ['l_min','l_max','a_min','a_max','b_min','b_max'] %}
                    <label>{{ key }}: <span id="{{key}}_val">{{ lab_range[key] if lab_range else 0 }}</span></label>
                    <input type="range" name="{{key}}" min="0" max="255" value="{{ lab_range[key] if lab_range else 0 }}" oninput="updateVal(this)">
                {% endfor %}
            </form>

            <input type="text" id="filename" placeholder="Enter file name" />
            <button onclick="saveProfile()">Save to File</button>
        </div>

        <div>
            <img id="video-frame" src="/video_feed" width="640" height="480" />
        </div>
    </div>

    <script>
        function updateVal(slider) {
            document.getElementById(slider.name + '_val').textContent = slider.value;
            const formData = new FormData(document.getElementById('labForm'));
            fetch('/set_lab', { method: 'POST', body: formData });
        }

        function changeColor() {
            const formData = new FormData();
            formData.append("color", document.getElementById('colorSelect').value);
            fetch('/set_color', { method: 'POST', body: formData })
            .then(() => location.reload());
        }

        function saveProfile() {
            const formData = new FormData();
            formData.append("filename", document.getElementById('filename').value);
            fetch('/save_profile', { method: 'POST', body: formData })
                .then(r => r.json())
                .then(data => alert("Saved to " + data.file));
        }
    </script>
</body>
</html>
'''

# ==============================
# Run Server
# ==============================
if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8080, threaded=True)
