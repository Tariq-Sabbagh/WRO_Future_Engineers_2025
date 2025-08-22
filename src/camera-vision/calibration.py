from flask import Flask, Response, render_template_string, request, jsonify, send_file
import cv2
import numpy as np
import json
import os
import pathlib
import time
import threading

from camera_module import Camera

app = Flask(__name__)

# -----------------------
# Camera
# -----------------------

camera = Camera()

# -----------------------
# File Path Setup
# -----------------------
SCRIPT_DIR = pathlib.Path(__file__).parent.resolve()
SAVED_PROFILES_DIR = SCRIPT_DIR / "saved_profiles"
SAVED_PROFILES_DIR.mkdir(exist_ok=True, parents=True)
SAVED_IMAGES_DIR = SCRIPT_DIR / "saved_images"
SAVED_IMAGES_DIR.mkdir(exist_ok=True, parents=True)

# -----------------------
# State
# -----------------------
# Default profiles (LAB) to get you started
color_profiles = {
    "red":    {"lower": [0, 0, 0],      "upper": [255, 175, 69],  "offset_adjust": 20},
    "green":  {"lower": [0, 0, 135],    "upper": [255, 113, 255], "offset_adjust": -20},
    "orange": {"lower": [0, 120, 81],   "upper": [255, 149, 100], "offset_adjust": 90},
    "blue":   {"lower": [0, 128, 138],  "upper": [119, 160, 252], "offset_adjust": -90},
}

selected_color = "red"
view_mode = "overlay"      # 'rgb', 'lab', 'mask', 'overlay'
clahe_enabled = False
app_mode = "live"
current_image = None
image_lock = threading.Lock()

# Initialize lab_ranges for all colors
lab_ranges = {
    color: {
        "l_min": profile["lower"][0],
        "l_max": profile["upper"][0],
        "a_min": profile["lower"][1],
        "a_max": profile["upper"][1],
        "b_min": profile["lower"][2],
        "b_max": profile["upper"][2]
    } for color, profile in color_profiles.items()
}

# Current active range (points to one of the entries in lab_ranges)
current_lab_range = lab_ranges[selected_color]

# Overlay color in RGB
overlay_rgb = {
    "red":    (255, 0, 0),
    "green":  (0, 255, 0),
    "orange": (255, 140, 0),
    "blue":   (0, 0, 255),
}

# -----------------------
# Helpers
# -----------------------
def set_lab_from_profile(color):
    global selected_color, current_lab_range
    selected_color = color
    current_lab_range = lab_ranges[color]

def build_mask_from_lab(lab_img):
    lower = np.array([current_lab_range["l_min"], current_lab_range["a_min"], current_lab_range["b_min"]], dtype=np.uint8)
    upper = np.array([current_lab_range["l_max"], current_lab_range["a_max"], current_lab_range["b_max"]], dtype=np.uint8)
    mask = cv2.inRange(lab_img, lower, upper)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask

def apply_clahe_l(lab_img):
    l, a, b = cv2.split(lab_img)
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
    l_eq = clahe.apply(l)
    return cv2.merge((l_eq, a, b))

def process_frame(frame):
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    if clahe_enabled:
        lab = apply_clahe_l(lab)

    mask = build_mask_from_lab(lab)

    if view_mode == "rgb":
        out = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    elif view_mode == "lab":
        combined_mask = cv2.bitwise_and(mask, mask, mask=mask)
        out = cv2.bitwise_and(frame, frame, mask=combined_mask)
    elif view_mode == "mask":
        out = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
    elif view_mode == "overlay":
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        overlay = np.zeros_like(frame_rgb)
        color = overlay_rgb.get(selected_color, (255, 255, 255))
        overlay[mask > 0] = color
        out = cv2.addWeighted(frame_rgb, 1.0, overlay, 0.8, 0)
    else:
        out = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    return out

# -----------------------
# Routes
# -----------------------
@app.route("/")
def index():
    return render_template_string(HTML_PAGE,
        colors=list(color_profiles.keys()),
        selected_color=selected_color,
        lab_range=current_lab_range,
        view_mode=view_mode,
        clahe_enabled=clahe_enabled,
        app_mode=app_mode
    )

@app.route("/set_lab", methods=["POST"])
def set_lab():
    for key in current_lab_range:
        if key in request.form:
            current_lab_range[key] = int(request.form[key])
    return jsonify(current_lab_range)

@app.route("/set_color", methods=["POST"])
def set_color():
    color = request.form.get("color", selected_color)
    if color in color_profiles:
        set_lab_from_profile(color)
    return jsonify(current_lab_range)

@app.route("/set_view", methods=["POST"])
def set_view():
    global view_mode
    vm = request.form.get("view_mode", view_mode)
    if vm in {"rgb","lab","mask","overlay"}:
        view_mode = vm
    return ("", 204)

@app.route("/set_clahe", methods=["POST"])
def set_clahe():
    global clahe_enabled
    val = request.form.get("enabled", "false").lower()
    clahe_enabled = (val == "true")
    return ("", 204)

@app.route("/set_mode", methods=["POST"])
def set_mode():
    global app_mode
    mode = request.form.get("mode", app_mode)
    if mode in {"live", "image"}:
        app_mode = mode
    return jsonify({"mode": app_mode})

@app.route("/capture", methods=["POST"])
def capture_image():
    global app_mode, current_image, view_mode
    frame = camera.capture_frame()
    with image_lock:
        current_image = frame.copy()
    view_mode = "rgb"
    app_mode = "image"
    return jsonify({
        "status": "captured",
        "mode": app_mode,
        "view_mode": view_mode
    })

@app.route("/list_profiles")
def list_profiles():
    files = [f.name for f in SAVED_PROFILES_DIR.glob("*.json")]
    return jsonify(files)

@app.route("/save_image", methods=["GET"])
def save_image():
    timestamp = time.strftime("%Y%m%d-%H%M%S")
    filename = f"calibration_image_{timestamp}.jpg"
    save_path = SAVED_IMAGES_DIR / filename
    if app_mode == "image" and current_image is not None:
        processed = process_frame(current_image)
    else:
        with image_lock:
            frame = current_image.copy() if current_image is not None else np.zeros((480,640,3), dtype=np.uint8)
        processed = process_frame(frame)
    ok, buffer = cv2.imencode(".jpg", processed)
    if not ok:
        return jsonify({"error": "Failed to encode image"}), 500
    with open(save_path, "wb") as f:
        f.write(buffer.tobytes())
    return send_file(
        pathlib.Path(save_path),
        mimetype='image/jpeg',
        as_attachment=True,
        download_name=filename
    )

@app.route("/upload_image", methods=["POST"])
def upload_image():
    global current_image, app_mode, view_mode
    if 'file' not in request.files:
        return jsonify({"error": "No file part"}), 400
    file = request.files['file']
    if file.filename == '':
        return jsonify({"error": "No selected file"}), 400
    try:
        nparr = np.frombuffer(file.read(), np.uint8)
        img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        if img is None:
            return jsonify({"error": "Could not decode image"}), 400
        # convert to RGB for display
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        with image_lock:
            current_image = img
        view_mode = "overlay"
        app_mode = "image"
        return jsonify({
            "status": "uploaded",
            "mode": app_mode,
            "view_mode": view_mode
        })
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route("/save_profiles", methods=["POST"])
def save_profiles():
    filename = request.form.get("filename", "color_profiles.json").strip()
    if not filename:
        filename = "color_profiles.json"
    if not filename.endswith(".json"):
        filename += ".json"
    save_path = SAVED_PROFILES_DIR / filename
    
    # Update all color profiles with their current lab ranges
    for color, profile in color_profiles.items():
        profile["lower"] = [
            lab_ranges[color]["l_min"],
            lab_ranges[color]["a_min"],
            lab_ranges[color]["b_min"]
        ]
        profile["upper"] = [
            lab_ranges[color]["l_max"],
            lab_ranges[color]["a_max"],
            lab_ranges[color]["b_max"]
        ]
    
    payload = {
        "color_space": "LAB",
        "profiles": color_profiles
    }
    with open(save_path, "w") as f:
        json.dump(payload, f, indent=4)
    return jsonify({"status": "ok", "file": str(save_path)})

@app.route("/load_profiles", methods=["POST"])
def load_profiles():
    filename = request.form.get("filename", "").strip()
    if not filename:
        return jsonify({"status": "error", "message": "No filename"}), 400
    load_path = SAVED_PROFILES_DIR / filename
    if not load_path.exists():
        return jsonify({"status": "error", "message": "File not found"}), 404
    
    with open(load_path, "r") as f:
        data = json.load(f)
    
    profiles = data.get("profiles", data)
    if not isinstance(profiles, dict):
        return jsonify({"status": "error", "message": "Invalid file format"}), 400
    
    for color, values in profiles.items():
        lower = values.get("lower", [0,0,0])
        upper = values.get("upper", [255,255,255])
        offset_adjust = values.get("offset_adjust", 0)
        
        color_profiles[color] = {
            "lower": [int(lower[0]), int(lower[1]), int(lower[2])],
            "upper": [int(upper[0]), int(upper[1]), int(upper[2])],
            "offset_adjust": int(offset_adjust)
        }
        
        # Update the lab ranges for this color
        lab_ranges[color] = {
            "l_min": int(lower[0]),
            "l_max": int(upper[0]),
            "a_min": int(lower[1]),
            "a_max": int(upper[1]),
            "b_min": int(lower[2]),
            "b_max": int(upper[2])
        }
    
    # Sync the current range to the selected color
    if selected_color in color_profiles:
        set_lab_from_profile(selected_color)
    
    return jsonify({
        "status": "ok",
        "loaded_colors": list(color_profiles.keys()),
        "lab_range": current_lab_range
    })

@app.route("/video_feed")
def video_feed():
    def stream():
        while True:
            if app_mode == "live":
                frame = camera.capture_frame()
                out = process_frame(frame)
            else:
                with image_lock:
                    if current_image is not None:
                        out = process_frame(current_image)
                    else:
                        out = np.zeros((450, 800, 3), dtype=np.uint8)
            ok, buffer = cv2.imencode(".jpg", out, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if not ok:
                continue
            yield (b"--frame\r\n"
                  b"Content-Type: image/jpeg\r\n\r\n" + buffer.tobytes() + b"\r\n")
            time.sleep(0.05)
    return Response(stream(), mimetype="multipart/x-mixed-replace; boundary=frame")

# -----------------------
# HTML (inline) - Updated with new controls
# -----------------------
HTML_PAGE = """
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8" />
  <title>Interactive Color Calibration</title>
  <style>
    :root {
      --bg-dark: #111;
      --panel-bg: #1a1a1a;
      --text-color: #eee;
      --text-muted: #bbb;
      --border-color: #333;
      --active-color: #4CAF50;
      --button-bg: #2a2a2a;
      --button-hover: #333;
      --slider-height: 8px;
    }
    
    * {
      box-sizing: border-box;
      margin: 0;
      padding: 0;
    }
    
    body {
      background: var(--bg-dark);
      color: var(--text-color);
      font-family: Arial, sans-serif;
      line-height: 1.5;
      padding: 20px;
      height: 100vh;
      overflow: hidden;
    }
    
    h1 {
      text-align: center;
      margin-bottom: 20px;
    }
    
    .app-container {
      display: grid;
      grid-template-columns: 350px 1fr;
      gap: 20px;
      height: calc(100vh - 80px);
    }
    
    .controls-panel {
      background: var(--panel-bg);
      padding: 16px;
      border-radius: 10px;
      overflow-y: auto;
    }
    
    .video-panel {
      background: var(--panel-bg);
      padding: 16px;
      border-radius: 10px;
      display: flex;
      flex-direction: column;
    }
    
    .video-container {
      flex-grow: 1;
      display: flex;
      flex-direction: column;
    }
    
    #video {
      width: 100%;
      height: auto;
      max-height: calc(100vh - 200px);
      object-fit: contain;
      border: 2px solid var(--border-color);
      border-radius: 8px;
    }
    
    label {
      display: block;
      margin-top: 12px;
      font-weight: bold;
    }
    
    input[type=range] {
      width: 100%;
      height: var(--slider-height);
      -webkit-appearance: none;
      background: var(--border-color);
      border-radius: 4px;
      margin: 8px 0;
    }
    
    input[type=range]::-webkit-slider-thumb {
      -webkit-appearance: none;
      width: 16px;
      height: 16px;
      background: var(--text-color);
      border-radius: 50%;
      cursor: pointer;
    }
    
    select, input[type=text], button {
      padding: 8px 12px;
      border-radius: 8px;
      border: none;
      background: var(--button-bg);
      color: var(--text-color);
      font-size: 14px;
    }
    
    button {
      cursor: pointer;
      transition: background 0.2s;
    }
    
    button:hover {
      background: var(--button-hover);
    }
    
    .inline-group {
      display: flex;
      gap: 8px;
      align-items: center;
      flex-wrap: wrap;
      margin: 8px 0;
    }
    
    .small {
      font-size: 12px;
      color: var(--text-muted);
    }
    
    .divider {
      height: 1px;
      background: var(--border-color);
      margin: 16px 0;
    }
    
    .mode-active {
      background: var(--active-color) !important;
    }
    
    .slider-container {
      display: flex;
      flex-direction: column;
      gap: 16px;
    }

    .channel-group {
      display: flex;
      flex-direction: column;
      gap: 8px;
      padding: 12px;
      background: var(--button-bg);
      border-radius: 8px;
    }

    .channel-group label {
      text-align: center;
      margin: 0 0 8px 0;
      font-weight: bold;
    }

    .slider-pair {
      display: flex;
      flex-direction: column;
      gap: 4px;
    }

    input[type=range] {
      width: 100%;
      height: var(--slider-height);
      -webkit-appearance: none;
      background: var(--border-color);
      border-radius: 4px;
      margin: 4px 0;
    }

    
    .slider-value {
      display: flex;
      justify-content: space-between;
      font-size: 12px;
    }
    
    .profile-list {
      max-height: 150px;
      overflow-y: auto;
      margin: 10px 0;
      border: 1px solid var(--border-color);
      border-radius: 8px;
      padding: 5px;
    }
    
    .profile-item {
      padding: 5px;
      cursor: pointer;
      font-size: 13px;
    }
    
    .profile-item:hover {
      background: var(--button-hover);
    }
    
    .profile-header {
      display: flex;
      justify-content: space-between;
      align-items: center;
      margin-bottom: 8px;
    }
    
    .checkbox-group {
      display: flex;
      align-items: center;
      gap: 8px;
      margin: 8px 0;
    }
    
    input[type="checkbox"] {
      width: 16px;
      height: 16px;
    }
    
    /* Custom scrollbar */
    ::-webkit-scrollbar {
      width: 8px;
    }
    
    ::-webkit-scrollbar-track {
      background: var(--panel-bg);
    }
    
    ::-webkit-scrollbar-thumb {
      background: var(--border-color);
      border-radius: 4px;
    }
    
    ::-webkit-scrollbar-thumb:hover {
      background: #555;
    }
  </style>
</head>
<body>
  <h1>Interactive Color Calibration</h1>

  <div class="app-container">
    <div class="controls-panel">
      <div class="mode-section">
        <label>Application Mode:</label>
        <div class="inline-group">
          <button id="liveModeBtn" onclick="setMode('live')" class="{% if app_mode=='live' %}mode-active{% endif %}">Live Video</button>
          <button id="imageModeBtn" onclick="setMode('image')" class="{% if app_mode=='image' %}mode-active{% endif %}">Image Mode</button>
        </div>
      </div>

      <div class="divider"></div>

      <div class="mode-section">
        <label>Image Controls:</label>
        <div class="inline-group">
          <button onclick="captureImage()">Capture</button>
          <button onclick="document.getElementById('fileInput').click()">Load</button>
          <button onclick="saveImage()">Save</button>
          <input type="file" id="fileInput" style="display:none" accept="image/*" onchange="uploadImage(this.files[0])">
        </div>
        <div class="small">Capture: saves current frame to image mode</div>
        <div class="small">Save: downloads current view as image file</div>
      </div>

      <div class="divider"></div>

      <label>Color:</label>
      <div class="inline-group">
        <select id="colorSelect" onchange="onColorChange()">
          {% for c in colors %}
          <option value="{{c}}" {% if c == selected_color %}selected{% endif %}>{{c}}</option>
          {% endfor %}
        </select>
        <span class="small">Switching updates sliders</span>
      </div>

      <div class="divider"></div>

      <label>View Mode:</label>
      <div class="inline-group">
        <select id="viewMode" onchange="onViewChange()">
          <option value="rgb" {% if view_mode=='rgb' %}selected{% endif %}>RGB</option>
          <option value="lab" {% if view_mode=='lab' %}selected{% endif %}>LAB</option>
          <option value="mask" {% if view_mode=='mask' %}selected{% endif %}>Mask</option>
          <option value="overlay" {% if view_mode=='overlay' %}selected{% endif %}>Overlay</option>
        </select>
      </div>

      <div class="checkbox-group">
        <input type="checkbox" id="clahe" onchange="onClaheChange()" {% if clahe_enabled %}checked{% endif %}/>
        <label for="clahe">Normalize lighting (CLAHE on L)</label>
      </div>

      <div class="divider"></div>

      <form id="labForm">
        <label>LAB Color Range:</label>
        <div class="slider-container">
          <div class="channel-group">
            <label>L Channel</label>
            <div class="slider-pair">
              <div class="slider-value">
                <span>Min: </span>
                <span id="l_min_val">{{ lab_range['l_min'] }}</span>
              </div>
              <input type="range" name="l_min" min="0" max="255" value="{{ lab_range['l_min'] }}" oninput="updateSlider(this)">
              
              <div class="slider-value">
                <span>Max: </span>
                <span id="l_max_val">{{ lab_range['l_max'] }}</span>
              </div>
              <input type="range" name="l_max" min="0" max="255" value="{{ lab_range['l_max'] }}" oninput="updateSlider(this)">
            </div>
          </div>

          <div class="channel-group">
            <label>A Channel</label>
            <div class="slider-pair">
              <div class="slider-value">
                <span>Min: </span>
                <span id="a_min_val">{{ lab_range['a_min'] }}</span>
              </div>
              <input type="range" name="a_min" min="0" max="255" value="{{ lab_range['a_min'] }}" oninput="updateSlider(this)">
              
              <div class="slider-value">
                <span>Max: </span>
                <span id="a_max_val">{{ lab_range['a_max'] }}</span>
              </div>
              <input type="range" name="a_max" min="0" max="255" value="{{ lab_range['a_max'] }}" oninput="updateSlider(this)">
            </div>
          </div>

          <div class="channel-group">
            <label>B Channel</label>
            <div class="slider-pair">
              <div class="slider-value">
                <span>Min: </span>
                <span id="b_min_val">{{ lab_range['b_min'] }}</span>
              </div>
              <input type="range" name="b_min" min="0" max="255" value="{{ lab_range['b_min'] }}" oninput="updateSlider(this)">
              
              <div class="slider-value">
                <span>Max: </span>
                <span id="b_max_val">{{ lab_range['b_max'] }}</span>
              </div>
              <input type="range" name="b_max" min="0" max="255" value="{{ lab_range['b_max'] }}" oninput="updateSlider(this)">
            </div>
          </div>
        </div>
      </form>
      <div class="divider"></div>

      <div>
        <div class="profile-header">
          <label>Color Profiles:</label>
          <button onclick="refreshProfileList()">Refresh</button>
        </div>
        <div class="inline-group">
          <input type="text" id="filename" placeholder="my_colors.json" />
          <button onclick="saveProfiles()">Save</button>
        </div>
        <div class="small">Saved in: saved_profiles/</div>

        <div class="profile-list" id="profileList">
          <div class="small">Loading profiles...</div>
        </div>
      </div>
    </div>

    <div class="video-panel">
      <div class="video-container">
        <img id="video" src="/video_feed"/>
        <div class="small" style="margin-top:8px;">
          RGB shows true camera colors. LAB view is reconstructed from LAB space.
          L/A/B views show individual channels. Overlay shows mask on original.
        </div>
        <div class="small">Current Mode: <span id="currentMode">{{ app_mode }}</span></div>
      </div>
    </div>
  </div>

<script>
// --- Global variable to keep current lab_range ---
let current_lab_range = {
  "l_min": {{ lab_range['l_min'] }},
  "l_max": {{ lab_range['l_max'] }},
  "a_min": {{ lab_range['a_min'] }},
  "a_max": {{ lab_range['a_max'] }},
  "b_min": {{ lab_range['b_min'] }},
  "b_max": {{ lab_range['b_max'] }}
};

// Function to set sliders to current lab_range
function syncSlidersToLabRange() {
  const keys = ['l_min','l_max','a_min','a_max','b_min','b_max'];
  keys.forEach(key => {
    const slider = document.querySelector(`input[name="${key}"]`);
    const span = document.getElementById(`${key}_val`);
    if (slider && span) {
      slider.value = current_lab_range[key];
      span.textContent = current_lab_range[key];
    }
  });
}

// Initialize sliders on page load
document.addEventListener("DOMContentLoaded", () => {
  syncSlidersToLabRange();
  refreshProfileList();
});

// Update current_lab_range when sliders change
function updateSlider(slider) {
  const valueSpan = document.getElementById(`${slider.name}_val`);
  if (valueSpan) {
    valueSpan.textContent = slider.value;
  }
  
  // Update the local copy
  current_lab_range[slider.name] = parseInt(slider.value);
  
  // Send the update to the server
  const formData = new FormData();
  formData.append(slider.name, slider.value);
  fetch('/set_lab', {
    method: 'POST',
    body: formData
  });
}

// When changing color, update lab_range and sliders
function onColorChange() {
  const fd = new FormData();
  fd.append("color", document.getElementById("colorSelect").value);
  fetch("/set_color", {method:"POST", body:fd})
    .then(response => response.json())
    .then(data => {
      current_lab_range = data;
      syncSlidersToLabRange();
    });
}

// When changing view mode
function onViewChange() {
  const fd = new FormData();
  fd.append("view_mode", document.getElementById("viewMode").value);
  fetch("/set_view", {method:"POST", body:fd});
}

function onClaheChange() {
  const fd = new FormData();
  fd.append("enabled", document.getElementById("clahe").checked ? "true" : "false");
  fetch("/set_clahe", {method:"POST", body:fd});
}

function setMode(mode) {
  const fd = new FormData();
  fd.append("mode", mode);
  fetch("/set_mode", {method:"POST", body:fd})
    .then(response => response.json())
    .then(data => {
      document.getElementById("currentMode").textContent = data.mode;
      document.getElementById("liveModeBtn").classList.toggle("mode-active", data.mode === "live");
      document.getElementById("imageModeBtn").classList.toggle("mode-active", data.mode === "image");
    });
}

function captureImage() {
  fetch("/capture", {method:"POST"})
    .then(response => response.json())
    .then(data => {
      setMode(data.mode);
      if (data.view_mode) {
        document.getElementById("viewMode").value = data.view_mode;
      }
      alert("Image captured! Switched to image mode.");
    });
}

function uploadImage(file) {
  if (!file) return;
  const formData = new FormData();
  formData.append("file", file);
  fetch("/upload_image", {method:"POST", body:formData})
    .then(response => response.json())
    .then(data => {
      if (data.error) {
        alert("Error: " + data.error);
      } else {
        setMode(data.mode);
        if (data.view_mode) {
          document.getElementById("viewMode").value = data.view_mode;
        }
        alert("Image uploaded! Switched to image mode.");
      }
    })
    .catch(error => {
      alert("Upload failed: " + error);
    });
}

function saveImage() {
  window.open("/save_image", "_blank");
}

function saveProfiles() {
  const fn = document.getElementById("filename").value || "color_profiles.json";
  const fd = new FormData();
  fd.append("filename", fn);
  fetch("/save_profiles", {method:"POST", body:fd})
    .then(r=>r.json())
    .then(d=> {
      alert("Saved to: " + d.file);
      refreshProfileList();
    });
}

function refreshProfileList() {
  fetch("/list_profiles")
    .then(r => r.json())
    .then(files => {
      const list = document.getElementById("profileList");
      list.innerHTML = "";
      if (files.length === 0) {
        list.innerHTML = "<div class='small'>No profiles saved yet</div>";
        return;
      }
      files.forEach(file => {
        const div = document.createElement("div");
        div.className = "profile-item";
        div.textContent = file;
        div.onclick = () => {
          if (confirm(`Load profile: ${file}?`)) {
            const fd = new FormData();
            fd.append("filename", file);
            fetch("/load_profiles", {method: "POST", body: fd})
              .then(r => r.json())
              .then(data => {
                if (data.status === "ok") {
                  alert(`Loaded: ${file}`);
                  current_lab_range = data.lab_range;
                  syncSlidersToLabRange();
                  // Update color select if the loaded profile has the current color
                  if (data.loaded_colors.includes(document.getElementById("colorSelect").value)) {
                    onColorChange();
                  }
                }
              });
          }
        };
        list.appendChild(div);
      });
    });
}
</script>
</body>
</html>
"""

# -----------------------
# Run
# -----------------------
if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8080, threaded=True)