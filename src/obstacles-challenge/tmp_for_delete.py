# ... [Keep all previous imports and class definition as-is] ...
import serial
import time
import cv2
import numpy as np
import struct
from picamera2 import Picamera2


class ObstacleDetector:
    def __init__(self, debug_mode=False, serial_port='/dev/ttyUSB0'):
        # Configuration
        self.SERIAL_PORT = serial_port
        self.BAUDRATE = 115200
        self.CALIBRATION_FILE = "calibration_data.npz"
        self.OPTIMIZED_RESOLUTION = (1280, 720)
        self.KNOWN_OBSTACLE_HEIGHT_CM = 10.0
        self.FOCAL_LENGTH = 570.0
        self.MAX_DISTANCE_CM = 90.0
        self.debug_mode = debug_mode
        self.last_turn_time = 0
        self.turn_cooldown = 3

        self.COLOR_PROFILES = {
            'red': {
                'lower': np.array([0, 135, 0]),
                'upper': np.array([134, 255, 108]),
                'offset_adjust': 15
            },
            'green': {
                'lower': np.array([0, 0, 151]),
                'upper': np.array([115, 110, 255]),
                'offset_adjust': -15
            },
            'orange': {
                'lower': np.array([0, 0, 0]),
                'upper': np.array([255, 136, 106]),
                'offset_adjust': 90
            },
            'blue': {
                'lower': np.array([0, 130, 145]),
                'upper': np.array([255, 255, 255]),
                'offset_adjust': -90
            }
        }

        self.picam2 = None
        self.arduino = None

        self.initialize_camera()
        self.load_calibration()

        if not self.debug_mode:
            self.initialize_serial()

    def load_calibration(self):
        calibration_data = np.load(self.CALIBRATION_FILE)
        mtx = calibration_data['mtx']
        dist = calibration_data['dist']

        SCALE_X = self.OPTIMIZED_RESOLUTION[0] / 4608
        SCALE_Y = self.OPTIMIZED_RESOLUTION[1] / 2592
        self.mtx_scaled = mtx.copy()
        self.mtx_scaled[0, 0] *= SCALE_X
        self.mtx_scaled[1, 1] *= SCALE_Y
        self.mtx_scaled[0, 2] *= SCALE_X
        self.mtx_scaled[1, 2] *= SCALE_Y

        self.map1, self.map2 = cv2.initUndistortRectifyMap(
            self.mtx_scaled, dist, None, self.mtx_scaled,
            self.OPTIMIZED_RESOLUTION,
            cv2.CV_16SC2
        )

    def initialize_camera(self):
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(
            main={"size": self.OPTIMIZED_RESOLUTION},
            raw={"size": (2304, 1296)}
        )
        self.picam2.configure(config)
        self.picam2.set_controls({"ExposureTime": 9000})
        self.picam2.start()
        print("Camera initialized.")

    def initialize_serial(self):
        try:
            self.arduino = serial.Serial(
                port=self.SERIAL_PORT,
                baudrate=self.BAUDRATE,
                timeout=0.1
            )
            print(f"Serial connection established on {self.SERIAL_PORT}.")
            self.arduino.write(b'TEST')
            self.arduino.flush()
        except serial.SerialException as e:
            print(f"Serial error: {str(e)}")
            self.arduino = None

    def detect_color(self, lab_frame, color_name):
        profile = self.COLOR_PROFILES[color_name]
        mask = cv2.inRange(lab_frame, profile['lower'], profile['upper'])
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        return mask

    def find_largest_contour(self, mask, min_area=300):
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        largest_contour = max(contours, key=cv2.contourArea)
        return largest_contour if cv2.contourArea(largest_contour) > min_area else None

    def calculate_distance(self, pixel_height):
        return (self.KNOWN_OBSTACLE_HEIGHT_CM * self.FOCAL_LENGTH) / pixel_height if pixel_height else 0

    def calculate_maneuver(self, frame_shape, bounding_rect, distance_cm, offset_adjust=15):
        x, y, w, h = bounding_rect
        obj_center_x = x + w / 2
        frame_center_x = frame_shape[1] / 2
        pixel_offset = obj_center_x - frame_center_x
        cm_per_pixel = self.KNOWN_OBSTACLE_HEIGHT_CM / h
        offset_x_cm = (pixel_offset * cm_per_pixel) + offset_adjust
        tendon = np.sqrt(offset_x_cm**2 + distance_cm**2)
        angle = np.arctan2(offset_x_cm, distance_cm)
        turn_angle = np.degrees(angle)
        return tendon, turn_angle

    def process_obstacle(self, contour, frame_rgb, color_type):
        profile = self.COLOR_PROFILES[color_type]
        x, y, w, h = cv2.boundingRect(contour)
        distance = self.calculate_distance(h)
        travel_dist, turn_angle = self.calculate_maneuver(frame_rgb.shape, (x, y, w, h), distance, profile['offset_adjust'])

        if travel_dist >= self.MAX_DISTANCE_CM:
            return False

        info_text = f"{color_type.upper()}: {travel_dist:.1f}cm, {turn_angle:.1f}deg"
        cv2.rectangle(frame_rgb, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(frame_rgb, info_text, (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        if not self.debug_mode:
            self.send_command('AVOID', travel_dist, turn_angle)

        print(f"{color_type.upper()}: Dist={travel_dist:.1f}cm, Angle={turn_angle:.1f}deg")
        return True

    def send_command(self, cmd_type, value1=0, value2=0):
        if self.arduino is None:
            return
        try:
            if cmd_type == 'AVOID':
                header = b'A'
                val1 = int(round(value1 * 10))
                val2 = int(round(value2 * 10))
            elif cmd_type == 'TURN':
                header = b'T'
                val1 = int(round(value1 * 10))
                val2 = 0
            else:
                print(f"Unknown command type: {cmd_type}")
                return
            packet = struct.pack('<chh', header, val1, val2)
            self.arduino.write(packet)
            self.arduino.flush()
        except (serial.SerialException, OSError) as e:
            print(f"ERROR: Failed to send command: {e}")
            self.arduino.close()
            self.arduino = None

    def capture_frame(self):
        frame = self.picam2.capture_array()
        frame = cv2.flip(frame, -1)
        return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    def detect_obstacles(self, frame_rgb):
        frame_lab = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2LAB)
        height, width, _ = frame_rgb.shape
        roi_top = int(height * 0.25)
        roi_bottom = int(height * 0.8)
        band_width = int(width * 0.25)
        roi_left = frame_lab[roi_top:roi_bottom, 0:band_width]
        roi_right = frame_lab[roi_top:roi_bottom, width - band_width:width]

        contours = {}

        for color in ['red', 'green']:
            mask_left = self.detect_color(roi_left, color)
            contour_left = self.find_largest_contour(mask_left)
            if contour_left is not None:
                contour_left += [0, roi_top]

            

            contours[color] = {
                'front': contour_front,
                
            }

            if self.debug_mode:
                for c, color_bgr in [(contour_left, (255, 0, 0)), (contour_right, (0, 255, 0))]:
                    if c is not None:
                        cv2.drawContours(frame_rgb, [c], -1, color_bgr, 2)
                cv2.rectangle(frame_rgb, (0, roi_top), (band_width, roi_bottom), (255, 255, 0), 2)
                cv2.rectangle(frame_rgb, (width - band_width, roi_top), (width, roi_bottom), (255, 255, 0), 2)

        return contours

    def find_dominant_obstacle(self, contours):
        dominant_color = None
        max_area = 0
        dominant_contour = None

        for color, positions in contours.items():
            for side in ['left', 'right']:
                contour = positions.get(side)
                if contour is not None:
                    area = cv2.contourArea(contour)
                    if area > max_area:
                        max_area = area
                        dominant_color = color
                        dominant_contour = contour

        return dominant_color, dominant_contour

    def detect_turn(self, frame_rgb, roi_top_ratio=0.8, roi_height_ratio=0.2, roi_width_ratio=0.25):
        frame_lab = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2LAB)
        height, width, _ = frame_rgb.shape
        roi_top = int(height * roi_top_ratio)
        roi_height = int(height * roi_height_ratio)
        roi_bottom = min(roi_top + roi_height, height)
        roi_width = int(width * roi_width_ratio)
        roi_left = int((width - roi_width) / 2)
        roi_right = roi_left + roi_width
        roi = frame_lab[roi_top:roi_bottom, roi_left:roi_right]

        if self.debug_mode:
            cv2.rectangle(frame_rgb, (roi_left, roi_top), (roi_right, roi_bottom), (255, 255, 0), 2)

        for color in ['orange', 'blue']:
            mask = self.detect_color(roi, color)
            pixel_ratio = np.sum(mask > 0) / mask.size
            if pixel_ratio > 0.1:
                return 'LEFT' if color == 'blue' else 'RIGHT'
        return None

    def handle_turn_detection(self, frame_rgb):
        current_time = time.time()
        if (current_time - self.last_turn_time) < self.turn_cooldown and not self.debug_mode:
            return
        direction = self.detect_turn(frame_rgb)
        if direction:
            angle = -90 if direction == 'LEFT' else 90
            if not self.debug_mode:
                self.send_command('TURN', angle)
            print(f"TURN DETECTED: {direction}, sent TURN command.")
            self.last_turn_time = current_time

    def process_frame(self):
        frame_rgb = self.capture_frame()
        contours = self.detect_obstacles(frame_rgb)
        color, contour = self.find_dominant_obstacle(contours)
        if contour is not None:
            self.process_obstacle(contour, frame_rgb, color)
        self.handle_turn_detection(frame_rgb)
        return frame_rgb

    def run_processing_loop(self):
        print("Starting obstacle detection in PRODUCTION mode...")
        try:
            while True:
                self.process_frame()
                time.sleep(0.08)
        except KeyboardInterrupt:
            print("\nStopping detection...")
        finally:
            if self.picam2:
                self.picam2.stop()
            if self.arduino:
                self.arduino.close()


# === Web interface for debugging ===
def create_flask_app(detector):
    from flask import Flask, Response

    app = Flask(__name__)

    def generate_frames():
        while True:
            frame = detector.process_frame()
            success, encoded_frame = cv2.imencode(".jpg", frame)
            if success:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' +
                       bytearray(encoded_frame) + b'\r\n')
            time.sleep(0.1)

    @app.route('/video_feed')
    def video_feed():
        return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

    @app.route('/')
    def index():
        width, height = (4608, 2592)
        if detector.picam2 and 'PixelArraySize' in detector.picam2.camera_properties:
            width, height = detector.picam2.camera_properties['PixelArraySize']
        display_width = 1280
        display_height = int(display_width * (height / width))
        return f"""
        <html>
        <head><title>WRO Obstacle Detection</title></head>
        <body><h2>Live Feed</h2><img src="/video_feed" width="{display_width}" height="{display_height}"></body>
        </html>
        """

    return app


# === Main Execution ===
if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Obstacle Detection System')
    parser.add_argument('--web', action='store_true', help='Enable web interface')
    parser.add_argument('--serial', type=str, default='/dev/ttyUSB0', help='Serial port path')
    args = parser.parse_args()

    detector = ObstacleDetector(debug_mode=args.web, serial_port=args.serial)

    if args.web:
        print("Starting web server in DEBUG MODE...")
        app = create_flask_app(detector)
        app.run(host='0.0.0.0', port=8080, threaded=True)
    else:
        print("Running in PRODUCTION MODE...")
        detector.run_processing_loop()
