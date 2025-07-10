import serial
import time
import cv2
import numpy as np
import struct
from picamera2 import Picamera2


class ObstacleDetector:
    def __init__(self, debug_mode=False):
        # Configuration
        self.SERIAL_PORT = '/dev/ttyUSB0'
        self.BAUDRATE = 115200
        self.CALIBRATION_FILE = "calibration_data.npz"
        self.OPTIMIZED_RESOLUTION = (1280, 720)
        self.KNOWN_OBSTACLE_HEIGHT_CM = 10.0
        self.FOCAL_LENGTH = 570.0
        self.MAX_DISTANCE_CM = 90.0
        self.debug_mode = debug_mode  # True = debugging, False = production

        # Color profiles
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
            'blue': {
                'lower': np.array([20, 110, 130]),
                'upper': np.array([255, 145, 180]),
                'offset_adjust': 0
            },
            'orange': {
                'lower': np.array([100, 140, 150]),
                'upper': np.array([190, 180, 200]),
                'offset_adjust': 0
            }
        }

        # State variables
        self.picam2 = None
        self.arduino = None

        # Initialize hardware and calibration
        self.initialize_camera()
        self.load_calibration()

        # Only initialize serial in production mode
        if not self.debug_mode:
            self.initialize_serial()

    def load_calibration(self):
        """Load and scale camera calibration data"""
        calibration_data = np.load(self.CALIBRATION_FILE)
        mtx = calibration_data['mtx']
        dist = calibration_data['dist']

        # Scale camera matrix
        SCALE_X = self.OPTIMIZED_RESOLUTION[0] / 4608
        SCALE_Y = self.OPTIMIZED_RESOLUTION[1] / 2592
        self.mtx_scaled = mtx.copy()
        self.mtx_scaled[0, 0] *= SCALE_X
        self.mtx_scaled[1, 1] *= SCALE_Y
        self.mtx_scaled[0, 2] *= SCALE_X
        self.mtx_scaled[1, 2] *= SCALE_Y

        # Precompute undistortion maps
        self.map1, self.map2 = cv2.initUndistortRectifyMap(
            self.mtx_scaled, dist, None, self.mtx_scaled,
            self.OPTIMIZED_RESOLUTION,
            cv2.CV_16SC2
        )

    def initialize_camera(self):
        """Set up the camera hardware"""
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(
            main={"size": self.OPTIMIZED_RESOLUTION},
            raw={"size": (2304, 1296)}
        )
        self.picam2.configure(config)
        self.picam2.start()
        print("Camera initialized.")

    def initialize_serial(self):
        """Set up serial communication with Arduino (production only)"""
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
        """Detect specific color in LAB space"""
        profile = self.COLOR_PROFILES[color_name]
        mask = cv2.inRange(lab_frame, profile['lower'], profile['upper'])
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        return mask

    def find_largest_contour(self, mask, min_area=300):
        """Find largest contour in mask"""
        contours, _ = cv2.findContours(
            mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        largest_contour = max(contours, key=cv2.contourArea)
        return largest_contour if cv2.contourArea(largest_contour) > min_area else None

    def calculate_distance(self, pixel_height):
        """Calculate distance to object"""
        return (self.KNOWN_OBSTACLE_HEIGHT_CM * self.FOCAL_LENGTH) / pixel_height if pixel_height else 0

    def calculate_maneuver(self, frame_shape, bounding_rect, distance_cm, offset_adjust=15):
        """Calculate movement parameters"""
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
        """Process detected obstacle and optionally send commands"""
        profile = self.COLOR_PROFILES[color_type]
        x, y, w, h = cv2.boundingRect(contour)
        distance = self.calculate_distance(h)
        travel_dist, turn_angle = self.calculate_maneuver(
            frame_rgb.shape, (x, y, w, h), distance, profile['offset_adjust']
        )

        # Skip if too far
        if travel_dist >= self.MAX_DISTANCE_CM:
            return False

        # Annotate frame for visualization
        info_text = f"{color_type.upper()}: {travel_dist:.1f}cm, {turn_angle:.1f}deg"
        cv2.rectangle(frame_rgb, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(frame_rgb, info_text, (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # Only send commands in production mode
        if not self.debug_mode:
            self.send_values(travel_dist, turn_angle)

        print(
            f"{color_type.upper()}: Dist={travel_dist:.1f}cm, Angle={turn_angle:.1f}deg")
        return True

    def send_values(self, travel_dist, turn_angle):
        """Send maneuver values to Arduino (production only)"""
        if self.arduino is None:
            return

        # Encode values
        distance_int = int(round(travel_dist * 10))
        angle_int = int(round(turn_angle * 10))
        packet = struct.pack('<hh', distance_int, angle_int)

        try:
            self.arduino.write(packet)
        except (serial.SerialException, OSError) as e:
            print(f"ERROR: Write failed. {e}")
            if self.arduino:
                self.arduino.close()
            self.arduino = None

    def capture_frame(self):
        """Capture and prepare a frame from camera"""
        frame = self.picam2.capture_array()
        frame = cv2.flip(frame, -1)
        return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    def detect_obstacles(self, frame_rgb):
        """Detect obstacles in a frame"""
        frame_lab = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2LAB)

        # Detect both colors
        contours = {}
        for color in self.COLOR_PROFILES:
            mask = self.detect_color(frame_lab, color)
            contours[color] = self.find_largest_contour(mask)

        return contours

    def find_dominant_obstacle(self, contours):
        """Determine which obstacle is most prominent"""
        dominant_color = None
        max_area = 0

        for color, contour in contours.items():
            if contour is not None:
                area = cv2.contourArea(contour)
                if area > max_area:
                    max_area = area
                    dominant_color = color

        return dominant_color

    def process_frame(self):
        """Process a single frame"""
        frame_rgb = self.capture_frame()
        contours = self.detect_obstacles(frame_rgb)
        dominant_color = self.find_dominant_obstacle(contours)

        if dominant_color:
            self.process_obstacle(
                contours[dominant_color],
                frame_rgb,
                dominant_color
            )

        return frame_rgb

    def run_processing_loop(self):
        """Main processing loop for production"""
        print("Starting obstacle detection in PRODUCTION mode...")
        try:
            while True:
                # Process frame without visualization
                self.process_frame()
                time.sleep(0.08)
        except KeyboardInterrupt:
            print("\nStopping detection...")
        finally:
            if self.picam2:
                self.picam2.stop()
            if self.arduino:
                self.arduino.close()


# =================================================================
# Web Application (Debugging Only)
# =================================================================
def create_flask_app(detector):
    from flask import Flask, Response

    app = Flask(__name__)

    def generate_frames():
        """Video streaming generator for debugging"""
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
        return Response(generate_frames(),
                        mimetype='multipart/x-mixed-replace; boundary=frame')

    @app.route('/')
    def index():
        # Determine display size
        width, height = (4608, 2592)
        if detector.picam2 and 'PixelArraySize' in detector.picam2.camera_properties:
            width, height = detector.picam2.camera_properties['PixelArraySize']

        display_width = 1280
        display_height = int(display_width * (height/width))

        return f"""
        <html>
        <head>
            <title>WRO Obstacle Detection Stream</title>
            <style>
                body {{
                    font-family: sans-serif;
                    text-align: center;
                    background-color: #282c34;
                    color: white;
                }}
                img {{
                    border: 2px solid #61dafb;
                    margin-top: 20px;
                }}
            </style>
        </head>
        <body>
            <h2>WRO Obstacle Detection (DEBUG MODE)</h2>
            <img src="/video_feed" width="{display_width}" height="{display_height}">
        </body>
        </html>
        """

    return app


# =================================================================
# Main Execution
# =================================================================
if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Obstacle Detection System')
    parser.add_argument('--web', action='store_true',
                        help='Enable web interface (debugging only)')
    args = parser.parse_args()

    # Create detector in appropriate mode
    detector = ObstacleDetector(debug_mode=args.web)

    if args.web:
        print("Starting web server in DEBUG MODE...")
        app = create_flask_app(detector)
        app.run(host='0.0.0.0', port=8080, threaded=True)
    else:
        print("Running in PRODUCTION MODE: Commands will be sent to Arduino")
        detector.run_processing_loop()
