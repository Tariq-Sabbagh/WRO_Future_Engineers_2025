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
        self.MAX_AREA = 3000.0
        self.debug_mode = debug_mode  # True = debugging, False = production
        self.last_turn_time = 0  # timestamp of the last detected turn
        self.turn_cooldown = 2   # seconds to wait before detecting a new turn
        self.movedPoint = 5

        # Color profiles
        self.COLOR_PROFILES = {
            'red': {
                'lower': np.array([0, 126, 0]),
                'upper': np.array([255, 255, 114]),
                'offset_adjust': 20
            },
            'green': {
                'lower': np.array([0, 0, 0]),
                'upper': np.array([255, 108, 255]),
                'offset_adjust': -25
            },
            'orange': {
                'lower': np.array([0, 0, 0]),
                'upper': np.array([255, 136, 106]),
                'offset_adjust': 90
            },
            'blue': {
                'lower': np.array([88, 132, 130]),
                'upper': np.array([255, 255, 255]),
                'offset_adjust': -90
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
        self.picam2.set_controls({"ExposureTime": 9000})
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
        """Find largest valid contour in mask"""
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        valid_contours = []

        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            contour_area = cv2.contourArea(contour)
            # Reject if width > height
            if w > h:
                continue

            # Reject if area (either rectangle or contour) exceeds MAX_AREA
            if contour_area <= self.MAX_AREA:
                continue

            valid_contours.append(contour)

        if not valid_contours:
            return None

        largest_contour = max(valid_contours, key=cv2.contourArea)
        return largest_contour if cv2.contourArea(largest_contour) > min_area else None


    def calculate_distance(self, pixel_height):
        """Calculate distance to object"""
        return (self.KNOWN_OBSTACLE_HEIGHT_CM * self.FOCAL_LENGTH) / pixel_height if pixel_height else 0

    def calculate_maneuver(self, frame_shape, bounding_rect, distance_cm, offset_adjust):
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
        distance = self.calculate_distance(h) - self.movedPoint
        travel_dist, turn_angle = self.calculate_maneuver(
            frame_rgb.shape, (x, y, w, h), distance, profile['offset_adjust']
        )

        

        # Annotate frame for visualization
        info_text = f"{color_type.upper()}: {distance:.1f}cm, {turn_angle:.1f}deg"
        cv2.rectangle(frame_rgb, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(frame_rgb, info_text, (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # Only send commands in production mode
        if not self.debug_mode:
            self.send_command('AVOID', travel_dist, turn_angle)

        print(
            f"{color_type.upper()}: Dist={travel_dist:.1f}cm, Angle={turn_angle:.1f}deg")
        return True

    def send_command(self, cmd_type, value1=0, value2=0):
        """
        Send a structured command to the Arduino.
        cmd_type: str ('AVOID' or 'TURN')
        value1, value2: float values (interpreted as tenths of units)
        """
        if self.arduino is None:
            return

        try:
            if cmd_type == 'AVOID':
                # Send both distance and angle
                header = b'A'
                val1 = int(round(value1 * 10))
                val2 = int(round(value2 * 10))
            elif cmd_type == 'TURN':
                # Send angle and dummy value (0)
                header = b'T'
                val1 = int(round(value1 * 10))
                val2 = 0
            else:
                print(f"Unknown command type: {cmd_type}")
                return

            packet = struct.pack('<chh', header, val1, val2)  # 5 bytes
            self.arduino.write(packet)
            self.arduino.flush()

        except (serial.SerialException, OSError) as e:
            print(f"ERROR: Failed to send command: {e}")
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
        for color in ['red', 'green']:
            mask = self.detect_color(frame_lab, color)
            contours[color] = self.find_largest_contour(mask)

        return contours

    def crop_frame(self, frame, x_ratio=0.25, y_ratio=0.25, width_ratio=0.5, height_ratio=0.5):
        """
        Crop the frame to a region of interest (ROI) using relative coordinates.

        Args:
            frame_rgb (np.ndarray): The input RGB frame.
            x_ratio (float): Horizontal start position as a fraction of frame width.
            y_ratio (float): Vertical start position as a fraction of frame height.
            width_ratio (float): Width of ROI as a fraction of frame width.
            height_ratio (float): Height of ROI as a fraction of frame height.

        Returns:
            np.ndarray: Cropped frame (ROI).
        """
        height, width, _ = frame.shape

        roi_left = int(width * x_ratio)
        roi_top = int(height * y_ratio)
        roi_width = int(width * width_ratio)
        roi_height = int(height * height_ratio)

        roi_right = min(roi_left + roi_width, width)
        roi_bottom = min(roi_top + roi_height, height)

        # Debug visualization
        if self.debug_mode:
            cv2.rectangle(frame, (roi_left, roi_top),
                          (roi_right, roi_bottom), (255, 255, 0), 2)

        return frame[roi_top:roi_bottom, roi_left:roi_right]

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

    def detect_turn(self, frame_rgb):
        """
        Detect turn direction based on the percentage of orange or blue in a vertical ROI.
        This avoids contours and simply measures pixel presence.
        """
        roi = self.crop_frame(frame_rgb, 0.375, 0.8, 0.25, 0.15)
        frame_lab = cv2.cvtColor(roi, cv2.COLOR_RGB2LAB)

        turn_map = {'blue': 'LEFT', 'orange': 'RIGHT'}
        debug_positions = {'blue': 30, 'orange': 60}

        for color in turn_map:
            mask = self.detect_color(frame_lab, color)
            pixel_ratio = np.sum(mask > 0) / mask.size

            if self.debug_mode:
                cv2.putText(frame_rgb, f"{color.upper()} RATIO: {pixel_ratio:.2f}",
                            (10, debug_positions[color]
                             ), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (255, 255, 255), 2)

            if pixel_ratio > 0.1:
                turn_detected = turn_map[color]
                print(f"TURN DETECTED: {turn_detected}")

                if self.debug_mode:
                    cv2.putText(frame_rgb, f"TURN DETECTED: {turn_detected}",
                                (10, frame_rgb.shape[0] -
                                 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                                (0, 255, 255), 2)

                return turn_detected

        return None

    def handle_turn_detection(self, frame_rgb):
        """Detect turn based on stripes and apply cooldown"""
        current_time = time.time()
        if (current_time - self.last_turn_time) < self.turn_cooldown and not self.debug_mode:
            return  # Still cooling down
        direction = self.detect_turn(frame_rgb)
        if direction:
            angle = -90 if direction == 'LEFT' else 90
            if not self.debug_mode:
                self.send_command('TURN', angle)
            self.last_turn_time = current_time

    def process_frame(self):
        """Process a single frame"""
        frame_rgb = self.capture_frame()
        obstacle_roi_rgb = self.crop_frame(frame_rgb, 0.05, 0.25, 0.9, 0.6)
        contours = self.detect_obstacles(obstacle_roi_rgb)
        dominant_color = self.find_dominant_obstacle(contours)

        if dominant_color:
            self.process_obstacle(
                contours[dominant_color],
                obstacle_roi_rgb,
                dominant_color
            )

        self.handle_turn_detection(frame_rgb)

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
    parser.add_argument('--serial', type=str, default='/dev/ttyUSB0',
                        help='Specify serial port (default: /dev/ttyUSB0)')
    args = parser.parse_args()

    # Create detector with serial port override
    detector = ObstacleDetector(debug_mode=args.web, serial_port=args.serial)

    if args.web:
        print("Starting web server in DEBUG MODE...")
        app = create_flask_app(detector)
        app.run(host='0.0.0.0', port=8080, threaded=True)
    else:
        print("Running in PRODUCTION MODE: Commands will be sent to Arduino")
        detector.run_processing_loop()
