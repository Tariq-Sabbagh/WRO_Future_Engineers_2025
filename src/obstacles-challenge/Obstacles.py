import serial
import time
import cv2
import numpy as np
import struct
from picamera2 import Picamera2

class ObstacleDetector:
    def __init__(self, debug_mode=False):
        # Configuration
        self.center_roi_mask = None  # Will be created after camera init
        self.CENTER_ROI_RATIO = 0.3  # 30% of the frame size
        
        # Initialize hardware and calibration
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
        
        
        # State variables
        self.picam2 = None
        self.arduino = None
        
        # Initialize hardware and calibration
        self.initialize_camera()
        self.load_calibration()
        self.create_center_roi_mask()
        
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
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if not contours: 
            return None
            
        largest_contour = max(contours, key=cv2.contourArea)
        return largest_contour if cv2.contourArea(largest_contour) > min_area else None
    
    def calculate_distance(self, pixel_height ,test = 1):
        """Calculate distance to object"""
        return (test * self.FOCAL_LENGTH) / pixel_height if pixel_height else 0
    
    def calculate_distance_sinside(self, frame_shape, bounding_rect, distance_cm, offset_adjust=15):
        x, y, w, h = bounding_rect
        obj_center_x = x + w / 2
        frame_center_x = frame_shape[1] / 2
        
        pixel_offset = obj_center_x - frame_center_x
        cm_per_pixel = self.KNOWN_OBSTACLE_HEIGHT_CM / h
        offset_x_cm = (pixel_offset * cm_per_pixel) + offset_adjust
        return offset_x_cm
        
    def calculate_maneuver(self, frame_shape, bounding_rect, distance_cm, offset_adjust=15):
        """Calculate movement parameters"""
        offset_x_cm =self.calculate_distance_sinside(frame_shape, bounding_rect, distance_cm, offset_adjust=15)
        
        tendon = np.sqrt(offset_x_cm**2 + distance_cm**2)
        angle = np.arctan2(offset_x_cm, distance_cm)
        turn_angle = np.degrees(angle)
        
        return tendon, turn_angle
    
    def process_obstacle(self, contour, frame_rgb, color_type):
        """Process detected obstacle and optionally send commands"""
        profile = self.COLOR_PROFILES[color_type]
        # print(profile)
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
            self.send_values(travel_dist, turn_angle, 0)
        
        # print(f"{color_type.upper()}: Dist={travel_dist:.1f}cm, Angle={turn_angle:.1f}deg")
        return True
    
   
    
    
    def process_obstacle_with_turn(self, contour, frame_rgb, cube_color_type , turn_color_type , turn_contour):
        """Process detected obstacle and optionally send commands"""
        profile = self.COLOR_PROFILES[cube_color_type]
        profile_turn = self.COLOR_PROFILES[turn_color_type]
        # print(profile['offset_adjust'])
        # print(profile)
        x, y, w, h = cv2.boundingRect(contour)
        x_turn, y_turn, w_turn, h_turn = cv2.boundingRect(turn_contour)
        distance = self.calculate_distance(h ,self.KNOWN_OBSTACLE_HEIGHT_CM)
        distance_turn = self.calculate_distance(h_turn)
        info_text = f"{cube_color_type.upper()}: {distance:.1f}cm"
        cv2.rectangle(frame_rgb, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(frame_rgb, info_text, (x, y - 10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        info_text = f"{turn_color_type.upper()}: {distance_turn:.1f}cm"
        
        cv2.rectangle(frame_rgb, (x_turn, y_turn), (x_turn + w_turn, y_turn + h_turn), (0, 255, 0), 2)
        cv2.putText(frame_rgb, info_text, (x_turn, y_turn - 10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
        if distance_turn < distance:
            # print(f"{cube_color_type.upper()}: Dist={travel_dist:.1f}cm, Angle={turn_angle:.1f}deg")
            self.send_values(distance + profile['offset_adjust'], 90 ,self.calculate_distance_sinside(
                frame_rgb.shape, (x, y, w, h), distance, profile['offset_adjust']
            ) )
            return False
        
        travel_dist, turn_angle = self.calculate_maneuver(
            frame_rgb.shape, (x, y, w, h), distance, profile['offset_adjust']
        )
        
        # Skip if too far
        if travel_dist >= self.MAX_DISTANCE_CM:
            return False
        
        # Annotate frame for visualization
        
        # Only send commands in production mode
        if not self.debug_mode:
            self.send_values(travel_dist, turn_angle)
        
        # print(f"{cube_color_type.upper()}: Dist={travel_dist:.1f}cm, Angle={turn_angle:.1f}deg")
        return True
    
    def send_values(self, travel_dist, turn_angle,distance_turn):
        """Send maneuver values to Arduino (production only)"""
        # print(travel_dist ,turn_angle)
        if self.arduino is None:
            return
            
        # Encode values
        distance_int = int(round(travel_dist * 10))
        angle_int = int(round(turn_angle * 10))
        distance_turn_int = int(round(distance_turn * 10))
        packet = struct.pack('<hh', distance_int, angle_int ,distance_turn_int)
        try:
            self.arduino.write(packet)
        except (serial.SerialException, OSError) as e:
            print(f"ERROR: Write failed. {e}")
            if self.arduino:
                self.arduino.close()
            self.arduino = None
            
    def create_center_roi_mask(self):
        """Create a mask for the center region of the frame"""
        h, w = self.OPTIMIZED_RESOLUTION[1], self.OPTIMIZED_RESOLUTION[0]
        roi_width = int(w * self.CENTER_ROI_RATIO)
        roi_height = int(h * self.CENTER_ROI_RATIO)
        
        # Calculate center region coordinates
        x1 = (w - roi_width) // 2
        y1 = (h - roi_height) // 2
        x2 = x1 + roi_width
        y2 = y1 + roi_height
        
        # Create a black mask (all zeros)
        self.center_roi_mask = np.zeros((h, w), dtype=np.uint8)
        # Set center region to white (255)
        cv2.rectangle(self.center_roi_mask, (x1, y1), (x2, y2), 255, -1)
        
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
            if color == 'orange' or color == 'blue':
                mask = cv2.bitwise_and(mask, self.center_roi_mask)
            
            contours[color] = self.find_largest_contour(mask)
            
        
        return contours
    
    def find_dominant_obstacle(self, contours):
        """Determine which obstacle is most prominent"""
        dominant_color = None
        max_area = 0
        
        for color, contour in contours.items():
            if contour is not None and (color is not "blue" and color is not "orange"):
                area = cv2.contourArea(contour)
                if area > max_area:
                    max_area = area
                    dominant_color = color
        return dominant_color
    
    def find_dominant_obstacle_turn(self, contours):
        """Determine which obstacle is most prominent"""
        dominant_color = None
        max_area = 0
        
        for color, contour in contours.items():
            if contour is not None and (color is not "red" and color is not "green"):
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
        turn_color = self.find_dominant_obstacle_turn(contours)
        
        # print(dominant_color +" " + turn_color)
        if dominant_color and turn_color is None:
            self.process_obstacle(
                contours[dominant_color], 
                frame_rgb, 
                dominant_color
            )
        
        elif dominant_color and turn_color:
            self.process_obstacle_with_turn(contours[dominant_color],frame_rgb ,dominant_color ,turn_color , contours[turn_color])
        
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
                .warning {{
                    color: #ff6b6b;
                    font-weight: bold;
                    margin: 20px;
                    padding: 10px;
                    border: 1px solid #ff6b6b;
                    border-radius: 5px;
                }}
            </style>
        </head>
        <body>
            <h2>WRO Obstacle Detection (DEBUG MODE)</h2>
            <div class="warning">⚠️ DEBUG MODE: No commands being sent to Arduino ⚠️</div>
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