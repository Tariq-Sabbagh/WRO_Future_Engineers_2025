import cv2
import numpy as np
import os
from datetime import datetime
from picamera2 import Picamera2

# === Checkerboard parameters ===
CHECKERBOARD = (7, 9)  # (rows, columns) of inner corners
SQUARE_SIZE = 25  # mm (real size of one square)

SAVE_DIR = "calibration_images"
os.makedirs(SAVE_DIR, exist_ok=True)

# === Start Camera ===
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (4608, 2592)})
picam2.configure(config)
picam2.start()

print("📷 Press 'T' to take photo for calibration.")
print("❌ Press 'Q' to quit and calibrate the camera.")

# === Take photos loop ===
image_count = 0

# while True:
#     frame = picam2.capture_array()
#     frame = cv2.flip(frame, -1)
#     preview = frame.copy()

#     # Display count overlay
#     cv2.putText(preview, f"Photos: {image_count}", (10, 30), 
#                 cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

#     # cv2.imshow("Calibration Preview", preview)
#     print("Type 't' then Enter to take photo, 'q' to quit:")
#     key = input().strip().lower()

#     if key == 't':
#         filename = os.path.join(SAVE_DIR, f"calib_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg")
#         cv2.imwrite(filename, frame)
#         print(f"[✔] Saved: {filename}")
#         image_count += 1

#     elif key == 'q':
#         print("📦 Quitting capture mode. Starting calibration...")
#         break

# cv2.destroyAllWindows()

# === Start Calibration ===
objpoints = []  # 3D points
imgpoints = []  # 2D points

objp = np.zeros((CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[1], 0:CHECKERBOARD[0]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

images = [os.path.join(SAVE_DIR, f) for f in os.listdir(SAVE_DIR) if f.endswith(".jpg")]
print(f"🖼 Found {len(images)} images. Calibrating...")

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    if ret:
        corners_sub = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1),
                                       (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        imgpoints.append(corners_sub)
        objpoints.append(objp)
        print(f"[✓] Checkerboard detected: {fname}")
    else:
        print(f"[x] Checkerboard not found: {fname}")

# Perform calibration
if len(objpoints) >= 5:
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    np.savez("calibration_data.npz", mtx=mtx, dist=dist)
    print("\n✅ Calibration successful!")
    print("Camera matrix:\n", mtx)
    print("Distortion coefficients:\n", dist)
    print("Saved as 'calibration_data.npz'")
else:
    print("❌ Not enough valid images for calibration. Need at least 5.")

