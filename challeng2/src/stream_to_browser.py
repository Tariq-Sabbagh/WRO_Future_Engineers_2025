# stream_to_browser.py
# ------------------------------------------------------------
# pip install flask opencv-python   (opencv‑python‑headless OK)
# ------------------------------------------------------------
import cv2, threading, atexit
from flask import Flask, Response

class WebFrameDisplay:
    """
    Simple MJPEG streamer.
    Call update_frame(np_array) whenever you have a new frame.
    Browse at http://<host>:<port>/video
    """
    def __init__(self, host="0.0.0.0", port=8080, jpeg_quality=80):
        self._latest_jpeg = None
        self._quality     = jpeg_quality
        self._app         = Flask(__name__)

        # Route that serves the MJPEG
        self._app.add_url_rule("/video", view_func=self._video_feed)
        # Optional tiny index page
        self._app.add_url_rule("/", view_func=
            lambda: '<h2>Live Stream</h2><img src="/video" width=640>')

        # Run Flask in a background thread
        self._thread = threading.Thread(
            target=self._app.run,
            kwargs=dict(host=host, port=port,
                        threaded=True, debug=False, use_reloader=False),
            daemon=True
        )
        atexit.register(self.stop)

    # public --------------------------------------------------
    def start(self):
        if not self._thread.is_alive():
            self._thread.start()

    def stop(self):
        pass  # Flask exits when main program ends

    def update_frame(self, frame_bgr_or_rgb):
        ok, jpeg = cv2.imencode(".jpg", frame_bgr_or_rgb,
                                [cv2.IMWRITE_JPEG_QUALITY, self._quality])
        if ok:
            self._latest_jpeg = jpeg.tobytes()

    # private -------------------------------------------------
    def _video_feed(self):
        return Response(self._generator(),
                        mimetype="multipart/x-mixed-replace; boundary=frame")

    def _generator(self):
        boundary = b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
        while True:
            if self._latest_jpeg:
                yield boundary + self._latest_jpeg + b"\r\n"

# ------------------------------------------------------------------
# EXAMPLE USAGE: stream frames from PiCamera2 (or any VideoCapture)
# ------------------------------------------------------------------
if __name__ == "__main__":
    from picamera2 import Picamera2                      # Pi camera
    import time

    display = WebFrameDisplay(port=8080)
    display.start()
    print("Open browser:  http://<pi‑ip>:8080/video")

    cam = Picamera2()
    cam.configure(cam.create_preview_configuration(main={"size": (640, 480),
                                                         "format": "RGB888"}))
    cam.start();  time.sleep(2)

    try:
        while True:
            frame = cam.capture_array()                  # numpy RGB
            display.update_frame(frame)                  # push to browser
    except KeyboardInterrupt:
        pass
