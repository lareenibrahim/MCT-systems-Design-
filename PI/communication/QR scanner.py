import cv2
from pyzbar.pyzbar import decode
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
import socket as _s

# ─────────────────────────────────────────────────────────────
# SHARED COLOR STATE
# Written by qr_scanner, read by the controller via this module.
# ─────────────────────────────────────────────────────────────
detected_color = None          # "Red" | "Green" | "Blue" | None
color_lock     = threading.Lock()
color_event    = threading.Event()   # set whenever a new color arrives

# Color → (R, G, B) for the DS4 light bar (0-255)
COLOR_MAP = {
    "red":   (255, 0,   0),
    "green": (0,   255, 0),
    "blue":  (0,   0,   255),
}

def _set_color(name: str):
    """Update shared color state and fire the event."""
    global detected_color
    normalised = name.strip().lower()
    if normalised not in COLOR_MAP:
        print(f"[CAMERA] QR color '{name}' not recognised — ignoring")
        return
    with color_lock:
        detected_color = normalised
    color_event.set()
    print(f"[CAMERA] Color set → {normalised}")


def main():

    # ─────────────────────────────────────────────────────────────
    # CAMERA
    # ─────────────────────────────────────────────────────────────
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    if not cap.isOpened():
        print("[CAMERA] ERROR: Could not open camera")
        return

    print("[CAMERA] Camera opened successfully")

    # ─────────────────────────────────────────────────────────────
    # SHARED FRAME
    # ─────────────────────────────────────────────────────────────
    latest_frame = None
    frame_lock   = threading.Lock()

    # ─────────────────────────────────────────────────────────────
    # MJPEG STREAM SERVER
    # ─────────────────────────────────────────────────────────────
    class StreamHandler(BaseHTTPRequestHandler):

        def log_message(self, format, *args):
            pass

        def do_GET(self):
            if self.path == '/stream':
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                self.end_headers()
                try:
                    while True:
                        with frame_lock:
                            frame = latest_frame
                        if frame is None:
                            continue
                        _, jpeg = cv2.imencode('.jpg', frame)
                        data = jpeg.tobytes()
                        self.wfile.write(b'--frame\r\n')
                        self.wfile.write(b'Content-Type: image/jpeg\r\n\r\n')
                        self.wfile.write(data)
                        self.wfile.write(b'\r\n')
                except Exception:
                    pass
            else:
                self.send_response(200)
                self.send_header('Content-Type', 'text/html')
                self.end_headers()
                html = (b'<html><body style="margin:0;background:#000">'
                        b'<img src="/stream" style="width:100%;height:100vh;object-fit:contain">'
                        b'</body></html>')
                self.wfile.write(html)

    def start_stream_server():
        server = HTTPServer(('0.0.0.0', 8080), StreamHandler)
        server.serve_forever()

    stream_thread = threading.Thread(target=start_stream_server, daemon=True)
    stream_thread.start()

    hostname = _s.gethostname()
    try:
        pi_ip = _s.gethostbyname(hostname)
    except Exception:
        pi_ip = "unknown"
    print(f"[CAMERA] Live stream at: http://{pi_ip}:8080")

    # ─────────────────────────────────────────────────────────────
    # MAIN LOOP
    # ─────────────────────────────────────────────────────────────
    last_reported = None

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                continue

            with frame_lock:
                latest_frame = frame.copy()

            decoded_objects = decode(frame)
            for obj in decoded_objects:
                data = obj.data.decode('utf-8').strip()
                print(f"[CAMERA] QR detected: {data}")
                # Only trigger if the color has actually changed
                if data.lower() != last_reported:
                    _set_color(data)
                    last_reported = data.lower()

    except Exception as e:
        print(f"[CAMERA] Loop error: {e}")

    finally:
        print("[CAMERA] Shutting down...")
        cap.release()


if __name__ == "__main__":
    main()