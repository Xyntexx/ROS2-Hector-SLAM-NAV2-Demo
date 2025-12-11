#!/usr/bin/env python3
"""
Simple MJPEG HTTP streaming server for webcam.
View in browser at http://<ip>:8081
"""

import subprocess
import sys
import argparse
from http.server import HTTPServer, BaseHTTPRequestHandler
import threading

# Defaults (can be overridden via command line)
PORT = 8081
DEVICE = "/dev/video0"
WIDTH = 1280
HEIGHT = 720
FPS = 5


class MJPEGHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/" or self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
            self.send_header("Pragma", "no-cache")
            self.send_header("Expires", "0")
            self.end_headers()

            # Use ffmpeg to capture MJPEG frames
            cmd = [
                "ffmpeg",
                "-f", "v4l2",
                "-input_format", "mjpeg",
                "-video_size", f"{WIDTH}x{HEIGHT}",
                "-framerate", str(FPS),
                "-i", DEVICE,
                "-c:v", "mjpeg",
                "-q:v", "5",
                "-f", "image2pipe",
                "-vcodec", "mjpeg",
                "-"
            ]

            try:
                proc = subprocess.Popen(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.DEVNULL,
                    bufsize=10**6
                )

                # Read JPEG frames from ffmpeg
                buffer = b""
                while True:
                    chunk = proc.stdout.read(4096)
                    if not chunk:
                        break
                    buffer += chunk

                    # Find JPEG boundaries (FFD8 = start, FFD9 = end)
                    start = buffer.find(b'\xff\xd8')
                    end = buffer.find(b'\xff\xd9')

                    if start != -1 and end != -1 and end > start:
                        jpg = buffer[start:end+2]
                        buffer = buffer[end+2:]

                        try:
                            self.wfile.write(b"--frame\r\n")
                            self.wfile.write(b"Content-Type: image/jpeg\r\n")
                            self.wfile.write(f"Content-Length: {len(jpg)}\r\n\r\n".encode())
                            self.wfile.write(jpg)
                            self.wfile.write(b"\r\n")
                        except (BrokenPipeError, ConnectionResetError):
                            break

            except Exception as e:
                print(f"Stream error: {e}")
            finally:
                if proc:
                    proc.terminate()

        elif self.path == "/snapshot":
            # Single JPEG snapshot
            cmd = [
                "ffmpeg",
                "-f", "v4l2",
                "-input_format", "mjpeg",
                "-video_size", f"{WIDTH}x{HEIGHT}",
                "-i", DEVICE,
                "-frames:v", "1",
                "-f", "image2",
                "-"
            ]
            try:
                result = subprocess.run(cmd, capture_output=True, timeout=5)
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(result.stdout)))
                self.end_headers()
                self.wfile.write(result.stdout)
            except Exception as e:
                self.send_error(500, str(e))

        else:
            # HTML page with auto-reconnect
            html = """<!DOCTYPE html>
<html>
<head>
<title>Webcam Stream</title>
<style>
body { margin:0; background:#000; display:flex; justify-content:center; align-items:center; height:100vh; flex-direction:column; }
img { max-width:100%; max-height:90vh; }
#status { color:#fff; font-family:sans-serif; margin:10px; }
</style>
</head>
<body>
<div id="status">Connecting...</div>
<img id="stream">
<script>
const img = document.getElementById('stream');
const status = document.getElementById('status');
let reconnectTimer = null;

function connect() {
    status.textContent = 'Connecting...';
    img.src = '/stream?' + Date.now();
}

img.onload = function() {
    status.textContent = 'Connected';
    if (reconnectTimer) {
        clearTimeout(reconnectTimer);
        reconnectTimer = null;
    }
};

img.onerror = function() {
    status.textContent = 'Disconnected - reconnecting in 2s...';
    reconnectTimer = setTimeout(connect, 2000);
};

// Also handle stalled stream (no new frames)
setInterval(function() {
    if (!img.complete || img.naturalHeight === 0) {
        status.textContent = 'Stream stalled - reconnecting...';
        connect();
    }
}, 5000);

connect();
</script>
</body>
</html>"""
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            self.wfile.write(html.encode())

    def log_message(self, format, *args):
        print(f"[{self.address_string()}] {args[0]}")


class ThreadedHTTPServer(HTTPServer):
    def process_request(self, request, client_address):
        thread = threading.Thread(target=self.process_request_thread, args=(request, client_address))
        thread.daemon = True
        thread.start()

    def process_request_thread(self, request, client_address):
        try:
            self.finish_request(request, client_address)
        except Exception:
            self.handle_error(request, client_address)
        finally:
            self.shutdown_request(request)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Webcam MJPEG HTTP streaming server')
    parser.add_argument('--port', type=int, default=PORT, help=f'HTTP port (default: {PORT})')
    parser.add_argument('--device', type=str, default=DEVICE, help=f'Video device (default: {DEVICE})')
    parser.add_argument('--width', type=int, default=WIDTH, help=f'Video width (default: {WIDTH})')
    parser.add_argument('--height', type=int, default=HEIGHT, help=f'Video height (default: {HEIGHT})')
    parser.add_argument('--fps', type=int, default=FPS, help=f'Frames per second (default: {FPS})')
    args = parser.parse_args()

    # Update globals from args
    PORT = args.port
    DEVICE = args.device
    WIDTH = args.width
    HEIGHT = args.height
    FPS = args.fps

    print(f"Starting webcam stream server")
    print(f"  Device: {DEVICE} @ {WIDTH}x{HEIGHT} {FPS}fps")
    print(f"  View at: http://<this-ip>:{PORT}")
    print(f"  Stream URL: http://<this-ip>:{PORT}/stream")
    print(f"  Snapshot URL: http://<this-ip>:{PORT}/snapshot")

    server = ThreadedHTTPServer(("0.0.0.0", PORT), MJPEGHandler)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down...")
        server.shutdown()
