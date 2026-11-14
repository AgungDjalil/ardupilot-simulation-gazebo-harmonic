#!/usr/bin/env python3
"""Gimbal cameras from Gazebo as MJPEG over HTTP, for the Symphony station's video sources.

    camera_bridge.py G1:8601 G2:8602 ...

For each NAME:PORT it subscribes to /NAME/camera/image (make_world.py) and serves

    http://127.0.0.1:PORT/            multipart/x-mixed-replace MJPEG (what the station opens with OpenCV/FFmpeg)
    http://127.0.0.1:PORT/snapshot.jpg  the latest frame

Put http://127.0.0.1:PORT/ in the aircraft's video_rtsp_url. The station accepts http:// sources (its TCP probe
and decoder handle them), so nothing in Symphony changes. On the aircraft the camera is RTSP from the Pi's
MediaMTX; this stands in for it because neither an RTSP server nor an H.264 encoder GstCameraPlugin can use is
installed here.

Frames are JPEG-encoded once per new image, at the camera's own rate, and every client gets the latest one: a
slow client skips frames instead of queueing them. SYM_JPEG_QUALITY sets the quality (default 80).
"""
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node

QUALITY = int(os.environ.get("SYM_JPEG_QUALITY", "80"))
# gz.msgs.PixelFormatType
RGB_INT8, BGR_INT8, L_INT8 = 3, 8, 1


class Camera:
    def __init__(self, name: str):
        self.name = name
        self.topic = f"/{name}/camera/image"
        self.cv = threading.Condition()
        self.jpeg = None
        self.seq = 0
        self.frames = 0
        self.last = 0.0

    def on_image(self, msg: Image) -> None:
        w, h, step = msg.width, msg.height, msg.step
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.pixel_format_type == L_INT8:
            img = buf.reshape(h, step)[:, :w]
        else:
            img = buf.reshape(h, step)[:, : w * 3].reshape(h, w, 3)
            if msg.pixel_format_type == RGB_INT8:
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, QUALITY])
        if not ok:
            return
        with self.cv:
            self.jpeg = jpg.tobytes()
            self.seq += 1
            self.frames += 1
            self.last = time.time()
            self.cv.notify_all()

    def wait(self, last_seq: int, timeout: float):
        with self.cv:
            if self.seq == last_seq:
                self.cv.wait(timeout)
            return self.jpeg, self.seq


def handler_for(cam: Camera):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass

        def do_GET(self):
            if self.path.startswith("/snapshot"):
                jpg, _ = cam.wait(-1, 5.0)
                if jpg is None:
                    self.send_error(503, "no frame from Gazebo yet")
                    return
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(jpg)))
                self.end_headers()
                self.wfile.write(jpg)
                return
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            seq = 0
            try:
                while True:
                    jpg, new = cam.wait(seq, 5.0)
                    if jpg is None or new == seq:
                        continue
                    seq = new
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(jpg))
                    self.wfile.write(jpg)
                    self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass

    return Handler


def main() -> None:
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    node = Node()
    cams = []
    for arg in sys.argv[1:]:
        name, port = arg.split(":")
        cam = Camera(name)
        if not node.subscribe(Image, cam.topic, cam.on_image):
            sys.exit(f"cannot subscribe to {cam.topic}")
        srv = ThreadingHTTPServer(("0.0.0.0", int(port)), handler_for(cam))
        srv.daemon_threads = True
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        print(f"{name}: {cam.topic} -> http://127.0.0.1:{port}/", flush=True)
        cams.append(cam)
    while True:
        time.sleep(30)
        print("  ".join(f"{c.name} {c.frames} frames, last {time.time() - c.last:.0f} s ago" if c.last
                        else f"{c.name} no frames yet" for c in cams), flush=True)


if __name__ == "__main__":
    main()
