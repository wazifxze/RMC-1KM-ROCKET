"""
gps_map_server.py
------------------
Lightweight, dependency-free live GPS map server for the recovery phase.

Runs a small background HTTP server (Python's built-in http.server, no Flask
required) that serves map.html and a /gps.json endpoint. The ground station
script calls update_gps(...) every time a new GPS field is parsed; this module
keeps an in-memory track history and writes it out as JSON on request.

Usage from groundStationV1.1.py:

    import gps_map_server
    gps_map_server.start_server(port=8000)
    ...
    # inside the parsing block, right after gps_sats is parsed:
    gps_map_server.update_gps(gps_lat, gps_lon, gps_alt, gps_fix, gps_sats, timestamp_ms)

Then open http://localhost:8000/ in a browser (on the ground laptop, or on a
phone connected to the same hotspot/network as the laptop) to see the live map.
"""

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

_LOCK = threading.Lock()
_STATE = {
    "lat": None,
    "lon": None,
    "alt": None,
    "fix": False,
    "sats": 0,
    "timestamp_ms": None,
    "last_update_wall": None,   # server-side time.time(), used to detect stale data
}
_TRAIL = []          # list of [lat, lon] breadcrumb points, valid fixes only
_MAX_TRAIL_POINTS = 5000   # bounded so a long recovery chase doesn't grow memory forever

_HTML_PATH = Path(__file__).parent / "map.html"
_TILES_DIR = Path(__file__).parent / "tiles"


def update_gps(lat, lon, alt, fix, sats, timestamp_ms):
    """Call this once per parsed telemetry packet from the ground station loop."""
    with _LOCK:
        _STATE["lat"] = lat
        _STATE["lon"] = lon
        _STATE["alt"] = alt
        _STATE["fix"] = bool(fix)
        _STATE["sats"] = sats
        _STATE["timestamp_ms"] = timestamp_ms
        _STATE["last_update_wall"] = time.time()

        if fix and lat != 0.0 and lon != 0.0:
            _TRAIL.append([lat, lon])
            if len(_TRAIL) > _MAX_TRAIL_POINTS:
                _TRAIL.pop(0)


def _snapshot_json():
    with _LOCK:
        payload = dict(_STATE)
        payload["trail"] = list(_TRAIL)
    return json.dumps(payload).encode("utf-8")


class _Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass  # silence default request logging so it doesn't spam the console

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self._serve_file(_HTML_PATH, "text/html")
        elif self.path == "/gps.json":
            body = _snapshot_json()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
        elif self.path.startswith("/tiles/"):
            # Expects /tiles/{z}/{x}/{y}.png, matching download_tiles.py's folder layout
            rel_path = self.path[len("/tiles/"):]
            tile_path = (_TILES_DIR / rel_path).resolve()

            # Guard against path traversal outside the tiles folder
            if _TILES_DIR.resolve() not in tile_path.parents:
                self.send_response(403)
                self.end_headers()
                return

            if tile_path.exists():
                self._serve_file(tile_path, "image/png")
            else:
                # Missing tile (outside cached radius/zoom) — respond quietly,
                # Leaflet just shows a blank square rather than erroring.
                self.send_response(404)
                self.end_headers()
        else:
            self.send_response(404)
            self.end_headers()

    def _serve_file(self, path, content_type):
        try:
            body = path.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        except FileNotFoundError:
            self.send_response(500)
            self.end_headers()
            self.wfile.write(b"map.html not found next to gps_map_server.py")


def start_server(port=8000):
    """Starts the server on a background daemon thread. Safe to call once at startup."""
    server = ThreadingHTTPServer(("0.0.0.0", port), _Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    print(f"[GPS MAP] Live tracker running at http://localhost:{port}/")
    return server
