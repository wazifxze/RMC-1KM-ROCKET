"""
download_tiles.py
------------------
Run this ONCE, ahead of time, while you still have internet — at home,
in the hotel, wherever. It downloads the actual map tile images for a
circular area around your launch/recovery site and saves them to a local
'tiles/' folder. On launch day, the ground station serves these files
directly, no internet required.

Usage:
    python download_tiles.py --lat 3.1201 --lon 101.6146 --radius-km 8 --min-zoom 12 --max-zoom 17

--lat / --lon      : center point (your launch pad, or best-guess recovery area center)
--radius-km        : how far out from center to cache tiles (rocket's expected max drift)
--min-zoom         : lowest zoom level to cache (12 ≈ regional view)
--max-zoom         : highest zoom level to cache (17 ≈ street-level detail)

Higher max-zoom and larger radius = more tiles = longer download + more disk space.
A radius of 5-10km at zoom 12-17 is typically a few thousand tiles, a few hundred MB.
"""

import argparse
import math
import os
import time
import urllib.request

USER_AGENT = "RocketRecoveryTracker/1.0 (personal offline cache, one-time use)"
TILE_URL_TEMPLATE = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"
REQUEST_DELAY_SEC = 0.3  # be a polite, low-volume client per OSM's tile usage policy


def deg2num(lat_deg, lon_deg, zoom):
    lat_rad = math.radians(lat_deg)
    n = 2.0 ** zoom
    x = int((lon_deg + 180.0) / 360.0 * n)
    y = int((1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n)
    return x, y


def km_to_tile_span(radius_km, lat_deg, zoom):
    """Rough number of tiles to cover radius_km at this latitude and zoom."""
    # Multiply by 256 to convert meters/pixel to meters/tile:
    meters_per_tile = (156543.03 * 256) * math.cos(math.radians(lat_deg)) / (2 ** zoom)
    tiles_needed = (radius_km * 1000) / meters_per_tile
    return max(1, int(math.ceil(tiles_needed)))


def download_tile(z, x, y, out_dir):
    tile_dir = os.path.join(out_dir, str(z), str(x))
    os.makedirs(tile_dir, exist_ok=True)
    out_path = os.path.join(tile_dir, f"{y}.png")

    if os.path.exists(out_path):
        return "skip"  # already cached from a previous run

    url = TILE_URL_TEMPLATE.format(z=z, x=x, y=y)
    req = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
    try:
        with urllib.request.urlopen(req, timeout=10) as resp:
            data = resp.read()
        with open(out_path, "wb") as f:
            f.write(data)
        return "ok"
    except Exception as e:
        print(f"  [WARN] failed {z}/{x}/{y}: {e}")
        return "fail"


def main():
    parser = argparse.ArgumentParser(description="Pre-download OSM tiles for offline recovery tracking")
    parser.add_argument("--lat", type=float, required=True)
    parser.add_argument("--lon", type=float, required=True)
    parser.add_argument("--radius-km", type=float, default=8.0)
    parser.add_argument("--min-zoom", type=int, default=12)
    parser.add_argument("--max-zoom", type=int, default=17)
    parser.add_argument("--out-dir", type=str, default="tiles")
    args = parser.parse_args()

    total_ok, total_skip, total_fail = 0, 0, 0

    for zoom in range(args.min_zoom, args.max_zoom + 1):
        cx, cy = deg2num(args.lat, args.lon, zoom)
        span = km_to_tile_span(args.radius_km, args.lat, zoom)

        x_range = range(cx - span, cx + span + 1)
        y_range = range(cy - span, cy + span + 1)
        total_tiles = len(x_range) * len(y_range)

        print(f"Zoom {zoom}: downloading ~{total_tiles} tiles...")

        for i, x in enumerate(x_range):
            for y in y_range:
                result = download_tile(zoom, x, y, args.out_dir)
                if result == "ok":
                    total_ok += 1
                    time.sleep(REQUEST_DELAY_SEC)
                elif result == "skip":
                    total_skip += 1
                else:
                    total_fail += 1

    print(f"\nDone. Downloaded: {total_ok}  Already cached: {total_skip}  Failed: {total_fail}")
    print(f"Tiles saved to: {os.path.abspath(args.out_dir)}")
    print("Copy this 'tiles' folder alongside gps_map_server.py before launch day.")


if __name__ == "__main__":
    main()
