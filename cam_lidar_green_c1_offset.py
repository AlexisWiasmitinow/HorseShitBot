import math
import time
import threading
from dataclasses import dataclass

import cv2
import numpy as np
from rplidar import RPLidar



LIDAR_PORT = "COM6"
LIDAR_BAUD = 460800

CAMERA_INDEX = 1  


FRAME_WIDTH = 1280
FRAME_HEIGHT = 720


LIDAR_IN_CAMERA_X_M = 0.10
LIDAR_IN_CAMERA_Y_M = 0.00
LIDAR_IN_CAMERA_Z_M = 0.00

CAMERA_HFOV_DEG = 70.0

CAMERA_YAW_IN_LIDAR_DEG = 0.0


BEARING_SIGN = +1.0  

ANGLE_WINDOW_DEG = 3.0
MIN_RANGE_M = 0.10
MAX_RANGE_M = 8.00

HSV_LOWER = (35, 80, 60)
HSV_UPPER = (85, 255, 255)
MIN_CONTOUR_AREA = 1500

USE_TRACKBARS = True
DRAW_LIDAR_MAP = True



def wrap_deg(a: float) -> float:
    return a % 360.0

def signed_deg(a: float) -> float:
    """Map degrees to [-180, 180)."""
    return (a + 180.0) % 360.0 - 180.0

def ang_diff_deg(a: float, b: float) -> float:
    d = (a - b + 180.0) % 360.0 - 180.0
    return abs(d)

def bearing_from_pixel(u: float, width: int, hfov_deg: float) -> float:
    """
    Pixel x -> bearing (deg) relative to camera forward (Z).
    Positive = to the right.
    """
    cx = width * 0.5
    hfov = math.radians(hfov_deg)
    fx = (width * 0.5) / math.tan(hfov * 0.5)
    x = (u - cx) / fx
    return math.degrees(math.atan(x))

def dir_from_theta_cam(theta_deg: float) -> np.ndarray:
    """
    Unit direction in camera frame in horizontal plane (Y=0):
    theta=0 -> +Z forward, theta>0 -> +X right
    """
    t = math.radians(theta_deg)
    return np.array([math.sin(t), 0.0, math.cos(t)], dtype=np.float32)

def draw_text_box(img, lines, x=10, y=10, font_scale=0.8, thickness=2,
                  text_color=(255, 255, 255), box_color=(0, 0, 0), alpha=0.65, line_gap=10):
    """Draw multi-line text with a semi-transparent background box."""
    font = cv2.FONT_HERSHEY_SIMPLEX
    sizes = [cv2.getTextSize(t, font, font_scale, thickness)[0] for t in lines]
    w = max(s[0] for s in sizes) + 20
    h = sum(s[1] for s in sizes) + (len(lines) - 1) * line_gap + 20

    overlay = img.copy()
    cv2.rectangle(overlay, (x, y), (x + w, y + h), box_color, -1)
    cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0, img)

    yy = y + 14
    for (t, (tw, th)) in zip(lines, sizes):
        yy += th
        cv2.putText(img, t, (x + 10, yy), font, font_scale, text_color, thickness, cv2.LINE_AA)
        yy += line_gap



@dataclass
class LidarScan:
    angles_deg: np.ndarray
    ranges_m: np.ndarray

class LidarStreamer:
    def __init__(self, port: str, baudrate: int):
        self.port = port
        self.baudrate = baudrate
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._last_scan: LidarScan | None = None
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._lidar: RPLidar | None = None

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2.0)
        self._cleanup()

    def _cleanup(self):
        if self._lidar is not None:
            try: self._lidar.stop()
            except Exception: pass
            try: self._lidar.stop_motor()
            except Exception: pass
            try: self._lidar.disconnect()
            except Exception: pass
            self._lidar = None

    def _run(self):
        try:
            self._lidar = RPLidar(self.port, baudrate=self.baudrate, timeout=3)
            time.sleep(0.5)

            for scan in self._lidar.iter_scans():
                if self._stop.is_set():
                    break

                angles = []
                ranges = []
                for (_, a, d_mm) in scan:
                    if d_mm <= 0:
                        continue
                    r_m = d_mm / 1000.0
                    if MIN_RANGE_M <= r_m <= MAX_RANGE_M:
                        angles.append(float(a))
                        ranges.append(float(r_m))

                if len(angles) < 10:
                    continue

                with self._lock:
                    self._last_scan = LidarScan(
                        angles_deg=np.array(angles, dtype=np.float32),
                        ranges_m=np.array(ranges, dtype=np.float32),
                    )
        except Exception as e:
            print(f"[LiDAR] ERROR: {e}")
        finally:
            self._cleanup()

    def get_range_at_angle(self, target_deg: float, window_deg: float) -> float | None:
        with self._lock:
            scan = self._last_scan
        if scan is None:
            return None

        diffs = np.array([ang_diff_deg(a, target_deg) for a in scan.angles_deg], dtype=np.float32)
        mask = diffs <= window_deg
        if not np.any(mask):
            return None

        return float(np.median(scan.ranges_m[mask]))

    def get_last_scan(self) -> LidarScan | None:
        with self._lock:
            return self._last_scan


def setup_trackbars():
    cv2.namedWindow("HSV Tuning", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("HSV Tuning", 500, 220)
    cv2.createTrackbar("H_low", "HSV Tuning", HSV_LOWER[0], 179, lambda x: None)
    cv2.createTrackbar("S_low", "HSV Tuning", HSV_LOWER[1], 255, lambda x: None)
    cv2.createTrackbar("V_low", "HSV Tuning", HSV_LOWER[2], 255, lambda x: None)
    cv2.createTrackbar("H_up",  "HSV Tuning", HSV_UPPER[0], 179, lambda x: None)
    cv2.createTrackbar("S_up",  "HSV Tuning", HSV_UPPER[1], 255, lambda x: None)
    cv2.createTrackbar("V_up",  "HSV Tuning", HSV_UPPER[2], 255, lambda x: None)

def read_trackbar_hsv():
    hL = cv2.getTrackbarPos("H_low", "HSV Tuning")
    sL = cv2.getTrackbarPos("S_low", "HSV Tuning")
    vL = cv2.getTrackbarPos("V_low", "HSV Tuning")
    hU = cv2.getTrackbarPos("H_up", "HSV Tuning")
    sU = cv2.getTrackbarPos("S_up", "HSV Tuning")
    vU = cv2.getTrackbarPos("V_up", "HSV Tuning")
    return (hL, sL, vL), (hU, sU, vU)

def detect_green_centroid(frame_bgr, hsv_lower, hsv_upper):
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(hsv_lower, dtype=np.uint8), np.array(hsv_upper, dtype=np.uint8))

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, mask

    c = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(c)
    if area < MIN_CONTOUR_AREA:
        return None, mask

    M = cv2.moments(c)
    if M["m00"] == 0:
        return None, mask

    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    return (cx, cy, c, area), mask


def draw_lidar_minimap_camera_frame(scan: LidarScan,
                                   size=600,
                                   meters_per_pixel=0.02,
                                   lidar_origin=(LIDAR_IN_CAMERA_X_M, LIDAR_IN_CAMERA_Z_M),
                                   yaw_deg=0.0):
    """
    Top-down mini-map in camera X-Z plane:
      - center = camera origin
      - +Z up (forward)
      - +X right
    Draw LiDAR origin at its offset.
    """
    img = np.zeros((size, size, 3), dtype=np.uint8)
    cx = cy = size // 2

    # axes
    cv2.line(img, (cx, cy), (cx, 0), (80, 80, 80), 1)      # +Z
    cv2.line(img, (cx, cy), (size, cy), (80, 80, 80), 1)   # +X

    # camera origin
    cv2.circle(img, (cx, cy), 5, (255, 255, 255), -1)

    # lidar origin
    lx, lz = lidar_origin
    lidar_px = int(cx + (lx / meters_per_pixel))
    lidar_py = int(cy - (lz / meters_per_pixel))
    cv2.circle(img, (lidar_px, lidar_py), 5, (0, 255, 255), -1)

    for a_deg, r in zip(scan.angles_deg, scan.ranges_m):
        theta_cam = signed_deg(a_deg - yaw_deg)  # 0 -> +Z
        d = dir_from_theta_cam(theta_cam)
        p = np.array([LIDAR_IN_CAMERA_X_M, 0.0, LIDAR_IN_CAMERA_Z_M], dtype=np.float32) + r * d

        X, Z = float(p[0]), float(p[2])
        px = int(cx + X / meters_per_pixel)
        py = int(cy - Z / meters_per_pixel)
        if 0 <= px < size and 0 <= py < size:
            img[py, px] = (0, 255, 0)

    return img



def main():
    global CAMERA_YAW_IN_LIDAR_DEG, BEARING_SIGN

    # Big resizable windows
    cv2.namedWindow("Fusion", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Fusion", 1300, 850)

    cv2.namedWindow("Green mask", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Green mask", 800, 600)

    if DRAW_LIDAR_MAP:
        cv2.namedWindow("LiDAR map (camera X-Z)", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("LiDAR map (camera X-Z)", 700, 700)

    if USE_TRACKBARS:
        setup_trackbars()

    lidar = LidarStreamer(LIDAR_PORT, LIDAR_BAUD)
    lidar.start()

    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print("[Camera] Could not open camera.")
        lidar.stop()
        return

    if FRAME_WIDTH is not None:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    if FRAME_HEIGHT is not None:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    lidar_origin_cam = np.array([LIDAR_IN_CAMERA_X_M, LIDAR_IN_CAMERA_Y_M, LIDAR_IN_CAMERA_Z_M], dtype=np.float32)

    print("Controls:")
    print("  Q      quit")
    print("  A/D    yaw -/+ 1 deg")
    print("  Z/C    yaw -/+ 0.1 deg")
    print("  F      flip left/right (BEARING_SIGN)")

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        h, w = frame.shape[:2]
        hsvL, hsvU = read_trackbar_hsv() if USE_TRACKBARS else (HSV_LOWER, HSV_UPPER)
        det, mask = detect_green_centroid(frame, hsvL, hsvU)

        info_lines = [
            f"COM={LIDAR_PORT} | camIndex={CAMERA_INDEX} | HFOV={CAMERA_HFOV_DEG:.1f}deg | YAW={CAMERA_YAW_IN_LIDAR_DEG:.1f}deg | SIGN={BEARING_SIGN:+.0f}",
            f"LiDAR in camera: X={LIDAR_IN_CAMERA_X_M:.3f}m, Y={LIDAR_IN_CAMERA_Y_M:.3f}m, Z={LIDAR_IN_CAMERA_Z_M:.3f}m",
            "Object: not detected",
        ]

        if det is not None:
            u, v, contour, area = det

            # Draw detection
            cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)
            cv2.circle(frame, (u, v), 7, (0, 0, 255), -1)
            cv2.line(frame, (w // 2, 0), (w // 2, h), (80, 80, 80), 1)

            bearing_deg = bearing_from_pixel(u, w, CAMERA_HFOV_DEG) * BEARING_SIGN
            target_lidar_deg = wrap_deg(CAMERA_YAW_IN_LIDAR_DEG + bearing_deg)

            r_m = lidar.get_range_at_angle(target_lidar_deg, ANGLE_WINDOW_DEG)

            if r_m is None:
                info_lines[2] = f"Object: bearing={bearing_deg:+.1f}deg | lidarAng={target_lidar_deg:.1f}deg | range=NONE"
            else:
                theta_cam_deg = signed_deg(target_lidar_deg - CAMERA_YAW_IN_LIDAR_DEG)
                d_cam = dir_from_theta_cam(theta_cam_deg)

                p_cam = lidar_origin_cam + r_m * d_cam
                X, Y, Z = float(p_cam[0]), float(p_cam[1]), float(p_cam[2])

                info_lines[2] = (
                    f"Object XYZ (camera frame):  X={X:.3f} m,  Y={Y:.3f} m (2D lidar -> ~0),  Z={Z:.3f} m   | r={r_m:.2f} m"
                )

                label = f"X={X:.2f}m  Y={Y:.2f}m  Z={Z:.2f}m"
                cv2.putText(frame, label, (u + 12, max(35, v - 12)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2, cv2.LINE_AA)

        draw_text_box(frame, info_lines, x=10, y=10, font_scale=0.85, thickness=2)

        cv2.imshow("Fusion", frame)
        cv2.imshow("Green mask", mask)

        if DRAW_LIDAR_MAP:
            scan = lidar.get_last_scan()
            if scan is not None:
                mini = draw_lidar_minimap_camera_frame(
                    scan,
                    size=650,
                    meters_per_pixel=0.02,
                    lidar_origin=(LIDAR_IN_CAMERA_X_M, LIDAR_IN_CAMERA_Z_M),
                    yaw_deg=CAMERA_YAW_IN_LIDAR_DEG,
                )
                cv2.imshow("LiDAR map (camera X-Z)", mini)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), ord('Q')):
            break
        elif key in (ord('a'), ord('A')):
            CAMERA_YAW_IN_LIDAR_DEG -= 1.0
        elif key in (ord('d'), ord('D')):
            CAMERA_YAW_IN_LIDAR_DEG += 1.0
        elif key in (ord('z'), ord('Z')):
            CAMERA_YAW_IN_LIDAR_DEG -= 0.1
        elif key in (ord('c'), ord('C')):
            CAMERA_YAW_IN_LIDAR_DEG += 0.1
        elif key in (ord('f'), ord('F')):
            BEARING_SIGN *= -1.0

    cap.release()
    cv2.destroyAllWindows()
    lidar.stop()


if __name__ == "__main__":
    main()
