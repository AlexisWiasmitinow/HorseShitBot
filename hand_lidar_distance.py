
import os, time, threading, urllib.request
import numpy as np
import cv2
import mediapipe as mp
from rplidar import RPLidar

MODEL_URL = "https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task"
MODEL_PATH = os.path.join(os.path.dirname(__file__), "hand_landmarker.task")

def ensure_model():
    if not os.path.exists(MODEL_PATH):
        print("Downloading MediaPipe hand model...")
        urllib.request.urlretrieve(MODEL_URL, MODEL_PATH)
        print("Saved:", MODEL_PATH)

def circular_diff(a, b):
    d = np.abs(a - b)
    return np.minimum(d, 360.0 - d)

class LidarReader:
    def __init__(self, port, baud=460800):
        self.port = port
        self.baud = baud
        self.lidar = None
        self.lock = threading.Lock()
        self.angles = np.array([], dtype=np.float32)
        self.dists_mm = np.array([], dtype=np.float32)
        self.ts = 0.0
        self.running = False
        self.thread = None

    def start(self):
        self.lidar = RPLidar(self.port, baudrate=self.baud, timeout=1)
        print("LIDAR info:", self.lidar.get_info())
        print("LIDAR health:", self.lidar.get_health())
        self.lidar.start_motor()
        time.sleep(0.5)

        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        try:
            for scan in self.lidar.iter_scans():
                if not self.running:
                    break
                a = []
                d = []
                for (_, ang, dist) in scan:
                    if dist > 0:
                        a.append(ang)
                        d.append(dist)
                if a:
                    with self.lock:
                        self.angles = np.array(a, dtype=np.float32)
                        self.dists_mm = np.array(d, dtype=np.float32)
                        self.ts = time.time()
        except Exception as e:
            print("LIDAR read error:", e)

    def get_distance_m(self, target_angle_deg, window_deg=2.5):
        with self.lock:
            if self.angles.size == 0:
                return None, None
            angles = self.angles.copy()
            dists = self.dists_mm.copy()
            ts = self.ts

        mask = circular_diff(angles, target_angle_deg) <= window_deg
        if not np.any(mask):
            return None, ts

        # choose the closest point in that angular window
        mm = float(np.min(dists[mask]))
        return mm / 1000.0, ts

    def stop(self):
        self.running = False
        try:
            if self.lidar:
                self.lidar.stop()
                self.lidar.stop_motor()
                self.lidar.disconnect()
        except Exception:
            pass

def main():
    # ---- SETTINGS YOU MUST EDIT ----
    CAMERA_INDEX = 0
    LIDAR_PORT   = "COM6"      # <-- change this
    LIDAR_BAUD   = 460800      # RPLIDAR C1 typical baud :contentReference[oaicite:4]{index=4}

    # Camera horizontal field-of-view (rough). 60 works ok for many webcams.
    HFOV_DEG     = 60.0

    # Angle (in lidar frame) that corresponds to the CENTER of the camera image.
    # You can auto-calibrate with key "C" (see below).
    LIDAR0_DEG    = 0.0

    ensure_model()

    # Start lidar reader thread
    lr = LidarReader(LIDAR_PORT, baud=LIDAR_BAUD)
    lr.start()

    # MediaPipe Tasks hand landmarker
    BaseOptions = mp.tasks.BaseOptions
    HandLandmarker = mp.tasks.vision.HandLandmarker
    HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions
    RunningMode = mp.tasks.vision.RunningMode

    options = HandLandmarkerOptions(
        base_options=BaseOptions(model_asset_path=MODEL_PATH),
        running_mode=RunningMode.VIDEO,
        num_hands=1,
        min_hand_detection_confidence=0.6,
        min_hand_presence_confidence=0.6,
        min_tracking_confidence=0.6,
    )

    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        lr.stop()
        raise RuntimeError("Cannot open camera.")

    start_t = time.time()

    print("Keys:")
    print("  Q = quit")
    print("  C = calibrate LIDAR0 (hold your hand CLOSE and in the scan plane, centered in the image)")

    try:
        with HandLandmarker.create_from_options(options) as landmarker:
            while True:
                ok, frame = cap.read()
                if not ok:
                    continue
                frame = cv2.flip(frame, 1)
                h, w = frame.shape[:2]

                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
                ts_ms = int((time.time() - start_t) * 1000)
                res = landmarker.detect_for_video(mp_image, ts_ms)

                hand_center_x = None
                hand_bbox = None

                if res.hand_landmarks:
                    lms = res.hand_landmarks[0]
                    xs = [lm.x for lm in lms]
                    ys = [lm.y for lm in lms]
                    x1, y1 = int(min(xs) * w), int(min(ys) * h)
                    x2, y2 = int(max(xs) * w), int(max(ys) * h)
                    hand_bbox = (x1, y1, x2, y2)
                    hand_center_x = (x1 + x2) / 2.0

                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

                    # pixel -> camera bearing (deg)
                    theta_cam = (hand_center_x - (w / 2.0)) / (w / 2.0) * (HFOV_DEG / 2.0)
                    # camera bearing -> lidar angle
                    theta_lidar = (LIDAR0_DEG + theta_cam) % 360.0

                    dist_m, scan_ts = lr.get_distance_m(theta_lidar, window_deg=3.0)

                    if dist_m is not None:
                        age = time.time() - scan_ts
                        cv2.putText(frame, f"Hand dist (lidar): {dist_m:.2f} m",
                                    (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                        cv2.putText(frame, f"angle={theta_lidar:.1f}deg  scan_age={age*1000:.0f}ms",
                                    (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    else:
                        cv2.putText(frame, f"Hand detected, no lidar return at {theta_lidar:.1f}deg",
                                    (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

                else:
                    cv2.putText(frame, "No hand", (10, 35),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

                cv2.imshow("Hand + LIDAR distance (Q quit, C calibrate)", frame)
                key = cv2.waitKey(1) & 0xFF

                if key in (ord('q'), ord('Q')):
                    break

                # Auto-calibration:
                # Hold your hand CLOSE (so it's the nearest object), centered in image, in the lidar scan plane.
                # Press C -> we set LIDAR0 so that camera-center aligns to the lidar angle of the closest measured point.
                if key in (ord('c'), ord('C')) and hand_center_x is not None:
                    with lr.lock:
                        if lr.angles.size > 0:
                            idx = int(np.argmin(lr.dists_mm))
                            closest_angle = float(lr.angles[idx])
                            # If hand is centered, theta_cam ~ 0, so LIDAR0 ~ closest_angle
                            LIDAR0_DEG = closest_angle
                            print(f"Calibrated LIDAR0_DEG = {LIDAR0_DEG:.2f} (assuming your hand was the closest object)")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        lr.stop()

if __name__ == "__main__":
    main()
