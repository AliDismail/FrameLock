import face_recognition
import cv2
import numpy as np
from picamera2 import Picamera2
import time
import pickle
import threading
import serial
from gpiozero_extended import Motor, PID

# List of names that will trigger the GPIO pin (case-sensitive)
authorized_names = ["Ali Ismail", "Nour Zebian", "Farah Micheal", "Maher AlRafei"]

# Map IR keys to names (edit to match your encodings.pickle order or preferences)
IR_BUTTON_TO_NAME = {
    "KEY_0": "Ali Ismail",
    "KEY_1": "Nour Zebian",
    "KEY_2": "Farah Micheal",
    "KEY_3": "Maher AlRafei",
    # extend as needed:
    # "KEY_4": "Someone Else",
    # "KEY_5": "...",
    # ...
}

# ------------------------------
# Serial IR selector (Arduino → Pi)
# ------------------------------
class SerialIRSelector:
    def __init__(self, tracker, port="/dev/ttyUSB0", baud=9600):
        self.tracker = tracker
        self.port = port
        self.baud = baud
        self._stop = threading.Event()
        self._thread = None
        self._ser = None

    def start(self):
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=0.1)
            print(f"[IR] Connected to {self.port} @ {self.baud}")
        except Exception as e:
            print(f"[IR][WARN] Could not open serial port {self.port}: {e}")
            self._ser = None
            return

        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        while not self._stop.is_set():
            try:
                if self._ser and self._ser.in_waiting > 0:
                    code = self._ser.readline().decode(errors="ignore").strip()
                    if code:
                        # Expecting strings like "KEY_0" .. "KEY_9" from Arduino
                        if code in IR_BUTTON_TO_NAME:
                            target = IR_BUTTON_TO_NAME[code]
                            self.tracker.set_target(target)
                            print(f"[IR] {code} → target set: {target}")
                        else:
                            print(f"[IR] Unmapped code: {code}")
            except Exception as e:
                print(f"[IR][ERR] Serial read error: {e}")
                time.sleep(0.1)
            time.sleep(0.01)

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        if self._ser:
            try:
                self._ser.close()
            except:
                pass


# ------------------------------
# Vision: face detection + distance + overlay
# ------------------------------
class FaceLocation:
    def __init__(self,
                 encoding_file="encodings.pickle",
                 tolerance=0.1,
                 target_face_height_norm=0.25,
                 distance_tolerance_norm=0.03,
                 reference_distance_m=1.0,
                 cv_scaler=4):
        self.tolerance = tolerance
        self.TARGET_FACE_HEIGHT_NORM = target_face_height_norm
        self.DISTANCE_TOLERANCE_NORM = distance_tolerance_norm
        self.REFERENCE_DISTANCE_M = reference_distance_m
        self.cv_scaler = cv_scaler

        print("[INFO] loading encodings...")
        with open(encoding_file, "rb") as f:
            data = pickle.loads(f.read())
        self.known_face_encodings = data["encodings"]
        self.known_face_names = data["names"]

        self.picam2 = Picamera2()
        self.picam2.configure(self.picam2.create_preview_configuration(
            main={"format": 'XRGB8888', "size": (1920, 1080)}
        ))
        self.picam2.start()

        self.face_locations = []
        self.face_names = []
        self.target_name = None  # chosen face to follow

    def set_target(self, name):
        """Choose which face to follow by name."""
        if name in self.known_face_names:
            self.target_name = name
            print(f"[INFO] Target face set to: {name}")
        else:
            print(f"[WARN] {name} not in known faces. Known: {self.known_face_names}")

    def clear_target(self):
        self.target_name = None
        print("[INFO] Target face cleared")

    def process_frame(self, frame):
        resized = cv2.resize(frame, (0, 0), fx=1/self.cv_scaler, fy=1/self.cv_scaler)
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)

        self.face_locations = face_recognition.face_locations(rgb)
        encodings = face_recognition.face_encodings(rgb, self.face_locations)

        self.face_names = []
        for encoding in encodings:
            matches = face_recognition.compare_faces(self.known_face_encodings, encoding)
            name = "Unknown"
            if matches:
                distances = face_recognition.face_distance(self.known_face_encodings, encoding)
                best = np.argmin(distances)
                if matches[best]:
                    name = self.known_face_names[best]
            self.face_names.append(name)

    def get_distance_from_camera(self, frame):
        results = []
        h, w = frame.shape[:2]
        for (top, right, bottom, left), name in zip(self.face_locations, self.face_names):
            top *= self.cv_scaler
            bottom *= self.cv_scaler
            face_height = bottom - top
            norm_height = face_height / h
            safe_height = max(norm_height, 1e-6)
            distance_m = self.REFERENCE_DISTANCE_M * (self.TARGET_FACE_HEIGHT_NORM / safe_height)

            if safe_height < (self.TARGET_FACE_HEIGHT_NORM - self.DISTANCE_TOLERANCE_NORM):
                state = "FAR"
            elif safe_height > (self.TARGET_FACE_HEIGHT_NORM + self.DISTANCE_TOLERANCE_NORM):
                state = "CLOSE"
            else:
                state = "OK"

            results.append((name, distance_m, state))
        return results

    def get_recognized_names(self):
        if self.target_name:
            return [name for name in self.face_names if name == self.target_name]
        else:
            return [name for name in self.face_names if name in authorized_names]

    def get_face_deviation(self, frame):
        """Return signed deviation from center [-0.5, +0.5] for the target (or first authorized)."""
        if not self.face_locations:
            return None
        h, w = frame.shape[:2]

        # pick target face if set, else first authorized
        idx = None
        if self.target_name and self.target_name in self.face_names:
            idx = self.face_names.index(self.target_name)
        else:
            for i, nm in enumerate(self.face_names):
                if nm in authorized_names:
                    idx = i
                    break
        if idx is None:
            return None

        (top, right, bottom, left) = self.face_locations[idx]
        top *= self.cv_scaler
        right *= self.cv_scaler
        bottom *= self.cv_scaler
        left *= self.cv_scaler
        center_x = (left + right) / 2.0
        norm_x = center_x / w
        deviation = norm_x - 0.5
        return deviation

    def annotate_frame(self, frame):
        h, w = frame.shape[:2]
        mid_x = w // 2
        cv2.line(frame, (mid_x, 0), (mid_x, h), (255, 0, 0), 2)

        for (top, right, bottom, left), name in zip(self.face_locations, self.face_names):
            top *= self.cv_scaler
            right *= self.cv_scaler
            bottom *= self.cv_scaler
            left *= self.cv_scaler

            face_height = bottom - top
            norm_height = face_height / h
            safe_height = max(norm_height, 1e-6)
            distance_m = self.REFERENCE_DISTANCE_M * (self.TARGET_FACE_HEIGHT_NORM / safe_height)

            center_x = (left + right) / 2.0
            norm_x = center_x / w
            deviation = norm_x - 0.5
            if abs(deviation) < self.tolerance:
                position = "CENTER"
            elif deviation < 0:
                position = "LEFT"
            else:
                position = "RIGHT"

            # highlight target in green, authorized in cyan, unknown in red
            if name == self.target_name:
                color = (0, 255, 0)
            elif name in authorized_names:
                color = (255, 255, 0)
            else:
                color = (0, 0, 255)

            cv2.rectangle(frame, (left, top), (right, bottom), color, 2)
            label = f"{name} | {distance_m:.2f}m | {position} ({deviation:.2f})"
            cv2.rectangle(frame, (left, top - 25), (right, top), color, cv2.FILLED)
            cv2.putText(frame, label, (left + 6, top - 6),
                        cv2.FONT_HERSHEY_DUPLEX, 0.6, (255, 255, 255), 1)
        return frame

    def cleanup(self):
        cv2.destroyAllWindows()
        self.picam2.stop()


# ------------------------------
# Motor control for distance (forward/backward)
# ------------------------------
class Motor_Control:
    def __init__(self,
                 left_pins,
                 right_pins,
                 kp, ki, kd,
                 Ts=0.05, tau=0.1,
                 setpoint=1.0, tolerance=1e-3,
                 min_torque=0.35, startup_boost_time=0.15,
                 tracker: FaceLocation = None):
        self.motor_left = Motor(enable1=left_pins[0], pwm1=left_pins[1], pwm2=left_pins[2])
        self.motor_right = Motor(enable1=right_pins[0], pwm1=right_pins[1], pwm2=right_pins[2])

        self.Ts = Ts
        self.pid = PID(Ts, kp, ki, kd, tau=tau)

        self.setpoint = setpoint
        self.tolerance = tolerance
        self.min_torque = min_torque
        self.startup_boost_time = startup_boost_time

        # Reuse the shared tracker to avoid multiple camera instances
        self.tracker = tracker

        self._start_time = time.perf_counter()

    def step_distance_control(self, frame):
        distances = self.tracker.get_distance_from_camera(frame)
        recognized = self.tracker.get_recognized_names()

        if not distances or not recognized:
            print("No target/authorized face detected. Stopping motors.")
            self.motor_left.set_output(0)
            self.motor_right.set_output(0)
            return

        # pick the target’s distance
        name_to_follow = self.tracker.target_name if self.tracker.target_name else recognized[0]
        distance = None
        for nm, dist, _ in distances:
            if nm == name_to_follow:
                distance = dist
                break

        if distance is None:
            self.motor_left.set_output(0)
            self.motor_right.set_output(0)
            return

        error = self.setpoint - distance

        if abs(error) > self.tolerance:
            output = self.pid.control(self.setpoint, distance)

            # Startup boost to overcome static friction
            elapsed = time.perf_counter() - self._start_time
            if elapsed < self.startup_boost_time:
                output = 1.0

            # Minimum torque floor
            if abs(output) < self.min_torque:
                output = self.min_torque * (1 if output >= 0 else -1)

            # Apply forward/backward equally
            self.motor_left.set_output(output)
            self.motor_right.set_output(output)
        else:
            self.motor_left.set_output(0)
            self.motor_right.set_output(0)
            print(f"Target distance {self.setpoint:.2f} m maintained.")


# ------------------------------
# Hybrid controller (centering priority, then distance)
# ------------------------------
class HybridController:
    def __init__(self,
                 left_pins, right_pins,
                 kp_dist, ki_dist, kd_dist,
                 kp_cen, ki_cen, kd_cen,
                 Ts=0.05, tau=0.1, setpoint=1.0,
                 ir_port="/dev/ttyUSB0"):
        # Shared vision tracker
        self.tracker = FaceLocation()

        # Start IR selector thread
        self.ir_selector = SerialIRSelector(self.tracker, port=ir_port, baud=9600)
        self.ir_selector.start()

        # PID controllers
        self.center_pid = PID(Ts, kp_cen, ki_cen, kd_cen, tau=tau)
        self.distance_ctrl = Motor_Control(left_pins, right_pins,
                                           kp=kp_dist, ki=ki_dist, kd=kd_dist,
                                           Ts=Ts, tau=tau, setpoint=setpoint,
                                           tracker=self.tracker)

        # Direct motor handles
        self.motor_left = self.distance_ctrl.motor_left
        self.motor_right = self.distance_ctrl.motor_right

        # Tolerances and floors
        self.deviation_tolerance = 0.05      # center tolerance (normalized x)
        self.min_spin_floor = 0.40           # spin-in-place floor
        self.center_startup_boost_time = 0.20

        # Timing
        self.Ts = Ts
        self._center_start = time.perf_counter()

    def run(self):
        try:
            while True:
                frame = self.tracker.picam2.capture_array()
                self.tracker.process_frame(frame)

                deviation = self.tracker.get_face_deviation(frame)
                recognized = self.tracker.get_recognized_names()

                if (self.tracker.target_name and self.tracker.target_name not in self.tracker.face_names):
                    print("Target not visible or no authorized face detected.")
                    self.motor_left.set_output(0)
                    self.motor_right.set_output(0)
                elif deviation is not None and abs(deviation) > self.deviation_tolerance:
                    # Centering mode (higher priority)
                    output = self.center_pid.control(0.0, deviation)

                    # Startup boost for centering
                    elapsed_c = time.perf_counter() - self._center_start
                    if elapsed_c < self.center_startup_boost_time:
                        output = 1.0

                    # Minimum spin torque floor
                    if abs(output) < self.min_spin_floor:
                        output = self.min_spin_floor * (1 if output >= 0 else -1)

                    # Spin in place to reduce horizontal error
                    self.motor_left.set_output(-output)
                    self.motor_right.set_output(output)
                else:
                    # Distance maintaining mode
                    self.distance_ctrl.step_distance_control(frame)

                # Show annotated feed
                annotated = self.tracker.annotate_frame(frame)
                cv2.imshow("Hybrid Face Tracking (IR-selectable target)", annotated)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

                time.sleep(self.Ts)
        finally:
            # Safe shutdown
            self.motor_left.set_output(0)
            self.motor_right.set_output(0)
            self.ir_selector.stop()
            self.tracker.cleanup()
            cv2.destroyAllWindows()


# ------------------------------
# Main: hard-coded values for your 37GB528 + L298N + ~5 kg robot
# ------------------------------
def main():
    # Motor pins (enable1, pwm1, pwm2)
    left_pins = (16, 17, 18)
    right_pins = (19, 20, 21)

    # Control timing and filter
    Ts = 0.05      # 20 Hz control loop (good for vision + L298N)
    tau = 0.10     # derivative low-pass

    # Target distance (meters)
    setpoint = 1.0

    # Distance PID (balanced)
    kp_dist = 0.55; ki_dist = 0.12; kd_dist = 0.04

    # Centering PID (balanced)
    kp_cen = 0.65; ki_cen = 0.15; kd_cen = 0.04

    # Set the serial port to your Arduino (check with ls /dev/ttyUSB* or /dev/ttyACM*)
    ir_port = "/dev/ttyUSB0"

    hybrid_ctrl = HybridController(
        left_pins=left_pins,
        right_pins=right_pins,
        kp_dist=kp_dist, ki_dist=ki_dist, kd_dist=kd_dist,
        kp_cen=kp_cen, ki_cen=ki_cen, kd_cen=kd_cen,
        Ts=Ts, tau=tau, setpoint=setpoint,
        ir_port=ir_port
    )

    print(f"Starting hybrid control with:"
          f" dist(kp={kp_dist}, ki={ki_dist}, kd={kd_dist}),"
          f" cen(kp={kp_cen}, ki={ki_cen}, kd={kd_cen}),"
          f" Ts={Ts}, tau={tau}, setpoint={setpoint}m")
    print("Use IR remote keys (KEY_0..KEY_3) to select target face.")

    try:
        hybrid_ctrl.run()
    finally:
        print("Cleaning up...")

if __name__ == "__main__":
    main()