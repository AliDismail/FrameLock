from gpiozero_extended import Motor, PID
import time
import cv2
from distance_from_middle import FaceTracker   # your FaceTracker class

class MotorFaceCentering:
    def __init__(self,
                 left_pins,
                 right_pins,
                 Ts=0.05,
                 kp=0.6, ki=0.2, kd=0.05, tau=0.1,
                 x_setpoint=0.0, tolerance=0.05,
                 min_torque=0.6, startup_boost_time=0.2):
        """
        Initialize motors, PID controller, and face tracker for centering.
        """
        # Motors
        self.motor_left = Motor(enable1=left_pins[0], pwm1=left_pins[1], pwm2=left_pins[2])
        self.motor_right = Motor(enable1=right_pins[0], pwm1=right_pins[1], pwm2=right_pins[2])

        # PID controller
        self.Ts = Ts
        self.pid = PID(Ts, kp, ki, kd, tau=tau)

        # Target and tolerance
        self.x_setpoint = x_setpoint
        self.tolerance = tolerance

        # Torque constraints
        self.min_torque = min_torque
        self.startup_boost_time = startup_boost_time

        # Vision tracker
        self.tracker = FaceTracker()

    def get_face_x(self):
        """
        Use FaceTracker results to get normalized x deviation.
        Returns deviation in range [-0.5, +0.5], where 0 = center.
        """
        frame = self.tracker.picam2.capture_array()
        self.tracker.process_frame(frame)
        results = self.tracker.get_face_distance(frame)

        if not results:
            return None

        # Pick the first face (or extend logic to choose closest/known)
        name, position, dist_px, dist_norm = results[0]

        # dist_norm is always positive, but we need signed deviation
        # So reconstruct signed deviation from position
        if position == "LEFT":
            deviation = -dist_norm
        elif position == "RIGHT":
            deviation = dist_norm
        else:
            deviation = 0.0

        return deviation

    def motor_control_centering(self):
        """
        Run motor control until face is centered (error within tolerance).
        Applies startup boost and minimum torque floor.
        """
        try:
            start_time = time.perf_counter()

            while True:
                x_measured = self.get_face_x()
                if x_measured is None:
                    print("No face detected. Stopping motors.")
                    self.motor_left.set_output(0)
                    self.motor_right.set_output(0)
                    break

                error = self.x_setpoint - x_measured

                if abs(error) > self.tolerance:
                    # PID control output
                    output = self.pid.control(self.x_setpoint, x_measured)

                    # --- TORQUE CONSTRAINTS ---
                    elapsed = time.perf_counter() - start_time

                    if elapsed < self.startup_boost_time:
                        output = 1.0
                    else:
                        if abs(output) < self.min_torque:
                            output = self.min_torque * (1 if output >= 0 else -1)

                    # Apply to motors (spin in place)
                    self.motor_left.set_output(-output)
                    self.motor_right.set_output(output)

                else:
                    self.motor_left.set_output(0)
                    self.motor_right.set_output(0)
                    print("Face centered. Stopping motors.")
                    break

                time.sleep(self.Ts)
        finally:
            self.motor_left.set_output(0)
            self.motor_right.set_output(0)
            cv2.destroyAllWindows()
            self.tracker.picam2.stop()


"""
from gpiozero_extended import Motor, PID
import time

# Initialize motor driver for rotation (left/right wheels or differential drive)
# If you have two motors, you can adapt this to spin in place
motor_left = Motor(enable1=16, pwm1=17, pwm2=18)
motor_right = Motor(enable1=19, pwm1=20, pwm2=21)

# PID controller with low-pass filter on derivative
Ts = 0.05   # 50 ms loop
kp = 0.6    # proportional gain
ki = 0.2    # integral gain
kd = 0.05   # derivative gain
tau = 0.1   # low-pass filter time constant (s)

pid = PID(Ts, kp, ki, kd, tau=tau)

x_setpoint = 0.0   # center of camera axis
tolerance = 0.05   # 5% tolerance

def get_face_x():
    """
    Placeholder: implement with OpenCV.
    Return normalized x position of face in range [-1, 1],
    where 0 = center, -1 = far left, +1 = far right.
    """
    # Example: (face_center_pixel - frame_center_pixel) / frame_center_pixel
    return 0.2  # dummy value for testing

while True:
    x_measured = get_face_x()
    error = x_setpoint - x_measured

    if abs(error) > tolerance:
        # Run PID to compute rotation command
        output = pid.control(x_setpoint, x_measured)

        # Map output to differential motor speeds
        # Positive output → spin right, Negative → spin left
        motor_left.set_output(-output)
        motor_right.set_output(output)
    else:
        # Stop motors when face is centered
        motor_left.set_output(0)
        motor_right.set_output(0)

    time.sleep(Ts)
"""