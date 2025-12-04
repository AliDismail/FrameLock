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
                 min_torque=0.4, startup_boost_time=0.2):
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
