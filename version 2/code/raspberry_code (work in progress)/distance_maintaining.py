from gpiozero_extended import Motor, PID
import time
from face_distance_tracker import FaceDistanceTracker

class DualMotorDistanceController:
    def __init__(self,
                 left_pins,
                 right_pins,
                 Ts=0.05,
                 kp=0.5, ki=0.1, kd=0.05, tau=0.1,
                 setpoint=1.0, tolerance=1e-3,
                 min_torque=0.5, startup_boost_time=0.1):
        """
        Initialize two motors and PID controller for distance control.
        """
        # Motors
        self.motor_left = Motor(enable1=left_pins[0], pwm1=left_pins[1], pwm2=left_pins[2])
        self.motor_right = Motor(enable1=right_pins[0], pwm1=right_pins[1], pwm2=right_pins[2])

        # PID controller
        self.Ts = Ts
        self.pid = PID(Ts, kp, ki, kd, tau=tau)

        # Target distance and tolerance
        self.setpoint = setpoint
        self.tolerance = tolerance

        # Torque constraints
        self.min_torque = min_torque
        self.startup_boost_time = startup_boost_time

        # Vision tracker
        self.tracker = FaceDistanceTracker()

    def get_camera_distance(self):
        """
        Use FaceDistanceTracker to measure distance in meters.
        Returns the closest face distance, or None if no face.
        """
        frame = self.tracker.picam2.capture_array()
        self.tracker.process_frame(frame)
        distances = self.tracker.get_face_distance(frame)

        if distances:
            # Pick the closest face (largest bounding box height)
            closest = min(distances, key=lambda d: d[1])  # d[1] = distance_m
            name, distance_m, state = closest
            return distance_m
        else:
            return None

    def motor_control_maintaining(self):
        """
        Run dual motor control until error is within tolerance.
        """
        try:
            start_time = time.perf_counter()

            while True:
                distance = self.get_camera_distance()
                if distance is None:
                    print("No face detected. Stopping motors.")
                    self.motor_left.set_output(0)
                    self.motor_right.set_output(0)
                    break

                error = self.setpoint - distance

                if abs(error) > self.tolerance:
                    # PID control output
                    output = self.pid.control(self.setpoint, distance)

                    # --- TORQUE CONSTRAINTS ---
                    elapsed = time.perf_counter() - start_time

                    if elapsed < self.startup_boost_time:
                        # Startup boost: max output
                        output = 1.0
                    else:
                        # Enforce minimum torque floor
                        if abs(output) < self.min_torque:
                            output = self.min_torque * (1 if output >= 0 else -1)

                    # Apply to both motors (forward/backward together)
                    self.motor_left.set_output(output)
                    self.motor_right.set_output(output)

                else:
                    # Stop motors when centered
                    self.motor_left.set_output(0)
                    self.motor_right.set_output(0)
                    print(f"Target distance {self.setpoint:.2f} m reached. Stopping motors.")
                    break

                time.sleep(self.Ts)
        finally:
            # Ensure motors are stopped on exit
            self.motor_left.set_output(0)
            self.motor_right.set_output(0)
            self.tracker.cleanup()


"""
from gpiozero_extended import Motor, PID
import time

motor = Motor(enable1=16, pwm1=17, pwm2=18)

Ts = 0.05   # 50 ms loop
kp = 0.5    # proportional gain
ki = 0.1    # integral gain
kd = 0.05   # derivative gain
tau = 0.1   # low-pass filter time constant (s) f = 1/(2*pi*tau) which in this case f = 1.59 Hz

pid = PID(Ts, kp, ki, kd, tau=tau)

setpoint = 1.0  # desired distance in meters

while True:
    distance = get_camera_distance()  # <-- your OpenCV function
    output = pid.control(setpoint, distance)
    motor.set_output(output)
    time.sleep(Ts)
"""