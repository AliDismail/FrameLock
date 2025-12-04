import time
import distance_maintaining
import centering

def main():
    # --- Define pins and initialize modules ---
    # Example pin definitions (adapt to your hardware)
    left_motor_pins = (16, 17, 18)
    right_motor_pins = (19, 20, 21)
    # camera_id = 0 or Picamera2 setup if using PiCam

    # Create centering and distance controllers
    centering_controller = centering.MotorFaceCentering(
        left_pins = left_motor_pins,
        right_pins = right_motor_pins,
        Ts=0.05
    )

    distance_controller = distance_maintaining.DualMotorDistanceController(
        left_pins = left_motor_pins,
        right_pins = right_motor_pins,
        Ts=0.05,
        setpoint=1.0,
        tolerance=0.01
    )


    # --- Main loop ---
    try:
        while True:
            # Run centering control
            centering_controller.motor_control_centering()

            # Small delay (2 ms)
            time.sleep(0.002)

            # Run distance maintaining control
            distance_controller.motor_control_maintaining()

    except KeyboardInterrupt:
        print("Main loop interrupted. Stopping motors.")
        centering_controller.motor_left.set_output(0)
        centering_controller.motor_right.set_output(0)
        distance_controller.motor_left.set_output(0)
        distance_controller.motor_right.set_output(0)



"""
main:
define the pins of motors, camera,
 while true:
 centering.motor_control_centering()
 delay of 2 ms
 distance_maintaining.motor_control_maintaining()
"""

