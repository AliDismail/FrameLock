import time
import numpy as np
from gpiozero_extended import Motor, PID

class MotorController:
    def __init__(self,
                 tsample=0.01,
                 tau=0.1,
                 kp=0.15, ki=0.35, kd=0.01, taupid=0.01,
                 enable1=16, pwm1=17, pwm2=18,
                 encoder1=24, encoder2=25, encoderppr=300.8):
        """
        Initialize motor controller with PID and motor setup.
        """
        # Sampling and filter parameters
        self.tsample = tsample
        self.tau = tau

        # PID controller
        self.pid = PID(tsample, kp, ki, kd, umin=0, tau=taupid)

        # Motor setup
        self.motor = Motor(enable1=enable1, pwm1=pwm1, pwm2=pwm2,
                           encoder1=encoder1, encoder2=encoder2,
                           encoderppr=encoderppr)
        self.motor.reset_angle()

        # State variables
        self.wfprev = 0
        self.thetaprev = 0
        self.tprev = 0

    def run_speed_control(self, wsp=20, duration=2.0):
        """
        Run closed-loop PID control to maintain motor speed at wsp (rad/s).
        """
        print(f"Running closed-loop PID for {duration} seconds at setpoint {wsp} rad/s ...")
        tstart = time.perf_counter()
        tcurr = 0

        while tcurr <= duration:
            time.sleep(self.tsample)
            tcurr = time.perf_counter() - tstart

            # Read encoder angle
            thetacurr = self.motor.get_angle()

            # Calculate speed (rad/s)
            wcurr = np.pi/180 * (thetacurr - self.thetaprev) / (tcurr - self.tprev if tcurr > self.tprev else self.tsample)

            # Low-pass filter
            wfcurr = self.tau/(self.tau+self.tsample)*self.wfprev + self.tsample/(self.tau+self.tsample)*wcurr
            self.wfprev = wfcurr

            # PID control output
            ucurr = self.pid.control(wsp, wfcurr)

            # Apply to motor
            self.motor.set_output(ucurr)

            # Update previous values
            self.thetaprev = thetacurr
            self.tprev = tcurr

        print("Done.")
        self.motor.set_output(0, brake=True)

    def close(self):
        """
        Release motor resources.
        """
        del self.motor


# ---- USAGE EXAMPLE ----
if __name__ == "__main__":
    controller = MotorController()
    controller.run_speed_control(wsp=20, duration=2.0)  # Maintain 20 rad/s for 2 seconds
    controller.close()


"""
# Importing modules and classes
import time
import numpy as np
from gpiozero_extended import Motor, PID

# Setting general parameters
tstop = 2  # Execution duration (s)
tsample = 0.01  # Sampling period (s)
wsp = 20  # Motor speed set point (rad/s)
tau = 0.1  # Speed low-pass filter response time (s)

# Creating PID controller object
kp = 0.15
ki = 0.35
kd = 0.01
taupid=0.01
pid = PID(tsample, kp, ki, kd, umin=0, tau=taupid)

# Creating motor object using GPIO pins 16, 17, and 18
# (using SN754410 quadruple half-H driver chip)
# Integrated encoder on GPIO pins 24 and 25.
mymotor = Motor(
    enable1=16, pwm1=17, pwm2=18,
    encoder1=24, encoder2=25, encoderppr=300.8)
mymotor.reset_angle()

# Initializing previous and current values
ucurr = 0  # x[n] (step input)
wfprev = 0  # y[n-1]
wfcurr = 0  # y[n]

# Initializing variables and starting clock
thetaprev = 0
tprev = 0
tcurr = 0
tstart = time.perf_counter()

# Running execution loop
print('Running code for', tstop, 'seconds ...')
while tcurr <= tstop:
    # Pausing for `tsample` to give CPU time to process encoder signal
    time.sleep(tsample)
    # Getting current time (s)
    tcurr = time.perf_counter() - tstart
    # Getting motor shaft angular position: I/O (data in)
    thetacurr = mymotor.get_angle()
    # Calculating motor speed (rad/s)
    wcurr = np.pi/180 * (thetacurr-thetaprev)/(tcurr-tprev)
    # Filtering motor speed signal
    wfcurr = tau/(tau+tsample)*wfprev + tsample/(tau+tsample)*wcurr
    wfprev = wfcurr
    # Calculating closed-loop output
    ucurr = pid.control(wsp, wfcurr)
    # Assigning motor output: I/O (data out)
    mymotor.set_output(ucurr)
    # Updating previous values
    thetaprev = thetacurr
    tprev = tcurr

print('Done.')
# Stopping motor and releasing GPIO pins
mymotor.set_output(0, brake=True)
del mymotor
"""