# servos.py  v0.1.1
from machine import Pin, PWM
import time

class Servo:
    """
    A class responsible for controlling servos via the OpenMV board.
    """

    def __init__(self):
        """
        Initialise the servo object and sets the tuning coefficients.
        """
        # Servo tuning coefficients; EDIT these values as required.
        self.pan_angle_corr = -8
        self.half_pan_us = 1000
        # Zero-offset trim: shifts each wheel's neutral pulse so it
        # actually STOPS at speed 0. Fixes creeping at rest.
        self.left_zero = 0
        self.right_zero = 0

        # Wheel servo pulse characteristics: 1500 us = stop, +-80 us is
        # deadband, +-80 to +-588 us is the linear range. set_speed maps
        # commands onto that band.
        self.wheel_deadband_us = 80
        self.wheel_max_us      = 588

        # Differential rate trim, applied in set_drive. Positive values
        # compensate for the left wheel running fast. Measured by
        # wheelcal.py and saved to wheeltrim.txt, loaded below.
        self.drive_trim = 0.0
        try:
            with open("wheeltrim.txt") as _f:
                self.drive_trim = float(_f.read().strip())
                print("wheel trim loaded: %+.4f" % self.drive_trim)
        except (OSError, ValueError):
            pass

        # Define servo pin names for the servo shield. EDIT these values as required.
        # NOTE: P9/P10 share one FlexPWM submodule (and P10 doubles as the camera
        # frame-sync line), so they are NOT independent - avoid pairing two servos
        # on P9+P10. Pins below are spread across separate submodules instead.
        self.pan_id = 'P9'
        self.left_id = 'P8'
        self.right_id = 'P7'

        # Set up servo angle limits
        self.degrees = 180
        self.min_deg = -self.degrees/2
        self.max_deg = self.degrees/2

        self.curr_l_speed = 0
        self.curr_r_speed = 0
        self.pan_pos = 0

        self.freq = 50

        # Pulse width range (microseconds) for the PWM signal.
        self.min_us = 1500 - self.half_pan_us
        self.max_us = 1500 + self.half_pan_us
        self.mid_us = (self.min_us + self.max_us) / 2
        self.span_us = (self.max_us - self.min_us)

        # Initialise a PWM channel per servo (newer servo shields drive servos
        # directly over GPIO rather than via a PCA9685 I2C chip).
        self.pan_pwm = PWM(Pin(self.pan_id), freq=self.freq)
        self.left_pwm = PWM(Pin(self.left_id), freq=self.freq)
        self.right_pwm = PWM(Pin(self.right_id), freq=self.freq)

    def set_differential_drive(self, speed: float, bias: float) -> None:
        """
        Set speeds for a differential drive robot using a speed coefficient and a steering bias.

        Args:
            speed_coeff (float): Overall speed coefficient of the robot (0 to 1).
            steering_bias (float): Steering bias for the robot (-1 to 1).
        """
        # Validate input ranges
        speed = max(min(speed, 1), 0)
        bias = max(min(bias, 1), -1)

        # Calculate individual wheel speeds
        left_speed = speed * (1 - bias)
        right_speed = speed * (1 + bias)

        # Normalize speeds if they exceed 1
        max_speed = max(abs(left_speed), abs(right_speed))
        if max_speed > 1:
            left_speed /= max_speed
            right_speed /= max_speed

        # Set the speeds
        self.set_speed(left_speed, right_speed)


    def set_drive(self, speed: float, turn: float) -> None:
        """
        Arcade-style differential drive: additive mixing, with reverse.

        Unlike set_differential_drive this accepts negative speed and can
        turn on the spot (speed 0, turn 1 gives one wheel each way).

        Args:
            speed (float): Forward (+) / reverse (-) rate, -1 to 1.
            turn (float):  Turn rate, -1 to 1. Works at any speed.
        """
        speed = max(min(speed, 1), -1)
        turn = max(min(turn, 1), -1)

        # trim scales with speed (it is a rate error); turn is added after
        left = speed * (1.0 - self.drive_trim) - turn
        right = speed * (1.0 + self.drive_trim) + turn

        # Scale back if either wheel saturates, preserving the turn/speed ratio
        peak = max(abs(left), abs(right))
        if peak > 1:
            left /= peak
            right /= peak

        self.set_speed(left, right)


    def set_angle(self, angle: float) -> float:
        """
        Set the pan servo to a specific angle (in degrees).

        Args:
            angle (float): Desired angle (deg) for the camera pan servo.

        Returns:
            float: Corrected angle (deg) of the camera pan servo.
        """
        # Correct for off centre angle
        self.pan_pos = angle

        angle = self.pan_angle_corr + angle

        # Constraint angle to limits
        angle = max(min(angle, self.max_deg), self.min_deg)

        # Compute pulse width (us) for the PWM signal
        pulse_us = self.mid_us + ( self.span_us * (angle / self.degrees) )

        # Set duty and send PWM signal
        self.pan_pwm.duty_ns(int(pulse_us * 1000))

        return self.pan_pos


    def _speed_to_us(self, s: float) -> float:
        """Map a speed command (-1..1) to a pulse offset from centre (us).

        Skips the deadband, so any non-zero command actually moves the
        wheel, and full scale lands on wheel_max_us.

        Args:
            s (float): Speed command, -1 to 1. Exactly 0 means stop.

        Returns:
            float: Pulse offset from mid_us, in microseconds.
        """
        if abs(s) < 1e-3:
            return 0.0
        mag = min(abs(s), 1.0)
        off = (self.wheel_deadband_us
               + mag * (self.wheel_max_us - self.wheel_deadband_us))
        return off if s > 0 else -off


    def set_speed(self, l_speed: float, r_speed: float) -> None:
        """
        Control the speed of the left and right wheel servos.

        Args:
            l_speed (float): Speed to set left wheel servo to (-1~1).\n
            r_speed (float): Speed to set right wheel servo to (-1~1).
        """

        # Constraint speeds to limits
        l_speed = max(min(l_speed, 1), -1)
        r_speed = max(min(r_speed, 1), -1)
        self.curr_l_speed = l_speed
        self.curr_r_speed = r_speed

        # Right is negated because the servos are mirror-mounted.
        l_us = self.mid_us + self._speed_to_us(self.curr_l_speed + self.left_zero)
        r_us = self.mid_us - self._speed_to_us(self.curr_r_speed + self.right_zero)

        # Ensure pulse width values are within the valid range
        l_us = max(min(l_us, self.max_us), self.min_us)
        r_us = max(min(r_us, self.max_us), self.min_us)

        # Set duty and send PWM signal
        self.left_pwm.duty_ns(int(l_us * 1000))
        self.right_pwm.duty_ns(int(r_us * 1000))

        return

    def release(self, pwm: PWM) -> None:
        """
        Simple servo release method

        Args:
            pwm (PWM): Servo PWM channel to reset (e.g. self.pan_pwm).
        """
        pwm.duty_ns(0)


    def release_all(self) -> None:
        """
        Release all servos.
        """
        for pwm in (self.pan_pwm, self.left_pwm, self.right_pwm):
            self.release(pwm)


    def soft_reset(self) -> None:
        """
        Method to reset the servos to default and print a delay prompt.
        """
        # Reset all servo shield pins
        self.release_all()

        # Reset pan to centre
        self.set_angle(0)

        # Print delay prompt
        for i in range(3, 0, -1):
            print(f"{i} seconds remaining.")
            time.sleep_ms(1000)

        print("___Running Code___")


if __name__ == "__main__":
    servo = Servo()
    servo.soft_reset()

    # Servo speed test
    print('\n0,0')
    servo.set_speed(0,0)
    time.sleep_ms(1000)

    print('\n0.1,0.1')
    servo.set_speed(0.2,0)
    time.sleep_ms(1000)

    print('\n-0.1, -0.1')
    servo.set_speed(-0.2,0)
    time.sleep_ms(1000)

    print('\n0.2,0.2')
    servo.set_speed(0,0.2)
    time.sleep_ms(1000)

    print('\n-0.2, -0.2')
    servo.set_speed(0, -0.2)
    time.sleep_ms(1000)

    print('\n0,0')
    servo.set_speed(0,0)
    time.sleep_ms(2000)

    # Pan servo angle test
    print('\n-45deg')
    servo.set_speed(0, 0)
    servo.set_angle(-45)
    time.sleep_ms(2000)

    print('\n45deg')
    servo.set_angle(45)
    time.sleep_ms(2000)

    print('\n-90deg')
    servo.set_angle(-90)
    time.sleep_ms(2000)

    print('\n90deg')
    servo.set_angle(90)
    time.sleep_ms(2000)

    servo.soft_reset()
