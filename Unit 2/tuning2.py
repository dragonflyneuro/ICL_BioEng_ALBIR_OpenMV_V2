from servos import *
from camera import *
import os, time, errno

class PanTuning(object):
    """
    A class for pan servo tracking, calibration and measurement.
    The servo is controlled directly from the detected target position —
    no PID controller is used.
    """

    def __init__(self, thresholds, gain=10):
        """
        Initialise the PanTuning object.

        Args:
            thresholds (list):  Colour detection thresholds.
            gain (float):       Camera gain.
        """
        self.servo = Servo()
        self.servo.soft_reset()
        self.cam = Cam(thresholds, gain)

        self.min_angle       = 0
        self.max_angle       = 0
        self.targetmax_angle = 25
        self.targetmin_angle = -self.targetmax_angle

        self.csv = None

        # Diagnostic colours for bounding boxes
        # One colour per threshold index — matches Red, Blue, Green order
        self.diag_colours = [
            (255,   0,   0),    # Red   — threshold index 0
            (  0,   0, 255),    # Blue  — threshold index 1
            (  0, 255,   0),    # Green — threshold index 2
        ]

        # How often to print diagnostic messages (every N frames)
        self.diag_interval = 10
        self._frame_count  = 0


    def draw_diagnostics(self, img, blobs, rotated_centres, phase=""):
        """
        Draw bounding boxes and crosses on the image for all detected blobs,
        and print a periodic diagnostic message to the serial terminal.

        Args:
            img (image):             OpenMV image object to draw on.
            blobs (list):            List of detected blobs.
            rotated_centres (list):  List of (rx, ry) corrected blob centres.
            phase (str):             Label for the current phase e.g. "Calibrate"
        """
        self._frame_count += 1

        if not blobs:
            if self._frame_count % self.diag_interval == 0:
                print("  [" + phase + "] Frame " + str(self._frame_count) + ": No blobs detected")
            return

        # Use cam.get_blob_colours() — already handles blob[8] correctly
        colours = self.cam.get_blob_colours(blobs)

        for i, blob in enumerate(blobs):
            # Match find_blob() logic: colour code == pow(2, threshold_idx)
            colour_code   = colours[i]
            threshold_idx = 0
            for idx in range(len(self.cam.thresholds)):
                if colour_code == pow(2, idx):
                    threshold_idx = idx
                    break

            # Pick bounding box colour based on threshold index
            box_colour = self.diag_colours[threshold_idx] \
                         if threshold_idx < len(self.diag_colours) \
                         else (255, 255, 255)

            # Draw raw bounding box and corrected centre cross
            img.draw_rectangle(blob.rect)
            rx, ry = rotated_centres[i]
            img.draw_cross((int(rx), int(ry)))

        # All text output goes to terminal only
        if self._frame_count % self.diag_interval == 0:
            print("  [" + phase + "] Frame " + str(self._frame_count) +
                  ": " + str(len(blobs)) + " blob(s) detected")
            for i, blob in enumerate(blobs):
                colour_code   = colours[i]
                threshold_idx = 0
                for idx in range(len(self.cam.thresholds)):
                    if colour_code == pow(2, idx):
                        threshold_idx = idx
                        break
                rx, ry = rotated_centres[i]
                print("    Blob " + str(i) +
                      ": threshold=" + str(threshold_idx) +
                      " raw_cx=" + str(blob.cx) +
                      " cy=" + str(blob.cy) +
                      " rotated_cx=" + str(rx) +
                      " ry=" + str(ry) +
                      " pixels=" + str(blob.pixels))


    def update_pan(self, rotated_cx: float) -> tuple:
        """
        Directly command the pan servo to point at the detected target.

        Args:
            rotated_cx (float): Rotation-corrected blob x coordinate.

        Returns:
            angle_error (float): Angular offset of target from image centre (degrees).
            pan_angle (float):   Commanded pan servo angle (degrees).
        """
        # Pixel offset from image centre
        pixel_error = rotated_cx - self.cam.w_centre

        # Convert pixel offset to angle using horizontal field of view
        # Maximum angle_error = ±h_fov/2 = ±15.75° at image edges
        angle_error = -(pixel_error / sensor.width() * self.cam.h_fov)

        # ✅ Scale angle_error to match the servo's useful pan range
        # angle_error is in camera FOV space (±15.75°)
        # servo operates in pan space (±targetmax_angle = ±25°)
        # scaling factor = targetmax_angle / (h_fov / 2)
        scale = self.targetmax_angle / (self.cam.h_fov / 2)
        pan_angle = angle_error * scale

        print("    pixel_error=" + str(pixel_error) +
              " angle_error="    + str(angle_error) +
              " scale="          + str(scale) +
              " pan_angle="      + str(pan_angle))

        self.servo.set_angle(pan_angle)

        return angle_error, pan_angle


    def measure(self, freq: float) -> None:
        """
        Measures the tracking error and pan angle of the red square target
        for a specified frequency of oscillation.

        Args:
            freq (float): Frequency of oscillation in Hz.
        """
        t_run = int(1000 * 10 / freq)

        self.prepare_csv(freq)

        # --- Calibration phase ---
        print("\n" + "=" * 50)
        print("  MEASURE: Starting at " + str(freq) + " Hz")
        print("=" * 50)
        print("  Phase: Calibration")
        self.calibrate()

        while ((self.min_angle > self.targetmin_angle) or
               (self.max_angle < self.targetmax_angle)):
            print("  Calibration failed.")
            print("  Check colour thresholds and move robot closer to screen.")
            self.calibrate()

        print("  Calibration complete.")
        print("  Min angle: " + str(self.min_angle))
        print("  Max angle: " + str(self.max_angle))

        # Reset pan to centre before measurement
        print("  Resetting pan to centre...")
        self.servo.set_angle(0)
        time.sleep_ms(500)
        self._frame_count = 0

        # Wait for red target
        print("\n  Phase: Waiting for red target...")
        flag = True
        while flag:
            blobs, img, rotated_centres = self.cam.get_blobs(self.servo.pan_pos)
            big_blob = self.cam.get_biggest_blob(blobs)
            self.draw_diagnostics(img, blobs, rotated_centres, phase="WaitRed")

            if big_blob and self.cam.find_blob([big_blob], 0) is not None:
                print("  Red target found — starting measurement.")
                flag = False

        # --- Measurement phase ---
        print("\n  Phase: Measuring")
        print("  Run time (ms), Error (deg), Pan angle (deg)")
        print("  " + "-" * 45)

        t_start = time.ticks_ms()
        t_end   = time.ticks_add(t_start, t_run)
        self._frame_count = 0

        while time.ticks_diff(t_end, time.ticks_ms()) > 0:
            blobs, img, rotated_centres = self.cam.get_blobs(self.servo.pan_pos)
            big_blob = self.cam.get_biggest_blob(blobs)
            self.draw_diagnostics(img, blobs, rotated_centres, phase="Measure")

            if big_blob and self.cam.find_blob([big_blob], 0) is not None:
                blob_idx      = blobs.index(big_blob)
                rotated_cx, _ = rotated_centres[blob_idx]

                # ✅ Direct servo control — no PID
                error, pan_angle = self.update_pan(rotated_cx)

                run_time = time.ticks_diff(time.ticks_ms(), t_start)
                print("  " + str(run_time) + ", " + str(error) + ", " + str(pan_angle))
                self.write_csv([run_time, error, pan_angle])
            else:
                run_time = time.ticks_diff(time.ticks_ms(), t_start)
                if self._frame_count % self.diag_interval == 0:
                    print("  Target lost at t=" + str(run_time) + " ms")

        print("\n  Measurement complete.")
        self.close_csv()


    def calibrate(self) -> None:
        """
        Calibrate the pan range by tracking the blue calibration square
        and recording the min/max pan angles achieved.
        """
        print("\n  Please start the target tracking video.")
        print("  Waiting for blue calibration square...")

        self.max_angle = 0
        self.min_angle = 0
        self.servo.set_angle(0)
        self._frame_count = 0

        t_lost = time.ticks_add(time.ticks_ms(), 3000)

        while time.ticks_diff(t_lost, time.ticks_ms()) > 0:
            blobs, img, rotated_centres = self.cam.get_blobs(self.servo.pan_pos)
            big_blob = self.cam.get_biggest_blob(blobs)
            self.draw_diagnostics(img, blobs, rotated_centres, phase="Calibrate")

            if big_blob and self.cam.find_blob([big_blob], 1) is not None:
                blob_idx      = blobs.index(big_blob)
                rotated_cx, _ = rotated_centres[blob_idx]

                # ✅ Direct servo control — no PID
                error, pan_angle = self.update_pan(rotated_cx)

                if error < 10:
                    if pan_angle < self.min_angle:
                        self.min_angle = pan_angle
                        print("  New min angle: " + str(self.min_angle))
                    if pan_angle > self.max_angle:
                        self.max_angle = pan_angle
                        print("  New max angle: " + str(self.max_angle))

                # Reset lost timer since blob was found
                t_lost = time.ticks_add(time.ticks_ms(), 1500)

        print("  Calibration range: " + str(self.min_angle) + " to " + str(self.max_angle))


    def prepare_csv(self, freq: float) -> None:
        """
        Prepare a new CSV file for tracking data, auto-incrementing
        the filename if one already exists.

        Args:
            freq (float): Frequency in Hz — used in the filename.
        """
        file_n   = 0

        try:
            os.mkdir("./CSV")
        except OSError as e:
            if e.args[0] != errno.EEXIST:
                raise

        filename = "./CSV/Curve" + str(freq) + "Hz_" + str(file_n) + ".csv"
        while True:
            try:
                with open(filename, 'r'):
                    pass
                file_n  += 1
                filename = "./CSV/Curve" + str(freq) + "Hz_" + str(file_n) + ".csv"
            except OSError:
                break

        print("\n  CSV: Opening " + filename)
        self.csv = open(filename, 'w')
        self.csv.write('Time,Error,Angle\n')
        self.csv.flush()


    def write_csv(self, data: tuple) -> None:
        """
        Write a row of tracking data to the CSV file.

        Args:
            data (tuple): Values to write as a comma-separated row.
        """
        if self.csv is None:
            print("  Warning: CSV not open — call prepare_csv() first.")
            return
        self.csv.write(','.join(map(str, data)) + '\n')
        self.csv.flush()


    def close_csv(self) -> None:
        """
        Flush and close the CSV file.
        """
        try:
            self.csv.flush()
            self.csv.close()
        except OSError as e:
            print("  Warning: Could not close CSV: " + str(e))
        print("  CSV: File closed.")# Untitled - By: huait - Sun Aug 2 2026
