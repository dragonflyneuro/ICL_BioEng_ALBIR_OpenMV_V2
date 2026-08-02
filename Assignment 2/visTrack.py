"""
Simple Visual Tracking — MVBot
==============================
The pan servo tracks the target by keeping it centred in the camera view.
The robot body then steers to align itself with where the pan is pointing.
Drive speed is proportional to target width — robot stops at equilibrium distance.

Behaviour:
    - Pan servo centres the target in the camera image
    - Robot body steers left/right proportional to periscope deviation
    - Robot drives forward if target is too narrow  (too far)
    - Robot reverses  if target is too wide   (too close)
    - Robot stops at equilibrium target width
    - Robot stops if target is lost
"""

from servos import *
from camera import *

# ============================================================
# USER SETTINGS
# ============================================================
GAIN              = 10      # Camera gain — match to your lighting

MAX_DRIVE_SPEED   = 0.8     # Maximum forward/reverse drive speed (0 to 1)

# Pan scaling — maps camera FOV angle error to pan servo movement
# Increase for faster pan response, decrease for smoother
PAN_SCALE         = 0.3

# Body steering scale:
# 1.0 = full steering (±1) when pan is at maximum angle (±90°)
# 0.5 = half steering at maximum pan angle
BODY_SCALE        = 1.0

# Target width equilibrium — robot stops when blob width equals this (pixels)
# QVGA image is 320px wide — so 50px = ~15% of image width
# Run the script, place robot at desired stopping distance,
# and read the w= value from the serial terminal to set this
TARGET_WIDTH_PX   = 100

# Colour thresholds — edit to match your target
thresholds = [
    (19, 44, 20, 68, -94, -48),   # Target colour
]
# ============================================================


# --- Initialise camera and servos ---
cam   = Cam(thresholds, gain=GAIN)
servo = Servo()
servo.soft_reset()

lost_count = 0

print("Visual tracking started.")
print("Target width equilibrium : " + str(TARGET_WIDTH_PX) + " px")
print("Max drive speed          : " + str(MAX_DRIVE_SPEED))
print("Pan scale                : " + str(PAN_SCALE))
print("Body scale               : " + str(BODY_SCALE))
print("Servo degrees            : " + str(servo.degrees))

while True:

    # --- Get image and find biggest blob ---
    blobs, img, rotated_centres = cam.get_blobs(servo.pan_pos)
    big_blob = cam.get_biggest_blob(blobs)

    if big_blob:
        lost_count = 0

        # Get rotation-corrected blob centre
        blob_idx    = blobs.index(big_blob)
        rx, ry      = rotated_centres[blob_idx]

        # --- Pan servo: centre the blob in the image ---
        # angle_error > 0 → blob is left of centre  → pan left
        # angle_error < 0 → blob is right of centre → pan right
        pixel_error = rx - cam.w_centre
        angle_error = -(pixel_error / sensor.width() * cam.h_fov)
        pan_angle   = servo.pan_pos + angle_error * PAN_SCALE
        servo.set_angle(pan_angle)

        # --- Drive speed: proportional to target width error ---
        # blob width < TARGET_WIDTH_PX → too far   → drive forward
        # blob width = TARGET_WIDTH_PX → just right → stop
        # blob width > TARGET_WIDTH_PX → too close  → reverse
        size_error  = 1 - (big_blob.w / TARGET_WIDTH_PX)
        drive_speed = MAX_DRIVE_SPEED * size_error
        drive_speed = max(min(drive_speed, MAX_DRIVE_SPEED), -MAX_DRIVE_SPEED)

        # --- Body steering: proportional to periscope deviation from neutral ---
        # pan_pos =   0° → steering =  0  → drive straight
        # pan_pos = +90° → steering = +1  → full right turn
        # pan_pos = -90° → steering = -1  → full left turn
        # Normalised by servo.degrees (180°) so scale is always ±1 at max pan
        steering = (servo.pan_pos / servo.degrees) * 2 * BODY_SCALE
        steering = max(min(steering, 1), -1)    # clamp to ±1
        servo.set_differential_drive(drive_speed, -steering)

        # Draw blob on image
        img.draw_rectangle(big_blob.rect)
        img.draw_cross((int(rx), int(ry)))

        print("pan="    + str(round(servo.pan_pos,  1)) +
              " err="   + str(round(angle_error,    1)) +
              " w="     + str(big_blob.w)               +
              " speed=" + str(round(drive_speed,    3)) +
              " steer=" + str(round(steering,       3)))

    else:
        # --- Target lost: stop and wait ---
        lost_count += 1
        servo.set_differential_drive(0, 0)
        print("Target lost (" + str(lost_count) + " frames)")
