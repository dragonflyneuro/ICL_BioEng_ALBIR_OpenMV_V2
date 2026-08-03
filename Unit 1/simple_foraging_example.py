from servos import *
from camera import *

servo = Servo()
servo.soft_reset()

thresholds = [
              (22, 86, -28, 0, -15, 30),    # Green
              (45, 60, -10, 10, -45, -25),  # Blue
              (55, 70, 0, 20, 20, 40)       # Orange
                      ]
camera = Cam(thresholds)

step = 0
going = 0
lost_timer = 0
while True:
    lost_timer += 1
    correct_blob_found = 0
    (blobs, img, rotated_centres) = camera.get_blobs_bottom()
    found_idx = camera.find_blob(blobs, step)
    if found_idx:
        if going == 0:
            going = 1
        leftRight = 1.3*(camera.w_centre-blobs[found_idx].cx)/camera.w_centre
        servo.set_differential_drive(0.13, leftRight)
        lost_timer = 0
        break
    elif lost_timer > 10:
        if going == 1:
            step += 1
        going = 0
        servo.set_speed(0.1, -0.1)
