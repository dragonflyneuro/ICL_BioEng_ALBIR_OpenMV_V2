from tuning2 import *
from machine import LED

led = LED("LED_BLUE")
led.on()
print("LED on")

thresholds = [
      (44, 70, 30, 87, 22, 75), # Red
      (19, 44, 20, 68, -94, -48), # Blue
      (49, 80, -80, -15, -2, 78), # Green
]
print("Thresholds defined")

# No PID
tuning2 = PanTuning(thresholds, gain=10)
print("PanTuning created")

tuning2.measure(0.5)
print("measure called")
