# exercise2.py  v0.1.0
from tuning2 import *
from machine import LED

led = LED("LED_BLUE")
led.on()

thresholds = [
      (20, 90, 40, 100, 15, 70), # Red
      (10, 80, 10, 90, -120, -30), # Blue
      (30, 100, -100, -20, 10, 90), # Green
]

tuning = PanTuning(thresholds, gain = 10)

tuning.measure(0.8)
