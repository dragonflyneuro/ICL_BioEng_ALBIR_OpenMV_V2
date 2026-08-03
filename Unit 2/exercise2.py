from tuning import *
from machine import LED

led = LED("LED_BLUE")
led.on()

thresholds = [
      (44, 70, 30, 87, 22, 75), # Red
      (19, 44, 20, 68, -94, -48), # Blue
      (49, 80, -80, -15, -2, 78), # Green
]

tuning = PanTuning(thresholds, gain = 10, p=1, i=0.05, d=0.05, imax=0)

tuning.measure(0.8)
