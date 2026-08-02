from tuning import *
from machine import LED

led = LED("LED_BLUE")
led.on()

thresholds = [
      (50, 60, 50, 75, 40, 60), # Red
      (22, 35, 50, 70, -100, -60), # Blue
      (70, 90, -80, -60, 35, 70), # Green
]

tuning = PanTuning(thresholds, gain = 5, p=0.2, i=0, d=0.005)

tuning.measure(0.5)
