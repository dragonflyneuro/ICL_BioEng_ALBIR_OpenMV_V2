from machine import Pin, PWM
import time

for name in ('P9', 'P7'):
    print("Testing", name)
    p = PWM(Pin(name), freq=50)
    p.duty_ns(1500000)  # ~centre
    time.sleep(1)
    p.duty_ns(700000)   # one extreme
    time.sleep(1)
    p.duty_ns(2300000)  # other extreme
    time.sleep(1)
    p.deinit()
    time.sleep_ms(500)
