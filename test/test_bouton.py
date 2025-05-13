#!/usr/bin/env python3

import RPi.GPIO as GPIO
import time

BUTTON_PIN = 15

GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

print("Waiting for button press...")

try:
    while True:
        if GPIO.input(BUTTON_PIN) == GPIO.LOW:
            print("Button pressed!")
            time.sleep(0.2)  # Debounce
except KeyboardInterrupt:
    print("Exiting program...")

finally:
    GPIO.cleanup()  # Clean up GPIO settings