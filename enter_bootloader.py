import RPi.GPIO as GPIO
import time

EN = 17      # Reset
GPIO0 = 27   # Boot

GPIO.setmode(GPIO.BCM)
GPIO.setup(EN, GPIO.OUT)
GPIO.setup(GPIO0, GPIO.OUT)

# Put ESP32 into bootloader mode
GPIO.output(GPIO0, GPIO.LOW)
GPIO.output(EN, GPIO.LOW)
time.sleep(0.1)
GPIO.output(EN, GPIO.HIGH)
time.sleep(0.1)
# Keep GPIO0 low during upload
time.sleep(2)

# Optionally release boot mode after
GPIO.output(GPIO0, GPIO.HIGH)

GPIO.cleanup()
