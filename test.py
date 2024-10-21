import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)


ON_SENSE_PIN = 4
SWITCH_PIN = 17

GPIO.setup(ON_SENSE_PIN, GPIO.IN)
GPIO.setup(SWITCH_PIN, GPIO.OUT)


if not GPIO.input(ON_SENSE_PIN):
    print("Hover was off, turning on")
    GPIO.output(SWITCH_PIN, GPIO.HIGH)
    time.sleep(0.1)
    GPIO.output(SWITCH_PIN, GPIO.LOW)
else:
    print("Hover was on already, turning off")
    GPIO.output(SWITCH_PIN, GPIO.HIGH)
    time.sleep(0.1)
    GPIO.output(SWITCH_PIN, GPIO.LOW)

GPIO.cleanup()