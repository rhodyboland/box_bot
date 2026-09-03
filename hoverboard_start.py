import Jetson.GPIO as GPIO
import time


ON_SENSE_PIN = 7
SWITCH_PIN = 11

def main():
    # Pin Setup:
    GPIO.setmode(GPIO.BOARD)  # Jetson board numbering scheme
    # set pin as an output pin with optional initial state of HIGH
    GPIO.setup(SWITCH_PIN, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(ON_SENSE_PIN, GPIO.IN)
    print("Starting demo now! Press CTRL+C to exit")
    try:
        while(True):
            print(GPIO.input(ON_SENSE_PIN))
        # if not GPIO.input(ON_SENSE_PIN):
        #     print("Hover was off, turning on")
        #     GPIO.output(SWITCH_PIN, GPIO.HIGH)
        #     time.sleep(0.1)
        #     GPIO.output(SWITCH_PIN, GPIO.LOW)
        # else:
        #     print("Hover was on already, turning off")
        #     GPIO.output(SWITCH_PIN, GPIO.HIGH)
        #     time.sleep(0.1)
        #     GPIO.output(SWITCH_PIN, GPIO.LOW)
        # print(GPIO.input(ON_SENSE_PIN))
    finally:
        GPIO.cleanup()

if __name__ == '__main__':
    main()
