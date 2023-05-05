import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

TRIG = 16
ECHO = 12
MIN_DIST = 0 # Minimum distance to detect an object, in centimeters
MAX_DIST = 15 # Maximum distance to detect an object, in centimeters

GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

try:
    while True:
        GPIO.output(TRIG, False)
        print("Waiting For Sensor To Settle")
        time.sleep(2)

        GPIO.output(TRIG, True)
        time.sleep(0.00001)
        GPIO.output(TRIG, False)

        while GPIO.input(ECHO) == 0:
            pulse_start = time.time()

        while GPIO.input(ECHO) == 1:
            pulse_end = time.time()

        pulse_duration = pulse_end - pulse_start

        distance = pulse_duration * 17150

        distance = round(distance, 2)

        if distance > MIN_DIST and distance < MAX_DIST:
            print(f"True {distance}")
            # Add code here to indicate the presence of an object (e.g. turn on a buzzer, activate a motor, etc.)
        else:
            print(f"False {distance}")
            # Add code here to indicate the absence of an object (e.g. turn off a buzzer, deactivate a motor, etc.)

        time.sleep(0.5) # delay for half a second between readings

except KeyboardInterrupt:
    # Clean up the GPIO pins when the program is interrupted
    GPIO.cleanup()
