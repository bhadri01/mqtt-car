import paho.mqtt.client as mqtt
import json
import sys
import RPi.GPIO as GPIO
import time
import threading
import math

gpiochannel1 = [0, 5, 6, 13, 19, 26]
# pin 5 and 6 acceleration
# pin 13 and 19 acceleration
# pin 22 and 26 are the PWM control pins

# for board setting
GPIO.setmode(GPIO.BCM)

# setup to pin mode
for i in gpiochannel1:
    GPIO.setup(i, GPIO.OUT)

# rgb in pwm mode in a list
motorDriver1_1 = GPIO.PWM(26, 100)
motorDriver1_2 = GPIO.PWM(0, 100)

# ultrasonic points
# front
FTRIG = 21
FECHO = 20
# back
BTRIG = 16
BECHO = 12
MIN_DIST = 0  # Minimum distance to detect an object, in centimeters
MAX_DIST = 15  # Maximum distance to detect an object, in centimeters
UltFront = False
UltBack = False
UltBackdata = 0
UltFrontdata = 0

GPIO.setup(FTRIG, GPIO.OUT)
GPIO.setup(FECHO, GPIO.IN)

GPIO.setup(BTRIG, GPIO.OUT)
GPIO.setup(BECHO, GPIO.IN)


def CAR(data):
    if len(data) > 0:
        acc = data[0]
        angle = data[1]
        dire = data[2]
        motorDriver1_1.start(acc)
        motorDriver1_2.start(acc)

        if angle < 60:
            GPIO.output(13, 0)
            GPIO.output(19, 1)
            GPIO.output(5, 1)
            GPIO.output(6, 0)
        elif angle > 120:
            GPIO.output(13, 1)
            GPIO.output(19, 0)
            GPIO.output(5, 0)
            GPIO.output(6, 1)
        else:
            if dire == 1:
                if UltFront:
                    motorDriver1_1.start(0)
                    motorDriver1_2.start(0)
                    GPIO.output(13, 0)
                    GPIO.output(19, 0)
                    GPIO.output(5, 0)
                    GPIO.output(6, 0)
                else:
                    GPIO.output(13, 1)
                    GPIO.output(19, 0)
                    GPIO.output(5, 1)
                    GPIO.output(6, 0)
            elif dire == 0:
                if UltBack:
                    motorDriver1_1.start(0)
                    motorDriver1_2.start(0)
                    GPIO.output(13, 0)
                    GPIO.output(19, 0)
                    GPIO.output(5, 0)
                    GPIO.output(6, 0)
                else:
                    GPIO.output(13, 0)
                    GPIO.output(19, 1)
                    GPIO.output(5, 0)
                    GPIO.output(6, 1)

    else:
        motorDriver1_1.start(0)
        motorDriver1_2.start(0)
        GPIO.output(13, 0)
        GPIO.output(19, 0)
        GPIO.output(5, 0)
        GPIO.output(6, 0)


def ObjectDetectFront():
    try:
        while True:
            GPIO.output(FTRIG, False)
            time.sleep(0.0001)
            GPIO.output(FTRIG, True)
            time.sleep(0.00001)
            GPIO.output(FTRIG, False)

            while GPIO.input(FECHO) == 0:
                pulse_start = time.time()

            while GPIO.input(FECHO) == 1:
                pulse_end = time.time()

            pulse_duration = pulse_end - pulse_start

            distance = pulse_duration * 17150

            distance = round(distance, 2)

            global UltFront
            global UltFrontdata

            UltFrontdata = distance

            if distance > MIN_DIST and distance < MAX_DIST:
                # print(f"True {distance}")
                UltFront = True
                # Add code here to indicate the presence of an object (e.g. turn on a buzzer, activate a motor, etc.)
            else:
                # print(f"False {distance}")
                UltFront = False
                # Add code here to indicate the absence of an object (e.g. turn off a buzzer, deactivate a motor, etc.)

    except KeyboardInterrupt:
        # Clean up the GPIO pins when the program is interrupted
        GPIO.cleanup()

# ultrasonic function


def ObjectDetectBack():
    try:
        while True:
            GPIO.output(BTRIG, False)
            time.sleep(0.0001)
            GPIO.output(BTRIG, True)
            time.sleep(0.00001)
            GPIO.output(BTRIG, False)

            while GPIO.input(BECHO) == 0:
                pulse_start = time.time()

            while GPIO.input(BECHO) == 1:
                pulse_end = time.time()

            pulse_duration = pulse_end - pulse_start

            distance = pulse_duration * 17150

            distance = round(distance, 2)

            global UltBack
            global UltBackdata
            UltBackdata = distance

            if distance > MIN_DIST and distance < MAX_DIST:
                # print(f"True {distance}")
                UltBack = True
                # Add code here to indicate the presence of an object (e.g. turn on a buzzer, activate a motor, etc.)
            else:
                # print(f"False {distance}")
                UltBack = False
                # Add code here to indicate the absence of an object (e.g. turn off a buzzer, deactivate a motor, etc.)

    except KeyboardInterrupt:
        # Clean up the GPIO pins when the program is interrupted
        GPIO.cleanup()


broker_address = "192.168.137.1"  # Replace with your MQTT broker address
topic_receive = "car_receive"
topic_send = "car_send"


def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe(topic_receive)


def on_message(client, userdata, message):
    str_data = message.payload.decode('utf-8')
    str_data = json.loads(str_data)
    CAR(str_data)
    str_data.insert(0,UltBackdata)
    str_data.insert(0,UltBack)
    str_data.insert(0,UltFrontdata)
    str_data.insert(0,UltFront)
    print(str_data)
    client.publish(topic_send, json.dumps(str_data))


client = mqtt.Client()
# Set username and password
client.username_pw_set("anish", "dotmail123")
client.on_connect = on_connect
client.on_message = on_message
client.connect(broker_address)

# this threading part for to detect Front ultrasonic
ultrasonicF = threading.Thread(target=ObjectDetectFront)
ultrasonicF.start()

# this threading part for to detect Back ultrasonic
ultrasonicB = threading.Thread(target=ObjectDetectBack)
ultrasonicB.start()

client.loop_forever()
