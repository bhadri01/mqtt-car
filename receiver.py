#!/usr/bin/env python
import asyncio
import sys
import RPi.GPIO as GPIO
from time import sleep
import pika
import json
import threading
import math

gpiochannel1 = [0, 5, 6, 13, 19, 26]
# pin 5 and 6 acceleration
# pin 13 and 19 acceleration
# pin 22 and 26 are the PWM control pins

gpiochannel2 = [17, 27, 22, 10, 9, 11]
# pin 27 and 22 acceleration
# pin 10 and 9 acceleration
# pin 17 and 11 are the PWM control pins

# for board setting
GPIO.setmode(GPIO.BCM)

# setup to pin mode
for i in gpiochannel1:
    GPIO.setup(i, GPIO.OUT)
for i in gpiochannel2:
    GPIO.setup(i, GPIO.OUT)

# rgb in pwm mode in a list
motorDriver1_1 = GPIO.PWM(26, 100)
motorDriver1_2 = GPIO.PWM(0, 100)
motorDriver2_1 = GPIO.PWM(17, 100)
motorDriver2_2 = GPIO.PWM(11, 100)

TRIG = 21
ECHO = 20
MIN_DIST = 0 # Minimum distance to detect an object, in centimeters
MAX_DIST = 15 # Maximum distance to detect an object, in centimeters

GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

def findPercents(inp, mi, ma, v):
    va = (inp - mi) * 100 / (ma - mi)
    if v == 100:
        va = v - va
    if va > 100:
        return 100
    elif va < 0:
        return 0
    else:
        return int(va)


def AccelerationOperation(rightHand):
    try:
        if len(rightHand) > 0:
            x0, x1 = rightHand[0][0], rightHand[12][0]
            y0, y1 = rightHand[0][1], rightHand[12][1]
            x3, x4 = rightHand[3][0], rightHand[4][0]
            acc = findPercents(
                math.hypot(x0-x1, y0-y1), 50, 140, 0)
            # accleration speed start
            motorDriver1_1.start(acc)
            motorDriver1_2.start(acc)
            motorDriver2_1.start(acc)
            motorDriver2_2.start(acc)
            # neutral Acceleration
            if acc > 0:
                angle = abs(math.atan2(y1 - y0, x1 - x0) * 180 / math.pi)
                print(angle)
                if angle < 60:
                    GPIO.output(13, 0)
                    GPIO.output(19, 1)
                    GPIO.output(5, 1)
                    GPIO.output(6, 0)
                    GPIO.output(27, 1)
                    GPIO.output(22, 0)
                    GPIO.output(10, 0)
                    GPIO.output(9, 1)
                elif angle > 120:
                    GPIO.output(13, 1)
                    GPIO.output(19, 0)
                    GPIO.output(5, 0)
                    GPIO.output(6, 1)
                    GPIO.output(27, 0)
                    GPIO.output(22, 1)
                    GPIO.output(10, 1)
                    GPIO.output(9, 0)
                else:
                    if x3 > x4:
                        print("direction back")
                        GPIO.output(13, 0)
                        GPIO.output(19, 1)
                        GPIO.output(5, 0)
                        GPIO.output(6, 1)
                        GPIO.output(27, 0)
                        GPIO.output(22, 1)
                        GPIO.output(10, 0)
                        GPIO.output(9, 1)
                    else:  # forward Acceleration
                        print("direction front")
                        GPIO.output(13, 1)
                        GPIO.output(19, 0)
                        GPIO.output(5, 1)
                        GPIO.output(6, 0)
                        GPIO.output(27, 1)
                        GPIO.output(22, 0)
                        GPIO.output(10, 1)
                        GPIO.output(9, 0)

            else:
                GPIO.output(13, 0)
                GPIO.output(19, 0)
                GPIO.output(5, 0)
                GPIO.output(6, 0)
                GPIO.output(27, 0)
                GPIO.output(22, 0)
                GPIO.output(10, 0)
                GPIO.output(9, 0)
            print("Acceleration:", acc)
        else:
            motorDriver1_1.start(0)
            motorDriver1_2.start(0)
            motorDriver2_1.start(0)
            motorDriver2_2.start(0)
            GPIO.output(13, 0)
            GPIO.output(19, 0)
            GPIO.output(5, 0)
            GPIO.output(6, 0)
            GPIO.output(27, 0)
            GPIO.output(22, 0)
            GPIO.output(10, 0)
            GPIO.output(9, 0)
    except KeyboardInterrupt:
        print("Force exit operation")
        motorDriver1_1.start(0)
        motorDriver1_2.start(0)
        motorDriver2_1.start(0)
        motorDriver2_2.start(0)
        GPIO.output(13, 0)
        GPIO.output(19, 0)
        GPIO.output(5, 0)
        GPIO.output(6, 0)
        GPIO.output(27, 0)
        GPIO.output(22, 0)
        GPIO.output(10, 0)
        GPIO.output(9, 0)
        sys.exit()


def AccessingTheGPIO(handData):
    # print(handData, width, height)
    rightHand = handData["right"]

    # Acceleration threading
    if len(rightHand) > 0:
        rightThread = threading.Thread(
            target=AccelerationOperation, args=(rightHand,)
        )
        # after defineing the thread model we need to start the thread
        rightThread.start()
    else:
        motorDriver1_1.start(0)
        motorDriver1_2.start(0)
        motorDriver2_1.start(0)
        motorDriver2_2.start(0)
        GPIO.output(13, 0)
        GPIO.output(19, 0)
        GPIO.output(5, 0)
        GPIO.output(6, 0)
        GPIO.output(27, 0)
        GPIO.output(22, 0)
        GPIO.output(10, 0)
        GPIO.output(9, 0)
# RabbitMQ receiveing data


def callback(ch, method, properties, body):
    data = json.loads(body)
    landmarks = data
    # print(landmarks)  # Replace this with your own processing code
    GPIOthread = threading.Thread(
        target=AccessingTheGPIO, args=(landmarks,))
    GPIOthread.start()


if __name__ == "__main__":
    cred = pika.PlainCredentials('anish', 'dotmail123')
    connection = pika.BlockingConnection(
        pika.ConnectionParameters(host='rabbitmq.youngstorage.in', port=5672, virtual_host='/', credentials=cred))
    channel = connection.channel()

    channel.queue_declare(queue='hand_gesture_data')

    channel.basic_consume(queue='hand_gesture_data',
                          on_message_callback=callback, auto_ack=True)

    print('Waiting for hand gesture data. To exit press CTRL+C')
    channel.start_consuming()
