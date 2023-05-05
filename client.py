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

speed = int(input("speed:"))
motorDriver1_1.start(speed)
motorDriver1_2.start(speed)
motorDriver2_1.start(speed)
motorDriver2_2.start(speed)

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
    if len(rightHand) > 0:
        acc = findPercents(math.hypot(
            rightHand[4][0]-rightHand[8][0], rightHand[4][1]-rightHand[8][1]), 20, 100, 0)
        # accleration speed start
        Acceleration.start(acc)
        # neutral Acceleration
        if acc > 0:
            if rightHand[12][1] < rightHand[11][1] and acc > 0:
                GPIO.output(5, 0)
                GPIO.output(6, 1)
            else:  # forward Acceleration
                GPIO.output(5, 1)
                GPIO.output(6, 0)
        else:
            GPIO.output(5, 0)
            GPIO.output(6, 0)
        print("Acceleration:", acc)
    else:
        Acceleration.start(0)
        GPIO.output(5, 0)
        GPIO.output(6, 0)


def SteeringOperation(leftHand):
    if len(leftHand) > 0:
        stee = findPercents(math.hypot(
            leftHand[4][0]-leftHand[8][0], leftHand[4][1]-leftHand[8][1]), 20, 100, 0)
        # Steering speed start
        Steering.start(stee)
        if stee > 0:
            if leftHand[4][0] < leftHand[8][0]:
                GPIO.output(13, 0)
                GPIO.output(19, 1)
            elif leftHand[4][0] > leftHand[8][0]:
                GPIO.output(13, 1)
                GPIO.output(19, 0)
        else:
            GPIO.output(13, 0)
            GPIO.output(19, 0)
        print("Steering:", stee)
    else:
        Steering.start(0)
        GPIO.output(13, 0)
        GPIO.output(19, 0)


def AccessingTheGPIO(handData):
    # print(handData, width, height)
    rightHand = handData["right"]
    leftHand = handData["left"]

    # Acceleration threading
    if len(rightHand) > 0:
        rightThread = threading.Thread(
            target=AccelerationOperation, args=(rightHand,)
        )
        # after defineing the thread model we need to start the thread
        rightThread.start()
    else:
        Acceleration.start(0)
        GPIO.output(5, 0)
        GPIO.output(6, 0)

    # Steering wheels threading
    if len(leftHand) > 0:
        leftThread = threading.Thread(
            target=SteeringOperation, args=(leftHand,)
        )
        # after defineing the thread model we need to start the thread
        leftThread.start()
    else:
        Steering.start(0)
        GPIO.output(13, 0)
        GPIO.output(19, 0)

#RabbitMQ receiveing data
def callback(ch, method, properties, body):
    data = json.loads(body)
    landmarks = data
    print(landmarks)  # Replace this with your own processing code
    GPIOthread = threading.Thread(
                target=AccessingTheGPIO, args=(landmarks,))
    GPIOthread.start()

if __name__ == "__main__":
    cred = pika.PlainCredentials('anish', 'dotmail123')
    connection = pika.BlockingConnection(
        pika.ConnectionParameters(host='172.19.0.2', port=5672, virtual_host='/', credentials=cred))
    channel = connection.channel()

    channel.queue_declare(queue='hand_gesture_data')

    channel.basic_consume(queue='hand_gesture_data',
                          on_message_callback=callback, auto_ack=True)

    print('Waiting for hand gesture data. To exit press CTRL+C')
    channel.start_consuming()
