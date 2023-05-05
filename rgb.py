#!/usr/bin/env python
import asyncio
import sys
import RPi.GPIO as GPIO
from time import sleep
import pika
import json
import threading
import math

channel = [16, 20, 21]

# for board setting
GPIO.setmode(GPIO.BCM)

# setup to pin mode
for i in channel:
    GPIO.setup(i, GPIO.OUT)


# rgb in pwm mode in a list
rgbcolor = [GPIO.PWM(c, 100) for c in channel]

# start the birghtness with 100
for i in range(len(rgbcolor)):
    rgbcolor[i].start(100)


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
            # rgb x and y axis point
            x0, y0 = rightHand[0][0], rightHand[0][1]
            rx, ry = rightHand[4][0], rightHand[4][1]
            gx, gy = rightHand[8][0], rightHand[8][1]
            bx, by = rightHand[12][0], rightHand[12][1]

            # canculate the length between the bottom to the each point
            Rlen = [findPercents(math.hypot(rx - x0, ry - y0), 155, 185, 0),
                    findPercents(math.hypot(rx - x0, ry - y0), 155, 185, 100)]
            Glen = [findPercents(math.hypot(gx - x0, gy - y0), 140, 240, 0),
                    findPercents(math.hypot(gx - x0, gy - y0), 140, 240, 100)]
            Blen = [findPercents(math.hypot(bx - x0, by - y0), 120, 260, 0),
                    findPercents(math.hypot(bx - x0, by - y0), 120, 260, 100)]

            rgb = [Rlen[1], Glen[1], Blen[1]]
            print(rgb)

            for i in range(3):
                rgbcolor[i].ChangeDutyCycle(rgb[i])
    except KeyboardInterrupt:
        GPIO.cleanup()
        # end the PWM
        for i in range(3):
            rgbcolor[i].stop()
        print("\nExit....")
        # close the socket connection
        sys.exit()

    except ValueError as e:
        print(f"[ERROR]:{e}")
        print("try again...")
        GPIO.cleanup()
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
        pass
# RabbitMQ receiveing data


def callback(ch, method, properties, body):
    data = json.loads(body)
    landmarks = data
    # print(landmarks)  # Replace this with your own processing code
    GPIOthread = threading.Thread(
        target=AccessingTheGPIO, args=(landmarks,))
    GPIOthread.start()


if __name__ == "__main__":
    try:
        cred = pika.PlainCredentials('anish', 'dotmail123')
        connection = pika.BlockingConnection(
            pika.ConnectionParameters(host='172.19.0.2', port=5672, virtual_host='/', credentials=cred))
        channel = connection.channel()

        channel.queue_declare(queue='hand_gesture_data')

        channel.basic_consume(queue='hand_gesture_data',
                              on_message_callback=callback, auto_ack=True)

        print('Waiting for hand gesture data. To exit press CTRL+C')
        channel.start_consuming()

    except KeyboardInterrupt:
        GPIO.cleanup()
        # end the PWM
        for i in range(3):
            rgbcolor[i].stop()
        print("\nExit....")
        # close the socket connection
        sys.exit()
