#!/usr/bin/env python
import RPi.GPIO as GPIO
from time import sleep


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

while 1:
    inp = int(input("wheel direct:"))
    #forward
    if inp == 1:
        GPIO.output(13, 1)
        GPIO.output(19, 0)
        GPIO.output(5, 1)
        GPIO.output(6, 0)
        GPIO.output(27, 1)
        GPIO.output(22, 0)
        GPIO.output(10, 1)
        GPIO.output(9, 0)
    #backward
    elif inp == 2:
        GPIO.output(13, 0)
        GPIO.output(19, 1)
        GPIO.output(5, 0)
        GPIO.output(6, 1)
        GPIO.output(27, 0)
        GPIO.output(22, 1)
        GPIO.output(10, 0)
        GPIO.output(9, 1)
    #left
    elif inp == 3:
        GPIO.output(13, 1)
        GPIO.output(19, 0)
        GPIO.output(5, 0)
        GPIO.output(6, 0)
        GPIO.output(27, 0)
        GPIO.output(22, 0)
        GPIO.output(10, 1)
        GPIO.output(9, 0)
    #right
    elif inp == 4:
        GPIO.output(13, 0)
        GPIO.output(19, 0)
        GPIO.output(5, 1)
        GPIO.output(6, 0)
        GPIO.output(27, 1)
        GPIO.output(22, 0)
        GPIO.output(10, 0)
        GPIO.output(9, 0)
    elif inp == 0:
        GPIO.output(13, 0)
        GPIO.output(19, 0)
        GPIO.output(5, 0)
        GPIO.output(6, 0)
        GPIO.output(27, 0)
        GPIO.output(22, 0)
        GPIO.output(10, 0)
        GPIO.output(9, 0)
