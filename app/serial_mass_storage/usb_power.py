#!/usr/bin/python
# -*- coding: utf-8 -*-
import RPi.GPIO as GPIO
import time
PRINT_LED = 32

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(PRINT_LED, GPIO.OUT)

time.sleep(7)
print ("relay triggered")
GPIO.output(PRINT_LED,1)
time.sleep(3)
print ("relay off")
GPIO.output(PRINT_LED,0)

