#! /usr/bin/python
# encoding:utf-8
#-------------------------------
# PWM to LED
#_______________________________
import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BOARD)
GPIO.setup(3,GPIO.OUT)

pwm = GPIO.PWM(3, 50)
pwm.start(0)

try:
	while True:
		for dc in range(0,101,2):
			pwm.ChangeDutyCycle(dc)
			time.sleep(0.05)
		for dc in range(100,-1,-2):
			pwm.ChangeDutyCycle(dc)
			time.sleep(0.05)
except KeyboardInterrupt:
	pass
pwm.stop()
GPIO.cleanup()

