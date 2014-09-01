#! /usr/bin/python
# encoding:utf-8
#------------------------------------
# RPI.GPIO for python
#------------------------------------
try:
	import RPi.GPIO as GPIO
except RuntimeError:
	print("Error importing RPI.GPIO!")
import time
GPIO.setmode(GPIO.BOARD)
GPIO.setup(3,GPIO.OUT)
GPIO.output(3,GPIO.HIGH)
flag=0
while True:
	if flag:
		GPIO.output(3,GPIO.HIGH)
		flag=0
	else:
		GPIO.output(3,GPIO.HIGH)
		flag=1
	time.sleep(0.01)

