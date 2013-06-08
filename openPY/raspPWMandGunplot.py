#! /usr/bin/python
# encoding:utf-8
#-------------------------------
# PWM to LED
#_______________________________
import RPi.GPIO as GPIO
import time
import os
#--------
# gnuplot
#--------
f = os.popen('gnuplot', 'w')
print >> f, 'set title "LED Light level" 1, 1 font "arial, 11"'
print >> f, 'set key font "arial, 9"'
print >> f, 'set tics font "arial, 8"'

print >> f, "set yrange[0:150]"
print >> f, "set xrange[0:3000]"
print >> f, 'set xlabel "time" font "arial, 11"'
print >> f, 'set ylabel "level" font "arial, 11"'
GPIO.setmode(GPIO.BOARD)
GPIO.setup(3,GPIO.OUT)

pwm = GPIO.PWM(3, 50)
pwm.start(0)
try:
	while True:
		for dc in range(0,101,2):
			pwm.ChangeDutyCycle(dc)
			print >> f, 'plot %i' %(dc)
			#f.flush()
			time.sleep(0.05)
		for dc in range(100,-1,-2):
			pwm.ChangeDutyCycle(dc)
			print >> f, 'plot %i' %(dc)
			#f.flush()
			time.sleep(0.05)
except KeyboardInterrupt:
	pass
f.flush()
time.sleep(60)
pwm.stop()
GPIO.cleanup()

