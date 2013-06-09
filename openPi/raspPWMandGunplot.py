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
outf = open("data.txt", "a")
f = os.popen("gnuplot", "w")
print >> f, 'set title "LED Light level" 1, 1 font "arial, 11"'
print >> f, 'set key font "arial, 9"'
print >> f, 'set tics font "arial, 8"'

print >> f, "set yrange[0:150]"
print >> f, "set xrange[0:3000]"
print >> f, 'set xlabel "time" font "arial, 11"'
print >> f, 'set ylabel "level" font "arial, 11"'
GPIO.setmode(GPIO.BOARD)
GPIO.setup(3,GPIO.OUT)
count=0
pwm = GPIO.PWM(3, 50)
pwm.start(0)
try:
	while True:
		#count += 1
		#if (count) > 3000:
		#	count=0;
		for dc in range(0,101,2):
			count += 1
			if (count) > 3000:
				count = 0 
			pwm.ChangeDutyCycle(dc)
			print >> outf, "%i\t%i" %(count,dc)
			#print >> f, 'plot x*%i, 0' %(dc)
			print >> f, 'plot "data.txt" with linespoints'
			f.flush()
			time.sleep(0.05)
		for dc in range(100,-1,-2):
			count +=1
			if(count) >3000:
				count = 0
			pwm.ChangeDutyCycle(dc)
			print >> outf, "%i\t%i" %(count,dc)
			print >> f, 'plot "data.txt" with linespoints'
			#print >> f, 'plot x*%i, 0' %(dc)
			f.flush()
			time.sleep(0.05)
except KeyboardInterrupt:
	pass
#print >> f, 'plot "data.txt" with linespoints'
#f.flush()
outf.close()
time.sleep(60)
pwm.stop()
GPIO.cleanup()

