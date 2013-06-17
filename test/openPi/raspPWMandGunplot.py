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
#outf = open("data.txt", "w")
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
		for dc in range(0,101,2):
			#print dc
			count += 1
			if (count) > 3000:
				count = 0 
			pwm.ChangeDutyCycle(dc)
			outf = open("data.txt","w")
			print >> outf, "%i\t%i" %(count,dc)
			outf.close()
			#print >> f, 'reset'
			print >> f, 'plot "data.txt" '
			f.flush()
			time.sleep(0.05)
			#print "plot"
		for dc in range(100,-1,-2):
			#print dc
			count +=1
			if(count) >3000:
				count = 0
			pwm.ChangeDutyCycle(dc)
			outf = open("data.txt","w")
			print >> outf, "%i\t%i" %(count,dc)
			outf.close()
			#print >> f, 'reset'			
			print >> f, 'plot "data.txt" '
			f.flush()
			time.sleep(0.05)
			#print "plot"
except KeyboardInterrupt:
	pass
#outf.close()
time.sleep(10)
pwm.stop()
GPIO.cleanup()

