#! /usr/bin/python
# encoding:utf-8
#-------------------------------
# CPU Temp
#_______________________________
import RPi.GPIO as GPIO
import time
import os
#--------
# gnuplot
#--------
tempf = open("/sys/class/thermal/thermal_zone0/temp","r")
outf = open("data1.txt", "w")
f = os.popen("gnuplot", "w")
print >> f, 'set title "CPU Temp" 1, 1 font "arial, 11"'
print >> f, 'set key font "arial, 9"'
print >> f, 'set tics font "arial, 8"'

print >> f, "set yrange[50500:51000]"
print >> f, "set xrange[0:100]"
#print >> f, 'set xdata time'
#print >> f, 'set timefmt "%H-%M-%S"'
#print >> f, 'set format x "%H-%M-%S"'
print >> f, 'set xlabel "time" font "arial, 11"'
print >> f, 'set ylabel "level" font "arial, 11"'
GPIO.setmode(GPIO.BOARD)
GPIO.setup(3,GPIO.OUT)
count=0
tempstr=""
#datestr=""
pwm = GPIO.PWM(3, 50)
pwm.start(0)
try:
	while True:
		tempstr=tempf.read(5)
		tempf.seek(0)
		#datestr=time.strftime("%H-%M-%S", time.localtime())
		#print tempstr
		count += 1
		if (count) > 100:
			count = 0
			print >> f, 'plot "data1.txt" using 1:2 with linespoints'
			f.flush()
			#outf.close()
			outf = open("data1.txt","w")
		#pwm.ChangeDutyCycle(dc)
		print >> outf, "%i\t%s" %(count,tempstr)
		time.sleep(0.05)
		print count
except KeyboardInterrupt:
	pass
outf.close()
tempf.close()
time.sleep(10)
pwm.stop()
GPIO.cleanup()

