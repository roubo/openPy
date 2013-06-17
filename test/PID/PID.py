#!/usr/bin/python2.7
# vim: set fileencoding=utf-8
"""
    easy PID control
	简单的PID控制器
"""
kp = 1.0
kb = 1.0
p  = 0

#input
rinput = 0
cinput = 1

def acquire(port):
    return 1

def pcontrol():
    rval = acquire(rinput)
    bval = acquire(cinput)*kb
    eval = rval - bval
    return (kp*eval) + p
