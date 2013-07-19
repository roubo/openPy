#!/usr/bin/python 
# encoding:utf-8
from distutils.core import setup, Extension
setup(name= 'foo', version = '1.0', ext_modules = [Extension('foo', ['foo.c'])])
