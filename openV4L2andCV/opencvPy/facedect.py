#!/usr/bin/python
# -*- coding: UTF-8 -*-

# face_detect.py

# Face Detection using OpenCV. Based on sample code from:
# http://python.pastebin.com/m76db1d6b

# Usage: python face_detect.py <image_file>

import sys, os
#from cv import *
import cv2.cv as cv
#from opencv.highgui import *
from PIL import Image, ImageDraw
from math import sqrt

def detectObjects(image):
    """Converts an image to grayscale and prints the locations of any faces found"""
    grayscale = cv.CreateImage((image.width, image.height), 8, 1)
    cv.CvtColor(image, grayscale, cv.CV_BGR2GRAY)

    storage = cv.CreateMemStorage(0)
    #cvClearMemStorage(storage)
    cv.EqualizeHist(grayscale, grayscale)

    cascade = cv.Load(
        '/usr/share/opencv-2.4.5/data/haarcascades/haarcascade_frontalface_default.xml',storage)
    faces = cv.HaarDetectObjects(grayscale, cascade, storage, 1.1, 2,
        cv.CV_HAAR_DO_CANNY_PRUNING, (50,50))

    result = []
    for f in faces:
        result.append((f[0][0], f[0][1], f[0][0]+f[0][2], f[0][1]+f[0][3]))
	#print(f)
    return result

def grayscale(r, g, b):
    return int(r * .3 + g * .59 + b * .11)

def process(infile, outfile):

    image = cv.LoadImageM(infile);
    if image:
        faces = detectObjects(image)

    im = Image.open(infile)

    if faces:
        draw = ImageDraw.Draw(im)
        for f in faces:
            draw.rectangle(f, outline=(255, 0, 0))

        im.save(outfile, "JPEG", quality=100)
    else:
        print "Error: cannot detect faces on %s" % infile

if __name__ == "__main__":
    process('input.jpg', 'output.jpg')
