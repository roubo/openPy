#!/usr/bin/python2.7
# vim: set fileencoding=utf-8:
""" 用随机数生成一幅8bpp的图片
    程序的思路是这样的：
    1、创建PGM header，包含：
	ID string (P5)
	Image width
	Image height
	Image data size
    2、生成随机矩阵
    3、将头和数据写入文件   

"""
import random as rnd #别名rnd
rnd.seed() # 启动随机码生成器

#图像参数 虽然都是简单的赋值，所有都是对象，实例
width = 256
height = 256
pxsize = 255

#创建PGM header
hdrstr = "P5\n%d\n%d\n%d\n" % (width, height, pxsize)

#创建一个列表
pixels = []
for i in range(0, width):
    for j in range(0, height):
        pixval = 2**rnd.randint(0,8)
	if pixval > pxsize:
		pixval = pxsize
	pixels.append(pixval)

outpix = "".join(map(chr, pixels))

outstr = hdrstr + outpix

FILE = open("image.pgm","w")
FILE.write(outstr)
FILE.close()
