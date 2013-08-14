/***************************************************
V4L2 for Python
roubo_dai@san412.in

****************************************************/

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include "pthread.h"
#include <sys/ioctl.h> 
#include <stdint.h>
#include <asm/types.h>
#include <linux/videodev2.h>
#include <sys/mman.h>
#include <malloc.h>
#include <linux/fb.h>
#include <math.h>
//为了在非嵌入式设备上使用，这里使用了SDL
#include <X11/Xlib.h>
#include <SDL/SDL.h>
#include "opencv/cv.h"
#include "opencv2/highgui/highgui.hpp"

#define X               640
#define Y               480
#define X_F 		1280
#define Y_F 		1024
#define FALSE		0
#define TRUE		1


static int video_fd;
static int fb;
char Data[512];

//只使用了SDL的消息机制，可以剔除
char sdl_quit = 1;
static unsigned int SDL_VIDEO_Flags = SDL_ANYFORMAT | SDL_DOUBLEBUF | SDL_RESIZABLE;

//运动检测算法中被使用---------------------------------------
// various tracking parameters (in seconds)
const double MHI_DURATION = 0.8;
const double MAX_TIME_DELTA = 0.5;
const double MIN_TIME_DELTA = 0.05;

// number of cyclic frame buffer used for motion detection
const int N = 4;

// ring image buffer
IplImage **mbuf = 0;
int last = 0;

// temporary images
IplImage *mhi = 0; // MHI 运动历史
IplImage *orient = 0; // orientation 方向
IplImage *mask = 0; // valid orientation mask 有效方向
IplImage *segmask = 0; // motion segmentation map 运动分割
CvMemStorage* mstorage = 0; // temporary storage 临时数据

//运动检测与ZOOM的联合测试-----------------------------------
unsigned int  catchflag=0;    //目标出现标志
unsigned int  tx=0,ty=0;      //目标坐标（初始映射为小图像上）
unsigned int  lx=0,ly=0;      //记录上一次目标的大图像坐标
unsigned int  delayN =0;      //记录丢失运动目标的倒数次数
#define      DELAYN       30
#define      DX           320
#define      DY           240 

//运动检测附加稳定算法---------------------------------------
unsigned int  ax=0,ay=0;       //记录目标算法中心坐标
unsigned int  tmpal1=0, tmpal2=0; 
unsigned int  Zmode = 0;       //放大或缩小模式
#define     ZMODE_Z       1    
#define     ZMOOE_S 	  0 

//用户空间视频缓冲区结构，用于MMap来自驱动程序的内存空间-----
typedef struct VideoBuffer
{
	unsigned char *start;
	size_t offset;
	size_t length;
} VideoBuffer;
VideoBuffer *buffers;

//fd显示设备结构,包含MMap后用户空间地址成员------------------
struct fb_dev
{
	//for frame buffer
	int fb;
	unsigned char *fb_mem;	//frame buffer mmap
	int fb_width, fb_height, fb_line_len, fb_size;
	int fb_bpp;
	int fb_gray;
} fbdev;

//RGB888格式数据保存的首地址-------------------------------
unsigned char *ptcur888;

//与fb驱动程序的交互，获取fb信息，为能正确显示----------------
int fb_stat(int fd)
{
	struct fb_fix_screeninfo fb_finfo;
	struct fb_var_screeninfo fb_vinfo;

	if (ioctl(fd, FBIOGET_FSCREENINFO, &fb_finfo))
	{
		perror(__func__);
		return (-1);
	}

	if (ioctl(fd, FBIOGET_VSCREENINFO, &fb_vinfo))
	{
		perror(__func__);
		return (-1);
	}
	//得到framebuffer的长、宽和位宽---------------------
	fbdev.fb_width = fb_vinfo.xres;
	fbdev.fb_height = fb_vinfo.yres;
	fbdev.fb_bpp = fb_vinfo.bits_per_pixel;
	fbdev.fb_gray = fb_vinfo.grayscale;	
	fbdev.fb_line_len = fb_finfo.line_length;
	fbdev.fb_size = fb_finfo.smem_len;

	return (0);
}
//与fb驱动程序交互，设置fb信息-------------------------------
int fb_set(int fd)
{
	struct fb_var_screeninfo fb_vinfo;
	
	if (ioctl(fd, FBIOGET_VSCREENINFO, &fb_vinfo))
	{
		perror(__func__);
		return (-1);
	}
	fb_vinfo.xres=480;
	fb_vinfo.yres=272;	
	fb_vinfo.bits_per_pixel=32;
	fb_vinfo.grayscale=0;
	if (ioctl(fd, FBIOPUT_VSCREENINFO, &fb_vinfo))
	{
		perror(__func__);
		return (-1);
	}
	return 0;
	
}

/************************************************************
YUYV格式转换为RGB888：
*************************************************************/
unsigned int convert_yuv_rgb_pixel(int y,int u,int v)
{
	unsigned int pixel32 = 0;
	unsigned char *pixel = (unsigned char *)&pixel32;//地址指针映射
	int r,g,b;
	r=y+(1.370705 * (v-128));
	g=y-(0.698001*(v-128))-(0.337633*(u-128));
	b=y+(1.732446*(u-128));
	if(r>255)
		r=255;
	if(g>255)
		g=255;
	if(b>255)
		b=255;
	if(r<0)
		r=0;
	if(g<0)
		g=0;
	if(b<0)
		b=0;
	pixel[0]=r*220/256;
	pixel[1]=g*220/256;
	pixel[2]=b*220/256;
	
	return pixel32;
}


/*************************************************************************
参数说明：  
  yuyv   :元数据首地址。
  rgb888 :24位
  width  :视频宽度。
  height :视频高度。
*************************************************************************/
int YUYVToRGB888(unsigned char *yuyv,unsigned char *rgb888,unsigned int width,unsigned int height)
{
	unsigned int in,out=0;
	unsigned int pixel_16;
	unsigned char pixel_24[3];
	unsigned int pixel32;
	int y0,u,y1,v;//YUV422数据格式
	//每次循环处理2个像素点--------------
	for(in=0;in<width*height*2;in+=4)
	{
		pixel_16=(yuyv[in+3]<<24)|(yuyv[in+2]<<16)|(yuyv[in+1]<<8)|(yuyv[in+0]);
		y0=(pixel_16&0x000000ff);
		u =(pixel_16&0x0000ff00)>>8;
		y1=(pixel_16&0x00ff0000)>>16;
		v =(pixel_16&0xff000000)>>24;
		
		pixel32=convert_yuv_rgb_pixel(y0,u,v);
		pixel_24[0]=(pixel32&0x000000ff);
		pixel_24[1]=(pixel32&0x0000ff00)>>8;
		pixel_24[2]=(pixel32&0x00ff0000)>>16;
				
		rgb888[out++]=pixel_24[2];//r1		
		rgb888[out++]=pixel_24[1];//g1
		rgb888[out++]=pixel_24[0];//b1
		

		pixel32=convert_yuv_rgb_pixel(y1,u,v);
		pixel_24[0]=(pixel32&0x000000ff);
		pixel_24[1]=(pixel32&0x0000ff00)>>8;
		pixel_24[2]=(pixel32&0x00ff0000)>>16;
				
		rgb888[out++]=pixel_24[2];//r2
		rgb888[out++]=pixel_24[1];//g2
		rgb888[out++]=pixel_24[0];//b2
		

	}
	return 0;
}

/***********************************************************
 * 感兴趣图像截取
 * 原始大图像：src
 * 最终小图像：dst
 * 感兴趣点在大图像中的坐标：（xI，yI）
 * 大图像大小：（swidth , sheight）
 * 小图像大小：（dwidth , dheight）
 * 
 * src-> --------------------------------------------
 *       |                                          |
 *       |                                          |
 *    ^  |                                          |
 *    |  |                                          |
 *    y  --------------------------------------------
 *       x坐标->
 * ********************************************************/
int cutinterest(unsigned char *src,unsigned char *dst,unsigned int xI,unsigned int yI,unsigned int swidth,unsigned int sheight,unsigned int dwidth,unsigned int dheight)
{
	unsigned int xe,ye;        //兴趣面左上角坐标
	unsigned int start;        //相对大图像的截取起始点偏移量
	unsigned int i,j;
	printf("Cent:(%d,%d)\n",xI,yI);
	//测试兴趣点是否在边缘
	if(xI<dwidth/2 || xI>swidth-dwidth/2 || yI<dheight/2 || yI>sheight-dheight/2)
	{
		//return 1;
		if(xI<dwidth/2)
		{
			if(yI<dheight/2)
			{
				xI = dwidth/2;
				yI = dheight/2;
			}
			else if(yI>sheight-dheight/2)
			{
				xI = dwidth/2;
				yI = sheight-dheight/2;
			}
			else
			{
				xI = dwidth/2;
				yI = yI;
			}
		}
		else if(xI>swidth-dwidth/2)
		{
			
			if(yI<dheight/2)
			{
				xI = swidth-dwidth/2;
				yI = dheight/2;
			}
			else if(yI>sheight-dheight/2)
			{
				xI = swidth-dwidth/2;
				yI = sheight-dheight/2;
			}
			else
			{
				xI = swidth-dwidth/2;
				yI = yI;
			}
		}
		else
		{
			if(yI < dheight/2)
			{
				xI = xI;
				yI = dheight/2;
			}
			else
			{
				xI = xI;
				yI = sheight-dheight/2;
			}
		}
	}

	xe = xI - dwidth/2;
	ye = yI + dheight/2;
	//printf("Left Top:(%d,%d)\n",xe,ye);
	start = (sheight - (ye-1))*swidth + xe; 
	//printf("start offset:%d\n",start);
	for(i=0;i<dheight;i++)
	{
		for(j=0;j<dwidth*2;j++)//yuv422
		{
			*dst++ = *(src+start*2+j+i*swidth*2);	
		}
	}
		

	return 0;
}
/************************************************************
 * 算法测试 灰度转化
 ***********************************************************/
int yuv2gray(unsigned char *yuyv,unsigned char *gray,unsigned int width,unsigned int height)
{
	int all = 0;
	
int count =0;
	unsigned char y1;
	unsigned char u;
	unsigned char y2;
	unsigned char v;
	//printf("gray test\n");
	while(all < width*height-2)
	{
		
		y1 = *yuyv++;
		u  = *yuyv++;
		y2 = *yuyv++;
		v  = *yuyv++;
		gray[count++] = y1;
		gray[count++] = 0x80;
		gray[count++] = y2;
		gray[count++] = 0x80;
		all += 2;
	}
	return 0;
}


/**************************************************************
 * 算法，运动检测
 * ***********************************************************/

static void  update_mhi( IplImage* img, IplImage* dst, int diff_threshold )
{
    double timestamp = (double)clock()/CLOCKS_PER_SEC; // get current time in seconds
    CvSize size = cvSize(img->width,img->height); // get current frame size
    int i, idx1 = last, idx2;
    IplImage* silh;
    CvSeq* seq;
    CvRect comp_rect;
    double count;
    double angle;
    CvPoint center;
    double magnitude;
    CvScalar color;

    unsigned  int tmpx=0,tmpy=0;
    int tmppcount=0;
    // allocate images at the beginning or
    // reallocate them if the frame size is changed
    if( !mhi || mhi->width != size.width || mhi->height != size.height ) {
        if( mbuf == 0 ) {
            mbuf = (IplImage**)malloc(N*sizeof(mbuf[0]));
            memset( mbuf, 0, N*sizeof(mbuf[0]));
        }

        for( i = 0; i < N; i++ ) {
            cvReleaseImage( &mbuf[i] );
            mbuf[i] = cvCreateImage( size, IPL_DEPTH_8U, 1 );
            cvZero( mbuf[i] );
        }
        cvReleaseImage( &mhi );
        cvReleaseImage( &orient );
        cvReleaseImage( &segmask );
        cvReleaseImage( &mask );

        mhi = cvCreateImage( size, IPL_DEPTH_32F, 1 );
        cvZero( mhi ); // clear MHI at the beginning
        orient = cvCreateImage( size, IPL_DEPTH_32F, 1 );
        segmask = cvCreateImage( size, IPL_DEPTH_32F, 1 );
        mask = cvCreateImage( size, IPL_DEPTH_8U, 1 );
    }

    cvCvtColor( img, mbuf[last], CV_BGR2GRAY ); // convert frame to grayscale

    idx2 = (last + 1) % N; // index of (last - (N-1))th frame
    last = idx2;

    silh=mbuf[idx2];
    cvAbsDiff( mbuf[idx1], mbuf[idx2], silh );                          //

    cvThreshold( silh, silh, diff_threshold, 1, CV_THRESH_BINARY ); // 类似二值化 可是只有0 1这么小的差距
    cvUpdateMotionHistory( silh, mhi, timestamp, MHI_DURATION );    // 更新以时间为单位的运动历史图

    // 将运动历史图转换为具有像素值的图片，并merge到输出图像
    cvCvtScale( mhi, mask, 255./MHI_DURATION,
                (MHI_DURATION - timestamp)*255./MHI_DURATION );
    cvZero( dst );
    cvMerge( mask,0,0,0,dst );

    // calculate motion gradient orientation and valid orientation mask
    cvCalcMotionGradient( mhi, mask, orient, MAX_TIME_DELTA, MIN_TIME_DELTA, 3 );//orient中保存每个运动点的方向角度值

    if( !mstorage )
        mstorage = cvCreateMemStorage(0);
    else
        cvClearMemStorage(mstorage);

    // segment motion: get sequence of motion components
    // segmask is marked motion components map. It is not used further
    seq = cvSegmentMotion( mhi, segmask, mstorage, timestamp, MAX_TIME_DELTA );//对运动历史图片进行运动单元的分割

    // iterate through the motion components,
    // One more iteration (i == -1) corresponds to the whole image (global motion)
    for( i = -1; i < seq->total; i++ ) {

        if( i < 0 ) { // case of the whole image
            comp_rect = cvRect( 0, 0, size.width, size.height );
            color = CV_RGB(255,255,255);
            magnitude = 100;
	   // printf("ALL image.\n");
        }
        else { // i-th motion component
            comp_rect = ((CvConnectedComp*)cvGetSeqElem( seq, i ))->rect;
            if( comp_rect.width + comp_rect.height < 30 ) // reject very small components
                continue;
            color = CV_RGB(255,0,0);
            magnitude = 30;
        }

        // select component ROI
        cvSetImageROI( silh, comp_rect );
        cvSetImageROI( mhi, comp_rect );
        cvSetImageROI( orient, comp_rect );
        cvSetImageROI( mask, comp_rect );

        // calculate orientation
        angle = cvCalcGlobalOrientation( orient, mask, mhi, timestamp, MHI_DURATION);//统计感兴趣区域内的运动单元的总体方向
        angle = 360.0 - angle;  // adjust for images with top-left origin

        count = cvNorm( silh, 0, CV_L1, 0 ); // calculate number of points within silhouette ROI

        cvResetImageROI( mhi );
        cvResetImageROI( orient );
        cvResetImageROI( mask );
        cvResetImageROI( silh );

        // check for the case of little motion
        if( count < comp_rect.width*comp_rect.height * 0.05*0.1  )
            continue;

        // draw a clock with arrow indicating the direction
        center = cvPoint( (comp_rect.x + comp_rect.width/2),
                          (comp_rect.y + comp_rect.height/2) );

	if(Zmode == ZMODE_Z && i !=-1)
	{
		//放大模式下的稳定算法++++++++++++++++++++++++++
		tmpx = comp_rect.x+comp_rect.width/2;//当前获取到的目标坐标之一
		tmpy = comp_rect.y+comp_rect.height/2;
		tmpal1 =  sqrt(abs(tx-ax))+sqrt(abs(ty-ay));

		if(tmpal2 > tmpal1 || tmppcount == 0)//更接近中心
		{
			tmpal2 = tmpal1;
			tx = tmpx;
			ty = tmpy;
		}
		catchflag = 1;
	}
	
	if(Zmode == ZMOOE_S)
	{	
		//缩小模式下的选择目标规则
		if(tmppcount == 0 && i != -1)
		{
			tx = comp_rect.x+comp_rect.width/2;
			ty = comp_rect.y+comp_rect.height/2;
			catchflag = 1;
		}
	}
	tmppcount++;
	printf("The %dth rect:(%d,%d)\n",tmppcount,comp_rect.x+comp_rect.width/2,comp_rect.y+comp_rect.height/2);
        cvCircle( img, center, cvRound(magnitude*1.2), color, 3, CV_AA, 0 );
        cvLine( img, center, cvPoint( cvRound( center.x + magnitude*cos(angle*CV_PI/180)),
                cvRound( center.y - magnitude*sin(angle*CV_PI/180))), color, 3, CV_AA, 0 );
    }

}
/**************************************************************
嵌入式设备上使用，为了能实现能在16位的LCD上显示，转换RGB888为RGB565
**************************************************************/
unsigned short RGB888toRGB565(unsigned char red, unsigned char green, unsigned char blue)
{
	unsigned short B = (blue >> 3) & 0x001F;
	unsigned short G = ((green >> 2) << 5) & 0x07E0;
	unsigned short R = ((red >> 3) << 11) & 0xF800;

	return (unsigned short) (R | G | B);
}


/**************************************************************
嵌入式设备上使用， 显示一个像素点的图像到framebuffer上，与上面函数可以对应使用
**************************************************************/
int fb_pixel(void *fbmem, int width, int height, int x, int y, unsigned short color)
{
	if ((x > width) || (y > height))
		return (-1);

	unsigned short *dst = ((unsigned short *) fbmem + y * width + x);

	*dst = color;
	return 0;
}

//释放framebuffer的映射------------------------------------
int fb_munmap(void *start, size_t length)
{
	return (munmap(start, length));
}

/*************************************************************
 * 以新配置重启设备
 * 注意指数形参的使用方法
 * **********************************************************/
int restartdev(int *vfd, VideoBuffer **bufs, struct v4l2_requestbuffers *req, struct v4l2_buffer *buf,unsigned int nx,unsigned int ny) 
{
	int i,j,nbufs,ret1;
	//clean up
	//++++++++++++++++++++++++++++++关闭视频流start+++++++++++++++++++++++++
	enum v4l2_buf_type type;			
	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (ioctl(*vfd, VIDIOC_STREAMOFF, &type) < 0)
	{
		printf("VIDIOC_STREAMOFF error\n");
		return 2;
	}
	//+++++++++++++++++++++++++++++关闭视频流end++++++++++++++++++++++++++
				
	//+++++++++++++++++++++++++++++清队列start++++++++++++++++++++++++++++++
	for(i=0;i<(*req).count;i++)
	{
		if(-1==munmap((*(*bufs+i)).start,(*(*bufs+i)).length))
			printf("munmap error:%d \n",i);
	}
	//++++++++++++++++++++++++++++清队列end+++++++++++++++++++++++++++++++++	
				
	//关设备
	close(*vfd);
				
	//重启设备
	*vfd = open("/dev/video0", O_RDWR, 0);//打开摄像头设备，使用阻塞方式打开
	if (*vfd<0)
	{
		printf("open error\n");
		return  1;
	}
	struct v4l2_format fmt1;	
	//---------------------重新设置获取视频的格式----------------//	
	memset( &fmt1, 0, sizeof(fmt1));
	fmt1.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;	     //视频数据流类型，永远都V4L2_BUF_TYPE_VIDEO_CAPTURE
	fmt1.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV; 	     //视频源的格式为JPEG或YUN4:2:2或RGB
	fmt1.fmt.pix.width = nx;                             //设置视频宽度
	fmt1.fmt.pix.height = ny;                            //设置视频高度
	//fmt.fmt.pix.field=V4L2_FIELD_INTERLACED;
	//fmt.fmt.pix.colorspace=8;
	//printf("color: %d \n",fmt.fmt.pix.colorspace);
	if (ioctl(*vfd, VIDIOC_S_FMT, &fmt1) < 0) //使配置生效
	{
		printf("set format failed\n");
		return 2;
	}
	//-------------------------------------------------------//

	//++++++++++++++++++++++++向video设备驱动申请帧缓冲start+++++++++++++++++
	memset(&(*req), 0, sizeof (*req));
	(*req).count = 3;	                                   //缓存数量，即可保存的图片数量
	(*req).type = V4L2_BUF_TYPE_VIDEO_CAPTURE;	           //数据流类型，永远都是V4L2_BUF_TYPE_VIDEO_CAPTURE
	(*req).memory = V4L2_MEMORY_MMAP;	                   //存储类型：V4L2_MEMORY_MMAP或V4L2_MEMORY_USERPTR
	if (ioctl(*vfd, VIDIOC_REQBUFS, &(*req)) == -1)   //使配置生效
	{
		perror("request buffer error \n");
		return 2;
	}
	//++++++++++++++++++++++++向video设备驱动申请帧缓冲end+++++++++++++++++
	
	//+++++++++++++++将VIDIOC_REQBUFS获取内存转为物理空间start+++++++++++++
	*bufs = calloc((*req).count, sizeof(VideoBuffer));	
	//printf("sizeof(VideoBuffer) is %d\n", sizeof(VideoBuffer));
	for (nbufs = 0; nbufs < (*req).count; nbufs++)
	{
		memset( &(*buf), 0, sizeof(*buf));
		(*buf).type = V4L2_BUF_TYPE_VIDEO_CAPTURE;	
		
		//存储类型：V4L2_MEMORY_MMAP（内存映射）或V4L2_MEMORY_USERPTR（用户指针）
		(*buf).memory = V4L2_MEMORY_MMAP;
		(*buf).index = nbufs;
		if (ioctl(*vfd, VIDIOC_QUERYBUF, &(*buf)) < 0)        //使配置生效
		{
			printf("VIDIOC_QUERYBUF error\n");
			return 2;
		}
		//printf("buf len is %d\n", sizeof(buf));
		(*(*bufs+nbufs)).length = (*buf).length;
		(*(*bufs+nbufs)).offset = (size_t) (*buf).m.offset;
		
		//使用mmap函数将申请的缓存地址转换应用程序的绝对地址------
		(*(*bufs+nbufs)).start = mmap(NULL, (*buf).length, PROT_READ | PROT_WRITE,
					MAP_SHARED, *vfd, (*buf).m.offset);	
		if ((*(*bufs+nbufs)).start == MAP_FAILED)
		{
			perror("buffers error\n");
			return 2;
		}
		if (ioctl(*vfd, VIDIOC_QBUF, &(*buf)) < 0)           //放入缓存队列
		{
			printf("VIDIOC_QBUF error\n");
			return 2;
		}

	}
	//+++++++++++++++将VIDIOC_REQBUFS获取内存转为物理空间end+++++++++++++
	
	//++++++++++++++++++++++++++++++打开视频流start+++++++++++++++++++++++++
	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (ioctl(*vfd, VIDIOC_STREAMON, &type) < 0)
	{
		printf("VIDIOC_STREAMON error\n");
		return 2;
	}
	//---------------------读取视频源格式---------------------//	
	fmt1.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;				
	if (ioctl(*vfd, VIDIOC_G_FMT, &fmt1) < 0)	
	{
		printf("get format failed\n");
		return 2 ;
	}
	else
	{
		printf("Picture:Width = %d   Height = %d\n", fmt1.fmt.pix.width, fmt1.fmt.pix.height);
		
	}
	//-------------------------------------------------------//
	
	for(j=0;j<5;j++)
	{
		fd_set fds;//文件描述符集，准备使用Select机制
		struct timeval tv;
		//++++++++++++++++++++IO select start++++++++++++++++++++++++++
		FD_ZERO(&fds);//清空文件描述符集
		FD_SET(*vfd,&fds);//将视频设备文件的描述符放入集合中
		
		//消息等待超时,可以完全阻塞-------------------------------
		tv.tv_sec =5;
		tv.tv_usec=0;
		//等待视频设备准备好--------------------------------------
		ret1=select(*vfd+1,&fds,NULL,NULL,&tv);
		if(-1==ret1)
		{
			if(EINTR==errno)
				continue;
			printf("select error. \n");
			exit(EXIT_FAILURE);
		}
		else if(0==ret1)
		{
			printf("select timeout. \n");
			if(j==4)
			{
				printf("Timeout too many times.\n");
				return 2;
			}
			else
				continue;
		}
		else
			break;
	}
	return 0;

}
/*************************************************************
  视频设备和显示设备初始化，预览函数
*************************************************************/
int video_fb_init_preview()
{
	
	//临时
	int tmpcount=0;

	//串口相关变量-------------------------------
	char buff[512];
	int nread=0;
	int FrameDone=0;//一帧数据结束标志
	int FrameCosunt=0;//记录帧长度
	int j=0;
	int key=0;//开关标志
	int stat=0;//视频设备状态标志
	//-------------------------------------------
	
	int numBufs;

	//--------------------------------------------
	//SDL yuv
	SDL_Surface      *pscreen;
	SDL_Overlay      *overlay;
	SDL_Rect         drect;
	SDL_Event        sdlevent;
	SDL_mutex        *affmutex;
	unsigned char    *p = NULL;
	unsigned char    frmrate;
	unsigned int     currtime;
	unsigned int     lasttime;
	char* status = NULL;

	//SDL RGB
	unsigned int     rmask;
	unsigned int     gmask;
	unsigned int     bmask;
	unsigned int     amask;	
	int              bpp;
	int 		 pitch;
	int 		 pixels_num;
	unsigned char    *pixels;
	unsigned char    *p_RGB = NULL;	
	SDL_Surface      *pscreen_RGB;
	SDL_Surface      *display_RGB;
	printf("USB Camera Test\n");



	//++++++++++++++++++++++++SDL初始化和设置start+++++++++++++++++++++++++++++++
	/*if(SDL_Init(SDL_INIT_VIDEO) < 0)
	{
		printf("SDL Init failed.\n");
		exit(1);
	}
	//SDL 设置:YUV输出
	pscreen = SDL_SetVideoMode(fmt.fmt.pix.width, fmt.fmt.pix.height,0,SDL_VIDEO_Flags);
	overlay = SDL_CreateYUVOverlay(fmt.fmt.pix.width, fmt.fmt.pix.height,SDL_YUY2_OVERLAY,pscreen);
	p = (unsigned char *)overlay->pixels[0];
	drect.x = 0;
	drect.y = 0;
	drect.w = pscreen->w;
	drect.h = pscreen->h;

	//SDL 设置:RGB输出
	//pscreen = SDL_SetVideoMode(fmt.fmt.pix.width, fmt.fmt.pix.height, 24, SDL_SWSURFACE | SDL_DOUBLEBUF);
	//rmask = 0x000000ff;
	//gmask = 0x0000ff00;
	//bmask = 0x00ff0000;
	//amask = 0x00000000;
	//bpp   = 24;
	pitch = fmt.fmt.pix.width*3;
	pixels_num = fmt.fmt.pix.width*fmt.fmt.pix.height*3;
	pixels = (unsigned char *)malloc(pixels_num);
	memset(pixels, 0, pixels_num);
	p_RGB = (unsigned char *)pixels;
	//pscreen_RGB = SDL_CreateRGBSurfaceFrom(pixels, 
					//fmt.fmt.pix.width, 
					//fmt.fmt.pix.height, 
					//bpp, 
					//pitch, 
					//rmask, 
					//gmask, 
					//bmask, 
					//amask);
	//lasttime = SDL_GetTicks();
	//affmutex = SDL_CreateMutex();*/
	//++++++++++++++++++++++++SDL初始化和设置end+++++++++++++++++++++++++++++++
	
	//++++++++++++++++++++++++openCV设置start+++++++++++++++++++++++++++++++
	CvMemStorage*  storage = cvCreateMemStorage(0);
	IplImage*      img     = cvCreateImageHeader(cvSize(X,Y), IPL_DEPTH_8U, 3);//image头，未开辟数据空间
	IplImage*      imggray = cvCreateImage(cvSize(X,Y), IPL_DEPTH_8U, 1);//image，开辟数据空间
	IplImage*      motion  = cvCreateImage(cvSize(X,Y), IPL_DEPTH_8U, 3);
	cvNamedWindow("image", 1);

	unsigned char *pRGB = NULL;
	pRGB = (unsigned char *)calloc(1,X*Y*3*sizeof(unsigned char));
	//++++++++++++++++++++++++openCV设置end+++++++++++++++++++++++++++++++++



	video_fd = open("/dev/video0", O_RDWR, 0);//打开摄像头设备，使用阻塞方式打开
	if (video_fd<0)
	{
		printf("open error\n");
		return  1;
	}

	/*************先向驱动尝试获取设备视频格式start*************/
	struct v4l2_fmtdesc fmt0;
	int ret0;
	memset(&fmt0,0,sizeof(fmt0));
	fmt0.index = 0;
	fmt0.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
	while((ret0 = ioctl(video_fd,VIDIOC_ENUM_FMT,&fmt0) == 0))
	{
		fmt0.index++;
		printf("%d> pixelformat =%c%c%c%c,description =%s\n",
			fmt0.index,fmt0.pixelformat&0xff,
			(fmt0.pixelformat>>8)&0xff,
			(fmt0.pixelformat>>16)&0xff,
			(fmt0.pixelformat>>24)&0xff,
			fmt0.description);
	}
	/**************************END***************************/
	
	//---------------------设置获取视频的格式----------------//
	struct v4l2_format fmt;	
	memset( &fmt, 0, sizeof(fmt));
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;	     //视频数据流类型，永远都V4L2_BUF_TYPE_VIDEO_CAPTURE
	fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV; //视频源的格式为JPEG或YUN4:2:2或RGB
	fmt.fmt.pix.width = X;                     //设置视频宽度
	fmt.fmt.pix.height = Y;                    //设置视频高度
	//fmt.fmt.pix.field=V4L2_FIELD_INTERLACED;
	//fmt.fmt.pix.colorspace=8;
	//printf("color: %d \n",fmt.fmt.pix.colorspace);
	if (ioctl(video_fd, VIDIOC_S_FMT, &fmt) < 0) //使配置生效
	{
		printf("set format failed\n");
		return 2;
	}
	//-------------------------------------------------------//
	

	//++++++++++++++++++++++++向video设备驱动申请帧缓冲start+++++++++++++++++
	struct v4l2_requestbuffers req;
	memset(&req, 0, sizeof (req));
	req.count = 3;	                                   //缓存数量，即可保存的图片数量
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;	           //数据流类型，永远都是V4L2_BUF_TYPE_VIDEO_CAPTURE
	req.memory = V4L2_MEMORY_MMAP;	                   //存储类型：V4L2_MEMORY_MMAP或V4L2_MEMORY_USERPTR
	if (ioctl(video_fd, VIDIOC_REQBUFS, &req) == -1)   //使配置生效
	{
		perror("request buffer error \n");
		return 2;
	}
	//++++++++++++++++++++++++向video设备驱动申请帧缓冲end+++++++++++++++++
	
	//+++++++++++++++将VIDIOC_REQBUFS获取内存转为物理空间start+++++++++++++
	buffers = calloc(req.count, sizeof(VideoBuffer));	
	//printf("sizeof(VideoBuffer) is %d\n", sizeof(VideoBuffer));
	struct v4l2_buffer buf;
	for (numBufs = 0; numBufs < req.count; numBufs++)
	{
		memset( &buf, 0, sizeof(buf));
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;	
		
		//存储类型：V4L2_MEMORY_MMAP（内存映射）或V4L2_MEMORY_USERPTR（用户指针）
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = numBufs;
		if (ioctl(video_fd, VIDIOC_QUERYBUF, &buf) < 0)        //使配置生效
		{
			printf("VIDIOC_QUERYBUF error\n");
			return 2;
		}
		//printf("buf len is %d\n", sizeof(buf));
		buffers[numBufs].length = buf.length;
		buffers[numBufs].offset = (size_t) buf.m.offset;
		
		//使用mmap函数将申请的缓存地址转换应用程序的绝对地址------
		buffers[numBufs].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
			MAP_SHARED, video_fd, buf.m.offset);	
		if (buffers[numBufs].start == MAP_FAILED)
		{
			perror("buffers error\n");
			return 2;
		}
		if (ioctl(video_fd, VIDIOC_QBUF, &buf) < 0)           //放入缓存队列
		{
			printf("VIDIOC_QBUF error\n");
			return 2;
		}

	}
	//+++++++++++++++将VIDIOC_REQBUFS获取内存转为物理空间end+++++++++++++
	
	//++++++++++++++++++++++++++++++打开视频流start+++++++++++++++++++++++++
	enum v4l2_buf_type type;
	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (ioctl(video_fd, VIDIOC_STREAMON, &type) < 0)
	{
		printf("VIDIOC_STREAMON error\n");
		return 2;
	}
	//++++++++++++++++++++++++++++++打开视频流end+++++++++++++++++++++++++
	
	//---------------------读取视频源格式---------------------//	
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;				
	if (ioctl(video_fd, VIDIOC_G_FMT, &fmt) < 0)	
	{
		printf("get format failed\n");
		return 2 ;
	}
	else
	{
		printf("Picture:Width = %d   Height = %d\n", fmt.fmt.pix.width, fmt.fmt.pix.height);
		
	}
	//-------------------------------------------------------//
	
	int i=0;	

	//++++++++++++++++++++++++直接使用FB设备显示结果start++++++++++++++++++++++++++++++
	//一些关于fb设备或者没有用到的变量---------------------------
	/*FILE * fd_y_file = 0;
	int a=0;
	int k = 0;
	int i=0;
	//设置显卡设备framebuffer------------------------------------
	struct jpeg_decompress_struct cinfo;
	struct jpeg_error_mgr jerr;
	FILE *infile;//Jpeg文件的句柄
	unsigned char *buffer;
	char *fb_device;
	unsigned int x;
	unsigned int y;
	//打开显卡设备------------------------------------------------
	if ((fb = open("/dev/fb0", O_RDWR)) < 0)
	{
		perror(__func__);
		return 2;
	}

	//获取framebuffer的状态-----------------------------------------
	fb_set(fb);//设置显存参数	
	fb_stat(fb);//获取显卡驱动中的长、宽和显示位宽
	
	printf("frame buffer: %dx%d,  %dbpp, 0x%xbyte= %d,graylevels= %d \n", 
		fbdev.fb_width, fbdev.fb_height, fbdev.fb_bpp, fbdev.fb_size, fbdev.fb_size,fbdev.fb_gray);

	//映射framebuffer的地址到用户空间----------------------------------
	fbdev.fb_mem = mmap (NULL, fbdev.fb_size, PROT_READ|PROT_WRITE,MAP_SHARED,fb,0);
	fbdev.fb = fb;
	*/
	//++++++++++++++++++++++++直接使用FB设备显示结果end++++++++++++++++++++++++++++++
		
	//预览采集到的图像（如果有需要可以添加capture功能）-------------------
	while (sdl_quit)
	{
		
		fd_set fds;//文件描述符集，准备使用Select机制
		struct timeval tv;
		int ret1;
		//++++++++++++++++++++IO select start++++++++++++++++++++++++++
		FD_ZERO(&fds);//清空文件描述符集
		FD_SET(video_fd,&fds);//将视频设备文件的描述符放入集合中
		
		//消息等待超时,可以完全阻塞-------------------------------
		tv.tv_sec =5;
		tv.tv_usec=0;
		//等待视频设备准备好--------------------------------------
		ret1=select(video_fd+1,&fds,NULL,NULL,&tv);
		if(-1==ret1)
		{
			if(EINTR==errno)
				continue;
			printf("select error. \n");
			exit(EXIT_FAILURE);
		}
		if(0==ret1)
		{
			printf("select timeout. \n");
			continue;
		}
		//++++++++++++++++++++IO select end++++++++++++++++++++++++++
	
		while(sdl_quit)		
		{
					 
			//检测退出消息
			while(SDL_PollEvent(&sdlevent))
			{
				if(sdlevent.type == SDL_QUIT)
				{
					sdl_quit = 0;
					break;
				}
			}
			currtime = SDL_GetTicks();
			if(currtime - lasttime >0)
				frmrate = 1000/(currtime-lasttime);
			lasttime = currtime;

			//开始获取FIFO中已经准备好的一帧数据-----------------------		
			memset(&buf ,0,sizeof(buf));
			buf.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
			buf.memory=V4L2_MEMORY_MMAP;
			//准备好的出列--------------------------------------------
			ret1=ioctl (video_fd,VIDIOC_DQBUF,&buf);
			if(ret1!=0)
			{					
				printf("Lost the video \n");					
			}	
			//从FIFO中数据获取完成------------------------------------

	
			//获取当前帧的用户空间首地址，用于格式转换------------------
			unsigned char *ptcur=buffers[buf.index].start;
			
			//++++++++++++++++++++++++++++++++++++++++
			//算法区
			//+++++++++++++++++++++++++++++++++++++++++
			//灰度变换
			/*
			unsigned char *pgray = NULL;
			pgray = (unsigned char *)calloc(1,fmt.fmt.pix.width*fmt.fmt.pix.height*2*sizeof(unsigned char));//避免被识别为段错误
			yuv2gray(ptcur,pgray,fmt.fmt.pix.width, fmt.fmt.pix.height);
			*/
			//YUV向RGB（24bit）转换
			YUYVToRGB888(ptcur, pRGB, fmt.fmt.pix.width, fmt.fmt.pix.height);
			
			cvSetData(img, pRGB, fmt.fmt.pix.width*3);     //将pRGB数据装入img中
			//运动检测联合测试
			update_mhi(img,motion,30);
			/*//opencv 检测人脸
			cvCvtColor(img, imggray, CV_RGB2GRAY);         //将img灰度转换到imggray,供opencv检测使用
			CvHaarClassifierCascade *cascade=(CvHaarClassifierCascade*)cvLoad("/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml", storage,0,0);
			cvClearMemStorage(storage);
			cvEqualizeHist(imggray, imggray);
			CvSeq* objects = cvHaarDetectObjects(imggray, cascade, storage, 1.1, 2, 0, cvSize(30,30),cvSize(500,500));
			//opencv 标记人脸
			CvScalar colors[] = {{{255,0,0}},{{0,0,0}}};
			int faces=0;
			printf("%d\n",objects->total);
			for(faces=0; faces < (objects ? objects->total:0); faces++)
			{
				CvRect* r = (CvRect *)cvGetSeqElem(objects,faces);
				cvRectangle(img, cvPoint(r->x, r->y), cvPoint(r->x+r->width, r->y+r->height),colors[0],2,8,0 );
			}*/
			//调整opencv img图像数据
			/*CvScalar s;
			int imgi=0,imgj=0,sdlcount=0;
			for(imgi=0;imgi<img->height;imgi++)
			{
				for(imgj=0; imgj<img->width; imgj++)
				{
					s=cvGet2D(img,imgi,imgj);
					pRGB[sdlcount++]=0xff;//s.val[0];//B
					pRGB[sdlcount++]=0xff;//s.val[1];//G
					pRGB[sdlcount++]=0xff;//s.val[2];//R
					//cvSet2D(img,imgi,imgj,s);
				}
			}
			*/
			//opencv 显示图像	
			cvShowImage("image", img);
			tmpcount++;
			if(cvWaitKey(10)>0)
				sdl_quit=0;
			
			//+++++++++++++++++++++++SDL显示start+++++++++++++++++++++++++++++++
			//yuv载入到SDL
			/*
			SDL_LockYUVOverlay(overlay);
			memcpy(p, pgray,pscreen->w*(pscreen->h)*2);
			SDL_UnlockYUVOverlay(overlay);
			SDL_DisplayYUVOverlay(overlay, &drect);
			*/

			//RGB载入到SDL
			//memcpy(pixels, pRGB, pscreen_RGB->w*(pscreen_RGB->h)*3);
			//SDL_BlitSurface(pscreen_RGB, NULL, display_RGB, NULL);
			//SDL_Flip(display_RGB);

			//统计帧率
			//status = (char *)calloc(1,20*sizeof(char));
			//sprintf(status, "Fps:%d",frmrate);
			//SDL_WM_SetCaption(status, NULL);
			//SDL_Delay(10);
			//++++++++++++++++++++++SDL显示end++++++++++++++++++++++++++++++

			//用完了的入列--------------------------------------------
			ret1=ioctl (video_fd,VIDIOC_QBUF,&buf);
			if(ret1!=0)
			{					
				printf("Lost the video \n");					
			}
			if( catchflag ==1)
			{
				printf("S:Target:(%d,%d)",tx,ty);
				//小图像到大图像的坐标映射
				tx = tx*X_F/X;
				ty = Y_F - ty*Y_F/Y;
				lx = tx;
				ly = ty;
				printf("Zooming...\n");
				if((ret1=restartdev(&video_fd,&buffers,&req,&buf,X_F,Y_F)) != 0 )
					return ret1;
				
        			for( tmpcount = 0; tmpcount < N; tmpcount++ ) 
				{
            				cvReleaseImage( &mbuf[tmpcount] );
            				mbuf[tmpcount] = cvCreateImage( cvSize(X,Y), IPL_DEPTH_8U, 1 );
            				cvZero( mbuf[tmpcount] );
        			}
				//开始获取FIFO中已经准备好的一帧数据-----------------------		
		                delayN = DELAYN;	
				while( catchflag == 1 ||delayN !=0)
				{
					catchflag = 0;
					memset(&buf ,0,sizeof(buf));
					buf.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
					buf.memory=V4L2_MEMORY_MMAP;
					//准备好的出列--------------------------------------------
					ret1=ioctl (video_fd,VIDIOC_DQBUF,&buf);
					if(ret1!=0)
					{					
						printf("Lost the video \n");					
					}	
					//从FIFO中数据获取完成------------------------------------

	
					printf("...\n");
					//获取当前帧的用户空间首地址，用于格式转换------------------
					unsigned char *tmpptcur=buffers[buf.index].start;
				
					unsigned char *pCUT = NULL;
					pCUT= (unsigned char *)calloc(1,X*Y*3*sizeof(unsigned char));
					if(cutinterest(tmpptcur,pCUT,tx,ty,X_F,Y_F,X,Y)==1)
					{
						printf("Not the Force field.\n");
					}
					else
					{
					
						//YUV向RGB（24bit）转换
						YUYVToRGB888(pCUT, pRGB, X, Y);
						cvSetData(img, pRGB, X*3);     //将pRGB数据装入img中
						update_mhi(img,motion,30);
						printf("catchflag:%d.\n",catchflag);
						if(catchflag == 1)
						{
							delayN=DELAYN;
							if(tx<X/2-DX || tx>X/2+DX || ty<Y/2-DY || ty>Y/2+DY)//是否出了监视窗口
							{
								catchflag = 1;
							}
							else
							{
								catchflag = 0;
							}
						}
						cvShowImage("image",img);
						//用完了的入列--------------------------------------------
						ret1=ioctl (video_fd,VIDIOC_QBUF,&buf);
						if(ret1!=0)
						{					
							printf("Lost the video \n");					
						}
						if(cvWaitKey(2)>0)
							sdl_quit=0;
						printf("NOW:catchflag:%d\n",catchflag);
						if(catchflag == 1)
						{	
							ax = tx;
							ay = ty;
							//目标坐标截取图像到大图像的坐标映射
							tx = lx-X/2+tx;
							ty = ly-Y/2+ty;
							lx = tx;
							ly = ty;
							printf("L:New taget:(%d,%d)",tx,ty);
						}
						else if (catchflag == 0 && delayN !=1)
						{
							delayN--;
							tx = lx;
							ty = ly;
						}
						else 
						{
							printf("out Zoom...\n");
							//图像回到小模式
							if((ret1=restartdev(&video_fd,&buffers,&req,&buf,X,Y)) != 0 )
								return ret1;
        						for( tmpcount = 0; tmpcount < N; tmpcount++ ) 
							{
            							cvReleaseImage( &mbuf[tmpcount] );
            							mbuf[tmpcount] = cvCreateImage( cvSize(X,Y), IPL_DEPTH_8U, 1 );
            							cvZero( mbuf[tmpcount] );
        						}
							delayN = 0;
						}printf("delayN:%d.\n",delayN);
					}
				}
			
			
			}
		}	
	}	

	//fb_munmap(fbdev.fb_mem, fbdev.fb_size);	//释放framebuffer映射
	//close(fb);                                    //关闭Framebuffer设备
	for(i=0;i<req.count;i++)
	{
		if(-1==munmap(buffers[i].start,buffers[i].length))
			printf("munmap error:%d \n",i);
	}

	cvDestroyWindow("image");
	close(video_fd);					
	SDL_DestroyMutex(affmutex);
	//SDL_FreeYUVOverlay(overlay);
	cvReleaseImage(&img);
	cvReleaseImage(&imggray);
	free(status);
	free(buffers);
	SDL_Quit();
	return 0;

}

int main(int argc, char **argv)
{
	int ret;
	int i;	
	while(1){
	//-------------------------------------------	
	ret=video_fb_init_preview();
	switch (ret)//不同等级的清理------
	{
		case 1: 
			close(video_fd);
			printf("clean level 1.\n");
			break;
		case 2: for(i=0;i<3;i++)
			{
				if(-1==munmap(buffers[i].start,buffers[i].length))
					printf("munmap error:%d \n",i);
			}
			close(video_fd);
			printf("clean level 2.\n");
			break;
		case 3: fb_munmap(fbdev.fb_mem, fbdev.fb_size);	//释放framebuffer映射
			close(fb);                              //关闭Framebuffer设备
			for(i=0;i<3;i++)
			{
				if(-1==munmap(buffers[i].start,buffers[i].length))
					printf("munmap error:%d \n",i);
			}
			close(video_fd);		
			printf("clean level 3.\n");			
			break;
		default:printf("unkown return .\n");
			break;
	}
	exit(1);                                              //去除quit时进入死循环	
	}
	return 0;
}


