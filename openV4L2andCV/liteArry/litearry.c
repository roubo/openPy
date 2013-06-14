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
#include"pthread.h"

#include <sys/ioctl.h> 
#include <stdint.h>
#include <asm/types.h>
#include <linux/videodev2.h>
#include <sys/mman.h>
#include <malloc.h>
#include <linux/fb.h>
//该型号仪器若需要拍摄保存图像文件，则需要JPEG库支持-----
//#include <jpeglib.h>
//#include <jerror.h>
//为了在非嵌入式设备上使用，这里使用了SDL
#include <X11/Xlib.h>
#include <SDL/SDL.h>

//opencv >2
#include "opencv.h"

#define FALSE		0
#define TRUE		1
#define WORDLEN		32


//视频设备状态-------------------------------------------
#define NOMAL   0
#define NG      1


static int video_fd;
static int fb;
char Data[512];
char sdl_quit = 1;
static unsigned int SDL_VIDEO_Flags = SDL_ANYFORMAT | SDL_DOUBLEBUF | SDL_RESIZABLE;




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
为了能实现能在16位的LCD上显示，转换RGB888为RGB565
**************************************************************/
unsigned short RGB888toRGB565(unsigned char red, unsigned char green, unsigned char blue)
{
	unsigned short B = (blue >> 3) & 0x001F;
	unsigned short G = ((green >> 2) << 5) & 0x07E0;
	unsigned short R = ((red >> 3) << 11) & 0xF800;

	return (unsigned short) (R | G | B);
}


//显示一个像素点的图像到framebuffer上，与上面函数可以对应使用（未使用）-------
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


//视频设备和显示设备初始化和预览函数（加设备状态检测）--------------------------------
int video_fb_init_preview()
{
	//串口相关变量-------------------------------
	char buff[512];
	int nread=0;
	int FrameDone=0;//一帧数据结束标志
	int FrameCount=0;//记录帧长度
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

	video_fd = open("/dev/video1", O_RDWR, 0);//打开摄像头设备，使用阻塞方式打开
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
		printf("%d> pixelformat =%c%c%c%c,description =%s\n",fmt0.index,fmt0.pixelformat&0xff,(fmt0.pixelformat>>8)&0xff,(fmt0.pixelformat>>16)&0xff,(fmt0.pixelformat>>24)&0xff,fmt0.description);
	}
	/**************************END***************************/
	
	//---------------------设置获取视频的格式----------------//
	struct v4l2_format fmt;	
	memset( &fmt, 0, sizeof(fmt));
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;	//视频数据流类型，永远都V4L2_BUF_TYPE_VIDEO_CAPTURE
	fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;//视频源的格式为JPEG或YUN4:2:2或RGB
	fmt.fmt.pix.width = 640;//设置视频宽度
	fmt.fmt.pix.height = 480;//设置视频高度
	//fmt.fmt.pix.field=V4L2_FIELD_INTERLACED;
	//fmt.fmt.pix.colorspace=8;
	//printf("color: %d \n",fmt.fmt.pix.colorspace);
	if (ioctl(video_fd, VIDIOC_S_FMT, &fmt) < 0)//使配置生效
	{
		printf("set format failed\n");
		return 2;
	}
	//-------------------------------------------------------//
	
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//SDL 设置:YUV输出
	if(SDL_Init(SDL_INIT_VIDEO) < 0)
	{
		printf("SDL Init failed.\n");
		exit(1);
	}
	pscreen = SDL_SetVideoMode(fmt.fmt.pix.width, fmt.fmt.pix.height,0,SDL_VIDEO_Flags);
	overlay = SDL_CreateYUVOverlay(fmt.fmt.pix.width, fmt.fmt.pix.height,SDL_YUY2_OVERLAY,pscreen);
	p = (unsigned char *)overlay->pixels[0];
	drect.x = 0;
	drect.y = 0;
	drect.w = pscreen->w;
	drect.h = pscreen->h;
	//SDL 设置:RGB输出
	pscreen = SDL_SetVideoMode(fmt.fmt.pix.width, fmt.fmt.pix.height, 32, SDL_SWSURFACE | SDL_DOUBLEBUF);
	rmask = 0x000000ff;
	gmask = 0x0000ff00;
	bmask = 0x00ff0000;
	amask = 0xff000000;
	bpp   = 32;
	pitch = fmt.fmt.pix.width*4;
	pixels_num = fmt.fmt.pix.width*fmt.fmt.pix.height*4;
	pixels = (unsigned char *)malloc(pixels_num);
	memset(pixels, 0, pixels_num);
	p_RGB = (unsigned char *)pixels;
	pscreen_RGB = SDL_CreateRGBSurfaceFrom(pixels, fmt.fmt.pix.width, fmt.fmt.pix.height, bpp, pitch, rmask, gmask, bmask, amask);

	
	lasttime = SDL_GetTicks();
	affmutex = SDL_CreateMutex();
	//SDL 设置end
	
	//openCV 设置
	CvMemStorage*  storage = cvCreateMemStorage(0);
	IplImage*      img     = cvCreateImageHeader(cvSize(fmt.fmt.pix.width,fmt.fmt.pix.height), IPL_DEPTH_8U, 3);//image头，未开辟数据空间
	IplImage*      imggray = cvCreateImage(cvSize(fmt.fmt.pix.width,fmt.fmt.pix.height), IPL_DEPTH_8U, 1);//image，开辟数据空间


	//openCV 设置 end

	//------------------------申请帧缓冲---------------------//
	struct v4l2_requestbuffers req;
	memset(&req, 0, sizeof (req));
	req.count = 3;	//缓存数量，即可保存的图片数量
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;	//数据流类型，永远都是V4L2_BUF_TYPE_VIDEO_CAPTURE
	req.memory = V4L2_MEMORY_MMAP;	//存储类型：V4L2_MEMORY_MMAP或V4L2_MEMORY_USERPTR
	if (ioctl(video_fd, VIDIOC_REQBUFS, &req) == -1)//使配置生效
	{
		perror("request buffer error \n");
		return 2;
	}
	//-------------------------------------------------------//
	
	//--------将VIDIOC_REQBUFS获取内存转为物理空间-------------//
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
		if (ioctl(video_fd, VIDIOC_QUERYBUF, &buf) < 0)//使配置生效
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
		if (ioctl(video_fd, VIDIOC_QBUF, &buf) < 0)//放入缓存队列
		{
			printf("VIDIOC_QBUF error\n");
			return 2;
		}

	}
	//-------------------------------------------------------//
	
	//----------------------开始视频显示----------------------//
	enum v4l2_buf_type type;
	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (ioctl(video_fd, VIDIOC_STREAMON, &type) < 0)
	{
		printf("VIDIOC_STREAMON error\n");
		return 2;
	}
	//-------------------------------------------------------//
	
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
		
	//预览采集到的图像（如果有需要可以添加capture功能）-------------------
	while (sdl_quit)
	{
		
		fd_set fds;//文件描述符集，准备使用Select机制
		struct timeval tv;
		int ret1;
		
		FD_ZERO(&fds);//清空文件描述符集
		FD_SET(video_fd,&fds);//将视频设备文件的描述符放入集合中
		
		//消息等待超时,可以完全阻塞-------------------------------
		tv.tv_sec =2;
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
	
			//获取当前帧的用户空间首地址，用于格式转换------------------
			unsigned char *ptcur=buffers[buf.index].start;
			//++++++++++++++++++++++++++++++++++++++++
			//算法区
			//+++++++++++++++++++++++++++++++++++++++++
			//灰度变换
			unsigned char *pgray = NULL;
			pgray = (unsigned char *)calloc(1,fmt.fmt.pix.width*fmt.fmt.pix.height*2*sizeof(unsigned char));//避免被识别为段错误
			yuv2gray(ptcur,pgray,fmt.fmt.pix.width, fmt.fmt.pix.height);
			
			//YUV向RGB（32bit）转换
			unsigned char *pRGB = NULL;
			pRGB = (unsigned char *)calloc(1,fmt.fmt.pix.width*fmt.fmt.pix.height*4*sizeof(unsigned char));
			YUYVToRGB888(ptcur, pRGB, fmt.fmt.pix.width, fmt.fmt.pix.height);
			
			//opencv 检测人脸
			cvSetData(img, pRGB, fmt.fmt.pix.width*3);//将pRGB数据装入img中
			cvCvtColor(img, imggray, CV_RGB2GRAY);//将img灰度转换到imggray,供opencv检测使用
			CvHaarClassifierCascade *cascade=(CvHaarClassifierCascade*)cvLoad("/usr/share/opencv-2.4.5/data/haarcascades/haarcascade_frontalface_alt2.xml", storage);
			cvClearMemStorage(storage);
			CvSeq* objects = cvHaarDetectObjects(imggray, cascade, storage, 1.1, 2, 0, cvSize(30,30));
			
			//opencv 标记人脸
			CvScalar colors[] = {{{255,0,0}},{{0,0,0}}};
			for(int faces=0; faces < (objects ? objects->total:0); faces++)
			{
				CvRect* r = (CvRect *)cvGetSeqElem(objects,faces);
				cvRectangle(img, cvPoint(r.x, r.y), cvPoint(r.x+r.width, r.y+r.height),colors[0] );//原始图像上加框
			}

			//yuv载入到SDL
			SDL_LockYUVOverlay(overlay);
			memcpy(p, pgray,pscreen->w*(pscreen->h)*2);
			SDL_UnlockYUVOverlay(overlay);
			SDL_DisplayYUVOverlay(overlay, &drect);


			//RGB载入到SDL
			memcpy(pixels, pgray, pscreen_RGB->w*(pscreen_RGB->h)*4);
			SDL_BlitSurface(pscreen_RGB, NULL, display_RGB, NULL);
			SDL_Flip(display_RGB);

			//统计帧率
			status = (char *)calloc(1,20*sizeof(char));
			sprintf(status, "Fps:%d",frmrate);
			SDL_WM_SetCaption(status, NULL);
			SDL_Delay(10);
			//用完了的入列--------------------------------------------
			ret1=ioctl (video_fd,VIDIOC_QBUF,&buf);
			if(ret1!=0)
			{					
				printf("Lost the video \n");					
			}
			
		}	
	}	

	//fb_munmap(fbdev.fb_mem, fbdev.fb_size);	//释放framebuffer映射
	//close(fb);//关闭Framebuffer设备
	for(i=0;i<req.count;i++)
	{
		if(-1==munmap(buffers[i].start,buffers[i].length))
			printf("munmap error:%d \n",i);
	}
	close(video_fd);					
	SDL_DestroyMutex(affmutex);
	SDL_FreeYUVOverlay(overlay);
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
			close(fb);//关闭Framebuffer设备
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
	exit(1);//去除quit时进入死循环	
	}
	return 0;
}


