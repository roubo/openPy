#ifdef _CH_
#pragma package <opencv>
#endif

#define CV_NO_BACKWARD_COMPATIBILITY

#ifndef _EiC
#include "cv.h"
#include "highgui.h"
#include <math.h>
#endif

#define w 500
#if 0
#define IMG_WIDTH       352
#define IMG_HEIGHT      288
#else
#define IMG_WIDTH       704
#define IMG_HEIGHT      576
#endif

#define RGB_SIZE        (IMG_WIDTH * IMG_HEIGHT * 3)
#define Y_SIZE          (IMG_WIDTH * IMG_HEIGHT)
#define UV_SIZE         (IMG_WIDTH * IMG_HEIGHT / 4)
#define COLOR_CHN       3 

#ifndef max
#define max(a,b) ({typeof(a) _a = (a); typeof(b) _b = (b); _a > _b ? _a : _b; })
#define min(a,b) ({typeof(a) _a = (a); typeof(b) _b = (b); _a < _b ? _a : _b; })
#endif

#define HIGH_GUI_ON 0

//int levels = 3;
int levels = 100;
CvSeq* contours = 0;
CvSeq* contourstemp = 0;
int count = 0;


//const int bytes_per_pixel = 2;
static void color_convert_common(const unsigned char *pY, const unsigned char *pU, const unsigned char *pV, int width, int height, unsigned char *buffer, int grey)
{

  int i, j;
  int nR, nG, nB;
  int nY, nU, nV;
  unsigned char *out = buffer;
  int offset = 0;
  int uvStep = 0;

  if (grey)
  {
    memcpy(out,pY,width*height*sizeof(unsigned char));
  }
  else
    // YUV 4:2:0
    for (i = 0; i < height; i++)
    {
      uvStep = i / 2 * width / 2;
      for (j = 0; j < width; j++)
      {
        nY = *(pY + i * width + j);
        //nV = *(pUV + (i / 2) * width + bytes_per_pixel * (j / 2));
        //nU = *(pUV + (i / 2) * width + bytes_per_pixel * (j / 2) + 1);
        nU = *(pU + uvStep + j / 2);
        nV = *(pV + uvStep + j / 2);

        // Yuv Convert
        nY -= 16;
        nU -= 128;
        nV -= 128;

        if (nY < 0)
          nY = 0;

        nB = (int)(1192 * nY + 2066 * nU);
        nG = (int)(1192 * nY - 833 * nV - 400 * nU);
        nR = (int)(1192 * nY + 1634 * nV);

        nR = min(262143, max(0, nR));
        nG = min(262143, max(0, nG));
        nB = min(262143, max(0, nB));

        nR >>= 10;
        nR &= 0xff;
        nG >>= 10;
        nG &= 0xff;
        nB >>= 10;
        nB &= 0xff;

#if 0
        out[offset++] = (unsigned char)nR;
        out[offset++] = (unsigned char)nG;
        out[offset++] = (unsigned char)nB;
#else
        out[offset++] = (unsigned char)nB;
        out[offset++] = (unsigned char)nG;
        out[offset++] = (unsigned char)nR;
#endif
      }
    }

}

void on_trackbar(int pos)
{
#if HIGH_GUI_ON
    //IplImage* cnt_img = cvCreateImage( cvSize(w,w), 8, 3 );
    IplImage* cnt_img = cvCreateImage( cvSize(IMG_WIDTH, IMG_HEIGHT), IPL_DEPTH_8U, 3);
#endif

    CvSeq* _contours = contours;
    int _levels = levels - 3;
    CvRect rect; 
    CvPoint pt1, pt2;

    contourstemp = contours;
    while(contourstemp)
    {
        printf("total : %d\n", contourstemp->total);
        printf("elem_size : %d\n", contourstemp->elem_size);
        printf("delta_elems : %d\n", contourstemp->delta_elems);
        printf("flags : %x\n", contourstemp->flags);
        printf("header_size : %d\n", contourstemp->header_size);

        rect = cvBoundingRect(contourstemp, 1);
        printf("rec x : %d\n", rect.x);
        printf("rec y : %d\n", rect.y);
        printf("rec width : %d\n", rect.width);
        printf("rec height : %d\n", rect.height);

#if HIGH_GUI_ON
        pt1 = cvPoint(rect.x, rect.y);
        pt2 = cvPoint(rect.x+rect.width, rect.y+rect.height);
        cvRectangle(cnt_img, pt1, pt2, CV_RGB(255,0,0),1);
        cvLine(cnt_img, pt1, pt2, cvScalar(0,255,0), 1);
#endif


        //cvDrawContours(cnt_img,contourstemp,CV_RGB(255,255,255),CV_RGB(255,255,255),0,1,8,cvPoint(0,0));

        count ++;
        printf("\ncount : %d\n\n\n", count);
        contourstemp = contourstemp->h_next;
    }

    //if( _levels <= 0 ) // get to the nearest face to make it look more funny
      //  _contours = _contours->h_next->h_next->h_next;
    //cvZero( cnt_img );
    //cvDrawContours( cnt_img, _contours, CV_RGB(255,0,0), CV_RGB(0,255,0), _levels, 3, CV_AA, cvPoint(0,0) );

#if HIGH_GUI_ON
    cvNamedWindow( "cntimage", 1 );
    cvShowImage( "cntimage", cnt_img );
    cvReleaseImage( &cnt_img );
#endif
}

int main( int argc, char** argv )
{
    int i, j;
    CvMemStorage* storage = cvCreateMemStorage(0);
    //IplImage* img = cvCreateImage( cvSize(w,w), 8, 1 );
    IplImage* img = cvCreateImageHeader(cvSize(IMG_WIDTH, IMG_HEIGHT), IPL_DEPTH_8U, COLOR_CHN);
    IplImage* gryimg = cvCreateImage(cvSize(IMG_WIDTH, IMG_HEIGHT), IPL_DEPTH_8U, 1);

    //cvZero( img );

#if 1
    unsigned char y_buf[Y_SIZE];
    unsigned char u_buf[UV_SIZE];
    unsigned char v_buf[UV_SIZE];
    unsigned char rgb_buf[RGB_SIZE];
#if 0
    FILE *fp = fopen("352_288_p420_1.yuv", "rb");
#else
    FILE *fp = fopen("704_576_p420_novideo.yuv", "rb");
#endif
    int ret = 0;

    ret = fread(y_buf, 1, Y_SIZE, fp);
    if(ret < 0)
    {
        printf("ERR !! fread : %d", ret);
        return -1;
    }
    else
        printf("read y %d : %d bytes\n", Y_SIZE, ret);

    ret = fread(u_buf, 1, UV_SIZE, fp);
    if(ret < 0)
eteImage   {
        printf("ERR !! fread : %d", ret);
        return -1;
    }
    else
        printf("read u %d : %d bytes\n", UV_SIZE, ret);

    ret = fread(v_buf, 1, UV_SIZE, fp);
    if(ret < 0)
    {
        printf("ERR !! fread : %d", ret);
        return -1;
    }
    else
        printf("read v %d : %d bytes\n", UV_SIZE, ret);

    color_convert_common(y_buf, u_buf, v_buf, IMG_WIDTH, IMG_HEIGHT, rgb_buf, 0);

    cvSetData(img, rgb_buf, IMG_WIDTH * COLOR_CHN);

#endif

#if HIGH_GUI_ON
    cvNamedWindow( "image", 1 );
    cvShowImage( "image", img );
    cvWaitKey(0);
#endif

    cvCvtColor(img, gryimg, CV_BGR2GRAY );

#if HIGH_GUI_ON
    cvNamedWindow( "grayimage", 1 );
    cvShowImage( "grayimage", gryimg );
    cvWaitKey(0);
#endif

    cvFindContours( gryimg, storage, &contours, sizeof(CvContour),
                    CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0) );

    on_trackbar(0);

#if HIGH_GUI_ON
    cvWaitKey(0);
#endif

    cvReleaseMemStorage( &storage );
    //cvReleaseImage( &img );
    cvReleaseImage( &gryimg );
    cvReleaseImageHeader( &img );

    return 0;
}

#ifdef _EiC
main(1,"");
#endif
