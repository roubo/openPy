/************************************************************************
* Copyright(c) 2011  Yang Xian
* All rights reserved.
*
* File:	main.cpp
* Brief: 利用级联分类器(基于haar特征)实现的人脸+人眼的目标检测
* Version: 1.0
* Author: Yang Xian
* Email: yang_xian521@163.com
* Date:	2011/11/16
* History:
************************************************************************/
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

#include <iostream>
#include <stdio.h>

using namespace std;
using namespace cv;

/** Function Headers */
void detectAndDisplay( Mat frame );

/** Global variables */
//-- Note, either copy these two files from opencv/data/haarscascades to your current folder, or change these locations
String face_cascade_name = "haarcascade_frontalface_alt.xml";
String eyes_cascade_name = "haarcascade_eye_tree_eyeglasses.xml";
CascadeClassifier face_cascade;
CascadeClassifier eyes_cascade;
string window_name = "Capture - Face detection";
RNG rng(12345);

int main( int argc, const char** argv )
{
	Mat frame;

	//-- 1. Load the cascades
	if( !face_cascade.load( face_cascade_name ) ){ printf("--(!)Error loading\n"); return -1; };
	if( !eyes_cascade.load( eyes_cascade_name ) ){ printf("--(!)Error loading\n"); return -1; };

	//-- 2. Read the video stream
// 	CvCapture* capture = cvCaptureFromCAM( -1 );	// 摄像头读取文件开关
	VideoCapture capture("Sample.avi");

	if( capture.isOpened()/*capture*/ )	// 摄像头读取文件开关
	{
		while( true )
		{
// 			frame = cvQueryFrame( capture );	// 摄像头读取文件开关
			capture >> frame;

			//-- 3. Apply the classifier to the frame
			if( !frame.empty() )
			{ detectAndDisplay( frame ); }
			else
			{ printf(" --(!) No captured frame -- Break!"); break; }

			int c = waitKey(10);
			if( (char)c == 'c' ) { break; } 

		}
	}
	return 0;
}

/**
* @function detectAndDisplay
*/
void detectAndDisplay( Mat frame )
{
	std::vector<Rect> faces;
	Mat frame_gray;

	cvtColor( frame, frame_gray, CV_BGR2GRAY );
	equalizeHist( frame_gray, frame_gray );
	//-- Detect faces
	face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );

	for( size_t i = 0; i < faces.size(); i++ )
	{
		Point center( int(faces[i].x + faces[i].width*0.5), int(faces[i].y + faces[i].height*0.5) );
		ellipse( frame, center, Size( int(faces[i].width*0.5), int(faces[i].height*0.5)), 0, 0, 360, Scalar( 255, 0, 255 ), 2, 8, 0 );

		Mat faceROI = frame_gray( faces[i] );
		std::vector<Rect> eyes;

		//-- In each face, detect eyes
		eyes_cascade.detectMultiScale( faceROI, eyes, 1.1, 2, 0 |CV_HAAR_SCALE_IMAGE, Size(30, 30) );

		for( size_t j = 0; j < eyes.size(); j++ )
		{
			Point center( int(faces[i].x + eyes[j].x + eyes[j].width*0.5), int(faces[i].y + eyes[j].y + eyes[j].height*0.5) ); 
			int radius = cvRound( (eyes[j].width + eyes[i].height)*0.25 );
			circle( frame, center, radius, Scalar( 255, 0, 0 ), 3, 8, 0 );
		}
	} 
	//-- Show what you got
	imshow( window_name, frame );
}