
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Computer Vision
#include "trig.h"

// Own Header
#include "opencv_code.h"

// Downlink
#include "messages.h"
#include "subsystems/datalink/downlink.h"

// OpenCV header
//#include <opencv/cv.h>
#include <opencv2/core/core_c.h>
#include <opencv2/highgui/highgui_c.h>
//#include <opencv2/opencv.hpp>
//#include <imgproc/imgproc_c.h>
//#include <ml.h>
//#include <cxcore.h>

//#include "paparazzi.h"

// Settable by pluging
unsigned int imgWidth, imgHeight;
unsigned int verbose = 0;

// Local variables

// Called by plugin
void my_plugin_init(void)
{

}

void my_plugin_run(unsigned char *frame)
{
	printf("AAAAAAAAAAAAA\n");
	IplImage* img = 0;
	img = cvLoadImage("/data/video/example.jpg",CV_LOAD_IMAGE_UNCHANGED);
//	CvSize img_sz = cvGetSize(img);
//	IplImage* gray_img = cvCreateImage(img_sz, IPL_DEPTH_8U, 1);
//	cvCvtColor(img,gray_img,CV_BGR2GRAY);
//
//	cvReleaseImage( &img );
}

