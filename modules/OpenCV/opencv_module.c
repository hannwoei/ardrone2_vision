/*
 * Copyright (C) 2012-2013
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */


// Own header
#include "opencv_module.h"

// Threaded computer vision
#include <pthread.h>

#include <opencv2/core/core_c.h>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc/imgproc_c.h>

#include <stdio.h>


void opencv_module_init(void) {
	opencv_module_start();
}


volatile uint8_t computervision_thread_has_results = 0;

void opencv_module_run(void) {
  // Read Latest Vision Module Results
  if (computervision_thread_has_results)
  {
  }
}


pthread_t computervision_thread;
volatile uint8_t computervision_thread_status = 0;
volatile uint8_t computer_vision_thread_command = 0;
void *computervision_thread_main(void* data);
void *computervision_thread_main(void* data)
{
  //init
  IplImage* img = 0;
  IplImage* gray_img = 0;
  img = cvLoadImage("/data/video/example.jpg",CV_LOAD_IMAGE_COLOR);
  gray_img = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 1);
  cvCvtColor( img, gray_img, CV_RGB2GRAY);
  cvSaveImage("/data/video/gray_example.jpg",gray_img, 0);

  while (computer_vision_thread_command > 0)
  {

    // Process

    computervision_thread_has_results++;
  }
  printf("Thread Closed\n");
  
  computervision_thread_status = -100;
  return 0;
}

void opencv_module_start(void)
{
  computer_vision_thread_command = 1;
  int rc = pthread_create(&computervision_thread, NULL, computervision_thread_main, NULL);
  if(rc) {
    printf("ctl_Init: Return code from pthread_create(mot_thread) is %d\n", rc);
  }
}

void opencv_module_stop(void)
{
  computer_vision_thread_command = 0;
}


