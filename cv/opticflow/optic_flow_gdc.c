/*
 * Copyright (C) 2014 Hann Woei Ho
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

/*
 * @file paparazzi/sw/ext/ardrone2_vision/cv/opticflow/optic_flow_gdc.c
 * @brief optical-flow based hovering for Parrot AR.Drone 2.0
 *
 * Sensors from vertical camera and IMU of Parrot AR.Drone 2.0
 */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "optic_flow_gdc.h"
#include "defs_and_types.h"
#include "nrutil.h"
#include "../../modules/OpticFlow/opticflow_module.h"

#define int_index(x,y) (y * IMG_WIDTH + x)
#define uint_index(xx, yy) (((yy * IMG_WIDTH + xx) * 2) & 0xFFFFFFFC)
#define NO_MEMORY -1
#define OK 0
#define N_VISUAL_INPUTS 51
#define N_ACTIONS 3
#define MAX_COUNT_PT 50

unsigned int IMG_WIDTH, IMG_HEIGHT;

void multiplyImages(int* ImA, int* ImB, int* ImC, int width, int height)
{
  int x,y;
  unsigned int ix;

  // printf("W = %d, H = %d\n\r", IMG_WIDTH, IMG_HEIGHT);

  for(x = 0; x < width; x++)
  {
    for(y = 0; y < height; y++)
    {
      ix = (y * width + x);
      ImC[ix] = ImA[ix] * ImB[ix];
      // If we want to keep the values in [0, 255]:
      // ImC[ix] /= 255;
    }
  }
}

void getImageDifference(int* ImA, int* ImB, int* ImC, int width, int height)
{
  int x,y;
  unsigned int ix;

  // printf("W = %d, H = %d\n\r", IMG_WIDTH, IMG_HEIGHT);

  for(x = 0; x < width; x++)
  {
    for(y = 0; y < height; y++)
    {
      ix = (y * width + x);
      ImC[ix] = ImA[ix] - ImB[ix];
    }
  }

}

void getSubPixel_gray(int* Patch, unsigned char* frame_buf, int center_x, int center_y, int half_window_size, int subpixel_factor)
{
  int x, y, x_0, y_0, x_0_or, y_0_or, i, j, window_size, alpha_x, alpha_y, max_x, max_y;
  //int printed, limit;
  unsigned int ix1, ix2, Y;
  window_size = half_window_size * 2 + 1;
  max_x = (IMG_WIDTH-1)*subpixel_factor;
  max_y = (IMG_HEIGHT-1)*subpixel_factor;
  //printed = 0; limit = 4;

  for(i = 0; i < window_size; i++)
  {
    for(j = 0; j < window_size; j++)
    {
      // index for this position in the patch:
      ix1 = (j * window_size + i);

      // determine subpixel coordinates of the current pixel:
      x = center_x + (i - half_window_size) * subpixel_factor;
      if(x < 0) x = 0;
      if(x > max_x) x = max_x;
      y = center_y + (j - half_window_size) * subpixel_factor;
      if(y < 0) y = 0;
      if(y > max_y) y = max_y;
      // pixel to the top left:
      x_0_or = (x / subpixel_factor);
      x_0 = x_0_or * subpixel_factor;
      y_0_or = (y / subpixel_factor);
      y_0 = y_0_or * subpixel_factor;
      /*if(printed < limit)
			{
				printf("x_0_or = %d, y_0_or = %d;\n\r", x_0_or, y_0_or);
				printf("x_0 = %d, y_0 = %d\n\r");
				printed++;
			}*/


      if(x == x_0 && y == y_0)
      {
        // simply copy the pixel:
//        ix2 = uint_index(x_0_or, y_0_or);
//          Y = ((unsigned int)frame_buf[ix2+1] + (unsigned int)frame_buf[ix2+3]) >> 1;
    	ix2 = y_0_or * IMG_WIDTH + x_0_or;
        Y = (unsigned int)frame_buf[ix2+1];
        Patch[ix1] = (int) Y;
      }
      else
      {
        // blending according to how far the subpixel coordinates are from the pixel coordinates
        alpha_x = (x - x_0);
        alpha_y = (y - y_0);

        // the patch pixel is a blend from the four surrounding pixels:
    	ix2 = y_0_or * IMG_WIDTH + x_0_or;
        Y = (unsigned int)frame_buf[ix2+1];
//        ix2 = uint_index(x_0_or, y_0_or);
//        Y = ((unsigned int)frame_buf[ix2+1] + (unsigned int)frame_buf[ix2+3]) >> 1;
        Patch[ix1] = (subpixel_factor - alpha_x) * (subpixel_factor - alpha_y) * ((int) Y);

    	ix2 = y_0_or * IMG_WIDTH + (x_0_or + 1);
        Y = (unsigned int)frame_buf[ix2+1];
//        ix2 = uint_index((x_0_or+1), y_0_or);
//        Y = ((unsigned int)frame_buf[ix2+1] + (unsigned int)frame_buf[ix2+3]) >> 1;
        //if(printed < limit) printf("subpixel: TR = %d\n\r", Y);
        Patch[ix1] += alpha_x * (subpixel_factor - alpha_y) * ((int) Y);

    	ix2 = (y_0_or + 1) * IMG_WIDTH + x_0_or;
        Y = (unsigned int)frame_buf[ix2+1];
//        ix2 = uint_index(x_0_or, (y_0_or+1));
//        Y = ((unsigned int)frame_buf[ix2+1] + (unsigned int)frame_buf[ix2+3]) >> 1;
        //if(printed < limit) printf("subpixel: BL = %d\n\r", Y);
        Patch[ix1] += (subpixel_factor - alpha_x) * alpha_y * ((int) Y);

    	ix2 = (y_0_or + 1) * IMG_WIDTH + (x_0_or + 1);
        Y = (unsigned int)frame_buf[ix2+1];
//        ix2 = uint_index((x_0_or+1), (y_0_or+1));
//        Y = ((unsigned int)frame_buf[ix2+1] + (unsigned int)frame_buf[ix2+3]) >> 1;
        //if(printed < limit) printf("subpixel: BR = %d\n\r", Y);
        Patch[ix1] += alpha_x * alpha_y * ((int) Y);

        // normalize patch value
        Patch[ix1] /= (subpixel_factor * subpixel_factor);

        /*if(printed < limit)
				{

					printf("alpha_x = %d, alpha_y = %d, x_0 = %d, y_0 = %d, x = %d, y = %d, Patch[ix1] = %d\n\r", alpha_x, alpha_y, x_0, y_0, x, y, Patch[ix1]);
					// printed++;
				}
         */

      }
    }
  }

  return;
}

void getGradientPatch(int* Patch, int* DX, int* DY, int half_window_size)
{
  unsigned int ix1, ix2;
  int x, y, padded_patch_size, patch_size, Y1, Y2;
  //	int printed; printed = 0;

  padded_patch_size = 2 * (half_window_size + 1)+ 1;
  patch_size = 2 * half_window_size + 1;
  // currently we use [0 0 0; -1 0 1; 0 0 0] as mask for dx
  for(x = 1; x < padded_patch_size - 1; x++)
  {
    for(y = 1; y < padded_patch_size - 1; y++)
    {
      // index in DX, DY:
      ix2 = (unsigned int) ((y-1) * patch_size + (x-1));

      ix1 = (unsigned int) (y * padded_patch_size + x-1);
      Y1 = Patch[ix1];
      ix1 = (unsigned int) (y * padded_patch_size + x+1);
      Y2 = Patch[ix1];
//      DX[ix2] = Y2 - Y1;
      DX[ix2] = (Y2 - Y1)/2;

      ix1 = (unsigned int) ((y-1) * padded_patch_size + x);
      Y1 = Patch[ix1];
      ix1 = (unsigned int) ((y+1) * padded_patch_size + x);
      Y2 = Patch[ix1];
//      DY[ix2] = Y2 - Y1;
      DY[ix2] = (Y2 - Y1)/2;

      /*if(printed < 1 && DX[ix2] > 0)
			{
				printf("DX = %d, DY = %d\n\r", DX[ix2], DY[ix2]);
				printed++;
			}
			else if(printed == 1 && DX[ix2] < 0)
			{
				printf("DX = %d, DY = %d\n\r", DX[ix2], DY[ix2]);
				printed++;
			}*/


    }
  }

  return;
}

int getSumPatch(int* Patch, int size)
{
  int x, y, sum; // , threshold
  unsigned int ix;

  // in order to keep the sum within range:
  //threshold = 50000; // typical values are far below this threshold

  sum = 0;
  for(x = 0; x < size; x++)
  {
    for(y = 0; y < size; y++)
    {
      ix = (y * size) + x;
      //if(sum < threshold && sum > -threshold)
      //{
      sum += Patch[ix]; // do not check thresholds
      //}
      /*else
			{
				if(sum > threshold)
				{
					sum = threshold;
				}
				else
				{
					sum = -threshold;
				}
			}*/
    }
  }

  return sum;
}

int calculateG(int* G, int* DX, int* DY, int half_window_size)
{
  int patch_size;
  int* DXX; int* DXY; int* DYY;

  patch_size = 2 * half_window_size + 1;

  // allocate memory:
  DXX = (int *) malloc(patch_size * patch_size * sizeof(int));
  DXY = (int *) malloc(patch_size * patch_size * sizeof(int));
  DYY = (int *) malloc(patch_size * patch_size * sizeof(int));

  if(DXX == 0 || DXY == 0 || DYY == 0)
    return NO_MEMORY;

  // then determine the second order gradients
  multiplyImages(DX, DX, DXX, patch_size, patch_size);
  multiplyImages(DX, DY, DXY, patch_size, patch_size);
  multiplyImages(DY, DY, DYY, patch_size, patch_size);

  // calculate G:
  G[0] = getSumPatch(DXX, patch_size);
  G[1] = getSumPatch(DXY, patch_size);
  G[2] = G[1];
  G[3] = getSumPatch(DYY, patch_size);

  // free memory:
  free((char*) DXX); free((char*) DXY); free((char*) DYY);

  // no errors:
  return OK;
}



int calculateError(int* ImC, int width, int height)
{
  int x,y, error;
  unsigned int ix;

  error = 0;

  for(x = 0; x < width; x++)
  {
    for(y = 0; y < height; y++)
    {
      ix = (y * width + x);
      error += ImC[ix]*ImC[ix];
    }
  }

  return error;
}

int opticFlowLK(unsigned char * new_image_buf, unsigned char * old_image_buf, int* p_x, int* p_y, int n_found_points, int imW, int imH, int* new_x, int* new_y, int* status, int half_window_size, int max_iterations)
{
	// A straightforward one-level implementation of Lucas-Kanade.
	// For all points:
	// (1) determine the subpixel neighborhood in the old image
	// (2) get the x- and y- gradients
	// (3) determine the 'G'-matrix [sum(Axx) sum(Axy); sum(Axy) sum(Ayy)], where sum is over the window
	// (4) iterate over taking steps in the image to minimize the error:
	//     [a] get the subpixel neighborhood in the new image
	//     [b] determine the image difference between the two neighborhoods
	//     [c] calculate the 'b'-vector
	//     [d] calculate the additional flow step and possibly terminate the iteration
	int p, subpixel_factor, x, y, it, step_threshold, step_x, step_y, v_x, v_y, Det;
	int b_x, b_y, patch_size, padded_patch_size, error;
	unsigned int ix1, ix2;
	int* I_padded_neighborhood; int* I_neighborhood; int* J_neighborhood;
	int* DX; int* DY; int* ImDiff; int* IDDX; int* IDDY;
	int G[4];
	int error_threshold;

	// set the image width and height
	IMG_WIDTH = imW;
	IMG_HEIGHT = imH;
	// spatial resolution of flow is 1 / subpixel_factor
	subpixel_factor = 10;
	// determine patch sizes and initialize neighborhoods
	patch_size = (2*half_window_size + 1);
	error_threshold = (25 * 25) * (patch_size * patch_size);

	padded_patch_size = (2*half_window_size + 3);
	I_padded_neighborhood = (int *) malloc(padded_patch_size * padded_patch_size * sizeof(int));
	I_neighborhood = (int *) malloc(patch_size * patch_size * sizeof(int));
	J_neighborhood = (int *) malloc(patch_size * patch_size * sizeof(int));
	if(I_padded_neighborhood == 0 || I_neighborhood == 0 || J_neighborhood == 0)
		return NO_MEMORY;

	DX = (int *) malloc(patch_size * patch_size * sizeof(int));
	DY = (int *) malloc(patch_size * patch_size * sizeof(int));
	IDDX = (int *) malloc(patch_size * patch_size * sizeof(int));
	IDDY = (int *) malloc(patch_size * patch_size * sizeof(int));
	ImDiff = (int *) malloc(patch_size * patch_size * sizeof(int));
	if(DX == 0 || DY == 0 || ImDiff == 0 || IDDX == 0 || IDDY == 0)
		return NO_MEMORY;

	for(p = 0; p < n_found_points; p++)
	{
		//printf("*** NEW POINT ***\n\r");
		// status: point is not yet lost:
		status[p] = 1;

		//printf("Normal coordinate: (%d,%d)\n\r", p_x[p], p_y[p]);
		// We want to be able to take steps in the image of 1 / subpixel_factor:
		p_x[p] *= subpixel_factor;
		p_y[p] *= subpixel_factor;
		//printf("Subpixel coordinate: (%d,%d)\n\r", p_x[p], p_y[p]);

		// if the pixel is outside the ROI in the image, do not track it:
		if(!(p_x[p] > ((half_window_size+1) * subpixel_factor) && p_x[p] < (IMG_WIDTH-half_window_size) * subpixel_factor && p_y[p] > ((half_window_size+1) * subpixel_factor) && p_y[p] < (IMG_HEIGHT-half_window_size)*subpixel_factor))
		{
//			printf("Outside of ROI, P1[%d,%d]\n\r",p_x[p],p_y[p]);
			status[p] = 0;
		}

		// (1) determine the subpixel neighborhood in the old image
		// we determine a padded neighborhood with the aim of subsequent gradient processing:
		getSubPixel_gray(I_padded_neighborhood, old_image_buf, p_x[p], p_y[p], half_window_size+1, subpixel_factor);

		// Also get the original-sized neighborhood
		for(x = 1; x < padded_patch_size - 1; x++)
		{
		  for(y = 1; y < padded_patch_size - 1; y++)
		  {
			ix1 = (y * padded_patch_size + x);
			ix2 = ((y-1) * patch_size + (x-1));
			I_neighborhood[ix2] = I_padded_neighborhood[ix1];
		  }
		}

		// (2) get the x- and y- gradients
		getGradientPatch(I_padded_neighborhood, DX, DY, half_window_size);

		// (3) determine the 'G'-matrix [sum(Axx) sum(Axy); sum(Axy) sum(Ayy)], where sum is over the window
		error = calculateG(G, DX, DY, half_window_size);
		if(error == NO_MEMORY) return NO_MEMORY;

		for(it = 0; it < 4; it++)
		{
	//		printf("G[%d] = %d\n\r", it, G[it]);
			G[it] /= 255; // to keep values in range
	//		printf("G[%d] = %d\n\r", it, G[it]);
		}
		// calculate G's determinant:
		Det = G[0] * G[3] - G[1] * G[2];
		//printf("Det = %d\n\r", Det);
		Det = Det / subpixel_factor; // so that the steps will be expressed in subpixel units
		//printf("Det = %d\n\r", Det);
		if(Det < 1)
		{
			status[p] = 0;
//			printf("irrevertible G\n");
		}

		// (4) iterate over taking steps in the image to minimize the error:
		it = 0;
		step_threshold = 2; // 0.2 as smallest step (L1)
		v_x = 0;
		v_y = 0;
		step_x = step_threshold + 1;
		step_y = step_threshold + 1;

		while(status[p] == 1 && it < max_iterations && (abs(step_x) >= step_threshold || abs(step_y) >= step_threshold))
		{
		  //printf("it = %d, (p_x+v_x,p_y+v_y) = (%d,%d)\n\r", it, p_x[p]+v_x, p_y[p]+v_y);
		  //printf("it = %d;", it);
		  // if the pixel goes outside the ROI in the image, stop tracking:
		  if(!(p_x[p]+v_x > ((half_window_size+1) * subpixel_factor) && p_x[p]+v_x < ((int)IMG_WIDTH-half_window_size) * subpixel_factor && p_y[p]+v_y > ((half_window_size+1) * subpixel_factor) && p_y[p]+v_y < ((int)IMG_HEIGHT-half_window_size)*subpixel_factor))
		  {
//			printf("Outside of ROI, P1[%d,%d]\n\r",p_x[p],p_y[p]);
			status[p] = 0;
			break;
		  }

		  //     [a] get the subpixel neighborhood in the new image
		  // clear J:
		  for(x = 0; x < patch_size; x++)
		  {
			for(y = 0; y < patch_size; y++)
			{
			  ix2 = (y * patch_size + x);
			  J_neighborhood[ix2] = 0;
			}
		  }


		  getSubPixel_gray(J_neighborhood, new_image_buf, p_x[p]+v_x, p_y[p]+v_y, half_window_size, subpixel_factor);
		  //     [b] determine the image difference between the two neighborhoods
		  //printf("I = ");
		  //printIntMatrix(I_neighborhood, patch_size, patch_size);
		  //printf("J = ");
		  //printIntMatrix(J_neighborhood, patch_size, patch_size);
		  //getSubPixel(J_neighborhood, new_image_buf, subpixel_factor * ((p_x[p]+v_x)/subpixel_factor), subpixel_factor * ((p_y[p]+v_y) / subpixel_factor), half_window_size, subpixel_factor);
		  //printf("J2 = ");
		  //printIntMatrix(J_neighborhood, patch_size, patch_size);
		  //printf("figure(); subplot(1,2,1); imshow(I/255); subplot(1,2,2); imshow(J/255);\n\r");
		  getImageDifference(I_neighborhood, J_neighborhood, ImDiff, patch_size, patch_size);
		  //printf("ImDiff = ");
		  //printIntMatrix(ImDiff, patch_size, patch_size);
		  error = calculateError(ImDiff, patch_size, patch_size)/255;

//	      if(error > error_threshold) printf("error threshold\n");
		  if(error > error_threshold && it > max_iterations / 2)
		  {
			status[p] = 0;
//			printf("occlusion\n");
			break;
		  }
		  //printf("error(%d) = %d;\n\r", it+1, error);
		  //     [c] calculate the 'b'-vector
		  //printf("DX = ");
		  //printIntMatrix(DX, patch_size, patch_size);
		  multiplyImages(ImDiff, DX, IDDX, patch_size, patch_size);
		  //printf("IDDX = ");
		  //printIntMatrix(IDDX, patch_size, patch_size);
		  multiplyImages(ImDiff, DY, IDDY, patch_size, patch_size);
		  //printf("DY = ");
		  //printIntMatrix(DY, patch_size, patch_size);
		  //printf("IDDY = ");
		  //printIntMatrix(IDDY, patch_size, patch_size);
		  //printf("figure(); subplot(2,3,1); imagesc(ImDiff); subplot(2,3,2); imagesc(DX); subplot(2,3,3); imagesc(DY);");
		  //printf("subplot(2,3,4); imagesc(IDDY); subplot(2,3,5); imagesc(IDDX);\n\r");
		  // division by 255 to keep values in range:
		  b_x = getSumPatch(IDDX, patch_size) / 255;
		  b_y = getSumPatch(IDDY, patch_size) / 255;
		  //printf("b_x = %d; b_y = %d;\n\r", b_x, b_y);
		  //     [d] calculate the additional flow step and possibly terminate the iteration
		  step_x = (G[3] * b_x - G[1] * b_y) / Det;
		  step_y = (G[0] * b_y - G[2] * b_x) / Det;
		  v_x += step_x;
		  v_y += step_y; // - (?) since the origin in the image is in the top left of the image, with y positive pointing down
		  //printf("step = [%d,%d]; v = [%d,%d];\n\r", step_x, step_y, v_x, v_y);
		  //printf("pause(0.5);\n\r");
		  // next iteration
		  it++;
	//      step_size = abs(step_x);
	//      step_size += abs(step_y);
		  //printf("status = %d, it = %d, step_size = %d\n\r", status[p], it, step_size);
		} // iteration to find the right window in the new image

		//printf("figure(); plot(error(1:(it+1)));\n\r");
//	printf("it = %d\n",it);
		new_x[p] = (p_x[p] + v_x) / subpixel_factor;
		new_y[p] = (p_y[p] + v_y) / subpixel_factor;
		p_x[p] /= subpixel_factor;
		p_y[p] /= subpixel_factor;
  }



  // free all allocated variables:
  free((int*) I_padded_neighborhood);
  free((int*) I_neighborhood);
  free((int*) J_neighborhood);
  free((int*) DX);
  free((int*) DY);
  free((int*) ImDiff);
  free((int*) IDDX);
  free((int*) IDDY);
  // no errors:
  return OK;
}

void quick_sort (float *a, int n)
{
    if (n < 2)
        return;
    float p = a[n / 2];
    float *l = a;
    float *r = a + n - 1;
    while (l <= r)
    {
        if (*l < p)
        {
            l++;
            continue;
        }
        if (*r > p)
        {
            r--;
            continue; // we need to check the condition (l <= r) every time we change the value of l or r
        }
        float t = *l;
        *l++ = *r;
        *r-- = t;
    }
    quick_sort(a, r - a + 1);
    quick_sort(l, a + n - l);
}

void quick_sort_int(int *a, int n)
{
    if (n < 2)
        return;
    int p = a[n / 2];
    int *l = a;
    int *r = a + n - 1;
    while (l <= r)
    {
        if (*l < p)
        {
            l++;
            continue;
        }
        if (*r > p)
        {
            r--;
            continue;
        }
        int t = *l;
        *l++ = *r;
        *r-- = t;
    }
    quick_sort_int(a, r - a + 1);
    quick_sort_int(l, a + n - l);
}

void CvtYUYV2Gray(unsigned char *grayframe, unsigned char *frame, int imW, int imH)
{
    int x, y;
    unsigned char *Y, *gray;
    for (y = 0; y < imH; y++) {
        Y = frame + (imW * 2 * y) + 1;
        gray = grayframe + (imW * y);
        for (x=0; x < imW; x += 2) {
            gray[x] = *Y;
            Y += 2;
            gray[x + 1] = *Y;
            Y += 2;
        }
    }
}

unsigned int OF_buf_point = 0;
unsigned int OF_buf_point2 = 0;
float x_avg, y_avg, x_buf[24], y_buf[24], x_buf2[24], y_buf2[24];

void OFfilter(float *OFx, float *OFy, float dx, float dy, int count, int OF_FilterType)
{

	if(OF_FilterType == 1) //1. moving average 2. moving median
	{

		x_avg = 0.0;
		y_avg = 0.0;

		if(count)
		{
			x_buf[OF_buf_point] = dx;
			y_buf[OF_buf_point] = dy;
		}
		else
		{
			x_buf[OF_buf_point] = 0.0;
			y_buf[OF_buf_point] = 0.0;
		}
		OF_buf_point = (OF_buf_point+1) %20;

		for (int i=0;i<20;i++) {
			x_avg+=x_buf[i]*0.05;
			y_avg+=y_buf[i]*0.05;
		}

		*OFx = x_avg;
		*OFy = y_avg;

	}
	else if(OF_FilterType == 2)
	{
		if(count)
		{
			x_buf2[OF_buf_point2] = dx;
			y_buf2[OF_buf_point2] = dy;
		}
		else
		{
			x_buf2[OF_buf_point2] = 0.0;
			y_buf2[OF_buf_point2] = 0.0;
		}
		OF_buf_point2 = (OF_buf_point2+1) %11; // 11

		quick_sort(x_buf2,11); // 11
		quick_sort(y_buf2,11); // 11

		*OFx = x_buf2[6]; // 6
		*OFy = y_buf2[6]; // 6
	}
	else
	{
		printf("no filter type selected!\n");
	}
}

// **********************************************************************************************************************
// Flow Field Fitting
// **********************************************************************************************************************

void MatVVMul(float* MVec, float** Mat, float* Vec, int MatW, int MatH)
{
  unsigned int i;
  unsigned int j;

  for(i = 0; i < MatH; i++)
  {
    for(j = 0; j < MatW; j++)
    {
    	MVec[i] += Mat[i][j] * Vec[j];
    }
  }
}

void ScaleAdd(float* Mat3, float* Mat1, float Scale, float* Mat2, int MatW, int MatH)
{
  unsigned int i;
  unsigned int j;
  unsigned int ii;

  for(i = 0; i < MatW; i++)
  {
    for(j = 0; j < MatH; j++)
    {
      ii = (j * MatW + i);
      Mat3[ii] = Scale*Mat1[ii] + Mat2[ii];
    }
  }
}
static float PYTHAG(float a, float b);
float PYTHAG(float a, float b)
{
    float at = fabs(a), bt = fabs(b), ct, result;

    if (at > bt)       { ct = bt / at; result = at * sqrt(1.0 + ct * ct); }
    else if (bt > 0.0) { ct = at / bt; result = bt * sqrt(1.0 + ct * ct); }
    else result = 0.0;
    return(result);
}

int dsvd(float **a, int m, int n, float *w, float **v)
{
    int flag, i, its, j, jj, k, l, nm;
    float c, f, h, s, x, y, z;
    float anorm = 0.0, g = 0.0, scale = 0.0;
    float *rv1;

    if (m < n)
    {
        fprintf(stderr, "#rows must be > #cols \n");
        return(0);
    }

    rv1 = (float *)malloc((unsigned int) n*sizeof(float));

/* Householder reduction to bidiagonal form */
    for (i = 0; i < n; i++)
    {
        /* left-hand reduction */
        l = i + 1;
        rv1[i] = scale * g;
        g = s = scale = 0.0;
        if (i < m)
        {
            for (k = i; k < m; k++)
                scale += fabs((float)a[k][i]);
            if (scale)
            {
                for (k = i; k < m; k++)
                {
                    a[k][i] = (float)((float)a[k][i]/scale);
                    s += ((float)a[k][i] * (float)a[k][i]);
                }
                f = (float)a[i][i];
                g = -SIGN(sqrt(s), f);
                h = f * g - s;
                a[i][i] = (float)(f - g);
                if (i != n - 1)
                {
                    for (j = l; j < n; j++)
                    {
                        for (s = 0.0, k = i; k < m; k++)
                            s += ((float)a[k][i] * (float)a[k][j]);
                        f = s / h;
                        for (k = i; k < m; k++)
                            a[k][j] += (float)(f * (float)a[k][i]);
                    }
                }
                for (k = i; k < m; k++)
                    a[k][i] = (float)((float)a[k][i]*scale);
            }
        }
        w[i] = (float)(scale * g);

        /* right-hand reduction */
        g = s = scale = 0.0;
        if (i < m && i != n - 1)
        {
            for (k = l; k < n; k++)
                scale += fabs((float)a[i][k]);
            if (scale)
            {
                for (k = l; k < n; k++)
                {
                    a[i][k] = (float)((float)a[i][k]/scale);
                    s += ((float)a[i][k] * (float)a[i][k]);
                }
                f = (float)a[i][l];
                g = -SIGN(sqrt(s), f);
                h = f * g - s;
                a[i][l] = (float)(f - g);
                for (k = l; k < n; k++)
                    rv1[k] = (float)a[i][k] / h;
                if (i != m - 1)
                {
                    for (j = l; j < m; j++)
                    {
                        for (s = 0.0, k = l; k < n; k++)
                            s += ((float)a[j][k] * (float)a[i][k]);
                        for (k = l; k < n; k++)
                            a[j][k] += (float)(s * rv1[k]);
                    }
                }
                for (k = l; k < n; k++)
                    a[i][k] = (float)((float)a[i][k]*scale);
            }
        }
        anorm = MAX(anorm, (fabs((float)w[i]) + fabs(rv1[i])));
    }

    /* accumulate the right-hand transformation */
    for (i = n - 1; i >= 0; i--)
    {
        if (i < n - 1)
        {
            if (g)
            {
                for (j = l; j < n; j++)
                    v[j][i] = (float)(((float)a[i][j] / (float)a[i][l]) / g);
                    /* float division to avoid underflow */
                for (j = l; j < n; j++)
                {
                    for (s = 0.0, k = l; k < n; k++)
                        s += ((float)a[i][k] * (float)v[k][j]);
                    for (k = l; k < n; k++)
                        v[k][j] += (float)(s * (float)v[k][i]);
                }
            }
            for (j = l; j < n; j++)
                v[i][j] = v[j][i] = 0.0;
        }
        v[i][i] = 1.0;
        g = rv1[i];
        l = i;
    }

    /* accumulate the left-hand transformation */
    for (i = n - 1; i >= 0; i--)
    {
        l = i + 1;
        g = (float)w[i];
        if (i < n - 1)
            for (j = l; j < n; j++)
                a[i][j] = 0.0;
        if (g)
        {
            g = 1.0 / g;
            if (i != n - 1)
            {
                for (j = l; j < n; j++)
                {
                    for (s = 0.0, k = l; k < m; k++)
                        s += ((float)a[k][i] * (float)a[k][j]);
                    f = (s / (float)a[i][i]) * g;
                    for (k = i; k < m; k++)
                        a[k][j] += (float)(f * (float)a[k][i]);
                }
            }
            for (j = i; j < m; j++)
                a[j][i] = (float)((float)a[j][i]*g);
        }
        else
        {
            for (j = i; j < m; j++)
                a[j][i] = 0.0;
        }
        ++a[i][i];
    }

    /* diagonalize the bidiagonal form */
    for (k = n - 1; k >= 0; k--)
    {                             /* loop over singular values */
        for (its = 0; its < 30; its++)
        {                         /* loop over allowed iterations */
            flag = 1;
            for (l = k; l >= 0; l--)
            {                     /* test for splitting */
                nm = l - 1;
                if (fabs(rv1[l]) + anorm == anorm)
                {
                    flag = 0;
                    break;
                }
                if (fabs((float)w[nm]) + anorm == anorm)
                    break;
            }
            if (flag)
            {
                c = 0.0;
                s = 1.0;
                for (i = l; i <= k; i++)
                {
                    f = s * rv1[i];
                    if (fabs(f) + anorm != anorm)
                    {
                        g = (float)w[i];
                        h = PYTHAG(f, g);
                        w[i] = (float)h;
                        h = 1.0 / h;
                        c = g * h;
                        s = (- f * h);
                        for (j = 0; j < m; j++)
                        {
                            y = (float)a[j][nm];
                            z = (float)a[j][i];
                            a[j][nm] = (float)(y * c + z * s);
                            a[j][i] = (float)(z * c - y * s);
                        }
                    }
                }
            }
            z = (float)w[k];
            if (l == k)
            {                  /* convergence */
                if (z < 0.0)
                {              /* make singular value nonnegative */
                    w[k] = (float)(-z);
                    for (j = 0; j < n; j++)
                        v[j][k] = (-v[j][k]);
                }
                break;
            }
            if (its >= 30) {
                free((void*) rv1);
                fprintf(stderr, "No convergence after 30,000! iterations \n");
                return(0);
            }

            /* shift from bottom 2 x 2 minor */
            x = (float)w[l];
            nm = k - 1;
            y = (float)w[nm];
            g = rv1[nm];
            h = rv1[k];
            f = ((y - z) * (y + z) + (g - h) * (g + h)) / (2.0 * h * y);
            g = PYTHAG(f, 1.0);
            f = ((x - z) * (x + z) + h * ((y / (f + SIGN(g, f))) - h)) / x;

            /* next QR transformation */
            c = s = 1.0;
            for (j = l; j <= nm; j++)
            {
                i = j + 1;
                g = rv1[i];
                y = (float)w[i];
                h = s * g;
                g = c * g;
                z = PYTHAG(f, h);
                rv1[j] = z;
                c = f / z;
                s = h / z;
                f = x * c + g * s;
                g = g * c - x * s;
                h = y * s;
                y = y * c;
                for (jj = 0; jj < n; jj++)
                {
                    x = (float)v[jj][j];
                    z = (float)v[jj][i];
                    v[jj][j] = (float)(x * c + z * s);
                    v[jj][i] = (float)(z * c - x * s);
                }
                z = PYTHAG(f, h);
                w[j] = (float)z;
                if (z)
                {
                    z = 1.0 / z;
                    c = f * z;
                    s = h * z;
                }
                f = (c * g) + (s * y);
                x = (c * y) - (s * g);
                for (jj = 0; jj < m; jj++)
                {
                    y = (float)a[jj][j];
                    z = (float)a[jj][i];
                    a[jj][j] = (float)(y * c + z * s);
                    a[jj][i] = (float)(z * c - y * s);
                }
            }
            rv1[l] = 0.0;
            rv1[k] = f;
            w[k] = (float)x;
        }
    }
    free((void*) rv1);
    return(1);
}

void svbksb(float **u, float *w, float **v, int m, int n, float *b, float *x)
{
	int jj, j, i;
	float s, *tmp;//, *vector(int nl, int nh);
	//void free_vector();

	tmp = vector(1,n);
	for(j=0; j<n; j++)
	{
		s = 0.0;
		if(w[j])
		{
			for(i=0; i<m; i++)
			{
				s += u[i][j]*b[i];
			}
			s /= w[j];
		}
		tmp[j] = s;
	}
	for(j=0; j<n; j++)
	{
		s = 0.0;
		for(jj=0; jj<n; jj++)
		{
			s += v[j][jj]*tmp[jj];
		}
		x[j] = s;
	}
	free_vector(tmp, 1, n);
}

void svdSolve(float *x_svd, float **u, int m, int n, float *b)
{
	// SVD
	int i, j;

	float *w, **v, **u_copy, *b_copy;
	w = (float *)malloc((unsigned int) n*sizeof(float));
	v = (float **)malloc((unsigned int) n*sizeof(float*));
	b_copy = (float *)malloc((unsigned int) m*sizeof(float));
	u_copy = (float **)malloc((unsigned int) m*sizeof(float*));
	for(i=0; i<n; i++) v[i] = (float *)malloc(n*sizeof(float));

	int ii;
	for(ii=0; ii<m; ii++)
	{
		u_copy[ii] = (float *)malloc(n*sizeof(float));
		u_copy[ii][0] = u[ii][0];
		u_copy[ii][1] = u[ii][1];
		u_copy[ii][2] = u[ii][2];
		b_copy[ii] =  b[ii];
		//printf("%d,%f,%f,%f,%f\n",ii,u_copy[ii][0] ,u_copy[ii][1] ,u_copy[ii][2] ,b_copy[ii]);
	}
//printf("svdSolve stop 1\n");
	dsvd(u_copy, m, n, w, v);
	//printf("SVD_DONE = %d\n",SVD_DONE);

/*	for (i=0;i<m;i++)
	{
		for(j=0;j<n;j++)
		{
			printf("%f ",u_copy[i][j]);
		}
		printf("\n");
	}*/

	// LS Solution
	float wmax, wmin;
	wmax = 0.0;
	for(j=0; j<n; j++)
	{
		if(w[j] > wmax)
		{
			wmax = w[j];
		}
	}

	wmin = wmax*1.0e-6;

	for(j=0; j<n; j++)
	{
		if(w[j] < wmin)
		{
			w[j] = 0.0;
		}
	}
//printf("svdSolve stop 2\n");
	svbksb(u_copy, w, v, m, n, b_copy, x_svd);
	for(ii=0; ii<m; ii++) free(u_copy[ii]);
	for(ii=0; ii<n; ii++) free(v[ii]);
	free(w);
	free(v);
	free(u_copy);
	free(b_copy);
//printf("svdSolve stop 3\n");
}

void fitLinearFlowField(float* pu, float* pv, float* divergence_error, int *x, int *y, int *dx, int *dy, int count, int n_samples, float* min_error_u, float* min_error_v, int n_iterations, float error_threshold, int *n_inlier_minu, int *n_inlier_minv)
{
//	printf("count=%d, n_sample=%d, n_iterations=%d, error_threshold=%f\n",count,n_samples,n_iterations,error_threshold);
//	for (int i=0; i<count;i++) {
//		printf("%d_%d, ",dx[i],dy[i]);
//	}
//	printf("\n");
		int *sample_indices;
		float **A, *bu, *bv, **AA, *bu_all, *bv_all;
		sample_indices =(int *) calloc(n_samples,sizeof(int));
		A = (float **) calloc(n_samples,sizeof(float*));// A1 is a N x 3 matrix with rows [x, y, 1]
		bu = (float *) calloc(n_samples,sizeof(float)); // bu is a N x 1 vector with elements dx (or dy)
		bv = (float *) calloc(n_samples,sizeof(float)); // bv is a N x 1 vector with elements dx (or dy)
		AA = (float **) calloc(count,sizeof(float*));   // AA contains all points with rows [x, y, 1]
		bu_all = (float *) calloc(count,sizeof(float)); // bu is a N x 1 vector with elements dx (or dy)
		bv_all = (float *) calloc(count,sizeof(float)); // bv is a N x 1 vector with elements dx (or dy)
		int si, add_si, p, i_rand, sam;
		for(sam = 0; sam < n_samples; sam++) A[sam] = (float *) calloc(3,sizeof(float));
		pu[0] = 0.0f; pu[1] = 0.0f; pu[2] = 0.0f;
		pv[0] = 0.0f; pv[1] = 0.0f; pv[2] = 0.0f;
		//		int n_inliers;
		float * PU, * errors_pu, * PV, * errors_pv;
		int * n_inliers_pu, * n_inliers_pv;
		PU = (float *) calloc(n_iterations*3,sizeof(float));
		PV = (float *) calloc(n_iterations*3,sizeof(float));
		errors_pu = (float *) calloc(n_iterations,sizeof(float));
		errors_pv = (float *) calloc(n_iterations,sizeof(float));
		n_inliers_pu = (int *) calloc(n_iterations,sizeof(int));
		n_inliers_pv = (int *) calloc(n_iterations,sizeof(int));

		float *bb, *C;
		bb = (float *) calloc(count,sizeof(float));
		C = (float *) calloc(count,sizeof(float));

		// initialize matrices and vectors for the full point set problem:
		// this is used for determining inliers
		for(sam = 0; sam < count; sam++)
		{
			AA[sam] = (float *) calloc(3,sizeof(float));
			AA[sam][0] = (float) x[sam];
			AA[sam][1] = (float) y[sam];
			AA[sam][2] = 1.0f;
			bu_all[sam] = (float) dx[sam];
			bv_all[sam] = (float) dy[sam];
		}

		// perform RANSAC:
		int it, ii;
		for(it = 0; it < n_iterations; it++)
		{
			// select a random sample of n_sample points:
			memset(sample_indices, 0, n_samples*sizeof(int));
			i_rand = 0;
//printf("stop1\n");
			while(i_rand < n_samples)
			{
				si = rand() % count;
				add_si = 1;
				for(ii = 0; ii < i_rand; ii++)
				{
					if(sample_indices[ii] == si) add_si = 0;
				}
				if(add_si)
				{
					sample_indices[i_rand] = si;
					i_rand ++;
				}
			}
//printf("stop2\n");
			// Setup the system:
			for(sam = 0; sam < n_samples; sam++)
			{
				A[sam][0] = (float) x[sample_indices[sam]];
				A[sam][1] = (float) y[sample_indices[sam]];
				A[sam][2] = 1.0f;
				bu[sam] = (float) dx[sample_indices[sam]];
				bv[sam] = (float) dy[sample_indices[sam]];
				//printf("%d,%d,%d,%d,%d\n",A[sam][0],A[sam][1],A[sam][2],bu[sam],bv[sam]);
			}
//printf("stop3\n");
			// Solve the small system:
/*            int i;
			for(i=0;i<3;i++)
			{
				printf("pu%d = %f, pv%d = %f\t",i,pu[i],i,pv[i]);
			}
			printf("\n");*/

			// for horizontal flow:
			svdSolve(pu, A, n_samples, 3, bu);
			PU[it*3] = pu[0];
			PU[it*3+1] = pu[1];
			PU[it*3+2] = pu[2];

			// for vertical flow:
			svdSolve(pv, A, n_samples, 3, bv);
			PV[it*3] = pv[0];
			PV[it*3+1] = pv[1];
			PV[it*3+2] = pv[2];

/*			int i;
			for(i=0;i<3;i++)
			{
				printf("pu%d = %f, pv%d = %f\t",i,pu[i],i,pv[i]);
			}
			printf("\n");*/

//printf("stop4\n");
			// count inliers and determine their error:
			errors_pu[it] = 0;
			errors_pv[it] = 0;
			n_inliers_pu[it] = 0;
			n_inliers_pv[it] = 0;

			// for horizontal flow:

			MatVVMul(bb, AA, pu, 3, count);
			float scaleM;
			scaleM = -1.0;
			ScaleAdd(C, bb, scaleM, bu_all, 1, count);

			for(p = 0; p < count; p++)
			{
//				printf("h=%f ",C[p]);
				if(C[p] < error_threshold)
				{
					errors_pu[it] += abs(C[p]);
					n_inliers_pu[it]++;
				}
			}
			// for vertical flow:
			MatVVMul(bb, AA, pv, 3, count);
			ScaleAdd(C, bb, scaleM, bv_all, 1, count);

//			printf("\n");

			for(p = 0; p < count; p++)
			{
//				printf("v=%f ",C[p]);
				if(C[p] < error_threshold)
				{
					errors_pv[it] += abs(C[p]);
					n_inliers_pv[it]++;
				}
			}
//			printf("\n");
		}
//printf("stop5\n");

		// select the parameters with lowest error:
		// for horizontal flow:
		int param;
		int min_ind = 0;
		*min_error_u = (float)errors_pu[0];
		for(it = 1; it < n_iterations; it++)
		{
			if(errors_pu[it] < *min_error_u)
			{
				*min_error_u = (float)errors_pu[it];
				min_ind = it;
			}
		}
		for(param = 0; param < 3; param++)
		{
			pu[param] = PU[min_ind*3+param];
		}
		//printf("pu_sel=%f,%f,%f\n",pu[0],pu[1],pu[2]);
		// for vertical flow:
		min_ind = 0;
		*min_error_v = (float)errors_pv[0];

		for(it = 0; it < n_iterations; it++)
		{
			if(errors_pv[it] < *min_error_v)
			{
				*min_error_v = (float)errors_pv[it];
				min_ind = it;
			}
		}
		for(param = 0; param < 3; param++)
		{
			pv[param] = PV[min_ind*3+param];
		}
		*n_inlier_minu = n_inliers_pu[min_ind];
		*n_inlier_minv = n_inliers_pv[min_ind];
//printf("stop6\n");
		// error has to be determined on the entire set:
		MatVVMul(bb, AA, pu, 3, count);
		float scaleM;
		scaleM = -1.0;
		ScaleAdd(C, bb, scaleM, bu_all, 1, count);

		*min_error_u = 0;
		for(p = 0; p < count; p++)
		{
			*min_error_u += abs(C[p]);
		}
		MatVVMul(bb, AA, pv, 3, count);
		ScaleAdd(C, bb, scaleM, bv_all, 1, count);

		*min_error_v = 0;
		for(p = 0; p < count; p++)
		{
			*min_error_v += abs(C[p]);
		}
		*divergence_error = (*min_error_u + *min_error_v) / (2 * count);

		// delete allocated dynamic arrays
//printf("stop7\n");
		for(sam = 0; sam < n_samples; sam++) free(A[sam]);
		for(sam = 0; sam < count; sam++) free(AA[sam]);
		free(A);
		free(PU);
		free(PV);
		free(n_inliers_pu);
		free(n_inliers_pv);
		free(errors_pu);
		free(errors_pv);
		free(bu);
		free(bv);
		free(AA);
		free(bu_all);
		free(bv_all);
		free(bb);
		free(C);
		free(sample_indices);
//printf("stop8\n");
}

unsigned int mov_block = 15; //default: 30
float div_buf[30];
unsigned int div_point = 0;
float OFS_BUTTER_NUM_1 = 0.0004260;
float OFS_BUTTER_NUM_2 = 0.0008519;
float OFS_BUTTER_NUM_3 = 0.0004260;
float OFS_BUTTER_DEN_2 = -1.9408;
float OFS_BUTTER_DEN_3 = 0.9425;
float ofs_meas_dx_prev = 0.0;
float ofs_meas_dx_prev_prev = 0.0;
float ofs_filter_val_dx_prev = 0.0;
float ofs_filter_val_dx_prev_prev = 0.0;
float temp_divergence = 0.0;

void extractInformationFromLinearFlowField(float *divergence, float *mean_tti, float *median_tti, float *d_heading, float *d_pitch, float* pu, float* pv, int imgWidth, int imgHeight, int *DIV_FILTER)
{
		// divergence:
		*divergence = pu[0] + pv[1];
		// minimal measurable divergence:
		float minimal_divergence = 2E-3;
		if(abs(*divergence) > minimal_divergence)
		{
			*mean_tti = 2.0f / *divergence;
			if(FPS > 1E-3) *mean_tti /= FPS;
			else *mean_tti = ((2.0f / minimal_divergence) / FPS);
//			if(FPS > 1E-3) *mean_tti /= 60;
//			else *mean_tti = ((2.0f / minimal_divergence) / 60);
			*median_tti = *mean_tti;
		}
		else
		{
			*mean_tti = ((2.0f / minimal_divergence) / FPS);
//			*mean_tti = ((2.0f / minimal_divergence) / 60);
			*median_tti = *mean_tti;
		}

		// also adjust the divergence to the number of frames:
		*divergence = *divergence * FPS;
//		*divergence = *divergence * 60;

		// translation orthogonal to the camera axis:
		// flow in the center of the image:
		*d_heading = (-(pu[2] + (imgWidth/2.0f) * pu[0] + (imgHeight/2.0f) * pu[1]));
		*d_pitch = (-(pv[2] + (imgWidth/2.0f) * pv[0] + (imgHeight/2.0f) * pv[1]));

		//apply a moving average
		int medianfilter = 1;
		int averagefilter = 0;
		int butterworthfilter = 0;
		int kalmanfilter = 0;
		float div_avg = 0.0f;

		if(averagefilter == 1)
		{
			*DIV_FILTER = 1;
			if (*divergence < 3.0 && *divergence > -3.0) {
				div_buf[div_point] = *divergence;
				div_point = (div_point+1) %mov_block; // index starts from 0 to mov_block
			}

			int im;
			for (im=0;im<mov_block;im++) {
				div_avg+=div_buf[im];
			}
			*divergence = div_avg/ mov_block;
		}
		else if(medianfilter == 1)
		{
			*DIV_FILTER = 2;
			//apply a median filter
//			if (*divergence < 3.0 && *divergence > -3.0) {
				div_buf[div_point] = *divergence;
				div_point = (div_point+1) %15;
//			}
			quick_sort(div_buf,15);
			*divergence  = div_buf[8];
		}
		else if(butterworthfilter == 1)
		{
			*DIV_FILTER = 3;
			temp_divergence = *divergence;
			*divergence = OFS_BUTTER_NUM_1* (*divergence) + OFS_BUTTER_NUM_2*ofs_meas_dx_prev+ OFS_BUTTER_NUM_3*ofs_meas_dx_prev_prev- OFS_BUTTER_DEN_2*ofs_filter_val_dx_prev- OFS_BUTTER_DEN_3*ofs_filter_val_dx_prev_prev;
		    ofs_meas_dx_prev_prev = ofs_meas_dx_prev;
		    ofs_meas_dx_prev = temp_divergence;
		    ofs_filter_val_dx_prev_prev = ofs_filter_val_dx_prev;
		    ofs_filter_val_dx_prev = *divergence;
		}
		else if(kalmanfilter == 1)
		{
			*DIV_FILTER = 4;
		}


}

void slopeEstimation(float *z_x, float *z_y, float *three_dimensionality, float *POE_x, float *POE_y, float d_heading, float d_pitch, float* pu, float* pv, float min_error_u, float min_error_v)
{
	float v_prop_x, v_prop_y, threshold_slope, eta;

	// extract proportional velocities / inclination from flow field:
	v_prop_x  = d_heading;
	v_prop_y = d_pitch;
	threshold_slope = 1.0;
	eta = 0.002;
	if(abs(pv[1]) < eta && abs(v_prop_y) < threshold_slope && abs(v_prop_x) >= 2* threshold_slope)
	{
		// there is not enough vertical motion, but also no forward motion:
		*z_x = pu[0] / v_prop_x;
	}
	else if(abs(v_prop_y) >= 2 * threshold_slope)
	{
		// there is sufficient vertical motion:
		*z_x = pv[0] / v_prop_y;
	}
	else
	{
		// there may be forward motion, then we can do a quadratic fit:
		*z_x = 0.0f;
	}

	*three_dimensionality = min_error_v + min_error_u;

	if(abs(pu[0]) < eta && abs(v_prop_x) < threshold_slope && abs(v_prop_y) >= 2*threshold_slope)
	{
		// there is little horizontal movement, but also no forward motion, and sufficient vertical motion:
		*z_y = pv[1] / v_prop_y;
	}
	else if(abs(v_prop_x) >= 2*threshold_slope)
	{
		// there is sufficient horizontal motion:
		*z_y = pu[1] / v_prop_x;
	}
	else
	{
		// there could be forward motion, then we can do a quadratic fit:
		*z_y = 0.0f;
	}

	// Focus of Expansion:
	// the flow planes intersect the flow=0 plane in a line
	// the FoE is the point where these 2 lines intersect (flow = (0,0))
	// x:
	float denominator = pv[0]*pu[1] - pu[0]*pv[1];
	if(abs(denominator) > 1E-5)
	{
		*POE_x = ((pu[2]*pv[1] - pv[2] * pu[1]) / denominator);
	}
	else *POE_x = 0.0f;
	// y:
	denominator = pu[1];
	if(abs(denominator) > 1E-5)
	{
		*POE_y = (-(pu[0] * *POE_x + pu[2]) / denominator);
	}
	else *POE_y = 0.0f;
}

void analyseTTI(float *z_x, float *z_y, float *three_dimensionality, float *POE_x, float *POE_y, float *divergence, float *mean_tti, float *median_tti, float *d_heading, float *d_pitch, float *divergence_error, int *x, int *y, int *dx, int *dy, int *n_inlier_minu, int *n_inlier_minv, int count, int imW, int imH, int *DIV_FILTER)
{
		// linear fit of the optic flow field
		float error_threshold = 10; // 10
		int n_iterations = 20; // 20

		int n_samples = (count < 5) ? count : 5;

		// minimum = 3
		if(n_samples < 3)
		{
			// set dummy values for tti, etc.
			*mean_tti = 1000.0f / 60;
			*median_tti = *mean_tti;
			*d_heading = 0;
			*d_pitch = 0;
			*three_dimensionality = 0;
			return;
		}

		float pu[3], pv[3], min_error_u, min_error_v;

		fitLinearFlowField(pu, pv, divergence_error, x, y, dx, dy, count, n_samples, &min_error_u, &min_error_v, n_iterations, error_threshold, n_inlier_minu, n_inlier_minv);

		extractInformationFromLinearFlowField(divergence, mean_tti, median_tti, d_heading, d_pitch, pu, pv, imW, imH, DIV_FILTER);

		slopeEstimation(z_x, z_y, three_dimensionality, POE_x, POE_y, *d_heading, *d_pitch, pu, pv, min_error_u, min_error_v);

}
