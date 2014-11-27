#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "optic_flow_gdc.h"
#include "defs_and_types.h"
#include "nrutil.h"
//#include "opticflow/fastRosten.h"
#include "opticflow/fast9/fastRosten.h"
#include "../../modules/OpticFlow/opticflow_module.h"

#define int_index(x,y) (y * IMG_WIDTH + x)
#define uint_index(xx, yy) (((yy * IMG_WIDTH + xx) * 2) & 0xFFFFFFFC)
#define NO_MEMORY -1
#define OK 0
#define N_VISUAL_INPUTS 51
#define N_ACTIONS 3
#define MAX_COUNT_PT 50

unsigned int IMG_WIDTH, IMG_HEIGHT;
int weights[153] = {-78, -46, 18, 59, 0, 100, 0, 0, 100, -29, -45, 0, 15, -30, 59, -100, -99, -100, -47, 0, -100, -100, 2, -78, 0, 10, -68, 53, 0, 0, -61, -28, 51, 0, -86, -73, 10, -65, -100, 98, -19, 63, -100, -42, -83, 21, 0, 3, 7, 0, -100, 24, -100, -99, -40, -100, 91, 0, 0, 54, 0, -90, -22, 13, 6, 31, 0, 100, -58, -31, 100, 5, 21, -100, 37, -100, 57, 100, -96, -3, -74, -3, -64, -68, 6, -100, -71, -81, 100, 13, 100, 0, -100, -57, 77, -100, -61, -100, 0, 37, -100, -100, -100, 10, -36, -100, 62, 8, 0, 21, 2, -61, -5, 32, -64, 15, -100, -90, -74, -18, -22, -28, 42, -92, 0, 3, -3, -13, 100, -5, 88, 0, 7, -100, 90, 73, -53, 100, 0, 2, 0, -95, -60, -62, 0, -6, 82, 0, -79, -69, 73, -38, 100};

void getVisualInputs(unsigned char *frame_buf, int x, int y, int* visual_inputs, unsigned int half_patch);
void applyNeuralNetwork(int* visual_inputs, int* actions, int RESOLUTION);


static inline void bluePixel(unsigned char *frame_buf, unsigned int ip)
{
  frame_buf[ip] = 0xff;
  frame_buf[ip+1] = 0xff;
  frame_buf[ip+2] = 0x00;
  frame_buf[ip+3] = 0xff;
}

static inline void redPixel(unsigned char *frame_buf, unsigned int ip)
{
  frame_buf[ip] = 0x00;
  frame_buf[ip+1] = 0xff;
  frame_buf[ip+2] = 0xff;
  frame_buf[ip+3] = 0xff;
}

static inline void greenPixel(unsigned char *frame_buf, unsigned int ip)
{
  frame_buf[ip] = 0x00;
  frame_buf[ip+1] = 0xff;
  frame_buf[ip+2] = 0x00;
  frame_buf[ip+3] = 0xff;
}

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

void getGradientPixelWH(unsigned char *frame_buf, int x, int y, int* dx, int* dy)
{
  unsigned int ix, Y1, Y2;
  unsigned int xx, yy;
  // currently we use [0 0 0; -1 0 1; 0 0 0] as mask for dx
  if(x >= 0 && x < (int)IMG_WIDTH && y >= 0 && y < (int)IMG_HEIGHT)
  {
    if(x > 0)
    {
      xx = x - 1;
      yy = y;
      ix = uint_index(xx,yy);
      Y1 = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
    }
    else
    {
      xx = 0; yy = y;
      ix = uint_index(xx,yy);
      Y1 = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
    }

    if(x < (int)IMG_WIDTH - 1)
    {
      xx = x+1; yy = y;
      ix = uint_index(xx,yy);
      Y2 = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
    }
    else
    {
      xx = IMG_WIDTH - 1; yy = y;
      ix = uint_index(xx,yy);
      Y2 = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
    }

    (*dx) = ((int)Y2) - ((int)Y1);
    /*
		if((*dx) > 510 || (*dx) < -510)
		{
			printf("\n\r*** dx = %d, Y1 = %d, Y2 = %d ***\n\r", (*dx), Y1, Y2);
		}
     */

    if(y > 0)
    {
      xx = x; yy = y - 1;
      ix = uint_index(xx,yy);
      Y1 = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
    }
    else
    {
      xx = x; yy = 0;
      ix = uint_index(xx,yy);
      Y1 = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
    }

    if(y < (int)IMG_HEIGHT - 1)
    {
      xx = x; yy = y + 1;
      ix = uint_index(xx,yy);
      Y2 = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
    }
    else
    {
      xx = x; yy = IMG_HEIGHT-1;
      ix = uint_index(xx,yy);
      Y2 = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
    }

    (*dy) = ((int)Y2) - ((int)Y1);
    /*
		if((*dx) > 510 || (*dx) < -510)
		{
			printf("\n\r*** dy = %d, Y1 = %d, Y2 = %d ***\n\r", (*dy), Y1, Y2);
		}
     */
  }
}


void getSimpleGradient(unsigned char* frame_buf, int* DX, int* DY)
{
  unsigned int x,y,ix;
  int dx,dy;
  for(x = 0; x < IMG_WIDTH; x++)
  {
    for(y = 0; y < IMG_HEIGHT; y++)
    {
      // get dx, dy for the current position:
      getGradientPixelWH(frame_buf, x, y, &dx, &dy);
      // put it in the matrix:
      ix = int_index(x,y);
      DX[ix] = dx;
      DY[ix] = dy;

    }
  }
}

void excludeArea(unsigned int* Mask, int x, int y, int suppression_distance_squared)
{
  // suppression_distance_squared:
  // in a future implementation we can make a circular mask used to exclude pixels,
  // for now we just implement a square with sides equal to this value
  int xx, yy;
  unsigned int ix;
  //printf("suppression distance = %d\n\r", suppression_distance_squared);
  for(xx = x-suppression_distance_squared; xx<= x+suppression_distance_squared; xx++)
  {
    for(yy = y -suppression_distance_squared; yy < y+suppression_distance_squared; yy++)
    {
      if(yy >= 0 && yy < (int)IMG_HEIGHT && xx >= 0 && xx < (int)IMG_WIDTH)
      {
        ix = int_index(xx,yy);
        Mask[ix] = 1;
      }
    }
  }
}


int findLocalMaxima(int* Harris, int max_val, int MAX_POINTS, int* p_x, int* p_y, int suppression_distance_squared, int* n_found_points)
{
  //printf("W = %d, H = %d\n\r", IMG_WIDTH, IMG_HEIGHT);

  unsigned int* Mask; // Mask contains pixels that are excluded
  int x, y, local_max, xx, yy;
  unsigned int ix, iy;
  Mask = (unsigned int *) malloc(IMG_WIDTH * IMG_HEIGHT * sizeof(unsigned int));
  if(Mask == 0) return NO_MEMORY;
  (*n_found_points) = 0;
  // initialize with zeros (none excluded yet)
  for(x = 0; x < (int)IMG_WIDTH; x++)
  {
    for(y = 0; y < (int)IMG_HEIGHT; y++)
    {
      ix = int_index(x,y);
      Mask[ix] = 0;
    }
  }

  //printf("MAX_POINTS = %d\n\r", MAX_POINTS);

  // Find local maxima, forget about the border pixels:
  for(x = 1; x < (int)IMG_WIDTH-1; x++)
  {
    for(y = 1; y < (int)IMG_HEIGHT-1; y++)
    {
      // stop if the maximum number of points has been reached:
      if((*n_found_points) == MAX_POINTS) break;

      ix = int_index(x,y);
      // only evaluate a pixel if it is not too close to another corner:
      if(Mask[ix] == 0)
      {
        // is the pixel higher than its direct neighbors?
        local_max = 1;
        for(xx = x - 1; xx <= x + 1; xx++)
        {
          for(yy = y - 1; yy <= y + 1; yy++)
          {
            iy = int_index(xx,yy);
            if(iy != ix)
            {
              if(Harris[iy] >= Harris[ix])
              {
                local_max = 0;
                break;
              }
            }
          }
          if(!local_max) break;
        }

        if(local_max)
        {
          // printf("maximum at %d,%d\n\r", x, y);
          // store the point:
          p_x[(*n_found_points)] = x;
          p_y[(*n_found_points)] = y;
          (*n_found_points)++;

          // clear the area around the point:
          excludeArea(Mask, x, y, suppression_distance_squared);
          //mask_val = Mask[ix];
          //ix = int_index(x,y+3);
          //printf("Mask x,y = %d, x,y+3 = %d", mask_val, Mask[ix]);
        }
      }
    }
    // stop if the maximum number of points has been reached:
    if((*n_found_points) == MAX_POINTS) break;
  }
  // free Mask:
  free((char*)Mask);
  return OK;
}

// This function gives the maximum of a subsampled version of the image
int getMaximum(int * Im)
{
  int x, y, val, max_val;
  unsigned int ix;
  int step = 1;
  max_val = Im[0];
  for(x = 0; x < (int)IMG_WIDTH; x += step)
  {
    for(y = 0; y < (int)IMG_HEIGHT; y += step)
    {
      ix = int_index(x,y);
      val = Im[ix];
      if(val > max_val) max_val = val;
    }
  }
  return max_val;
}

// This function gives the maximum of a subsampled version of the image
int getMinimum(int * Im)
{
  int x, y, val, min_val;
  unsigned int ix;
  int step = 1;
  min_val = Im[0];
  for(x = 0; x < (int)IMG_WIDTH; x += step)
  {
    for(y = 0; y < (int)IMG_HEIGHT; y += step)
    {
      ix = int_index(x,y);
      val = Im[ix];
      if(val < min_val) min_val = val;
    }
  }
  return min_val;
}


void getHarris(int* DXX, int* DXY, int* DYY, int* Harris)
{
  // Harris = (dx2 * dy2 - dxy*dxy) - ((dx2 + dy2) * (dx2 + dy2)) / 25;
  int x, y, sumDXXDYY;
  unsigned int ix;
  int reciprocal_K = 25;
  //int printed = 0;
  //int PRECISION = 1;
  for(x = 0; x < (int)IMG_WIDTH; x++)
  {
    for(y = 0; y < (int)IMG_HEIGHT; y++)
    {
      ix = int_index(x,y);
      sumDXXDYY = DXX[ix] + DYY[ix];
      if(sumDXXDYY > 255) sumDXXDYY = 255;
      Harris[ix] = (DXX[ix] * DYY[ix] - DXY[ix] * DXY[ix]) - (sumDXXDYY * sumDXXDYY) / reciprocal_K;

      /*
			if(printed < 10 && (Harris[ix] > 65025 || Harris[ix] < -65025))
			{
				printf("dxxdyy = %d, dxydxy = %d, (dxx+dyy) = %d, Harris / k = %d, Harris = %d\n\r", DXX[ix] * DYY[ix], DXY[ix] * DXY[ix], sumDXXDYY, (DXX[ix] * DYY[ix] - DXY[ix] * DXY[ix]) - (sumDXXDYY * sumDXXDYY), (DXX[ix] * DYY[ix] - DXY[ix] * DXY[ix]) - (sumDXXDYY * sumDXXDYY) / reciprocal_K);
				printed++;
			}
       */

      //Harris[ix] = (DXX[ix] * DYY[ix] / PRECISION - DXY[ix] * DXY[ix] / PRECISION) - ((sumDXXDYY / PRECISION) * (sumDXXDYY / PRECISION)) / reciprocal_K;
    }
  }
}

void smoothGaussian(int* Src, int* Dst)
{
  int x, y, min_x, min_y, xx, yy, it;
  unsigned int ix, ixx;
  int smooth[9]; // = {1,2,1,2,4,2,1,2,1};
  int smooth_factor_1 = 14; // retain energy
  int smooth_factor_2 = 255; // for Harris to stay within
  //int printed = 0;

  // set the smoothing filter:
  smooth[0] = 1;
  smooth[1] = 2;
  smooth[2] = 1;
  smooth[3] = 2;
  smooth[4] = 4;
  smooth[5] = 2;
  smooth[6] = 1;
  smooth[7] = 2;
  smooth[8] = 1; // in MATLAB-language: [1,2,1;2,4,2;1,2,1]

  // is the following necessary, or is it already filled with zeros?
  // fill the borders with zeros:
  for(x = 0; x < (int)IMG_WIDTH; x += (int)IMG_WIDTH-1)
  {
    for(y = 0; y < (int)IMG_HEIGHT; y++)
    {
      ix = int_index(x,y);
      Dst[ix] = 0;
    }
  }
  for(y = 0; y < (int)IMG_HEIGHT; y += (int)IMG_HEIGHT-1)
  {
    for(x = 0; x < (int)IMG_WIDTH; x++)
    {
      ix = int_index(x,y);
      Dst[ix] = 0;
    }
  }

  for(x = 1; x < (int)IMG_WIDTH-1; x++)
  {
    for(y = 1; y < (int)IMG_HEIGHT-1; y++)
    {
      min_x = x - 1;
      min_y = y - 1;
      ix = int_index(x,y);
      Dst[ix] = 0;
      // use the patch to determine dx2, dxy, and dy2
      it = 0;
      for(yy = min_y; yy < min_y + 3; yy++)
      {
        for(xx = min_x; xx < min_x + 3; xx++)
        {
          ixx = int_index(xx,yy);

          Dst[ix] += smooth[it] * (Src[ixx] / smooth_factor_1);

          /*if(printed < 9 && x > 30 && y > 30)
					{
						printf("SG: it = %d, s = %d, src = %d, src / sf = %d, filtered = %d, Dst = %d\n\r", it, smooth[it], Src[ixx], Src[ixx] / smooth_factor_1, smooth[it] * (Src[ixx] / smooth_factor_1), Dst[ix]);
						printed += 1;
					}*/

          // update iterator of the smoothing filter:
          it++;
        }
      }

      Dst[ix] /= smooth_factor_2;
    }
  }
}

void thresholdImage(int* Harris, int max_val, int max_factor)
{
  // only retain values that are larger than max_val / max_factor:
  // are there images for which Harris' maximal value is negative?
  int x, y, threshold, n_remaining;
  unsigned int ix;
  threshold = max_val / max_factor;
  n_remaining = 0;
  for(x = 0; x < (int)IMG_WIDTH; x++)
  {
    for(y = 0; y < (int)IMG_HEIGHT; y++)
    {
      ix = int_index(x,y);
      if(Harris[ix] < threshold) Harris[ix] = 0;
      else n_remaining++;
    }
  }
  //printf("Remaining points = %d\n\r", n_remaining);
}

int findCorners(unsigned char *frame_buf, int MAX_POINTS, int *x, int *y, int suppression_distance_squared, int* n_found_points, int mark_points, int imW, int imH)
{
  // Algorithmic steps:
  // (1) get the dx, dy gradients in the image
  // (2) determine dxx, dxy, dyy
  // (3) smooth dxx, dxy, dyy
  // (4) determine the Harris values
  // (5) find maximal values (with suppression of points close by)
  // (6) mark the points green in the image

  int max_val, error, p, i, j; // min_val,
  unsigned int ix;
  unsigned int im_size;
  int* DX; int* DY; int* DXX; int* DXY; int* DYY;
  int* SDXX; int* SDXY; int* SDYY; int* Harris;
  // for debugging:
  //int n_rows, n_cols, x_center, y_center;

  IMG_WIDTH = imW;
  IMG_HEIGHT = imH;
  im_size = IMG_WIDTH * IMG_HEIGHT;
  // for debugging:
  //n_rows = 20; n_cols = 20;
  //x_center = IMG_WIDTH / 2;
  //y_center = IMG_HEIGHT / 2;

  // (1) get the dx, dy gradients in the image
  DX = (int *) malloc(im_size * sizeof(int));
  DY = (int *) malloc(im_size * sizeof(int));
  if(DX == 0 || DY == 0) return NO_MEMORY;
  getSimpleGradient(frame_buf, DX, DY);

  // DEBUGGING:
  //i_center = int_index(x_center, y_center);
  /*printf("DX = ");
	printIntMatrixPart(DX, IMG_WIDTH, IMG_HEIGHT, n_cols, n_rows, x_center, y_center);
	printf("DY = ");
	printIntMatrixPart(DY, IMG_WIDTH, IMG_HEIGHT, n_cols, n_rows, x_center, y_center);
   */
  //max_val = getMaximum(DX);
  //min_val = getMinimum(DX);
  //printf("DX[center] = %d, max = %d, min = %d\n\r", DX[i_center], max_val, min_val);

  // (2) determine dxx, dxy, dyy

  // form DXX and DXY
  DXX = (int *) malloc(im_size * sizeof(int));
  DXY = (int *) malloc(im_size * sizeof(int));
  if(DXX == 0 || DXY == 0) return NO_MEMORY;
  multiplyImages(DX, DX, DXX, IMG_WIDTH, IMG_HEIGHT);
  multiplyImages(DX, DY, DXY, IMG_WIDTH, IMG_HEIGHT);
  // free DX
  free((char*) DX);

  // DEBUGGING:
  /*	printf("DXY = ");
	printIntMatrixPart(DXY, IMG_WIDTH, IMG_HEIGHT, n_cols, n_rows, x_center, y_center);
	printf("DXX = ");
	printIntMatrixPart(DXX, IMG_WIDTH, IMG_HEIGHT, n_cols, n_rows, x_center, y_center);
   */
  //max_val = getMaximum(DXX);
  //min_val = getMinimum(DXX);
  //printf("DXX[center] = %d, max = %d, min = %d\n\r", DXX[i_center], max_val, min_val);

  // form DYY
  DYY = (int *) malloc(im_size * sizeof(int));
  if(DYY == 0) return NO_MEMORY;
  multiplyImages(DY, DY, DYY, IMG_WIDTH, IMG_HEIGHT);
  // free DY
  free((char*) DY);

  // (3) smooth dxx, dxy, dyy
  // unfortunately, this smoothing is quite necessary:

  SDXX = (int *) malloc(im_size * sizeof(int));
  if(SDXX == 0) return NO_MEMORY;
  smoothGaussian(DXX, SDXX);

  // free DXX
  free((char*) DXX);

  // DEBUGGING:
  /*	printf("DYY = ");
	printIntMatrixPart(DYY, IMG_WIDTH, IMG_HEIGHT, n_cols, n_rows, x_center, y_center);
	printf("SDXX = ");
	printIntMatrixPart(SDXX, IMG_WIDTH, IMG_HEIGHT, n_cols, n_rows, x_center, y_center);
   */
  //max_val = getMaximum(SDXX);
  //min_val = getMinimum(SDXX);
  //printf("SDXX[center] = %d, max = %d, min = %d\n\r", SDXX[i_center], max_val, min_val);

  SDXY = (int *) malloc(im_size * sizeof(int));
  if(SDXY == 0) return NO_MEMORY;
  smoothGaussian(DXY, SDXY);

  // free DXY
  free((char*) DXY);

  SDYY = (int *) malloc(im_size * sizeof(int));
  smoothGaussian(DYY, SDYY);
  /*
	printf("SDYY = ");
	printIntMatrixPart(SDYY, IMG_WIDTH, IMG_HEIGHT, n_cols, n_rows, x_center, y_center);
	printf("SDXY = ");
	printIntMatrixPart(SDXY, IMG_WIDTH, IMG_HEIGHT, n_cols, n_rows, x_center, y_center);
   */
  if(SDXX == 0 || SDXY == 0 || SDYY == 0) return NO_MEMORY;
  // free DYY
  free((char*) DYY);

  // (4) determine the Harris values
  Harris = (int *) malloc(im_size * sizeof(int));
  if(Harris == 0) return NO_MEMORY;
  getHarris(SDXX, SDXY, SDYY, Harris);
  //printf("Harris1 = ");
  //printIntMatrixPart(Harris, IMG_WIDTH, IMG_HEIGHT, n_cols, n_rows, x_center, y_center);

  // free SDXX, SDYY, SDXY:
  free((char*)SDXX); free((char*)SDYY); free((char*)SDXY);

  // DEBUGGING:
  //max_val = getMaximum(Harris);
  //min_val = getMinimum(Harris);
  //printf("Harris[center] = %d, max = %d, min = %d\n\r", Harris[i_center], max_val, min_val);

  // (5) find maximal values (with suppression of points close by)
  // (a) find the maximum
  max_val = getMaximum(Harris);
  // (b) threshold the image on the basis of the found (approximative) maximum:
  thresholdImage(Harris, max_val, 5);
  //printf("Harris2 = ");
  //printIntMatrixPart(Harris, IMG_WIDTH, IMG_HEIGHT, n_cols, n_rows, x_center, y_center);
  // (c) find local maxima
  error = findLocalMaxima(Harris, max_val, MAX_POINTS, x, y, suppression_distance_squared, n_found_points);
  if(error == NO_MEMORY) return NO_MEMORY;
  // free Harris:
  free((char*) Harris);

  // DEBUGGING:
  // printf("Points found: %d\n\r", (*n_found_points));

  // (6) mark the points green in the image
  if(mark_points > 0)
  {
    // printf("IMG_WIDTH = %d, IMG_HEIGHT = %d\n\r", IMG_WIDTH, IMG_HEIGHT);

    for(p = 0; p < (*n_found_points); p++)
    {
      if(x[p] >= 1 && y[p] >= 1 && x[p] < (int)IMG_WIDTH - 1 && y[p] < (int)IMG_HEIGHT - 1)
      {
        // printf("(x,y) = (%d,%d)\n\r", x[p],y[p]);
        for(i = -1; i <= 1; i++)
        {
          for(j = -1; j <= 1; j++)
          {
            // printf("(x+i,y+j) = (%d,%d)\n\r", x[p]+i,y[p]+j);
            ix = uint_index((unsigned int) (x[p]+i), (unsigned int) (y[p]+j));
            // printf("ix = %d, ixx = %d\n\r", ix, (((y[p]+j) * IMG_WIDTH + (x[p]+i)) * 2) & 0xFFFFFFFC);
            redPixel(frame_buf, ix);
          }
        }
      }
    }
  }
  // routine successful:
  return OK;
}

int findActiveCorners(unsigned char *frame_buf, unsigned int GRID_ROWS, int ONLY_STOPPED, int *x, int *y, int* active, int* n_found_points, int mark_points, int imW, int imH)
{
  // Algorithmic steps:
  // 1) initialize the agents' positions
  // 2) let the agents search in the image
  // 3) select the points to be returned

  unsigned int grid_step_x, grid_step_y, border, n_agents, gr, gc, a;
  unsigned int t, n_time_steps, n_active, finished, active_agents;
  int visual_inputs[N_VISUAL_INPUTS];
  int actions[N_ACTIONS];
  int dx, dy, MAX_JUMP;
  unsigned int RESOLUTION, half_patch;
  int p, i, j;
  unsigned int ix;

  // copy image parameters:
  IMG_WIDTH = imW;
  IMG_HEIGHT = imH;

  // The resolution will allow subpixel positions:
  RESOLUTION = 100;

  // ***********************************
  // 1) initialize the agents' positions
  // ***********************************

  n_agents = GRID_ROWS * GRID_ROWS;
  border = 10;
  grid_step_x = (imW - 2 * border) / (GRID_ROWS-1);
  grid_step_y = (imH - 2 * border) / (GRID_ROWS-1);

  a = 0;
  for(gr = 0; gr < GRID_ROWS; gr++)
  {
    for(gc = 0; gc < GRID_ROWS; gc++)
    {
      x[a] = (border + gr * grid_step_x) * RESOLUTION;
      y[a] = (border + gc * grid_step_y) * RESOLUTION;
      active[a] = 1;
      a++;
      if(a == n_agents) break;
    }

    if(a == n_agents) break;
  }

  // *************************************
  // 2) let the agents search in the image
  // *************************************

  MAX_JUMP = 10;
  half_patch = 2;
  n_time_steps = 20;
  t = 0;
  finished = 0;
  while(finished == 0)
  {
    // number of agents still active:
    n_active = 0;

    // loop over the agents:
    for(a = 0; a < n_agents; a++)
    {
      // only extract inputs and move if still active:
      if(active[a] == 1)
      {
        // printf("Agent %d at (%d,%d)\n\r", a, x[a], y[a]);
        n_active++;

        // sensing, inputs in [0,255]:
        getVisualInputs(frame_buf, x[a] / RESOLUTION, y[a] / RESOLUTION, visual_inputs, half_patch);

        // determining actions, in [-RESOLUTION, RESOLUTION]:
        applyNeuralNetwork(visual_inputs, actions, RESOLUTION);

        // printf("Actions = (%d, %d, %d)\n\r", actions[0], actions[1], actions[2]);

        // possibly stopping:
        if(actions[2] < 0)
        {
          // printf("Agent %d stopped.\n\r", a);
          active[a] = 0;
        }

        // if moving:
        if(active[a] == 1)
        {
          dx = (actions[0] * MAX_JUMP);
          dy = (actions[1] * MAX_JUMP);

          x[a] += dx;
          y[a] += dy;

          // checking the limits, making the agents go round the image when they leave the image:
          if(x[a] / RESOLUTION < half_patch + 1) // the + 1 is necessary for the convolution with image filters
          {
            x[a] = (IMG_WIDTH - half_patch - 2) * RESOLUTION;
          }
          else if(x[a] / RESOLUTION > IMG_WIDTH - half_patch - 2)
          {
            x[a] = (half_patch + 1) * RESOLUTION;
          }
          if(y[a] / RESOLUTION < half_patch + 1) // the + 1 is necessary for the convolution with image filters
          {
            y[a] = (IMG_HEIGHT - half_patch - 2) * RESOLUTION;
          }
          else if(y[a] / RESOLUTION > IMG_HEIGHT - half_patch - 2)
          {
            y[a] = (half_patch + 1) * RESOLUTION;
          }

        }
      }
    }

    // update time and check end conditions:
    // printf("Time = %d / %d\n\r", t, n_time_steps);
    t++;
    if(t == n_time_steps || n_active == 0)
    {
      finished = 1;
    }
  }

  // ***********************************
  // 3) select the points to be returned
  // ***********************************

  if(!ONLY_STOPPED)
  {
    for(a = 0; a < n_agents; a++)
    {
      x[a] = x[a] / RESOLUTION;
      y[a] = y[a] / RESOLUTION;
    }
    (*n_found_points) = n_agents;
  }
  else
  {

    active_agents = 0;

    for(a = 0; a < n_agents; a++)
    {
      if(active[a] == 0)
      {
        x[active_agents] = x[a] / RESOLUTION;
        y[active_agents] = y[a] / RESOLUTION;
        active_agents++;
      }
    }

    (*n_found_points) = active_agents;
  }

  // ********************************
  // (4) mark the points in the image
  // ********************************

  if(mark_points > 0)
  {
    //printf("Mark points\n\r");
    // printf("IMG_WIDTH = %d, IMG_HEIGHT = %d\n\r", IMG_WIDTH, IMG_HEIGHT);

    for(p = 0; p < (*n_found_points); p++)
    {
      if(x[p] >= 1 && y[p] >= 1 && x[p] < (int)IMG_WIDTH - 1 && y[p] < (int)IMG_HEIGHT - 1)
      {
        //printf("(x,y) = (%d,%d)\n\r", x[p],y[p]);
        for(i = -1; i <= 1; i++)
        {
          for(j = -1; j <= 1; j++)
          {
            // printf("(x+i,y+j) = (%d,%d)\n\r", x[p]+i,y[p]+j);
            ix = uint_index((unsigned int) (x[p]+i), (unsigned int) (y[p]+j));
            // printf("ix = %d, ixx = %d\n\r", ix, (((y[p]+j) * IMG_WIDTH + (x[p]+i)) * 2) & 0xFFFFFFFC);
            redPixel(frame_buf, ix);
          }
        }
      }
    }
  }


  return 0;
}

void getVisualInputs(unsigned char *frame_buf, int x, int y, int* visual_inputs, unsigned int half_patch)
{
  int i, dx, dy, half_inputs, xx, yy;
  // will round off correctly (bias is counted as visual input):
  half_inputs = N_VISUAL_INPUTS / 2;
  i = 0;
  for(xx = x - (int)half_patch; xx <= x + (int)half_patch; xx++)
  {
    for(yy = y - (int)half_patch; yy <= y + (int)half_patch; yy++)
    {
      //printf("gp(%d, %d), ", xx, yy);
      getGradientPixelWH(frame_buf, xx, yy, &dx, &dy);
      visual_inputs[i] = dx;
      visual_inputs[half_inputs+i] = dy;
      i++;
    }
  }
  // bias, scaled to the input range:
  visual_inputs[N_VISUAL_INPUTS-1] = 255;

  /*printf("Visual inputs:\n");
	for(i = 0; i < N_VISUAL_INPUTS; i++)
	{
		printf("%d, ", visual_inputs[i]);
	}
	printf("\n\r");*/

  return;
}

void applyNeuralNetwork(int* visual_inputs, int* actions, int RESOLUTION)
{

  int a, i, w, factor;

  //printf("NN\n\r");

  // inputs in [0,255] weights in [-100,100]
  // 51 weights per neuron
  // 25500 * 51 = 1,300,500 (max)
  // float point:
  // inputs in [-1, 1] weights in [-1, 1]
  // 51 weights, 51 (max) (* RESOLUTION = 5100)
  // but the max are very unlikely, so:
  factor = 25500 / RESOLUTION;
  w = 0;
  // Perceptron neural network:
  for(a = 0; a < N_ACTIONS; a++)
  {
    actions[a] = 0;

    for(i = 0; i < N_VISUAL_INPUTS; i++)
    {
      actions[a] += weights[w] * visual_inputs[i];
      w++;
    }

    //printf("actions[%d] after summing: %d\n\r", a, actions[a]);

    // account for changes in value ranges (25500 vs. 1 -> 1*RESOLUTION)
    actions[a] /= factor;

    //printf("actions[%d] after factor: %d\n\r", a, actions[a]);

    // activation function:
    actions[a] /= 2;

    //printf("actions[%d] after /2: %d\n\r", a, actions[a]);

    // saturate at RESOLUTION / -RESOLUTION:
    actions[a] = (actions[a] > RESOLUTION) ? RESOLUTION : actions[a];
    actions[a] = (actions[a] < -RESOLUTION) ? -RESOLUTION : actions[a];

    //printf("actions[%d] after saturation: %d\n\r", a, actions[a]);

  }

  return;
}



void getSubPixel(int* Patch, unsigned char* frame_buf, int center_x, int center_y, int half_window_size, int subpixel_factor)
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
        ix2 = uint_index(x_0_or, y_0_or);
        Y = ((unsigned int)frame_buf[ix2+1] + (unsigned int)frame_buf[ix2+3]) >> 1;
        Patch[ix1] = (int) Y;
      }
      else
      {
        // blending according to how far the subpixel coordinates are from the pixel coordinates
        alpha_x = (x - x_0);
        alpha_y = (y - y_0);

        // the patch pixel is a blend from the four surrounding pixels:
        ix2 = uint_index(x_0_or, y_0_or);
        Y = ((unsigned int)frame_buf[ix2+1] + (unsigned int)frame_buf[ix2+3]) >> 1;
        Patch[ix1] = (subpixel_factor - alpha_x) * (subpixel_factor - alpha_y) * ((int) Y);

        ix2 = uint_index((x_0_or+1), y_0_or);
        Y = ((unsigned int)frame_buf[ix2+1] + (unsigned int)frame_buf[ix2+3]) >> 1;
        //if(printed < limit) printf("subpixel: TR = %d\n\r", Y);
        Patch[ix1] += alpha_x * (subpixel_factor - alpha_y) * ((int) Y);

        ix2 = uint_index(x_0_or, (y_0_or+1));
        Y = ((unsigned int)frame_buf[ix2+1] + (unsigned int)frame_buf[ix2+3]) >> 1;
        //if(printed < limit) printf("subpixel: BL = %d\n\r", Y);
        Patch[ix1] += (subpixel_factor - alpha_x) * alpha_y * ((int) Y);

        ix2 = uint_index((x_0_or+1), (y_0_or+1));
        Y = ((unsigned int)frame_buf[ix2+1] + (unsigned int)frame_buf[ix2+3]) >> 1;
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

// only for UYVY image, not grayscale
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
	int b_x, b_y, patch_size, padded_patch_size, error, step_size;
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

/*********************************************************************
 * _convolveImageHor
 */

void convolveImg_H(float *img, int ncols, int nrows, float *kernel, int kernel_width, float*conv_img)
{
  float *ptrrow = img;           /* Points to row's first pixel */
  float *ptrout = conv_img, /* Points to next output pixel */
    *ppp;
  float sum;
  int radius = kernel_width / 2;
  int i, j, k;
  /* Kernel width must be odd */
  /* Must read from and write to different images */
  /* Output image must be large enough to hold result */

  /* For each row, do ... */
  for (j = 0 ; j < nrows ; j++)  {

    /* Zero leftmost columns */
    for (i = 0 ; i < radius ; i++)
      *ptrout++ = 0;

    /* Convolve middle columns with kernel */
    for ( ; i < ncols - radius ; i++)  {
      ppp = ptrrow + i - radius;
      sum = 0;
      for (k = kernel_width-1 ; k >= 0 ; k--)
        sum += *ppp++ * kernel[k];
      *ptrout++ = sum;
    }

    /* Zero rightmost columns */
    for ( ; i < ncols ; i++)
      *ptrout++ = 0;

    ptrrow += ncols;
  }
}

void convolveImg_H_int(unsigned char *img, int ncols, int nrows, int *kernel, int kernel_width, unsigned char *conv_img)
{
  unsigned char *ptrrow = img;           /* Points to row's first pixel */
  unsigned char *ptrout = conv_img, /* Points to next output pixel */
    *ppp;
  unsigned char sum;
  int radius = kernel_width / 2;
  int i, j, k;
  /* Kernel width must be odd */
  /* Must read from and write to different images */
  /* Output image must be large enough to hold result */

  /* For each row, do ... */
  for (j = 0 ; j < nrows ; j++)  {

    /* Zero leftmost columns */
    for (i = 0 ; i < radius ; i++)
      *ptrout++ = 0;

    /* Convolve middle columns with kernel */
    for ( ; i < ncols - radius ; i++)  {
      ppp = ptrrow + i - radius;
      sum = 0;
      for (k = kernel_width-1 ; k >= 0 ; k--)
        sum += *ppp++ >> kernel[k];
      *ptrout++ = sum;
    }

    /* Zero rightmost columns */
    for ( ; i < ncols ; i++)
      *ptrout++ = 0;

    ptrrow += ncols;
  }
}

/*********************************************************************
 * _convolveImageVert
 */

void convolveImg_V(float *img, int ncols, int nrows, float *kernel, int kernel_width, float *conv_img)
{
  float *ptrcol = img;            /* Points to row's first pixel */
  float *ptrout = conv_img,  /* Points to next output pixel */
    *ppp;
  float sum;
  int radius = kernel_width / 2;
  int i, j, k;

  /* Kernel width must be odd */
  /* Must read from and write to different images */
  /* Output image must be large enough to hold result */

  /* For each column, do ... */
  for (i = 0 ; i < ncols ; i++)  {

    /* Zero topmost rows */
    for (j = 0 ; j < radius ; j++)  {
      *ptrout = 0;
      ptrout += ncols;
    }

    /* Convolve middle rows with kernel */
    for ( ; j < nrows - radius ; j++)  {
      ppp = ptrcol + ncols * (j - radius);
      sum = 0;
      for (k = kernel_width-1 ; k >= 0 ; k--)  {
        sum += *ppp * kernel[k];
        ppp += ncols;
      }
      *ptrout = sum;
      ptrout += ncols;
    }

    /* Zero bottommost rows */
    for ( ; j < nrows ; j++)  {
      *ptrout = 0;
      ptrout += ncols;
    }

    ptrcol++;
    ptrout -= nrows * ncols - 1;
  }
}

void convolveImg_V_int(unsigned char *img, int ncols, int nrows, int *kernel, int kernel_width, unsigned char *conv_img)
{
  unsigned char *ptrcol = img;            /* Points to row's first pixel */
  unsigned char *ptrout = conv_img,  /* Points to next output pixel */
    *ppp;
  unsigned char sum;
  int radius = kernel_width / 2;
  int i, j, k;

  /* Kernel width must be odd */
  /* Must read from and write to different images */
  /* Output image must be large enough to hold result */

  /* For each column, do ... */
  for (i = 0 ; i < ncols ; i++)  {

    /* Zero topmost rows */
    for (j = 0 ; j < radius ; j++)  {
      *ptrout = 0;
      ptrout += ncols;
    }

    /* Convolve middle rows with kernel */
    for ( ; j < nrows - radius ; j++)  {
      ppp = ptrcol + ncols * (j - radius);
      sum = 0;
      for (k = kernel_width-1 ; k >= 0 ; k--)  {
        sum += *ppp >> kernel[k];
        ppp += ncols;
      }
      *ptrout = sum;
      ptrout += ncols;
    }

    /* Zero bottommost rows */
    for ( ; j < nrows ; j++)  {
      *ptrout = 0;
      ptrout += ncols;
    }

    ptrcol++;
    ptrout -= nrows * ncols - 1;
  }
}

/*********************************************************************
 * _KLTComputeSmoothedImage
 */

void SmoothPyramid(float *img, int ncols, int nrows, float *gauss_kernel, int kernel_width, float *smooth_img)
{
  /* Output image must be large enough to hold result */

  /* Create temporary image */
	float *tmpimg;
  tmpimg = (float *)malloc(ncols*nrows*sizeof(float));

  /* Do convolution */
  convolveImg_H(img, ncols, nrows, gauss_kernel, kernel_width, tmpimg);

  convolveImg_V(tmpimg, ncols, nrows, gauss_kernel, kernel_width, smooth_img);
  /* Free memory */
  free(tmpimg);
}

void SmoothPyramid_int(unsigned char *img, int ncols, int nrows, int *gauss_kernel, int kernel_width, unsigned char *smooth_img)
{
  /* Output image must be large enough to hold result */

  /* Create temporary image */
  unsigned char *tmpimg;
  tmpimg = (unsigned char *)malloc(ncols*nrows*sizeof(unsigned char));

  /* Do convolution */
  convolveImg_H_int(img, ncols, nrows, gauss_kernel, kernel_width, tmpimg);

  convolveImg_V_int(tmpimg, ncols, nrows, gauss_kernel, kernel_width, smooth_img);
  /* Free memory */
  free(tmpimg);
}

void ComputePyramid( float* img, float** pyramid, int nLevels, int imgW, int imgH, int subsampling, float *gauss_kernel, int kernel_width)
{
  float *tmpimg, *currimg;
  int ncols = imgW, nrows = imgH;
  int subhalf = subsampling / 2;
//  float sigma = subsampling * sigma_fact;  /* empirically determined */
  int oldncols;
  int i, x, y;

//  if (subsampling != 2 && subsampling != 4 &&
//      subsampling != 8 && subsampling != 16 && subsampling != 32)
//    printf("(_KLTComputePyramid)  Pyramid's subsampling must "
//             "be either 2, 4, 8, 16, or 32");

  /* Copy original image to level 0 of pyramid */
  memcpy(pyramid[0], img, ncols*nrows*sizeof(float));

//  printf("pyramid 0 = \n");
//  for(i=0;i<nrows;i++)
//  {
//	  for(j=0;j<ncols;j++)
//	  {
//		  printf("%f ",pyramid[0][i*ncols+j]);
//	  }
//	  printf("\n");
//  }
//  printf("\n");
//  memcpy(currimg, img, ncols*nrows*sizeof(float));

  currimg = img;
  for (i = 1 ; i < nLevels ; i++)
  {
	  tmpimg = (float *) malloc (ncols*nrows*sizeof(float));
//	  memcpy(tmpimg, pyramid[i-1], ncols*nrows*sizeof(float));
	  SmoothPyramid(currimg, ncols, nrows, gauss_kernel, kernel_width, tmpimg);
//    _KLTComputeSmoothedImage(currimg, sigma, tmpimg);


    /* Subsample */
    oldncols = ncols;
    ncols /= subsampling;  nrows /= subsampling;
    for (y = 0 ; y < nrows ; y++)
    {
    	for (x = 0 ; x < ncols ; x++)
    	{
            pyramid[i][y*ncols+x] =
              tmpimg[(subsampling*y+subhalf)*oldncols +
                          (subsampling*x+subhalf)];
    	}
    }

    /* Reassign current image */

    currimg = pyramid[i];

    free(tmpimg);
  }
}

void ComputePyramid_int( unsigned char *img, unsigned char **pyramid, int nLevels, int imgW, int imgH, int subsampling, float *gauss_kernel, int kernel_width)
{
  unsigned char *tmpimg, *currimg;
  int ncols = imgW, nrows = imgH;
  int subhalf = subsampling / 2;
  int oldncols;
  int i, x, y;

  /* Copy original image to level 0 of pyramid */
  memcpy(pyramid[0], img, ncols*nrows*sizeof(unsigned char));


  currimg = img;
  for (i = 1 ; i < nLevels ; i++)
  {
	  tmpimg = (unsigned char *) malloc (ncols*nrows*sizeof(unsigned char));
	  SmoothPyramid_int(currimg, ncols, nrows, gauss_kernel, kernel_width, tmpimg);

    /* Subsample */
    oldncols = ncols;
    ncols /= subsampling;  nrows /= subsampling;
    for (y = 0 ; y < nrows ; y++)
    {
    	for (x = 0 ; x < ncols ; x++)
    	{
            pyramid[i][y*ncols+x] =
              tmpimg[(subsampling*y+subhalf)*oldncols +
                          (subsampling*x+subhalf)];
    	}
    }

    /* Reassign current image */

    currimg = pyramid[i];

    free(tmpimg);
  }
}

/*********************************************************************
 * Compute Gradient
 */

void convolveSeparate(float *img, int ncols, int nrows, float *horiz_kernel, int horiz_kernel_width, float *vert_kernel, int vert_kernel_width, float *smooth_img)
{
  /* Output image must be large enough to hold result */

  /* Create temporary image */
	float *tmpimg;
  tmpimg = (float *)malloc(ncols*nrows*sizeof(float));

  /* Do convolution */
  convolveImg_H(img, ncols, nrows, horiz_kernel, horiz_kernel_width, tmpimg);

  convolveImg_V(tmpimg, ncols, nrows, vert_kernel, vert_kernel_width, smooth_img);
  /* Free memory */
  free(tmpimg);
}

void convolveSeparate_int(unsigned char *img, int ncols, int nrows, float *horiz_kernel, int horiz_kernel_width, float *vert_kernel, int vert_kernel_width, unsigned char *smooth_img)
{
  /* Output image must be large enough to hold result */

  /* Create temporary image */
  unsigned char *tmpimg;
  tmpimg = (unsigned char *)malloc(ncols*nrows*sizeof(unsigned char));

  /* Do convolution */
  convolveImg_H_int(img, ncols, nrows, horiz_kernel, horiz_kernel_width, tmpimg);

  convolveImg_V_int(tmpimg, ncols, nrows, vert_kernel, vert_kernel_width, smooth_img);
  /* Free memory */
  free(tmpimg);
}

void ComputeGradients(float *img, int ncols, int nrows, float *gauss_kernel, int kernel_width, float *gaussderiv_kernel, int kernel_deriv_width, float *gradx, float *grady)
{

  /* Output images must be large enough to hold result */

  convolveSeparate(img, ncols, nrows, gaussderiv_kernel, kernel_deriv_width, gauss_kernel, kernel_width, gradx);
  convolveSeparate(img, ncols, nrows, gauss_kernel, kernel_width, gaussderiv_kernel, kernel_deriv_width, grady);

}

void ComputeGradients_int(unsigned char *img, int ncols, int nrows, float *gauss_kernel, int kernel_width, float *gaussderiv_kernel, int kernel_deriv_width, unsigned char *gradx, unsigned char *grady)
{

  /* Output images must be large enough to hold result */

  convolveSeparate_int(img, ncols, nrows, gaussderiv_kernel, kernel_deriv_width, gauss_kernel, kernel_width, gradx);
  convolveSeparate_int(img, ncols, nrows, gauss_kernel, kernel_width, gaussderiv_kernel, kernel_deriv_width, grady);

}

/*********************************************************************
 * Track Corners
 */

float interpolate(float x, float y, float *img, int ncols, int nrows)
{
  int xt = (int) x;  /* coordinates of top-left corner */
  int yt = (int) y;
  float ax = x - xt;
  float ay = y - yt;
  float *ptr = img + (ncols*yt) + xt;

//  printf("xt = %d, yt = %d, ncols-2 = %d, nrows-2 = %d\n",xt,yt,ncols-2,nrows-2);
//  assert (xt >= 0 && yt >= 0 && xt <= ncols - 2 && yt <= nrows - 2);
//printf("ax = %f, ay = %f, p1 = %f, p2 = %f, p3 = %f, p4 = %f\n",ax, ay, *ptr, *(ptr+1),*(ptr+ncols),*(ptr+ncols+1));

  return ( (1-ax) * (1-ay) * *ptr +
           ax   * (1-ay) * *(ptr+1) +
           (1-ax) *   ay   * *(ptr+(ncols)) +
           ax   *   ay   * *(ptr+(ncols)+1) );
}

void computeIntensityDifference(float *img1, float *img2, int ncols, int nrows, float x11, float y11, float x22, float y22, int width, int height, float *imgdiff)
{
  int hw = width/2, hh = height/2;
  float g1, g2;
  int i, j;

  /* Compute values */
  for (j = -hh ; j <= hh ; j++)
    for (i = -hw ; i <= hw ; i++)  {
//      printf("x11+i = %f, y11+j = %f, x22+i = %f, y22+j = %f\n", x11+i, y11+j, x22+i, y22+j);
      g1 = interpolate(x11+i, y11+j, img1, ncols, nrows);
      g2 = interpolate(x22+i, y22+j, img2, ncols, nrows);
//      printf("g1 = %f, g2 = %f\n", g1, g2);
      *imgdiff++ = g1 - g2;
    }
}

void computeGradientSum(float *gradx1, float *grady1,float *gradx2, float *grady2, int ncols, int nrows, float x11, float y11, float x22, float y22, int width, int height, float *gradx, float *grady)
{
  int hw = width/2, hh = height/2;
  float g1, g2;
  int i, j;

  /* Compute values */
  for (j = -hh ; j <= hh ; j++)
    for (i = -hw ; i <= hw ; i++)  {
      g1 = interpolate(x11+i, y11+j, gradx1, ncols, nrows);
      g2 = interpolate(x22+i, y22+j, gradx2, ncols, nrows);
      *gradx++ = g1 + g2;
      g1 = interpolate(x11+i, y11+j, grady1, ncols, nrows);
      g2 = interpolate(x22+i, y22+j, grady2, ncols, nrows);
      *grady++ = g1 + g2;
    }
}

void compute2by2GradientMatrix(float *gradx, float *grady, int width, int height, float *gxx, float *gxy, float *gyy)
{
  float gx, gy;
  int i;

  /* Compute values */
  *gxx = 0.0;  *gxy = 0.0;  *gyy = 0.0;
  for (i = 0 ; i < width * height ; i++)  {
    gx = *gradx++;
    gy = *grady++;
    *gxx += gx*gx;
    *gxy += gx*gy;
    *gyy += gy*gy;
  }
}

void compute2by1ErrorVector(float *imgdiff, float *gradx,float *grady, int width, int height, float step_factor, float *ex, float *ey)
{
  float diff;
  int i;

  /* Compute values */
  *ex = 0;  *ey = 0;
  for (i = 0 ; i < width * height ; i++)  {
    diff = *imgdiff++;
    *ex += diff * (*gradx++);
    *ey += diff * (*grady++);
  }
  *ex *= step_factor;
  *ey *= step_factor;
}

int solveEquation(float gxx, float gxy, float gyy, float ex, float ey, float small, float *dx, float *dy)
{
  float det = gxx*gyy - gxy*gxy;

  if (det < small)  return -2;

  *dx = (gyy*ex - gxy*ey)/det;
  *dy = (gxx*ey - gxy*ex)/det;
  return 0;
}

float sumAbsFloatWindow(float *fw, int width, int height)
{
  float sum = 0.0;
  int w;

  for ( ; height > 0 ; height--)
    for (w=0 ; w < width ; w++)
      sum += (float) fabs(*fw++);

  return sum;
}

int trackFeature(float x11, float y11, float *x22, float *y22, float *img1, float *gradx1, float *grady1, float *img2, float *gradx2, float *grady2, int ncols, int nrows, int width, int height, float step_factor, int max_iterations, float small, float th, float max_residue)
{
  float *imgdiff, *gradx, *grady;
  float gxx, gxy, gyy, ex, ey, dx, dy;
  int iteration = 0;
  int status;
  int hw = width/2;
  int hh = height/2;
  int nc = ncols;
  int nr = nrows;
  float one_plus_eps = 1.001f;   /* To prevent rounding errors */


  /* Allocate memory for windows */
  imgdiff = (float *)malloc(width*height*sizeof(float));
  gradx   = (float *)malloc(width*height*sizeof(float));
  grady   = (float *)malloc(width*height*sizeof(float));

  /* Iteratively update the window position */
  do  {

    /* If out of bounds, exit loop */
    if (  x11-hw < 0.0f || nc-( x11+hw) < one_plus_eps ||
         *x22-hw < 0.0f || nc-(*x22+hw) < one_plus_eps ||
          y11-hh < 0.0f || nr-( y11+hh) < one_plus_eps ||
         *y22-hh < 0.0f || nr-(*y22+hh) < one_plus_eps) {
      status = -4;
//      printf("out of bound\n");
      break;
    }

    /* Compute gradient and difference windows */
//printf("track_1\n");
    computeIntensityDifference(img1, img2, ncols, nrows, x11, y11, *x22, *y22, width, height, imgdiff);

//    printf("imgdiff = \n");
//    for(i=0;i<height;i++)
//    {
//    	for(j=0; j<width; j++)
//    	{
//    		printf("%f ", imgdiff[i*width+j]);
//    	}
//    	printf("\n");
//    }


//printf("track_2\n");
    computeGradientSum(gradx1, grady1, gradx2, grady2, ncols, nrows, x11, y11, *x22, *y22, width, height, gradx, grady);
//printf("track_3\n");
    /* Use these windows to construct matrices */
    compute2by2GradientMatrix(gradx, grady, width, height, &gxx, &gxy, &gyy);
//printf("track_4\n");
    compute2by1ErrorVector(imgdiff, gradx, grady, width, height, step_factor, &ex, &ey);
//printf("track_5\n");
    /* Using matrices, solve equation for new displacement */
    status = solveEquation(gxx, gxy, gyy, ex, ey, small, &dx, &dy);
//printf("track_6\n");
    if (status == -2)  break;

    *x22 += dx;
    *y22 += dy;
//    printf("status = %d, gxx = %f, gxy = %f, gyy = %f, ex = %f, ey = %f, dx = %f, dy = %f, x22 = %f, y22 = %f\n",status,gxx,gxy,gyy,ex,ey,dx,dy,*x22,*y22);
    iteration++;

  }  while ((fabs(dx)>=th || fabs(dy)>=th) && iteration < max_iterations);
//printf("track_7\n");
  /* Check whether window is out of bounds */
  if (*x22-hw < 0.0f || nc-(*x22+hw) < one_plus_eps ||
      *y22-hh < 0.0f || nr-(*y22+hh) < one_plus_eps)
    status = -4;
//printf("track_8\n");
  /* Check whether residue is too large */
  if (status == 0)  {
      computeIntensityDifference(img1, img2, ncols, nrows, x11, y11, *x22, *y22,
                                  width, height, imgdiff);
    if (sumAbsFloatWindow(imgdiff, width, height)/(width*height) > max_residue)
      status = -5;
  }
//printf("track_9\n");
  /* Free memory */
  free(imgdiff);  free(gradx);  free(grady);

  /* Return appropriate value */
  if (status == -2)  return -2;
  else if (status == -4)  return -4;
  else if (status == -5)  return -5;
  else if (iteration >= max_iterations)  return -3;
  else  return 0;

}

// uses gayscale images
int opticFlowLKPyramidal(unsigned char **new_gray_buf, unsigned char **old_gray_buf, int nLevels, int subsampling, int* p_x, int* p_y, int n_found_points, int imW, int imH, int* new_x, int* new_y, int *flow_point, int* status, int half_window_size, int max_iterations)
{
	// A Pyramidal implementation of Lucas-Kanade.
	// For all points:
	// (1) determine the subpixel neighborhood in the old image
	// (2) get the x- and y- gradients
	// (3) determine the 'G'-matrix [sum(Axx) sum(Axy); sum(Axy) sum(Ayy)], where sum is over the window
	// (4) iterate over taking steps in the image to minimize the error:
	//     [a] get the subpixel neighborhood in the new image
	//     [b] determine the image difference between the two neighborhoods
	//     [c] calculate the 'b'-vector
	//     [d] calculate the additional flow step and possibly terminate the iteration
	int p, subpixel_factor, x, y, it, step_threshold, step_x, step_y, v_x, v_y, Det, j;
	int b_x, b_y, patch_size, padded_patch_size, error, step_size;
	unsigned int ix1, ix2;
	int* I_padded_neighborhood; int* I_neighborhood; int* J_neighborhood;
	int* DX; int* DY; int* ImDiff; int* IDDX; int* IDDY;
	int G[4];
	int error_threshold;

	// set the image width and height
	IMG_WIDTH = imW;
	IMG_HEIGHT = imH;
	// spatial resolution of flow is 1 / subpixel_factor
	subpixel_factor = 100;
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

//	int xp, yp;
	int xpout, ypout;
	*flow_point = 0;
//	int one_plus_eps = 1.001f*subpixel_factor;   /* To prevent rounding errors */

	for(p = 0; p < n_found_points; p++)
	{
		// We want to be able to take steps in the image of 1 / subpixel_factor:
//		xp = p_x[p] * subpixel_factor;
//		yp = p_y[p] * subpixel_factor;
		p_x[p] = p_x[p] * subpixel_factor;
		p_y[p] = p_y[p] * subpixel_factor;
//		printf("Subpixel coordinate: (%d,%d)\n\r", xp, yp);

		for(j=nLevels-1; j>=0; j--)
		{
//			xp /= subsampling;  yp /= subsampling;
			p_x[p] /= subsampling;  p_y[p] /= subsampling;
			IMG_WIDTH /= subsampling; IMG_HEIGHT /= subsampling;
			error_threshold /= subsampling;
		}
//		printf("Lowest level coordinate: (%d,%d)\n\r",xp,yp);
		xpout = p_x[p]; ypout = p_y[p];

		for(j=nLevels-1; j>=0; j--)
		{
//			printf("j = %d p = %d\n",j,p);
			p_x[p] *= subsampling;  p_y[p] *= subsampling;
			xpout *= subsampling;  ypout *= subsampling;
			IMG_WIDTH *= subsampling; IMG_HEIGHT *= subsampling;
			error_threshold *= subsampling;
			printf("p[%d] = (%d, %d)\n\r",p, p_x[p], p_y[p]);
//			printf("imgSize = (%d,%d)\n",IMG_WIDTH,IMG_HEIGHT);
			//printf("*** NEW POINT ***\n\r");
			// status: point is not yet lost:
			status[p] = 1;

//			printf("current coordinate: (%d,%d)\n\r",xp,yp);
//			printf("condition: !(%d > %d && %d < %d && %d > %d && %d < %d ) \n",xp,((half_window_size+1) * subpixel_factor),xp, (IMG_WIDTH-half_window_size) * subpixel_factor,yp, ((half_window_size+1) * subpixel_factor), yp, (IMG_HEIGHT-half_window_size)*subpixel_factor);

			// if the pixel is outside the ROI in the image, do not track it:
//			if(!(xp > ((half_window_size+1) * subpixel_factor) && xp < (IMG_WIDTH-half_window_size) * subpixel_factor && yp > ((half_window_size+1) * subpixel_factor) && yp < (IMG_HEIGHT-half_window_size)*subpixel_factor))
			if(!(p_x[p] > ((half_window_size+1) * subpixel_factor) && p_x[p] < (IMG_WIDTH-half_window_size) * subpixel_factor && p_y[p] > ((half_window_size+1) * subpixel_factor) && p_y[p] < (IMG_HEIGHT-half_window_size)*subpixel_factor))
//			if( p_x[p] - ((half_window_size+1) * subpixel_factor) < 0 || (IMG_WIDTH-half_window_size) * subpixel_factor - p_x[p] < one_plus_eps ||
//				p_y[p] - ((half_window_size+1) * subpixel_factor) < 0 || (IMG_HEIGHT-half_window_size)* subpixel_factor - p_y[p] < one_plus_eps)
			{
//			  printf("Outside of ROI p_x[%d] = (%d, %d)\n\r",p, p_x[p], p_y[p]);
			  status[p] = 0;
			}

			// (1) determine the subpixel neighborhood in the old image
			// we determine a padded neighborhood with the aim of subsequent gradient processing:
			getSubPixel_gray(I_padded_neighborhood, old_gray_buf[j], p_x[p], p_y[p], half_window_size+1, subpixel_factor);
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
			  //printf("G[%d] = %d\n\r", it, G[it]);
			  G[it] /= 255; // to keep values in range
			  //printf("G[%d] = %d\n\r", it, G[it]);
			}
			// calculate G's determinant:
			Det = G[0] * G[3] - G[1] * G[2];
			//printf("Det = %d\n\r", Det);
			Det = Det / subpixel_factor; // so that the steps will be expressed in subpixel units
//			printf("Det = %d\n\r", Det);
			if(Det < 0.01f*subpixel_factor)
			{
//			  printf("small\n");
			  status[p] = 0;
			}

			// (4) iterate over taking steps in the image to minimize the error:
			it = 0;
			step_threshold = 2; // 0.2 as smallest step (L1)
			v_x = 0;
			v_y = 0;
			step_size = step_threshold + 1;

			while(status[p] == 1 && it < max_iterations && step_size >= step_threshold)
			{
			  //printf("it = %d, (p_x+v_x,p_y+v_y) = (%d,%d)\n\r", it, xp+v_x, yp+v_y);
			  //printf("it = %d;", it);
			  // if the pixel goes outside the ROI in the image, stop tracking:
			  if(!(xpout+v_x> ((half_window_size+1) * subpixel_factor) && xpout+v_x< ((int)IMG_WIDTH-half_window_size) * subpixel_factor && ypout+v_y>((half_window_size+1) * subpixel_factor) && ypout+v_y< ((int)IMG_HEIGHT-half_window_size)*subpixel_factor))
			  {
				printf("Outside of ROI p_xout[%d]\n\r",p);
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


			  getSubPixel_gray(J_neighborhood, new_gray_buf[j], xpout+v_x, ypout+v_y, half_window_size, subpixel_factor);
			  //     [b] determine the image difference between the two neighborhoods
			  //printf("I = ");
			  //printIntMatrix(I_neighborhood, patch_size, patch_size);
			  //printf("J = ");
			  //printIntMatrix(J_neighborhood, patch_size, patch_size);
			  //getSubPixel(J_neighborhood, new_gray_buf, subpixel_factor * ((xp+v_x)/subpixel_factor), subpixel_factor * ((yp+v_y) / subpixel_factor), half_window_size, subpixel_factor);
			  //printf("J2 = ");
			  //printIntMatrix(J_neighborhood, patch_size, patch_size);
			  //printf("figure(); subplot(1,2,1); imshow(I/255); subplot(1,2,2); imshow(J/255);\n\r");
			  getImageDifference(I_neighborhood, J_neighborhood, ImDiff, patch_size, patch_size);
			  //printf("ImDiff = ");
			  //printIntMatrix(ImDiff, patch_size, patch_size);
			  error = calculateError(ImDiff, patch_size, patch_size);
			  if(error > error_threshold && it > max_iterations / 2)
			  {
//				  printf("error\n");
				status[p] = 0;
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
//			  printf("b_x = %d; b_y = %d;\n\r", b_x, b_y);
			  //     [d] calculate the additional flow step and possibly terminate the iteration
			  step_x = (G[3] * b_x - G[1] * b_y) / Det;
			  step_y = (G[0] * b_y - G[2] * b_x) / Det;
			  v_x += step_x;
			  v_y += step_y; // - (?) since the origin in the image is in the top left of the image, with y positive pointing down
//			  xpout += v_x;
//			  ypout += v_y;
//			  printf("step = [%d,%d]; v = [%d,%d];\n\r", step_x, step_y, v_x, v_y);
			  //printf("pause(0.5);\n\r");
			  // next iteration
			  it++;
			  step_size = abs(step_x);
			  step_size += abs(step_y);
//			  printf("status = %d, it = %d, step_size = %d\n\r", status[p], it, step_size);
			} // iteration to find the right window in the new image
			xpout += v_x;
			ypout += v_y;
			//printf("figure(); plot(error(1:(it+1)));\n\r");

		}
//		printf("it = %d\n",it);
//		new_x[p] = xpout / subpixel_factor;
//		new_y[p] = ypout / subpixel_factor;
		p_x[p] /= subpixel_factor;
		p_y[p] /= subpixel_factor;
		new_x[*flow_point] = xpout / subpixel_factor;
		new_y[*flow_point] = ypout / subpixel_factor;
		(*flow_point)++;
//		if(status[p] && !(xpout < borderx*subpixel_factor || xpout > (imW-1-borderx)*subpixel_factor || ypout < bordery*subpixel_factor || ypout > (imH-1-bordery)*subpixel_factor))
//		{
//			new_x[*flow_point] = xpout / subpixel_factor;
//			new_y[*flow_point] = ypout / subpixel_factor;
//			(*flow_point)++;
//		}

  }


  // free all allocated variables:
  free((char*) I_padded_neighborhood);
  free((char*) I_neighborhood);
  free((char*) J_neighborhood);
  free((char*) DX);
  free((char*) DY);
  free((char*) ImDiff);
  free((char*) IDDX);
  free((char*) IDDY);
  // no errors:
  return OK;
}


extern void showFlow(unsigned char * frame_buf, int* x, int* y, int* status, int n_found_points, int* new_x, int* new_y, int imgW, int imgH)
{
  int p, i, j;
  unsigned int ix;

  IMG_WIDTH = imgW;
  IMG_HEIGHT = imgH;
  // in this simple version, we do not draw lines:
  for(p = 0; p < n_found_points; p++)
  {
    //printf("Point %d, status = %d, (x, y) = (%d, %d), (new_x, new_y) = (%d, %d)\n\r", p, status[p], x[p], y[p], new_x[p], new_y[p]);
    if(x[p] >= 1 && y[p] >= 1 && x[p] < (int)IMG_WIDTH - 1 && y[p] < (int)IMG_HEIGHT - 1)
    {
      for(i = -1; i <= 1; i++)
      {
        for(j = -1; j <= 1; j++)
        {
//          if(status[p] == 1)
//          {
//            ix = uint_index((unsigned int) (x[p]+i), (unsigned int) (y[p]+j));
//            redPixel(frame_buf, ix);
//            ix = uint_index((unsigned int) (new_x[p]+i), (unsigned int) (new_y[p]+j));
//            greenPixel(frame_buf, ix);
//          }
//          else
//          {
            ix = uint_index((unsigned int) (x[p]+i), (unsigned int) (y[p]+j));
            redPixel(frame_buf, ix);
//          }
        }
      }
    }
  }
}

void MatMul(float* Mat3, float* Mat1, float* Mat2, int MatW, int MatH)
{
  unsigned int i;
  unsigned int j;
  unsigned int ii;

  for(i = 0; i < MatW; i++)
  {
    for(j = 0; j < MatH; j++)
    {
      ii = (j * MatW + i);
      Mat3[ii] = Mat1[ii] * Mat2[ii];
    }
  }
}

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
			if (*divergence < 3.0 && *divergence > -3.0) {
				div_buf[div_point] = *divergence;
				div_point = (div_point+1) %15;
			}
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

void quick_sort (float *a, int n) {
    if (n < 2)
        return;
    float p = a[n / 2];
    float *l = a;
    float *r = a + n - 1;
    while (l <= r) {
        if (*l < p) {
            l++;
            continue;
        }
        if (*r > p) {
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

void quick_sort_int (int *a, int n) {
    if (n < 2)
        return;
    int p = a[n / 2];
    int *l = a;
    int *r = a + n - 1;
    while (l <= r) {
        if (*l < p) {
            l++;
            continue;
        }
        if (*r > p) {
            r--;
            continue; // we need to check the condition (l <= r) every time we change the value of l or r
        }
        int t = *l;
        *l++ = *r;
        *r-- = t;
    }
    quick_sort_int(a, r - a + 1);
    quick_sort_int(l, a + n - l);
}

//void CvtYUYV2Gray(unsigned char *grayframe, unsigned char *frame, int imW, int imH){
//    int x, y;
//    unsigned char *Y, *gray;
//    //get only Y component for grayscale from (Y1)(U1,2)(Y2)(V1,2)
//    for (y = 0; y < imH; y++) {
//        Y = frame + (imW * 2 * y);
//        gray = grayframe + (imW * y);
//        for (x=0; x < imW; x += 2) {
//            gray[x] = *Y;
//            Y += 2;
//            gray[x + 1] = *Y;
//            Y += 2;
//        }
//    }
//}

void CvtYUYV2Gray(unsigned char *grayframe, unsigned char *frame, int imW, int imH){
    int x, y;
    unsigned char *Y, *gray;
    //get only Y component for grayscale from (U1,2)(Y1)(V1,2)(Y2)
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

/* convert from 4:2:2 YUYV interlaced to RGB24 */
/* based on ccvt_yuyv_bgr32() from camstream */
//http://answers.opencv.org/question/9659/solved-how-to-make-opencv-to-ask-libv4l-yuyv/
#define SAT(c) \
   if (c & (~255)) { if (c < 0) c = 0; else c = 255; }

void yuyv_to_rgb24 (int width, int height, unsigned char *src, unsigned char *dst)
{
   int l, c;
   int r, g, b, cr, cg, cb, kl1, kl2;

   l = height;
   while (l--) {
      c = width >> 1;
      while (c--) {
         kl1 = *src++;
         cb = ((*src - 128) * 454) >> 8;
         cg = (*src++ - 128) * 88;
         kl2 = *src++;
         cr = ((*src - 128) * 359) >> 8;
         cg = (cg + (*src++ - 128) * 183) >> 8;

         r = kl1 + cr;
         b = kl1 + cb;
         g = kl1 - cg;
         SAT(r);
         SAT(g);
         SAT(b);

     *dst++ = b;
     *dst++ = g;
     *dst++ = r;

         r = kl2 + cr;
         b = kl2 + cb;
         g = kl2 - cg;
         SAT(r);
         SAT(g);
         SAT(b);

     *dst++ = b;
     *dst++ = g;
     *dst++ = r;
      }
   }
}


void uyvy_to_rgb24 (int width, int height, unsigned char *src, unsigned char *dst)
{
   int l, c;
   int r, g, b, cr, cg, cb, kl1, kl2;

   l = height;
   while (l--) {
      c = width >> 1;
      while (c--) {
         cb = ((*src++ - 128) * 454) >> 8;
         kl1 = *src;
         cg = (*src++ - 128) * 88;
         cr = ((*src++ - 128) * 359) >> 8;
         kl2 = *src;
         cg = (cg + (*src++ - 128) * 183) >> 8;

         r = kl1 + cr;
         b = kl1 + cb;
         g = kl1 - cg;
         SAT(r);
         SAT(g);
         SAT(b);

     *dst++ = b;
     *dst++ = g;
     *dst++ = r;

         r = kl2 + cr;
         b = kl2 + cb;
         g = kl2 - cg;
         SAT(r);
         SAT(g);
         SAT(b);

     *dst++ = b;
     *dst++ = g;
     *dst++ = r;
      }
   }
}


void setPointsToFlowPoints(struct flowPoint flow_points[], struct detectedPoint detected_points[], int *flow_point_size, int *count, int MAX_COUNT)
{
	// set the points array to match the flow points:
	int new_size = (*flow_point_size < MAX_COUNT) ? *flow_point_size : MAX_COUNT;
	*count = new_size;
	int i;
	for(i = 0; i < new_size; i++)
	{
		detected_points[i].x = flow_points[i].x;
		detected_points[i].y = flow_points[i].y;
	}
}

void findPoints(unsigned char *gray_frame, unsigned char *frame, int imW, int imH, int *count, int max_count, int MAX_COUNT, struct flowPoint flow_points[],int *flow_point_size, struct detectedPoint detected_points[])
{
	// a) find suitable points in the image
	// b) compare their locations with those of flow_points, only allowing new points if far enough
	// c) update flow_points (and points) to include the new points
	// d) put the flow point into the points-array, which will be used for the flow

	// a)

	// FAST corner:
	int fast_threshold = 15; //10
	xyFAST* pnts_fast;

	CvtYUYV2Gray(gray_frame, frame, imW, imH); // convert to gray scaled image is a must for FAST corner

	pnts_fast = fast9_detect((const byte*)gray_frame, imW, imH, imW, fast_threshold, count); //widthstep for gray-scaled image = its width; int widthstep = ((width*sizeof(unsigned char)*nchannels)%4!=0)?((((width*sizeof(unsigned char)*nchannels)/4)*4) + 4):(width*sizeof(unsigned char)*nchannels);

	// transform the points to the format we need (is also done in the other corner finders
	*count = (*count > MAX_COUNT) ? MAX_COUNT : *count;
	int i,j;
	for(i = 0; i < *count; i++)
	{
		detected_points[i].x = pnts_fast[i].x;
		detected_points[i].y = pnts_fast[i].y;
	}

	free(pnts_fast);

	// if more points than the user-determined maximum are found, ignore the superfluous points:
	if(*count > max_count) *count = max_count;
//	if(*count < 5)
//	{
//		printf("Number of points found: %d\n", *count);
//	}

	// b)
	float distance2;
	float min_distance = 10; //10
	float min_distance2 = min_distance*min_distance;
	int new_point;

	int max_new_points = (*count < max_count - *flow_point_size) ? *count : max_count - *flow_point_size; //flow_point_size = [0,25]

	for(i = 0; i < max_new_points; i++)
	{
		new_point = 1;

		for(j = 0; j < *flow_point_size; j++)
		{
			// distance squared:
			distance2 = (detected_points[i].x - flow_points[j].x)*(detected_points[i].x - flow_points[j].x) +
						(detected_points[i].y - flow_points[j].y)*(detected_points[i].y - flow_points[j].y);
			if(distance2 < min_distance2)
			{
				new_point = 0;
			}
		}

		// c)
		if(new_point)
		{
			// add the flow_points:
			flow_points[*flow_point_size].x = detected_points[i].x;
			flow_points[*flow_point_size].y = detected_points[i].y;
			flow_points[*flow_point_size].prev_x = detected_points[i].x;
			flow_points[*flow_point_size].prev_y = detected_points[i].y;
			flow_points[*flow_point_size].dx = 0;
			flow_points[*flow_point_size].dy = 0;
			flow_points[*flow_point_size].new_dx = 0;
			flow_points[*flow_point_size].new_dy = 0;
			(*flow_point_size)++;
		}
	}
	setPointsToFlowPoints(flow_points, detected_points, flow_point_size, count, MAX_COUNT);
}

void trackPoints(unsigned char *frame, unsigned char *prev_frame, int imW, int imH, int *count, int max_count, int MAX_COUNT, struct flowPoint flow_points[],int *flow_point_size, struct detectedPoint detected_points0[],struct detectedPoint detected_points1[], int *x, int *y, int *new_x, int *new_y, int *dx, int *dy, int *status)
{

	// a) track the points to the new image
	// b) quality checking  for eliminating points (status / match error / tracking the features back and comparing / etc.)
	// c) update the points (immediate updatCvtYUYV2Gray(gray_frame, frame, imW, imH); e / Kalman update)
	int error_opticflow = 0;

	int i;

	// a) track the points to the new image

	if( *count > 0)
    {
			for(i=0; i<*count; i++)
			{
				x[i] = detected_points0[i].x;
				y[i] = detected_points0[i].y;
			}
			error_opticflow = opticFlowLK(frame, prev_frame, x, y, *count, imW, imH, new_x, new_y, status, 5, MAX_COUNT);
//			for(i=0; i<*count; i++)
//			{
//				detected_points1[i].x = new_x[i];
//				detected_points1[i].y = new_y[i];
//			}
	}

	if(error_opticflow == 0)
	{
		// b) quality checking  for eliminating points (status / match error / tracking the features back and comparing / etc.)
		int remove_point = 0;
		int c;
		for(i = *flow_point_size-1; i >= 0; i-- )
	    {
	        if(!status[i])
			{
				remove_point = 1;
			}

			// error[i] can also be used, etc.

			if(remove_point)
			{
				// we now erase the point if it is not observed in the new image
				// later we may count it as a single miss, allowing for a few misses
				for(c = i; c < *flow_point_size-1; c++)
				{
					flow_points[c].x = flow_points[c+1].x;
					flow_points[c].y = flow_points[c+1].y;
					flow_points[c].prev_x= flow_points[c+1].prev_x;
					flow_points[c].prev_y = flow_points[c+1].prev_y;
					flow_points[c].dx = flow_points[c+1].dx;
					flow_points[c].dy = flow_points[c+1].dy;
					flow_points[c].new_dx = flow_points[c+1].new_dx;
					flow_points[c].new_dy = flow_points[c+1].new_dy;
				}
				(*flow_point_size)--;
			}
			else
			{
				flow_points[i].new_dx = new_x[i] - x[i];
				flow_points[i].new_dy = new_y[i] - y[i];
			}
		}

		// c) update the points (immediate update / Kalman update)
		*count = *flow_point_size;

		for(i = 0; i < *count; i++)
		{
			// immediate update:
			flow_points[i].dx = flow_points[i].new_dx;
			flow_points[i].dy = flow_points[i].new_dy;
			flow_points[i].prev_x = flow_points[i].x;
			flow_points[i].prev_y = flow_points[i].y;
			flow_points[i].x = flow_points[i].x + flow_points[i].dx;
			flow_points[i].y = flow_points[i].y + flow_points[i].dy;
			// save good tracked points for the next frame
			detected_points1[i].x = flow_points[i].x;
			detected_points1[i].y = flow_points[i].y;
		}
	}

	return;
}
unsigned int OF_buf_point = 0;
unsigned int OF_buf_point2 = 0;
float tot_x, tot_y, x_avg, y_avg, x_buf[24], y_buf[24], x_buf2[24], y_buf2[24];

#ifdef FLOW_DEROTATE
void OFfilter(float *OFx, float *OFy, float * struct flowPoint flow_points[], int count, int OF_FilterType, float diff_roll, float diff_pitch)
#else
void OFfilter(float *OFx, float *OFy, struct flowPoint flow_points[], int count, int OF_FilterType)
#endif
{
	tot_x = 0.0;
	tot_y = 0.0;

	for (int i=0; i<count;i++)
	{
		tot_x = tot_x + (float) flow_points[i].dx;
		tot_y = tot_y + (float) flow_points[i].dy;
		tot_x = tot_x/count;
		tot_y = tot_y/count;

#ifdef FLOW_DEROTATE

#endif
	}

	if(OF_FilterType == 1) //1. moving average 2. moving median
	{

		x_avg = 0.0;
		y_avg = 0.0;

		if(count)
		{
			x_buf[OF_buf_point] = tot_x;
			y_buf[OF_buf_point] = tot_y;
		}
		else
		{
			x_buf[OF_buf_point] = 0.0;
			y_buf[OF_buf_point] = 0.0;
		}
		OF_buf_point = (OF_buf_point+1) %5;

		for (int i=0;i<5;i++) {
			x_avg+=x_buf[i]*0.2;
			y_avg+=y_buf[i]*0.2;
		}

		*OFx = x_avg;
		*OFy = y_avg;

	}
	else if(OF_FilterType == 2)
	{
		if(count)
		{
			x_buf2[OF_buf_point2] = tot_x;
			y_buf2[OF_buf_point2] = tot_y;
		}
		else
		{
			x_buf2[OF_buf_point2] = 0.0;
			y_buf2[OF_buf_point2] = 0.0;
		}
		OF_buf_point2 = (OF_buf_point2+1) %11;

		quick_sort(x_buf2,11);
		quick_sort(y_buf2,11);

		*OFx = x_buf2[6];
		*OFy = y_buf2[6];
	}
	else
	{
		printf("no filter type selected!\n");
	}


}

void OFfilter2(float *OFx, float *OFy, float dx, float dy, int count, int OF_FilterType)
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
			*z_x = 0.0;
			*z_y = 0.0;
			*three_dimensionality = 0.0;
			*POE_x = 0.0;
			*POE_y = 0.0;
			*divergence = 0.0;
			*mean_tti = 1000.0f / 60;
			*median_tti = *mean_tti;
			*d_heading = 0;
			*d_pitch = 0;
			return;
		}
		float pu[3], pv[3];

		//float divergence_error;
		float min_error_u, min_error_v;
		fitLinearFlowField(pu, pv, divergence_error, x, y, dx, dy, count, n_samples, &min_error_u, &min_error_v, n_iterations, error_threshold, n_inlier_minu, n_inlier_minv);

		extractInformationFromLinearFlowField(divergence, mean_tti, median_tti, d_heading, d_pitch, pu, pv, imW, imH, DIV_FILTER);

		slopeEstimation(z_x, z_y, three_dimensionality, POE_x, POE_y, *d_heading, *d_pitch, pu, pv, min_error_u, min_error_v);

//		printf("0:%d\n1:%f\n",count,divergence[0]);
}

unsigned int line_mov_block = 6; //default: 30
float line_div_buf[6];
unsigned int line_div_point = 0;

void lineDivergence(float *divergence, int *x, int *y, int *new_x, int *new_y, int count)
{
	float inner = 0;
	float outer = 0;

	for(int i=0; i<(count-1); i++)
	{
		inner = inner + sqrt((float)((x[i+1]-x[i])*(x[i+1]-x[i])+(y[i+1]-y[i])*(y[i+1]-y[i])));
		outer = outer + sqrt((float)((new_x[i+1]-new_x[i])*(new_x[i+1]-new_x[i])+(new_y[i+1]-new_y[i])*(new_y[i+1]-new_y[i])));
	}

	if (inner == outer)
	{
		*divergence = 0;
	}
	else
	{
		*divergence = inner/(outer-inner);
	}

	float div_avg = 0.0f;
	int averagefilter = 0;

	if(averagefilter == 1)
	{
		//*DIV_FILTER = 1;
		if (*divergence < 100.0 && *divergence > -100.0) {
			line_div_buf[line_div_point] = *divergence;
			line_div_point = (line_div_point+1) %line_mov_block; // index starts from 0 to line_mov_block
		}

		int im;
		for (im=0;im<line_mov_block;im++) {
			div_avg+=line_div_buf[im];
		}
		*divergence = div_avg/ line_mov_block;
	}

}

void subimage(unsigned char *gray_frame, unsigned char *subframe, int subimH, int subimW, int wInit, int hInit)
{
    int x, y;
    unsigned int ix, isub;
    isub = 0;
    for (x=wInit; x < wInit+subimW; x++)
    {
    	for (y = hInit; y < hInit+subimH; y++)
        {
            ix = (y * subimW + x);
            subframe[isub] = gray_frame[ix];
            isub++;
        }
    }
}
/*
void findDistributedPoints(unsigned char *gray_frame, unsigned char *frame, int imW, int imH, int *count, int max_count, int MAX_COUNT, struct flowPoint flow_points[],int *flow_point_size, struct detectedPoint detected_points[], int *status)
{
	// a) Divide image into 9 regions
	// b) Run corner detection in these regions

	int nWidth, nHeight, subimH, subimW, wInit, hInit, xcount, next_region, n_accu, n_run;
	nWidth = 3;
	nHeight = 3;
	subimH = (imH-imH%nHeight)/nHeight; // a few pixels (imW%nHeight) are ignored!!
	subimW = (imW-imW%nWidth)/nWidth;
	wInit = 0; hInit = 0; xcount = 0; next_region = 0; n_accu = 1; n_run = 0;
	unsigned char *subframe;
	subframe = (unsigned char *) calloc(subimH*subimW,sizeof(unsigned char));

	CvtYUYV2Gray(gray_frame, frame, imW, imH); // convert to gray scaled image is a must for FAST corner

	// FAST corner:
	int fast_threshold = 15; //10
	xyFAST* pnts_fast;

	for (int i=0; i<nHeight; i++)
	{
		for (int j=0; j<nWidth; j++)
		{
			if(!status[xcount] || next_region)
			{
				subimage(gray_frame, subframe, subimH, subimW, wInit, hInit);
				pnts_fast = fast9_detect((const byte*)subframe, subimW, subimH, subimW, fast_threshold, count); //widthstep for gray-scaled image = its width; int widthstep = ((width*sizeof(unsigned char)*nchannels)%4!=0)?((((width*sizeof(unsigned char)*nchannels)/4)*4) + 4):(width*sizeof(unsigned char)*nchannels);

				if(*count)
				{
					n_run = (n_accu < *count) ? n_accu : n_accu - *count;
					for(int k=0; k<n_run; k++)
					{
						flow_points[xcount-k].x = pnts_fast[k].x + wInit;
						flow_points[xcount-k].y = pnts_fast[k].y + hInit;
						flow_points[xcount-k].prev_x = pnts_fast[k].x + wInit;
						flow_points[xcount-k].prev_y = pnts_fast[k].y + hInit;
						flow_points[xcount-k].dx = 0;
						flow_points[xcount-k].dy = 0;
						flow_points[xcount-k].new_dx = 0;
						flow_points[xcount-k].new_dy = 0;
					}
					n_accu -= n_run;
					next_region = 0;
				}
				else
				{
					next_region = 1;
					n_accu++; // accumulate corners to next region
				}
				free(pnts_fast);
			}
			xcount ++;
			wInit += subimW;
		}
		wInit = 0;
		hInit += subimH;
	}
	*flow_point_size = (xcount + 1) - (n_accu -1);
	free(subframe);
}
*/
void trackDistributedPoints(unsigned char *frame, unsigned char *prev_frame, int imW, int imH, int *count, int max_count, int MAX_COUNT, struct flowPoint flow_points[],int *flow_point_size, struct detectedPoint detected_points[], int *x, int *y, int *new_x, int *new_y, int *dx, int *dy, int *status)
{

	int error_opticflow = 0;

	int i;

	// a) track the points to the new image

	if( *count > 0)
    {
			for(i=0; i<*count; i++)
			{
				x[i] = detected_points[i].x;
				y[i] = detected_points[i].y;
			}
			error_opticflow = opticFlowLK(frame, prev_frame, x, y, *count, imW, imH, new_x, new_y, status, 5, MAX_COUNT);
	}

	if(error_opticflow == 0)
	{
		// b) quality checking  for eliminating points (status / match error / tracking the features back and comparing / etc.)
		int remove_point = 0;
		int c;
		for(i = *flow_point_size-1; i >= 0; i-- )
	    {
	        if(!status[i])
			{
				remove_point = 1;
			}

			// error[i] can also be used, etc.https://www.facebook.com/kwongwah.my/photos/a.144520172328175.29951.144505178996341/656484874465033/?type=1&theater

			if(remove_point)
			{
				// we now erase the point if it is not observed in the new image
				// later we may count it as a single miss, allowing for a few misses
				for(c = i; c < *flow_point_size-1; c++)
				{
					flow_points[c].x = flow_points[c+1].x;
					flow_points[c].y = flow_points[c+1].y;
					flow_points[c].prev_x= flow_points[c+1].prev_x;
					flow_points[c].prev_y = flow_points[c+1].prev_y;
					flow_points[c].dx = flow_points[c+1].dx;
					flow_points[c].dy = flow_points[c+1].dy;
					flow_points[c].new_dx = flow_points[c+1].new_dx;
					flow_points[c].new_dy = flow_points[c+1].new_dy;
				}
				(*flow_point_size)--;
			}
			else
			{
				flow_points[i].new_dx = new_x[i] - x[i];
				flow_points[i].new_dy = new_y[i] - y[i];
			}
		}

		// c) update the points (immediate update / Kalman update)
		*count = *flow_point_size;

		for(i = 0; i < *count; i++)
		{
			// immediate update:
			flow_points[i].dx = flow_points[i].new_dx;
			flow_points[i].dy = flow_points[i].new_dy;
			flow_points[i].prev_x = flow_points[i].x;
			flow_points[i].prev_y = flow_points[i].y;
			flow_points[i].x = flow_points[i].x + flow_points[i].dx;
			flow_points[i].y = flow_points[i].y + flow_points[i].dy;
		}
	}

	return;
}

void YUV422TORGB(unsigned char *YUV, unsigned char *RGB, unsigned char *GRAY, int Width, int Height)
{
    int i, j;
    unsigned char *Y, *gray;
    unsigned char *rgb;
    int y11, y22, u, v, temp;

    //get only Y component for grayscale from (U)(Y11)(V)(Y22)
    for (j = 0; j < Height; j++) {
    	Y = YUV + (Width * 2 * j);
    	gray = GRAY + (Width * j);
    	rgb = RGB + (Width * j * 3);
        for (i=0; i < Width; i += 2)
        {
        	// http://en.wikipedia.org/wiki/YUV#Converting_between_Y.27UV_and_RGB
        	// ITU-R version
        	u = (int) *Y - 128;
        	Y += 1;
        	gray[i] = *Y;
        	y11 = (int) *Y;
        	Y += 1;
        	v = (int) *Y - 128;
        	Y += 1;
        	gray[i+1] = *Y;
        	y22 = (int) *Y;
        	Y += 1;

        	temp = (y11 + v + (v>>2) + (v>>3) + (v>>5));
        	rgb [3*i] = (unsigned char) ((temp < 0) ? 0: ((temp > 255) ? 255: temp));
        	temp = (y11 - ((u>>2) + (u>>4) + (u>>5)) - ((v>>1) + (v>>3) + (v>>4) + (v>>5)));
        	rgb [3*i + 1] = (unsigned char) ((temp < 0) ? 0: ((temp > 255) ? 255: temp));
        	temp = (y11 + u + (u>>1) + (u>>2) + (u>>6));
        	rgb [3*i + 2] = (unsigned char) ((temp < 0) ? 0: ((temp > 255) ? 255: temp));
        	temp = (y22 + v + (v>>2) + (v>>3) + (v>>5));
        	rgb [3*i + 3] = (unsigned char) ((temp < 0) ? 0: ((temp > 255) ? 255: temp));
        	temp = (y22 - ((u>>2) + (u>>4) + (u>>5)) - ((v>>1) + (v>>3) + (v>>4) + (v>>5)));
        	rgb [3*i + 4] = (unsigned char) ((temp < 0) ? 0: ((temp > 255) ? 255: temp));
        	temp = (y22 + u + (u>>1) + (u>>2) + (u>>6));
        	rgb [3*i + 5] = (unsigned char) ((temp < 0) ? 0: ((temp > 255) ? 255: temp));

        	// android
//        	rgb [3*i] =  (unsigned char) (y11 + ((v * 359) >> 8));
//        	rgb [3*i + 1] = (unsigned char)  (y11 - ((u * 88 + v * 183) >> 8));
//        	rgb [3*i + 2] = (unsigned char)  (y11 + ((u * 454) >> 8));
//        	rgb [3*i + 3] = (unsigned char) (y22 + ((v * 359) >> 8));
//        	rgb [3*i + 4] = (unsigned char)  (y22 - ((u * 88 + v * 183) >> 8));
//        	rgb [3*i + 5] = (unsigned char)  (y22 + ((u * 454) >> 8));
//
//            SAT(rgb [3*i]);
//            SAT(rgb [3*i + 1]);
//            SAT(rgb [3*i + 2]);
//            SAT(rgb [3*i + 3]);
//            SAT(rgb [3*i + 4]);
//            SAT(rgb [3*i + 5]);

        	// NTSC standard
//        	u = (int) *Y - 128;
//        	Y += 1;
//        	y11 = (int)*Y - 16;
//        	Y += 1;
//        	v = (int) *Y - 128;
//        	Y += 1;
//        	y22 = (int) *Y - 16;
//        	Y += 1;
//
//        	temp = (298 * y11 + 409 * v) >> 8;
//        	rgb [3*i] =(unsigned char) ((temp < 0) ? 0: ((temp > 255) ? 255: temp));
//        	temp = (298 * y11 - 100 * u - 208 * v) >> 8;
//        	rgb [3*i + 1] = (unsigned char) ((temp < 0) ? 0: ((temp > 255) ? 255: temp));
//        	temp = (298 * y11 + 516 * u) >> 8;
//        	rgb [3*i + 2] = (unsigned char) ((temp < 0) ? 0: ((temp > 255) ? 255: temp));
//
//         	temp = (298 * y22 + 409 * v) >> 8;
//			rgb [3*i + 3] =(unsigned char) ((temp < 0) ? 0: ((temp > 255) ? 255: temp));
//			temp = (298 * y22 - 100 * u - 208 * v) >> 8;
//			rgb [3*i + 4] = (unsigned char) ((temp < 0) ? 0: ((temp > 255) ? 255: temp));
//			temp = (298 * y22 + 516 * u) >> 8;
//			rgb [3*i + 4] = (unsigned char) ((temp < 0) ? 0: ((temp > 255) ? 255: temp));
        }
    }
}

int i_frame = 0;

void saveSingleImageDataFile(unsigned char *frame_buf, int width, int height, char filename[100])
{

	FILE *fp;

	fp=fopen(filename, "w");

	// convert to grayscale image (verified)
//	unsigned char *grayframe;
//	grayframe = (unsigned char*) calloc(width*height,sizeof(unsigned char));
//	CvtYUYV2Gray(grayframe, frame_buf, width, height);

	// convert to rgb
//	unsigned char *RGB;
//	RGB = (unsigned char *)calloc(width*height*3,sizeof(unsigned char));
//	unsigned char *grayframe;
//	grayframe = (unsigned char*) calloc(width*height,sizeof(unsigned char));
//	YUV422TORGB(frame_buf, RGB, grayframe, width, height);
//	uyvy_to_rgb24 (width, height, frame_buf, RGB);
	if(fp == NULL)
	{
		perror("Error while opening the file.\n");
	}
	else
	{
		for(int i = 0; i<height; i++)
		{
	//		for(int j = 0; j<width*3; j++) //for RGB
			for(int j = 0; j<width*2; j++) // for UYVY
	//		for(int j = 0; j<width; j++)   // for grayscale
			{
	//			fprintf(fp, "%u\n",RGB[i * width * 3 + j]); // for RGB
				fprintf(fp, "%u\n",frame_buf[i * width * 2 + j]); // for UYVY
	//			fprintf(fp, "%u\n",grayframe[i * width + j]); // use "mat2gray()" to convert it to grayscale image and then show it with "imshow()"
			}
		}
		fclose(fp);
	}
//	free(grayframe);
//	free(RGB);
}

void DictionaryTrainingYUV(float ****color_words, unsigned char *frame, int n_words, int patch_size, int *learned_samples, int n_samples_image, float alpha, int Width, int Height, int *filled)
{
	int i, j, w, s, word, c; //loop variables
	int x,y;
	float error_word;
	int n_clusters = n_words;
	int clustered_ps = patch_size; // use even number fo YUV image

	unsigned char *buf;
//	buf = NULL;
	// ************************
	//       LEARNING
	// ************************
	//printf("Learning\n");
	if(!(*filled))
	{
		// **************
		// INITIALISATION
		// **************

		// we fix the values, so that the user cannot change them anymore with the control of the module.
//		printf("n_words = %d, patch_size = %d, n_samples = %d\n", n_words, patch_size, n_samples);

		// in the first image, we fill the neighbours/ words with the present patches
		for(w = 0; w < n_words; w++)
		{
			// select a coordinate
			x = rand() % (Width - clustered_ps);
			y = rand() % (Height - clustered_ps);

			// create a word
//			float *** c_word;
//			c_word = (float ***)calloc(clustered_ps,sizeof(float**));
//
//			for(i = 0; i < clustered_ps; i++)
//			{
//				c_word[i] = (float **)calloc(clustered_ps,sizeof(float *));
//
//				for(j = 0; j < clustered_ps;j++)
//				{
//					c_word[i][j] = (float *)calloc(3,sizeof(float));
//				}
//			}
//			int i_frame = 0;

			// take the sample
			for(i = 0; i < clustered_ps; i++)
			{
				buf = frame + (Width * 2 * (i+y)) + 2*x;
				for(j = 0; j < clustered_ps; j++)
				{
					// put it in a word
					// U/V component
//					c_word[i][j][1]
					color_words[w][i][j][0] = (float) *buf;
					// Y1/Y2 component
//					c_word[i][j][0]
					buf += 1;
					color_words[w][i][j][1] = (float) *buf;
					buf += 1;
//					printf("%f %f ",color_words[w][i][j][0],color_words[w][i][j][1]);
//					printf("%u %u ",frame[(Width * 2 * (i+y)) + 2*(x + j)],frame[(Width * 2 * (i+y)) + 2*(x + j ) + 1]);
//					printf("%u",frame[i_frame++]);
				}
			}
//			printf("\n");
		}
		*filled = 1;
	}
	else
	{
		printf("Learning\n");
		float *word_distances, ***p;
		word_distances = (float *)calloc(n_clusters,sizeof(float));
		p = (float ***)calloc(clustered_ps,sizeof(float**));

		for(i = 0; i < clustered_ps; i++)
		{
			p[i] = (float **)calloc(clustered_ps,sizeof(float*));
			for(j = 0; j < clustered_ps;j++)
			{
				p[i][j] = (float *)calloc(2,sizeof(float));
			}
		}

		for(s = 0; s < n_samples_image; s++)
		{
			// select a random sample from the image
			x = rand() % (Width - clustered_ps);
			y = rand() % (Height - clustered_ps);

			// reset word_distances
			for(word = 0; word < n_clusters; word++)
			{
				word_distances[word] = 0;
			}
			// extract sample
			for(i = 0; i < clustered_ps; i++)
			{
				buf = frame + (Width * 2 * (i+y)) + 2*x;
				for(j = 0; j < clustered_ps; j++)
				{
					// U/V component
		        	p[i][j][0] = (float) *buf;
					// Y1/Y2 component
					buf += 1;
					p[i][j][1] = (float) *buf;
					buf += 1;
				}
			}

			// determine distances to the words:
			for(i = 0; i < clustered_ps; i++)
			{
				for(j = 0; j < clustered_ps; j++)
				{
					for(c = 0; c < 2; c++)
					{
						// determine the distance to words
						for(word = 0; word < n_clusters; word++)
						{
							word_distances[word] += (p[i][j][c] - color_words[word][i][j][c])
													* (p[i][j][c] - color_words[word][i][j][c]);
						}
					}
				}
			}
			// determine the nearest neighbour
			// search the closest centroid
			int assignment = 0;
			float min_dist = word_distances[0];
			for(word = 1; word < n_clusters; word++)
			{
				if(word_distances[word] < min_dist)
				{
					min_dist = word_distances[word];
					assignment = word;
				}
			}

			// move the neighbour closer to the input
			for(i = 0; i < clustered_ps; i++)
			{
				for(j = 0; j < clustered_ps; j++)
				{
					for(c = 0; c < 2; c++)
					{
						// CDW: opgelet: weet je zeker dat deze error moet worden afgerond? Wat hier staat is error=floor(p-word)
						error_word = p[i][j][c] - color_words[assignment][i][j][c];
						color_words[assignment][i][j][c] += (alpha * error_word);
					}
				}
			}

			*learned_samples = *learned_samples + 1;
		}

		for(i = 0; i < clustered_ps; i++)
		{
			for(j = 0; j < clustered_ps; j++)
			{
                free(p[i][j]);
			}
			free(p[i]);
		}
		free(p);
		free(word_distances);
	}
	buf = NULL;
	free(buf);

//	printf("gathered samples = %d / %d.\n", learned_samples, N_SAMPLES);

}

void DistributionExtraction(float ****color_words, unsigned char *frame, float* word_distribution, int n_words, int patch_size, int n_samples_image, int RANDOM_SAMPLES, int Width, int Height, int border_width, int border_height)
{
	int i, j, s, word, c; //loop variables
	int x, y;
	int n_clusters = n_words;
	int clustered_ps = patch_size; // use even number fo YUV image
	int n_extracted_words = 0;
	int FULL_SAMPLING;

	unsigned char *buf;

	if(RANDOM_SAMPLES == 1)
	{
		FULL_SAMPLING = 0;
	}
	else
	{
		FULL_SAMPLING = 1;
	}
	// ************************
	//       EXECUTION
	// ************************
//	printf("Execution\n");

	float *word_distances, ***p;
	word_distances = (float *)calloc(n_clusters,sizeof(float));
	p = (float ***)calloc(clustered_ps,sizeof(float**));

	for(i = 0; i < clustered_ps; i++)
	{
		p[i] = (float **)calloc(clustered_ps,sizeof(float*));
		for(j = 0; j < clustered_ps;j++)
		{
			p[i][j] = (float *)calloc(2,sizeof(float));
		}
	}

	int finished = 0;
	x = 0;
	y = 0;
	s = 0;
	while(!finished)
	{
		if(RANDOM_SAMPLES)
		{
			s++;
			x = border_width + rand() % (Width - clustered_ps - 2*border_width);
			y = border_height + rand() % (Height - clustered_ps - 2*border_height);
		}
		// FASTER: What if we determine the closest word while updating the distances at the last pixel?
		// reset word_distances
		for(word = 0; word < n_clusters; word++)
		{
			word_distances[word] = 0;
		}
		// extract sample
		for(i = 0; i < clustered_ps; i++)
		{
			buf = frame + (Width * 2 * (i+y)) + 2*x;
			for(j = 0; j < clustered_ps; j++)
			{
				// U/V component
	        	p[i][j][0] = (float) *buf;
				// Y1/Y2 component
				buf += 1;
				p[i][j][1] = (float) *buf;
				buf += 1;
			}
		}

		// The following comparison with all words in the dictionary can be made faster by:
		// a) not comparing all pixels, but stopping early
		// b) using a different distance measure such as L1
		// determine distances:
		for(i = 0; i < clustered_ps; i++)
		{
			for(j = 0; j < clustered_ps; j++)
			{
				for(c = 0; c < 2; c++)
				{
					// determine the distance to words
					for(word = 0; word < n_clusters; word++)
					{
						word_distances[word] += (p[i][j][c] - color_words[word][i][j][c])
												* (p[i][j][c] - color_words[word][i][j][c]);
					}
				}
			}
		}

		// determine the nearest neighbour
		// search the closest centroid
		int assignment = 0;
		float min_dist = word_distances[0];
		for(word = 1; word < n_clusters; word++)
		{
			if(word_distances[word] < min_dist)
			{
				min_dist = word_distances[word];
				assignment = word;
			}
		}

		// put the assignment in the histogram
		word_distribution[assignment]++;

		n_extracted_words++;

		if(RANDOM_SAMPLES)
		{
			if(s == n_samples_image)
			{
				finished = 1;
			}
		}
		else
		{
			if(!FULL_SAMPLING)
				y += clustered_ps;
			else
				y++;

			if(y > Height - clustered_ps)
			{
				if(!FULL_SAMPLING)
					x += clustered_ps;
				else
					x++;
				y = 0;
			}
			if(x > Width - clustered_ps)
			{
				finished = 1;
			}
		}
	} // sampling

	// Normalize distribution:
	// can be made faster by not determining max, min, and avg (only needed for visualization)
	float max_p = 0;
	float min_p = 1;
	float avg_p = 0;
	for(i = 0; i < n_clusters; i++)
	{
		word_distribution[i] = word_distribution[i] / (float) n_extracted_words;
		if(word_distribution[i] > max_p) max_p = word_distribution[i];
		if(word_distribution[i] < min_p) min_p = word_distribution[i];
		avg_p += word_distribution[i];
	}
	avg_p = avg_p /(float) n_clusters;

//	if(FULL_SAMPLING)
//	{
//		FILE *myfile;
//		// if file left empty by user, fill it.
//		char full_filename[255];
//		strcpy(full_filename,"./full_sampling.dat");
//
//		myfile = fopen(full_filename,"a");
//		if (myfile == 0)
//		{
//			printf("VisualWordsKohonen_C failed to open file '%s'\n",full_filename);
//		}
//
//		for(i = 0; i < n_clusters; i++)
//		{
//			if(i < n_clusters - 1)
//				fprintf(myfile, "%f ", word_distribution[i]);
//			else
//				fprintf(myfile, "%f\n", word_distribution[i]);
//		}
//
//		if(myfile != 0)
//		{
//			fclose(myfile);
//		}
//	}

	for(i = 0; i < clustered_ps; i++)
	{
		for(j = 0; j < clustered_ps; j++)
		{
			   free(p[i][j]);
		}
		free(p[i]);
	}
	free(p);

	buf = NULL;
	free(buf);

} // EXECUTION

