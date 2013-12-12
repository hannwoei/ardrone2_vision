
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Computer Vision
#include "opticflow/optic_flow_gdc.h"
#include "trig.h"

// Own Header
#include "opticflow_code.h"

// Communication
#include "video_message_structs.h"

struct gst2ppz_message_struct gst2ppz;
struct ppz2gst_message_struct ppz2gst;

//#include "paparazzi.h"

// Settable by pluging
unsigned int imgWidth, imgHeight;
unsigned int verbose = 0;

// Local variables
//static unsigned char * img_uncertainty;
unsigned char * old_img;
int old_pitch, old_roll, old_alt;
float opt_angle_y_prev;
float opt_angle_x_prev;

// Called by plugin
void my_plugin_init(void)
{

	// Init variables
	ppz2gst.pitch = 0;
	ppz2gst.roll = 0;

	old_img = (unsigned char *) calloc(imgWidth*imgHeight*2,sizeof(unsigned char));
	old_pitch = 0;
	old_roll = 0;
	old_alt = 0;
	opt_angle_y_prev = 0;
	opt_angle_x_prev = 0;

	gst2ppz.counter = 0;

}

void my_plugin_run(unsigned char *frame)
{
	int MAX_POINTS, error;
	int n_found_points,mark_points;
	int *x, *y, *new_x, *new_y, *status;
	mark_points = 0;

	//save most recent values of attitude for the currently available frame
	int current_pitch = ppz2gst.pitch;
	int current_roll = ppz2gst.roll;
	int current_alt = ppz2gst.alt;

	x = (int *) calloc(40,sizeof(int));
	new_x = (int *) calloc(40,sizeof(int));
	y = (int *) calloc(40,sizeof(int));
	new_y = (int *) calloc(40,sizeof(int));
	status = (int *) calloc(40,sizeof(int));

	MAX_POINTS = 40;

	//active corner:
	int *active;
	active =(int *) calloc(40,sizeof(int));
	int GRID_ROWS = 5;
	int ONLY_STOPPED = 0;
	error = findActiveCorners(frame, GRID_ROWS, ONLY_STOPPED, x, y, active, &n_found_points, mark_points,imgWidth,imgHeight);

	//printf("error active corners = %d\n",error);
	//printf("num_points = %d\n",n_found_points);
	/*
	//normal corner:
	int suppression_distance_squared;
	suppression_distance_squared = 3 * 3;
	error = findCorners(img, MAX_POINTS, x, y, suppression_distance_squared, &n_found_points, mark_points,imgWidth,imgHeight);
	*/

	if(error == 0)
	{
		error = opticFlowLK(frame, old_img, x, y, n_found_points, imgWidth, imgHeight, new_x, new_y, status, 5, MAX_POINTS);

		//printf("error optic flow = %d\n",error);

		//calculate roll and pitch diff:
		float diff_roll = (float)(current_roll- old_roll)/36.0; // 72 factor is to convert to degrees
		float diff_pitch = (float)(current_pitch- old_pitch)/36.0;

		//calculate mean altitude between the to samples:
		int mean_alt;
		if (current_alt>old_alt)
			mean_alt = (current_alt-old_alt)/2 + old_alt;
		else
			mean_alt = (old_alt-current_alt)/2 + current_alt;


		//remember the frame and meta info
		memcpy(old_img,frame,imgHeight*imgWidth*2);
		old_pitch = current_pitch;
		old_roll = current_roll;
		old_alt = current_alt;

		if(error == 0)
		{
			showFlow(frame, x, y, status, n_found_points, new_x, new_y, imgWidth, imgHeight);

			int tot_x=0;
			int tot_y=0;
			for (int i=0; i<n_found_points;i++) {
				tot_x = tot_x+(new_x[i]-x[i]);
				tot_y = tot_y+(new_y[i]-y[i]);
			}

			//convert pixels/frame to degrees/frame
			float scalef = 64.0/400.0; //64 is vertical camera diagonal view angle (sqrt(320²+240²)=400)
			float opt_angle_x = tot_x*scalef; //= (tot_x/imgWidth) * (scalef*imgWidth); //->degrees/frame
			float opt_angle_y = tot_y*scalef;

			if (abs(opt_angle_x-opt_angle_x_prev)> 3.0) {
				opt_angle_x = opt_angle_x_prev;
			} else	{
				opt_angle_x_prev = opt_angle_x;
			}

			if (abs(opt_angle_y-opt_angle_y_prev)> 3.0) {
				opt_angle_y = opt_angle_y_prev;
			} else	{
				opt_angle_y_prev = opt_angle_y;
			}

			//g_print("Opt_angle x: %f, diff_roll: %d; result: %f. Opt_angle_y: %f, diff_pitch: %d; result: %f. Height: %d\n",opt_angle_x,diff_roll,opt_angle_x-diff_roll,opt_angle_y,diff_pitch,opt_angle_y-diff_pitch,mean_alt);

			//compensate optic flow for attitude (roll,pitch) change:
			opt_angle_x -=  diff_roll;
			opt_angle_y -= diff_pitch;

			//calculate translation in cm/frame from optical flow in degrees/frame
			float opt_trans_x = (float)tan_zelf(opt_angle_x)/1000.0*(float)mean_alt;
			float opt_trans_y = (float)tan_zelf(opt_angle_y)/1000.0*(float)mean_alt;


			//g_print("%f;%f;%f;%f;%f;%f;%d;%f;%f\n",opt_angle_x+diff_roll,diff_roll,opt_angle_x,opt_angle_y+diff_pitch,diff_pitch,opt_angle_y,mean_alt,opt_trans_x,opt_trans_y);
//			printf("dx = %f, dy = %f, alt = %d, p = %f, q = %f\n", opt_trans_x, opt_trans_y, mean_alt, diff_roll, diff_pitch);
			printf("dx_t = %f, dy_t = %f, dx = %f, dy = %f \n", opt_trans_x, opt_trans_y, opt_angle_x+diff_roll, opt_angle_y+diff_pitch );


		} //else g_print("error1\n");
	} //else g_print("error2\n");

	free(x);
	free(new_x);
	free(y);
	free(new_y);
	free(status);
	free(active);

	// Send to paparazzi
	gst2ppz.ID = 0x0001;
	gst2ppz.counter++; // to keep track of data through ppz communication

/*
  // Verbose
  if (verbose > 0)
  {
    printf("*od*%d*",  gst2ppz.counter); // protocol start for obstacle info
    for(int bin = 0; bin < N_BINS; bin++)
    {
      printf("%d,", gst2ppz.obstacle_bins[bin]);
    }
    if (verbose > 1)
    {
      printf("u");
      for(int bin = 0; bin < N_BINS; bin++)
      {
        printf("%d,", gst2ppz.uncertainty_bins[bin]);
      }
    }
    printf("s\n"); // protocol end
  }
*/

}

