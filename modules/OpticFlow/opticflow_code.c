
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

// Downlink
#include "messages.h"
#include "subsystems/datalink/downlink.h"

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
float opt_trans_x;
float opt_trans_y;

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
	opt_trans_x = 0;
	opt_trans_y = 0;

	gst2ppz.counter = 0;

}

void my_plugin_run(unsigned char *frame)
{
	int MAX_POINTS, error;
	int n_found_points,mark_points;
	int *x, *y, *new_x, *new_y, *status, *dx, *dy;
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
	dx = (int *) calloc(40,sizeof(int));
	dy = (int *) calloc(40,sizeof(int));

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
		float diff_roll = (float)(current_roll- old_roll)/ 71.488686161687739470794373877294f; // 72 factor is to convert to degrees
		float diff_pitch = (float)(current_pitch- old_pitch)/ 71.488686161687739470794373877294f; //previously it is divided by 36

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
			//showFlow(frame, x, y, status, n_found_points, new_x, new_y, imgWidth, imgHeight);

			int tot_x=0;
			int tot_y=0;
			for (int i=0; i<n_found_points;i++) {
				dx[i] = new_x[i]-x[i];
				dy[i] = new_y[i]-y[i];
				tot_x = tot_x+(dx[i]);
				tot_y = tot_y+(dy[i]);
			}

			//convert pixels/frame to degrees/frame
			float scalef = 64.0/400.0; //64 is vertical camera diagonal view angle (sqrt(320²+240²)=400)
			float opt_angle_x = tot_x*scalef; //= (tot_x/imgWidth) * (scalef*imgWidth); //->degrees/frame
			float opt_angle_y = tot_y*scalef;

			if (abs(opt_angle_x-opt_angle_x_prev)> 10.0) {
				opt_angle_x = opt_angle_x_prev;
			} else	{
				opt_angle_x_prev = opt_angle_x;
			}

			if (abs(opt_angle_y-opt_angle_y_prev)> 10.0) {
				opt_angle_y = opt_angle_y_prev;
			} else	{
				opt_angle_y_prev = opt_angle_y;
			}

			//g_print("Opt_angle x: %f, diff_roll: %d; result: %f. Opt_angle_y: %f, diff_pitch: %d; result: %f. Height: %d\n",opt_angle_x,diff_roll,opt_angle_x-diff_roll,opt_angle_y,diff_pitch,opt_angle_y-diff_pitch,mean_alt);

			//raw optic flow (for telemetry purpose)
			float opt_angle_x_raw = opt_angle_x;
			float opt_angle_y_raw = opt_angle_y;

			//compensate optic flow for attitude (roll,pitch) change:
			opt_angle_x -=  diff_roll;
			opt_angle_y -= diff_pitch;

			//calculate translation in cm/frame from optical flow in degrees/frame
			opt_trans_x = (float)tan_zelf(opt_angle_x)/1000.0*(float)mean_alt;
			opt_trans_y = (float)tan_zelf(opt_angle_y)/1000.0*(float)mean_alt;

			// linear fit of the optic flow field
			float error_threshold = 10;
			int n_iterations = 10;  //20
			int n_samples = (n_found_points < 5) ? n_found_points : 5;
			float count = n_found_points;
			float divergence, mean_tti, median_tti, d_heading, d_pitch;
			// minimum = 3
			//TODO: fps
			float FPS = 1;
			//TODO:
  			if(n_samples < 3)
			{
				// set dummy values for tti, etc.
				mean_tti = 1000.0f / FPS;
				median_tti = mean_tti;
				d_heading = 0;
				d_pitch = 0;
//				POE_x = (float)(in->width/2);
//				POE_y = (float)(in->height/2);
				return;
			}
			float pu[3], pv[3];

			float divergence_error;
			float min_error_u, min_error_v;
			fitLinearFlowField(pu, pv, &divergence_error, x, y, dx, dy, n_found_points, n_samples, &min_error_u, &min_error_v, n_iterations, error_threshold);

			extractInformationFromLinearFlowField(&divergence, &mean_tti, &median_tti, &d_heading, &d_pitch, pu, pv, imgWidth, imgHeight, FPS);
//			printf("div = %f\n", divergence);
//			g_print("%f;%f;%f;%f;%f;%f;%d;%f;%f\n",opt_angle_x+diff_roll,diff_roll,opt_angle_x,opt_angle_y+diff_pitch,diff_pitch,opt_angle_y,mean_alt,opt_trans_x,opt_trans_y);
//			printf("dx = %f, dy = %f, alt = %d, p = %f, q = %f\n", opt_trans_x, opt_trans_y, mean_alt, diff_roll, diff_pitch);
//			printf("dx_t = %f, dy_t = %f, dx = %f, dy = %f \n", opt_trans_x, opt_trans_y, opt_angle_x+diff_roll, opt_angle_y+diff_pitch );
			DOWNLINK_SEND_OPTIC_FLOW(DefaultChannel, DefaultDevice, &opt_angle_x_raw, &opt_angle_y_raw, &opt_trans_x, &opt_trans_y, &diff_roll, &diff_pitch, &mean_alt, &n_found_points, &divergence, &mean_tti, &median_tti, &d_heading, &d_pitch);

		} //else g_print("error1\n");
	} //else g_print("error2\n");

	free(x);
	free(new_x);
	free(y);
	free(new_y);
	free(status);
	free(active);
	free(dx);
	free(dy);

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

