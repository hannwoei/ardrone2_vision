
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
int opt_angle_y_prev;
int opt_angle_x_prev;
int opt_trans_x;
int opt_trans_y;

int x_buf[24];
int y_buf[24];
int diff_roll_buf[24];
int diff_pitch_buf[24];

int opt_trans_x_buf[32];
int opt_trans_y_buf[32];


unsigned int buf_point;
unsigned int buf_imu_point;
unsigned int buf_opt_trans_point;

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

void my_plugin_run(unsigned char *frame, float FPS)
{
	int MAX_POINTS, error_corner, error_opticflow;
	int n_found_points,mark_points;
	int *x, *y, *new_x, *new_y, *status, *dx, *dy;
	mark_points = 0;
	MAX_POINTS = 25;

	//save most recent values of attitude for the currently available frame
	int current_pitch = ppz2gst.pitch;
	int current_roll = ppz2gst.roll;
	int current_alt = ppz2gst.alt;

	x = (int *) calloc(MAX_POINTS,sizeof(int));
	new_x = (int *) calloc(MAX_POINTS,sizeof(int));
	y = (int *) calloc(MAX_POINTS,sizeof(int));
	new_y = (int *) calloc(MAX_POINTS,sizeof(int));
	status = (int *) calloc(MAX_POINTS,sizeof(int));
	dx = (int *) calloc(MAX_POINTS,sizeof(int));
	dy = (int *) calloc(MAX_POINTS,sizeof(int));

	float *divergence;
	divergence = (float *) calloc(1,sizeof(float));

	//active corner:
	int *active;
	active =(int *) calloc(MAX_POINTS,sizeof(int));
	int GRID_ROWS = 5;
	int ONLY_STOPPED = 0;
	error_corner = findActiveCorners(frame, GRID_ROWS, ONLY_STOPPED, x, y, active, &n_found_points, mark_points,imgWidth,imgHeight);

	//printf("num_points = %d\n",n_found_points);
/*
	//normal corner:
	int suppression_distance_squared;
	suppression_distance_squared = 3 * 3;
	error_corner = findCorners(frame, MAX_POINTS, x, y, suppression_distance_squared, &n_found_points, mark_points,imgWidth,imgHeight);
*/
	if(error_corner == 0)
	{
		error_opticflow = opticFlowLK(frame, old_img, x, y, n_found_points, imgWidth, imgHeight, new_x, new_y, status, 5, MAX_POINTS);

		//printf("error optic flow = %d\n",error_opticflow);

		//calculate roll and pitch diff:
		int diff_roll = (current_roll- old_roll)*1024;
		int diff_pitch = (current_pitch- old_pitch)*1024;

		//remember the last 6 values of the imu values, because of the better subtraction, due to the delay from the moving average on the opitcal flow
		diff_roll_buf[buf_imu_point] = diff_roll;
		diff_pitch_buf[buf_imu_point] = diff_pitch;
		buf_imu_point = (buf_imu_point+1) %24;

		//use delayed values for IMU, to compensate for moving average. Phase delay of moving average is about 50% of window size.
		diff_roll = diff_roll_buf[(buf_imu_point+12)%24];
		diff_pitch = diff_pitch_buf[(buf_imu_point+12)%24];

		//calculate mean altitude between the to samples:
		int mean_alt;
		if (current_alt>old_alt)
			mean_alt = (current_alt-old_alt)/2 + old_alt;
		else
			mean_alt = (old_alt-current_alt)/2 + current_alt;

		//remember the frame and meta info
		memcpy(old_img,frame,imgHeight*imgWidth*2); //physical size: each four bytes is two pixels
		old_pitch = current_pitch;
		old_roll = current_roll;
		old_alt = current_alt;

		if(error_opticflow == 0)
		{
			//showFlow(frame, x, y, status, n_found_points, new_x, new_y, imgWidth, imgHeight);

			int tot_x=0;
			int tot_y=0;
			int total_weight = 0;
			//int n_used_points = 0;
			int weight_stopped = 2;
			int weight_active = 1;
			for (int i=0; i<n_found_points;i++) {
				if(status[i] == 1)
				{
					//n_used_points++;
					if (active[i]) // not really seems to have much effect
					{
                        dx[i] = new_x[i]-x[i];
                        dy[i] = new_y[i]-y[i];
						tot_x = tot_x+weight_active*(new_x[i]-x[i]);
						tot_y = tot_y+weight_active*(new_y[i]-y[i]);
						total_weight += weight_active;
					}
					else
					{
                        dx[i] = new_x[i]-x[i];
                        dy[i] = new_y[i]-y[i];
						tot_x = tot_x+weight_stopped*(new_x[i]-x[i]);
						tot_y = tot_y+weight_stopped*(new_y[i]-y[i]);
						total_weight += weight_stopped;
					}
				}
			}

			//apply a moving average of 24
			if (total_weight) {

				//magical scaling needed in order to calibrate opt flow angles to imu angles
				int scalex = 1024; //1024*(1/0.75)
				int scaley = 1024; //1024*(1/0.76)

				x_buf[buf_point] = (tot_x*scalex)/total_weight;
				y_buf[buf_point] = (tot_y*scaley)/total_weight;
				buf_point = (buf_point+1) %24;
			}
			int x_avg = 0;
			int y_avg = 0;
			for (int i=0;i<24;i++) {
				x_avg+=x_buf[i];
				y_avg+=y_buf[i];
			}

			//raw optic flow (for telemetry purpose)
			int opt_angle_x_raw = x_avg;
			int opt_angle_y_raw = y_avg;

			//compensate optic flow for attitude (roll,pitch) change:
			x_avg -=  diff_roll;
			y_avg -= diff_pitch;

			//calculate translation in cm/frame from optical flow in degrees/frame

			opt_trans_x = tan_zelf(x_avg/1024)*mean_alt/1000;
			opt_trans_y = tan_zelf(y_avg/1024)*mean_alt/1000;

			if (abs(opt_trans_y) < 1000 && abs(opt_trans_x) < 1000) {
				opt_trans_x_buf[buf_opt_trans_point] = opt_trans_x;
				opt_trans_y_buf[buf_opt_trans_point] = opt_trans_y;
				buf_opt_trans_point = (buf_opt_trans_point + 1) % 32;
			}
			for (int i=0;i<32;i++) {
				opt_trans_x+=opt_trans_x_buf[i];
				opt_trans_y+=opt_trans_y_buf[i];
			}
			opt_trans_x = opt_trans_x /32;
			opt_trans_y = opt_trans_y /32;

			// linear fit of the optic flow field
			float error_threshold = 10;
			int n_iterations = 40; // 20
			int n_samples = (n_found_points < 5) ? n_found_points : 5;
			float mean_tti, median_tti, d_heading, d_pitch;

			// minimum = 3
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

/*
			int i;
			for(i=0;i<3;i++)
			{
				printf("pu%d = %f, pv%d = %f\t",i,pu[i],i,pv[i]);
			}
			printf("\n");
*/

			extractInformationFromLinearFlowField(divergence, &mean_tti, &median_tti, &d_heading, &d_pitch, pu, pv, imgWidth, imgHeight, FPS);
//			printf("div = %f\n", divergence);
//			printf("dx = %f, dy = %f, alt = %d, p = %f, q = %f\n", opt_trans_x, opt_trans_y, mean_alt, diff_roll, diff_pitch);
//			printf("dx_t = %f, dy_t = %f, dx = %f, dy = %f \n", opt_trans_x, opt_trans_y, opt_angle_x+diff_roll, opt_angle_y+diff_pitch );
			// divergence flow message
			DOWNLINK_SEND_OPTIC_FLOW(DefaultChannel, DefaultDevice, &FPS, &opt_angle_x_raw, &opt_angle_y_raw, &opt_trans_x, &opt_trans_y, &diff_roll, &diff_pitch, &mean_alt, &n_found_points, divergence, &mean_tti, &median_tti, &d_heading, &d_pitch, &pu[2], &pv[2], &divergence_error);

			// optic flow message
			//float empty;
			//empty = 0;
			//DOWNLINK_SEND_OPTIC_FLOW(DefaultChannel, DefaultDevice, &FPS, &opt_angle_x_raw, &opt_angle_y_raw, &opt_trans_x, &opt_trans_y, &diff_roll, &diff_pitch, &mean_alt, &n_found_points, &empty, &empty, &empty, &empty, &empty);

		}
	}
	DOWNLINK_SEND_OF_ERROR(DefaultChannel, DefaultDevice, &error_corner, &error_opticflow);

	free(x);
	free(new_x);
	free(y);
	free(new_y);
	free(status);
	free(active);
	free(dx);
	free(dy);
	free(divergence);
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

