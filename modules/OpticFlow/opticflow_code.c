
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Computer Vision
#include "opticflow/optic_flow_gdc.h"
#include "trig.h"
#include "opticflow/fastRosten.h"

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
unsigned char * old_img, *gray_img;
int old_pitch, old_roll, old_alt;
int opt_angle_y_prev;
int opt_angle_x_prev;
int opt_trans_x;
int opt_trans_y;
float div_flow;

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

	gray_img = (unsigned char *) calloc(imgWidth*imgHeight,sizeof(unsigned char));
	old_img = (unsigned char *) calloc(imgWidth*imgHeight*2,sizeof(unsigned char));

	old_pitch = 0;
	old_roll = 0;
	old_alt = 0;
	opt_angle_y_prev = 0;
	opt_angle_x_prev = 0;
	opt_trans_x = 0;
	opt_trans_y = 0;
	div_flow = 0.0f;

	gst2ppz.counter = 0;
}

void my_plugin_run(unsigned char *frame, float FPS)
{

	// select which corner detector to be used
	unsigned int CORNER_METHOD = 3; //1. Guido ACT 2. Harris 3. Rosten FAST

	int MAX_POINTS, error_corner, error_opticflow;
	int n_found_points,mark_points;
	int *x, *y, *new_x, *new_y, *status, *dx, *dy, *xc, *yc, *n_inlier_minu, *n_inlier_minv;
	int *active;
	mark_points = 0;
	MAX_POINTS = 40;

	//save most recent values of attitude for the currently available frame
	int current_pitch = ppz2gst.pitch;
	int current_roll = ppz2gst.roll;
	int current_alt = ppz2gst.alt;

	x = (int *) calloc(MAX_POINTS,sizeof(int));
	xc = (int *) calloc(MAX_POINTS,sizeof(int));
	new_x = (int *) calloc(MAX_POINTS,sizeof(int));
	y = (int *) calloc(MAX_POINTS,sizeof(int));
	yc = (int *) calloc(MAX_POINTS,sizeof(int));
	new_y = (int *) calloc(MAX_POINTS,sizeof(int));
	status = (int *) calloc(MAX_POINTS,sizeof(int));
	dx = (int *) calloc(MAX_POINTS,sizeof(int));
	dy = (int *) calloc(MAX_POINTS,sizeof(int));
	n_inlier_minu = (int *)calloc(1,sizeof(int));
	n_inlier_minv = (int *)calloc(1,sizeof(int));

	float *divergence;
	divergence = (float *) calloc(1,sizeof(float));

	if (CORNER_METHOD == 1)
	{
		//active corner:
//		int *active;
		active =(int *) calloc(MAX_POINTS,sizeof(int));
		int GRID_ROWS = 5;
		int ONLY_STOPPED = 0;
		error_corner = findActiveCorners(frame, GRID_ROWS, ONLY_STOPPED, x, y, active, &n_found_points, mark_points,imgWidth,imgHeight);
	}
	else if(CORNER_METHOD == 2)
	{
		//normal corner:
		int suppression_distance_squared;
		suppression_distance_squared = 3 * 3;
		error_corner = findCorners(frame, MAX_POINTS, x, y, suppression_distance_squared, &n_found_points, mark_points,imgWidth,imgHeight);
	}
	else
	{

		// FAST corner:
		int fast_threshold = 20;

		xyFAST* pnts_fast;
		CvtYUYV2Gray(gray_img, frame, imgWidth, imgHeight);
		pnts_fast = fast9_detect((const byte*)gray_img, imgWidth, imgHeight, imgWidth, fast_threshold, &n_found_points);
printf("num_pts = %d\n",n_found_points);
		// transform the points to the format we need (is also done in the other corner finders
		n_found_points = (n_found_points > MAX_POINTS) ? MAX_POINTS : n_found_points;
		for(int i = 0; i < n_found_points; i++)
		{
			x[i] = pnts_fast[i].x;
			y[i] = pnts_fast[i].y;
		}

		error_corner = 0;
		free(pnts_fast);
	}

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
			// showFlow cannot be used to show flow in a downsized image unless it is corrected with downsized factor
//			showFlow(frame, x, y, status, n_found_points, new_x, new_y, imgWidth, imgHeight);

			int tot_x=0;
			int tot_y=0;
			int x_avg = 0;
			int y_avg = 0;
			int n_used_points = 0;

			//magical scaling needed in order to calibrate opt flow angles to imu angles
			int scalex = 1024; //1024*(1/0.75)
			int scaley = 1024; //1024*(1/0.76)

			if(CORNER_METHOD == 1)
			{
				int total_weight = 0;
				int weight_stopped = 2;
				int weight_active = 1;
				int stopped_only = 0;

				for (int i=0; i<n_found_points;i++) {
					if(status[i] == 1)
					{
						if (active[i]) // not really seems to have much effect
						{
							tot_x = tot_x+weight_active*(new_x[i]-x[i]);
							tot_y = tot_y+weight_active*(new_y[i]-y[i]);
							total_weight += weight_active;
							if(!stopped_only)
							{
								xc[n_used_points] = x[i]-imgWidth/2;
								yc[n_used_points] = y[i]-imgHeight/2;
								dx[n_used_points] = (new_x[i]-x[i]);
								dy[n_used_points] = (new_y[i]-y[i]);
								n_used_points++;
							}
						}
						else
						{
							tot_x = tot_x+weight_stopped*(new_x[i]-x[i]);
							tot_y = tot_y+weight_stopped*(new_y[i]-y[i]);
							total_weight += weight_stopped;

							// only use the stopped agents
							xc[n_used_points] = x[i]-imgWidth/2;
							yc[n_used_points] = y[i]-imgHeight/2;
							dx[n_used_points] = (new_x[i]-x[i]);
							dy[n_used_points] = (new_y[i]-y[i]);
							n_used_points++;
						}
					}
				}

				//apply a moving average of 24
				if (total_weight) {

					x_buf[buf_point] = (tot_x*scalex)/total_weight;
					y_buf[buf_point] = (tot_y*scaley)/total_weight;
					buf_point = (buf_point+1) %24;
				}

				for (int i=0;i<24;i++) {
					x_avg+=x_buf[i];
					y_avg+=y_buf[i];
				}
			}
			else
			{
				for (int i=0; i<n_found_points;i++)
				{
					xc[i] = x[i]-imgWidth/2;
					yc[i] = y[i]-imgHeight/2;
					dx[i] = (new_x[i]-x[i]);
					dy[i] = (new_y[i]-y[i]);
					tot_x = tot_x + dx[i];
					tot_y = tot_y + dy[i];
					n_used_points++;
				}
				if(n_used_points)
				{
					x_buf[buf_point] = (tot_x*scalex)/n_found_points;
					y_buf[buf_point] = (tot_y*scaley)/n_found_points;
					buf_point = (buf_point+1) %5;
				}

				for (int i=0;i<5;i++) {
					x_avg+=x_buf[i];
					y_avg+=y_buf[i];
				}
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
			float error_threshold = 10; // 10
			int n_iterations = 20; // 40
			int count;
			if (CORNER_METHOD == 1) count = n_used_points;
			else count = n_found_points;
			int n_samples = (count < 5) ? count : 5;
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
			fitLinearFlowField(pu, pv, &divergence_error, x, y, dx, dy, count, n_samples, &min_error_u, &min_error_v, n_iterations, error_threshold, n_inlier_minu, n_inlier_minv);

/*
			int i;
			for(i=0;i<3;i++)
			{
				printf("pu%d = %f, pv%d = %f\t",i,pu[i],i,pv[i]);
			}
			printf("\n");
*/

			extractInformationFromLinearFlowField(divergence, &mean_tti, &median_tti, &d_heading, &d_pitch, pu, pv, imgWidth, imgHeight, FPS);
			div_flow = *divergence;
//			printf("div = %f\n", divergence);
//			printf("dx = %f, dy = %f, alt = %d, p = %f, q = %f\n", opt_trans_x, opt_trans_y, mean_alt, diff_roll, diff_pitch);
//			printf("dx_t = %f, dy_t = %f, dx = %f, dy = %f \n", opt_trans_x, opt_trans_y, opt_angle_x+diff_roll, opt_angle_y+diff_pitch );
			// divergence flow message
			DOWNLINK_SEND_OPTIC_FLOW(DefaultChannel, DefaultDevice, &FPS, &opt_angle_x_raw, &opt_angle_y_raw, &opt_trans_x, &opt_trans_y, &diff_roll, &diff_pitch, &mean_alt, &n_found_points, &count, &div_flow, &mean_tti, &median_tti, &d_heading, &d_pitch, &pu[2], &pv[2], &divergence_error, n_inlier_minu, n_inlier_minv);
			DOWNLINK_SEND_OF_FITTING(DefaultChannel, DefaultDevice, &pu[0], &pu[1], &pu[2], &pv[0], &pv[1], &pv[2]);
			// optic flow message
			//float empty;
			//empty = 0;
			//DOWNLINK_SEND_OPTIC_FLOW(DefaultChannel, DefaultDevice, &FPS, &opt_angle_x_raw, &opt_angle_y_raw, &opt_trans_x, &opt_trans_y, &diff_roll, &diff_pitch, &mean_alt, &n_found_points, &empty, &empty, &empty, &empty, &empty);

		}
	}
	DOWNLINK_SEND_OF_ERROR(DefaultChannel, DefaultDevice, &error_corner, &error_opticflow);


	if(CORNER_METHOD == 1) free(active);
	free(n_inlier_minu);
	free(n_inlier_minv);
	free(x);
	free(xc);
	free(new_x);
	free(y);
	free(yc);
	free(new_y);
	free(status);
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

