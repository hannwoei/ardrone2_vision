
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Computer Vision
#include "opticflow/optic_flow_gdc.h"
#include "trig.h"
#include "opticflow/fastRosten.h"
#include "opticflow_module.h"

// Own Header
#include "opticflow_code_new.h"

// Paparazzi Data
#include "state.h"
#include "subsystems/ins/ins_int.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"
#include "subsystems/imu.h"

// Downlink
#include "messages.h"
#include "subsystems/datalink/downlink.h"

// Timer
#include <sys/time.h>

//#include "paparazzi.h"

// Settable by pluging
unsigned int imgWidth, imgHeight;

// Division
#define nImgW 3
#define nImgH 3
#define nRegion nImgW*nImgH

// Image frame
unsigned char *prev_frame, *gray_frame;
int old_img_init;

// Corner Detection &  Optic Flow Computation
int dx_buf[24];
int dy_buf[24];
unsigned int buf_point_new = 0;
int *opt_x, *opt_y, *new_opt_x, *new_opt_y, *opt_status, *opt_dx, *opt_dy, *n_inlier_minu, *n_inlier_minv, tot_dx, tot_dy, dx_avg, dy_avg;
float divergence, new_divergence;
int error_corner, error_opticflow;
int count_new = 0;
int max_count_new = nRegion;
int flow_point_size_new = 0;
#define MAX_COUNT nRegion	// Maximum number of flow points
flowPoint flow_points[MAX_COUNT];
detectedPoint detected_points0[MAX_COUNT];
detectedPoint detected_points1[MAX_COUNT];
detectedPoint swap_points[MAX_COUNT];

// Flow Derotation
/*
 * 1 deg = (2*arctan(0.5*imW/f))/FOV = xDerotate = yDerotate
 * FOVx = 50 deg, FOVy = 38 deg, imgW = 320, imgH = 240, Fx = 343.1211, Fy = 348.5053
 * TODO: validate data as F is computed from FOV so xDerotate = yDerotate = 1
 */

float curr_pitch, curr_roll, curr_yaw, prev_pitch, prev_roll, prev_yaw;
float cam_h, prev_cam_h, diff_roll, diff_pitch, opt_trans_dx, opt_trans_dy;

// Lateral Velocity Computation
#define Fx_ARdrone 343.1211
#define Fy_ARdrone 348.5053
float Velx, Vely, Velz, Velz_buf[12];
unsigned int Velz_buf_point_new = 0;

int DIV_FILTER_new = 0;

// Kalman fusion: Optic flow and Accelerometers


struct FloatVect3 accel_update;
struct FloatRates rate_update;

// Called by plugin
void opticflow_new_init(void)
{

	// Initialize variables
	gray_frame = (unsigned char *) calloc(imgWidth*imgHeight,sizeof(unsigned char));
	prev_frame = (unsigned char *) calloc(imgWidth*imgHeight*2,sizeof(unsigned char));
	old_img_init = 1;

	tot_dx=0;
	tot_dy=0;
	dx_avg = 0;
	dy_avg = 0;

	diff_roll = 0.0;
	diff_pitch = 0.0;
	cam_h = 0.0;
	prev_cam_h = 0.0;
	prev_pitch = 0.0;
	prev_roll = 0.0;
	prev_yaw = 0.0;
	curr_pitch = 0.0;
	curr_roll = 0.0;
	curr_yaw = 0.0;
	opt_trans_dx = 0.0;
	opt_trans_dy = 0.0;

	Velx = 0.0;
	Vely = 0.0;
	Velz = 0.0;

	opt_dx = 0;
	opt_dy = 0;

	opt_x = (int *) calloc(MAX_COUNT,sizeof(int));
	new_opt_x = (int *) calloc(MAX_COUNT,sizeof(int));
	opt_y = (int *) calloc(MAX_COUNT,sizeof(int));
	new_opt_y = (int *) calloc(MAX_COUNT,sizeof(int));
	opt_status = (int *) calloc(MAX_COUNT,sizeof(int));
	opt_dx = (int *) calloc(MAX_COUNT,sizeof(int));
	opt_dy = (int *) calloc(MAX_COUNT,sizeof(int));
	n_inlier_minu = (int *)calloc(1,sizeof(int));
	n_inlier_minv = (int *)calloc(1,sizeof(int));

	divergence = 0.0;
	new_divergence = 0.0;
}

void opticflow_new_run(unsigned char *frame)
{
	if(old_img_init == 1)
	{
		memcpy(prev_frame,frame,imgHeight*imgWidth*2);
		old_img_init = 0;
	}

    findDistributedPoints(gray_frame, frame, imgWidth, imgHeight, &count_new, max_count_new, MAX_COUNT, flow_points, &flow_point_size_new, detected_points0, opt_status);


    if(count_new)
    {
    	trackDistributedPoints(frame, prev_frame, imgWidth, imgHeight, &count_new, max_count_new, MAX_COUNT, flow_points, &flow_point_size_new, detected_points0, opt_x, opt_y, new_opt_x, new_opt_y, opt_dx, opt_dy, opt_status);

		tot_dx=0;
		tot_dy=0;
		dx_avg = 0;
		dy_avg = 0;

		//magical scaling needed in order to calibrate opt flow angles to imu angles
		int scalex = 1024; //1024*(1/0.75) //default 1024
		int scaley = 1024; //1024*(1/0.76) //default 1024

		for (int i=0; i<count_new;i++)
		{
			opt_dx[i] = flow_points[i].dx;
			opt_dy[i] = flow_points[i].dy;

			tot_dx = tot_dx + opt_dx[i];
			tot_dy = tot_dy + opt_dy[i];
		}
		// using moving average to filter out the noise
		if(count_new)
		{
			dx_buf[buf_point_new] = (tot_dx*scalex)/count_new;
			dy_buf[buf_point_new] = (tot_dy*scaley)/count_new;
			buf_point_new = (buf_point_new+1) %5;
		}

		for (int i=0;i<5;i++) {
			dx_avg+=dx_buf[i];
			dy_avg+=dy_buf[i];
		}

		// Flow Derotation


		curr_pitch = stateGetNedToBodyEulers_i()->theta*0.0139882;
		curr_roll = stateGetNedToBodyEulers_i()->phi*0.0139882;
		curr_yaw = stateGetNedToBodyEulers_i()->psi*0.0139882;

		diff_pitch = (curr_pitch - prev_pitch)/FPS*scaley*Fy_ARdrone*240/38.4;
		diff_roll = (curr_roll - prev_roll)/FPS*scalex*Fx_ARdrone*320/51.2;

		prev_pitch = curr_pitch;
		prev_roll = curr_roll;
		prev_yaw = curr_yaw;

		opt_trans_dx = dx_avg - diff_roll;
		opt_trans_dy = dy_avg - diff_pitch;


		// Velocity Computation
#if USE_SONAR
		cam_h = ins_impl.sonar_z;
#else
		cam_h = 1;
		prev_cam_h = 1;
#endif
		Velz = (cam_h-prev_cam_h)*FPS;
		prev_cam_h = cam_h;

		int velz_win = 6;
		Velz_buf[Velz_buf_point_new] = Velz;
		Velz_buf_point_new = (Velz_buf_point_new+1) %velz_win;

		for (int i=0;i<velz_win;i++) {
			Velz+=Velz_buf[i]/velz_win;
		}

		if(count_new)
		{
			Velx = opt_trans_dx*cam_h/Fx_ARdrone;
			Vely = opt_trans_dy*cam_h/Fy_ARdrone;
		}
		else
		{
			Velx = 0.0;
			Vely = 0.0;
		}


		// Kalman fusion: Optic flow and Accelerometers
	    ACCELS_FLOAT_OF_BFP(accel_update,imu.accel);
	    rate_update.p = RATE_FLOAT_OF_BFP(stateGetBodyRates_i()->p);
	    rate_update.q = RATE_FLOAT_OF_BFP(stateGetBodyRates_i()->q);
	    rate_update.r = RATE_FLOAT_OF_BFP(stateGetBodyRates_i()->r);


		DOWNLINK_SEND_EKF_VISION_ACCEL(DefaultChannel, DefaultDevice, &accel_update.x, &accel_update.y, &accel_update.z, &rate_update.p, &rate_update.q, &rate_update.r, &curr_roll, &curr_pitch, &curr_yaw, &opt_dx, &opt_dy, &opt_trans_dx, &opt_trans_dy, &Velz, &cam_h, &FPS);

		//tele purpose

		float mean_tti, median_tti, d_heading, d_pitch, pu[3], pv[3], divergence_error;

		int USE_FITTING = 1;

		if(USE_FITTING == 1)
		{
			analyseTTI(&divergence, &mean_tti, &median_tti, &d_heading, &d_pitch, &divergence_error, opt_x, opt_y, opt_dx, opt_dy, n_inlier_minu, n_inlier_minv, count_new, imgWidth, imgHeight, &DIV_FILTER_new);
		}

		// new method for computing divergence
		// lineDivergence(&new_divergence, opt_x, opt_y, new_opt_x, new_opt_y, count_new);


		// *********************************************
		// (5) housekeeping to prepare for the next call
		// *********************************************

		memcpy(prev_frame,frame,imgHeight*imgWidth*2);

		showFlow(frame, opt_x, opt_y, opt_status, count_new, new_opt_x, new_opt_y, imgWidth, imgHeight);

		int i;
		for (i=0;i<count_new;i++)
		{
			swap_points[i] = detected_points0[i];
			detected_points0[i] = detected_points1[i];
			detected_points1[i] = swap_points[i];
		}

		DOWNLINK_SEND_OPTIC_FLOW(DefaultChannel, DefaultDevice, &FPS, &opt_dx, &opt_dy, &opt_trans_dx, &opt_trans_dy, &Velx, &Vely, &diff_roll, &diff_pitch, &cam_h, &count_new, &flow_point_size_new, &divergence, &new_divergence, &mean_tti, &median_tti, &d_heading, &d_pitch, &pu[2], &pv[2], &divergence_error, n_inlier_minu, n_inlier_minv, &DIV_FILTER_new);
    }

	//DOWNLINK_SEND_OF_ERROR(DefaultChannel, DefaultDevice, &error_corner, &error_opticflow);

}

