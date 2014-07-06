
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <semaphore.h>

// Computer Vision
#include "opticflow/optic_flow_gdc.h"
#include "trig.h"
#include "opticflow/fastRosten.h"
#include "opticflow_module.h"

// Own Header
#include "opticflow_code.h"

// Paparazzi Data
#include "state.h"
#include "subsystems/ins/ins_int.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"
#include "subsystems/imu.h"

// Communication
#include "video_message_structs.h"
struct gst2ppz_message_struct gst2ppz;
struct ppz2gst_message_struct ppz2gst;

// Downlink
#include "messages.h"
#include "subsystems/datalink/downlink.h"

// Timer
#include <sys/time.h>

//#include "paparazzi.h"

// Settable by pluging
unsigned int imgWidth, imgHeight;
unsigned int verbose = 0;

// Local variables
//static unsigned char * img_uncertainty;
unsigned char *prev_frame, *gray_frame, *prev_gray_frame;

int old_img_init;

int opt_angle_x_raw;
int opt_angle_y_raw;

int x_buf[24];
int y_buf[24];

int opt_trans_x_buf[32];
int opt_trans_y_buf[32];

unsigned int buf_point = 0;
unsigned int buf_imu_point;
unsigned int buf_opt_trans_point;

int *x, *y, *new_x, *new_y, *status, *dx, *dy, *dx_scaled, *dy_scaled, *n_inlier_minu, *n_inlier_minv, *active;
float divergence, new_divergence;
int error_corner, error_opticflow, mark_points;

// Corner Detection
int count = 0;
int max_count = 25;
int flow_point_size = 0;
#define MAX_COUNT 150	// Maximum number of flow points
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
#define xDerotate 1.000000019
#define yDerotate 1.000000014
unsigned int att_buf_point = 0;
float curr_pitch, curr_roll, curr_yaw, prev_pitch, prev_roll, prev_yaw;
float cam_h, prev_cam_h, diff_roll, diff_pitch, diff_roll_buf[12], diff_pitch_buf[12], opt_trans_x, opt_trans_y;

// Lateral Velocity Computation
#define Fx_ARdrone 343.1211
#define Fy_ARdrone 348.5053
float Velx, Vely, Velz, Velz_buf[12];
unsigned int Velz_buf_point = 0;

int DIV_FILTER = 0;

// Kalman fusion: Optic flow and Accelerometers


struct FloatVect3 accel_update;
struct FloatRates rate_update;

// Called by plugin
void my_plugin_init(void)
{

	// Init variables
	ppz2gst.pitch = 0;
	ppz2gst.roll = 0;
	gray_frame = (unsigned char *) calloc(imgWidth*imgHeight,sizeof(unsigned char));
	prev_gray_frame = (unsigned char *) calloc(imgWidth*imgHeight,sizeof(unsigned char));
	prev_frame = (unsigned char *) calloc(imgWidth*imgHeight*2,sizeof(unsigned char));
	old_img_init = 1;

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
	opt_trans_x = 0.0;
	opt_trans_y = 0.0;

	Velx = 0.0;
	Vely = 0.0;
	Velz = 0.0;

	opt_angle_x_raw = 0;
	opt_angle_y_raw = 0;

	gst2ppz.counter = 0;

	mark_points = 0;

	x = (int *) calloc(MAX_COUNT,sizeof(int));
	new_x = (int *) calloc(MAX_COUNT,sizeof(int));
	y = (int *) calloc(MAX_COUNT,sizeof(int));
	new_y = (int *) calloc(MAX_COUNT,sizeof(int));
	status = (int *) calloc(MAX_COUNT,sizeof(int));
	dx = (int *) calloc(MAX_COUNT,sizeof(int));
	dy = (int *) calloc(MAX_COUNT,sizeof(int));
	dx_scaled = (int *) calloc(MAX_COUNT,sizeof(int));
	dy_scaled = (int *) calloc(MAX_COUNT,sizeof(int));
	n_inlier_minu = (int *)calloc(1,sizeof(int));
	n_inlier_minv = (int *)calloc(1,sizeof(int));

	divergence = 0.0;
	new_divergence = 0.0;
}

void my_plugin_run(unsigned char *frame)
{
	if(old_img_init == 1)
	{
		memcpy(prev_frame,frame,imgHeight*imgWidth*2);
		old_img_init = 0;
	}

	// ***********************************************************************************************************************
	// (1) possibly find new points - keeping possible old ones (normal cv methods / efficient point finding / active corners)
	// ***********************************************************************************************************************

    int ALWAYS_NEW_POINTS = 0;

    if(ALWAYS_NEW_POINTS)
    {
    	// Clear corners
    	memset(flow_points,0,sizeof(flowPoint)*flow_point_size);
    	findPoints(gray_frame, frame, imgWidth, imgHeight, &count, max_count, MAX_COUNT, flow_points, &flow_point_size, detected_points0);
    }
    else
    {
    	int threshold_n_points = 25; //25
    	if(flow_point_size < threshold_n_points)
    	{
        	findPoints(gray_frame, frame, imgWidth, imgHeight, &count, max_count, MAX_COUNT, flow_points, &flow_point_size, detected_points0);
    	}
    }

	// **********************************************************************************************************************
	// (2) track the points to the new image, possibly using external information (TTC, known lateral / rotational movements)
	// **********************************************************************************************************************
    if(count)
    {
    	trackPoints(frame, prev_frame, imgWidth, imgHeight, &count, max_count, MAX_COUNT, flow_points, &flow_point_size, detected_points0, x, y, new_x, new_y, dx, dy, status);

		int tot_x=0;
		int tot_y=0;
		int x_avg = 0;
		int y_avg = 0;

		//magical scaling needed in order to calibrate opt flow angles to imu angles
		int scalex = 1024; //1024*(1/0.75) //default 1024
		int scaley = 1024; //1024*(1/0.76) //default 1024

		for (int i=0; i<count;i++)
		{
			dx[i] = flow_points[i].dx;
			dy[i] = flow_points[i].dy;

			tot_x = tot_x + dx[i];
			tot_y = tot_y + dy[i];
		}
		// using moving average to filter out the noise
		if(count)
		{
			x_buf[buf_point] = (tot_x*scalex)/count;
			y_buf[buf_point] = (tot_y*scaley)/count;
			buf_point = (buf_point+1) %5;
		}

		for (int i=0;i<5;i++) {
			x_avg+=x_buf[i];
			y_avg+=y_buf[i];
		}

		//raw optic flow (for telemetry purpose)
		opt_angle_x_raw = x_avg;
		opt_angle_y_raw = y_avg;

		// Flow Derotation


//		curr_pitch = stateGetNedToBodyEulers_i()->theta*0.0139882;
//		curr_roll = stateGetNedToBodyEulers_i()->phi*0.0139882;
//		curr_yaw = stateGetNedToBodyEulers_i()->psi*0.0139882;
//
//		diff_pitch = (curr_pitch - prev_pitch)/FPS*scaley*Fy_ARdrone*240/38.4;
//		diff_roll = (curr_roll - prev_roll)/FPS*scalex*Fx_ARdrone*320/51.2;
//
//		prev_pitch = curr_pitch;
//		prev_roll = curr_roll;
//		prev_yaw = curr_yaw;
//
//		opt_trans_x = opt_angle_x_raw - diff_roll;
//		opt_trans_y = opt_angle_y_raw - diff_pitch;
//
//
//		// Velocity Computation
//#if USE_SONAR
//		cam_h = ins_impl.sonar_z;
//#else
//		cam_h = 1;
//		prev_cam_h = 1;
//#endif
//		Velz = (cam_h-prev_cam_h)*FPS;
//		prev_cam_h = cam_h;
//
//		int velz_win = 6;
//		Velz_buf[Velz_buf_point] = Velz;
//		Velz_buf_point = (Velz_buf_point+1) %velz_win;
//
//		for (int i=0;i<velz_win;i++) {
//			Velz+=Velz_buf[i]/velz_win;
//		}
//
//		if(count)
//		{
//			Velx = opt_trans_x*cam_h/Fx_ARdrone;
//			Vely = opt_trans_y*cam_h/Fy_ARdrone;
//		}
//		else
//		{
//			Velx = 0.0;
//			Vely = 0.0;
//		}


		// Kalman fusion: Optic flow and Accelerometers
//	    ACCELS_FLOAT_OF_BFP(accel_update,imu.accel);
//	    rate_update.p = RATE_FLOAT_OF_BFP(stateGetBodyRates_i()->p);
//	    rate_update.q = RATE_FLOAT_OF_BFP(stateGetBodyRates_i()->q);
//	    rate_update.r = RATE_FLOAT_OF_BFP(stateGetBodyRates_i()->r);

//	    ACCELS_FLOAT_OF_BFP(accel_update,mean_accel);
//	    RATES_FLOAT_OF_BFP(rate_update,mean_rate);
//		INT_RATES_ZERO(mean_rate);
//		INT32_VECT3_ZERO(mean_accel);
//		count_input = 0;

//		DOWNLINK_SEND_EKF_VISION_ACCEL(DefaultChannel, DefaultDevice, &accel_update.x, &accel_update.y, &accel_update.z, &rate_update.p, &rate_update.q, &rate_update.r, &curr_roll, &curr_pitch, &curr_yaw, &opt_angle_x_raw, &opt_angle_y_raw, &opt_trans_x, &opt_trans_y, &Velz, &cam_h, &FPS);

		//tele purpose

		float mean_tti, median_tti, d_heading, d_pitch, pu[3], pv[3], divergence_error;

		int USE_FITTING = 1;

		if(USE_FITTING == 1)
		{
			analyseTTI(&divergence, &mean_tti, &median_tti, &d_heading, &d_pitch, &divergence_error, x, y, dx, dy, n_inlier_minu, n_inlier_minv, count, imgWidth, imgHeight, &DIV_FILTER);
		}

		// new method for computing divergence
		// lineDivergence(&new_divergence, x, y, new_x, new_y, count);


		// *********************************************
		// (5) housekeeping to prepare for the next call
		// *********************************************

		memcpy(prev_frame,frame,imgHeight*imgWidth*2);

		//showFlow(frame, x, y, status, count, new_x, new_y, imgWidth, imgHeight);

		int i;
		for (i=0;i<count;i++)
		{
			swap_points[i] = detected_points0[i];
			detected_points0[i] = detected_points1[i];
			detected_points1[i] = swap_points[i];
		}

		DOWNLINK_SEND_OPTIC_FLOW(DefaultChannel, DefaultDevice, &FPS, &opt_angle_x_raw, &opt_angle_y_raw, &opt_trans_x, &opt_trans_y, &Velx, &Vely, &diff_roll, &diff_pitch, &cam_h, &count, &flow_point_size, &divergence, &new_divergence, &mean_tti, &median_tti, &d_heading, &d_pitch, &pu[2], &pv[2], &divergence_error, n_inlier_minu, n_inlier_minv, &DIV_FILTER);
    }

	//DOWNLINK_SEND_OF_ERROR(DefaultChannel, DefaultDevice, &error_corner, &error_opticflow);

	// Send to paparazzi
	gst2ppz.ID = 0x0001;
	gst2ppz.counter++; // to keep track of data through ppz communication

}

