
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Computer Vision
#include "opticflow/optic_flow_gdc.h"
#include "trig.h"
//#include "opticflow/fastRosten.h"
#include "opticflow_module.h"

// Own Header
#include "opticflow_code.h"

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
unsigned int verbose = 0;

// Local variables
//static unsigned char * img_uncertainty;
//#define showframe 1
unsigned char *prev_frame, *gray_frame, *prev_gray_frame;
#ifdef showframe
unsigned char *copy_frame;
#endif

int old_img_init;

int tot_x, tot_y, x_buf[24], y_buf[24], opt_trans_x_buf[32], opt_trans_y_buf[32];
float opt_angle_x_raw, opt_angle_y_raw, x_avg, y_avg, camh_buf[24], cam_h_med;

unsigned int buf_point = 0;
unsigned int buf_point_camh = 0;

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

// Flow fitting
float mean_tti, median_tti, d_heading, d_pitch, pu[3], pv[3], divergence_error;

// tryout
struct NedCoor_i OF_speed;

// Called by plugin
void my_plugin_init(void)
{

	// Init variables
	gray_frame = (unsigned char *) calloc(imgWidth*imgHeight,sizeof(unsigned char));
	prev_gray_frame = (unsigned char *) calloc(imgWidth*imgHeight,sizeof(unsigned char));
	prev_frame = (unsigned char *) calloc(imgWidth*imgHeight*2,sizeof(unsigned char));
#ifdef showframe
	copy_frame = (unsigned char *) calloc(imgWidth*imgHeight*2,sizeof(unsigned char));
#endif
	old_img_init = 1;

	tot_x = 0;
	tot_y = 0;
	x_avg = 0.0;
	y_avg = 0.0;
	opt_angle_x_raw = 0.0;
	opt_angle_y_raw = 0,0;

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

	cam_h_med = 0.0;

	Velx = 0.0;
	Vely = 0.0;
	Velz = 0.0;

	mean_tti = 0.0;
	median_tti = 0.0;
	d_heading = 0.0;
	d_pitch = 0.0;
	divergence_error = 0.0;
	pu[0] = 0.0; pu[1] = 0.0; pu[2] = 0.0;
	pv[0] = 0.0; pv[1] = 0.0; pv[2] = 0.0;

	opt_angle_x_raw = 0;
	opt_angle_y_raw = 0;

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

	// tryout
	OF_speed.x = 0;
	OF_speed.y = 0;
	OF_speed.z = 0;
}

void my_plugin_run(unsigned char *frame)
{

#ifdef showframe
	memcpy(copy_frame,frame,imgHeight*imgWidth*2);
#endif

	if(old_img_init == 1)
	{
		memcpy(prev_frame,frame,imgHeight*imgWidth*2);
		old_img_init = 0;
	}

	// ***********************************************************************************************************************
	// Additional information from other sensors
	// ***********************************************************************************************************************

#if USE_SONAR
		cam_h = ins_impl.sonar_z;
#else
		cam_h = 1;
		prev_cam_h = 1;
#endif

		camh_buf[buf_point_camh] = cam_h;
		buf_point_camh = (buf_point_camh+1) %11;
		quick_sort(camh_buf,11);
		cam_h_med = camh_buf[6];

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
    	int threshold_n_points = 15; //25
    	if(flow_point_size < threshold_n_points)
    	{
        	findPoints(gray_frame, frame, imgWidth, imgHeight, &count, max_count, MAX_COUNT, flow_points, &flow_point_size, detected_points0);
    	}
    }

	// **********************************************************************************************************************
	// (2) track the points to the new image, possibly using external information (TTC, known lateral / rotational movements)
	// **********************************************************************************************************************
    trackPoints(frame, prev_frame, imgWidth, imgHeight, &count, max_count, MAX_COUNT, flow_points, &flow_point_size, detected_points0, detected_points1, x, y, new_x, new_y, dx, dy, status);

#ifdef showframe
    	showFlow(frame, x, y, status, count, new_x, new_y, imgWidth, imgHeight);
#endif
		for (int i=0; i<count;i++)
		{
			dx[i] = flow_points[i].dx;
			dy[i] = flow_points[i].dy;
		}

//    	OFfilter(&opt_angle_x_raw, &opt_angle_y_raw, flow_points, count, 1);
    	OFfilter(&opt_angle_x_raw, &opt_angle_y_raw, flow_points, count, 2);

		/*
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
*/
	// Velocity Computation

	Velx = opt_angle_y_raw*cam_h_med/Fy_ARdrone*FPS;
	Vely = -opt_angle_x_raw*cam_h_med/Fx_ARdrone*FPS;

	OF_speed.x = (int) (Velx*(1<<(INT32_SPEED_FRAC)));
	OF_speed.y = (int) (Vely*(1<<(INT32_SPEED_FRAC)));

	stateSetSpeedNed_i(&OF_speed);

    // compute divergence/ TTI
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
#ifdef showframe
    memcpy(prev_frame,copy_frame,imgHeight*imgWidth*2);
#else
    memcpy(prev_frame,frame,imgHeight*imgWidth*2);
#endif

	for (int i=0;i<count;i++)
	{
		swap_points[i] = detected_points0[i];
		detected_points0[i] = detected_points1[i];
		detected_points1[i] = swap_points[i];
	}

	DOWNLINK_SEND_OPTIC_FLOW(DefaultChannel, DefaultDevice, &FPS, &opt_angle_x_raw, &opt_angle_y_raw, &OF_speed.x, &OF_speed.y, &stateGetSpeedNed_i()->x, &stateGetSpeedNed_i()->y, &diff_roll, &diff_pitch, &cam_h_med, &count, &flow_point_size, &divergence, &new_divergence, &mean_tti, &median_tti, &d_heading, &d_pitch, &pu[2], &pv[2], &divergence_error, n_inlier_minu, n_inlier_minv, &DIV_FILTER);

}

