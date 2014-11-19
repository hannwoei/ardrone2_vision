#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Computer Vision
#include "opticflow/optic_flow_gdc.h"
#include "trig.h"
//#include "opticflow/fastRosten.h"
#include "opticflow/fast9/fastRosten.h"
#include "opticflow_module.h"

// Own Header
#include "opticflow_code.h"

// Paparazzi Data
#include "state.h"
#include "subsystems/ins/ins_int.h"
#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"
#include "subsystems/imu.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "subsystems/gps.h"

// Downlink
#include "messages.h"
#include "subsystems/datalink/downlink.h"

// Timer
#include <sys/time.h>

// Settable by pluging
unsigned int imgWidth, imgHeight;
unsigned int verbose = 0;
//#define showframe 1

// Local variables
unsigned char *prev_frame, *gray_frame, *prev_gray_frame;
#ifdef showframe
unsigned char *copy_frame;
#endif

int old_img_init;

float OFx, OFy, dx_sum, dy_sum;

#define PI 3.14159265359
#define FOV_H 0.67020643276 //(38.4/180*PI);
#define FOV_W 0.89360857702 //(51.2/180*PI);

// Corner Detection
int32_t fast_tune;
int *x, *y;
int count = 0;
int max_count = 25;
#define MAX_COUNT 100	// Maximum number of flow points

// Corner Tracking
int *new_x, *new_y, *status, *dx, *dy;
int error_opticflow;
int flow_count = 0;
//int flow_point = 0;
int remove_point;
int c;
int borderx = 24, bordery = 24;

// Remove bad corners
float distance2, min_distance, min_distance2;

// Flow Derotation
/*
 * 1 deg = (2*arctan(0.5*imW/f))/FOV = xDerotate = yDerotate
 * FOVx = 50 deg, FOVy = 38 deg, imgW = 320, imgH = 240, Fx = 343.1211, Fy = 348.5053
 */
#define FLOW_DEROTATION

float curr_pitch, curr_roll, curr_yaw, prev_pitch, prev_roll;
float cam_h, diff_roll, diff_pitch, OFx_trans, OFy_trans;

// Lateral Velocity Computation
#define Fx_ARdrone 343.1211
#define Fy_ARdrone 348.5053
float Velx, Vely;

// snapshot
char filename[100];
int i_frame;
bool_t snapshot;

#ifndef VISION_SNAPSHOT
#define VISION_SNAPSHOT FALSE
#endif

#ifndef VISION_FAST_THRES
#define VISION_FAST_THRES 20
#endif

// Compute body velocities
struct FloatVect3 V_Ned;
struct FloatRMat Rmat_Ned2Body;
struct FloatVect3 V_body;

struct FloatVect3 A_body;
struct FloatRates curr_rate;

// Called by plugin
void my_plugin_init(void)
{

	// Init variables
	gray_frame = (unsigned char *) calloc(imgWidth*imgHeight,sizeof(unsigned char));
	prev_frame = (unsigned char *) calloc(imgWidth*imgHeight*2,sizeof(unsigned char));
	prev_gray_frame = (unsigned char *) calloc(imgWidth*imgHeight,sizeof(unsigned char));
#ifdef showframe
	copy_frame = (unsigned char *) calloc(imgWidth*imgHeight*2,sizeof(unsigned char));
#endif
	old_img_init = 1;

	x = (int *) calloc(MAX_COUNT,sizeof(int));
	new_x = (int *) calloc(MAX_COUNT,sizeof(int));
	y = (int *) calloc(MAX_COUNT,sizeof(int));
	new_y = (int *) calloc(MAX_COUNT,sizeof(int));
	status = (int *) calloc(MAX_COUNT,sizeof(int));
	dx = (int *) calloc(MAX_COUNT,sizeof(int));
	dy = (int *) calloc(MAX_COUNT,sizeof(int));FOV_W

	OFx = 0.0;
	OFy = 0.0;
	dx_sum = 0.0;
	dy_sum = 0.0;

	diff_roll = 0.0;
	diff_pitch = 0.0;
	cam_h = 0.0;
	prev_pitch = 0.0;
	prev_roll = 0.0;
	curr_pitch = 0.0;
	curr_roll = 0.0;
	curr_yaw = 0.0;
	OFx_trans = 0.0;
	OFy_trans = 0.0;

	Velx = 0.0;
	Vely = 0.0;

	// snapshot
	i_frame = 0;
	snapshot = VISION_SNAPSHOT;
	fast_tune = VISION_FAST_THRES;

}

void my_plugin_run(unsigned char *frame)
{

#ifdef showframe
	memcpy(copy_frame,frame,imgHeight*imgWidth*2);
#endif

	if(old_img_init == 1)
	{
		memcpy(prev_frame,frame,imgHeight*imgWidth*2);
		CvtYUYV2Gray(prev_gray_frame, prev_frame, imgWidth, imgHeight); // convert to gray scaled image is a must for FAST corner
		old_img_init = 0;
	}

	// ***********************************************FOV_W************************************************************************
	// Additional information from other sensors
	// ***********************************************************************************************************************

    // Compute body velocities from ENU
    V_Ned.x = stateGetSpeedNed_f()->x;
    V_Ned.y = stateGetSpeedNed_f()->y;
    V_Ned.z = stateGetSpeedNed_f()->z;

    struct FloatQuat* BodyQuaternions = stateGetNedToBodyQuat_f();

    FLOAT_RMAT_OF_QUAT(Rmat_Ned2Body,*BodyQuaternions);

    //FLOAT_RMAT_VECT3_MUL(V_body, Rmat_Ned2Body, V_Ned);
    RMAT_VECT3_MUL(V_body, Rmat_Ned2Body, V_Ned);

    ACCELS_FLOAT_OF_BFP(A_body,imu.accel);
    curr_rate.p = RATE_FLOAT_OF_BFP(stateGetBodyRates_i()->p);
    curr_rate.q = RATE_FLOAT_OF_BFP(stateGetBodyRates_i()->q);
    curr_rate.r = RATE_FLOAT_OF_BFP(stateGetBodyRates_i()->r);

		// ***********************************************************************************************************************
		// FAST corner detection
		// ***********************************************************************************************************************

		// FAST corner:
		int fast_threshold = fast_tune; //20
		xyFAST* pnts_fast;

		pnts_fast = fast9_detect((const byte*)prev_gray_frame, imgWidth, imgHeight, imgWidth,
				fast_threshold, &count);

		if(count > MAX_COUNT) count = MAX_COUNT;

		for(int i = 0; i < count; i++)
		{
			x[i] = pnts_fast[i].x;
			y[i] = pnts_fast[i].y;
		}

		free(pnts_fast);

		// b) remove neighbouring corners
		min_distance = 3;
		min_distance2 = min_distance*min_distance;
		int *labelmin;
		labelmin = (int *) calloc(MAX_COUNT,sizeof(int));

		for(int i = 0; i < count; i++)
		{
			for(int j = i+1; j < count; j++)
			{
				// distance squared:
				distance2 = (x[i] - x[j])*(x[i] - x[j]) + (y[i] - y[j])*(y[i] - y[j]);
				if(distance2 < min_distance2)
				{
					labelmin[i] = 1;
				}
			}
		}

		int count_fil = count;
		for(int i = count-1; i >= 0; i-- )
	    {
			remove_point = 0;

	        if(labelmin[i])
			{
				remove_point = 1;
			}

			if(remove_point)
			{
				for(c = i; c <count_fil-1; c++)
				{
					x[c] = x[c+1];
					y[c] = y[c+1];
				}
				count_fil--;
			}
		}

		if(count_fil>max_count) count_fil = max_count;

		count = count_fil;
		free(labelmin);

		// **********************************************************************************************************************
		// (2) track the points to the new image, possibly using external information (TTC, known lateral / rotational movements)
		// **********************************************************************************************************************
		CvtYUYV2Gray(gray_frame, frame, imgWidth, imgHeight); // convert to gray scaled image is a must for FAST corner

		error_opticflow = opticFlowLK(gray_frame, prev_gray_frame, x, y, count_fil, imgWidth, imgHeight,
				new_x, new_y, status, 5, 100);

		flow_count = count_fil;
		for(int i=count_fil-1; i>=0; i--)
		{
			remove_point = 1;

			if(status[i] && !(new_x[i] < borderx || new_x[i] > (imgWidth-1-borderx) ||
					new_y[i] < bordery || new_y[i] > (imgHeight-1-bordery)))
			{
				remove_point = 0;
			}

			if(remove_point)
			{
				for(c = i; c <flow_count-1; c++)
				{
					x[c] = x[c+1];
					y[c] = y[c+1];
					new_x[c] = new_x[c+1];
					new_y[c] = new_y[c+1];
				}
				flow_count--;
			}
		}

		dx_sum = 0.0;
		dy_sum = 0.0;

		for(int i=0; i<flow_count; i++)
		{
			dx[i] = new_x[i] - x[i];
			dy[i] = new_y[i] - y[i];
//			dx_sum += (float) dx[i];
//			dy_sum += (float) dy[i];
		}

//		if(flow_count)
//		{
//			dx_sum = (float) dx_sum/flow_count;
//			dy_sum = (float) dy_sum/flow_count;
//		}
//		else
//		{
//			dx_sum = 0.0;
//			dy_sum = 0.0;
//		}

		if(flow_count)
		{
			quick_sort_int(dx,flow_count); // 11
			quick_sort_int(dy,flow_count); // 11

			dx_sum = (float) dx[flow_count/2];
			dy_sum = (float) dy[flow_count/2];
		}
		else
		{
			dx_sum = 0.0;
			dy_sum = 0.0;
		}




//		error_opticflow = opticFlowLK(frame, prev_frame, x, y, count_fil, imgWidth, imgHeight, new_x, new_y, status, 20, MAX_COUNT);
//
//		dx_sum = 0.0;
//		dy_sum = 0.0;
//
//		// remove bad tracking
//		flow_count = 0;
//
//		for (int i=0; i<count_fil;i++)
//		{
//			// ignore bad tracking
//			if(status[i] == 1)
//			{
//				dx[flow_count] = new_x[i] - x[i];
//				dy[flow_count] = new_y[i] - y[i];
//				dx_sum += (float) dx[i];
//				dy_sum += (float) dy[i];
//				flow_count++;
//			}
//
//		}




#ifdef showframe
	showFlow(frame, x, y, status, count, new_x, new_y, imgWidth, imgHeight);
#endif

		curr_pitch = stateGetNedToBodyEulers_f()->theta;
		curr_roll = stateGetNedToBodyEulers_f()->phi;
		curr_yaw = stateGetNedToBodyEulers_f()->psi;

//		diff_pitch = (curr_pitch - prev_pitch)*atan((double)imgHeight/Fy_ARdrone)/FOV_H;
//		diff_roll = (curr_roll - prev_roll)*atan((double)imgWidth/Fx_ARdrone)/FOV_W;
		diff_pitch = (curr_pitch - prev_pitch)*imgHeight/FOV_H;
		diff_roll = (curr_roll - prev_roll)*imgWidth/FOV_W;

		prev_pitch = curr_pitch;
		prev_roll = curr_roll;

#ifdef FLOW_DEROTATION
		if(flow_count)
		{
			OFx_trans = dx_sum - diff_roll;
			OFy_trans = dy_sum - diff_pitch;

			// avoid over-correlated flow
			if((OFx_trans<=0) != (dx_sum<=0))
			{
				OFx_trans = 0;
				OFy_trans = 0;
			}
		}
		else
		{
			OFx_trans = dx_sum;
			OFy_trans = dy_sum;
		}
#else
		OFx_trans = dx_sum;
		OFy_trans = dy_sum;
#endif


	OFfilter2(&OFx, &OFy, OFx_trans, OFy_trans, flow_count, 1);
//	float alpha = 0.16;
//	OFx = alpha*OFx_trans + (1-alpha)*OFx;
//	OFy = alpha*OFy_trans + (1-alpha)*OFy;
//	OFx = dx_sum;
//	OFy = dy_sum;

	// Velocity Computation
	#ifdef USE_SONAR
		cam_h = ins_impl.sonar_z;
	#else
		cam_h = 1;
	#endif

	Velx = OFy*cam_h*FPS/Fy_ARdrone + 0.05;
	Vely = -OFx*cam_h*FPS/Fx_ARdrone - 0.1;


	if(snapshot)
	{
		sprintf(filename, "/data/video/usb0/image_%d.dat", i_frame);
		saveSingleImageDataFile(frame, imgWidth, imgHeight, filename);

		//snapshot = FALSE;
		i_frame ++ ;
	}

	// *********************************************
	// (5) housekeeping to prepare for the next call
	// *********************************************

	#ifdef showframe
		memcpy(prev_frame,copy_frame,imgHeight*imgWidth*2);
	#else
		memcpy(prev_frame,frame,imgHeight*imgWidth*2);
		memcpy(prev_gray_frame,gray_frame,imgHeight*imgWidth);
	#endif

//	for (int i=0;i<count;i++)
//	{
//		swap_points[i] = detected_points0[i];
//		detected_points0[i] = detected_points1[i];
//		detected_points1[i] = swap_points[i];
//	}

//	DOWNLINK_SEND_OF_CHECK(DefaultChannel, DefaultDevice, &FPS, &dx_sum, &dy_sum, &curr_pitch, &curr_roll, &curr_yaw, &curr_rate.p, &curr_rate.q, &curr_rate.r, &V_body.x, &V_body.y, &V_body.z, &A_body.x, &A_body.y, &A_body.z, &cam_h, &count, &i_frame);


	  DOWNLINK_SEND_OF_HOVER(DefaultChannel, DefaultDevice, &FPS, &dx_sum, &dy_sum, &OFx, &OFy, &diff_roll, &diff_pitch, &Velx, &Vely, &V_body.x, &V_body.y, &cam_h, &count);
}

