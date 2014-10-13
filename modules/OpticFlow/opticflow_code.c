#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Computer Vision
#include "opticflow/optic_flow_gdc.h"
#include "trig.h"
//#include "opticflow/fastRosten.h"
#include "opticflow/fast12/fastRosten.h"
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

//#include "paparazzi.h"

// Settable by pluging
unsigned int imgWidth, imgHeight;
unsigned int verbose = 0;
//#define showframe 1

// Local variables
unsigned char *prev_frame, *gray_frame;
#ifdef showframe
unsigned char *copy_frame;
#endif

int old_img_init;

float OFx, OFy, dx_sum, dy_sum;

int *x, *y, *new_x, *new_y, *status, *dx, *dy;
int error_opticflow;

#define PI 3.14159265359
#define FOV_H 0.67020643276 //(38.4/180*PI);
#define FOV_W 0.89360857702 //(51.2/180*PI);

// Corner Detection
int32_t fast_tune;
int count = 0;
int max_count = 25;
int flow_point_size = 0;
#define MAX_COUNT 150	// Maximum number of flow points
//flowPoint flow_points[MAX_COUNT];
//detectedPoint detected_points0[MAX_COUNT];
//detectedPoint detected_points1[MAX_COUNT];
//detectedPoint swap_points[MAX_COUNT];

// Flow Derotation
/*
 * 1 deg = (2*arctan(0.5*imW/f))/FOV = xDerotate = yDerotate
 * FOVx = 50 deg, FOVy = 38 deg, imgW = 320, imgH = 240, Fx = 343.1211, Fy = 348.5053
 */

float curr_pitch, curr_roll, prev_pitch, prev_roll;
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
#define VISION_FAST_THRES 15
#endif

// Compute body velocities
struct FloatVect3 V_Ned;
struct FloatRMat Rmat_Ned2Body;
struct FloatVect3 V_body;

// Called by plugin
void my_plugin_init(void)
{

	// Init variables
	gray_frame = (unsigned char *) calloc(imgWidth*imgHeight,sizeof(unsigned char));
	prev_frame = (unsigned char *) calloc(imgWidth*imgHeight*2,sizeof(unsigned char));
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
	dy = (int *) calloc(MAX_COUNT,sizeof(int));

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
		old_img_init = 0;
	}

	// ***********************************************************************************************************************
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

	// ***********************************************************************************************************************
	// (1) possibly find new points - keeping possible old ones (normal cv methods / efficient point finding / active corners)
	// ***********************************************************************************************************************

//    int ALWAYS_NEW_POINTS = 1;
//
//    if(ALWAYS_NEW_POINTS)
//    {
//    	// Clear corners
//    	// memset(flow_points,0,sizeof(flowPoint)*flow_point_size);
//    	findPoints(gray_frame, frame, imgWidth, imgHeight, &count, max_count, MAX_COUNT, flow_points, &flow_point_size, detected_points0);
//    }
//    else
//    {
//    	int threshold_n_points = 25; //25
//    	if(flow_point_size < threshold_n_points)
//    	{
//        	findPoints(gray_frame, frame, imgWidth, imgHeight, &count, max_count, MAX_COUNT, flow_points, &flow_point_size, detected_points0);
//    	}
//    }
	// FAST corner:
	int fast_threshold = fast_tune; //10
	xyFAST* pnts_fast;

	CvtYUYV2Gray(gray_frame, frame, imgWidth, imgHeight); // convert to gray scaled image is a must for FAST corner

	pnts_fast = fast12_detect((const byte*)gray_frame, imgWidth, imgHeight, imgWidth, fast_threshold, &count);

	if(count > max_count) count = max_count;

	for(int i = 0; i < count; i++)
	{
		x[i] = pnts_fast[i].x;
		y[i] = pnts_fast[i].y;
	}

	free(pnts_fast);
	// **********************************************************************************************************************
	// (2) track the points to the new image, possibly using external information (TTC, known lateral / rotational movements)
	// **********************************************************************************************************************
//    trackPoints(frame, prev_frame, imgWidth, imgHeight, &count, max_count, MAX_COUNT, flow_points, &flow_point_size, detected_points0, detected_points1, x, y, new_x, new_y, dx, dy, status);

	error_opticflow = opticFlowLK(frame, prev_frame, x, y, count, imgWidth, imgHeight, new_x, new_y, status, 5, MAX_COUNT);

#ifdef showframe
	showFlow(frame, x, y, status, count, new_x, new_y, imgWidth, imgHeight);
#endif
	dx_sum = 0.0;
	dy_sum = 0.0;
	for (int i=0; i<count;i++)
	{
//			dx[i] = flow_points[i].dx;
//			dy[i] = flow_points[i].dy;
		dx[i] = new_x[i] - x[i];
		dy[i] = new_y[i] - y[i];
		dx_sum += (float) dx[i];
		dy_sum += (float) dy[i];
	}
	if(count)
	{
		dx_sum = dx_sum/count;
		dy_sum = dy_sum/count;
	}

	// Flow Derotation
	curr_pitch = stateGetNedToBodyEulers_f()->theta;
	curr_roll = stateGetNedToBodyEulers_f()->phi;

	diff_pitch = (curr_pitch - prev_pitch)*imgHeight/FOV_H;
	diff_roll = (curr_roll - prev_roll)*imgWidth/FOV_W;

	prev_pitch = curr_pitch;
	prev_roll = curr_roll;

	if(count)
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

	OFfilter2(&OFx, &OFy, OFx_trans, OFy_trans, count, 2);

	// Velocity Computation
	#ifdef USE_SONAR
		cam_h = ins_impl.sonar_z;
	#else
		cam_h = 1;
	#endif

	Velx = OFy*cam_h*FPS/Fy_ARdrone;
	Vely = -OFx*cam_h*FPS/Fx_ARdrone;


	if(snapshot)
	{
		sprintf(filename, "/data/video/image_%d.dat", i_frame);
		saveSingleImageDataFile(frame, imgWidth, imgHeight, filename);

		snapshot = FALSE;
		i_frame ++ ;
	}

	// *********************************************
	// (5) housekeeping to prepare for the next call
	// *********************************************
#ifdef showframe
    memcpy(prev_frame,copy_frame,imgHeight*imgWidth*2);
#else
    memcpy(prev_frame,frame,imgHeight*imgWidth*2);
#endif

//	for (int i=0;i<count;i++)
//	{
//		swap_points[i] = detected_points0[i];
//		detected_points0[i] = detected_points1[i];
//		detected_points1[i] = swap_points[i];
//	}
	DOWNLINK_SEND_OF_HOVER(DefaultChannel, DefaultDevice, &FPS, &dx_sum, &dy_sum, &OFx, &OFy, &diff_roll, &diff_pitch, &Velx, &Vely, &V_body.x, &V_body.y, &cam_h, &count);

}

