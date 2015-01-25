/*
 * Copyright (C) 2014 Hann Woei Ho
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/*
 * @file paparazzi/sw/ext/ardrone2_vision/modules/OpticFlow/opticflow_code.c
 * @brief optical-flow based hovering for Parrot AR.Drone 2.0
 *
 * Sensors from vertical camera and IMU of Parrot AR.Drone 2.0
 */

#include <stdio.h>
#include <stdlib.h>

// Own Header
#include "opticflow_code.h"

// Computer Vision
#include "opticflow/optic_flow_gdc.h"
#include "opticflow/fast9/fastRosten.h"
#include "opticflow_module.h"

// Paparazzi Data
#include "subsystems/ins/ins_int.h"
#include "subsystems/imu.h"
#include "subsystems/gps.h"

// Waypoints
#include "navigation.h"
#include "generated/flight_plan.h"
struct EnuCoor_i waypoint_ob1;

// Downlink
#include "subsystems/datalink/downlink.h"

// Timer
#include <sys/time.h>

// Settable by plugin
unsigned int imgWidth, imgHeight;
unsigned int verbose = 0;

// Local variables
unsigned char *prev_frame, *gray_frame, *prev_gray_frame;
int old_img_init;
float OFx, OFy, dx_sum, dy_sum;

// ARDrone Vertical Camera Parameters
#define FOV_H 0.67020643276
#define FOV_W 0.89360857702
#define Fx_ARdrone 343.1211
#define Fy_ARdrone 348.5053

// Corner Detection
int *x, *y;
int count = 0;
int max_count = 25;
#define MAX_COUNT 100

// Corner Tracking
int *new_x, *new_y, *status, *dx, *dy, *dx_copy, *dy_copy;
int error_opticflow;
int flow_count = 0;
int remove_point;
int c;
int borderx = 24, bordery = 24;

// Remove bad corners
float distance2, min_distance, min_distance2;

// Flow Derotation
#define FLOW_DEROTATION
float curr_pitch, curr_roll, curr_yaw, prev_pitch, prev_roll;
float cam_h, diff_roll, diff_pitch, OFx_trans, OFy_trans;

// Lateral Velocity Computation
float Velx, Vely;

// Compute body velocities
struct FloatVect3 V_Ned;
struct FloatRMat Rmat_Ned2Body;
struct FloatVect3 V_body;

// Flow fitting
float mean_tti, median_tti, d_heading, d_pitch, divergence_error, divergence,
      z_x, z_y, flatness, POE_x, POE_y, ground_divergence,  div_buf_3D[30], div_avg_3D;
unsigned int mov_block_3D, div_point_3D;
int *n_inlier_minu, *n_inlier_minv, DIV_FILTER;

// Called by plugin
void my_plugin_init(void)
{
	// Initialize variables
	gray_frame = (unsigned char *) calloc(imgWidth*imgHeight,sizeof(unsigned char));
	prev_frame = (unsigned char *) calloc(imgWidth*imgHeight*2,sizeof(unsigned char));
	prev_gray_frame = (unsigned char *) calloc(imgWidth*imgHeight,sizeof(unsigned char));
	x = (int *) calloc(MAX_COUNT,sizeof(int));
	new_x = (int *) calloc(MAX_COUNT,sizeof(int));
	y = (int *) calloc(MAX_COUNT,sizeof(int));
	new_y = (int *) calloc(MAX_COUNT,sizeof(int));
	status = (int *) calloc(MAX_COUNT,sizeof(int));
	dx = (int *) calloc(MAX_COUNT,sizeof(int));
	dy = (int *) calloc(MAX_COUNT,sizeof(int));
	dx_copy = (int *) calloc(MAX_COUNT,sizeof(int));
	dy_copy = (int *) calloc(MAX_COUNT,sizeof(int));
	old_img_init = 1;
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

	// Flow fitting
	n_inlier_minu = (int *)calloc(1,sizeof(int));
	n_inlier_minv = (int *)calloc(1,sizeof(int));
	mean_tti = 0.0, median_tti = 0.0, d_heading = 0.0, d_pitch = 0.0, divergence_error = 0.0, divergence = 0.0,
	z_x = 0.0, z_y = 0.0, flatness = 0.0, POE_x = 0.0, POE_y = 0.0 , ground_divergence = 0.0;
	DIV_FILTER = 0;
	div_point_3D = 0, div_avg_3D = 0.0, mov_block_3D = 6;
}

void my_plugin_run(unsigned char *frame)
{
//	// set heading
//	nav_set_heading_rad(5.73515192); // 328.6 deg = 5.73515192 rad

#ifdef VISION_OBSTACLE
	// update obstacle waypoint
	enu_of_ecef_pos_i(&waypoint_ob1, &state.ned_origin_i, &obstacle.ecef_pos);

	nav_move_waypoint(WP_ob1, &waypoint_ob1);
#endif

	if(old_img_init == 1)
	{
		memcpy(prev_frame,frame,imgHeight*imgWidth*2);
		CvtYUYV2Gray(prev_gray_frame, prev_frame, imgWidth, imgHeight);
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
    RMAT_VECT3_MUL(V_body, Rmat_Ned2Body, V_Ned);

	// ***********************************************************************************************************************
	// Corner detection
	// ***********************************************************************************************************************

	// FAST corner detection
	int fast_threshold = 20;
	xyFAST* pnts_fast;
	pnts_fast = fast9_detect((const byte*)prev_gray_frame, imgWidth, imgHeight, imgWidth, fast_threshold, &count);

	if(count > MAX_COUNT) count = MAX_COUNT;
	for(int i = 0; i < count; i++)
	{
		x[i] = pnts_fast[i].x;
		y[i] = pnts_fast[i].y;
	}
	free(pnts_fast);

	// Remove neighbouring corners
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
	// Corner Tracking
	// **********************************************************************************************************************
	CvtYUYV2Gray(gray_frame, frame, imgWidth, imgHeight);

	error_opticflow = opticFlowLK(gray_frame, prev_gray_frame, x, y, count_fil, imgWidth, imgHeight, new_x, new_y, status, 5, 100);

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

	// Flow Derotation
	curr_pitch = stateGetNedToBodyEulers_f()->theta;
	curr_roll = stateGetNedToBodyEulers_f()->phi;
	curr_yaw = stateGetNedToBodyEulers_f()->psi;

	diff_pitch = (curr_pitch - prev_pitch)*imgHeight/FOV_H;
	diff_roll = (curr_roll - prev_roll)*imgWidth/FOV_W;

	prev_pitch = curr_pitch;
	prev_roll = curr_roll;

	dx_sum = 0.0;
	dy_sum = 0.0;
	int dx_trans = 0, dy_trans = 0;
	int sub_flow = 100; //100

	// Optical Flow Computation
	for(int i=0; i<flow_count; i++)
	{
		dx[i] = new_x[i] - x[i];
		dy[i] = new_y[i] - y[i];

#ifdef FLOW_DEROTATION
		dx_trans = (dx[i]*sub_flow - diff_roll*sub_flow);
		dy_trans = (dy[i]*sub_flow - diff_pitch*sub_flow);
		x[i] = x[i]*sub_flow;
		y[i] = y[i]*sub_flow;

		if((dx_trans<=0) != (dx[i]<=0))
		{
			dx[i] = 0;
		}
		else
		{
			dx[i] = dx_trans;
		}

		if((dy_trans<=0) != (dy[i]<=0))
		{
			dy[i] = 0;
		}
		else
		{
			dy[i] = dy_trans;
		}
#endif
		// copy dx and dy for sorting in Median Filter
		dx_copy[i] = dx[i];
		dy_copy[i] = dy[i];
	}

	// Median Filter
	if(flow_count)
	{
		quick_sort_int(dx_copy,flow_count); // 11
		quick_sort_int(dy_copy,flow_count); // 11

		dx_sum = (float) dx_copy[flow_count/2];
		dy_sum = (float) dy_copy[flow_count/2];
	}
	else
	{
		dx_sum = 0.0;
		dy_sum = 0.0;
	}

	OFx_trans = dx_sum/sub_flow;
	OFy_trans = dy_sum/sub_flow;

	// Average Filter
	OFfilter(&OFx, &OFy, OFx_trans, OFy_trans, flow_count, 1);

	// Velocity Computation
	#ifdef USE_SONAR
		cam_h = ins_impl.sonar_z;
	#else
		cam_h = stateGetPositionEnu_f()->z;
	#endif

//	Velx = OFy*cam_h*FPS/Fy_ARdrone + 0.05;
//	Vely = -OFx*cam_h*FPS/Fx_ARdrone - 0.1;
//	Velx = OFy*cam_h*FPS/Fy_ARdrone + 0.1;
//	Vely = -OFx*cam_h*FPS/Fx_ARdrone - 0.1;
	if(flow_count)
	{
		Velx = OFy*cam_h*FPS/Fy_ARdrone + 0.1;
		Vely = -OFx*cam_h*FPS/Fx_ARdrone - 0.1;
	}
	else
	{
		Velx = 0.0;
		Vely = 0.0;
	}


	// **********************************************************************************************************************
	// Flow Field Fitting
	// **********************************************************************************************************************
    // compute divergence/ TTI
	int USE_FITTING = 1;
	int USE_MEAN_3D = 0;
	int USE_MEDIAN_3D = 1;
//	int USE_MEAN_3D = 0;
//	int USE_MEDIAN_3D = 1;
//	threshold_3D_low = 450.0;
//	threshold_3D_high = 1000.0;

	if(USE_FITTING == 1)
	{
		analyseTTI(&z_x, &z_y, &flatness, &POE_x, &POE_y, &divergence, &mean_tti, &median_tti, &d_heading, &d_pitch, &divergence_error, x, y, dx, dy, n_inlier_minu, n_inlier_minv, flow_count, imgWidth, imgHeight, &DIV_FILTER);

			if (USE_MEAN_3D == 1)
			{

				div_buf_3D[div_point_3D] = flatness;
				div_point_3D = (div_point_3D+1) %mov_block_3D; // index starts from 0 to mov_block
				div_avg_3D = 0.0;
				for (int im=0;im<mov_block_3D;im++)
				{
					div_avg_3D+=div_buf_3D[im];
				}
				flatness = (float) div_avg_3D/ mov_block_3D;
			}
			else if(USE_MEDIAN_3D == 1)
			{
				div_buf_3D[div_point_3D] = flatness;
				div_point_3D = (div_point_3D+1) %mov_block_3D;
				quick_sort(div_buf_3D,mov_block_3D);
				flatness = div_buf_3D[mov_block_3D/2];
			}
			else
			{

			}

	}

	// compute ground divergence (Only when OpticTrack is connected)
	if(cam_h)
	{
		ground_divergence = V_body.z/cam_h;
	}

	// **********************************************************************************************************************
	// Next Loop Preparation
	// **********************************************************************************************************************

	memcpy(prev_frame,frame,imgHeight*imgWidth*2);
	memcpy(prev_gray_frame,gray_frame,imgHeight*imgWidth);

	// **********************************************************************************************************************
	// Downlink Message
	// **********************************************************************************************************************
//	DOWNLINK_SEND_OF_HOVER(DefaultChannel, DefaultDevice, &FPS, &dx_sum, &dy_sum, &OFx, &OFy, &diff_roll, &diff_pitch, &Velx, &Vely, &V_body.x, &V_body.y, &cam_h, &flow_count);
#ifdef VISION_OBSTACLE
	DOWNLINK_SEND_OF_LAND_OBSTACLE(DefaultChannel, DefaultDevice, &FPS, &z_x, &z_y, &flatness, &POE_x, &POE_y,
			&divergence, &ground_divergence, &d_heading, &d_pitch, &flow_count,
			&V_body.x, &V_body.y, &V_body.z, &cam_h, &curr_pitch, &curr_roll, &curr_yaw,
			&stateGetPositionEnu_i()->x, &stateGetPositionEnu_i()->y, &stateGetPositionEnu_i()->z,
			&waypoint_ob1.x, &waypoint_ob1.y, &waypoint_ob1.z);
#else
	DOWNLINK_SEND_OF_LAND(DefaultChannel, DefaultDevice, &z_x, &z_y, &flatness, &POE_x, &POE_y, &divergence, &d_heading, &d_pitch, &V_body.z, &cam_h, &imu.accel.x, &imu.accel.y, &imu.accel.z);
#endif
	// rotation angle, height, obstacle position (measured size),
}

