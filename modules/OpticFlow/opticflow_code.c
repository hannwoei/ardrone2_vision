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
int *x, *y, *x_copy, *y_copy;
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

// Appearance Landing
#define APPEARANCE_LANDING
#ifdef APPEARANCE_LANDING
float **** dictionary, alpha, *word_distribution, flatness_appearance, flatness_appearance2, appearance_uncertainty;
int n_words, patch_size, n_samples, learned_samples, n_samples_image, filled, WORDS, save_dictionary, RANDOM_SAMPLES, border_width, border_height;
float WR0, WR1, WR2, WR3, WR4, WR5, WR6, WR7, WR8, WR9, WR10, WR11, WR12, WR13, WR14, WR15, WR16, WR17, WR18, WR19, WR20, WR21, WR22, WR23, WR24, WR25,
WR26, WR27, WR28, WR29, WR30;
//float WR0_2, WR1_2, WR2_2, WR3_2, WR4_2, WR5_2, WR6_2, WR7_2, WR8_2, WR9_2, WR10_2, WR11_2, WR12_2, WR13_2, WR14_2, WR15_2, WR16_2, WR17_2, WR18_2, WR19_2, WR20_2,
//WR21_2, WR22_2, WR23_2, WR24_2, WR25_2, WR26_2, WR27_2, WR28_2, WR29_2, WR30_2;
#endif

// snapshot
char filename[100];
int i_frame;
bool_t train_dictionary, extract_distribution, snapshot;
unsigned int land_distribution;

#ifndef VISION_TRAIN_DICTIONARY
#define VISION_TRAIN_DICTIONARY FALSE
#endif

#ifndef VISION_EXTRACT_DISTRIBUTION
#define VISION_EXTRACT_DISTRIBUTION FALSE
#endif

#ifndef VISION_SNAPSHOT
#define VISION_SNAPSHOT FALSE
#endif

// Called by plugin
void my_plugin_init(void)
{
	// Initialize variables
	gray_frame = (unsigned char *) calloc(imgWidth*imgHeight,sizeof(unsigned char));
	prev_frame = (unsigned char *) calloc(imgWidth*imgHeight*2,sizeof(unsigned char));
	prev_gray_frame = (unsigned char *) calloc(imgWidth*imgHeight,sizeof(unsigned char));
	x = (int *) calloc(MAX_COUNT,sizeof(int));
	x_copy = (int *) calloc(MAX_COUNT,sizeof(int));
	new_x = (int *) calloc(MAX_COUNT,sizeof(int));
	y = (int *) calloc(MAX_COUNT,sizeof(int));
	y_copy = (int *) calloc(MAX_COUNT,sizeof(int));
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

	// Appearance Landing
#ifdef APPEARANCE_LANDING
	n_words = 30, patch_size = 6, n_samples = 400000, learned_samples = 0, filled = 0, save_dictionary = 1, RANDOM_SAMPLES = 1, border_width = 80, border_height = 60;
	alpha = 0.5, word_distribution = (float*)calloc(n_words,sizeof(float)), flatness_appearance = 0.0, flatness_appearance2 = 0.0;
	n_samples_image = 50; // 100: train dictionary
	WORDS = 0; // 0: train a dictionary
	appearance_uncertainty = 0.0;

	// create a dictionary
	dictionary = (float ****)calloc(n_words,sizeof(float***));

	for(int i = 0; i < n_words; i++)
	{
		dictionary[i] = (float ***)calloc(patch_size,sizeof(float **));

		for(int j = 0; j < patch_size;j++)
		{
			dictionary[i][j] = (float **)calloc(patch_size,sizeof(float*));

			for(int k = 0; k < patch_size; k++)
			{
				dictionary[i][j][k] = (float *)calloc(2,sizeof(float));
			}
		}
	}

	WR0 = 0.0, WR1 = 0.0, WR2 = 0.0, WR3 = 0.0, WR4 = 0.0, WR5 = 0.0, WR6 = 0.0, WR7 = 0.0, WR8 = 0.0, WR9 = 0.0, WR10 = 0.0,
	WR11 = 0.0, WR12 = 0.0, WR13 = 0.0, WR14 = 0.0, WR15 = 0.0, WR16 = 0.0, WR17 = 0.0, WR18 = 0.0, WR19 = 0.0, WR20 = 0.0,
	WR21 = 0.0, WR22 = 0.0, WR23 = 0.0, WR24 = 0.0, WR25 = 0.0, WR26 = 0.0, WR27 = 0.0, WR28 = 0.0, WR29 = 0.0, WR30 = 0.0;

//	WR0_2 = 0.0, WR1_2 = 0.0, WR2_2 = 0.0, WR3_2 = 0.0, WR4_2 = 0.0, WR5_2 = 0.0, WR6_2 = 0.0, WR7_2 = 0.0, WR8_2 = 0.0, WR9_2 = 0.0, WR10_2 = 0.0,
//	WR11_2 = 0.0, WR12_2 = 0.0, WR13_2 = 0.0, WR14_2 = 0.0, WR15_2 = 0.0, WR16_2 = 0.0, WR17_2 = 0.0, WR18_2 = 0.0, WR19_2 = 0.0, WR20_2 = 0.0,
//	WR21_2 = 0.0, WR22_2 = 0.0, WR23_2 = 0.0, WR24_2 = 0.0, WR25_2 = 0.0, WR26_2 = 0.0, WR27_2 = 0.0, WR28_2 = 0.0, WR29_2 = 0.0, WR30_2 = 0.0;
#endif

	// snapshot
	i_frame = 0;
	snapshot = VISION_SNAPSHOT;
	train_dictionary = VISION_TRAIN_DICTIONARY; // initialize false
	extract_distribution = VISION_EXTRACT_DISTRIBUTION;
	land_distribution = 0;
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
//		x[i] = x[i]*sub_flow;
//		y[i] = y[i]*sub_flow;

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
		x_copy[i] = x[i];
		y_copy[i] = y[i];
		dx[i] = dy_copy[i]/sub_flow;
		dy[i] = -dx_copy[i]/sub_flow;
		x[i] = imgHeight/2 - y_copy[i];
		y[i] = x_copy[i] - imgWidth/2;
//		printf("(%d, %d) ",x[i],y[i]);
	}
//	printf("\n");

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
	// Appearance Landing
	// **********************************************************************************************************************
#ifdef APPEARANCE_LANDING
	// Dictionary Training
	if(!WORDS && train_dictionary)
	{
		n_samples_image = 100;
		DictionaryTrainingYUV(dictionary, frame, n_words, patch_size, &learned_samples, n_samples_image, alpha, imgWidth, imgHeight, &filled);
	}

	if(extract_distribution && !WORDS)
	{
		//load a dictionary
		FILE *fdl;
		fdl=fopen("/data/video/VisualWords3.dat", "r");

		if(fdl == NULL)
		{
			perror("Error while opening the file.\n");
		}
		else
		{
			for(int i = 0; i < n_words; i++)
			{
				for(int j = 0; j < patch_size;j++)
				{
					for(int k = 0; k < patch_size; k++)
					{
						if(fscanf(fdl, "%f\n", &dictionary[i][j][k][0]) == EOF) break;
						if(fscanf(fdl, "%f\n", &dictionary[i][j][k][1]) == EOF) break;
					}
				}
			}
			fclose(fdl);
			WORDS = 1;
			printf("load dictionary done!\n");
		}

		WR0 = 48099, WR1 = -46842, WR2 = -46928, WR3 = -47082, WR4 = -46889, WR5 = -47530, WR6 = -46625, WR7 = -47127, WR8 = -47083, WR9 = -47162, WR10 = -47225,
		WR11 = -47363, WR12 = -46966, WR13 = -47174, WR14 = -47015, WR15 = -46143, WR16 = -47118, WR17 = -47126, WR18 = -47235, WR19 = -47345, WR20 = -47044,
		WR21 = -46892, WR22 = -47285, WR23 = -46936, WR24 = -47102, WR25 = -6692,	WR26 = -46994, WR27 = -46805, WR28 = -47011, WR29 = -47124, WR30 = -47048;

//		WR0_2 = 51071, WR1_2 = -49796, WR2_2 = -49812, WR3_2 = -49935, WR4_2 = -49792, WR5_2 = -50474, WR6_2 = -49638, WR7_2 = -50056, WR8_2 = -49998, WR9_2 = -50072, WR10_2 = -50155,
//		WR11_2 = -50274, WR12_2 = -49892, WR13_2 = -50049, WR14_2 = -49936, WR15_2 = -49184, WR16_2 = -50030, WR17_2 = -50038, WR18_2 = -50098, WR19_2 = -50251, WR20_2 = -49926,
//		WR21_2 = -49734, WR22_2 = -50232, WR23_2 = -49847, WR24_2 = -50003, WR25_2 = -49509, WR26_2 = -49912, WR27_2 = -49705, WR28_2 = -49935, WR29_2 = -50038, WR30_2 = -49978;
	}

	if(learned_samples >= n_samples && !WORDS)
	{
		printf("Done !!!\n");
		WORDS = 1;
		// lower learning rate
		alpha = 0.0;

		if(save_dictionary)
		{
//			printf("save dictionary\n");
			//save a dictionary
			FILE *fdd;
			fdd=fopen("/data/video/VisualWords3.dat", "w");

			if(fdd == NULL)
			{
				perror("Error while opening the file.\n");
			}
			else
			{
				// delete dictionary
				for(int i = 0; i < n_words; i++)
				{
					for(int j = 0; j < patch_size;j++)
					{
						for(int k = 0; k < patch_size; k++)
						{
							fprintf(fdd, "%f\n",dictionary[i][j][k][0]);
							fprintf(fdd, "%f\n",dictionary[i][j][k][1]);
	//						free(dictionary[i][j][k]);
						}
	//					free(dictionary[i][j]);
					}
	//				free(dictionary[i]);
				}
	//			free(dictionary);
				fclose(fdd);
			}
			save_dictionary = 0;
		}
	}

	// flatness model
	if(extract_distribution)
	{
		n_samples_image = 50;
		DistributionExtraction(dictionary, frame, word_distribution, n_words, patch_size, n_samples_image, RANDOM_SAMPLES, imgWidth, imgHeight, border_width, border_height, &appearance_uncertainty);

//		flatness_appearance = WR0
//				+ word_distribution[0]*WR1 + word_distribution[1]*WR2 + word_distribution[2]*WR3 + word_distribution[3]*WR4 + word_distribution[4]*WR5
//				+ word_distribution[5]*WR6 + word_distribution[6]*WR7 + word_distribution[7]*WR8 + word_distribution[8]*WR9 + word_distribution[9]*WR10
//				+ word_distribution[10]*WR11 + word_distribution[11]*WR12 + word_distribution[12]*WR13 + word_distribution[13]*WR14 + word_distribution[14]*WR15
//				+ word_distribution[15]*WR16 + word_distribution[16]*WR17 + word_distribution[17]*WR18 + word_distribution[18]*WR19 + word_distribution[19]*WR20
//				+ word_distribution[20]*WR21 + word_distribution[21]*WR22 + word_distribution[22]*WR23 + word_distribution[23]*WR24 + word_distribution[24]*WR25
//				+ word_distribution[25]*WR26 + word_distribution[26]*WR27 + word_distribution[27]*WR28 + word_distribution[28]*WR29 + word_distribution[29]*WR30;

//		flatness_appearance2 = WR0_2
//				+ word_distribution[0]*WR1_2 + word_distribution[1]*WR2_2 + word_distribution[2]*WR3_2 + word_distribution[3]*WR4_2 + word_distribution[4]*WR5_2
//				+ word_distribution[5]*WR6_2 + word_distribution[6]*WR7_2 + word_distribution[7]*WR8_2 + word_distribution[8]*WR9_2 + word_distribution[9]*WR10_2
//				+ word_distribution[10]*WR11_2 + word_distribution[11]*WR12_2 + word_distribution[12]*WR13_2 + word_distribution[13]*WR14_2 + word_distribution[14]*WR15_2
//				+ word_distribution[15]*WR16_2 + word_distribution[16]*WR17_2 + word_distribution[17]*WR18_2 + word_distribution[18]*WR19_2 + word_distribution[19]*WR20_2
//				+ word_distribution[20]*WR21_2 + word_distribution[21]*WR22_2 + word_distribution[22]*WR23_2 + word_distribution[23]*WR24_2 + word_distribution[24]*WR25_2
//				+ word_distribution[25]*WR26_2 + word_distribution[26]*WR27_2 + word_distribution[27]*WR28_2 + word_distribution[28]*WR29_2 + word_distribution[29]*WR30_2;

//		flatness_appearance = 42869
//				+ word_distribution[0]*(-41943) + word_distribution[1]*(-41908) + word_distribution[2]*(-41932) + word_distribution[3]*(-41837) + word_distribution[4]*(-41990)
//				+ word_distribution[5]*(-42006) + word_distribution[6]*(-41931) + word_distribution[7]*(-42106) + word_distribution[8]*(-41802) + word_distribution[9]*(-41945)
//				+ word_distribution[10]*(-41966) + word_distribution[11]*(-42035) + word_distribution[12]*(-41730) + word_distribution[13]*(-41973) + word_distribution[14]*(-41866)
//				+ word_distribution[15]*(-41744) + word_distribution[16]*(-41843) + word_distribution[17]*(-41949) + word_distribution[18]*(-42041) + word_distribution[19]*(-41818)
//				+ word_distribution[20]*(-41786) + word_distribution[21]*(-42194) + word_distribution[22]*(-42003) + word_distribution[23]*(-42161) + word_distribution[24]*(-41849)
//				+ word_distribution[25]*(-42364) + word_distribution[26]*(-41797) + word_distribution[27]*(-42065) + word_distribution[28]*(-41914) + word_distribution[29]*(-41848);


//		flatness_appearance = 2294.5
//				+ word_distribution[0]*(-2145.1) + word_distribution[1]*(583.7) + word_distribution[2]*(-2166.2) + word_distribution[3]*(1624.4) + word_distribution[4]*(1012)
//				+ word_distribution[5]*(-2860.2) + word_distribution[6]*(-790.6) + word_distribution[7]*(5165.1) + word_distribution[8]*(226.8) + word_distribution[9]*(-3481.1)
//				+ word_distribution[10]*(-1765.4) + word_distribution[11]*(-3412) + word_distribution[12]*(-1787.9) + word_distribution[13]*(-2698.2) + word_distribution[14]*(67.6)
//				+ word_distribution[15]*(-6578.4) + word_distribution[16]*(-2859.1) + word_distribution[17]*(-8525.5) + word_distribution[18]*(4740.1) + word_distribution[19]*(37066.1)
//				+ word_distribution[20]*(-2455.2) + word_distribution[21]*(-459) + word_distribution[22]*(8448.4) + word_distribution[23]*(-2449) + word_distribution[24]*(-1939.3)
//				+ word_distribution[25]*(-482.3) + word_distribution[26]*(-6793.7) + word_distribution[27]*(552.6) + word_distribution[28]*(-2314.5) + word_distribution[29]*(-1205.5);
//
//		if(flatness_appearance < 400.0)
//		{
//			land_distribution = 1;
//		}
//		else
//		{
//			land_distribution = 0;
//		}

		DOWNLINK_SEND_Distribution(DefaultChannel, DefaultDevice, &flatness
								, &word_distribution[0], &word_distribution[1], &word_distribution[2], &word_distribution[3], &word_distribution[4]
		                        , &word_distribution[5], &word_distribution[6], &word_distribution[7], &word_distribution[8], &word_distribution[9]
		                        , &word_distribution[10], &word_distribution[11], &word_distribution[12], &word_distribution[13], &word_distribution[14]
		                        , &word_distribution[15], &word_distribution[16], &word_distribution[17], &word_distribution[18], &word_distribution[19]
		                        , &word_distribution[20], &word_distribution[21], &word_distribution[22], &word_distribution[23], &word_distribution[24]
		                        , &word_distribution[25], &word_distribution[26], &word_distribution[27], &word_distribution[28], &word_distribution[29]);

	}

#endif

	// **********************************************************************************************************************
	// Save an image
	// **********************************************************************************************************************
	if(snapshot)
	{
		if(land_distribution)
		{
			sprintf(filename, "/data/video/safe_%d.dat", i_frame);
			saveSingleImageDataFile(frame, imgWidth, imgHeight, filename);
		}
		else
		{
			sprintf(filename, "/data/video/unsafe_%d.dat", i_frame);
			saveSingleImageDataFile(frame, imgWidth, imgHeight, filename);
		}
		snapshot = FALSE;
		i_frame ++ ;
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
//	DOWNLINK_SEND_OF_LAND_OBSTACLE(DefaultChannel, DefaultDevice, &FPS, &z_x, &z_y, &flatness, &POE_x, &POE_y,
//			&divergence, &ground_divergence, &d_heading, &d_pitch, &flow_count,
//			&V_body.x, &V_body.y, &V_body.z, &cam_h, &curr_pitch, &curr_roll, &curr_yaw,
//			&stateGetPositionEnu_i()->x, &stateGetPositionEnu_i()->y, &stateGetPositionEnu_i()->z,
//			&waypoint_ob1.x, &waypoint_ob1.y, &waypoint_ob1.z);
	DOWNLINK_SEND_OF_LAND_OBSTACLE(DefaultChannel, DefaultDevice, &FPS, &z_x, &z_y, &flatness, &flatness_appearance, &appearance_uncertainty,
			&divergence, &ground_divergence, &d_heading, &d_pitch, &flow_count,
			&V_body.x, &V_body.y, &V_body.z, &cam_h, &curr_pitch, &curr_roll, &curr_yaw,
			&stateGetPositionEnu_i()->x, &stateGetPositionEnu_i()->y, &stateGetPositionEnu_i()->z,
			&waypoint_ob1.x, &waypoint_ob1.y, &waypoint_ob1.z);
#else
	DOWNLINK_SEND_OF_LAND(DefaultChannel, DefaultDevice, &z_x, &z_y, &flatness, &POE_x, &POE_y, &divergence, &d_heading, &d_pitch, &V_body.z, &cam_h, &imu.accel.x, &imu.accel.y, &imu.accel.z);
#endif
	// rotation angle, height, obstacle position (measured size),
}

