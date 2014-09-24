
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
float mean_tti, median_tti, d_heading, d_pitch, divergence_error;
float z_x, z_y, three_dimensionality, POE_x, POE_y, threshold_3D_low, threshold_3D_high, land_safe, land_safe_false, div_buf_3D[30], div_avg_3D;
unsigned int mov_block_3D, div_point_3D;

// tryout
struct NedCoor_i OF_speed;
int kl;

// waypoint
#include "navigation.h"
#include "generated/flight_plan.h"
#define win_3D 300
struct EnuCoor_i waypoints_3D[win_3D];
struct EnuCoor_i waypoints_Distribution;
struct EnuCoor_i waypoints_3D_min;
float buf_3D[win_3D], avg_3D, min_3D;
unsigned int buf_point_3D, init3D, stay_waypoint_3D, land_safe_count, max_safe, active_3D;

// appearance
float **** dictionary;
int n_words, patch_size, n_samples, learned_samples, n_samples_image, filled;
float alpha;
int WORDS, skipped_frame;
int save_dictionary;
bool_t train_dictionary;
bool_t extract_distribution;

int RANDOM_SAMPLES;
float *word_distribution;
float fake_3D;
int border_width, border_height;

// snapshot
char filename[100];
int i_frame;
bool_t snapshot;
unsigned int land_distribution;
unsigned int land_distribution_count;
unsigned int max_safe_distribution;

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
	z_x = 0.0;
	z_y = 0.0;
	three_dimensionality = 0.0;
	POE_x = 0.0;
	POE_y = 0.0;
	threshold_3D_low = 0.0;
	threshold_3D_high = 0.0;
	land_safe = 0.0;
	land_safe_false = 0.0;
	mov_block_3D = 6;
	div_point_3D = 0;
	div_avg_3D = 0.0;

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
	kl = 1;

	//waypoints
	avg_3D = 0.0;
	buf_point_3D = 0;
	init3D = 0;
	stay_waypoint_3D = 0;
	land_safe_count = 0;
	max_safe = 0;
	active_3D = 0;

	// appearance

	n_words = 30;
	patch_size = 6;
	n_samples = 200000;
	learned_samples = 0;
	n_samples_image = 50; // 100: train dictionary
	filled = 0;
	alpha = 0.5;
	WORDS = 0; // 0: train a dictionary
	skipped_frame = 0; // 0:skip frame
	train_dictionary = VISION_TRAIN_DICTIONARY; // initialize false
	extract_distribution = VISION_EXTRACT_DISTRIBUTION;
	save_dictionary = 1;
	word_distribution = (float*)calloc(n_words,sizeof(float));
	fake_3D = 0.0;
	border_width = 80; // 80
	border_height = 60; //60
	i_frame = 0;
	snapshot = VISION_SNAPSHOT;
	land_distribution = 0;
	land_distribution_count = 0;
	max_safe_distribution = 0;

	RANDOM_SAMPLES = 1;

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



}

void my_plugin_run(unsigned char *frame)
{
	// save one YUV/RGB/GRAY image
//	if(kl <5)
//	{
//		saveSingleImageDataFile(frame, imgWidth, imgHeight, 0);
//		kl ++;
//	}

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

//#if USE_SONAR
//		cam_h = ins_impl.sonar_z;
//#else
//		cam_h = 1;
//		prev_cam_h = 1;
//#endif
//
//		camh_buf[buf_point_camh] = cam_h;
//		buf_point_camh = (buf_point_camh+1) %11;
//		quick_sort(camh_buf,11);
//		cam_h_med = camh_buf[6];

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

		curr_pitch = stateGetNedToBodyEulers_i()->theta*0.0139882;
		curr_roll = stateGetNedToBodyEulers_i()->phi*0.0139882;
		diff_pitch = (curr_pitch - prev_pitch);
		diff_roll = (curr_roll - prev_roll);
		prev_pitch = curr_pitch;
		prev_roll = curr_roll;
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

//	Velx = opt_angle_y_raw*cam_h_med/Fy_ARdrone*FPS;
//	Vely = -opt_angle_x_raw*cam_h_med/Fx_ARdrone*FPS;
//
//	OF_speed.x = (int) (Velx*(1<<(INT32_SPEED_FRAC)));
//	OF_speed.y = (int) (Vely*(1<<(INT32_SPEED_FRAC)));

	//stateSetSpeedNed_i(&OF_speed);

	/*
	 * comment out "stateSetSpeedNed_i(&ins_impl.ltp_speed);" in ins_int.c if optic flow velocities are used to set the speeds in the states
	 * // copy position and speed to state interface
		static void ins_ned_to_state(void) {
		  stateSetPositionNed_i(&ins_impl.ltp_pos);
		  //stateSetSpeedNed_i(&ins_impl.ltp_speed);
		  stateSetAccelNed_i(&ins_impl.ltp_accel);

		#if defined SITL && USE_NPS
		  if (nps_bypass_ins)
			sim_overwrite_ins();
		#endif
}
	 */

    // compute divergence/ TTI
	int USE_FITTING = 1;
	int USE_MEAN_3D = 0;
	int USE_MEDIAN_3D = 1;
	threshold_3D_low = 450.0;
	threshold_3D_high = 1000.0;

	if(USE_FITTING == 1)
	{
		analyseTTI(&z_x, &z_y, &three_dimensionality, &POE_x, &POE_y, &divergence, &mean_tti, &median_tti, &d_heading, &d_pitch, &divergence_error, x, y, dx, dy, n_inlier_minu, n_inlier_minv, count, imgWidth, imgHeight, &DIV_FILTER);

		if (USE_MEAN_3D == 1)
		{

			div_buf_3D[div_point_3D] = three_dimensionality;
			div_point_3D = (div_point_3D+1) %mov_block_3D; // index starts from 0 to mov_block
			div_avg_3D = 0.0;
			for (int im=0;im<mov_block_3D;im++) {
				div_avg_3D+=div_buf_3D[im];
			}
			three_dimensionality = (float) div_avg_3D/ mov_block_3D;
		}
		else if(USE_MEDIAN_3D == 1)
		{
			div_buf_3D[div_point_3D] = three_dimensionality;
			div_point_3D = (div_point_3D+1) %mov_block_3D;
			quick_sort(div_buf_3D,mov_block_3D);
			three_dimensionality = div_buf_3D[mov_block_3D/2];
		}
		else
		{

		}
		if ((three_dimensionality < threshold_3D_low))
		{
			land_safe = 1.0;
		}
		else
		{
			land_safe = 0.0;
		}
		if (three_dimensionality > threshold_3D_high)
		{
			land_safe_false = 1.0;
		}
		else
		{
			land_safe_false = 0.0;
		}

		if(count < 3)
		{
			land_safe = 0.0;
			land_safe_false = 0.0;
		}
	}


	//set the first initial 3D/ pre-defined 3D as minimum 3D
	//if((autopilot_in_flight == 1) && (stateGetPositionEnu_i()->z > POS_BFP_OF_REAL(3)) && ((abs(stateGetSpeedEnu_i()->x) > SPEED_BFP_OF_REAL(0.3)) || (abs(stateGetSpeedEnu_i()->y) > SPEED_BFP_OF_REAL(0.3)))) // new waypoints are created only when it is in flight with height > 3m

//	if(((abs(stateGetSpeedEnu_i()->x) > SPEED_BFP_OF_REAL(0.3)) || (abs(stateGetSpeedEnu_i()->y) > SPEED_BFP_OF_REAL(0.3)))) // new waypoints are created only when it is in flight with height > 3m
//	{
////		NavCopyWaypoint(WP_safe,WP_p7);
////		land_safe = 1.0;
//
//		waypoints_3D[buf_point_3D].x = stateGetPositionEnu_i()->x;
//		waypoints_3D[buf_point_3D].y = stateGetPositionEnu_i()->y;
//		waypoints_3D[buf_point_3D].z = stateGetPositionEnu_i()->z; // SONAR problem: when USE_SONAR is activated, the fusion result gives wrong height estimate
//		buf_3D[buf_point_3D] = three_dimensionality;
//		buf_point_3D = (buf_point_3D+1) %win_3D; // index starts from 0 to mov_block
//
//		for (int im=0;im<win_3D;im++) {
//			avg_3D+=buf_3D[im];
//		}
//		avg_3D = avg_3D/ win_3D;
//
//		if (init3D == 0)
//		{
//			// min_3D = avg_3D;
//			min_3D = 100.0;
//			init3D = 1;
//		}
//
//		if(avg_3D < min_3D)
//		{
//			min_3D = avg_3D;
//			nav_move_waypoint(WP_safe, &waypoints_3D[6]); // save the waypoint having the minimum 3D value
//		}
//	}
//	else
//	{
////		land_safe = 0.0;
////		NavCopyWaypoint(WP_safe,WP_p2);
//	}



	// *********************************************
	// Dictionary Training
	// *********************************************


//	if(!WORDS && skipped_frame)
	if(!WORDS && train_dictionary)
	{
		DictionaryTrainingYUV(dictionary, frame, n_words, patch_size, &learned_samples, n_samples_image, alpha, imgWidth, imgHeight, &filled);
//		printf("number of samples learnt = %d\n",learned_samples);
	}

//	skipped_frame = 1;

	if(extract_distribution && !WORDS)
	{
		//load a dictionary
		FILE *fdl;
		fdl=fopen("/data/video/VisualWords2.dat", "r");

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
		}
	}

	if(learned_samples >= n_samples && !WORDS)
	{
//		printf("Done !!!\n");
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

	if(extract_distribution)
	{
		DistributionExtraction(dictionary, frame, word_distribution, n_words, patch_size, n_samples_image, RANDOM_SAMPLES, imgWidth, imgHeight, border_width, border_height);

		fake_3D = 2294.5
				+ word_distribution[0]*(-2145.1) + word_distribution[1]*(583.7) + word_distribution[2]*(-2166.2) + word_distribution[3]*(1624.4) + word_distribution[4]*(1012)
				+ word_distribution[5]*(-2860.2) + word_distribution[6]*(-790.6) + word_distribution[7]*(5165.1) + word_distribution[8]*(226.8) + word_distribution[9]*(-3481.1)
				+ word_distribution[10]*(-1765.4) + word_distribution[11]*(-3412) + word_distribution[12]*(-1787.9) + word_distribution[13]*(-2698.2) + word_distribution[14]*(67.6)
				+ word_distribution[15]*(-6578.4) + word_distribution[16]*(-2859.1) + word_distribution[17]*(-8525.5) + word_distribution[18]*(4740.1) + word_distribution[19]*(37066.1)
				+ word_distribution[20]*(-2455.2) + word_distribution[21]*(-459) + word_distribution[22]*(8448.4) + word_distribution[23]*(-2449) + word_distribution[24]*(-1939.3)
				+ word_distribution[25]*(-482.3) + word_distribution[26]*(-6793.7) + word_distribution[27]*(552.6) + word_distribution[28]*(-2314.5) + word_distribution[29]*(-1205.5);

		if(fake_3D < 400.0)
		{
			land_distribution = 1;
		}
		else
		{
			land_distribution = 0;
		}

//		DOWNLINK_SEND_Distribution(DefaultChannel, DefaultDevice, &three_dimensionality, &fake_3D, &snapshot, &land_distribution
//																					   , &word_distribution[0], &word_distribution[1], &word_distribution[2], &word_distribution[3], &word_distribution[4]
//		                                                                               , &word_distribution[5], &word_distribution[6], &word_distribution[7], &word_distribution[8], &word_distribution[9]
//		                                                                               , &word_distribution[10], &word_distribution[11], &word_distribution[12], &word_distribution[13], &word_distribution[14]
//		                                                                               , &word_distribution[15], &word_distribution[16], &word_distribution[17], &word_distribution[18], &word_distribution[19]
//		                                                                               , &word_distribution[20], &word_distribution[21], &word_distribution[22], &word_distribution[23], &word_distribution[24]
//		                                                                               , &word_distribution[25], &word_distribution[26], &word_distribution[27], &word_distribution[28], &word_distribution[29]);

	}
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

	// *********************************************
	// move way point to the safe location
	// *********************************************

	if(extract_distribution)
	{
		if((!stay_waypoint_3D) && ((abs(stateGetSpeedEnu_i()->x) < SPEED_BFP_OF_REAL(0.1)) && (abs(stateGetSpeedEnu_i()->y) < SPEED_BFP_OF_REAL(0.1))))
		{
			if(land_distribution)
			{
				land_distribution_count++;
			}
			else
			{
				land_distribution_count = 0;
			}

			if(land_distribution_count > 120) // stay there 2 seconds
			{
				waypoints_Distribution.x = stateGetPositionEnu_i()->x;
				waypoints_Distribution.y = stateGetPositionEnu_i()->y;
				waypoints_Distribution.z = stateGetPositionEnu_i()->z;
				nav_move_waypoint(WP_safe, &waypoints_Distribution);
			}
		}
		else
		{
			land_distribution_count = 0;
		}
	}
	else
	{
		if((!stay_waypoint_3D) && ((abs(stateGetSpeedEnu_i()->x) > SPEED_BFP_OF_REAL(0.3)) || (abs(stateGetSpeedEnu_i()->y) > SPEED_BFP_OF_REAL(0.3))))
		{
			active_3D = 1;
			if(land_safe == 1)
			{
				waypoints_3D[buf_point_3D].x = stateGetPositionEnu_i()->x;
				waypoints_3D[buf_point_3D].y = stateGetPositionEnu_i()->y;
				waypoints_3D[buf_point_3D].z = stateGetPositionEnu_i()->z; // SONAR problem?: when USE_SONAR is activated, the fusion result gives wrong height estimate
				buf_point_3D = (buf_point_3D+1) %win_3D; // index starts from 0 to mov_block

				land_safe_count ++;
			}
			else
			{
				if(land_safe_count > max_safe) //land with the largest possibility of safe region
				{
					max_safe = land_safe_count;
					if (buf_point_3D > 2) nav_move_waypoint(WP_safe, &waypoints_3D[buf_point_3D/2]); // save the waypoint having the minimum 3D value
				}
				land_safe_count = 0;
				buf_point_3D = 0;
			}
		}
		else
		{
			active_3D = 0;
			if(land_safe_count > max_safe)
			{
				max_safe = land_safe_count;
				if (buf_point_3D > 2) nav_move_waypoint(WP_safe, &waypoints_3D[buf_point_3D/2]); // save the waypoint having the minimum 3D value
			}
			land_safe_count = 0;
			buf_point_3D = 0;
		}
	}


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

//	DOWNLINK_SEND_OPTIC_FLOW(DefaultChannel, DefaultDevice, &FPS, &opt_angle_x_raw, &opt_angle_y_raw, &stateGetPositionEnu_i()->x, &stateGetPositionEnu_i()->y, &stateGetPositionEnu_i()->z, &Vely, &diff_roll, &diff_pitch, &cam_h_med, &count, &ins_impl.ltp_pos.z, &divergence, &ins_impl.ltp_speed.z, &land_safe, &land_safe_false, &d_heading, &d_pitch, &z_x, &z_y, &ins_impl.ltp_accel.z, n_inlier_minu, n_inlier_minv, &stabilization_cmd[COMMAND_THRUST], &three_dimensionality);
//	DOWNLINK_SEND_OPTIC_FLOW(DefaultChannel, DefaultDevice, &FPS, &opt_angle_x_raw, &opt_angle_y_raw, &stateGetPositionEnu_i()->x, &stateGetPositionEnu_i()->y, &stateGetPositionEnu_i()->z, &Vely, &diff_roll, &diff_pitch, &cam_h_med, &count, &ins_impl.ltp_pos.z, &divergence, &ins_impl.ltp_speed.z, &land_safe, &land_safe_false, &d_heading, &d_pitch, &z_x, &z_y, &ins_impl.ltp_accel.z, n_inlier_minu, n_inlier_minv, &stay_waypoint_3D, &three_dimensionality);
//	DOWNLINK_SEND_OPTIC_FLOW(DefaultChannel, DefaultDevice, &FPS, &opt_angle_x_raw, &opt_angle_y_raw, &stateGetSpeedEnu_i()->x, &stateGetSpeedEnu_i()->y, &stateGetSpeedEnu_i()->z, &Vely, &diff_roll, &diff_pitch, &cam_h_med, &count, &ins_impl.ltp_pos.z, &divergence, &ins_impl.ltp_speed.z, &land_safe, &land_safe_false, &d_heading, &d_pitch, &z_x, &z_y, &ins_impl.ltp_accel.z, n_inlier_minu, n_inlier_minv, &max_safe, &three_dimensionality);
	DOWNLINK_SEND_LAND_SAFE(DefaultChannel, DefaultDevice, &FPS, &opt_angle_x_raw, &opt_angle_y_raw, &stateGetPositionEnu_i()->x, &stateGetPositionEnu_i()->y, &stateGetPositionEnu_i()->z,&stateGetSpeedEnu_i()->x, &stateGetSpeedEnu_i()->y, &stateGetSpeedEnu_i()->z, &ins_impl.ltp_pos.z, &diff_roll, &diff_pitch, &count, &divergence, &land_safe, &fake_3D, &d_pitch, &z_x, &z_y, n_inlier_minu, n_inlier_minv, &max_safe, &active_3D, &three_dimensionality);

}

