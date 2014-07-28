/*
 * Copyright (C) 2012-2013
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

// Own Header
#include "land_guidance.h"

// Paparazzi Data
#include "state.h"
#include "subsystems/ins/ins_int.h"

// Vision Data
#include "video_message_structs.h"
#include "opticflow_code.h"

// Interact with navigation
//#include "navigation.h"
#include "stabilization.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "autopilot.h"
#include "subsystems/nav.h"

// Know waypoint numbers and blocks
//#include "generated/flight_plan.h"


// Downlink
#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif
#include "messages.h"
#include "subsystems/datalink/downlink.h"

#include "boards/ardrone/navdata.h"

struct LandGuidanceStruct land_guidance_data;

/* error if some gains are negative */
#if (VISION_PHI_PGAIN < 0)      ||                   \
  (VISION_PHI_DGAIN < 0)        ||                   \
  (VISION_THETA_PGAIN < 0)      ||                   \
  (VISION_THETA_DGAIN < 0)/*        ||                   \
  (VISION_LAND_PGAIN < 0)   ||                   \
  (VISION_LAND_DGAIN < 0)*/
#error "ALL control gains have to be positive!!!"
#endif

int32_t vision_phi_pgain;
int32_t vision_phi_dgain;
int32_t vision_theta_pgain;
int32_t vision_theta_dgain;
float vision_land_pgain;
float vision_land_dgain;
float OF_dx;
float OF_dy;
float OF_ddx;
float OF_ddy;
float OF_dx_prev;
float OF_dy_prev;
float cam_h;
float X_of;
float Y_of;
float dt_FPS;

struct Int32Eulers cmd_euler;

int USE_TRANSLATIONAL;

float vision_div_const;
float div_err;
float div_err_prev;
int off_count;
float ground_divergence;

struct NedCoor_f ned_speed_opticflow;

// Called once on paparazzi autopilot start
void init_land_guidance()
{
  OF_dx = 0.0;
  OF_dy = 0.0;
  OF_dx_prev = 0.0;
  OF_dy_prev = 0.0;
  OF_ddx = 0.0;
  OF_ddy = 0.0;

  cam_h = 0.0;

  X_of = 0.0;
  Y_of = 0.0;
  dt_FPS = 0.0;

  INT_EULERS_ZERO(cmd_euler);

  div_err = 0.0;
  div_err_prev = 0.0;
  off_count = 0;
  ground_divergence = 0.0;

  land_guidance_data.mode = 0;
  vision_phi_pgain = VISION_PHI_PGAIN;
  vision_phi_dgain = VISION_PHI_DGAIN;
  vision_theta_pgain = VISION_THETA_PGAIN;
  vision_theta_dgain = VISION_THETA_DGAIN;
  vision_land_pgain = VISION_LAND_PGAIN;
  vision_land_dgain = VISION_LAND_DGAIN;
  vision_div_const = VISION_DIV_CONST;
  USE_TRANSLATIONAL = 0;
}

void run_opticflow_hover(void);
void run_opticflow_land(void);

// Called on each vision analysis result after receiving the struct
void run_land_guidance_onvision(void)
{

  if(autopilot_mode == AP_MODE_HOVER_CLIMB)
  {
	  run_opticflow_land();
  }
  else if(autopilot_mode == AP_MODE_HOVER_Z_HOLD)
  {
	  run_opticflow_hover();
  }
}

/////////////////////////////////////////////////////////////////////////////////////////
//
//  HELPER FUNCTIONS

/////////////////////////////////////////////////////////////////////////////////////////
//
//  HOVERING AND LANDING FUNCTIONS

void run_opticflow_hover(void)
{
  // TODO:Somewhere in the loop change the the autopilot mode to attitude mode
	// change autopilot mode
//	autopilot_set_mode(AP_MODE_ATTITUDE_Z_HOLD); //AP_MODE_HOVER_Z_HOLD or AP_MODE_ATTITUDE_Z_HOLD
//	autopilot_mode = AP_MODE_ATTITUDE_Z_HOLD;
//	autopilot_mode_auto2 = AP_MODE_ATTITUDE_Z_HOLD;

#if USE_SONAR
	cam_h = ins_impl.sonar_z;
#else
	cam_h = 1;
#endif
	// augment controller parameters
	USE_TRANSLATIONAL = 1;
	if (USE_TRANSLATIONAL == 0)
	{
		  OF_dx = (float)opt_angle_x_raw;
		  OF_dy = (float)opt_angle_y_raw;
	}
	else
	{
		  OF_dx = Velx;
		  OF_dy = Vely;
/*
		  if (FPS == 0)
		  {
			  dt_FPS = 0;
		  }
		  else
		  {
			  dt_FPS = 1.0/FPS;
		  }

		  X_of = X_of + Velx*dt_FPS;
		  Y_of = Y_of + Vely*dt_FPS;
*/
	}
	ned_speed_opticflow.x = OF_dx;
	ned_speed_opticflow.y = OF_dy;
	ned_speed_opticflow.z = -Velz;
	stateSetSpeedNed_f(&ned_speed_opticflow);

	  OF_ddx = OF_dx - OF_dx_prev;
	  OF_ddy = OF_dy - OF_dy_prev;
//	  stab_att_sp_euler.phi = vision_dgain*OF_ddx/100 + vision_pgain*OF_dx/100;
//	  stab_att_sp_euler.theta = vision_dgain*OF_ddy/100 + vision_pgain*OF_dy/100;

//	  cmd_euler.phi = -vision_phi_pgain*(OF_dy*(1<<(INT32_SPEED_FRAC-INT32_POS_FRAC)));// + vision_phi_dgain*OF_ddx;
//	  cmd_euler.theta = vision_theta_pgain*(OF_dx*(1<<(INT32_SPEED_FRAC-INT32_POS_FRAC)));// + vision_theta_dgain*OF_ddy;

	  OF_dx_prev = OF_dx;
	  OF_dy_prev = OF_dy;

//	  stabilization_attitude_set_rpy_setpoint_i(&cmd_euler);

	 DOWNLINK_SEND_VISION_STABILIZATION(DefaultChannel, DefaultDevice, &stateGetPositionEnu_i()->z, &ins_impl.baro_z, &cam_h, &stateGetNedToBodyEulers_i()->phi, &stateGetNedToBodyEulers_i()->theta, &stateGetNedToBodyEulers_i()->psi, &cmd_euler.phi, &cmd_euler.theta, &stateGetSpeedNed_f()->x, &stateGetSpeedNed_f()->y, &stateGetSpeedNed_f()->z);
}

void run_opticflow_land(void)
{
  // TODO: Using divergence flow for landing
  // Land if the drone is close to ground
//	if (ppz2gst.alt == 0) NavKillThrottle();
	int USE_GROUND_DIVERGENCE = 0;

	if(USE_GROUND_DIVERGENCE == 1)
	{
		if(ins_impl.ltp_pos.z == 0)
		{
			ground_divergence = 0.0;
			div_err = 0.0;
		}
		else
		{
			ground_divergence = -50000.0*((float) ins_impl.ltp_speed.z)*0.0000019/((float)ins_impl.ltp_pos.z)*0.0039063;
			div_err = vision_div_const-ground_divergence;
		}

	}
	else
	{
		div_err = vision_div_const-divergence;
	}

	guidance_v_zd_sp = guidance_v_zd_sp + ((int)(vision_land_pgain*div_err/0.0000019));

	if(guidance_v_zd_sp > (int)(1.0/0.0000019)) guidance_v_zd_sp = (int)(1.0/0.0000019);
	if(guidance_v_zd_sp < -(int)(1.0/0.0000019)) guidance_v_zd_sp = -(int)(1.0/0.0000019);


	if(div_err == div_err_prev)
	{
		off_count++;
	}
	else
	{
		off_count = 0;
	}
	// Land if the drone is close to ground after 3 seconds touchdown assuming 60fps
	if(off_count > 180) NavKillThrottle();
	div_err_prev = div_err;
	DOWNLINK_SEND_VISION_LAND(DefaultChannel, DefaultDevice, &guidance_v_zd_sp, &vision_div_const, &div_err, &vision_land_pgain, &ground_divergence);
}




