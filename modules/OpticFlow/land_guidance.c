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
#if (VISION_PGAIN < 0)      ||                   \
  (VISION_DGAIN < 0)        ||                   \
  (VISION_LAND_PGAIN < 0)   ||                   \
  (VISION_LAND_DGAIN < 0)
#error "ALL control gains have to be positive!!!"
#endif

int32_t vision_pgain;
int32_t vision_dgain;
float vision_land_pgain;
float vision_land_dgain;
int OF_x;
int OF_y;
int OF_dx;
int OF_dy;
int OF_x_prev;
int OF_y_prev;

int USE_TRANSLATIONAL;

float vision_div_const;
float div_err;

// Called once on paparazzi autopilot start
void init_land_guidance()
{
  OF_x = 0;
  OF_y = 0;
  OF_x_prev = 0;
  OF_y_prev = 0;
  OF_dx = 0;
  OF_dy = 0;

  div_err = 0.0;

  land_guidance_data.mode = 0;
  vision_pgain = VISION_PGAIN;
  vision_dgain = VISION_DGAIN;
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

	/*
  if(autopilot_mode == AP_MODE_ATTITUDE_Z_HOLD)
  {
	  land_guidance_data.mode = 1;
  }
  else if (autopilot_mode == AP_MODE_HOVER_CLIMB)
  //or AP_MODE_ATTITUDE_CLIMB / AP_MODE_HOVER_CLIMB
  {
	  land_guidance_data.mode = 2;
  }
  else
  {
	  land_guidance_data.mode = 0;
  }

  switch (land_guidance_data.mode)
  {
  case 1:     // hover using optic flow
	  run_opticflow_hover();
    break;
  case 2:	  // land using optic flow
	  run_opticflow_land();
    break;
  default:    // do nothing
    break;
  }
*/
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
	// augment controller parameters
	USE_TRANSLATIONAL = 0;
	if (USE_TRANSLATIONAL == 0)
	{
		  OF_x = opt_angle_x_raw;
		  OF_y = opt_angle_y_raw;
	}
	else
	{
		  OF_x = opt_trans_x;
		  OF_y = opt_trans_y;
	}
	  OF_dx = OF_x - OF_x_prev;
	  OF_dy = OF_y - OF_y_prev;
	  stab_att_sp_euler.phi = vision_dgain*OF_dx/100 + vision_pgain*OF_x/100;
	  stab_att_sp_euler.theta = vision_dgain*OF_dy/100 + vision_pgain*OF_y/100;
	  OF_x_prev = OF_x;
	  OF_y_prev = OF_y;
}

void run_opticflow_land(void)
{
  // TODO: Using divergence flow for landing
  // Land if the drone is close to ground
//	if (ppz2gst.alt == 0) NavKillThrottle();

	div_err = vision_div_const-divergence;

//	if(divergence > 0.0f)
//	{
		//guidance_v_zd_sp = (vision_land_pgain*abs((int)((div_rate_const-divergence)/0.0000019)));
//	div_err = div_rate_const-divergence;

	guidance_v_zd_sp = guidance_v_zd_sp + ((int)(vision_land_pgain*div_err/0.0000019));

	if(guidance_v_zd_sp > (int)(1.0/0.0000019)) guidance_v_zd_sp = (int)(1.0/0.0000019);
	if(guidance_v_zd_sp < -(int)(1.0/0.0000019)) guidance_v_zd_sp = -(int)(1.0/0.0000019);
//	}
//	else
//	{
		//guidance_v_zd_sp = 0; //keep the same as previous control input
//	}

	DOWNLINK_SEND_VISION_CONTROL(DefaultChannel, DefaultDevice, &guidance_v_zd_sp, &vision_div_const, &div_err, &vision_land_pgain);
}




