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
int32_t vision_land_pgain;
int32_t vision_land_dgain;
int opt_trans_dx;
int opt_trans_dy;
int opt_trans_x_prev;
int opt_trans_y_prev;

// Called once on paparazzi autopilot start
void init_land_guidance()
{
  opt_trans_dx = 0;
  opt_trans_dy = 0;
  opt_trans_x_prev = 0;
  opt_trans_y_prev = 0;
  land_guidance_data.mode = 0;
  vision_pgain = VISION_PGAIN;
  vision_dgain = VISION_DGAIN;
  vision_land_pgain = VISION_LAND_PGAIN;
  vision_land_dgain = VISION_LAND_DGAIN;
}

void run_opticflow_hover(void);
void run_opticflow_land(void);

// Called on each vision analysis result after receiving the struct
void run_land_guidance_onvision(void)
{
  // Send ALL vision data to the ground
  // TODO: optic flow telemetry
//  DOWNLINK_SEND_PAYLOAD(DefaultChannel, DefaultDevice, N_BINS, gst2ppz.obstacle_bins);
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
	opt_trans_dx = opt_trans_x - opt_trans_x_prev;
	opt_trans_dy = opt_trans_y - opt_trans_y_prev;
	stab_att_sp_euler.phi = vision_dgain*opt_trans_dx + vision_pgain*opt_trans_x;
	stab_att_sp_euler.theta = vision_dgain*opt_trans_dy + vision_pgain*opt_trans_y;

	opt_trans_x_prev = opt_trans_x;
	opt_trans_y_prev = opt_trans_y;
}

void run_opticflow_land(void)
{
  // TODO: Using divergence flow for landing
  // Land if the drone is close to ground
//	if (ppz2gst.alt == 0) NavKillThrottle();
	if(div_flow > 0.0f) guidance_v_zd_sp = (int)(vision_land_pgain*div_flow*10000);
}




