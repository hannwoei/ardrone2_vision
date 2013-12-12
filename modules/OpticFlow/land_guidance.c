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

// Interact with navigation
//#include "navigation.h"
#include "stabilization.h"
#include "autopilot.h"

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

// Called once on paparazzi autopilot start
void init_land_guidance()
{
  land_guidance_data.mode = 0;
}

void run_opticflow_hover(void);
void run_opticflow_land(void);

// Called on each vision analysis result after receiving the struct
void run_land_guidance_onvision(void)
{
  // Send ALL vision data to the ground
  // TODO: optic flow telemetry
  //DOWNLINK_SEND_PAYLOAD(DefaultChannel, DefaultDevice, N_BINS, gst2ppz.obstacle_bins);

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
  // TODO:
	// change autopilot mode
	autopilot_set_mode(AP_MODE_HOVER_Z_HOLD);
	// augment controller parameters

}

void run_opticflow_land(void)
{
  // TODO: Using divergence flow for landing

}




