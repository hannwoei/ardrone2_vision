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
#include "hover_stabilization.h"

// Paparazzi Data
#include "state.h"
#include "subsystems/ins/ins_int.h"

// Vision Data
#include "video_message_structs.h"
#include "opticflow_code.h"

// Interact with navigation
#include "stabilization.h"
#include "autopilot.h"

// Downlink
#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif
#include "messages.h"
#include "subsystems/datalink/downlink.h"
#include "boards/ardrone/navdata.h"

//#ifndef VISION_PHI_PGAIN
//#define VISION_PHI_PGAIN 0
//#endif
//
//#ifndef VISION_PHI_IGAIN
//#define VISION_PHI_IGAIN 0
//#endif
//
//#ifndef VISION_THETA_PGAIN
//#define VISION_THETA_PGAIN 0
//#endif
//
//#ifndef VISION_THETA_IGAIN
//#define VISION_THETA_IGAIN 0
//#endif

/* error if some gains are negative */
#if (VISION_PHI_PGAIN < 0)      ||                   \
  (VISION_PHI_IGAIN < 0)        ||                   \
  (VISION_THETA_PGAIN < 0)      ||                   \
  (VISION_THETA_IGAIN < 0)
#error "ALL control gains have to be positive!!!"
#endif

int32_t vision_phi_pgain;
int32_t vision_phi_igain;
int32_t vision_theta_pgain;
int32_t vision_theta_igain;

struct Int32Eulers cmd_euler;

// hover stabilization
float Velx_Int;
float Vely_Int;

#define CMD_OF_SAT	350 // 40 deg = 2859.1851
unsigned char saturateX = 0, saturateY = 0;
unsigned int set_heading;
int phi0;
int theta0;
unsigned int OF_P_HOVER;
unsigned int OF_I_HOVER;

void init_hover_stabilization_onvision()
{
	INT_EULERS_ZERO(cmd_euler);

	vision_phi_pgain = VISION_PHI_PGAIN;
	vision_phi_igain = VISION_PHI_IGAIN;
	vision_theta_pgain = VISION_THETA_PGAIN;
	vision_theta_igain = VISION_THETA_IGAIN;

	set_heading = 1;
	phi0 = 0;
	theta0 = 0;
	OF_P_HOVER = 12;
	OF_I_HOVER = 6;

	Velx_Int = 0;
	Vely_Int = 0;
}

void run_hover_stabilization_onvision(void)
{

  if(autopilot_mode == AP_MODE_VISION_HOVER)
  {
	  run_opticflow_hover();
  }
  else
  {
		Velx_Int = 0;
		Vely_Int = 0;
  }

}

/////////////////////////////////////////////////////////////////////////////////////////
//
//  HELPER FUNCTIONS

/////////////////////////////////////////////////////////////////////////////////////////
//
//  HOVER FUNCTIONS

void run_opticflow_hover(void)
{
	if(saturateX==0)
	{
		Velx_Int += vision_theta_igain*Velx;
	}
	if(saturateY==0)
	{
		Vely_Int += vision_phi_igain*Vely;
	}

	if(set_heading)
	{
		cmd_euler.psi = stateGetNedToBodyEulers_i()->psi;
//		phi0 = stab_att_sp_euler.phi;
//		theta0 = stab_att_sp_euler.theta;
		set_heading = 0;
	}

	cmd_euler.phi =  (phi0 - (vision_phi_pgain*Vely + Vely_Int));
	cmd_euler.theta =  (theta0 + (vision_theta_pgain*Velx + Velx_Int));

	saturateX = 0; saturateY = 0;
	if(cmd_euler.phi<-CMD_OF_SAT){cmd_euler.phi = -CMD_OF_SAT; saturateX = 1;}
	else if(cmd_euler.phi>CMD_OF_SAT){cmd_euler.phi = CMD_OF_SAT; saturateX = 1;}
	if(cmd_euler.theta<-CMD_OF_SAT){cmd_euler.theta = -CMD_OF_SAT; saturateY = 1;}
	else if(cmd_euler.theta>CMD_OF_SAT){cmd_euler.theta = CMD_OF_SAT;saturateY = 1;}


	stabilization_attitude_set_rpy_setpoint_i(&cmd_euler);
	DOWNLINK_SEND_VISION_STABILIZATION(DefaultChannel, DefaultDevice, &Velx, &Vely, &Velx_Int, &Vely_Int, &cmd_euler.phi, &cmd_euler.theta);
}
