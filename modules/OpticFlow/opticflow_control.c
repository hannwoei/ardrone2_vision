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
 * @file paparazzi/sw/ext/ardrone2_vision/modules/OpticFlow/hover_stabilization.c
 * @brief optical-flow based hovering for Parrot AR.Drone 2.0
 *
 * Sensors from vertical camera and IMU of Parrot AR.Drone 2.0
 */

// Own Header
#include "opticflow_control.h"

// Vision Data
#include "opticflow_code.h"

// Stabilization
//#include "stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "autopilot.h"

// Landing
#include "firmwares/rotorcraft/guidance/guidance_v.h"

// Downlink
#include "subsystems/datalink/downlink.h"

// Controller Gains
/* error if some gains are negative */
#if (VISION_PHI_PGAIN < 0)      ||                   \
  (VISION_PHI_IGAIN < 0)        ||                   \
  (VISION_THETA_PGAIN < 0)      ||                   \
  (VISION_THETA_IGAIN < 0)
#error "ALL control gains have to be positive!!!"
#endif
bool activate_opticflow_hover;
int32_t vision_phi_pgain;
int32_t vision_phi_igain;
int32_t vision_theta_pgain;
int32_t vision_theta_igain;

// Controller Commands
struct Int32Eulers cmd_euler;

// Hover Stabilization
float Velx_Int;
float Vely_Int;

// Land with divergence
float div_err;
float vision_div_const;
float vision_land_pgain;
int land_guidance_threshold;

#define CMD_OF_SAT	1500 // 40 deg = 2859.1851
unsigned char saturateX = 0, saturateY = 0;
unsigned int set_heading;

#ifndef VISION_HOVER
#define VISION_HOVER 0
#endif

#ifndef VISION_PHI_PGAIN
#define VISION_PHI_PGAIN 500
#endif

#ifndef VISION_PHI_IGAIN
#define VISION_PHI_IGAIN 10
#endif

#ifndef VISION_THETA_PGAIN
#define VISION_THETA_PGAIN 500
#endif

#ifndef VISION_THETA_IGAIN
#define VISION_THETA_IGAIN 10
#endif

#ifndef VISION_DIV_CONST
#define VISION_DIV_CONST 0
#endif

#ifndef VISION_LAND_PGAIN
#define VISION_LAND_PGAIN 0
#endif

void init_hover_stabilization_onvision()
{
	INT_EULERS_ZERO(cmd_euler);

	activate_opticflow_hover = VISION_HOVER;
	vision_phi_pgain = VISION_PHI_PGAIN;
	vision_phi_igain = VISION_PHI_IGAIN;
	vision_theta_pgain = VISION_THETA_PGAIN;
	vision_theta_igain = VISION_THETA_IGAIN;

	set_heading = 1;

	Velx_Int = 0;
	Vely_Int = 0;
}

void init_land_guidance_onvision()
{
	div_err = 0.0;
	vision_div_const = VISION_DIV_CONST;
	vision_land_pgain = VISION_LAND_PGAIN;
	land_guidance_threshold = (int) (1.0/0.0000019);
}

void run_hover_stabilization_onvision(void)
{
	if(autopilot_mode == AP_MODE_VISION_HOVER)
	{
		run_opticflow_hover();
	}
	else if(autopilot_mode == AP_MODE_HOVER_CLIMB)
	{
		run_opticflow_land();
	}
	else
	{
		Velx_Int = 0;
		Vely_Int = 0;
	}
}

void run_opticflow_hover(void)
{
	if(saturateX==0)
	{
		if(activate_opticflow_hover==TRUE)
		{
			Velx_Int += vision_theta_igain*Velx;
		}
		else
		{
			Velx_Int += vision_theta_igain*V_body.x;
		}
	}
	if(saturateY==0)
	{
		if(activate_opticflow_hover==TRUE)
		{
			Vely_Int += vision_phi_igain*Vely;
		}
		else
		{
			Vely_Int += vision_phi_igain*V_body.y;
		}
	}

	if(set_heading)
	{
		cmd_euler.psi = stateGetNedToBodyEulers_i()->psi;
		set_heading = 0;
	}

	if(activate_opticflow_hover==TRUE)
	{
		cmd_euler.phi =  - (vision_phi_pgain*Vely + Vely_Int);
		cmd_euler.theta =  (vision_theta_pgain*Velx + Velx_Int);
	}
	else
	{
		cmd_euler.phi =  - (vision_phi_pgain*V_body.y + Vely_Int);
		cmd_euler.theta =  (vision_theta_pgain*V_body.x + Velx_Int);
	}

	saturateX = 0; saturateY = 0;
	if(cmd_euler.phi<-CMD_OF_SAT){cmd_euler.phi = -CMD_OF_SAT; saturateX = 1;}
	else if(cmd_euler.phi>CMD_OF_SAT){cmd_euler.phi = CMD_OF_SAT; saturateX = 1;}
	if(cmd_euler.theta<-CMD_OF_SAT){cmd_euler.theta = -CMD_OF_SAT; saturateY = 1;}
	else if(cmd_euler.theta>CMD_OF_SAT){cmd_euler.theta = CMD_OF_SAT;saturateY = 1;}

	stabilization_attitude_set_rpy_setpoint_i(&cmd_euler);
	DOWNLINK_SEND_VISION_STABILIZATION(DefaultChannel, DefaultDevice, &Velx, &Vely, &Velx_Int, &Vely_Int, &cmd_euler.phi, &cmd_euler.theta);
}

void run_opticflow_land(void)
{

	int USE_GROUND_DIVERGENCE = 0;

	if(USE_GROUND_DIVERGENCE == 1)
	{

	}
	else
	{
		div_err = vision_div_const-divergence;
	}

	guidance_v_zd_sp = guidance_v_zd_sp + ((int)(vision_land_pgain*div_err/0.0000019));

	if(guidance_v_zd_sp > land_guidance_threshold) guidance_v_zd_sp = land_guidance_threshold;
	if(guidance_v_zd_sp < -land_guidance_threshold) guidance_v_zd_sp = -land_guidance_threshold;

	DOWNLINK_SEND_VISION_LAND(DefaultChannel, DefaultDevice, &guidance_v_zd_sp, &vision_div_const, &div_err, &vision_land_pgain, &ground_divergence);
}
