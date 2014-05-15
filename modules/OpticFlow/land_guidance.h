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


#ifndef LAND_GUIDANCE_H_
#define LAND_GUIDANCE_H_

#include <std.h>

struct LandGuidanceStruct {
  uint8_t mode; ///< 0 = nothing, 1 =  hover, 2 = land
};

/** global VIDEO state */
extern struct LandGuidanceStruct land_guidance_data;


void init_land_guidance(void);
void run_land_guidance_onvision(void);

extern int32_t vision_pgain;
extern int32_t vision_dgain;

extern float vision_land_pgain;
extern float vision_land_dgain;
extern float vision_div_const;

#endif /* LAND_GUIDANCE_H_ */
