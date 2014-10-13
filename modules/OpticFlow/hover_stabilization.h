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


#ifndef HOVER_STABILIZATION_H_
#define HOVER_STABILIZATION_H_

#include <std.h>

void init_hover_stabilization_onvision(void);
void run_hover_stabilization_onvision(void);
void run_opticflow_hover(void);

extern int32_t vision_phi_pgain;
extern int32_t vision_phi_igain;
extern int32_t vision_theta_pgain;
extern int32_t vision_theta_igain;

#endif /* HOVER_STABILIZATION_H_ */
