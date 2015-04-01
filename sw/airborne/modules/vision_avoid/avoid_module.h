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
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/computer_vision/opticflow_module.h
 * @brief optical-flow based hovering for Parrot AR.Drone 2.0
 *
 * Sensors from vertical camera and IMU of Parrot AR.Drone 2.0
 */

#ifndef OPTICFLOW_MODULE_H
#define OPTICFLOW_MODULE_H

#include "std.h"
#include "../../math/pprz_algebra_int.h"
#include "../../math/pprz_geodetic_int.h"

// Anton's Avoidance stuff
unsigned char avoid_nav_init;
struct EnuCoor_i nav_avoid_carrot;
float flight_check_counter;

bool_t avoid_map_init(void);
void avoid_nav_goto_wp(void);
bool_t flight_check(void);
bool_t flight_check_complete(void);
#define obs_avoid_init() ({avoid_map_init(); FALSE; })

// Module functions
extern void opticflow_module_init(void);
extern void opticflow_module_run(void);
extern void opticflow_module_start(void);
extern void opticflow_module_stop(void);

extern bool avoid_module_post_ahrs_init(void);
extern bool vision_avoid_update_WP(uint8_t wpID);
extern bool markArenaLimsAsWp(uint8_t wpIndex);

////////////////////////////////////////////
// Save map
extern bool stopMapRecording(void);


#endif /* OPTICFLOW_MODULE_H */
