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
 * @file modules/avoid_vision/avoid_module.c
 * @brief 
 */


#include "avoid_module.h"
#include "avoid_nav.h"
#include "avoid_nav_transportFcns.h"

// Computervision Runs in a thread
#include "opticflow/opticflow_thread.h"
#include "opticflow/inter_thread_data.h"

// Threaded computer vision
#include <pthread.h>

// Sockets
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>

//For saving the obstacle map
#include "opticflow/visual_estimator.h"

int cv_sockets[2];

// Paparazzi Data
#include "state.h"
#include "../../firmwares/rotorcraft/navigation.h"
//#include "subsystems/abi.h"


// Downlink
#include "subsystems/datalink/downlink.h"

// Navigation Routines
#include "subsystems/navigation/waypoints.h"


struct PPRZinfo   opticflow_module_data;
struct EnuCoor_f  currentTargetWPPos;
float             currentTargetHeading;

/** height above ground level, from ABI
 * Used for scale computation, negative value means invalid.
 */
/** default sonar/agl to use in opticflow visual_estimator */

/*
#ifndef OPTICFLOW_AGL_ID
#define OPTICFLOW_AGL_ID ABI_BROADCAST
#endif
abi_event agl_ev;
static void agl_cb(uint8_t sender_id, float distance);

static void agl_cb(uint8_t sender_id __attribute__((unused)), float distance)
{
  if (distance > 0) {
    opticflow_module_data.agl = distance;
  }
}
*/
#define DEBUG_INFO(X, ...) ;

void opticflow_module_init(void)
{/*
{
    vec2d a,b,c;
    float gamma1,gamma2;

    a = vec2d_init(0.45,0);
    b = vec2d_init(0,0);
    gamma1 = -0.04;//vision_results.angle;

    gamma2 = -0.6;//vision_results.angle;
    c = vec2d_triangulate(&a,&b,gamma1,gamma2);
    printf("\n\n\nCoords: %f,%f\n\n\n\n",c.x,c.y);
    perror("bla");
}*/

  // get AGL from sonar via ABI
//  AbiBindMsgAGL(OPTICFLOW_AGL_ID, &agl_ev, agl_cb);

  // Initialize local data
//  opticflow_module_data.cnt = 0;
//  opticflow_module_data.phi = 0;
//  opticflow_module_data.theta = 0;
//  opticflow_module_data.agl = 0;

  opticflow_module_data.phi     = 0;
  opticflow_module_data.theta   = 0;
  opticflow_module_data.psi     = 0;
  opticflow_module_data.enuPosX = 0;
  opticflow_module_data.enuPosY = 0;
}

struct CVresults vision_results;
void opticflow_module_run(void)
{
  // Send Updated data to thread
//  opticflow_module_data.cnt++;
//  opticflow_module_data.phi = stateGetNedToBodyEulers_f()->phi;
//  opticflow_module_data.theta = stateGetNedToBodyEulers_f()->theta;

  opticflow_module_data.phi     = stateGetNedToBodyEulers_f()->phi;
  opticflow_module_data.theta   = stateGetNedToBodyEulers_f()->theta;

// Calculate yaw rate form GPS heading rate, not from body angle. NOTE: nav_heading is defined in NED. We use ENU, hence the minus sign
  opticflow_module_data.psi     = -ANGLE_FLOAT_OF_BFP(nav_heading);   
  opticflow_module_data.enuPosX = stateGetPositionEnu_f()->x;
  opticflow_module_data.enuPosX = stateGetPositionEnu_f()->y;
  
  // Calculate the distance between the current waypoint and the current position
  float dx = POS_FLOAT_OF_BFP(navigation_target.x) - opticflow_module_data.enuPosX;
  float dy = POS_FLOAT_OF_BFP(navigation_target.y) - opticflow_module_data.enuPosY;
  opticflow_module_data.targetDist = sqrt(dx*dx + dy*dy); 
  
  int bytes_written = write(cv_sockets[0], &opticflow_module_data, sizeof(opticflow_module_data));
  if (bytes_written != sizeof(opticflow_module_data) && errno !=4){
    printf("[module] Failed to write to socket: written = %d, error=%d, %s.\n",bytes_written, errno, strerror(errno));
  }
  else {
    DEBUG_INFO("[module] Write # %d (%d bytes)\n",opticflow_module_data.cnt, bytes_written);
  }

  // Read Latest Vision Module Results
  // Warning: if the vision runs faster than the module, you need to read multiple times
  int bytes_read = recv(cv_sockets[0], &vision_results, sizeof(vision_results), MSG_DONTWAIT);
  if (bytes_read != sizeof(vision_results)) {
    if (bytes_read != -1) {
      printf("[module] Failed to read %d bytes: CV results from socket errno=%d.\n",bytes_read, errno);
    }
  } else {
    ////////////////////////////////////////////
    // Module-Side Code
    ////////////////////////////////////////////
    DEBUG_INFO("[module] Read vision %d\n",vision_results.cnt);
    
    /*currentTargetHeading = vision_results.head_cmd;    */
    /*currentTargetWPPos.x = vision_results.WP_pos_X;*/
    /*currentTargetWPPos.y = vision_results.WP_pos_Y;*/
    avoid_nav_goto_wp();
  }
}

void opticflow_module_start(void)
{
  pthread_t computervision_thread;
  if (socketpair(AF_UNIX, SOCK_DGRAM, 0, cv_sockets) == 0) {
    ////////////////////////////////////////////
    // Thread-Side Code
    ////////////////////////////////////////////
    int rc = pthread_create(&computervision_thread, NULL, computervision_thread_main,
                            &cv_sockets[1]);
    if (rc) {
      printf("ctl_Init: Return code from pthread_create(mot_thread) is %d\n", rc);
    }
  }
  else {
    perror("Could not create socket.\n");
  }
}

void opticflow_module_stop(void)
{
  computervision_thread_request_exit();
}
bool stopMapRecording(void)
{
  closeMap();
  return false;
}
bool avoid_module_post_ahrs_init(void)
{
  init_map();
  vehicle_cache_init();
  initMapSaveFile();
  avoid_nav_init = 1;
  return true;
}
// Function to be called from the flight plan!
bool vision_avoid_update_WP(uint8_t wpID)
{ 
  avoid_nav_goto_wp();
  /*
  // Move the WP to its target location
  nav_set_waypoint_enu_f(wpID, (struct EnuCoor_f*)&currentTargetWPPos);
  
  // Set the heading of the drone
  nav_heading = ANGLE_BFP_OF_REAL(currentTargetHeading);
  
  // Go to the moved waypoint
  NavGotoWaypoint(wpID);
  */
  return true;
}
bool markArenaLimsAsWp(uint8_t wpIndex)
{
  arenaLimits.X[arenaLimits.arenaLimIndex] = WaypointX(wpIndex);
  arenaLimits.Y[arenaLimits.arenaLimIndex] = WaypointY(wpIndex);
  ++arenaLimits.arenaLimIndex;
  
  return false;
}

// Anton's navigation stuff
void avoid_nav_goto_wp(void){
    float range = vec2d_dist(&veh.xy_abs,&veh.wp_abs);
    /*printf("range %f\n",range);*/

    if(range<WP_MINDIST){
        int best;
        float q;
        plan_action(veh.gridij[0],veh.gridij[1],veh.o_disc,&best,&q);
        int new_i =veh.gridij[0]+arena.st_wp_i[best]; 
        int new_j =veh.gridij[1]+arena.st_wp_j[best]; 
        veh.gridij[0] = new_i;
        veh.gridij[1] = new_j;

        set_discrete_wp(new_i,new_j,arena.st_headings[best]);
        arena.grid_weights_exp[new_i*GRID_RES+new_j]++;
        veh.o_disc = best;
        counter_nav++;
    }
    struct EnuCoor_i target_wp;
    target_wp.x = veh.wp_abs.x*256;
    target_wp.y = veh.wp_abs.y*256;
    target_wp.z = 1;

    horizontal_mode = HORIZONTAL_MODE_WAYPOINT;
    VECT3_COPY(navigation_target, target_wp);
    NavVerticalAutoThrottleMode(RadOfDeg(0.000000));
    nav_heading = ANGLE_BFP_OF_REAL(PI/2-veh.wp_abs.o);
}
bool_t flight_check_complete(void){
    if(flight_check_counter == 1){
        printf("Flight check complete!\n");
        print2darr_float(arena.grid_weights_obs,GRID_RES,GRID_RES);
        printf("\n");
        print2darr_float(arena.grid_weights_exp,GRID_RES,GRID_RES);
        return TRUE;
    }
    else return FALSE;
}
bool_t avoid_map_init(void) {
    printf("Initializing arena\n");
    
    printf("Running init_map()...\n");
//    init_map();
//    printf("Running vehicle_cache_init()... ");
//    vehicle_cache_init();
    printf("OK\n");

#ifndef OPTI_REAL
    printf("Initializing obstacle sim... \n");
    obstacle_sim_init();
    printf("OK\n");
    float vv[10];;
    int n;

    obstacle_sim_return_angle(vv,&n);
    vv[0]=vv[0]*180/PI;
    printarr_float(vv,10);


#endif

#ifdef UDP_SHIZZLE


#endif
    
    return FALSE;
}
