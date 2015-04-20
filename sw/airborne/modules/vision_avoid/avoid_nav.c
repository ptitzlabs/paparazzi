#include "avoid_nav.h"
#include "state.h"
#include "generated/flight_plan.h"
#include "../subsystems/navigation/waypoints.h"

#include "avoid_nav_transportFcns.h"

#include <time.h>
#include <stdio.h>
/*Switch between the code written for nps and ap*/
#define OBS_AVOID_ON
/*Enable debug messages sent via UDP*/
#ifdef AVOID_NAV_DEBUG
float nav_debug_downlink[AVOID_NAV_DEBUG_DOWNLINK_SIZE];
#endif

unsigned char avoid_nav_init = 0;

void sayhello(void) { printf("hello!\n"); }

/*Here various vector algebra functions are defined, mainly used to switch
 * between ENU and gridworld coordinate frames
 * */
/*Absolute value of a vector*/
void vec2d_abs(vec2d* a) { a->mag = sqrt(pow(a->x, 2) + pow(a->y, 2)); }
/*Pre-calculation of sine and cosine values of a vector*/
void vec2d_cs(vec2d* a) {
    vec2d_abs(a);
    a->c1 = a->x / a->mag;
    a->s1 = a->y / a->mag;
}
/*Vector quarternion calculations in 2d*/
void vec2d_q(vec2d* a) {
    vec2d_cs(a);
    a->q.w = sqrt(2 + 2 * a->c1) / 2;
    a->q.y = a->s1 / 2 / a->q.w;
}
/*Return angle between a vector and x axis from -PI to +PI*/
void vec2d_a_q(vec2d* a) {
    vec2d_q(a);
    a->angle = atan2(2 * a->q.y * a->q.w, 1 - 2 * pow(a->q.y, 2));
    if (isnan(a->angle)) a->angle = PI;
}

/*Initialize a 2d vector specifying just the coordinate*/
vec2d vec2d_init(float x, float y) {
    vec2d v;
    v.x = x;
    v.y = y;
    v.o = 0;
    return v;
}

/*Initialize a 2d vector, specifying the coordinate and  an orientation*/
vec2d vec2d_init_o(float x, float y, float o) {
    vec2d v;
    v.x = x;
    v.y = y;
    v.o = o;
    return v;
}
/*Set vector x and y values */
void vec2d_set(vec2d* v, float x, float y) {
    v->x = x;
    v->y = y;
}
/*Set vector orientation*/
void vec2d_set_o(vec2d* v, float x, float y, float o) {
    v->x = x;
    v->y = y;
    v->o = o;
}
/*Convert a 2d array to vector*/
vec2d arr2vec2d(float* arrxy) {
    vec2d v;
    v.x = arrxy[0];
    v.y = arrxy[1];
    return v;
}
/*Add two vectors*/
vec2d vec2d_add(vec2d* a, vec2d* b) {
    vec2d v;
    v.x = a->x + b->x;
    v.y = a->y + b->y;
    return v;
}
/*Substract one vector from another*/
vec2d vec2d_sub(vec2d* a, vec2d* b) {
    vec2d v;
    v.x = b->x - a->x;
    v.y = b->y - a->y;
    return v;
}
/*Dot product of two vectors*/
float vec2d_dot(vec2d* a, vec2d* b) { return (a->x * b->x + a->y * b->y); }

/*Simple way to get an angle between 2 vectors, always positive*/
float vec2d_ang(vec2d* a, vec2d* b) {
    vec2d_abs(a);
    vec2d_abs(b);
    float dot = vec2d_dot(a, b);
    return acos(dot / a->mag / b->mag);
}
/*Finding an angle between two vectors using quarternions, with a sign*/
float vec2d_ang_q(vec2d* a, vec2d* b) {
    /*vec2d_cs(a);*/
    /*vec2d_cs(b);*/
    vec2d_a_q(a);
    vec2d_a_q(b);
    return b->angle - a->angle;
}
/*Normalizing a vector into a unit vector*/
vec2d vec2d_norm(vec2d a) {
    vec2d_abs(&a);
    a.x = a.x / a.mag;
    a.y = a.y / a.mag;
    return a;
}
/*Convert the vector from ENU coordinate into gridworld coordinate*/
vec2d hme2grd(vec2d* a) {
    vec2d v;
    vec2d out;
    v = vec2d_add(a, &xy_grd.offset);
    out.x = vec2d_dot(&v, &xy_grd.x);
    out.y = vec2d_dot(&v, &xy_grd.y);
    return out;
}
/*Convert the vector from gridworld coordinate into ENU coordinate*/
vec2d grd2hme(vec2d* a) {
    vec2d v;
    vec2d out;
    v = vec2d_add(a, &xy_hme.offset);
    out.x = vec2d_dot(&v, &xy_hme.x);
    out.y = vec2d_dot(&v, &xy_hme.y);
    return out;
}
/*Convert the vector from gridworld coordinate into ENU coordinate and also
 * calculate orientation change*/
vec2d grd2hme_o(vec2d* a) {
    vec2d v = grd2hme(a);
    v.o = a->o + eta;
    return v;
}
/*Convert the vector from ENU coordinate into gridworld coordinate and also
 * calculate orientation change*/
vec2d hme2grd_o(vec2d* a) {
    vec2d v = hme2grd(a);
    v.o = a->o - eta;
    /*printf("a.o:%f,v.o:%f,v.o-eta:%f\n",a->o,v.o,v.o-eta);*/
    return v;
}
/*Find the absolute distance between two vectors*/
float vec2d_dist(vec2d* a, vec2d* b) {
    vec2d tmp = vec2d_sub(a, b);
    vec2d_abs(&tmp);
    return tmp.mag;
}

/*Triangulating between two vector locations, using the observed object
 * angle-position within the field of view.*/
vec2d vec2d_triangulate(vec2d* a, vec2d* b, float gamma1, float gamma2) {
    /*Pre-calculating sines, cosines and products so the calculation is done
     * faster*/
    float c1 = cos(gamma1);
    float s1 = sin(gamma1);
    float c2 = cos(gamma2);
    float s2 = sin(gamma2);
    float c1c2 = c1 * c2;
    float c1s2 = c1 * s2;
    float c2s1 = c2 * s1;
    float s1s2 = s1 * s2;
    float det = 1 / (c1s2 - c2s1);
    /*Solving for the new vector*/
    vec2d v;
    v.x = det * (c1c2 * a->y - c1c2 * b->y - c2s1 * a->x + c1s2 * b->x);
    v.y = det * (c1s2 * a->y - c2s1 * b->y - s1s2 * a->x + s1s2 * b->x);
    return v;
}

/*Here various debug and output messages are specified*/

/*Print out an array of floats*/
void printarr_float(float a[], int n) {
    int i;
    printf("[");
    for (i = 0; i < n; i++) {
        printf("%f, ", a[i]);
    }
    printf("]\n");
}
/*Print out an array of angles, converting them to degrees*/
void printarr_float_angles(float a[], int n) {
    int i;
    printf("[");
    for (i = 0; i < n; i++) {
        printf("%f, ", a[i] * 180 / PI);
    }
    printf("]\n");
}
/*Print out an array of integers*/
void printarr_int(int a[], int n) {
    int i;
    printf("[");
    for (i = 0; i < n; i++) {
        printf("%i, ", a[i]);
    }
    printf("]\n");
}
/*Print out a two-dimensional array of floats*/
void print2darr_float(float* a, int n, int m) {
    int i;
    int j;

    for (i = 0; i < n; i++) {
        printf("[");
        for (j = 0; j < m; j++) {
            printf("%.2f", a[i * m + j]);
            if (j < m - 1) {
                printf(", ");
            }
        }
        printf("]\n");
    }
}
/*Print out a status message about performance of point matcher */
void arena_report(void) {
    printf("Track report\npoints matched: %i\nmatches:\n", arena.n_matches);
    int i;
    for (i = 0; i < arena.n_matches; i++) {
        printf("p%i(%f) to p%i(%f)\n", arena.matches_s[i],
               arena.angles_s[arena.matches_s[i]], arena.matches_d[i],
               arena.angles_d[arena.matches_d[i]]);
    }
    printf("dropped points:\n");
    for (i = 0; i < arena.n_drops; i++) printf("%i\n", arena.drop[i]);
    printf("new points:\n");
    for (i = 0; i < arena.n_new; i++) printf("%i\n", arena.new[i]);
}

/*Print out info about a 2d vector*/
void print_vec2d(vec2d v) {
    printf("v: [%f,%f], orient:%f\n", v.x, v.y, v.o * 180 / PI);
}

/*Here point matching functions are defined.*/
/*Swap two integer values within an array*/
void sw_arr_int(int* a, int s, int d) {
    int tmp = a[d];
    a[d] = a[s];
    a[s] = tmp;
}
/*Swap two float values within an array*/
void sw_arr_float(float* a, int s, int d) {
    float tmp = a[d];
    a[d] = a[s];
    a[s] = tmp;
}

/*Match visual reference points between two frames*/
void angle_matcher(void) {
    /*Number of currently tracked points and incoming points*/
    int dn = arena.dn;
    int sn = arena.sn;

    /*Quit if nothing is currently tracked and no data coming in*/
    if (dn == 0 || sn == 0) return;

    /*Initializing the matrices to keep track of distance between the points and
     * their location in source and destination arrays*/
    float D[sn * dn];
    int dn_i[dn];
    int sn_j[sn];

    int i = 0;
    int j = 0;

    /*Filter out NAN values from the data to be matched and fill up the
     * placeholder arrays. This is necessary because the source data points coud
     * be found at any location within the source array due to continous
     * adding/dropping */
    while (j < sn) {
        if (!isnan(arena.angles_s[i])) {
            sn_j[j] = i;
            j++;
        }
        i++;
    }

    i = 0;
    j = 0;

    while (j < dn) {
        if (!isnan(arena.angles_d[i])) {
            dn_i[j] = i;
            j++;
        }
        i++;
    }
    /*Calculate the distances between points in the source array and the
     * destination array*/
    for (i = 0; i < dn; i++) {
        for (j = 0; j < sn; j++) {
            D[i * sn + j] =
                fabs(arena.angles_s[sn_j[j]] - arena.angles_d[dn_i[i]]);
        }
    }

    float tmp = INFINITY;
    float tmp2 = INFINITY;
    /*Find the pivot distance location. It's simply the smallest distance
     * between
     * two points found within the array. */
    int pivot[] = {0, 0};
    for (i = 0; i < dn; i++) {
        for (j = 0; j < sn; j++) {
            if (D[i * sn + j] < tmp) {
                pivot[0] = i;
                pivot[1] = j;
                tmp = D[i * sn + j];
            }
        }
    }
    /*Original index values of the point locations within source and destination
     * arrays*/
    int i_index[dn];
    int j_index[sn];
    for (i = 0; i < dn; i++) i_index[i] = i;
    for (i = 0; i < sn; i++) j_index[i] = i;
    /*Swapping the rows and colum indexes for the pivot found previously to
     * place
     * it at the beginning of the distance array*/
    sw_arr_int(i_index, 0, pivot[0]);
    sw_arr_int(j_index, 0, pivot[1]);

    /*Calculating the number of potential matches by taking the minimum of
     * source/destination points*/
    int n = (dn < sn) ? dn : sn;

    int k;

    /*Iterating through the distance values array by moving the pivot along the
     * diagonal and finding the minimum distance values across rows/columns.
     * This
     * sorts the distance values between potential matches in an ascending
     * order.
     * A better way to do this would be to use a simplex method, but this works
     * as
     * well.*/
    for (k = 0; k < n; k++) {
        /*Iterating across the diagonal*/
        tmp = D[i_index[k] * sn + j_index[k]];
        /*Check if current pivot is minumum across the column, swap the rows if
         * a
         * smaller distance value is found*/
        for (i = k; i < dn; i++) {
            tmp2 = D[i_index[i] * sn + j_index[k]];
            if (tmp2 < tmp) {
                sw_arr_int(i_index, i, k);
                tmp = tmp2;
            }
        }
        /*Check if current pivot is minimum across the row, swap the columns if
         * a
         * smaller distance is found*/
        for (i = k; i < sn; i++) {
            tmp2 = D[i_index[k] * sn + j_index[i]];
            if (tmp2 < tmp) {
                sw_arr_int(j_index, i, k);
                tmp = tmp2;
            }
        }
    }
    /*Fill up a new distance array that has the diagonal values in an ascending
     * order*/
    float D_new[sn * dn];
    for (i = 0; i < dn; i++) {
        for (j = 0; j < sn; j++) {
            D_new[i * sn + j] = D[i_index[i] * sn + j_index[j]];
        }
    }

    /*Find out how many potential drops there are*/
    int m = (dn > sn) ? dn : sn;
    int n_matches = 0;

    /*Count the matching points. TRACKING_LIMIT defines the maximum distance
     * between two features between two
     * consecutive frames. */
    for (i = 0; i < n; i++) {
        if (D_new[i * sn + i] < TRACKING_LIMIT) n_matches++;
    }

    /*Calculate the number of matched points, source points to be dropped and
     * new
     * destination points to be added*/
    arena.n_matches = n_matches;
    arena.n_drops = sn - n_matches;
    arena.n_new = dn - n_matches;

    j = 0;
    k = 0;
    /*Update the ange values in the tracker slots for both source and
     * destination.
     * Updating the destination is not
     * strictly necessary. Also add the new angles and drop the ones for which
     * no
     * match was found.*/
    for (i = 0; i < m; i++) {
        if (i < n && i < n_matches) {
            arena.matches_d[i] = dn_i[i_index[i]];
            arena.matches_s[i] = sn_j[j_index[i]];
            arena.matches_d[i] = dn_i[i_index[i]];
            arena.matches_s[i] = sn_j[j_index[i]];
        } else {
            if (i < dn) {
                arena.new[j] = dn_i[i_index[i]];
                j++;
            }
            if (i < sn) {
                arena.drop[k] = sn_j[j_index[i]];
                k++;
            }
        }
    }
}

/*SIMULATION*/

/*Here various testing routines are specified to test the code outside of
 * paparazzi*/
#define SIM_TIME 5

/*Obstacle simulator*/
/*void obstacle_sim_init(void){
  sim_obstacle[0].xy = vec2d_init(0,3);
  sim_obstacle[1].xy = vec2d_init(3,3);
  sim_obstacle[2].xy = vec2d_init(6,6);
  }*/
/*Vehicle simulator*/
/*void vehicle_sim_init(void){
  v_sim.xy_abs = vec2d_init_o(-2,-6,PI/2+eta);
  v_sim.xy_g = hme2grd_o(&v_sim.xy_abs);
  v_sim.v = arena.dl/4;
  }*/

/*void vehicle_sim(void){
  vec2d v_dest = vec2d_sub(&v_sim.xy_abs,&v_sim.wp_abs);
  vec2d v_vel = vec2d_norm(v_dest);

  v_sim.xy_abs.x += v_sim.v*v_vel.x;
  v_sim.xy_abs.y += v_sim.v*v_vel.y;
  v_sim.xy_g = hme2grd_o(&v_sim.xy_abs);
  }*/

/*Here are the placeholder functions to test everything outside of paparazzi,
 * later to be replaced by real ones that would use data coming from the
 * vehicle*/
#ifndef OBS_AVOID_ON
/*Return an array of observed reference points within the field of view and the
 * number of these points. Later simulated
 * data would be replaced with real data coming from the visual sensor*/
void obstacle_sim_return_angle(float vv[], int* n) {
    int i;

    vec2d tmp;
    for (i = 0; i < SIM_OBSTACLES; i++) {
        tmp = vec2d_sub(&v_sim.xy_g, &sim_obstacle[i].xy);
        vec2d_a_q(&tmp);
        vv[i] = v_sim.xy_g.o - tmp.angle;
    }
    *n = SIM_OBSTACLES;
    for (i = SIM_OBSTACLES; i < OBS_SLOTS; i++) vv[i] = NAN;
}

/*Return the arena corner points in ENU coordinates to construct the gridworld.
 * In the test these corner points are
 * hard-coded, in paparazzi those would be the waypoints defined in the flight
 * plan*/
void request_arena_coord(float* coord) {
    float map_borders[4][2] = {{-2, -6}, {6, -2}, {2, 6}, {-6, 2}};
    int i;
    int j;

    for (i = 0; i < 4; i++) {
        for (j = 0; j < 2; j++) {
            coord[i * 2 + j] = map_borders[i][j];
        }
    }
}
/*Request vehicle position in ENU coordinates*/
void request_position_abs(void) { veh.xy_abs = v_sim.xy_abs; }
/*Set new waypoint*/
void set_wp(float loc[], float* orient) {
    v_sim.wp_abs.x = loc[0];
    v_sim.wp_abs.y = loc[1];
    v_sim.xy_abs.o = *orient;
}

/*Check the status of the waypoint, whether it is reached already or not. Used
 * to set new waypoint.*/
int wp_status(void) {
    float range = vec2d_dist(&v_sim.xy_abs, &v_sim.wp_abs);
    if (range < arena.dl / 10) {
        return 1;
    } else {
        return 0;
    }
}

#else
/*The following functions use the paparazzi code and meant to be run in NPS mode
 * or on a real drone*/

/*Request the coordinates of arena corner points to construct the grid world and
 * output some status message about arena dimensions. The poitns are picked up
 * from the flight plan. */
void request_arena_coord(float* coord) {
    struct EnuCoor_f arena_corner[4];
    struct EnuCoor_i arena_corner_i[4];
    printf("Reading arena limits...\n");
    VECT3_COPY(arena_corner[0], waypoints[WP_FA2].enu_f);
    VECT3_COPY(arena_corner[1], waypoints[WP_FA1].enu_f);
    VECT3_COPY(arena_corner[2], waypoints[WP_FA4].enu_f);
    VECT3_COPY(arena_corner[3], waypoints[WP_FA3].enu_f);
    VECT3_COPY(arena_corner_i[0], waypoints[WP_FA2].enu_i);
    VECT3_COPY(arena_corner_i[1], waypoints[WP_FA1].enu_i);
    VECT3_COPY(arena_corner_i[2], waypoints[WP_FA4].enu_i);
    VECT3_COPY(arena_corner_i[3], waypoints[WP_FA3].enu_i);
    int i;
    for (i = 0; i < 4; i++) {
        printf("Point %i: %.3f,%.3f ", i, arena_corner[i].x, arena_corner[i].y);
    }
    printf(" OK\n");
    for (i = 0; i < 4; i++) {
        printf("Point %i: %i,%i ", i, arena_corner_i[i].x, arena_corner_i[i].y);
    }
    printf(" OK\n");

    for (i = 0; i < 4; i++) {
        coord[i * 2] = arena_corner[i].x;
        coord[i * 2 + 1] = arena_corner[i].y;
    }
}
/*Request the position data from the ardron in ENU coordinates as well as camera
 * orientation*/
void request_position_abs(void) {
    veh.xy_abs.x = state.enu_pos_f.x;
    veh.xy_abs.y = state.enu_pos_f.y;
    float psi = stateGetNedToBodyEulers_f()->psi;
    veh.xy_abs.o = (psi < -0.5 * PI) ? -1.5 * PI - psi : 0.5 * PI - psi;
}

/*Set new waypoint and orientation*/
void set_wp(float loc[], float* orient) {
    veh.wp_abs.x = loc[0];
    veh.wp_abs.y = loc[1];
    veh.wp_abs.o = *orient;
}

/*Check the waypoint status*/
int wp_status(void) {
    float range = vec2d_dist(&veh.xy_abs, &veh.wp_abs);
    if (range < 1) {
        return 1;
    } else {
        return 0;
    }
}

#endif

/*Set a discrete waypoint in the gridworld*/
void set_discrete_wp(int i, int j, float orient) {
    vec2d wp;
    /*Convert the discrete location within a grid map into real coordinates in
     * gridworld frame of reference*/
    wp.x = arena.gridx_coord[i];
    wp.y = arena.gridy_coord[j];
    wp.o = orient;

    /*Convert the coordinates from gridworld frame into ENU*/
    wp = grd2hme_o(&wp);

    float loc[2];
    loc[0] = wp.x;
    loc[1] = wp.y;
    float o = wp.o;

    /*Set new waypoint location and vehicle orientation*/
    set_wp(loc, &o);
    /*veh.xy_abs.o = o;*/
}

void request_obstacles(float* v, int* n) {
    //    obstacle_sim_return_angle(v,&nn);
    //    *n = nn;

    /*Request obstacle data from the vision algorithm. First value in the array
     * is
     * always returned as 0 due to some sorting errors, so instead it is
     * replaced
     * by NAN and the number of obstacles is reduced by one.*/
    memcpy(v, flowPeaks.angles, flowPeaks.nAngles * sizeof(float));
    *n = flowPeaks.nAngles;
    int kk = *n;

    if (n > 0) {
        v[0] = NAN;
        n--;
    };
    printf("Obstacles requested, %i returned\n", kk);

    /*printarr_float(v,*n);*/
}
/*Request vehicle position in ENU and convert it to gridworld coordinates*/
void request_position(void) {
    request_position_abs();
    veh.xy_g = hme2grd_o(&veh.xy_abs);
}

void set_disc_o(void) {}

/*Find the grid point closest to the vehicle*/
void vehicle_place_on_grid(void) {
    int i = round(veh.xy_g.x / arena.dl);
    int j = round(veh.xy_g.y / arena.dl);

    veh.gridij[0] = i;
    veh.gridij[1] = j;
    set_disc_o();
}

/*The following code is used to manage obstacle detection and tracking*/

/*Add an obstacle to an empty slot*/
void obstacle_add(float gamma) {
    int i = 0;
    int freeslot = -1;
    /*Go through available slots to find a free one*/
    while (freeslot == -1 && i < OBS_SLOTS) {
        if (arena.free_slots[i] == 1) freeslot = i;
        i++;
    }
    /*Save feature location in the field of view to an empty slot*/
    arena.angles_s[freeslot] = gamma;
    /*Mark the slot as taken*/
    arena.free_slots[freeslot] = 0;
    /*Save vehicle location from when the feature is first encountered*/
    arena.obs[freeslot].orig = veh.xy_g;
    /*Save the feature location in the field of view from when the feature is
     * first encountered*/
    arena.obs[freeslot].gamma_orig = veh.xy_g.o - gamma;
    /*Number to keep track of measurements taken*/
    arena.obs[freeslot].n_tracked = 0;
    /*Initialize a guessed location in the middle of arena*/
    arena.obs[freeslot].xy = vec2d_init(10000, 10000);
    /*Initialize the error*/
    arena.obs[freeslot].err = INFINITY;
    /*Up the source obstacles counter*/
    arena.sn++;
    /*printf("Obstacle added gamma: %.3f slot: %i\n",gamma,freeslot);*/
}
/*Destroy obstacle data, free up everything and drop the obstacle from the
 * tracker*/
void obstacle_destroy(int i) {
    arena.angles_s[i] = NAN;
    arena.free_slots[i] = 1;
    arena.sn--;
    obstacle obs;
    obs.err = INFINITY;
    arena.obs[i] = obs;
    /*printf("Obstacle destroyed slot: %i\n",i);*/
}
/*Update potential obstacle location  based on new data coming in*/
void obstacle_update(float gamma, int i) {
    /*Find the distance between the location of the vehicle when the tracking
     * began and current location*/
    float spread = vec2d_dist(&veh.xy_g, &arena.obs[i].orig);

    /*Pick up the vehicle position data and calculate relative angles*/
    arena.obs[i].upd = veh.xy_g;
    arena.obs[i].gamma_upd = veh.xy_g.o - gamma;
    arena.angles_s[i] = gamma;
    /*First rough approximation, no error*/
    if (arena.obs[i].n_tracked == 0) {
        vec2d xy_n =
            vec2d_triangulate(&arena.obs[i].orig, &arena.obs[i].upd,
                              arena.obs[i].gamma_orig, arena.obs[i].gamma_upd);
        arena.obs[i].xy = vec2d_init(xy_n.x, xy_n.y);

        arena.obs[i].n_tracked++;
        /*Check if the spread between the consecutive positions of the UAV is
         * big
         * enough, i.e. if the parallax is good, updat the location
         * approximation (running average) and approximate the error*/
    } else if (spread > MIN_SPREAD) {
        vec2d xy_n =
            vec2d_triangulate(&arena.obs[i].orig, &arena.obs[i].upd,
                              arena.obs[i].gamma_orig, arena.obs[i].gamma_upd);
        vec2d* xy_p = &arena.obs[i].xy;
        int kk = arena.obs[i].n_tracked;
        arena.obs[i].xy = vec2d_init((xy_n.x + xy_p->x * kk) / (1 + kk),
                                     (xy_n.y + xy_p->y * kk) / (1 + kk));

        arena.obs[i].n_tracked++;

        arena.obs[i].err = vec2d_dist(&xy_n, xy_p);
    }
    /*If the error is within specified limits and the object had been tracked
     * long enough update the map*/
    if (arena.obs[i].err < MAXERROR && spread > MIN_SPREAD &&
        arena.obs[i].n_tracked > MIN_READINGS) {
        int loc_x;
        int loc_y;

        int loc_x_tmp;
        int loc_y_tmp;
        if (arena.obs[i].xy.x <= arena.lim && arena.obs[i].xy.y <= arena.lim) {
            /*Find the closest grid coordinates*/
            loc_x = floor(arena.obs[i].xy.x / arena.dl);
            loc_y = floor(arena.obs[i].xy.y / arena.dl);

            int m;
            int n;
            vec2d gridlock;
            float dist;
            int loc_w;
            float sig;
            /*Update the weight values within a 3x3 matrix, with the approximate
             * obstacle location situated rougly in the center*/
            for (m = -1; m < 3; m++) {
                for (n = -1; n < 3; n++) {
                    loc_x_tmp = loc_x + m;
                    loc_y_tmp = loc_y + n;

                    if (loc_x_tmp >= 0 && loc_y_tmp >= 0 &&
                        loc_x_tmp < GRID_RES && loc_y_tmp < GRID_RES) {
/*Initially it just assumed that the location is good*/
#ifndef WEIGHTS_NORMALIZED
                        vec2d_set(&gridlock, arena.gridx_coord[loc_x_tmp],
                                  arena.gridy_coord[loc_y_tmp]);
                        dist = vec2d_dist(&gridlock, &arena.obs[i].xy);
                        loc_w = loc_x_tmp * GRID_RES + loc_y_tmp;
                        sig = MAXSCORE / (1 + exp(dist * arena.dl12 - 18));
                        /*sig = 100;*/

                        /*)      printf("Weights updated! %i,%i :%f\n",
                         * loc_x_tmp, loc_y, sig);*/

                        arena.grid_weights_obs[loc_w] =
                            (arena.grid_weights_obs[loc_w] > sig)
                                ? arena.grid_weights_obs[loc_w]
                                : sig;
/*Then normalization was applied later to fix the false positives. It works as
 * follows: some initial weight is assigned to every grid location. Once some
 * weight is added to "dangerous" grid point, it is substracted from the rest of
 * the values.*/
#else

                        vec2d_set(&gridlock, arena.gridx_coord[loc_x_tmp],
                                  arena.gridy_coord[loc_y_tmp]);
                        dist = vec2d_dist(&gridlock, &arena.obs[i].xy);
                        loc_w = loc_x_tmp * GRID_RES + loc_y_tmp;
                        /*Updates are calculated using a sigmoid function, based
                         * on distance from estimated obstacle location*/
                        sig = WEIGHT_INCR / (1 + exp(dist * arena.dl12 - 18));
                        /*sig = 100;*/
                        if (arena.grid_weights_obs[loc_w] < WEIGHT_MAX) {
                            sig_tmp += sig;
                            arena.grid_weights_obs[loc_w] += sig;
                        } else {
                            /*A limit is imposed on the maximum value that a
                             * weight can have*/
                            arena.grid_weights_obs[loc_w] = WEIGHT_MAX;
                        }

#endif
                        printf("WEIGHTS UPDATED\n");
                    }
                }
            }
            /*printf("MATCH FOUND\n");*/
            /*obstacle_destroy(i);*/
            /*obstacle_add(gamma);*/
        } else {
            /*If something is tracked outside of the arena, the obstacle is no
             * longer tracked */
            printf("GRID EXCEEDED\n");
            obstacle_destroy(i);
        }

        if (arena.obs[i].n_tracked > MAX_READINGS) {
            /*If something had been tracked long enough, the obstacle tracker is
             * re-set*/
            obstacle_destroy(i);
            printf("MAX_READINGS EXCEED\n");
        }
    }
}

/*This is the function for the 3rd possible action, i.e. 2 steps ahead*/
void plan_aheadp1(int i_loc, int j_loc, int head, float* q) {
    int i;
    int i_tmp;
    int j_tmp;
    float min = INFINITY;
    for (i = 0; i < 7; i++) {
        i_tmp = i_loc + arena.st_wp_i[i];
        j_tmp = j_loc + arena.st_wp_j[i];
        /*Here the exploration weights are added to the danger weights for each
         * possible action*/
        if (i_tmp >= 0 && i_tmp < GRID_RES && j_tmp >= 0 && j_tmp < GRID_RES) {
            *q = arena.grid_weights_exp[i_tmp * GRID_RES + j_tmp] +
                 arena.grid_weights_obs[i_tmp * GRID_RES + j_tmp] +
                 arena.scores[8 + i - head];
        } else {
            *q = INFINITY;
        }
        if (*q < min) {
            min = *q;
        }
    }
    /*Minimum possible score is found */
    *q = min;
}
/*One step ahead predicion*/
void plan_ahead(int i_loc, int j_loc, int head, float* q) {
    int i;
    int i_tmp;
    int j_tmp;
    float min = INFINITY;
    for (i = 0; i < 7; i++) {
        i_tmp = i_loc + arena.st_wp_i[i];
        j_tmp = j_loc + arena.st_wp_j[i];
        if (i_tmp >= 0 && i_tmp < GRID_RES && j_tmp >= 0 && j_tmp < GRID_RES) {
/*Trigger for two steps ahead prediction*/
#ifdef PLANAHEADP1
            plan_aheadp1(i_tmp, j_tmp, i, &q_ahead);
            *q = arena.grid_weights_exp[i_tmp * GRID_RES + j_tmp] +
                 arena.grid_weights_obs[i_tmp * GRID_RES + j_tmp] +
                 arena.scores[8 + i - head] + W_PREDICT * q_ahead;
#else

            *q = arena.grid_weights_exp[i_tmp * GRID_RES + j_tmp] +
                 arena.grid_weights_obs[i_tmp * GRID_RES + j_tmp] +
                 arena.scores[8 + i - head];
#endif
        } else {
            *q = INFINITY;
        }
        if (*q < min) {
            min = *q;
        }
    }
    *q = min;
}

/*Here the movement strategy is defined. Implementation is not optimal, with
 * three separate function, but it was a quick fix. The arguments are the
 * discrete location of the vehicle on the grid and it's discrete
 * orientation(North, north-east, east, etc...)*/
void plan_action(int i_loc, int j_loc, int head, int* best, float* q) {
    int i;
    int i_tmp;
    int j_tmp;
    float min = INFINITY;
#ifdef PLAN_AHEAD
    float q_ahead;
#endif
    for (i = 0; i < 7; i++) {
        i_tmp = i_loc + arena.st_wp_i[i];
        j_tmp = j_loc + arena.st_wp_j[i];
        if (i_tmp >= 0 && i_tmp < GRID_RES && j_tmp >= 0 && j_tmp < GRID_RES) {
#ifdef PLAN_AHEAD
            /*The action scores for each possible movement are added with the
             * best possible outcomes of the actions up ahead. The best one is
             * selected.*/
            plan_ahead(i_tmp, j_tmp, i, &q_ahead);
            *q = arena.grid_weights_exp[i_tmp * GRID_RES + j_tmp] +
                 arena.grid_weights_obs[i_tmp * GRID_RES + j_tmp] +
                 arena.scores[8 + i - head] + W_PREDICT * q_ahead;
#else

            plan_ahead(i_tmp, j_tmp, i, &q_ahead);
            *q = arena.grid_weights_exp[i_tmp * GRID_RES + j_tmp] +
                 arena.grid_weights_obs[i_tmp * GRID_RES + j_tmp] +
                 arena.scores[8 + i - head];
#endif
            /*printf("action:%i -> score %f\n,
             * min:%f,best@%i",i,*q,min,*best);*/
        } else {
            *q = INFINITY;
        }
        if (*q < min) {
            *best = i;
            min = *q;
        }
    }
    /*printf("best action is %i\nmove from %i,%i to
     * %i,%i\n",*best,i_loc,j_loc,i_loc+arena.st_wp_i[*best],j_loc+arena.st_wp_j[*best]);*/
}

/*void move(){}*/
#ifdef WEIGHTS_NORMALIZED
float total_weight = GRID_RES * GRID_RES * WEIGHT_BASE;
#endif
/*This function deals with the obstacle data coming in. Arguments are locations
 * of the obstacles as an angle within field of view and the number of these
 * locations.*/
void arena_update(float v[], int n) {
    /*printarr_float(v,OBS_SLOTS);*/
    /*printf("%i",n);*/

    /*printf("arena_update v:%.3f n:%i\n",v[0],n);*/
    int i;
    int j;
    if (n > 0) {
        if (arena.sn == 0) {
            /*If nothing is currently tracked, new obstacles are added to the
             * obstacle tracker and everything else is cleared*/
            for (i = 0; i < OBS_SLOTS; i++) {
                if (!isnan(v[i])) {
                    obstacle_add(v[i]);
                }
            }
            for (i = arena.sn; i < OBS_SLOTS; i++) {
                arena.angles_s[i] = NAN;
            }
        } else {
            /*If somethin is already tracked the new data is compared with the
             * old data to do the matching*/
            for (j = 0; j < OBS_SLOTS; j++) arena.angles_d[j] = v[j];
            arena.dn = n;
            angle_matcher();

            /*Once the angle matcher is done, new obstacles are added, the ones
             * that had no matches are destroyed, and the
             * ones that did have a match are updated*/

            for (i = 0; i < arena.n_drops; i++) obstacle_destroy(arena.drop[i]);
            for (i = 0; i < arena.n_new; i++) obstacle_add(v[arena.new[i]]);
            for (i = 0; i < arena.n_matches; i++)
                obstacle_update(v[arena.matches_d[i]], arena.matches_s[i]);
#ifdef WEIGHTS_NORMALIZED
            /*Here the normalization is run, based on the amount of points that
             * were updated in the previous step*/
            if (sig_tmp > 0) {
                float norm_factor = total_weight / (total_weight + sig_tmp);

                for (i = 0; i < GRID_RES; i++) {
                    for (j = 0; j < GRID_RES; j++) {
                        arena.grid_weights_obs[i * GRID_RES + j] *= norm_factor;
                    }
                }
                sig_tmp = 0;
            }
#endif
        }
    }
}
/*The main loop that is called every time some new visual data is coming in*/
void navigate(void) {
    /*If the navigation had not been run yet then quit.*/
    if (!avoid_nav_init) return;
    /*Otherwise request position, data from the camera and update the arena*/
    request_position();
    float vision_tmp[OBS_SLOTS];
    int n_tmp = 0;
    request_obstacles(vision_tmp, &n_tmp);

    arena_update(vision_tmp, n_tmp);
}

/*Here the vehicle cache is initizlized. It is used to keep track of UAV.*/
void vehicle_cache_init(void) {
    request_position();
    vehicle_place_on_grid();
    set_discrete_wp(veh.gridij[0], veh.gridij[1], 0);

    printf("Vehicle xy location in ENU: %.3f,%.3f orientation: %.3f\n",
           veh.xy_abs.x, veh.xy_abs.y, veh.xy_abs.o);
    printf("Vehicle xy location in GRD: %.3f,%.3f orientation: %.3f\n",
           veh.xy_g.x, veh.xy_g.y, veh.xy_g.o);
}
/*Here the map is initiazlized.*/
void init_map(void) {
    int i, j;

    float bord[8];

    float xy[4][2];
    request_arena_coord(bord);
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 2; j++) {
            xy[i][j] = bord[i * 2 + j];
        }
    }

    /*Paparazzi waypoints are turned into grid corners*/
    xy_grd.x = vec2d_norm(vec2d_init(xy[1][0] - xy[0][0], xy[1][1] - xy[0][1]));
    xy_grd.y = vec2d_norm(vec2d_init(xy[3][0] - xy[0][0], xy[3][1] - xy[0][1]));
    xy_grd.offset = vec2d_init(-xy[0][0], -xy[0][1]);

    xy_hme.x =
        vec2d_norm(vec2d_init(xy[1][0] - xy[0][0], -xy[1][1] + xy[0][1]));
    xy_hme.y =
        vec2d_norm(vec2d_init(-xy[3][0] + xy[0][0], xy[3][1] - xy[0][1]));
    /*xy_hme.offset =
     * vec2d_sub((*)hme2grd(&xy_grd.offset),(*)vec2d_init(0,0));*/

    vec2d tmp = vec2d_init(0, 0);
    tmp = hme2grd_o(&tmp);
    tmp.x = -tmp.x;
    tmp.y = -tmp.y;
    xy_hme.offset = tmp;
    /*xy_hme.x = vec2d_init(1,0);*/
    /*xy_hme.y = vec2d_init(0,1);*/
    /*xy_hme.offset = vec2d_init(0,0);*/

    /*xy_grd.x = hme2grd(&xy_hme.x);*/
    /*xy_grd.y = hme2grd(&xy_hme.y);*/

    /*Various stuff is calculated. Rotation of the grid, grid size, grid unit
     * length, etc...*/
    lside = sqrt(pow(xy[1][0] - xy[0][0], 2) + pow(xy[1][1] - xy[0][1], 2));
    eta = atan2(abs(xy[1][1] - xy[0][1]), abs(xy[1][0] - xy[0][0]));
    ceta = cos(eta);
    seta = sin(eta);

    float dl = lside / (GRID_RES - 1);
    arena.dl = dl;
    arena.dl12 = 12 / dl;
    arena.lim = lside;

    for (i = 0; i < GRID_RES; i++) {
        arena.gridx_coord[i] = i * dl;
        arena.gridy_coord[i] = i * dl;
    }

    for (i = 0; i < GRID_RES * GRID_RES; i++) {
        arena.grid_weights_exp[i] = 0;
        arena.grid_weights_obs[i] = WEIGHT_BASE;
    }

    for (i = 0; i < OBS_SLOTS; i++) {
        arena.angles_s[i] = NAN;
        arena.angles_d[i] = NAN;
        arena.free_slots[i] = 1;
    }

    arena.sn = 0;
    arena.dn = 0;
    arena.maxerr = MAXERROR * dl;
    int st_wp_i[] = {0, 1, 1, 1, 0, -1, -1, -1};
    int st_wp_j[] = {1, 1, 0, -1, -1, -1, 0, 1};

    for (i = 0; i < 8; i++) arena.st_wp_i[i] = st_wp_i[i];
    for (i = 0; i < 8; i++) arena.st_wp_j[i] = st_wp_j[i];

    arena.action_scores[0] = 0;
    arena.action_scores[1] = 1;
    arena.action_scores[2] = 3;
    arena.action_scores[3] = 5;
    arena.action_scores[4] = 4;
    arena.action_scores[5] = 5;
    arena.action_scores[6] = 3;
    arena.action_scores[7] = 1;

    /*Here the decision weights for immediate action are defined. Going forwards
     * has the highest score(0), while turning around has the lowest(4).*/
    int kk[] = {0, 0.5, 3, 5, 4, 5, 3, 0.5, 0, 0.5, 3, 5, 4, 5, 3, 0.5};
    for (i = 0; i < 16; i++) arena.scores[i] = kk[i];
    for (i = 0; i < 8; i++) {
        for (j = 0; j < 8; j++) {
            arena.action_scores[i * 8 + j] = kk[(i + 8 - j)];
        }
    }

    for (i = 0; i < 7; i++) arena.st_headings[i] = PI / 2 - (i)*PI / 4.0;
    arena.st_headings[7] = 135 * PI / 180;

    printf("Arena size: %.3fx%.3f m\n", lside, lside);

    /*sim_init();*/
}

/*Following are various debug functions.*/
#define PRINT_THIS

void nav_print_full_report(void) {
    float v[OBS_SLOTS];
    int tmp = OBS_SLOTS;
    request_obstacles((float*)&v, &tmp);

    printf("Status report:\n");
    printf("Angles coming in:\n");
    printarr_float_angles(v, tmp);

#ifdef PRINT_THIS
    int i;
    obstacle o;
#ifdef AVOID_NAV_DEBUG
    nav_debug_downlink[0] = arena.obs[0].xy.x;
    nav_debug_downlink[1] = arena.obs[0].xy_n.x;
    nav_debug_downlink[2] = arena.obs[1].xy.x;
    nav_debug_downlink[3] = arena.obs[1].xy_n.x;
    nav_debug_downlink[4] = arena.obs[0].xy.y;
    nav_debug_downlink[5] = arena.obs[0].xy_n.y;
    nav_debug_downlink[6] = arena.obs[1].xy.y;
    nav_debug_downlink[7] = arena.obs[1].xy_n.y;

    nav_debug_downlink[8] = arena.obs[1].err;
    nav_debug_downlink[9] = arena.obs[1].err;
#endif
    printf("Curr. position: (abs,grd,discrete) %.2f,%.2f %.2f,%.2f %i,%i\n",
           veh.xy_abs.x, veh.xy_abs.y, veh.xy_g.x, veh.xy_g.y, veh.gridij[0],
           veh.gridij[1]);
    printf("Curr. wp. setting: (abs,grd) %.2f,%.2f,%.2f,%.2f\n", veh.wp_abs.x,
           veh.xy_abs.y, veh.wp_g.x, veh.wp_g.y);
    float range = vec2d_dist(&veh.xy_abs, &veh.wp_abs);
    printf("Range to next waypoint: %f\n", range);
    printf("Obstacle parallax:\n");
    /*printf("p0: %f p1:
     * %f\n",arena.angles_d[0]*180/PI,arena.angles_d[1]*180/PI);*/
    printf("Current field of view:\n");
    printarr_float(arena.angles_s, OBS_SLOTS);
    printf("Currently tracked obstacles:\n");
    /*#ifdef WITH_NAVIGATION*/
    for (i = 0; i < arena.sn; i++) {
        o = arena.obs[i];
        printf(
            "%i: original fix at %f,%f, latest fix at %f,%f, angle1: %f, "
            "angle2: "
            "%f current xy fix %f,%f\n)",
            i, o.orig.x, o.orig.y, o.upd.x, o.upd.y, o.gamma_orig, o.gamma_upd,
            o.xy.x, o.xy.y);
    }

    printf("Current obstacle weights\n");

    print2darr_float(arena.grid_weights_obs, GRID_RES, GRID_RES);

#endif

    /*#endif*/
}

#ifdef NTEST
int test_all;
float angle1;
float angle2;
int testes_counter;

vec2d xy_1;
vec2d xy_2;

int cccounter;
void testing_routines(void) {
    float v;
    int n;
    request_obstacles(&v, &n);
    if (n > 1) {
    }
    float vfirst =
        &v[1]

        switch (test_all) case 1
        : for (testes_counter = 0; testes_counter < 500; testes_counter++) {
        printf("Running angle checks\n");
        printf("Place drone in position 1\n");
        request_obstacles(&v, &n);

        if (n > 1) {
            printf("leftmost value: %f, rightmost value: %f\n", v[1], )
                /*do something*/
                break;

            case 2:
                printf("Place drone in position 2...");
                break;

            /*do something*/
            case 3:
                printf("Running triangulation checks\n");
                printf("Place drone in position 1...");
                break;

            case 4:
                printf("Place drone in position 2...");
                break;

            case 5:
                printf("Running tracking checks\n");
                printf("Move the drone around...");
                break;
        }

#endif
