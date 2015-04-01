#include <stdio.h>
#include <stdlib.h>
#include <math.h>



#include <limits.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <sys/time.h>

//#define AVOID_NAV_DEBUG
/*#define AVOID_NAV_DEBUG_DOWNLINK_SIZE 10*/

#ifdef AVOID_NAV_DEBUG
  extern float nav_debug_downlink[AVOID_NAV_DEBUG_DOWNLINK_SIZE];
#endif




float lside;
float eta;
float ceta;
float seta;

float mav_xy[2];
#define PI 3.14159265358979323846

#define OBS_SLOTS 100
#define GRID_RES 20
/*Tracker settings*/
#define TRACKING_LIMIT 0.2
/*Detection thresholds*/
#define MAXERROR 0.8
#define MIN_PARALLAX 0.0
#define MIN_SPREAD 0.2
/*Grid weights settings */
#define MAXSCORE 100
/*Simulated obstacle settings*/
//#define SIM_OBSTACLES 3
//#define SIM_FOV 40

#define PLAN_AHEAD
#define PLAN_AHEADP1
#define W_PREDICT 0.75

#define WEIGHTS_NORMALIZED

#ifdef WEIGHTS_NORMALIZED
#define WEIGHT_MAX 20.0
#define WEIGHT_INCR 2.0
#define WEIGHT_BASE 10.0

#define WP_MINDIST 0.5


float sig_tmp;
float total_weight; 

#endif
#define OPTI_REAL

typedef struct {
    float w;
    float y;
} qua2d;
typedef struct {
    float x;
    float y;
    float c1;
    float s1;
    float mag;
    float angle;
    qua2d q;
    float o;
} vec2d;

typedef struct {
    vec2d x;
    vec2d y;
    vec2d offset;
} coord_sys;

coord_sys xy_abs;
coord_sys xy_grd;
coord_sys xy_hme;


typedef struct {
    vec2d orig;
    float gamma_orig;
    vec2d upd;
    float gamma_upd;
    vec2d xy;
    vec2d xy_n;
    int id;
    int n_tracked;
    float err;
} obstacle;


typedef struct {
    obstacle obs[OBS_SLOTS];
    int free_slots[OBS_SLOTS];

    int n_matches;
    int n_drops;
    int n_new;
    int sn;
    int dn;
    float angles_s[OBS_SLOTS];
    float angles_d[OBS_SLOTS];
    int matches_s[OBS_SLOTS];
    int matches_d[OBS_SLOTS];
    int drop[OBS_SLOTS];
    int new[OBS_SLOTS];

    float gridx_coord[GRID_RES];
    float gridy_coord[GRID_RES];

    float grid_weights_exp[GRID_RES*GRID_RES];
    float grid_weights_obs[GRID_RES*GRID_RES];
    float dl;
    float dl12;

    float lim;
    float maxerr;
    float action_scores[64];
    float scores[16];
    float st_headings[8];
    int st_wp_i[8];
    int st_wp_j[8];
} Arena;

Arena arena;


typedef struct {
    vec2d xy_abs;
    vec2d wp_abs;
    vec2d xy_g;
    vec2d wp_g;
    int gridij[2];
    int o_disc;
    float v;
} Vehicle;

Vehicle veh;
Vehicle v_sim;

void sayhello(void);
void vec2d_abs(vec2d* a);

void vec2d_cs(vec2d* a) ;
void vec2d_q(vec2d* a) ;
void vec2d_a_q(vec2d* a) ;
vec2d vec2d_init(float x, float y) ;
vec2d vec2d_init_o(float x, float y, float o) ;
void vec2d_set(vec2d* v, float x, float y) ;
void vec2d_set_o(vec2d* v, float x, float y,float o) ;
vec2d arr2vec2d(float * arrxy) ;
vec2d vec2d_add(vec2d* a, vec2d* b) ;
vec2d vec2d_sub(vec2d* a, vec2d* b) ;
float vec2d_dot(vec2d* a, vec2d* b);
float vec2d_ang(vec2d* a, vec2d* b);
float vec2d_ang_q(vec2d* a,vec2d* b);
vec2d vec2d_norm(vec2d a);
vec2d hme2grd(vec2d* a);
vec2d grd2hme(vec2d* a);
vec2d grd2hme_o(vec2d* a);
vec2d hme2grd_o(vec2d* a);
float vec2d_dist(vec2d *a, vec2d * b);
void printarr_float(float a[],int n);
void printarr_float_angles(float a[],int n);
void printarr_int(int a[],int n);
void print2darr_float(float* a,int n, int m);
void sw_arr_int(int *a,int s,int d);
void sw_arr_float(float *a,int s,int d);
void arena_report (void);
void angle_matcher(void);
vec2d vec2d_triangulate(vec2d* a, vec2d* b, float gamma1, float gamma2) ;
void print_vec2d(vec2d v) ;
void vehicle_sim_init(void);
//obstacle sim_obstacle[SIM_OBSTACLES];
void obstacle_sim_init(void);
void vehicle_sim(void);
void obstacle_sim_return_angle(float vv[],int *n);
void request_arena_coord(float *coord);
void request_position_abs(void);
void set_wp(float loc[], float *orient);
int wp_status(void);
void set_discrete_wp(int i, int j, float orient);
void request_obstacles(float *v,int *n);
void request_position(void);
void set_disc_o(void);
void vehicle_place_on_grid(void);
void obstacle_add(float gamma);
void obstacle_destroy(int i);
void obstacle_update(float gamma, int i);
void plan_ahead(int i_loc, int j_loc, int head,float *q);
void plan_aheadp1(int i_loc, int j_loc, int head,float *q);
void plan_action(int i_loc, int j_loc, int head,int *best,float *q);
void move(void);
void arena_update(float v[],int n);
int counter_nav;
void navigate(void);
void vehicle_cache_init(void);
void init_map(void) ;



extern void  nav_print_full_report(void);



    /*void init_map();*/
    /*void navigate();*/
    /*void vehicle_sim();*/
    /*void vehicle_sim_init();*/
    /*void vehicle_cache_init();*/
    /*void obstacle_sim_init();*/

    /*void print_vec2d(vec2d v); */
    /*void print2darr_float(float* a,int n, int m);*/
    /*void printarr_float(float a[],int n);*/

