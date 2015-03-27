#include <stdio.h>
#include <stdlib.h>
#include <math.h>

float lside;
float eta;
float ceta;
float seta;

float mav_xy[2];
#define PI 3.14159265358979323846

#define OBS_SLOTS 10
#define GRID_RES 20
#define TRACKING_LIMIT 2
#define MAXERROR 1
#define MAXSCORE 100
#define EXPSCORE 3
#define STRSCORE 0
#define TRNSCORE 1

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

void init_map();
void navigate();
