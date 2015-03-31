#include "avoid_nav.h"
#include "state.h"
#include "generated/flight_plan.h"
#include "../subsystems/navigation/waypoints.h"

#include "avoid_nav_transportFcns.h"

#define OBS_AVOID_ON


unsigned char avoid_nav_init = 0;

void sayhello(void)
{
    printf("hello!\n");
}


void vec2d_abs(vec2d* a) {
    a->mag = sqrt(pow(a->x,2)+pow(a->y,2));
}

void vec2d_cs(vec2d* a) {
    vec2d_abs(a);
    a->c1 = a->x/a->mag;
    a->s1 = a->y/a->mag;
}
void vec2d_q(vec2d* a) {
    vec2d_cs(a);
    a->q.w = sqrt(2+2*a->c1)/2;
    a->q.y = a->s1/2/a->q.w;
}
void vec2d_a_q(vec2d* a) {
    vec2d_q(a);
    a->angle = atan2(2*a->q.y*a->q.w,1-2*pow(a->q.y,2));
    if(isnan(a->angle)) a->angle = PI;
}

vec2d vec2d_init(float x, float y) {
    vec2d v;
    v.x = x;
    v.y = y;
    v.o = 0;
    return v;
}
vec2d vec2d_init_o(float x, float y, float o) {
    vec2d v;
    v.x = x;
    v.y = y;
    v.o = o;
    return v;
}

void vec2d_set(vec2d* v, float x, float y) {
    v->x=x;
    v->y=y;
}
void vec2d_set_o(vec2d* v, float x, float y,float o) {
    v->x=x;
    v->y=y;
    v->o=o;
}

vec2d arr2vec2d(float * arrxy) {
    vec2d v;
    v.x = arrxy[0];
    v.y = arrxy[1];
    return v;
}
vec2d vec2d_add(vec2d* a, vec2d* b) {
    vec2d v;
    v.x = a->x+b->x;
    v.y = a->y+b->y;
    return v;
}
vec2d vec2d_sub(vec2d* a, vec2d* b) {
    vec2d v;
    v.x = b->x-a->x;
    v.y = b->y-a->y;
    return v;
}
float vec2d_dot(vec2d* a, vec2d* b){
    return (a->x*b->x+a->y*b->y);
    
}
float vec2d_ang(vec2d* a, vec2d* b){
    vec2d_abs(a);
    vec2d_abs(b);
    float dot = vec2d_dot(a,b);
    return acos(dot/a->mag/b->mag);
}
float vec2d_ang_q(vec2d* a,vec2d* b){
    /*vec2d_cs(a);*/
    /*vec2d_cs(b);*/
    vec2d_a_q(a);
    vec2d_a_q(b);
    return b->angle-a->angle;
}
vec2d vec2d_norm(vec2d a){
    vec2d_abs(&a);
    a.x = a.x/a.mag;
    a.y = a.y/a.mag;
    return a;
}
vec2d hme2grd(vec2d* a){
    vec2d v;
    vec2d out;
    v = vec2d_add(a,&xy_grd.offset);
    out.x = vec2d_dot(&v,&xy_grd.x);
    out.y = vec2d_dot(&v,&xy_grd.y);
    return out;
}
vec2d grd2hme(vec2d* a){
    vec2d v;
    vec2d out;
    v = vec2d_add(a,&xy_hme.offset);
    out.x = vec2d_dot(&v,&xy_hme.x);
    out.y = vec2d_dot(&v,&xy_hme.y);
    return out;
}

vec2d grd2hme_o(vec2d* a){
    vec2d v=grd2hme(a);
    v.o = a->o+eta;
    return v;
}
vec2d hme2grd_o(vec2d* a){
    vec2d v=hme2grd(a);
    v.o = a->o-eta;
    /*printf("a.o:%f,v.o:%f,v.o-eta:%f\n",a->o,v.o,v.o-eta);*/
    return v;
}
float vec2d_dist(vec2d *a, vec2d * b){
    vec2d tmp = vec2d_sub(a,b);
    vec2d_abs(&tmp);
    return tmp.mag;
}




void printarr_float(float a[],int n){
    int i;
    printf("[");
    for (i=0; i<n;i++) {

        printf("%f, ",a[i]);
    }
    printf("]\n");
}
void printarr_int(int a[],int n){
    int i;
    printf("[");
    for (i=0; i<n;i++) {

        printf("%i, ",a[i]);
    }
    printf("]\n");
}

void print2darr_float(float* a,int n, int m){
    int i;
    int j;
    
    for(i=0;i<n;i++){
        printf("[");
        for(j=0;j<m;j++){
            printf("%.2f",a[i*m+j]);
            if (j<m-1){
                printf(", ");
            }
        }
        printf("]\n");
    }
}

void sw_arr_int(int *a,int s,int d){
    int tmp = a[d];
    a[d]=a[s];
    a[s]=tmp;
}
void sw_arr_float(float *a,int s,int d){
    float tmp = a[d];
    a[d]=a[s];
    a[s]=tmp;
}

void arena_report (void){
    printf("Track report\npoints matched: %i\nmatches:\n",arena.n_matches);
    int i;
    for(i=0;i<arena.n_matches;i++){
        printf("p%i(%f) to p%i(%f)\n",arena.matches_s[i],arena.angles_s[arena.matches_s[i]],arena.matches_d[i],arena.angles_d[arena.matches_d[i]]);
    }
    printf("dropped points:\n");
    for(i=0;i<arena.n_drops;i++) printf("%i\n",arena.drop[i]);
    printf("new points:\n");
    for(i=0;i<arena.n_new;i++) printf("%i\n",arena.new[i]);
    
}
void angle_matcher(void){


    int dn = arena.dn;
    int sn = arena.sn;
    /*printf("Matching sn: %i, dn: %i\n",arena.sn,arena.dn);*/
    /*printarr_float(arena.angles_s,10);*/
    /*printarr_float(arena.angles_d,10);*/


    if(dn == 0||sn == 0) return;

    float D[sn*dn];
    int dn_i[dn];
    int sn_j[sn];

    int i=0;
    int j=0;



    while (j<sn) {
        if (!isnan(arena.angles_s[i])){
            sn_j[j]=i;
            j++;
        }
        i++;
    }

    i=0;
    j=0;

    while (j<dn) {
        if (!isnan(arena.angles_d[i])){
            dn_i[j]=i;
            j++;
        }
        i++;
    }

    for (i=0;i<dn;i++){
        for(j=0;j<sn;j++){
            D[i*sn+j]=fabs(arena.angles_s[sn_j[j]]-arena.angles_d[dn_i[i]]);
        }
    }

    float tmp = INFINITY;
    float tmp2 = INFINITY;
    int pivot[] = {0,0};
    for (i=0;i<dn;i++){
        for(j=0;j<sn;j++){
            if(D[i*sn+j]<tmp){
                pivot[0]=i;
                pivot[1]=j;
                tmp = D[i*sn+j];
            }
        }
    }

    int i_index[dn];
    int j_index[sn];
    for (i = 0; i<dn; i++) i_index[i] = i;
    for (i = 0; i<sn; i++) j_index[i] = i;

    sw_arr_int(i_index,0,pivot[0]);
    sw_arr_int(j_index,0,pivot[1]);

    int n = (dn < sn) ? dn:sn;

    int k;

    for (k=0;k<n;k++) {
        tmp = D[i_index[k]*sn+j_index[k]];

        for (i=k;i<dn;i++){
            tmp2 = D[i_index[i]*sn+j_index[k]];
            if (tmp2<tmp){
                sw_arr_int(i_index,i,k);
                tmp = tmp2;
            }
        }
        for (i=k;i<sn;i++){
            tmp2 = D[i_index[k]*sn+j_index[i]];
            if (tmp2<tmp){
                sw_arr_int(j_index,i,k);
                tmp = tmp2;
            }
        }
    }

    float D_new[sn*dn];
    for (i=0;i<dn;i++){
        for (j=0;j<sn;j++){
            D_new[i*sn+j]=D[i_index[i]*sn+j_index[j]];
        }
    }



    int m = (dn > sn) ? dn:sn;
    int n_matches = 0;


    for (i=0;i<n;i++) {
        if (D_new[i*sn+i]<TRACKING_LIMIT) n_matches++;
    }

    arena.n_matches = n_matches;
    arena.n_drops = sn-n_matches;
    arena.n_new = dn-n_matches;
    
    j = 0;
    k = 0;

    for(i=0;i<m;i++){
        if(i<n&&i<n_matches){
            arena.matches_d[i] = dn_i[i_index[i]];
            arena.matches_s[i] = sn_j[j_index[i]];
            arena.matches_d[i] = dn_i[i_index[i]];
            arena.matches_s[i] = sn_j[j_index[i]];
        }
        else{
            if(i<dn){
                arena.new[j]=dn_i[i_index[i]];
                j++;
            }
            if(i<sn){
                arena.drop[k]=sn_j[j_index[i]];
                k++;
            }
        }
    }
}


vec2d vec2d_triangulate(vec2d* a, vec2d* b, float gamma1, float gamma2) {
    float c1 = cos(gamma1);
    float s1 = sin(gamma1);
    float c2 = cos(gamma2);
    float s2 = sin(gamma2);
    float c1c2 = c1*c2;
    float c1s2 = c1*s2;
    float c2s1 = c2*s1;
    float s1s2 = s1*s2;
    float det = 1/(c1s2-c2s1);
    
    vec2d v;
    v.x = det*(c1c2*a->y-c1c2*b->y-c2s1*a->x+c1s2*b->x);
    v.y = det*(c1s2*a->y-c2s1*b->y-s1s2*a->x+s1s2*b->x);
    return v;
    
}
void print_vec2d(vec2d v) {
    printf("v: [%f,%f], orient:%f\n",v.x,v.y,v.o*180/PI);
}


/*SIMULATION*/
#define SIM_TIME 5


/*void vehicle_sim_init(void){
    v_sim.xy_abs = vec2d_init_o(-2,-6,PI/2+eta);
    v_sim.xy_g = hme2grd_o(&v_sim.xy_abs);
    v_sim.v = arena.dl/4;
}*/

/*void obstacle_sim_init(void){
    sim_obstacle[0].xy = vec2d_init(0,3);
    sim_obstacle[1].xy = vec2d_init(3,3);
    sim_obstacle[2].xy = vec2d_init(6,6);
}*/

/*void vehicle_sim(void){
    vec2d v_dest = vec2d_sub(&v_sim.xy_abs,&v_sim.wp_abs);
    vec2d v_vel = vec2d_norm(v_dest);

    v_sim.xy_abs.x += v_sim.v*v_vel.x;
    v_sim.xy_abs.y += v_sim.v*v_vel.y;
    v_sim.xy_g = hme2grd_o(&v_sim.xy_abs);
}*/



#ifndef OBS_AVOID_ON
void obstacle_sim_return_angle(float vv[],int *n){
    int i;

    vec2d tmp;
    for(i=0;i<SIM_OBSTACLES;i++){
        tmp = vec2d_sub(&v_sim.xy_g,&sim_obstacle[i].xy);
        vec2d_a_q(&tmp);
        vv[i] = v_sim.xy_g.o-tmp.angle;
    }
    *n = SIM_OBSTACLES;
    for(i=SIM_OBSTACLES;i<OBS_SLOTS;i++) vv[i] = NAN;
}
void request_arena_coord(float *coord){
    float map_borders[4][2] = {
        {-2,-6},
        {6,-2},
        {2,6},
        {-6,2}};
        int i;
        int j;

        for(i=0;i<4;i++){
            for(j=0;j<2;j++){
                coord[i*2+j]=map_borders[i][j];
            }
        }
    
}
void request_position_abs(void){
    veh.xy_abs = v_sim.xy_abs;
    

}
void set_wp(float loc[], float *orient){
    v_sim.wp_abs.x = loc[0];
    v_sim.wp_abs.y = loc[1];
    v_sim.xy_abs.o = *orient;    
}


int wp_status(void){
    float range = vec2d_dist(&v_sim.xy_abs,&v_sim.wp_abs);
    if (range < arena.dl/10){
        return 1;
    }else{
        return 0;
    }
}

void obstacle_sim_return_angle(float vv[],int *n){
    int i;

    vec2d tmp;
    for(i=0;i<SIM_OBSTACLES;i++){


        tmp = vec2d_sub(&veh.xy_g,&sim_obstacle[i].xy);
        vec2d_a_q(&tmp);
        vv[i] = veh.xy_g.o-tmp.angle;
    }
    *n = SIM_OBSTACLES;
    for(i=SIM_OBSTACLES;i<OBS_SLOTS;i++) vv[i] = NAN;
   
}
#else
void request_arena_coord(float *coord){
    struct EnuCoor_f arena_corner[4];
    struct EnuCoor_i arena_corner_i[4];
    printf("Reading arena limits...\n");
    VECT3_COPY(arena_corner[0],waypoints[WP_FA2].enu_f);
    VECT3_COPY(arena_corner[1],waypoints[WP_FA1].enu_f);
    VECT3_COPY(arena_corner[2],waypoints[WP_FA4].enu_f);
    VECT3_COPY(arena_corner[3],waypoints[WP_FA3].enu_f);
    VECT3_COPY(arena_corner_i[0],waypoints[WP_FA2].enu_i);
    VECT3_COPY(arena_corner_i[1],waypoints[WP_FA1].enu_i);
    VECT3_COPY(arena_corner_i[2],waypoints[WP_FA4].enu_i);
    VECT3_COPY(arena_corner_i[3],waypoints[WP_FA3].enu_i);
    int i;
    for(i=0;i<4;i++){
        printf("Point %i: %.3f,%.3f ",i,arena_corner[i].x,arena_corner[i].y);
    }
    printf(" OK\n");
    for(i=0;i<4;i++){
        printf("Point %i: %i,%i ",i,arena_corner_i[i].x,arena_corner_i[i].y);
    }
    printf(" OK\n");

    for(i=0;i<4;i++){
        coord[i*2]=arena_corner[i].x;
        coord[i*2+1]=arena_corner[i].y;
    }
    
}
void request_position_abs(void){
    veh.xy_abs.x = state.enu_pos_f.x;
    veh.xy_abs.y = state.enu_pos_f.y; 
    float psi = stateGetNedToBodyEulers_f()->psi;
    veh.xy_abs.o = (psi <-0.5*PI ) ? -1.5*PI-psi:0.5*PI-psi;

    

}
void set_wp(float loc[], float *orient){
    veh.wp_abs.x = loc[0];
    veh.wp_abs.y = loc[1];
    veh.wp_abs.o = *orient;
}


int wp_status(void){
    float range = vec2d_dist(&veh.xy_abs,&veh.wp_abs);
    if (range < 1){
        return 1;
    }else{
        return 0;
    }
}

#endif

void set_discrete_wp(int i, int j, float orient){
    vec2d wp;
    wp.x = arena.gridx_coord[i];
    wp.y = arena.gridy_coord[j];
    wp.o = orient;


    wp = grd2hme_o(&wp);

    float loc[2];
    loc[0]=wp.x;
    loc[1]=wp.y;
    float o = wp.o;


    set_wp(loc,&o);
    /*veh.xy_abs.o = o;*/
}

void request_obstacles(float *v,int *n)
{

//    obstacle_sim_return_angle(v,&nn);
//    *n = nn;
  memcpy(v, flowPeaks.angles,flowPeaks.nAngles*sizeof(float));
  *n = flowPeaks.nAngles;
}





/*void request_position(vec2d *loc, float *orient){*/
void request_position(void){


    /*float lc[2];*/
    /*float o;*/

    request_position_abs();

    /*veh.xy_abs = vec2d_init_o(lc[0],lc[1],o);*/
    veh.xy_g = hme2grd_o(&veh.xy_abs);

    /**loc = veh.xy_g;*/
    /**orient = veh.xy_g.o;*/

    /**loc = sim_vehxy[sim_timestep];*/
    /**orient = sim_veho[sim_timestep];*/
}
void set_disc_o(void){
    /*int i;
    float tmp=0;
    float min=INFINITY;
    int o_disc = 0;
    for (i=0;i<8;i++){
        tmp = fabs(veh.xy_g.o-arena.st_headings[i]);
        //printf("%.3f\n",tmp);
        if(tmp<min) {
            o_disc = i;
            min = tmp;
        }

    }*/
    
    /*printf("discrete orient:%i\n",o_disc);*/
}
void vehicle_place_on_grid(void){

    int i = round(veh.xy_g.x/arena.dl);
    int j = round(veh.xy_g.y/arena.dl);

    veh.gridij[0] = i;
    veh.gridij[1] = j;
    set_disc_o();


}
void obstacle_add(float gamma){

    int i = 0;
    int freeslot = -1;
    while(freeslot==-1&&i<OBS_SLOTS){
        if(arena.free_slots[i]==1) freeslot = i;
        i++;
    }
    arena.angles_s[freeslot]=gamma;
    arena.free_slots[freeslot] = 0;
    arena.obs[freeslot].orig = veh.xy_g;
    arena.obs[freeslot].gamma_orig = veh.xy_g.o-gamma;
    arena.obs[freeslot].n_tracked = 0;
    arena.obs[freeslot].xy = vec2d_init(lside,lside);
    arena.obs[freeslot].err = INFINITY;
    arena.sn++;
    /*printf("Obstacle added gamma: %.3f slot: %i\n",gamma,freeslot);*/
}
void obstacle_destroy(int i){

    arena.angles_s[i] = NAN;
    arena.free_slots[i] = 1;
    arena.sn--;
    obstacle obs;
    obs.err = INFINITY;
    arena.obs[i]=obs;
    /*printf("Obstacle destroyed slot: %i\n",i);*/
}
void obstacle_update(float gamma, int i){

    float spread = vec2d_dist(&veh.xy_g,&arena.obs[i].orig);
    /*if(loc.x!=arena.obs[i].orig.x&&loc.y!=arena.obs[i].orig.y){*/


    arena.obs[i].upd = veh.xy_g;
    arena.obs[i].gamma_upd = veh.xy_g.o-gamma;
    arena.angles_s[i] = gamma;
    if(spread>MIN_SPREAD){
        vec2d xy_n = vec2d_triangulate(&arena.obs[i].orig,&arena.obs[i].upd,arena.obs[i].gamma_orig,arena.obs[i].gamma_upd);
        vec2d *xy_p = &arena.obs[i].xy;
        int kk = arena.obs[i].n_tracked;
        arena.obs[i].xy = vec2d_init((xy_n.x+xy_p->x*kk)/(1+kk),(xy_n.y+xy_p->y*kk)/(1+kk));

        arena.obs[i].n_tracked++;

        arena.obs[i].err = vec2d_dist(&xy_n,xy_p);
        /*printf("(%.2f,%.2f),(%.2f,%.2f)\n",arena.obs[i].orig.x,arena.obs[i].orig.y,arena.obs[i].upd.x,arena.obs[i].upd.y);*/
        /*printf("curr at: %.3f,%.3f\n",xy_n.x,xy_n.y);*/
        /*printf("prev at: %.3f,%.3f\n",xy_p->x,xy_p->y);*/
        /*printf("new at: %.3f,%.3f\n",arena.obs[i].xy.x,arena.obs[i].xy.y);*/
        /*printf("Obstacle updated gamma: %.3f slot: %i\n",gamma,i);*/
    }


    if(arena.obs[i].err<MAXERROR&&spread>MIN_SPREAD){
        int loc_x;
        int loc_y;

        int loc_x_tmp;
        int loc_y_tmp;
        if(arena.obs[i].xy.x <= arena.lim && arena.obs[i].xy.y <=arena.lim){
            loc_x = floor(arena.obs[i].xy.x/arena.dl);
            loc_y = floor(arena.obs[i].xy.y/arena.dl);

            int m;
            int n;
            vec2d gridlock;
            float dist;
            int loc_w;
            float sig;

            for(m=-1;m<3;m++){
                for(n=-1;n<3;n++){
                    loc_x_tmp = loc_x+m;
                    loc_y_tmp = loc_y+n;

                    if(loc_x_tmp>=0&&loc_y_tmp>=0&&loc_x_tmp<GRID_RES&&loc_y_tmp<GRID_RES){
                        vec2d_set(&gridlock,arena.gridx_coord[loc_x_tmp],arena.gridy_coord[loc_y_tmp]);
                        dist = vec2d_dist(&gridlock,&arena.obs[i].xy);
                        loc_w = loc_x_tmp*GRID_RES+loc_y_tmp;
                        sig = MAXSCORE/(1+exp(dist*arena.dl12-12));
                        /*sig = 100;*/

                        printf("Weights updated! %i,%i :%f\n",loc_x_tmp,loc_y,sig);

                        arena.grid_weights_obs[loc_w]=\
                            (arena.grid_weights_obs[loc_w]>sig)?\
                            arena.grid_weights_obs[loc_w]:sig;
                    }
                }
            }
            printf("MATCH FOUND\n");
            obstacle_destroy(i);
            obstacle_add(gamma);            
        }
        else{
            obstacle_destroy(i);
        }
        
    }
}
void plan_ahead(int i_loc, int j_loc, int head,float *q){


    int i;
    int i_tmp;
    int j_tmp;
    float min = INFINITY;
    for(i=0;i<7;i++){
        i_tmp = i_loc+arena.st_wp_i[i];
        j_tmp = j_loc+arena.st_wp_j[i];
        if(i_tmp>=0&&i_tmp<GRID_RES&&\
           j_tmp>=0&&j_tmp<GRID_RES){
            
            *q = arena.grid_weights_exp[i_tmp*GRID_RES+j_tmp]+arena.grid_weights_obs[i_tmp*GRID_RES+j_tmp]+arena.scores[8+i-head];
        }
        else{
            *q = INFINITY;
        }
            if(*q<min){
                min = *q;
            }

    }
}

void plan_action(int i_loc, int j_loc, int head,int *best,float *q){


    int i;
    int i_tmp;
    int j_tmp;
    /*float q[8] = {0,0,0,0,0,0,0,0};*/
    float min = INFINITY;
    /*float q_ahead;*/
    for(i=0;i<7;i++){
        i_tmp = i_loc+arena.st_wp_i[i];
        j_tmp = j_loc+arena.st_wp_j[i];
        if(i_tmp>=0&&i_tmp<GRID_RES&&\
           j_tmp>=0&&j_tmp<GRID_RES){
            /*plan_ahead(i_tmp,j_tmp,i,&q_ahead);*/
            
            *q = arena.grid_weights_exp[i_tmp*GRID_RES+j_tmp]+arena.grid_weights_obs[i_tmp*GRID_RES+j_tmp]+arena.scores[8+i-head];
            /*printf("action:%i -> score %f\n, min:%f,best@%i",i,*q,min,*best);*/
        }
        else{
            *q = INFINITY;
        }
            if(*q<min){
                *best = i;
                min = *q;
            }

    }
    /*printf("best action is %i\nmove from %i,%i to %i,%i\n",*best,i_loc,j_loc,i_loc+arena.st_wp_i[*best],j_loc+arena.st_wp_j[*best]);*/
}

/*void move(){}*/

void arena_update(float v[],int n){
    /*printarr_float(v,OBS_SLOTS);*/
    /*printf("%i",n);*/

    /*printf("arena_update v:%.3f n:%i\n",v[0],n);*/
    int i;
    int j;
    if(n>0){
        if(arena.sn == 0){
            for(i=0;i<OBS_SLOTS;i++){
                if (!isnan(v[i])){
                    obstacle_add(v[i]);

                }
            }
            for(i=arena.sn;i<OBS_SLOTS;i++){
                arena.angles_s[i]=NAN;
            }
        }
        else{
            /*arena_report();*/
            for(j=0;j<OBS_SLOTS;j++)arena.angles_d[j] = v[j];
            arena.dn = n;
            angle_matcher();

            for(i = 0;i<arena.n_drops;i++) obstacle_destroy(arena.drop[i]);
            for(i=0;i<arena.n_new;i++) obstacle_add(v[arena.new[i]]);
            for(i=0;i<arena.n_matches;i++) obstacle_update(v[arena.matches_d[i]],arena.matches_s[i]);
        }
    }

}
void navigate(void){
    if (!avoid_nav_init) 
	return;
    request_position();
    float vision_tmp[OBS_SLOTS];
    int n_tmp = 0;
    request_obstacles(vision_tmp,&n_tmp);
    /*printf("navigate() v: %.3f n:%i\n",vision_tmp[0],n_tmp);*/



    /*float range = vec2d_dist(&veh.xy_abs,&veh.wp_abs);*/

    /*if(range<0.7){*/
    /*int best;*/
    /*float q;*/
    /*plan_action(veh.gridij[0],veh.gridij[1],veh.o_disc,&best,&q);*/
    /*int new_i =veh.gridij[0]+arena.st_wp_i[best]; */
    /*int new_j =veh.gridij[1]+arena.st_wp_j[best]; */
    /*veh.gridij[0] = new_i;*/
    /*veh.gridij[1] = new_j;*/

    /*set_discrete_wp(new_i,new_j,arena.st_headings[best]);*/
    /*arena.grid_weights_exp[new_i*GRID_RES+new_j]++;*/
    /*veh.o_disc = best;*/
    /*printf("\n#############%i##############\n",counter_nav);*/
    /*counter_nav++;*/


    /*return;*/

    /*}*/



    arena_update(vision_tmp,n_tmp);
}


void vehicle_cache_init(void){
    request_position();
    vehicle_place_on_grid();
    set_discrete_wp(veh.gridij[0],veh.gridij[1],0);

    printf("Vehicle xy location in ENU: %.3f,%.3f orientation: %.3f\n",veh.xy_abs.x,veh.xy_abs.y,veh.xy_abs.o);
    printf("Vehicle xy location in GRD: %.3f,%.3f orientation: %.3f\n",veh.xy_g.x,veh.xy_g.y,veh.xy_g.o);
}
void init_map(void) {
    int i,j;

    float bord[8];

    float xy[4][2];
    request_arena_coord(bord);
    for(i=0;i<4;i++){
        for(j=0;j<2;j++){
            xy[i][j] = bord[i*2+j];
        }
    }


    


    xy_grd.x = vec2d_norm(vec2d_init(xy[1][0]-xy[0][0],xy[1][1]-xy[0][1]));
    xy_grd.y = vec2d_norm(vec2d_init(xy[3][0]-xy[0][0],xy[3][1]-xy[0][1]));
    xy_grd.offset = vec2d_init(-xy[0][0],-xy[0][1]);

    xy_hme.x = vec2d_norm(vec2d_init(xy[1][0]-xy[0][0],-xy[1][1]+xy[0][1]));
    xy_hme.y = vec2d_norm(vec2d_init(-xy[3][0]+xy[0][0],xy[3][1]-xy[0][1]));
    /*xy_hme.offset = vec2d_sub((*)hme2grd(&xy_grd.offset),(*)vec2d_init(0,0));*/

    vec2d tmp=vec2d_init(0,0);
    tmp = hme2grd_o(&tmp);
    tmp.x = -tmp.x;
    tmp.y = -tmp.y;
    xy_hme.offset = tmp;
    /*xy_hme.x = vec2d_init(1,0);*/
    /*xy_hme.y = vec2d_init(0,1);*/
    /*xy_hme.offset = vec2d_init(0,0);*/

    /*xy_grd.x = hme2grd(&xy_hme.x);*/
    /*xy_grd.y = hme2grd(&xy_hme.y);*/

    lside = sqrt(pow(xy[1][0]-xy[0][0],2)+pow(xy[1][1]-xy[0][1],2));
    eta = atan2(abs(xy[1][1]-xy[0][1]),abs(xy[1][0]-xy[0][0]));
    ceta = cos(eta);
    seta = sin(eta);

    float dl = lside/(GRID_RES-1);
    arena.dl = dl;
    arena.dl12 = 12/dl;
    arena.lim = lside;

    for(i=0;i<GRID_RES;i++){
        arena.gridx_coord[i]=i*dl;
        arena.gridy_coord[i]=i*dl;
    }

    for(i=0;i<GRID_RES*GRID_RES;i++){
        arena.grid_weights_exp[i]=0;
        arena.grid_weights_obs[i]=0;
    }

    for(i=0;i<OBS_SLOTS;i++){
        arena.angles_s[i] = NAN;
        arena.angles_d[i] = NAN;
        arena.free_slots[i]=1;
    }
    
    arena.sn = 0;
    arena.dn = 0;
    arena.maxerr = MAXERROR*dl;
    int st_wp_i[] = {0,1,1,1,0,-1,-1,-1};
    int st_wp_j[] = {1,1,0,-1,-1,-1,0,1};
    
    for(i=0;i<8;i++) arena.st_wp_i[i] = st_wp_i[i];
    for(i=0;i<8;i++) arena.st_wp_j[i] = st_wp_j[i];

    arena.action_scores[0]=0;
    arena.action_scores[1]=1;
    arena.action_scores[2]=3;
    arena.action_scores[3]=5;
    arena.action_scores[4]=4;
    arena.action_scores[5]=5;
    arena.action_scores[6]=3;
    arena.action_scores[7]=1;

    int kk[] = {0,0.5,3,5,4,5,3,0.5,0,0.5,3,5,4,5,3,0.5};
    for(i=0;i<16;i++) arena.scores[i] = kk[i];
    for(i=0;i<8;i++){
        for(j=0;j<8;j++){
            arena.action_scores[i*8+j]=kk[(i+8-j)];
        }
    }

    for(i=0;i<7;i++) arena.st_headings[i]=PI/2-(i)*PI/4.0;
    arena.st_headings[7]=135*PI/180;


    printf("Arena size: %.3fx%.3f m\n",lside,lside);
    

    

    
    /*sim_init();*/
}






void nav_print_full_report(void)
{
#ifdef WITH_NAVIGATION
    printf("Status report:\n");

    printf("Curr. position: (abs,grd,discrete) %.2f,%.2f %.2f,%.2f %i,%i\n",\
    veh.xy_abs.x,veh.xy_abs.y,veh.xy_g.x,veh.xy_g.y,veh.gridij[0],veh.gridij[1]);
    printf("Curr. wp. setting: (abs,grd) %.2f,%.2f,%.2f,%.2f\n",\
    veh.wp_abs.x,veh.xy_abs.y,veh.wp_g.x,veh.wp_g.y);
    float range = vec2d_dist(&veh.xy_abs,&veh.wp_abs);
    printf("Range to next waypoint: %f\n",range);
    printf("Obstacle parallax:\n");
    printf("p0: %f p1: %f\n",arena.angles_d[0]*180/PI,arena.angles_d[1]*180/PI);
    printf("Current field of view:\n");
    printarr_float(arena.angles_s,OBS_SLOTS);
    printf("Currently tracked obstacles:\n");

    int i;
    obstacle o;

    for(i=0;i<arena.sn;i++){
        o = arena.obs[i];
        printf("%i: original fix at %f,%f, latest fix at %f,%f, angle1: %f, angle2: %f current xy fix %f,%f\n)",\
        o.orig.x,o.orig.y,o.upd.x,o.upd.y,o.gamma_orig,o.gamma_upd,o.xy.x,o.xy.y);
    }


    printf("Current obstacle weights\n");

    print2darr_float(arena.grid_weights_obs,GRID_RES,GRID_RES);
    


#endif
}
