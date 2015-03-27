#include "avoid_nav.h"
#include <time.h>
    float map_borders[4][2] = {
        {-2,-6},
        {6,-2},
        {2,6},
        {-6,2}};

    float home_pos[2] = {0,0};
    float mav_xy[2] = {2,6};
int
main (void)
{
    /*sayhello();*/

    

    /*int i;*/
    /*double time_spent;*/
    /*clock_t begin, end;*/




    /*begin = clock();*/
    printf("##########################################\n");
    printf("##########################################\n");
    printf("Running simulation!\n");
    printf("##########################################\n");
    printf("##########################################\n");


    init_map();
    vehicle_sim_init();
    vehicle_cache_init();
    obstacle_sim_init();
    printf("Vehicle grid position(cached,sim)\n");
    print_vec2d(veh.xy_g);
    print_vec2d(v_sim.xy_g);
    printf("##########################################\n");
    printf("##########################################\n");
    printf("NAVIGATING\n");
    printf("##########################################\n");
    printf("##########################################\n");
    long kk;
    for(kk=0;kk<40;kk++){
        vehicle_sim();
        navigate();
        /*printf("\nVehicle grid position(cached,sim)\n");*/
        /*print_vec2d(veh.xy_g);*/
        /*print_vec2d(v_sim.xy_g);*/
    }



    printf("Vehicle absolute position(cached,sim)\n");
    print_vec2d(veh.xy_abs);
    print_vec2d(v_sim.xy_abs);

    /*init_map(map_borders, home_pos);*/
    /*end = clock();*/
    /*time_spent = (double)(end - begin) / CLOCKS_PER_SEC;*/
    /*printf("Initialization time:%f\n",time_spent);*/
    /*for(i=0;i<SIM_TIME;i++){*/
    /*begin = clock();*/
    /*sim_timestep=i;*/
    /*navigate();*/
    /*end = clock();*/
    /*time_spent = (double)(end - begin) / CLOCKS_PER_SEC;*/
    /*printf("Step %i time:%f\n",i,time_spent);*/
    /*}*/


    /*printf("Testing coordinate conversion:\nldim:%f,eta:%f\n",lside,180/PI*eta);*/

    /*vec2d vec[5];*/

    /*vec[0] = vec2d_init_o(-2,-6,PI/4);*/
    /*vec[1] = vec2d_init_o(6,-2,PI/4);*/
    /*vec[2] = vec2d_init_o(2,6,PI/4);*/
    /*vec[3] = vec2d_init_o(-6,2,PI/4);*/
    /*vec[4] = vec2d_init_o(0,0,PI/4);*/
    /*printf("Original vectors\n");*/
    /*for(i=0;i<5;i++){*/
    /*print_vec2d(vec[i]);*/
    /*}*/

    /*printf("Converting to grid\n");*/
    /*for(i=0;i<5;i++){*/
    /*vec[i]=hme2grd_o(&vec[i]);*/
    /*print_vec2d(vec[i]);*/
    /*}*/
    /*printf("Converting back to home\n");*/
    /*for(i=0;i<5;i++){*/
    /*vec[i]=grd2hme_o(&vec[i]);*/
    /*print_vec2d(vec[i]);*/
    /*}*/


    /*printf("\n\n\n\n##############\n\nSimulating vehicle model\n");*/

    /*vehicle_sim_init();*/
    /*obstacle_sim_init();*/
    /*printf("Obstacles initizlized at:\n");*/
    /*print_vec2d(sim_obstacle[0].xy);*/
    /*print_vec2d(sim_obstacle[1].xy);*/
    /*printf("\n");*/
    /**//*float wp[] = {0,0};*/
    /**//*float o = PI/2+eta;*/
    /**//*set_wp(wp,&o);*/
    /*float vv[2];*/
    /*int n=2;*/
    /*int j;*/
    /**//*while(wp_status(){*/
    /*printf("veh track\n");*/
    /*print_vec2d(veh.xy_g);*/
    /*printf("veh sim\n");*/
    /*print_vec2d(v_sim.xy_g);*/
    /*for(i=0;i<10;i++){*/
    /*vehicle_sim();*/
    /*obstacle_sim_return_angle(vv,&n);*/
    /*navigate();*/


    /*for(j=0;j<SIM_OBSTACLES;j++){*/
    /**//*printf("Angle to obstacle %i: %f\n",j,vv[j]*180/PI);*/
    /*}*/
    /**//*printf("\n");*/

    /*}*/


    /*printf("\n\n\n");*/

    /*v_sim.xy_g.x = 0;*/
    /*v_sim.xy_g.y = 0;*/
    /*obstacle_sim_return_angle(vv,&n);*/
    printf("\n\n\n");

    print2darr_float(arena.grid_weights_obs,GRID_RES,GRID_RES);
    printf("\n\n\n");

    print2darr_float(arena.grid_weights_exp,GRID_RES,GRID_RES);
    printarr_float(arena.st_headings,8);

    /*int best;*/
    /*float q;*/
    /*plan_action(9,0,5,&best,&q);*/

    /*set_discrete_wp(19,19,0);*/


    /*print2darr_float(arena.action_scores,8,8);*/
    /*vec2d aa = vec2d_init(1,0);*/
    /*vec2d bb = vec2d_init(1,1);*/

    /*print_vec2d(aa);*/
    /*print_vec2d(bb);*/


    /*vec2d_a_q(&aa);*/
    /*vec2d_cs(&bb);*/

    /*float angle = vec2d_ang_q(&aa,&bb);*/
    /*printf("aa: mag %f, c1 %f, s1 %f, qw, %f, qy %f, a %f\n",aa.mag,aa.c1,aa.s1,aa.q.w,aa.q.y,\*/
    /*aa.angle);*/
    /*printf("bb: mag %f, c1 %f, s1 %f, qw, %f, qy %f, a %f\n",bb.mag,bb.c1,bb.s1,bb.q.w,bb.q.y,\*/
    /*bb.angle);*/
    /**/
    /**/
    /*printf("Angle check: %f\n",angle*180/PI);*/


  return 0;
}
