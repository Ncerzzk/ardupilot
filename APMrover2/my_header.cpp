/*
 * my_header.cpp
 *
 *  Created on: 2018-10-15
 *      Author: Shiguang.Wu
 */


    float throttle_temp;
    float lateral_temp;
    float distance_wp_x;
    float distance_wp_y;
    //float distance;
    float desired_vx;
    float desired_vy;
    float display1;
    float display2;
    float display3;
    float v_dt_display;
    bool ch5_input;
    int ch8_input;
    float desired_x_dt;
    float desired_y_dt;
    float desired_v_x[100];
    float desired_v_y[100];
    float g_actual_velocity_x;
    float g_actual_velocity_y;
    float g_desired_yaw_log;
    float g_actual_yaw_log;
    float update_dt_display;
    float guide_goal_pos_x;
    float guide_goal_pos_y;
    float guide_pos_x;
    float guide_pos_y;
    float g_desired_pos_x_log;
    float g_actual_pos_x_log;
    float g_desired_pos_y_log;
    float g_actual_pos_y_log;
    float g_pi_desired_x;
    float g_pi_desired_y;
    float acro_vx;
    float acro_vy;
    float g_actual_vx;//uwb coordination velocity vx;
    float g_actual_vy;//uwb coordination velocity vy;
    float g_actual_vx_mp;//the current control velocity of rover.
    float g_actual_vy_mp;//the current control velocity of rover.
    bool is_mannual;
