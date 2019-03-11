/*
 * my_header.h
 *
 *  Created on: 2018-10-15
 *      Author: Shiguang.Wu
 */

#ifndef MY_HEADER_H_
#define MY_HEADER_H_


extern float throttle_temp;
extern float lateral_temp;
extern float distance_wp_x;
extern float distance_wp_y;
//extern float distance;
extern float desired_vx;
extern float desired_vy;
extern float display1;
extern float display2;
extern float display3;
extern float v_dt_display;
extern bool ch5_input;
extern int ch8_input;
extern float desired_x_dt;
extern float desired_y_dt;
extern float desired_v_x[100];
extern float desired_v_y[100];
extern float g_actual_velocity_x;
extern float g_actual_velocity_y;
extern float g_desired_yaw_log;
extern float g_actual_yaw_log;
extern float update_dt_display;
extern float guide_goal_pos_x;
extern float guide_goal_pos_y;
extern float guide_pos_x;
extern float guide_pos_y;
extern float g_desired_pos_x_log;
extern float g_actual_pos_x_log;
extern float g_desired_pos_y_log;
extern float g_actual_pos_y_log;
extern float g_pi_desired_x;
extern float g_pi_desired_y;
extern float acro_vx;
extern float acro_vy;
extern float g_actual_vy;//uwb coordination velocity vy;
extern float g_actual_vx;//uwb coordination velocity vx;
extern float g_actual_vx_mp;//the current control velocity of rover.
extern float g_actual_vy_mp;//the current control velocity of rover.
extern bool is_mannual;

#endif /* MY_HEADER_H_ */
