#include "mode.h"
#include "Rover.h"

void ModeAcro::update()
{ 	//D channels switch
	is_mannual=false;
	if(ch5_input==true){
	//get actual velocity in the global coordinate.
	Vector2f speed_g;
	if(!attitude_control.get_global_speed(speed_g))
	{return;}
	g_actual_velocity_x=speed_g.x;
	g_actual_velocity_y=speed_g.y;
	///////////////////

		/////velocity control/////
		//when g2.para5==0,is linear velocity control
		// get the desired velocity  in the global or UWB coordinate system defined by mp parameters write by shiguang.wu 2018.10.22
		float vx_goal=attitude_control.get_desired_vx();
		float vy_goal=attitude_control.get_desired_vy();

		//update the desired velocity in the global or UWB coordinate system,write by shiguang.wu on 2018.10.22
		//when g2.para5==1,is circular velocity control
		if(g2.para5==1){
			vx_goal=desired_x_dt;
			vy_goal=desired_y_dt;
		}
		g_actual_vx_mp=vx_goal;//global variables for upload
		g_actual_vy_mp=vy_goal;//global variables for upload
	    attitude_control.set_desired_vel(vx_goal,vy_goal);
	    Vector2f pos_loc=location_diff(original_position,rover.current_loc);
	    guide_pos_x=pos_loc.y;
	    guide_pos_y=pos_loc.x;
	    // position with PID control
	    Vector2f vel_output=attitude_control.update_x_control(pos_loc);
		// The UWB coordinate system convert to the global coordinate system. write by shiguang.wu on 2018.10.22
	    float vx_global=vel_output.x;
	    float vy_global=vel_output.y;
		if(g2.uwbxyz==1){
			 vx_global=cosf(radians(360-g2.angle_error_uwbtoglo))*vel_output.x+sinf(radians(360-g2.angle_error_uwbtoglo))*vel_output.y;
			 vy_global=-sinf(radians(360-g2.angle_error_uwbtoglo))*vel_output.x+cosf(radians(360-g2.angle_error_uwbtoglo))*vel_output.y;
			}

		float speed_x=vx_global;
		float speed_y=vy_global;
		///////////////////////////////////////////
		//control the velocity with PID, where speed_x denotes the velocity output in the global coordinate system.speed_y in the same.
		// The global coordinate system convert to the rover coordinate system. write by shiguang.wu on 2018.10.22
        float vx=cosf(radians((36000-g2.angle_error_uwbtoglo-ahrs.yaw_sensor)*0.01f))*speed_x+sinf(radians((36000-g2.angle_error_uwbtoglo-ahrs.yaw_sensor)*0.01f))*speed_y;
        float vy=-sinf(radians((36000-g2.angle_error_uwbtoglo-ahrs.yaw_sensor)*0.01f))*speed_x+cosf(radians((36000-g2.angle_error_uwbtoglo-ahrs.yaw_sensor)*0.01f))*speed_y;
	    if(g2.para3==1)
	    {
	    	g2.motors.set_velocity_x(vx);//output the  x control
	    	g2.motors.set_velocity_y(vy);//output the y control
	    	lateral_temp=vx;//used for display in mp
	    	throttle_temp=-vy;//used for display in mp
	    }
        if (g2.para1==1){
        //  control the yaw angle with P and PI.
        float yaw_rate=attitude_control.get_yaw(radians((36000-g2.angle_error_uwbtoglo)*0.01f),rover.G_Dt);
        float yaw_out=attitude_control.get_yaw_rate(yaw_rate,rover.G_Dt);
        g2.motors.set_angular_z(yaw_out);//output the yaw control

        }
   }else{

	   acro_vx=0;
	   acro_vy=0;
   }
}

bool ModeAcro::requires_velocity() const
{
    return g2.motors.have_skid_steering()? false: true;
}
