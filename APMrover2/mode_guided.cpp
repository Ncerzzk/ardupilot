#include "mode.h"
#include "Rover.h"

bool ModeGuided::_enter()
{
    // initialise waypoint speed  初始化wp速度，或者cruise 速度。将这些参数赋给_desired_speed.该参数为mode类下的保护变量。
    set_desired_speed_to_default();

    // set desired location to reasonable stopping point
    Location stopping_point;
    calc_stopping_location(_destination);//_destination为mode类下的保护变量，计算目标位置
    set_desired_location(stopping_point);
    //write by shiguang.wu on 2018.10.23
    //define the initial position with lat,longitude.
    original_position.lat=399789466;
    original_position.lng=1163261748;
    original_position.alt=100;
    initial_position=rover.current_loc;
    Vector2f pos_init=location_diff(original_position,rover.current_loc);//从经纬度坐标转换为uwb坐标，并初始化当前点。
    attitude_control.set_pos_init(pos_init);//初始化相应的参数。

    gcs().send_text(MAV_SEVERITY_NOTICE, "go to guide mode!.");
    is_mannual=false;
    return true;
}

void ModeGuided::update()
{
    switch (_guided_mode) {
        case Guided_WP:
        {
            break;
        }


        case Guided_HeadingAndSpeed:
        {
    		g_actual_vx_mp=_desired_yaw_cd;//global variables for upload
    		g_actual_vy_mp=_desired_speed;//global variables for upload
            nav_filter_status filt_status;
            rover.ahrs.get_filter_status(filt_status);

            // check position estimate.  requires origin and at least one horizontal position flag to be true
            Location origin;
            const bool position_ok = ahrs.get_origin(origin) &&
                                    (filt_status.flags.horiz_pos_abs || filt_status.flags.pred_horiz_pos_abs ||
                                     filt_status.flags.horiz_pos_rel || filt_status.flags.pred_horiz_pos_rel);
            // stop vehicle if target not updated within 3 seconds
            if (have_attitude_target && (millis() - _des_att_time_ms) > 1500 && !position_ok && requires_position()) {
                gcs().send_text(MAV_SEVERITY_WARNING, "target not received last 3secs, stopping");
                have_attitude_target = false;
            }
            if (have_attitude_target) {
            	//g2.v_control==1.则进行速度控制，否则直接输出。
            	if(g2.v_control==1)
            	{
        	    Vector2f pos_loc=location_diff(original_position,rover.current_loc);//将经纬度转换为uwb坐标，从而进行下一步的控制。
        	    // position with PID control
        	    Vector2f vel_output=attitude_control.update_x_control(pos_loc);
        	    float vx_global=vel_output.x;
        	    float vy_global=vel_output.y;

        		if(g2.uwbxyz==1){

        			//float angle_error=4.0f;
        			 vx_global=cosf(radians(360-g2.angle_error_uwbtoglo))*vel_output.x+sinf(radians(360-g2.angle_error_uwbtoglo))*vel_output.y;
        			 vy_global=-sinf(radians(360-g2.angle_error_uwbtoglo))*vel_output.x+cosf(radians(360-g2.angle_error_uwbtoglo))*vel_output.y;

        			}

        		float speed_x=vx_global;
        		float speed_y=vy_global;
        		// The global coordinate system convert to the rover coordinate system. write by shiguang.wu on 2018.10.22
                float vx=cosf(radians((36000-g2.angle_error_uwbtoglo-ahrs.yaw_sensor)*0.01f))*speed_x+sinf(radians((36000-g2.angle_error_uwbtoglo-ahrs.yaw_sensor)*0.01f))*speed_y;
                float vy=-sinf(radians((36000-g2.angle_error_uwbtoglo-ahrs.yaw_sensor)*0.01f))*speed_x+cosf(radians((36000-g2.angle_error_uwbtoglo-ahrs.yaw_sensor)*0.01f))*speed_y;
        	    if(g2.para3==1)
        	    {
        	    	g2.motors.set_velocity_x(vx);//output the  x control
        	    	g2.motors.set_velocity_y(vy);//output the y control
        	    }
                if (g2.para1==1){
                //  control the yaw angle with P and PI.
                float yaw_rate=attitude_control.get_yaw(radians((36000-g2.angle_error_uwbtoglo)*0.01f),rover.G_Dt);
                float yaw_out=attitude_control.get_yaw_rate(yaw_rate,rover.G_Dt);
                g2.motors.set_angular_z(yaw_out);//output the yaw control
                }
            	}else{
            		float speed_x=_desired_yaw_cd;
            		float speed_y=_desired_speed;
            		// The global coordinate system convert to the rover coordinate system. write by shiguang.wu on 2018.10.22
                    float vx=cosf(radians((36000-g2.angle_error_uwbtoglo-ahrs.yaw_sensor)*0.01f))*speed_x+sinf(radians((36000-g2.angle_error_uwbtoglo-ahrs.yaw_sensor)*0.01f))*speed_y;
                    float vy=-sinf(radians((36000-g2.angle_error_uwbtoglo-ahrs.yaw_sensor)*0.01f))*speed_x+cosf(radians((36000-g2.angle_error_uwbtoglo-ahrs.yaw_sensor)*0.01f))*speed_y;
            	    if(g2.para3==1)
            	    {
            	    	g2.motors.set_velocity_x(vx);//output the  x control
            	    	g2.motors.set_velocity_y(vy);//output the y control
            	    }
                    if (g2.para1==1){
                    //  control the yaw angle with P and PI.
                    float yaw_rate=attitude_control.get_yaw(radians((36000-g2.angle_error_uwbtoglo)*0.01f),rover.G_Dt);
                    float yaw_out=attitude_control.get_yaw_rate(yaw_rate,rover.G_Dt);
                    g2.motors.set_angular_z(yaw_out);//output the yaw control
                    }
            	}
            }else {
            	       //stop_vehicle();
    	    	g2.motors.set_velocity_x(0.0f);//output the  x control
    	    	g2.motors.set_velocity_y(0.0f);//output the y control
            	g2.motors.set_angular_z(0.0f);//output the yaw control
            	  }

            break;
        }

        case Guided_TurnRateAndSpeed:
        {
            break;
        }

        default:
            gcs().send_text(MAV_SEVERITY_WARNING, "Unknown GUIDED mode");
            break;
    }
}

// return distance (in meters) to destination
float ModeGuided::get_distance_to_destination() const
{
    if (_guided_mode != Guided_WP || _reached_destination) {
        return 0.0f;
    }
    return _distance_to_destination;
}

// set desired location
void ModeGuided::set_desired_location(const struct Location& destination)
{
    // call parent
    Mode::set_desired_location(destination);

    // handle guided specific initialisation and logging
    _guided_mode = ModeGuided::Guided_WP;
    rover.Log_Write_GuidedTarget(_guided_mode, Vector3f(_destination.lat, _destination.lng, 0), Vector3f(_desired_speed, 0.0f, 0.0f));
}

// set desired attitude
void ModeGuided::set_desired_heading_and_speed(float yaw_angle_cd, float target_speed)
{
    // call parent
    Mode::set_desired_heading_and_speed(yaw_angle_cd, target_speed);

    // handle guided specific initialisation and logging
    _guided_mode = ModeGuided::Guided_HeadingAndSpeed;
    _des_att_time_ms = AP_HAL::millis();
    _reached_destination = false;

    // record targets
    attitude_control.set_desired_vel(yaw_angle_cd,target_speed);

    _desired_yaw_cd = yaw_angle_cd;
    _desired_speed = target_speed;
    have_attitude_target = true;

    // log new target
    rover.Log_Write_GuidedTarget(_guided_mode, Vector3f(_desired_yaw_cd, 0.0f, 0.0f), Vector3f(_desired_speed, 0.0f, 0.0f));
}

void ModeGuided::set_desired_heading_delta_and_speed(float yaw_delta_cd, float target_speed)
{
    // handle initialisation
    if (_guided_mode != ModeGuided::Guided_HeadingAndSpeed) {
        _guided_mode = ModeGuided::Guided_HeadingAndSpeed;
        _desired_yaw_cd = ahrs.yaw_sensor;
    }
    set_desired_heading_and_speed(wrap_180_cd(_desired_yaw_cd + yaw_delta_cd), target_speed);
}

// set desired velocity
void ModeGuided::set_desired_turn_rate_and_speed(float turn_rate_cds, float target_speed)
{
    // handle initialisation
    _guided_mode = ModeGuided::Guided_TurnRateAndSpeed;
    _des_att_time_ms = AP_HAL::millis();
    _reached_destination = false;

    // record targets
    _desired_yaw_rate_cds = turn_rate_cds;
    _desired_speed = target_speed;
    have_attitude_target = true;

    // log new target
    rover.Log_Write_GuidedTarget(_guided_mode, Vector3f(_desired_yaw_rate_cds, 0.0f, 0.0f), Vector3f(_desired_speed, 0.0f, 0.0f));
}
