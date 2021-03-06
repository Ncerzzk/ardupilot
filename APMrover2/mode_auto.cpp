#include "mode.h"
#include "Rover.h"

// constructor
ModeAuto::ModeAuto(ModeRTL& mode_rtl) :
    _mode_rtl(mode_rtl)
{
}

bool ModeAuto::_enter()
{
    // fail to enter auto if no mission commands
    if (mission.num_commands() == 0) {
        gcs().send_text(MAV_SEVERITY_NOTICE, "No Mission. Can't set AUTO.");
        return false;
    }

    // initialise waypoint speed
    set_desired_speed_to_default();

    // init location target
    set_desired_location(rover.current_loc);
    //r_wp_nav->wp_and_spline_init();
    // other initialisation
    auto_triggered = false;
    _yaw_angle_start=(ahrs.yaw_sensor);

    // restart mission processing
    mission.start_or_resume();
    gcs().send_text(MAV_SEVERITY_NOTICE, "enter modeAuto!");
    return true;
}

void ModeAuto::_exit()
{
    // stop running the mission
    if (mission.state() == AP_Mission::MISSION_RUNNING) {
        mission.stop();
    }
}

void ModeAuto::update()
{

    switch (_submode) {

       case Auto_WP:
        {
           //gcs().send_text(MAV_SEVERITY_NOTICE, "enter auto_mp!");
            _distance_to_destination = get_distance(rover.current_loc, _destination);
            const bool near_wp = _distance_to_destination <= rover.g.waypoint_radius;
            // check if we've reached the destination
            if (!_reached_destination && (near_wp || location_passed_point(rover.current_loc, _origin, _destination))) {
                // trigger reached
                _reached_destination = true;
                gcs().send_text(MAV_SEVERITY_NOTICE, "arrived destination!");
            }
            // determine if we should keep navigating
            if (!_reached_destination || (rover.is_boat() && !near_wp)) {
             Vector2f vector_diff=location_diff(rover.current_loc, _destination);
             distance_wp_x=vector_diff.y;
             distance_wp_y=vector_diff.x;

             int x_sign,y_sign;
             if(vector_diff.x > 0)
             {	  x_sign =1;}
            	 else{
            		 x_sign=-1;
             }
             if(vector_diff.y > 0)
             {	  y_sign =1;}
            	 else{
            		 y_sign=-1;
             }
             float throttle_max=50;
             float lateral_max=50;
             if(_distance_to_destination<0.5)
             { 	throttle_max=30;
             	 lateral_max=30;}
             float throttle_out_y=50*(abs(vector_diff.x)/_distance_to_destination) *x_sign;
             float lateral_out_x=50*(abs(vector_diff.y)/_distance_to_destination)* y_sign;
             float throttle_out =200*attitude_control.get_throttle_out(vector_diff,rover.G_Dt);
             float lateral_out =200*attitude_control.get_lateral_out(vector_diff,rover.G_Dt);

             if (throttle_out>throttle_max)
            	 {throttle_out=throttle_max;}
             else if(throttle_out<-throttle_max)
        	 	 {throttle_out=-throttle_max;}
             else
             { }
             if (lateral_out>lateral_max)
            	 {lateral_out=lateral_max;}
             else if(lateral_out<-lateral_max)
        	 	 {lateral_out=-lateral_max;}
             else
             { }
             float vx=cosf((36000-ahrs.yaw_sensor)*3.1415926/18000)*lateral_out+sinf((36000-ahrs.yaw_sensor)*3.1415926/18000)*throttle_out;
             float vy=-sinf((36000-ahrs.yaw_sensor)*3.1415926/18000)*lateral_out+cosf((36000-ahrs.yaw_sensor)*3.1415926/18000)*throttle_out;
             float error_angle=(_yaw_angle_start- ahrs.yaw_sensor);
             throttle_temp=vx;
             lateral_temp=-vy;
             g2.motors.set_lateral(vx);
             g2.motors.set_throttle(-vy);
             if(error_angle>18000)
             {  g2.motors.set_steering(36000-error_angle);}
             else if(error_angle<-18000)
             {   g2.motors.set_steering(-36000-error_angle);}
             else
             {  g2.motors.set_steering(-error_angle);}


             //gogo2:
             //+ is anticlockwise,
             //g2.motors.set_steering(1000);
            } else {
                // we have reached the destination so stop
                stop_vehicle();
            }
            break;
        }

        //L1 control
/*
        case Auto_WP:
        {
            _distance_to_destination = get_distance(rover.current_loc, _destination);
            const bool near_wp = _distance_to_destination <= rover.g.waypoint_radius;
            // check if we've reached the destination
            if (!_reached_destination && (near_wp || location_passed_point(rover.current_loc, _origin, _destination))) {
                // trigger reached
                _reached_destination = true;
            }
            // determine if we should keep navigating
            if (!_reached_destination || (rover.is_boat() && !near_wp)) {
                // continue driving towards destination
                calc_steering_to_waypoint(_reached_destination ? rover.current_loc : _origin, _destination, _reversed);
                calc_throttle(calc_reduced_speed_for_turn_or_distance(_reversed ? -_desired_speed : _desired_speed), true, false);
            } else {
                // we have reached the destination so stop
                stop_vehicle();
            }
            break;
        }*/

        case Auto_HeadingAndSpeed:
        {
            if (!_reached_heading) {
                // run steering and throttle controllers
                calc_steering_to_heading(_desired_yaw_cd, _desired_speed < 0);
                calc_throttle(_desired_speed, true, false);
                // check if we have reached within 5 degrees of target
                _reached_heading = (fabsf(_desired_yaw_cd - ahrs.yaw_sensor) < 500);
            } else {
                stop_vehicle();
            }
            break;
        }

        case Auto_RTL:
            _mode_rtl.update();
            break;
    }
}

// return distance (in meters) to destination
float ModeAuto::get_distance_to_destination() const
{
    if (_submode == Auto_RTL) {
        return _mode_rtl.get_distance_to_destination();
    }
    return _distance_to_destination;
}
///////////////////////*****************************************//////////////////////////////
///////////////////////*****************************************//////////////////////////////
///////////////////////*****************************************//////////////////////////////
//////////new set desired location for rover
void ModeAuto::r_set_desired_location(const Location_Class& dest_class,const struct Location& destination, float next_leg_bearing_cd)
{
    // call parent
    Mode::set_desired_location(destination, next_leg_bearing_cd);
    r_wp_nav->set_wp_destination(dest_class);

    _submode = Auto_WP;
}

// set desired location to drive to
void ModeAuto::set_desired_location(const struct Location& destination, float next_leg_bearing_cd)
{
    // call parent
    Mode::set_desired_location(destination, next_leg_bearing_cd);

    _submode = Auto_WP;
}

// return true if vehicle has reached or even passed destination
bool ModeAuto::reached_destination()
{
    if (_submode == Auto_WP) {
        return _reached_destination;
    }
    if (_submode == Auto_RTL) {
        return _mode_rtl.reached_destination();
    }
    // we should never reach here but just in case, return true to allow missions to continue
    return true;
}

// set desired heading in centidegrees (vehicle will turn to this heading)
void ModeAuto::set_desired_heading_and_speed(float yaw_angle_cd, float target_speed)
{
    // call parent
    Mode::set_desired_heading_and_speed(yaw_angle_cd, target_speed);

    _submode = Auto_HeadingAndSpeed;
    _reached_heading = false;
}

// return true if vehicle has reached desired heading
bool ModeAuto::reached_heading()
{
    if (_submode == Auto_HeadingAndSpeed) {
        return _reached_heading;
    }
    // we should never reach here but just in case, return true to allow missions to continue
    return true;
}

// start RTL (within auto)
void ModeAuto::start_RTL()
{
    if (_mode_rtl.enter()) {
        _submode = Auto_RTL;
    }
}

// execute the mission in reverse (i.e. backing up)
void ModeAuto::set_reversed(bool value)
{
    if (_reversed != value) {
        _reversed = value;
    }
}

/*
    check for triggering of start of auto mode
*/
bool ModeAuto::check_trigger(void)
{
    // check for user pressing the auto trigger to off
    if (auto_triggered && g.auto_trigger_pin != -1 && rover.check_digital_pin(g.auto_trigger_pin) == 1) {
        gcs().send_text(MAV_SEVERITY_WARNING, "AUTO triggered off");
        auto_triggered = false;
        return false;
    }

    // if already triggered, then return true, so you don't
    // need to hold the switch down
    if (auto_triggered) {
        return true;
    }

    // return true if auto trigger and kickstart are disabled
    if (g.auto_trigger_pin == -1 && is_zero(g.auto_kickstart)) {
        // no trigger configured - let's go!
        auto_triggered = true;
        return true;
    }

    // check if trigger pin has been pushed
    if (g.auto_trigger_pin != -1 && rover.check_digital_pin(g.auto_trigger_pin) == 0) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Triggered AUTO with pin");
        auto_triggered = true;
        return true;
    }

    // check if mission is started by giving vehicle a kick with acceleration > AUTO_KICKSTART
    if (!is_zero(g.auto_kickstart)) {
        const float xaccel = rover.ins.get_accel().x;
        if (xaccel >= g.auto_kickstart) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Triggered AUTO xaccel=%.1f", static_cast<double>(xaccel));
            auto_triggered = true;
            return true;
        }
    }

    return false;
}

void ModeAuto::calc_throttle(float target_speed, bool nudge_allowed, bool avoidance_enabled)
{
    // If not autostarting set the throttle to minimum
    if (!check_trigger()) {
        stop_vehicle();
        return;
    }
    Mode::calc_throttle(target_speed, nudge_allowed, avoidance_enabled);
}
