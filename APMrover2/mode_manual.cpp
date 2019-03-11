#include "mode.h"
#include "Rover.h"

void ModeManual::_exit()
{
    // clear lateral when exiting manual mode
    g2.motors.set_lateral(0);
     is_mannual=false;
}

void ModeManual::update()
{
    float desired_steering, desired_throttle, desired_lateral;

    //stm32 control write by Shiguang wu
    if (rover.failsafe.bits & FAILSAFE_EVENT_THROTTLE) {
    	desired_steering = 0;
    	desired_throttle = 0;
    	desired_lateral=0;
    }else{
    	//get the input of Remote
    	desired_throttle=rover.channel_throttle->get_control_in();
    	desired_lateral=rover.channel_lateral->get_control_in();
    	desired_steering=rover.channel_steer->get_control_in();
    }
    //control the rover with Vx,Vy,Vz.
    g2.motors.set_velocity_x(desired_lateral/100.0f);
    g2.motors.set_velocity_y(desired_throttle/100.0f);
    g2.motors.set_angular_z(desired_steering/450.0f);
    is_mannual=true;


}
