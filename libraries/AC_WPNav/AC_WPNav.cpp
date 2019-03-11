#include <AP_HAL/AP_HAL.h>
#include "AC_WPNav.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AC_WPNav::var_info[] = {
    // index 0 was used for the old orientation matrix

    // @Param: SPEED
    // @DisplayName: Waypoint Horizontal Speed Target
    // @Description: Defines the speed in cm/s which the aircraft will attempt to maintain horizontally during a WP mission
    // @Units: cm/s
    // @Range: 20 2000
    // @Increment: 50
    // @User: Standard
    AP_GROUPINFO("SPEED",       0, AC_WPNav, _wp_speed_cms, WPNAV_WP_SPEED),

    // @Param: RADIUS
    // @DisplayName: Waypoint Radius
    // @Description: Defines the distance from a waypoint, that when crossed indicates the wp has been hit.
    // @Units: cm
    // @Range: 10 1000
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("RADIUS",      1, AC_WPNav, _wp_radius_cm, WPNAV_WP_RADIUS),

    // @Param: SPEED_UP
    // @DisplayName: Waypoint Climb Speed Target
    // @Description: Defines the speed in cm/s which the aircraft will attempt to maintain while climbing during a WP mission
    // @Units: cm/s
    // @Range: 10 1000
    // @Increment: 50
    // @User: Standard
    AP_GROUPINFO("SPEED_UP",    2, AC_WPNav, _wp_speed_up_cms, WPNAV_WP_SPEED_UP),

    // @Param: SPEED_DN
    // @DisplayName: Waypoint Descent Speed Target
    // @Description: Defines the speed in cm/s which the aircraft will attempt to maintain while descending during a WP mission
    // @Units: cm/s
    // @Range: 10 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("SPEED_DN",    3, AC_WPNav, _wp_speed_down_cms, WPNAV_WP_SPEED_DOWN),

    // @Param: ACCEL
    // @DisplayName: Waypoint Acceleration 
    // @Description: Defines the horizontal acceleration in cm/s/s used during missions
    // @Units: cm/s/s
    // @Range: 50 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("ACCEL",       5, AC_WPNav, _wp_accel_cmss, WPNAV_ACCELERATION),

    // @Param: ACCEL_Z
    // @DisplayName: Waypoint Vertical Acceleration
    // @Description: Defines the vertical acceleration in cm/s/s used during missions
    // @Units: cm/s/s
    // @Range: 50 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("ACCEL_Z",     6, AC_WPNav, _wp_accel_z_cmss, WPNAV_WP_ACCEL_Z_DEFAULT),

    // @Param: RFND_USE
    // @DisplayName: Waypoint missions use rangefinder for terrain following
    // @Description: This controls if waypoint missions use rangefinder for terrain following
    // @Values: 0:Disable,1:Enable
    // @User: Advanced
    AP_GROUPINFO("RFND_USE",   10, AC_WPNav, _rangefinder_use, 1),

    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
AC_WPNav::AC_WPNav(const AP_InertialNav& inav, const AP_AHRS_View& ahrs, AC_PosControl& pos_control, const AC_AttitudeControl& attitude_control) :
    _inav(inav),
    _ahrs(ahrs),
    _pos_control(pos_control),
    _attitude_control(attitude_control)
{
    AP_Param::setup_object_defaults(this, var_info);

    // init flags
    _flags.reached_destination = false;
    _flags.fast_waypoint = false;
    _flags.slowing_down = false;
    _flags.recalc_wp_leash = false;
    _flags.new_wp_destination = false;
    _flags.segment_type = SEGMENT_STRAIGHT;

    // sanity check some parameters
    _wp_accel_cmss = MIN(_wp_accel_cmss, GRAVITY_MSS * 100.0f * tanf(ToRad(_attitude_control.lean_angle_max() * 0.01f)));
    _wp_radius_cm = MAX(_wp_radius_cm, WPNAV_WP_RADIUS_MIN);
}


/// init_brake_target - initializes stop position from current position and velocity
void AC_WPNav::init_brake_target(float accel_cmss)
{
    const Vector3f& curr_vel = _inav.get_velocity();
    Vector3f stopping_point;

    // initialise position controller
    _pos_control.set_desired_velocity_xy(0.0f,0.0f);
    _pos_control.set_desired_accel_xy(0.0f,0.0f);
    _pos_control.init_xy_controller();

    // initialise pos controller speed and acceleration
    _pos_control.set_speed_xy(curr_vel.length());
    _pos_control.set_accel_xy(accel_cmss);
    _pos_control.calc_leash_length_xy();

    // set target position
    _pos_control.get_stopping_point_xy(stopping_point);
    _pos_control.set_xy_target(stopping_point.x, stopping_point.y);
}

// update_brake - run the stop controller - gets called at 400hz
void AC_WPNav::update_brake(float ekfGndSpdLimit, float ekfNavVelGainScaler)
{
    // send adjusted feed forward velocity back to position controller
    _pos_control.set_desired_velocity_xy(0.0f, 0.0f);
    _pos_control.update_xy_controller(ekfNavVelGainScaler);
}

///
/// waypoint navigation
///

/// wp_and_spline_init - initialise straight line and spline waypoint controllers
///     updates target roll, pitch targets and I terms based on vehicle lean angles
///     should be called once before the waypoint controller is used but does not need to be called before subsequent updates to destination


/// set_speed_xy - allows main code to pass target horizontal velocity for wp navigation
void AC_WPNav::set_speed_xy(float speed_cms)
{
    // range check new target speed and update position controller
    if (speed_cms >= WPNAV_WP_SPEED_MIN) {
        _wp_speed_cms = speed_cms;
        _pos_control.set_speed_xy(_wp_speed_cms);
        // flag that wp leash must be recalculated
        _flags.recalc_wp_leash = true;
    }
}

/// set_wp_destination waypoint using location class
///     returns false if conversion from location to vector from ekf origin cannot be calculated


bool AC_WPNav::get_wp_destination(Location_Class& destination) {
    Vector3f dest = get_wp_destination();
    if (!AP::ahrs().get_origin(destination)) {
        return false;
    }
    destination.offset(dest.x*0.01f, dest.y*0.01f);
    destination.alt += dest.z;
    return true;
}

/// set_wp_destination waypoint using position vector (distance from home in cm)
///     terrain_alt should be true if destination.z is a desired altitude above terrain


/// set waypoint destination using NED position vector from ekf origin in meters
bool AC_WPNav::set_wp_destination_NED(const Vector3f& destination_NED)
{
    // convert NED to NEU and do not use terrain following
    return set_wp_destination(Vector3f(destination_NED.x * 100.0f, destination_NED.y * 100.0f, -destination_NED.z * 100.0f), false);
}

/// set_origin_and_destination - set origin and destination waypoints using position vectors (distance from home in cm)
///     terrain_alt should be true if origin.z and destination.z are desired altitudes above terrain (false if these are alt-above-ekf-origin)
///     returns false on failure (likely caused by missing terrain data)


/// shift_wp_origin_to_current_pos - shifts the origin and destination so the origin starts at the current position
///     used to reset the position just before takeoff
///     relies on set_wp_destination or set_wp_origin_and_destination having been called first
void AC_WPNav::shift_wp_origin_to_current_pos()
{
    // return immediately if vehicle is not at the origin
    if (_track_desired > 0.0f) {
        return;
    }

    // get current and target locations
    const Vector3f curr_pos = _inav.get_position();
    const Vector3f pos_target = _pos_control.get_pos_target();

    // calculate difference between current position and target
    Vector3f pos_diff = curr_pos - pos_target;

    // shift origin and destination
    _origin += pos_diff;
    _destination += pos_diff;

    // move pos controller target and disable feed forward
    _pos_control.set_pos_target(curr_pos);
    _pos_control.freeze_ff_z();
}

/// get_wp_stopping_point_xy - returns vector to stopping point based on a horizontal position and velocity
void AC_WPNav::get_wp_stopping_point_xy(Vector3f& stopping_point) const
{
	_pos_control.get_stopping_point_xy(stopping_point);
}

/// get_wp_stopping_point - returns vector to stopping point based on 3D position and velocity
void AC_WPNav::get_wp_stopping_point(Vector3f& stopping_point) const
{
    _pos_control.get_stopping_point_xy(stopping_point);
    _pos_control.get_stopping_point_z(stopping_point);
}

/// advance_wp_target_along_track - move target location along track from origin to destination


/// get_wp_distance_to_destination - get horizontal distance to destination in cm
float AC_WPNav::get_wp_distance_to_destination() const
{
    // get current location
    Vector3f curr = _inav.get_position();
    return norm(_destination.x-curr.x,_destination.y-curr.y);
}

/// get_wp_bearing_to_destination - get bearing to next waypoint in centi-degrees
int32_t AC_WPNav::get_wp_bearing_to_destination() const
{
    return get_bearing_cd(_inav.get_position(), _destination);
}


/// update_wpnav - run the wp controller - should be called at 100hz or higher




/// calculate_wp_leash_length - calculates horizontal and vertical leash lengths for waypoint controller


// returns target yaw in centi-degrees (used for wp and spline navigation)


// set heading used for spline and waypoint navigation

///
/// spline methods
///

/// set_spline_destination waypoint using location class
///     returns false if conversion from location to vector from ekf origin cannot be calculated
///     stopped_at_start should be set to true if vehicle is stopped at the origin
///     seg_end_type should be set to stopped, straight or spline depending upon the next segment's type
///     next_destination should be set to the next segment's destination if the seg_end_type is SEGMENT_END_STRAIGHT or SEGMENT_END_SPLINE
bool AC_WPNav::set_spline_destination(const Location_Class& destination, bool stopped_at_start, spline_segment_end_type seg_end_type, Location_Class next_destination)
{
    // convert destination location to vector
    Vector3f dest_neu;
    bool dest_terr_alt;
    if (!get_vector_NEU(destination, dest_neu, dest_terr_alt)) {
        return false;
    }

    // make altitude frames consistent
    if (!next_destination.change_alt_frame(destination.get_alt_frame())) {
        return false;
    }

    // convert next destination to vector
    Vector3f next_dest_neu;
    bool next_dest_terr_alt;
    if (!get_vector_NEU(next_destination, next_dest_neu, next_dest_terr_alt)) {
        return false;
    }

    // set target as vector from EKF origin
    return set_spline_destination(dest_neu, dest_terr_alt, stopped_at_start, seg_end_type, next_dest_neu);
}

/// set_spline_destination waypoint using position vector (distance from home in cm)
///     returns false if conversion from location to vector from ekf origin cannot be calculated
///     terrain_alt should be true if destination.z is a desired altitudes above terrain (false if its desired altitudes above ekf origin)
///     stopped_at_start should be set to true if vehicle is stopped at the origin
///     seg_end_type should be set to stopped, straight or spline depending upon the next segment's type
///     next_destination should be set to the next segment's destination if the seg_end_type is SEGMENT_END_STRAIGHT or SEGMENT_END_SPLINE
bool AC_WPNav::set_spline_destination(const Vector3f& destination, bool terrain_alt, bool stopped_at_start, spline_segment_end_type seg_end_type, const Vector3f& next_destination)
{
    Vector3f origin;

    // if waypoint controller is active and copter has reached the previous waypoint use current pos target as the origin
    if ((AP_HAL::millis() - _wp_last_update) < 1000) {
        origin = _pos_control.get_pos_target();
    }else{
        // otherwise calculate origin from the current position and velocity
        _pos_control.get_stopping_point_xy(origin);
        _pos_control.get_stopping_point_z(origin);
    }

    // convert origin to alt-above-terrain
    if (terrain_alt) {
        float terr_offset;
        if (!get_terrain_offset(terr_offset)) {
            return false;
        }
        origin.z -= terr_offset;
    }

    // set origin and destination
    return set_spline_origin_and_destination(origin, destination, terrain_alt, stopped_at_start, seg_end_type, next_destination);
}

/// set_spline_origin_and_destination - set origin and destination waypoints using position vectors (distance from home in cm)
///     terrain_alt should be true if origin.z and destination.z are desired altitudes above terrain (false if desired altitudes above ekf origin)
///     seg_type should be calculated by calling function based on the mission
bool AC_WPNav::set_spline_origin_and_destination(const Vector3f& origin, const Vector3f& destination, bool terrain_alt, bool stopped_at_start, spline_segment_end_type seg_end_type, const Vector3f& next_destination)
{
    // mission is "active" if wpnav has been called recently and vehicle reached the previous waypoint
    bool prev_segment_exists = (_flags.reached_destination && ((AP_HAL::millis() - _wp_last_update) < 1000));

    float dt = _pos_control.time_since_last_xy_update();
    if (dt >= 0.2f) {
        dt = 0.0f;
    }

    // check _wp_accel_cmss is reasonable to avoid divide by zero
    if (_wp_accel_cmss <= 0) {
        _wp_accel_cmss.set_and_save(WPNAV_ACCELERATION);
    }

    // segment start types
    // stop - vehicle is not moving at origin
    // straight-fast - vehicle is moving, previous segment is straight.  vehicle will fly straight through the waypoint before beginning it's spline path to the next wp
    //     _flag.segment_type holds whether prev segment is straight vs spline but we don't know if it has a delay
    // spline-fast - vehicle is moving, previous segment is splined, vehicle will fly through waypoint but previous segment should have it flying in the correct direction (i.e. exactly parallel to position difference vector from previous segment's origin to this segment's destination)

    // calculate spline velocity at origin
    if (stopped_at_start || !prev_segment_exists) {
    	// if vehicle is stopped at the origin, set origin velocity to 0.02 * distance vector from origin to destination
    	_spline_origin_vel = (destination - origin) * dt;
    	_spline_time = 0.0f;
    	_spline_vel_scaler = 0.0f;
    }else{
    	// look at previous segment to determine velocity at origin
        if (_flags.segment_type == SEGMENT_STRAIGHT) {
            // previous segment is straight, vehicle is moving so vehicle should fly straight through the origin
            // before beginning it's spline path to the next waypoint. Note: we are using the previous segment's origin and destination
            _spline_origin_vel = (_destination - _origin);
            _spline_time = 0.0f;	// To-Do: this should be set based on how much overrun there was from straight segment?
            _spline_vel_scaler = _pos_control.get_vel_target().length();    // start velocity target from current target velocity
        }else{
            // previous segment is splined, vehicle will fly through origin
            // we can use the previous segment's destination velocity as this segment's origin velocity
            // Note: previous segment will leave destination velocity parallel to position difference vector
            //       from previous segment's origin to this segment's destination)
            _spline_origin_vel = _spline_destination_vel;
            if (_spline_time > 1.0f && _spline_time < 1.1f) {    // To-Do: remove hard coded 1.1f
                _spline_time -= 1.0f;
            }else{
                _spline_time = 0.0f;
            }
            // Note: we leave _spline_vel_scaler as it was from end of previous segment
        }
    }

    // calculate spline velocity at destination
    switch (seg_end_type) {

    case SEGMENT_END_STOP:
        // if vehicle stops at the destination set destination velocity to 0.02 * distance vector from origin to destination
        _spline_destination_vel = (destination - origin) * dt;
        _flags.fast_waypoint = false;
        break;

    case SEGMENT_END_STRAIGHT:
        // if next segment is straight, vehicle's final velocity should face along the next segment's position
        _spline_destination_vel = (next_destination - destination);
        _flags.fast_waypoint = true;
        break;

    case SEGMENT_END_SPLINE:
        // if next segment is splined, vehicle's final velocity should face parallel to the line from the origin to the next destination
        _spline_destination_vel = (next_destination - origin);
        _flags.fast_waypoint = true;
        break;
    }

    // code below ensures we don't get too much overshoot when the next segment is short
    float vel_len = _spline_origin_vel.length() + _spline_destination_vel.length();
    float pos_len = (destination - origin).length() * 4.0f;
    if (vel_len > pos_len) {
        // if total start+stop velocity is more than twice position difference
        // use a scaled down start and stop velocityscale the  start and stop velocities down
        float vel_scaling = pos_len / vel_len;
        // update spline calculator
        update_spline_solution(origin, destination, _spline_origin_vel * vel_scaling, _spline_destination_vel * vel_scaling);
    }else{
        // update spline calculator
        update_spline_solution(origin, destination, _spline_origin_vel, _spline_destination_vel);
    }

    // store origin and destination locations
    _origin = origin;
    _destination = destination;
    _terrain_alt = terrain_alt;

    // calculate slow down distance
    calc_slow_down_distance(_wp_speed_cms, _wp_accel_cmss);

    // get alt-above-terrain
    float terr_offset = 0.0f;
    if (terrain_alt) {
        if (!get_terrain_offset(terr_offset)) {
            return false;
        }
    }

    // initialise intermediate point to the origin
    _pos_control.set_pos_target(origin + Vector3f(0,0,terr_offset));
    _flags.reached_destination = false;
    _flags.segment_type = SEGMENT_SPLINE;
    _flags.new_wp_destination = true;   // flag new waypoint so we can freeze the pos controller's feed forward and smooth the transition
    _flags.wp_yaw_set = false;

    // initialise yaw related variables
    _track_length_xy = safe_sqrt(sq(_destination.x - _origin.x)+sq(_destination.y - _origin.y));  // horizontal track length (used to decide if we should update yaw)

    return true;
}

/// update_spline - update spline controller
bool AC_WPNav::update_spline()
{
    // exit immediately if this is not a spline segment
    if (_flags.segment_type != SEGMENT_SPLINE) {
        return false;
    }

    bool ret = true;

    // calculate dt
    float dt = _pos_control.time_since_last_xy_update();
    if (dt >= 0.2f) {
        dt = 0.0f;
    }

    // advance the target if necessary
    if (!advance_spline_target_along_track(dt)) {
        // To-Do: handle failure to advance along track (due to missing terrain data)
        ret = false;
    }

    // freeze feedforwards during known discontinuities
    if (_flags.new_wp_destination) {
        _flags.new_wp_destination = false;
        _pos_control.freeze_ff_z();
    }

    // run horizontal position controller
    _pos_control.update_xy_controller(1.0f);

    _wp_last_update = AP_HAL::millis();

    return ret;
}

/// update_spline_solution - recalculates hermite_spline_solution grid
///		relies on _spline_origin_vel, _spline_destination_vel and _origin and _destination
void AC_WPNav::update_spline_solution(const Vector3f& origin, const Vector3f& dest, const Vector3f& origin_vel, const Vector3f& dest_vel)
{
    _hermite_spline_solution[0] = origin;
    _hermite_spline_solution[1] = origin_vel;
    _hermite_spline_solution[2] = -origin*3.0f -origin_vel*2.0f + dest*3.0f - dest_vel;
    _hermite_spline_solution[3] = origin*2.0f + origin_vel -dest*2.0f + dest_vel;
 }

/// advance_spline_target_along_track - move target location along track from origin to destination
bool AC_WPNav::advance_spline_target_along_track(float dt)
{
    if (!_flags.reached_destination) {
        Vector3f target_pos, target_vel;

        // update target position and velocity from spline calculator
        calc_spline_pos_vel(_spline_time, target_pos, target_vel);

        // if target velocity is zero the origin and destination must be the same
        // so flag reached destination (and protect against divide by zero)
        float target_vel_length = target_vel.length();
        if (is_zero(target_vel_length)) {
            _flags.reached_destination = true;
            return true;
        }

        _pos_delta_unit = target_vel / target_vel_length;
        calculate_wp_leash_length();

        // get current location
        Vector3f curr_pos = _inav.get_position();

        // get terrain altitude offset for origin and current position (i.e. change in terrain altitude from a position vs ekf origin)
        float terr_offset = 0.0f;
        if (_terrain_alt && !get_terrain_offset(terr_offset)) {
            return false;
        }

        // calculate position error
        Vector3f track_error = curr_pos - target_pos;
        track_error.z -= terr_offset;

        // calculate the horizontal error
        _track_error_xy = norm(track_error.x, track_error.y);

        // calculate the vertical error
        float track_error_z = fabsf(track_error.z);

        // get position control leash lengths
        float leash_xy = _pos_control.get_leash_xy();
        float leash_z;
        if (track_error.z >= 0) {
            leash_z = _pos_control.get_leash_up_z();
        }else{
            leash_z = _pos_control.get_leash_down_z();
        }

        // calculate how far along the track we could move the intermediate target before reaching the end of the leash
        float track_leash_slack = MIN(_track_leash_length*(leash_z-track_error_z)/leash_z, _track_leash_length*(leash_xy-_track_error_xy)/leash_xy);
        if (track_leash_slack < 0.0f) {
            track_leash_slack = 0.0f;
        }

        // update velocity
        float spline_dist_to_wp = (_destination - target_pos).length();
        float vel_limit = _wp_speed_cms;
        if (!is_zero(dt)) {
            vel_limit = MIN(vel_limit, track_leash_slack/dt);
        }

        // if within the stopping distance from destination, set target velocity to sqrt of distance * 2 * acceleration
        if (!_flags.fast_waypoint && spline_dist_to_wp < _slow_down_dist) {
            _spline_vel_scaler = safe_sqrt(spline_dist_to_wp * 2.0f * _wp_accel_cmss);
        }else if(_spline_vel_scaler < vel_limit) {
            // increase velocity using acceleration
            _spline_vel_scaler += _wp_accel_cmss * dt;
        }

        // constrain target velocity
        _spline_vel_scaler = constrain_float(_spline_vel_scaler, 0.0f, vel_limit);

        // scale the spline_time by the velocity we've calculated vs the velocity that came out of the spline calculator
        _spline_time_scale = _spline_vel_scaler / target_vel_length;

        // update target position
        target_pos.z += terr_offset;
        _pos_control.set_pos_target(target_pos);

        // update the target yaw if origin and destination are at least 2m apart horizontally
        if (_track_length_xy >= WPNAV_YAW_DIST_MIN) {
            if (_pos_control.get_leash_xy() < WPNAV_YAW_DIST_MIN) {
                // if the leash is very short (i.e. flying at low speed) use the target point's velocity along the track
                if (!is_zero(target_vel.x) && !is_zero(target_vel.y)) {
                    set_yaw_cd(RadiansToCentiDegrees(atan2f(target_vel.y,target_vel.x)));
                }
            } else {
                // point vehicle along the leash (i.e. point vehicle towards target point on the segment from origin to destination)
                float track_error_xy_length = safe_sqrt(sq(track_error.x)+sq(track_error.y));
                if (track_error_xy_length > MIN(WPNAV_YAW_DIST_MIN, _pos_control.get_leash_xy()*WPNAV_YAW_LEASH_PCT_MIN)) {
                    // To-Do: why is track_error sign reversed?
                    set_yaw_cd(RadiansToCentiDegrees(atan2f(-track_error.y,-track_error.x)));
                }
            }
        }

        // advance spline time to next step
        _spline_time += _spline_time_scale*dt;

        // we will reach the next waypoint in the next step so set reached_destination flag
        // To-Do: is this one step too early?
        if (_spline_time >= 1.0f) {
            _flags.reached_destination = true;
        }
    }
    return true;
}

// calc_spline_pos_vel_accel - calculates target position, velocity and acceleration for the given "spline_time"
/// 	relies on update_spline_solution being called when the segment's origin and destination were set
void AC_WPNav::calc_spline_pos_vel(float spline_time, Vector3f& position, Vector3f& velocity)
{
    float spline_time_sqrd = spline_time * spline_time;
    float spline_time_cubed = spline_time_sqrd * spline_time;

    position = _hermite_spline_solution[0] + \
               _hermite_spline_solution[1] * spline_time + \
               _hermite_spline_solution[2] * spline_time_sqrd + \
               _hermite_spline_solution[3] * spline_time_cubed;

    velocity = _hermite_spline_solution[1] + \
               _hermite_spline_solution[2] * 2.0f * spline_time + \
               _hermite_spline_solution[3] * 3.0f * spline_time_sqrd;
}

// get terrain's altitude (in cm above the ekf origin) at the current position (+ve means terrain below vehicle is above ekf origin's altitude)


// convert location to vector from ekf origin.  terrain_alt is set to true if resulting vector's z-axis should be treated as alt-above-terrain
//      returns false if conversion failed (likely because terrain data was not available)


///
/// shared methods
///

/// calc_slow_down_distance - calculates distance before waypoint that target point should begin to slow-down assuming it is travelling at full speed


/// get_slow_down_speed - returns target speed of target point based on distance from the destination (in cm)

