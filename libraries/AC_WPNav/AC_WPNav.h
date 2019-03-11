#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/Location.h>
#include <AP_InertialNav/AP_InertialNav.h>     // Inertial Navigation library
#include <AC_AttitudeControl/AC_PosControl.h>      // Position control library
#include <AC_AttitudeControl/AC_AttitudeControl.h> // Attitude control library
#include <AP_Terrain/AP_Terrain.h>
#include <AC_Avoidance/AC_Avoid.h>                 // Stop at fence library

// maximum velocities and accelerations
#define WPNAV_ACCELERATION              100.0f      // defines the default velocity vs distant curve.  maximum acceleration in cm/s/s that position controller asks for from acceleration controller
#define WPNAV_ACCELERATION_MIN           50.0f      // minimum acceleration in cm/s/s - used for sanity checking _wp_accel parameter

#define WPNAV_WP_SPEED                  500.0f      // default horizontal speed between waypoints in cm/s
#define WPNAV_WP_SPEED_MIN               20.0f      // minimum horizontal speed between waypoints in cm/s
#define WPNAV_WP_TRACK_SPEED_MIN         50.0f      // minimum speed along track of the target point the vehicle is chasing in cm/s (used as target slows down before reaching destination)
#define WPNAV_WP_RADIUS                 200.0f      // default waypoint radius in cm
#define WPNAV_WP_RADIUS_MIN              10.0f      // minimum waypoint radius in cm

#define WPNAV_WP_SPEED_UP               250.0f      // default maximum climb velocity
#define WPNAV_WP_SPEED_DOWN             150.0f      // default maximum descent velocity

#define WPNAV_WP_ACCEL_Z_DEFAULT        100.0f      // default vertical acceleration between waypoints in cm/s/s

#define WPNAV_LEASH_LENGTH_MIN          100.0f      // minimum leash lengths in cm

#define WPNAV_WP_FAST_OVERSHOOT_MAX     200.0f      // 2m overshoot is allowed during fast waypoints to allow for smooth transitions to next waypoint

#define WPNAV_YAW_DIST_MIN                 200      // minimum track length which will lead to target yaw being updated to point at next waypoint.  Under this distance the yaw target will be frozen at the current heading
#define WPNAV_YAW_LEASH_PCT_MIN         0.134f      // target point must be at least this distance from the vehicle (expressed as a percentage of the maximum distance it can be from the vehicle - i.e. the leash length)

#define WPNAV_RANGEFINDER_FILT_Z         0.25f      // range finder distance filtered at 0.25hz

class AC_WPNav
{
public:

    // spline segment end types enum
    enum spline_segment_end_type {
        SEGMENT_END_STOP = 0,
        SEGMENT_END_STRAIGHT,
        SEGMENT_END_SPLINE
    };

    /// Constructor
    AC_WPNav(const AP_InertialNav& inav, const AP_AHRS_View& ahrs, AC_PosControl& pos_control, const AC_AttitudeControl& attitude_control);

    /// provide pointer to terrain database
    void set_terrain(AP_Terrain* terrain_ptr) { _terrain = terrain_ptr; }

    /// provide pointer to avoidance library
    void set_avoidance(AC_Avoid* avoid_ptr) { _avoid = avoid_ptr; }

    /// provide rangefinder altitude
    void set_rangefinder_alt(bool use, bool healthy, float alt_cm) { _rangefinder_available = use; _rangefinder_healthy = healthy; _rangefinder_alt_cm = alt_cm; }

    ///
    /// brake controller
    ///
    /// init_brake_target - initialize's position and feed-forward velocity from current pos and velocity
    void init_brake_target(float accel_cmss);
    ///
    /// update_brake - run the brake controller - should be called at 400hz
    void update_brake(float ekfGndSpdLimit, float ekfNavVelGainScaler);

    ///
    /// waypoint controller
    ///

    /// wp_and_spline_init - initialise straight line and spline waypoint controllers
    ///     updates target roll, pitch targets and I terms based on vehicle lean angles
    ///     should be called once before the waypoint controller is used but does not need to be called before subsequent updates to destination
    void wp_and_spline_init()
    {
        // check _wp_accel_cmss is reasonable
        if (_wp_accel_cmss <= 0) {
            _wp_accel_cmss.set_and_save(WPNAV_ACCELERATION);
        }

        // initialise position controller
        _pos_control.set_desired_accel_xy(0.0f,0.0f);
        _pos_control.init_xy_controller();
        _pos_control.clear_desired_velocity_ff_z();

        // initialise feed forward velocity to zero
        _pos_control.set_desired_velocity_xy(0.0f, 0.0f);

        // initialise position controller speed and acceleration
        _pos_control.set_speed_xy(_wp_speed_cms);
        _pos_control.set_accel_xy(_wp_accel_cmss);
        _pos_control.set_speed_z(-_wp_speed_down_cms, _wp_speed_up_cms);
        _pos_control.set_accel_z(_wp_accel_z_cmss);
        _pos_control.calc_leash_length_xy();
        _pos_control.calc_leash_length_z();

        // initialise yaw heading to current heading target
        _flags.wp_yaw_set = false;
    }
    /// set_speed_xy - allows main code to pass target horizontal velocity for wp navigation
    void set_speed_xy(float speed_cms);

    /// get_speed_xy - allows main code to retrieve target horizontal velocity for wp navigation
    float get_speed_xy() const { return _wp_speed_cms; }

    /// get_speed_up - returns target climb speed in cm/s during missions
    float get_speed_up() const { return _wp_speed_up_cms; }

    /// get_speed_down - returns target descent speed in cm/s during missions.  Note: always positive
    float get_speed_down() const { return _wp_speed_down_cms; }

    /// get_speed_z - returns target descent speed in cm/s during missions.  Note: always positive
    float get_accel_z() const { return _wp_accel_z_cmss; }

    /// get_wp_acceleration - returns acceleration in cm/s/s during missions
    float get_wp_acceleration() const { return _wp_accel_cmss.get(); }

    /// get_wp_destination waypoint using position vector (distance from ekf origin in cm)


    const Vector3f &get_wp_destination() const { return _destination; }

    /// get origin using position vector (distance from ekf origin in cm)
    const Vector3f &get_wp_origin() const { return _origin; }

    /// set_wp_destination waypoint using location class
    ///     returns false if conversion from location to vector from ekf origin cannot be calculated
    bool set_wp_destination(const Location_Class& destination)
    {
        bool terr_alt;
        Vector3f dest_neu;

        // convert destination location to vector
        if (!get_vector_NEU(destination, dest_neu, terr_alt)) {
            return false;
        }

        // set target as vector from EKF origin
        return set_wp_destination(dest_neu, terr_alt);
    }
    // returns wp location using location class.
    // returns false if unable to convert from target vector to global
    // coordinates
    bool get_wp_destination(Location_Class& destination);

    /// set_wp_destination waypoint using position vector (distance from ekf origin in cm)
    ///     terrain_alt should be true if destination.z is a desired altitude above terrain
    bool set_wp_destination(const Vector3f& destination, bool terrain_alt = false)
    {
        Vector3f origin;

        // if waypoint controller is active use the existing position target as the origin
        if ((AP_HAL::millis() - _wp_last_update) < 1000) {
            origin = _pos_control.get_pos_target();
        } else {
            // if waypoint controller is not active, set origin to reasonable stopping point (using curr pos and velocity)
            _pos_control.get_stopping_point_xy(origin);
            _pos_control.get_stopping_point_z(origin);
        }

        // convert origin to alt-above-terrain
        if (terrain_alt) {
            float origin_terr_offset;
            if (!get_terrain_offset(origin_terr_offset)) {
                return false;
            }
            origin.z -= origin_terr_offset;
        }

        // set origin and destination
        return set_wp_origin_and_destination(origin, destination, terrain_alt);
    }

    /// set waypoint destination using NED position vector from ekf origin in meters
    bool set_wp_destination_NED(const Vector3f& destination_NED);

    /// set_wp_origin_and_destination - set origin and destination waypoints using position vectors (distance from ekf origin in cm)
    ///     terrain_alt should be true if origin.z and destination.z are desired altitudes above terrain (false if these are alt-above-ekf-origin)
    ///     returns false on failure (likely caused by missing terrain data)
    bool set_wp_origin_and_destination(const Vector3f& origin, const Vector3f& destination, bool terrain_alt = false)
    {
        // store origin and destination locations
        _origin = origin;
        _destination = destination;
        _terrain_alt = terrain_alt;
        Vector3f pos_delta = _destination - _origin;

        _track_length = pos_delta.length(); // get track length
        _track_length_xy = safe_sqrt(sq(pos_delta.x)+sq(pos_delta.y));  // get horizontal track length (used to decide if we should update yaw)

        // calculate each axis' percentage of the total distance to the destination
        if (is_zero(_track_length)) {
            // avoid possible divide by zero
            _pos_delta_unit.x = 0;
            _pos_delta_unit.y = 0;
            _pos_delta_unit.z = 0;
        }else{
            _pos_delta_unit = pos_delta/_track_length;
        }

        // calculate leash lengths
        calculate_wp_leash_length();

        // get origin's alt-above-terrain
        float origin_terr_offset = 0.0f;
        if (terrain_alt) {
            if (!get_terrain_offset(origin_terr_offset)) {
                return false;
            }
        }

        // initialise intermediate point to the origin
        _pos_control.set_pos_target(origin + Vector3f(0,0,origin_terr_offset));
        _track_desired = 0;             // target is at beginning of track
        _flags.reached_destination = false;
        _flags.fast_waypoint = false;   // default waypoint back to slow
        _flags.slowing_down = false;    // target is not slowing down yet
        _flags.segment_type = SEGMENT_STRAIGHT;
        _flags.new_wp_destination = true;   // flag new waypoint so we can freeze the pos controller's feed forward and smooth the transition
        _flags.wp_yaw_set = false;

        // initialise the limited speed to current speed along the track
        const Vector3f &curr_vel = _inav.get_velocity();
        // get speed along track (note: we convert vertical speed into horizontal speed equivalent)
        float speed_along_track = curr_vel.x * _pos_delta_unit.x + curr_vel.y * _pos_delta_unit.y + curr_vel.z * _pos_delta_unit.z;
        _limited_speed_xy_cms = constrain_float(speed_along_track,0,_wp_speed_cms);

        return true;
    }
    /// shift_wp_origin_to_current_pos - shifts the origin and destination so the origin starts at the current position
    ///     used to reset the position just before takeoff
    ///     relies on set_wp_destination or set_wp_origin_and_destination having been called first
    void shift_wp_origin_to_current_pos();

    /// get_wp_stopping_point_xy - calculates stopping point based on current position, velocity, waypoint acceleration
    ///		results placed in stopping_position vector
    void get_wp_stopping_point_xy(Vector3f& stopping_point) const;
    void get_wp_stopping_point(Vector3f& stopping_point) const;

    /// get_wp_distance_to_destination - get horizontal distance to destination in cm
    float get_wp_distance_to_destination() const;

    /// get_bearing_to_destination - get bearing to next waypoint in centi-degrees
    int32_t get_wp_bearing_to_destination() const;

    /// reached_destination - true when we have come within RADIUS cm of the waypoint
    bool reached_wp_destination() const { return _flags.reached_destination; }

    /// set_fast_waypoint - set to true to ignore the waypoint radius and consider the waypoint 'reached' the moment the intermediate point reaches it
    void set_fast_waypoint(bool fast) { _flags.fast_waypoint = fast; }

    /// update_wpnav - run the wp controller - should be called at 100hz or higher
    bool update_wpnav()
    {
        bool ret = true;

        // calculate dt
        float dt = _pos_control.time_since_last_xy_update();
        if (dt >= 0.2f) {
            dt = 0.0f;
        }

        // allow the accel and speed values to be set without changing
        // out of auto mode. This makes it easier to tune auto flight


        _pos_control.set_accel_xy(_wp_accel_cmss);
        _pos_control.set_accel_z(_wp_accel_z_cmss);

        // advance the target if necessary
        if (!advance_wp_target_along_track(dt)) {
            // To-Do: handle inability to advance along track (probably because of missing terrain data)
            ret = false;
        }

        // freeze feedforwards during known discontinuities
        if (_flags.new_wp_destination) {
            _flags.new_wp_destination = false;
            _pos_control.freeze_ff_z();
        }

        _pos_control.update_xy_controller(1.0f);
        check_wp_leash_length();

        _wp_last_update = AP_HAL::millis();

        return ret;
    }
    bool r_update_wpnav()
    {
        bool ret = true;
/*
        // calculate dt
        float dt = _pos_control.time_since_last_xy_update();
        if (dt >= 0.2f) {
            dt = 0.0f;
        }

        // allow the accel and speed values to be set without changing
        // out of auto mode. This makes it easier to tune auto flight


        _pos_control.set_accel_xy(_wp_accel_cmss);
        _pos_control.set_accel_z(_wp_accel_z_cmss);

        // advance the target if necessary
        if (!advance_wp_target_along_track(dt)) {
            // To-Do: handle inability to advance along track (probably because of missing terrain data)
            ret = false;
        }

        // freeze feedforwards during known discontinuities
        if (_flags.new_wp_destination) {
            _flags.new_wp_destination = false;
            _pos_control.freeze_ff_z();
        }

        _pos_control.update_xy_controller(1.0f);
        check_wp_leash_length();

        _wp_last_update = AP_HAL::millis();
*/
        return ret;
    }
    // check_wp_leash_length - check recalc_wp_leash flag and calls calculate_wp_leash_length() if necessary
    //  should be called after _pos_control.update_xy_controller which may have changed the position controller leash lengths
    void check_wp_leash_length()
    {
        // exit immediately if recalc is not required
        if (_flags.recalc_wp_leash) {
            calculate_wp_leash_length();
        }
    }
    /// calculate_wp_leash_length - calculates track speed, acceleration and leash lengths for waypoint controller
    void calculate_wp_leash_length()
    {
        // length of the unit direction vector in the horizontal
        float pos_delta_unit_xy = norm(_pos_delta_unit.x, _pos_delta_unit.y);
        float pos_delta_unit_z = fabsf(_pos_delta_unit.z);

        float speed_z;
        float leash_z;
        if (_pos_delta_unit.z >= 0.0f) {
            speed_z = _wp_speed_up_cms;
            leash_z = _pos_control.get_leash_up_z();
        }else{
            speed_z = _wp_speed_down_cms;
            leash_z = _pos_control.get_leash_down_z();
        }

        // calculate the maximum acceleration, maximum velocity, and leash length in the direction of travel
        if(is_zero(pos_delta_unit_z) && is_zero(pos_delta_unit_xy)){
            _track_accel = 0;
            _track_speed = 0;
            _track_leash_length = WPNAV_LEASH_LENGTH_MIN;
        }else if(is_zero(_pos_delta_unit.z)){
            _track_accel = _wp_accel_cmss/pos_delta_unit_xy;
            _track_speed = _wp_speed_cms/pos_delta_unit_xy;
            _track_leash_length = _pos_control.get_leash_xy()/pos_delta_unit_xy;
        }else if(is_zero(pos_delta_unit_xy)){
            _track_accel = _wp_accel_z_cmss/pos_delta_unit_z;
            _track_speed = speed_z/pos_delta_unit_z;
            _track_leash_length = leash_z/pos_delta_unit_z;
        }else{
            _track_accel = MIN(_wp_accel_z_cmss/pos_delta_unit_z, _wp_accel_cmss/pos_delta_unit_xy);
            _track_speed = MIN(speed_z/pos_delta_unit_z, _wp_speed_cms/pos_delta_unit_xy);
            _track_leash_length = MIN(leash_z/pos_delta_unit_z, _pos_control.get_leash_xy()/pos_delta_unit_xy);
        }

        // calculate slow down distance (the distance from the destination when the target point should begin to slow down)
        calc_slow_down_distance(_track_speed, _track_accel);

        // set recalc leash flag to false
        _flags.recalc_wp_leash = false;
    }
    ///
    /// spline methods
    ///

    // segment start types
    // stop - vehicle is not moving at origin
    // straight-fast - vehicle is moving, previous segment is straight.  vehicle will fly straight through the waypoint before beginning it's spline path to the next wp
    //     _flag.segment_type holds whether prev segment is straight vs spline but we don't know if it has a delay
    // spline-fast - vehicle is moving, previous segment is splined, vehicle will fly through waypoint but previous segment should have it flying in the correct direction (i.e. exactly parallel to position difference vector from previous segment's origin to this segment's destination)

    // segment end types
    // stop - vehicle is not moving at destination
    // straight-fast - next segment is straight, vehicle's destination velocity should be directly along track from this segment's destination to next segment's destination
    // spline-fast - next segment is spline, vehicle's destination velocity should be parallel to position difference vector from previous segment's origin to this segment's destination

    // get target yaw in centi-degrees (used for wp and spline navigation)
    float get_yaw() const
    {
        if (_flags.wp_yaw_set) {
            return _yaw;
        } else {
            // if yaw has not been set return attitude controller's current target
            return _attitude_control.get_att_target_euler_cd().z;
        }
    }

    /// set_spline_destination waypoint using location class
    ///     returns false if conversion from location to vector from ekf origin cannot be calculated
    ///     terrain_alt should be true if destination.z is a desired altitude above terrain (false if its desired altitude above ekf origin)
    ///     stopped_at_start should be set to true if vehicle is stopped at the origin
    ///     seg_end_type should be set to stopped, straight or spline depending upon the next segment's type
    ///     next_destination should be set to the next segment's destination if the seg_end_type is SEGMENT_END_STRAIGHT or SEGMENT_END_SPLINE
    bool set_spline_destination(const Location_Class& destination, bool stopped_at_start, spline_segment_end_type seg_end_type, Location_Class next_destination);

    /// set_spline_destination waypoint using position vector (distance from ekf origin in cm)
    ///     returns false if conversion from location to vector from ekf origin cannot be calculated
    ///     terrain_alt should be true if destination.z is a desired altitudes above terrain (false if its desired altitudes above ekf origin)
    ///     stopped_at_start should be set to true if vehicle is stopped at the origin
    ///     seg_end_type should be set to stopped, straight or spline depending upon the next segment's type
    ///     next_destination should be set to the next segment's destination if the seg_end_type is SEGMENT_END_STRAIGHT or SEGMENT_END_SPLINE
    ///     next_destination.z  must be in the same "frame" as destination.z (i.e. if destination is a alt-above-terrain, next_destination should be too)
    bool set_spline_destination(const Vector3f& destination, bool terrain_alt, bool stopped_at_start, spline_segment_end_type seg_end_type, const Vector3f& next_destination);

    /// set_spline_origin_and_destination - set origin and destination waypoints using position vectors (distance from ekf origin in cm)
    ///     terrain_alt should be true if origin.z and destination.z are desired altitudes above terrain (false if desired altitudes above ekf origin)
    ///     stopped_at_start should be set to true if vehicle is stopped at the origin
    ///     seg_end_type should be set to stopped, straight or spline depending upon the next segment's type
    ///     next_destination should be set to the next segment's destination if the seg_end_type is SEGMENT_END_STRAIGHT or SEGMENT_END_SPLINE
    bool set_spline_origin_and_destination(const Vector3f& origin, const Vector3f& destination, bool terrain_alt, bool stopped_at_start, spline_segment_end_type seg_end_type, const Vector3f& next_destination);

    /// reached_spline_destination - true when we have come within RADIUS cm of the waypoint
    bool reached_spline_destination() const { return _flags.reached_destination; }

    /// update_spline - update spline controller
    bool update_spline();

    bool test_functions(){
        bool ret = true;

                // calculate dt
                float dt = _pos_control.time_since_last_xy_update();
                if (dt >= 0.2f) {
                    dt = 0.0f;
                }

                // allow the accel and speed values to be set without changing
                // out of auto mode. This makes it easier to tune auto flight


                _pos_control.set_accel_xy(_wp_accel_cmss);
                _pos_control.set_accel_z(_wp_accel_z_cmss);

                // advance the target if necessary
                if (!advance_wp_target_along_track(dt)) {
                    // To-Do: handle inability to advance along track (probably because of missing terrain data)
                    ret = false;
                }

                // freeze feedforwards during known discontinuities
                if (_flags.new_wp_destination) {
                    _flags.new_wp_destination = false;
                    _pos_control.freeze_ff_z();
                }

                _pos_control.update_xy_controller(1.0f);
                check_wp_leash_length();

                _wp_last_update = AP_HAL::millis();

                return ret;
    }
    ///
    /// shared methods
    ///

    /// get desired roll, pitch which should be fed into stabilize controllers
    int32_t get_roll() const { return _pos_control.get_roll(); }
    int32_t get_pitch() const { return _pos_control.get_pitch(); }

    /// advance_wp_target_along_track - move target location along track from origin to destination
    bool advance_wp_target_along_track(float dt)
    {
        float track_covered;        // distance (in cm) along the track that the vehicle has traveled.  Measured by drawing a perpendicular line from the track to the vehicle.
        Vector3f track_error;       // distance error (in cm) from the track_covered position (i.e. closest point on the line to the vehicle) and the vehicle
        float track_desired_max;    // the farthest distance (in cm) along the track that the leash will allow
        float track_leash_slack;    // additional distance (in cm) along the track from our track_covered position that our leash will allow
        bool reached_leash_limit = false;   // true when track has reached leash limit and we need to slow down the target point

        // get current location
        Vector3f curr_pos = _inav.get_position();

        // calculate terrain adjustments
        float terr_offset = 0.0f;
        if (_terrain_alt && !get_terrain_offset(terr_offset)) {
            return false;
        }

        // calculate 3d vector from segment's origin
        Vector3f curr_delta = (curr_pos - Vector3f(0,0,terr_offset)) - _origin;

        // calculate how far along the track we are
        track_covered = curr_delta.x * _pos_delta_unit.x + curr_delta.y * _pos_delta_unit.y + curr_delta.z * _pos_delta_unit.z;

        // calculate the point closest to the vehicle on the segment from origin to destination
        Vector3f track_covered_pos = _pos_delta_unit * track_covered;

        // calculate the distance vector from the vehicle to the closest point on the segment from origin to destination
        track_error = curr_delta - track_covered_pos;

        // calculate the horizontal error
        _track_error_xy = norm(track_error.x, track_error.y);

        // calculate the vertical error
        float track_error_z = fabsf(track_error.z);

        // get up leash if we are moving up, down leash if we are moving down
        float leash_z = track_error.z >= 0 ? _pos_control.get_leash_up_z() : _pos_control.get_leash_down_z();

        // use pythagoras's theorem calculate how far along the track we could move the intermediate target before reaching the end of the leash
        //   track_desired_max is the distance from the vehicle to our target point along the track.  It is the "hypotenuse" which we want to be no longer than our leash (aka _track_leash_length)
        //   track_error is the line from the vehicle to the closest point on the track.  It is the "opposite" side
        //   track_leash_slack is the line from the closest point on the track to the target point.  It is the "adjacent" side.  We adjust this so the track_desired_max is no longer than the leash
        float track_leash_length_abs = fabsf(_track_leash_length);
        float track_error_max_abs = MAX(_track_leash_length*track_error_z/leash_z, _track_leash_length*_track_error_xy/_pos_control.get_leash_xy());
        track_leash_slack = (track_leash_length_abs > track_error_max_abs) ? safe_sqrt(sq(_track_leash_length) - sq(track_error_max_abs)) : 0;
        track_desired_max = track_covered + track_leash_slack;

        // check if target is already beyond the leash
        if (_track_desired > track_desired_max) {
            reached_leash_limit = true;
        }

        // get current velocity
        const Vector3f &curr_vel = _inav.get_velocity();
        // get speed along track
        float speed_along_track = curr_vel.x * _pos_delta_unit.x + curr_vel.y * _pos_delta_unit.y + curr_vel.z * _pos_delta_unit.z;

        // calculate point at which velocity switches from linear to sqrt
        float linear_velocity = _wp_speed_cms;
        float kP = _pos_control.get_pos_xy_p().kP();
        if (is_positive(kP)) {   // avoid divide by zero
            linear_velocity = _track_accel/kP;
        }

        // let the limited_speed_xy_cms be some range above or below current velocity along track
        if (speed_along_track < -linear_velocity) {
            // we are traveling fast in the opposite direction of travel to the waypoint so do not move the intermediate point
            _limited_speed_xy_cms = 0;
        }else{
            // increase intermediate target point's velocity if not yet at the leash limit
            if(dt > 0 && !reached_leash_limit) {
                _limited_speed_xy_cms += 2.0f * _track_accel * dt;
            }
            // do not allow speed to be below zero or over top speed
            _limited_speed_xy_cms = constrain_float(_limited_speed_xy_cms, 0.0f, _track_speed);

            // check if we should begin slowing down
            if (!_flags.fast_waypoint) {
                float dist_to_dest = _track_length - _track_desired;
                if (!_flags.slowing_down && dist_to_dest <= _slow_down_dist) {
                    _flags.slowing_down = true;
                }
                // if target is slowing down, limit the speed
                if (_flags.slowing_down) {
                    _limited_speed_xy_cms = MIN(_limited_speed_xy_cms, get_slow_down_speed(dist_to_dest, _track_accel));
                }
            }

            // if our current velocity is within the linear velocity range limit the intermediate point's velocity to be no more than the linear_velocity above or below our current velocity
            if (fabsf(speed_along_track) < linear_velocity) {
                _limited_speed_xy_cms = constrain_float(_limited_speed_xy_cms,speed_along_track-linear_velocity,speed_along_track+linear_velocity);
            }
        }
        // advance the current target
        if (!reached_leash_limit) {
            _track_desired += _limited_speed_xy_cms * dt;

            // reduce speed if we reach end of leash
            if (_track_desired > track_desired_max) {
                _track_desired = track_desired_max;
                _limited_speed_xy_cms -= 2.0f * _track_accel * dt;
                if (_limited_speed_xy_cms < 0.0f) {
                    _limited_speed_xy_cms = 0.0f;
                }
            }
        }

        // do not let desired point go past the end of the track unless it's a fast waypoint
        if (!_flags.fast_waypoint) {
            _track_desired = constrain_float(_track_desired, 0, _track_length);
        } else {
            _track_desired = constrain_float(_track_desired, 0, _track_length + WPNAV_WP_FAST_OVERSHOOT_MAX);
        }

        // recalculate the desired position
        Vector3f final_target = _origin + _pos_delta_unit * _track_desired;
        // convert final_target.z to altitude above the ekf origin
        final_target.z += terr_offset;
        _pos_control.set_pos_target(final_target);

        // check if we've reached the waypoint
        if( !_flags.reached_destination ) {
            if( _track_desired >= _track_length ) {
                // "fast" waypoints are complete once the intermediate point reaches the destination
                if (_flags.fast_waypoint) {
                    _flags.reached_destination = true;
                }else{
                    // regular waypoints also require the copter to be within the waypoint radius
                    Vector3f dist_to_dest = (curr_pos - Vector3f(0,0,terr_offset)) - _destination;
                    if( dist_to_dest.length() <= _wp_radius_cm ) {
                        _flags.reached_destination = true;
                    }
                }
            }
        }

        // update the target yaw if origin and destination are at least 2m apart horizontally
        if (_track_length_xy >= WPNAV_YAW_DIST_MIN) {
            if (_pos_control.get_leash_xy() < WPNAV_YAW_DIST_MIN) {
                // if the leash is short (i.e. moving slowly) and destination is at least 2m horizontally, point along the segment from origin to destination
                set_yaw_cd(get_bearing_cd(_origin, _destination));
            } else {
                Vector3f horiz_leash_xy = final_target - curr_pos;
                horiz_leash_xy.z = 0;
                if (horiz_leash_xy.length() > MIN(WPNAV_YAW_DIST_MIN, _pos_control.get_leash_xy()*WPNAV_YAW_LEASH_PCT_MIN)) {
                    set_yaw_cd(RadiansToCentiDegrees(atan2f(horiz_leash_xy.y,horiz_leash_xy.x)));
                }
            }
        }

        // successfully advanced along track
        return true;
    }

    /// return the crosstrack_error - horizontal error of the actual position vs the desired position
    float crosstrack_error() const { return _track_error_xy;}

    static const struct AP_Param::GroupInfo var_info[];

protected:

    // segment types, either straight or spine
    enum SegmentType {
        SEGMENT_STRAIGHT = 0,
        SEGMENT_SPLINE = 1
    };

    // flags structure
    struct wpnav_flags {
        uint8_t reached_destination     : 1;    // true if we have reached the destination
        uint8_t fast_waypoint           : 1;    // true if we should ignore the waypoint radius and consider the waypoint complete once the intermediate target has reached the waypoint
        uint8_t slowing_down            : 1;    // true when target point is slowing down before reaching the destination
        uint8_t recalc_wp_leash         : 1;    // true if we need to recalculate the leash lengths because of changes in speed or acceleration
        uint8_t new_wp_destination      : 1;    // true if we have just received a new destination.  allows us to freeze the position controller's xy feed forward
        SegmentType segment_type        : 1;    // active segment is either straight or spline
        uint8_t wp_yaw_set              : 1;    // true if yaw target has been set
    } _flags;

    /// calc_slow_down_distance - calculates distance before waypoint that target point should begin to slow-down assuming it is traveling at full speed
    void calc_slow_down_distance(float speed_cms, float accel_cmss)
    {
        // protect against divide by zero
        if (accel_cmss <= 0.0f) {
            _slow_down_dist = 0.0f;
            return;
        }
        // To-Do: should we use a combination of horizontal and vertical speeds?
        // To-Do: update this automatically when speed or acceleration is changed
        _slow_down_dist = speed_cms * speed_cms / (4.0f*accel_cmss);
    }

    /// get_slow_down_speed - returns target speed of target point based on distance from the destination (in cm)
    float get_slow_down_speed(float dist_from_dest_cm, float accel_cmss)
    {
        // return immediately if distance is zero (or less)
        if (dist_from_dest_cm <= 0) {
            return WPNAV_WP_TRACK_SPEED_MIN;
        }

        // calculate desired speed near destination
        float target_speed = safe_sqrt(dist_from_dest_cm * 4.0f * accel_cmss);

        // ensure desired speed never becomes too low
        if (target_speed < WPNAV_WP_TRACK_SPEED_MIN) {
            return WPNAV_WP_TRACK_SPEED_MIN;
        } else {
            return target_speed;
        }
    }

    /// spline protected functions

    /// update_spline_solution - recalculates hermite_spline_solution grid
    void update_spline_solution(const Vector3f& origin, const Vector3f& dest, const Vector3f& origin_vel, const Vector3f& dest_vel);

    /// advance_spline_target_along_track - move target location along track from origin to destination
    ///     returns false if it is unable to advance (most likely because of missing terrain data)
    bool advance_spline_target_along_track(float dt);

    /// calc_spline_pos_vel - update position and velocity from given spline time
    /// 	relies on update_spline_solution being called since the previous
    void calc_spline_pos_vel(float spline_time, Vector3f& position, Vector3f& velocity);

    // get terrain's altitude (in cm above the ekf origin) at the current position (+ve means terrain below vehicle is above ekf origin's altitude)
    bool get_terrain_offset(float& offset_cm)
    {
        // use range finder if connected
        if (_rangefinder_available && _rangefinder_use) {
            if (_rangefinder_healthy) {
                offset_cm = _inav.get_altitude() - _rangefinder_alt_cm;
                return true;
            }
            return false;
        }

    #if AP_TERRAIN_AVAILABLE
        // use terrain database
        float terr_alt = 0.0f;
        if (_terrain != nullptr && _terrain->height_above_terrain(terr_alt, true)) {
            offset_cm = _inav.get_altitude() - (terr_alt * 100.0f);
            return true;
        }
    #endif
        return false;
    }
    // convert location to vector from ekf origin.  terrain_alt is set to true if resulting vector's z-axis should be treated as alt-above-terrain
    //      returns false if conversion failed (likely because terrain data was not available)
    bool get_vector_NEU(const Location_Class &loc, Vector3f &vec, bool &terrain_alt)
    {
        // convert location to NE vector2f
        Vector2f res_vec;
        if (!loc.get_vector_xy_from_origin_NE(res_vec)) {
            return false;
        }

        // convert altitude
        if (loc.get_alt_frame() == Location_Class::ALT_FRAME_ABOVE_TERRAIN) {
            int32_t terr_alt;
            if (!loc.get_alt_cm(Location_Class::ALT_FRAME_ABOVE_TERRAIN, terr_alt)) {
                return false;
            }
            vec.z = terr_alt;
            terrain_alt = true;
        } else {
            terrain_alt = false;
            int32_t temp_alt;
            if (!loc.get_alt_cm(Location_Class::ALT_FRAME_ABOVE_ORIGIN, temp_alt)) {
                return false;
            }
            vec.z = temp_alt;
            terrain_alt = false;
        }

        // copy xy (we do this to ensure we do not adjust vector unless the overall conversion is successful
        vec.x = res_vec.x;
        vec.y = res_vec.y;

        return true;
    }

    // set heading used for spline and waypoint navigation
    void set_yaw_cd(float heading_cd)
    {
        _yaw = heading_cd;
        _flags.wp_yaw_set = true;
    }

    // references and pointers to external libraries
    const AP_InertialNav&   _inav;
    const AP_AHRS_View&     _ahrs;
    AC_PosControl&          _pos_control;
    const AC_AttitudeControl& _attitude_control;
    AP_Terrain              *_terrain = nullptr;
    AC_Avoid                *_avoid = nullptr;

    // parameters
    AP_Float    _wp_speed_cms;          // maximum horizontal speed in cm/s during missions
    AP_Float    _wp_speed_up_cms;       // climb speed target in cm/s
    AP_Float    _wp_speed_down_cms;     // descent speed target in cm/s
    AP_Float    _wp_radius_cm;          // distance from a waypoint in cm that, when crossed, indicates the wp has been reached
    AP_Float    _wp_accel_cmss;          // horizontal acceleration in cm/s/s during missions
    AP_Float    _wp_accel_z_cmss;        // vertical acceleration in cm/s/s during missions

    // waypoint controller internal variables
    uint32_t    _wp_last_update;        // time of last update_wpnav call
    uint8_t     _wp_step;               // used to decide which portion of wpnav controller to run during this iteration
    Vector3f    _origin;                // starting point of trip to next waypoint in cm from ekf origin
    Vector3f    _destination;           // target destination in cm from ekf origin
    Vector3f    _pos_delta_unit;        // each axis's percentage of the total track from origin to destination
    float       _track_error_xy;        // horizontal error of the actual position vs the desired position
    float       _track_length;          // distance in cm between origin and destination
    float       _track_length_xy;       // horizontal distance in cm between origin and destination
    float       _track_desired;         // our desired distance along the track in cm
    float       _limited_speed_xy_cms;  // horizontal speed in cm/s used to advance the intermediate target towards the destination.  used to limit extreme acceleration after passing a waypoint
    float       _track_accel;           // acceleration along track
    float       _track_speed;           // speed in cm/s along track
    float       _track_leash_length;    // leash length along track
    float       _slow_down_dist;        // vehicle should begin to slow down once it is within this distance from the destination

    // spline variables
    float       _spline_time;           // current spline time between origin and destination
    float       _spline_time_scale;     // current spline time between origin and destination
    Vector3f    _spline_origin_vel;     // the target velocity vector at the origin of the spline segment
    Vector3f    _spline_destination_vel;// the target velocity vector at the destination point of the spline segment
    Vector3f    _hermite_spline_solution[4]; // array describing spline path between origin and destination
    float       _spline_vel_scaler;	    //
    float       _yaw;                   // heading according to yaw

    // terrain following variables
    bool        _terrain_alt = false;   // true if origin and destination.z are alt-above-terrain, false if alt-above-ekf-origin
    bool        _ekf_origin_terrain_alt_set = false;
    bool        _rangefinder_available;
    AP_Int8     _rangefinder_use;
    bool        _rangefinder_healthy = false;
    float       _rangefinder_alt_cm = 0.0f;
};
