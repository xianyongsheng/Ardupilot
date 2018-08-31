#include "Copter.h"

static bool land_with_gps;

static uint32_t land_start_time;
static bool land_pause;

// land_init - initialise land controller
bool Copter::land_init(bool ignore_checks)
{
    // check if we have GPS and decide which LAND we're going to do
    // 检查如果我们有GPS和决定哪种下降模式我们将采用
    land_with_gps = position_ok();
    if (land_with_gps) {
        // set target to stopping point
        // 设置目标到终点
        Vector3f stopping_point;
        wp_nav->get_loiter_stopping_point_xy(stopping_point);
        wp_nav->init_loiter_target(stopping_point);
    }

    // initialize vertical speeds and leash lengths
    // 初始化垂直速度和皮带长度
    pos_control->set_speed_z(wp_nav->get_speed_down(), wp_nav->get_speed_up());
    pos_control->set_accel_z(wp_nav->get_accel_z());

    // initialise position and desired velocity
    // 初始化位置和期望速度
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }
    
    land_start_time = millis();

    land_pause = false;

    // reset flag indicating if pilot has applied roll or pitch inputs during landing
    //重置标志，指示飞行员在着陆过程中是否应用了翻滚或俯仰输入
    ap.land_repo_active = false;

    return true;
}

// land_run - runs the land controller
// should be called at 100hz or more
void Copter::land_run()
{
    if (land_with_gps) {
        land_gps_run();
    }else{
        land_nogps_run();
    }
}

// land_run - runs the land controller
//      horizontal position controlled with loiter controller
//      should be called at 100hz or more
void Copter::land_gps_run()
{
    // if not auto armed or landed or motor interlock not enabled set throttle to zero and exit immediately
    if (!motors->armed() || !ap.auto_armed || ap.land_complete || !motors->get_interlock()) {
#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0, 0, 0, get_smoothing_gain());
        attitude_control->set_throttle_out(0,false,g.throttle_filt);
#else
        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        // multicopters do not stabilize roll/pitch/yaw when disarmed
        // 多轴在解除武装时不稳定滚/螺距/偏航
        attitude_control->set_throttle_out_unstabilized(0,true,g.throttle_filt);
#endif
        wp_nav->init_loiter_target();

        // disarm when the landing detector says we've landed
        // 上锁，当降落检测器标志已经降落
        if (ap.land_complete) {
            init_disarm_motors();
        }
        return;
    }
    
    // set motors to full range
    // 设置电机到满量程
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
    
    // pause before beginning land descent
    // 降落之前暂停
    if(land_pause && millis()-land_start_time >= LAND_WITH_DELAY_MS) {
        land_pause = false;
    }
    
    land_run_horizontal_control();
    land_run_vertical_control(land_pause);
}

// land_nogps_run - runs the land controller
//      pilot controls roll and pitch angles
//      should be called at 100hz or more
void Copter::land_nogps_run()
{
    float target_roll = 0.0f, target_pitch = 0.0f;
    float target_yaw_rate = 0;

    // process pilot inputs
    // 处理飞行员输入
    if (!failsafe.radio) {
        if ((g.throttle_behavior & THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND) != 0 && rc_throttle_control_in_filter.get() > LAND_CANCEL_TRIGGER_THR){
            Log_Write_Event(DATA_LAND_CANCELLED_BY_PILOT);
            // exit land if throttle is high
            // 如果油门为高，退出降落
            set_mode(ALT_HOLD, MODE_REASON_THROTTLE_LAND_ESCAPE);
        }

        if (g.land_repositioning) {
            // apply SIMPLE mode transform to pilot inputs
            // 应用简单模式传输到飞行员输入
            update_simple_mode();

            // get pilot desired lean angles
            // 获取飞行员期望倾斜角度
            get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, aparm.angle_max);
        }

        // get pilot's desired yaw rate
        // 获取飞行员期望偏航速率
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
    }

    // if not auto armed or landed or motor interlock not enabled set throttle to zero and exit immediately
    // 如果不是汽车武装或着陆或汽车联锁，则没有使设定节流阀变为零和立即退出
    if (!motors->armed() || !ap.auto_armed || ap.land_complete || !motors->get_interlock()) {
#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
        attitude_control->set_throttle_out(0,false,g.throttle_filt);
#else
        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        // multicopters do not stabilize roll/pitch/yaw when disarmed
        attitude_control->set_throttle_out_unstabilized(0,true,g.throttle_filt);
#endif

        // disarm when the landing detector says we've landed
        // 上锁，当降落检测标志为已降落
        if (ap.land_complete) {
            init_disarm_motors();
        }
        return;
    }

    // set motors to full range
    // 设置电机到满量程
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // call attitude controller
    // 姿态控制器
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

    // pause before beginning land descent
    // 降落之前暂停操作
    if(land_pause && millis()-land_start_time >= LAND_WITH_DELAY_MS) {
        land_pause = false;
    }

    land_run_vertical_control(land_pause);
}

/*
  get a height above ground estimate for landing
 */
int32_t Copter::land_get_alt_above_ground(void)
{
    int32_t alt_above_ground;
    if (rangefinder_alt_ok()) {
        alt_above_ground = rangefinder_state.alt_cm_filt.get();
    } else {
        bool navigating = pos_control->is_active_xy();
        if (!navigating || !current_loc.get_alt_cm(Location_Class::ALT_FRAME_ABOVE_TERRAIN, alt_above_ground)) {
            alt_above_ground = current_loc.alt;
        }
    }
    return alt_above_ground;
}

void Copter::land_run_vertical_control(bool pause_descent)
{
    bool navigating = pos_control->is_active_xy();

#if PRECISION_LANDING == ENABLED
    bool doing_precision_landing = !ap.land_repo_active && precland.target_acquired() && navigating;
#else
    bool doing_precision_landing = false;
#endif

    // compute desired velocity
    // 计算期望速度
    const float precland_acceptable_error = 15.0f;
    const float precland_min_descent_speed = 10.0f;
    int32_t alt_above_ground = land_get_alt_above_ground();

    float cmb_rate = 0;
    if (!pause_descent) {
        float max_land_descent_velocity;
        if (g.land_speed_high > 0) {
            max_land_descent_velocity = -g.land_speed_high;
        } else {
            max_land_descent_velocity = pos_control->get_speed_down();
        }

        // Don't speed up for landing.
        // 不要加速着陆。
        max_land_descent_velocity = MIN(max_land_descent_velocity, -abs(g.land_speed));

        // Compute a vertical velocity demand such that the vehicle approaches LAND_START_ALT. Without the below constraint, this would cause the vehicle to hover at LAND_START_ALT.
		// 计算垂直速度的需求，使车辆接近landstartalt。如果没有下面的限制，这将导致车辆在landstartalt停留。
		cmb_rate = AC_AttitudeControl::sqrt_controller(LAND_START_ALT-alt_above_ground, g.p_alt_hold.kP(), pos_control->get_accel_z());

        // Constrain the demanded vertical velocity so that it is between the configured maximum descent speed and the configured minimum descent speed.
		// 限制要求的垂直速度，使其处于配置的最大下降速度和配置的最小下降速度之间。
		cmb_rate = constrain_float(cmb_rate, max_land_descent_velocity, -abs(g.land_speed));

        if (doing_precision_landing && rangefinder_alt_ok() && rangefinder_state.alt_cm > 35.0f && rangefinder_state.alt_cm < 200.0f) {
            float max_descent_speed = abs(g.land_speed)/2.0f;
            float land_slowdown = MAX(0.0f, pos_control->get_horizontal_error()*(max_descent_speed/precland_acceptable_error));
            cmb_rate = MIN(-precland_min_descent_speed, -max_descent_speed+land_slowdown);
        }
    }

    // update altitude target and call position controller
    //  更新高度目标、运行位置控制器	
    pos_control->set_alt_target_from_climb_rate_ff(cmb_rate, G_Dt, true);
    pos_control->update_z_controller();
}

void Copter::land_run_horizontal_control()
{
    int16_t roll_control = 0, pitch_control = 0;
    float target_yaw_rate = 0;
    
    // relax loiter target if we might be landed
    // 如果我们可能在降落，放松留待目标
    if (ap.land_complete_maybe) {
        wp_nav->loiter_soften_for_landing();
    }
    
    // process pilot inputs
    // 处理飞行员输入
    if (!failsafe.radio) {
        if ((g.throttle_behavior & THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND) != 0 && rc_throttle_control_in_filter.get() > LAND_CANCEL_TRIGGER_THR){
            Log_Write_Event(DATA_LAND_CANCELLED_BY_PILOT);
            // exit land if throttle is high
            if (!set_mode(LOITER, MODE_REASON_THROTTLE_LAND_ESCAPE)) {
                set_mode(ALT_HOLD, MODE_REASON_THROTTLE_LAND_ESCAPE);
            }
        }

        if (g.land_repositioning) {
            // apply SIMPLE mode transform to pilot inputs
            // 应用简单模式传输到飞行员输入
            update_simple_mode();

            // process pilot's roll and pitch input
            // 处理飞行员的 翻滚和俯仰输入
            roll_control = channel_roll->get_control_in();
            pitch_control = channel_pitch->get_control_in();

            // record if pilot has overriden roll or pitch
            // 记录如果飞行员覆盖了翻滚和俯仰
            if (roll_control != 0 || pitch_control != 0) {
                ap.land_repo_active = true;
            }
        }

        // get pilot's desired yaw rate
        // 获取飞行员期望偏航速率
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
    }

#if PRECISION_LANDING == ENABLED
    bool doing_precision_landing = !ap.land_repo_active && precland.target_acquired();
    // run precision landing
    // 运行精密降落
    if (doing_precision_landing) {
        Vector2f target_pos, target_vel_rel;
        if (!precland.get_target_position_cm(target_pos)) {
            target_pos.x = inertial_nav.get_position().x;
            target_pos.y = inertial_nav.get_position().y;
        }
        if (!precland.get_target_velocity_relative_cms(target_vel_rel)) {
            target_vel_rel.x = -inertial_nav.get_velocity().x;
            target_vel_rel.y = -inertial_nav.get_velocity().y;
        }
        pos_control->set_xy_target(target_pos.x, target_pos.y);
        pos_control->override_vehicle_velocity_xy(-target_vel_rel);
    }
#endif
    
    // process roll, pitch inputs
    // 处理翻滚、俯仰输入
    wp_nav->set_pilot_desired_acceleration(roll_control, pitch_control);

    // run loiter controller
    // 留待控制器
    wp_nav->update_loiter(ekfGndSpdLimit, ekfNavVelGainScaler);

    int32_t nav_roll  = wp_nav->get_roll();
    int32_t nav_pitch = wp_nav->get_pitch();

    if (g2.wp_navalt_min > 0) {
        // user has requested an altitude below which navigation
        // attitude is limited. This is used to prevent commanded roll
        // over on landing, which particularly affects helicopters if
        // there is any position estimate drift after touchdown. We
        // limit attitude to 7 degrees below this limit and linearly
        // interpolate for 1m above that
        //用户要求在下面的高度导航
		//态度是有限的。这是用来防止被命令的滚动
		//在着陆时，尤其影响到直升机
		//在着陆后，有任何位置估计漂移。
		//我们将态度限制在这个极限以下7度和线性度
		//在上面插入1 m
        int alt_above_ground = land_get_alt_above_ground();
        float attitude_limit_cd = linear_interpolate(700, aparm.angle_max, alt_above_ground,
                                                     g2.wp_navalt_min*100U, (g2.wp_navalt_min+1)*100U);
        float total_angle_cd = norm(nav_roll, nav_pitch);
        if (total_angle_cd > attitude_limit_cd) {
            float ratio = attitude_limit_cd / total_angle_cd;
            nav_roll *= ratio;
            nav_pitch *= ratio;

            // tell position controller we are applying an external limit
            // 告诉位置控制器，我们应用一个外部限制
            pos_control->set_limit_accel_xy();
        }
    }

    
    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(nav_roll, nav_pitch, target_yaw_rate, get_smoothing_gain());
}

// land_do_not_use_GPS - forces land-mode to not use the GPS but instead rely on pilot input for roll and pitch
//  called during GPS failsafe to ensure that if we were already in LAND mode that we do not use the GPS
//  has no effect if we are not already in LAND mode
void Copter::land_do_not_use_GPS()
{
    land_with_gps = false;
}

// set_mode_land_with_pause - sets mode to LAND and triggers 4 second delay before descent starts
//  this is always called from a failsafe so we trigger notification to pilot
void Copter::set_mode_land_with_pause(mode_reason_t reason)
{
    set_mode(LAND, reason);
    land_pause = true;

    // alert pilot to mode change
    // 警告飞行员模式改变
    AP_Notify::events.failsafe_mode_change = 1;
}

// landing_with_GPS - returns true if vehicle is landing using GPS
bool Copter::landing_with_GPS() {
    return (control_mode == LAND && land_with_gps);
}
