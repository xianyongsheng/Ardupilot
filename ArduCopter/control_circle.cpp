#include "Copter.h"

/*
 * Init and run calls for circle flight mode
 */

// circle_init - initialise circle controller flight mode
bool Copter::circle_init(bool ignore_checks)
{
    if (position_ok() || ignore_checks) {
        circle_pilot_yaw_override = false;

        // initialize speeds and accelerations
        // 初始化速度和加速度
        pos_control->set_speed_xy(wp_nav->get_speed_xy());
        pos_control->set_accel_xy(wp_nav->get_wp_acceleration());
        pos_control->set_jerk_xy_to_default();
        pos_control->set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
        pos_control->set_accel_z(g.pilot_accel_z);

        // initialise circle controller including setting the circle center based on vehicle speed
        //初始化绕圈控制器，包括基于车辆速度设置圆形中心
        circle_nav->init();

        return true;
    }else{
        return false;
    }
}

// circle_run - runs the circle flight mode
// should be called at 100hz or more
void Copter::circle_run()
{
    float target_yaw_rate = 0;
    float target_climb_rate = 0;

    // initialize speeds and accelerations
    // 初始化速度和加速度
    pos_control->set_speed_xy(wp_nav->get_speed_xy());
    pos_control->set_accel_xy(wp_nav->get_wp_acceleration());
    pos_control->set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control->set_accel_z(g.pilot_accel_z);
    
    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
    // 如果没自动解锁和电机互锁没使能，设置油门为0、立即退出
    if (!motors->armed() || !ap.auto_armed || ap.land_complete || !motors->get_interlock()) {
        // To-Do: add some initialisation of position controllers
#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        // 姿态控制器
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0, 0, 0, get_smoothing_gain());
        attitude_control->set_throttle_out(0,false,g.throttle_filt);
#else
        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        // multicopters do not stabilize roll/pitch/yaw when disarmed
        // 多轴在解除武装时不稳定滚/螺距/偏航
        attitude_control->set_throttle_out_unstabilized(0,true,g.throttle_filt);
#endif
        pos_control->set_alt_target_to_current_alt();
        return;
    }

    // process pilot inputs
    // 处理飞行员输入
    if (!failsafe.radio) {
        // get pilot's desired yaw rate
        // 获取飞行员期望偏航速率
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            circle_pilot_yaw_override = true;
        }

        // get pilot desired climb rate
        // 获取飞行员期望爬升速率
        target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());

        // check for pilot requested take-off
        if (ap.land_complete && target_climb_rate > 0) {
            // indicate we are taking off
            // 表明我们正在起飞
            set_land_complete(false);
            // clear i term when we're taking off
            // 当我们在起飞的时候清除积分项
            set_throttle_takeoff();
        }
    }

    // set motors to full range
    //设置电机满量程
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // run circle controller
    // 绕圈控制器
    circle_nav->update();

    // call attitude controller
    // 姿态控制器
    if (circle_pilot_yaw_override) {
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(circle_nav->get_roll(), circle_nav->get_pitch(), target_yaw_rate, get_smoothing_gain());
    }else{
        attitude_control->input_euler_angle_roll_pitch_yaw(circle_nav->get_roll(), circle_nav->get_pitch(), circle_nav->get_yaw(),true, get_smoothing_gain());
    }

    // adjust climb rate using rangefinder
    // 使用超声调整爬升速率
    if (rangefinder_alt_ok()) {
        // if rangefinder is ok, use surface tracking
        target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control->get_alt_target(), G_Dt);
    }
    // update altitude target and call position controller
    // 更新高度目标和位置控制器
    pos_control->set_alt_target_from_climb_rate(target_climb_rate, G_Dt, false);
    pos_control->update_z_controller();
}
