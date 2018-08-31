#include "Copter.h"

#define ARM_DELAY               20  // called at 10hz so 2 seconds
#define DISARM_DELAY            20  // called at 10hz so 2 seconds
#define AUTO_TRIM_DELAY         100 // called at 10hz so 10 seconds
#define LOST_VEHICLE_DELAY      10  // called at 10hz so 1 second

static uint32_t auto_disarm_begin;

// arm_motors_check - checks for pilot input to arm or disarm the copter
// called at 10hz
//解锁检测
void Copter::arm_motors_check()
{
    static int16_t arming_counter;

    // ensure throttle is down
    // 确保油门拉低
    if (channel_throttle->get_control_in() > 0) {
        arming_counter = 0;
        return;
    }

    int16_t tmp = channel_yaw->get_control_in();

    // full right
    //满右 - 上锁
    if (tmp > 4000) {

        // increase the arming counter to a maximum of 1 beyond the auto trim counter
        //将解锁计数器增加到最大的1，超出自动微调计数器
        if( arming_counter <= AUTO_TRIM_DELAY ) {
            arming_counter++;
        }

        // arm the motors and configure for flight
        // 解锁电机和为飞行做准备
        if (arming_counter == ARM_DELAY && !motors->armed()) {
            // reset arming counter if arming fail
            // 如果解锁失败，复位计数器
            if (!init_arm_motors(false)) {
                arming_counter = 0;
            }
        }

        // arm the motors and configure for flight
        // 当模式为稳定模式时，超过10S，解锁电机和为飞行做准备
        if (arming_counter == AUTO_TRIM_DELAY && motors->armed() && control_mode == STABILIZE) {
            auto_trim_counter = 250;
            // ensure auto-disarm doesn't trigger immediately
            //确保自动解除武装不会立即触发
            auto_disarm_begin = millis();
        }

    // full left
    // 满左 - 上锁
    }else if (tmp < -4000) {
    	//不是自稳和ACRO模式并且没有触地，不允许上锁
        if (!mode_has_manual_throttle(control_mode) && !ap.land_complete) {
            arming_counter = 0;
            return;
        }

        // increase the counter to a maximum of 1 beyond the disarm delay
        if( arming_counter <= DISARM_DELAY ) {
            arming_counter++;
        }

        // disarm the motors
        // 触地并且大于2秒，上锁
        if (arming_counter == DISARM_DELAY && motors->armed()) {
            init_disarm_motors();
        }

    // Yaw is centered so reset arming counter
    // 如果偏航居中，复位解锁计数器
    }else{
        arming_counter = 0;
    }
}

// auto_disarm_check - disarms the copter if it has been sitting on the ground in manual mode with throttle low for at least 15 seconds
void Copter::auto_disarm_check()
{
    uint32_t tnow_ms = millis();
    uint32_t disarm_delay_ms = 1000*constrain_int16(g.disarm_delay, 0, 127);

    // exit immediately if we are already disarmed, or if auto
    // disarming is disabled
    if (!motors->armed() || disarm_delay_ms == 0 || control_mode == THROW) {
        auto_disarm_begin = tnow_ms;
        return;
    }

#if FRAME_CONFIG == HELI_FRAME
    // if the rotor is still spinning, don't initiate auto disarm
    if (motors->rotor_speed_above_critical()) {
        auto_disarm_begin = tnow_ms;
        return;
    }
#endif

    // always allow auto disarm if using interlock switch or motors are Emergency Stopped
    if ((ap.using_interlock && !motors->get_interlock()) || ap.motor_emergency_stop) {
#if FRAME_CONFIG != HELI_FRAME
        // use a shorter delay if using throttle interlock switch or Emergency Stop, because it is less
        // obvious the copter is armed as the motors will not be spinning
        disarm_delay_ms /= 2;
#endif
    } else {
        bool sprung_throttle_stick = (g.throttle_behavior & THR_BEHAVE_FEEDBACK_FROM_MID_STICK) != 0;
        bool thr_low;
        if (mode_has_manual_throttle(control_mode) || !sprung_throttle_stick) {
            thr_low = ap.throttle_zero;
        } else {
            float deadband_top = channel_throttle->get_control_mid() + g.throttle_deadzone;
            thr_low = channel_throttle->get_control_in() <= deadband_top;
        }

        if (!thr_low || !ap.land_complete) {
            // reset timer
            auto_disarm_begin = tnow_ms;
        }
    }

    // disarm once timer expires
    if ((tnow_ms-auto_disarm_begin) >= disarm_delay_ms) {
        init_disarm_motors();
        auto_disarm_begin = tnow_ms;
    }
}

// init_arm_motors - performs arming process including initialisation of barometer and gyros
//  returns false if arming failed because of pre-arm checks, arming checks or a gyro calibration failure
bool Copter::init_arm_motors(bool arming_from_gcs)
{
    static bool in_arm_motors = false;

    // exit immediately if already in this function
    if (in_arm_motors) {
        return false;
    }
    in_arm_motors = true;

    // return true if already armed
    if (motors->armed()) {
        in_arm_motors = false;
        return true;
    }

    // run pre-arm-checks and display failures
    if (!arming.all_checks_passing(arming_from_gcs)) {
        AP_Notify::events.arming_failed = true;
        in_arm_motors = false;
        return false;
    }

    // let dataflash know that we're armed (it may open logs e.g.)
    DataFlash_Class::instance()->set_vehicle_armed(true);

    // disable cpu failsafe because initialising everything takes a while
    failsafe_disable();

    // reset battery failsafe
    set_failsafe_battery(false);

    // notify that arming will occur (we do this early to give plenty of warning)
    AP_Notify::flags.armed = true;
    // call update_notify a few times to ensure the message gets out
    for (uint8_t i=0; i<=10; i++) {
        update_notify();
    }

#if HIL_MODE != HIL_MODE_DISABLED || CONFIG_HAL_BOARD == HAL_BOARD_SITL
    gcs_send_text(MAV_SEVERITY_INFO, "Arming motors");
#endif

    // Remember Orientation
    // --------------------
    init_simple_bearing();

    initial_armed_bearing = ahrs.yaw_sensor;

    if (ap.home_state == HOME_UNSET) {
        // Reset EKF altitude if home hasn't been set yet (we use EKF altitude as substitute for alt above home)
        ahrs.resetHeightDatum();
        Log_Write_Event(DATA_EKF_ALT_RESET);
    } else if (ap.home_state == HOME_SET_NOT_LOCKED) {
        // Reset home position if it has already been set before (but not locked)
        set_home_to_current_location();
    }
    calc_distance_and_bearing();

    // enable gps velocity based centrefugal force compensation
    ahrs.set_correct_centrifugal(true);
    hal.util->set_soft_armed(true);

#if SPRAYER == ENABLED
    // turn off sprayer's test if on
    sprayer.test_pump(false);
#endif

    // enable output to motors
    enable_motor_output();

    // finally actually arm the motors
    motors->armed(true);

    // log arming to dataflash
    Log_Write_Event(DATA_ARMED);

    // log flight mode in case it was changed while vehicle was disarmed
    DataFlash.Log_Write_Mode(control_mode, control_mode_reason);

    // reenable failsafe
    failsafe_enable();

    // perf monitor ignores delay due to arming
    perf_ignore_this_loop();

    // flag exiting this function
    in_arm_motors = false;

    // Log time stamp of arming event
    arm_time_ms = millis();

    // Start the arming delay
    ap.in_arming_delay = true;

    // return success
    return true;
}

// init_disarm_motors - disarm motors
void Copter::init_disarm_motors()
{
    // return immediately if we are already disarmed
    if (!motors->armed()) {
        return;
    }

#if HIL_MODE != HIL_MODE_DISABLED || CONFIG_HAL_BOARD == HAL_BOARD_SITL
    gcs_send_text(MAV_SEVERITY_INFO, "Disarming motors");
#endif

    // save compass offsets learned by the EKF if enabled
    if (ahrs.use_compass() && compass.get_learn_type() == Compass::LEARN_EKF) {
        for(uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
            Vector3f magOffsets;
            if (ahrs.getMagOffsets(i, magOffsets)) {
                compass.set_and_save_offsets(i, magOffsets);
            }
        }
    }

#if AUTOTUNE_ENABLED == ENABLED
    // save auto tuned parameters
    autotune_save_tuning_gains();
#endif

    // we are not in the air
    set_land_complete(true);
    set_land_complete_maybe(true);

    // log disarm to the dataflash
    Log_Write_Event(DATA_DISARMED);

    // send disarm command to motors
    motors->armed(false);

    // reset the mission
    mission.reset();

    DataFlash_Class::instance()->set_vehicle_armed(false);

    // disable gps velocity based centrefugal force compensation
    ahrs.set_correct_centrifugal(false);
    hal.util->set_soft_armed(false);

    ap.in_arming_delay = false;
}

// motors_output - send output to motors library which will adjust and send to ESCs and servos
void Copter::motors_output()
{
#if ADVANCED_FAILSAFE == ENABLED
    // this is to allow the failsafe module to deliberately crash 
    // the vehicle. Only used in extreme circumstances to meet the
    // OBC rules
    // 这是为了让故障安全模块故意崩溃车辆。
	// 只有在极端情况下才会遇到
	// OBC规则
    if (g2.afs.should_crash_vehicle()) {
        g2.afs.terminate_vehicle();
        return;
    }
#endif

    // Update arming delay state
    // 更新解锁延时状态
    if (ap.in_arming_delay && (!motors->armed() || millis()-arm_time_ms > ARMING_DELAY_SEC*1.0e3f || control_mode == THROW)) {
        ap.in_arming_delay = false;
    }

    // output any servo channels
    // 输出到任何伺服通道
    SRV_Channels::calc_pwm();

    // cork now, so that all channel outputs happen at once
    //软木塞，所有通道输出同时发生
    hal.rcout->cork();
    
    // update output on any aux channels, for manual passthru
    //更新任何一个频道的输出，以供手动通过。
    SRV_Channels::output_ch_all();
    
    // check if we are performing the motor test
    // 检查是否执行电机测试
    if (ap.motor_test) {
        motor_test_output();
    } else {
        bool interlock = motors->armed() && !ap.in_arming_delay && (!ap.using_interlock || ap.motor_interlock_switch) && !ap.motor_emergency_stop;
        if (!motors->get_interlock() && interlock) {
            motors->set_interlock(true);
            Log_Write_Event(DATA_MOTORS_INTERLOCK_ENABLED);
        } else if (motors->get_interlock() && !interlock) {
            motors->set_interlock(false);
            Log_Write_Event(DATA_MOTORS_INTERLOCK_DISABLED);
        }

        // send output signals to motors
        // 发送输出信号到电机
        motors->output();
    }

    // push all channels
    // 推送所有通道
    hal.rcout->push();
}

// check for pilot stick input to trigger lost vehicle alarm
void Copter::lost_vehicle_check()
{
    static uint8_t soundalarm_counter;

    // disable if aux switch is setup to vehicle alarm as the two could interfere
    if (check_if_auxsw_mode_used(AUXSW_LOST_COPTER_SOUND)) {
        return;
    }

    // ensure throttle is down, motors not armed, pitch and roll rc at max. Note: rc1=roll rc2=pitch
    if (ap.throttle_zero && !motors->armed() && (channel_roll->get_control_in() > 4000) && (channel_pitch->get_control_in() > 4000)) {
        if (soundalarm_counter >= LOST_VEHICLE_DELAY) {
            if (AP_Notify::flags.vehicle_lost == false) {
                AP_Notify::flags.vehicle_lost = true;
                gcs_send_text(MAV_SEVERITY_NOTICE,"Locate Copter alarm");
            }
        } else {
            soundalarm_counter++;
        }
    } else {
        soundalarm_counter = 0;
        if (AP_Notify::flags.vehicle_lost == true) {
            AP_Notify::flags.vehicle_lost = false;
        }
    }
}
