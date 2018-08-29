#include "Copter.h"

// read_inertia - read inertia in from accelerometers
void Copter::read_inertia()
{
    // inertial altitude estimates
    // 惯性高度估计
    inertial_nav.update(G_Dt);

    // pull position from interial nav library
    // 拉取位置
    current_loc.lng = inertial_nav.get_longitude();
    current_loc.lat = inertial_nav.get_latitude();

    // exit immediately if we do not have an altitude estimate
    // 立即退出，如果我们没有高度估计
    if (!inertial_nav.get_filter_status().flags.vert_pos) {
        return;
    }

    // without home return alt above the EKF origin
    //没有home返回的alt水平高于EKF来源
    if (ap.home_state == HOME_UNSET) {
        // with inertial nav we can update the altitude and climb rate at 50hz
        //惯性导航我们可以在50赫兹上更新高度和爬升率
        current_loc.alt = inertial_nav.get_altitude();
    } else {
        // with inertial nav we can update the altitude and climb rate at 50hz
        current_loc.alt = pv_alt_above_home(inertial_nav.get_altitude());
    }

    // set flags and get velocity
    // 设置标志和获取速度
    current_loc.flags.relative_alt = true;
    climb_rate = inertial_nav.get_velocity_z();
}
