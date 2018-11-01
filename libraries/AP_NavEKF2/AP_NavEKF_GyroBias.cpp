#include <AP_HAL/AP_HAL.h>

#if HAL_CPU_CLASS >= HAL_CPU_CLASS_150

#include "AP_NavEKF2.h"
#include "AP_NavEKF2_core.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Vehicle/AP_Vehicle.h>

#include <stdio.h>

extern const AP_HAL::HAL& hal;

// reset the body axis gyro bias states to zero and re-initialise the corresponding covariances
// Assume that the calibration is performed to an accuracy of 0.5 deg/sec which will require averaging under static conditions
// WARNING - a non-blocking calibration method must be used
//重置身体轴陀螺偏差状态为零，并重新初始化相应的协方差
//假设校准的精度为0.5/秒，这将要求在静态条件下进行平均。
//警告-必须使用非阻塞校准方法
void NavEKF2_core::resetGyroBias(void)
{
    stateStruct.gyro_bias.zero();
    zeroRows(P,9,11);
    zeroCols(P,9,11);

    P[9][9] = sq(radians(0.5f * dtIMUavg));
    P[10][10] = P[9][9];
    P[11][11] = P[9][9];
}

/*
   vehicle specific initial gyro bias uncertainty in deg/sec
   特定于车辆的初始陀螺偏差不确定度
 */
float NavEKF2_core::InitialGyroBiasUncertainty(void) const
{
    return 2.5f;
}


#endif // HAL_CPU_CLASS
