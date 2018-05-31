#pragma once

#include "ap_version.h"

#define THISFIRMWARE "APM:Copter V3.5.4"
#define FIRMWARE_VERSION 3,5,4,FIRMWARE_VERSION_TYPE_OFFICIAL

#ifndef GIT_VERSION
#define FIRMWARE_STRING THISFIRMWARE
#else
#define FIRMWARE_STRING THISFIRMWARE " (" GIT_VERSION ")"
#endif

/*
 * 2018.5.31
 * add code in AP_NavEKF2_MagFusion.cpp lines: 191
 * add code in AP_NavEKF2_Contorl.cpp lines: 391
 * remove code of AP_AHRS_View
 * remove code of AP_AHRS_NAVEKF EKF2.set_rtk_yaw(_gps.ground_course_valid(),ToRad(_gps.ground_course_cd() * 0.01f));
 *
 * 2018.5.7
 * add yaw parameters
 *  parameters.cpp/h lines: 896/368
 *  config.h lines: 31
 *  gps_nmea
 *
 * 2018.3.16
 * modify AP_GPS_NMEA add rtk gps to NMEA
 * modify AP_AHRS_DCM   drift_correction_yaw
 * modify AP_AHRS_View  if(ahrs._gps.ground_course_valid()){yaw=ToRad(ahrs._gps.ground_course_cd() * 0.01f);}
 * modify AP_AHRS_NAVEKF    EKF2.set_rtk_yaw(_gps.ground_course_valid(),ToRad(_gps.ground_course_cd() * 0.01f));
 * modify AP_NavEKF2.h      void set_rtk_yaw(bool valid, float yaw){rtk_yaw_valid=valid, rtk_yaw=yaw;};
 *
 */
