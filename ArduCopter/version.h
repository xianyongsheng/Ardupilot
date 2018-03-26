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
 * 2.18.3.16
 * add rtk gps to NMEA
 * modify drift_correction_yaw 
 *
 *
 */
