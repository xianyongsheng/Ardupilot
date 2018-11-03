#pragma once

#ifndef FORCE_VERSION_H_INCLUDE
#error version.h should never be included directly. You probably want to include AP_Common/AP_FWVersion.h
#endif

#include "ap_version.h"

#define THISFIRMWARE "ArduCopter V3.6.0"

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 3,6,0,FIRMWARE_VERSION_TYPE_OFFICIAL

#define FW_MAJOR 3
#define FW_MINOR 6
#define FW_PATCH 0
#define FW_TYPE FIRMWARE_VERSION_TYPE_OFFICIAL


/*
 *
 *
 *
 * 2018.11.3
 *  modify
 *      waf scripting for copy bin file
 *      hwdef for first board change
 *      baro init detect
 *      mpu init detect
 *      HAL/board/Chibios.h
 *      motores_class.h
 *      GCS_commin.cpp for send_servo_output_raw
 */
