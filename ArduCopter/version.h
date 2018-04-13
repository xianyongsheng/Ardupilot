#pragma once

#include "ap_version.h"

#define THISFIRMWARE "#Copter V3.5.5-io"
#define FIRMWARE_VERSION 3,5,5,FIRMWARE_VERSION_TYPE_OFFICIAL

#ifndef GIT_VERSION
#define FIRMWARE_STRING THISFIRMWARE
#else
#define FIRMWARE_STRING THISFIRMWARE " (" GIT_VERSION ")"
#endif



/*
 *-2018.4.13
 * modify board_config.h
 * modify px4fmu_spi/init.c
 * modify SPIDevice.cpp ms5611
 * modify fmu.cpp adc.cpp
 *
 *
 *-2018.04.10
 * modify AC_ATTITUDE_ACCEL_Y_CONTROLLER_MAX_RADSS 120 to 1200
 * modify POSCONTROL_LEASH_LENGTH_MIN 100 to 1000
 *
 */


