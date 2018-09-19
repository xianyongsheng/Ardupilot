#pragma once

#include "ap_version.h"

#define THISFIRMWARE "GTen - V3.5.5"
#define FIRMWARE_VERSION 3,5,5,FIRMWARE_VERSION_TYPE_OFFICIAL

#ifndef GIT_VERSION
#define FIRMWARE_STRING THISFIRMWARE
#else
#define FIRMWARE_STRING THISFIRMWARE " (" GIT_VERSION ")"
#endif



/*
 *
 *
 *2018.8.10
 * add mavflow and mavrngfnd
 *
 *2018.8
 * add flowme yaw
 * modify rtk nova
 *
 */
