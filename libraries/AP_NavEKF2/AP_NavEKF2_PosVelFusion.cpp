#include <AP_HAL/AP_HAL.h>

#if HAL_CPU_CLASS >= HAL_CPU_CLASS_150

#include "AP_NavEKF2.h"
#include "AP_NavEKF2_core.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Vehicle/AP_Vehicle.h>

#include <stdio.h>

extern const AP_HAL::HAL& hal;

/********************************************************
*                   RESET FUNCTIONS                     *
********************************************************/

// Reset velocity states to last GPS measurement if available or to zero if in constant position mode or if PV aiding is not absolute
// Do not reset vertical velocity using GPS as there is baro alt available to constrain drift
void NavEKF2_core::ResetVelocity(void)
{
    // Store the position before the reset so that we can record the reset delta
    velResetNE.x = stateStruct.velocity.x;
    velResetNE.y = stateStruct.velocity.y;

    // reset the corresponding covariances
    zeroRows(P,3,4);
    zeroCols(P,3,4);

    if (PV_AidingMode != AID_ABSOLUTE) {
        stateStruct.velocity.zero();
        // set the variances using the measurement noise parameter
        P[4][4] = P[3][3] = sq(frontend->_gpsHorizVelNoise);
    } else {
        // reset horizontal velocity states to the GPS velocity if available
        if (imuSampleTime_ms - lastTimeGpsReceived_ms < 250) {
            stateStruct.velocity.x  = gpsDataNew.vel.x;
            stateStruct.velocity.y  = gpsDataNew.vel.y;
            // set the variances using the reported GPS speed accuracy
            P[4][4] = P[3][3] = sq(MAX(frontend->_gpsHorizVelNoise,gpsSpdAccuracy));
            // clear the timeout flags and counters
            velTimeout = false;
            lastVelPassTime_ms = imuSampleTime_ms;
        } else {
            stateStruct.velocity.x  = 0.0f;
            stateStruct.velocity.y  = 0.0f;
            // set the variances using the likely speed range
            P[4][4] = P[3][3] = sq(25.0f);
            // clear the timeout flags and counters
            velTimeout = false;
            lastVelPassTime_ms = imuSampleTime_ms;
        }
    }
    for (uint8_t i=0; i<imu_buffer_length; i++) {
        storedOutput[i].velocity.x = stateStruct.velocity.x;
        storedOutput[i].velocity.y = stateStruct.velocity.y;
    }
    outputDataNew.velocity.x = stateStruct.velocity.x;
    outputDataNew.velocity.y = stateStruct.velocity.y;
    outputDataDelayed.velocity.x = stateStruct.velocity.x;
    outputDataDelayed.velocity.y = stateStruct.velocity.y;

    // Calculate the position jump due to the reset
    velResetNE.x = stateStruct.velocity.x - velResetNE.x;
    velResetNE.y = stateStruct.velocity.y - velResetNE.y;

    // store the time of the reset
    lastVelReset_ms = imuSampleTime_ms;


}

// resets position states to last GPS measurement or to zero if in constant position mode
void NavEKF2_core::ResetPosition(void)
{
    // Store the position before the reset so that we can record the reset delta
    posResetNE.x = stateStruct.position.x;
    posResetNE.y = stateStruct.position.y;

    // reset the corresponding covariances
    zeroRows(P,6,7);
    zeroCols(P,6,7);

    if (PV_AidingMode != AID_ABSOLUTE) {
        // reset all position state history to the last known position
        stateStruct.position.x = lastKnownPositionNE.x;
        stateStruct.position.y = lastKnownPositionNE.y;
        // set the variances using the position measurement noise parameter
        P[6][6] = P[7][7] = sq(frontend->_gpsHorizPosNoise);
    } else  {
        // Use GPS data as first preference if fresh data is available
        if (imuSampleTime_ms - lastTimeGpsReceived_ms < 250) {
            // record the ID of the GPS for the data we are using for the reset
            last_gps_idx = gpsDataNew.sensor_idx;
            // write to state vector and compensate for offset  between last GPS measurement and the EKF time horizon
            stateStruct.position.x = gpsDataNew.pos.x  + 0.001f*gpsDataNew.vel.x*(float(imuDataDelayed.time_ms) - float(gpsDataNew.time_ms));
            stateStruct.position.y = gpsDataNew.pos.y  + 0.001f*gpsDataNew.vel.y*(float(imuDataDelayed.time_ms) - float(gpsDataNew.time_ms));
            // set the variances using the position measurement noise parameter
            P[6][6] = P[7][7] = sq(MAX(gpsPosAccuracy,frontend->_gpsHorizPosNoise));
            // clear the timeout flags and counters
            posTimeout = false;
            lastPosPassTime_ms = imuSampleTime_ms;
        } else if (imuSampleTime_ms - rngBcnLast3DmeasTime_ms < 250) {
            // use the range beacon data as a second preference
            stateStruct.position.x = receiverPos.x;
            stateStruct.position.y = receiverPos.y;
            // set the variances from the beacon alignment filter
            P[6][6] = receiverPosCov[0][0];
            P[7][7] = receiverPosCov[1][1];
            // clear the timeout flags and counters
            rngBcnTimeout = false;
            lastRngBcnPassTime_ms = imuSampleTime_ms;
        }
    }
    for (uint8_t i=0; i<imu_buffer_length; i++) {
        storedOutput[i].position.x = stateStruct.position.x;
        storedOutput[i].position.y = stateStruct.position.y;
    }
    outputDataNew.position.x = stateStruct.position.x;
    outputDataNew.position.y = stateStruct.position.y;
    outputDataDelayed.position.x = stateStruct.position.x;
    outputDataDelayed.position.y = stateStruct.position.y;

    // Calculate the position jump due to the reset
    posResetNE.x = stateStruct.position.x - posResetNE.x;
    posResetNE.y = stateStruct.position.y - posResetNE.y;

    // store the time of the reset
    lastPosReset_ms = imuSampleTime_ms;

}

// reset the vertical position state using the last height measurement
void NavEKF2_core::ResetHeight(void)
{
    // Store the position before the reset so that we can record the reset delta
    posResetD = stateStruct.position.z;

    // write to the state vector
    stateStruct.position.z = -hgtMea;
    outputDataNew.position.z = stateStruct.position.z;
    outputDataDelayed.position.z = stateStruct.position.z;

    // reset the terrain state height
    if (onGround) {
        // assume vehicle is sitting on the ground
        terrainState = stateStruct.position.z + rngOnGnd;
    } else {
        // can make no assumption other than vehicle is not below ground level
        terrainState = MAX(stateStruct.position.z + rngOnGnd , terrainState);
    }
    for (uint8_t i=0; i<imu_buffer_length; i++) {
        storedOutput[i].position.z = stateStruct.position.z;
    }

    // Calculate the position jump due to the reset
    posResetD = stateStruct.position.z - posResetD;

    // store the time of the reset
    lastPosResetD_ms = imuSampleTime_ms;

    // clear the timeout flags and counters
    hgtTimeout = false;
    lastHgtPassTime_ms = imuSampleTime_ms;

    // reset the corresponding covariances
    zeroRows(P,8,8);
    zeroCols(P,8,8);

    // set the variances to the measurement variance
    P[8][8] = posDownObsNoise;

    // Reset the vertical velocity state using GPS vertical velocity if we are airborne
    // Check that GPS vertical velocity data is available and can be used
    if (inFlight && !gpsNotAvailable && frontend->_fusionModeGPS == 0) {
        stateStruct.velocity.z =  gpsDataNew.vel.z;
    } else if (onGround) {
        stateStruct.velocity.z = 0.0f;
    }
    for (uint8_t i=0; i<imu_buffer_length; i++) {
        storedOutput[i].velocity.z = stateStruct.velocity.z;
    }
    outputDataNew.velocity.z = stateStruct.velocity.z;
    outputDataDelayed.velocity.z = stateStruct.velocity.z;

    // reset the corresponding covariances
    zeroRows(P,5,5);
    zeroCols(P,5,5);

    // set the variances to the measurement variance
    P[5][5] = sq(frontend->_gpsVertVelNoise);

}

// Zero the EKF height datum
// Return true if the height datum reset has been performed
bool NavEKF2_core::resetHeightDatum(void)
{
    if (activeHgtSource == HGT_SOURCE_RNG) {
        // by definition the height datum is at ground level so cannot perform the reset
        return false;
    }
    // record the old height estimate
    float oldHgt = -stateStruct.position.z;
    // reset the barometer so that it reads zero at the current height
    frontend->_baro.update_calibration();
    // reset the height state
    stateStruct.position.z = 0.0f;
    // adjust the height of the EKF origin so that the origin plus baro height before and after the reset is the same
    if (validOrigin) {
        EKF_origin.alt += oldHgt*100;
    }
    // adjust the terrain state
    terrainState += oldHgt;
    return true;
}

/********************************************************
*                   FUSE MEASURED_DATA                  *
********************************************************/
// select fusion of velocity, position and height measurements
//选择融合速度、位置、高度测量
void NavEKF2_core::SelectVelPosFusion()
{
    // Check if the magnetometer has been fused on that time step and the filter is running at faster than 200 Hz
    // If so, don't fuse measurements on this time step to reduce frame over-runs
    // Only allow one time slip to prevent high rate magnetometer data preventing fusion of other measurements
    if (magFusePerformed && dtIMUavg < 0.005f && !posVelFusionDelayed) {
        posVelFusionDelayed = true;
        return;
    } else {
        posVelFusionDelayed = false;
    }

    // read GPS data from the sensor and check for new data in the buffer
    readGpsData();
    gpsDataToFuse = storedGPS.recall(gpsDataDelayed,imuDataDelayed.time_ms);
    // Determine if we need to fuse position and velocity data on this time step
    if (gpsDataToFuse && PV_AidingMode == AID_ABSOLUTE) {
        // correct GPS data for position offset of antenna phase centre relative to the IMU
        Vector3f posOffsetBody = _ahrs->get_gps().get_antenna_offset(gpsDataDelayed.sensor_idx) - accelPosOffset;
        if (!posOffsetBody.is_zero()) {
            if (fuseVelData) {
                // TODO use a filtered angular rate with a group delay that matches the GPS delay
                Vector3f angRate = imuDataDelayed.delAng * (1.0f/imuDataDelayed.delAngDT);
                Vector3f velOffsetBody = angRate % posOffsetBody;
                Vector3f velOffsetEarth = prevTnb.mul_transpose(velOffsetBody);
                gpsDataDelayed.vel -= velOffsetEarth;
            }
            Vector3f posOffsetEarth = prevTnb.mul_transpose(posOffsetBody);
            gpsDataDelayed.pos.x -= posOffsetEarth.x;
            gpsDataDelayed.pos.y -= posOffsetEarth.y;
            gpsDataDelayed.hgt += posOffsetEarth.z;
        }


        // Don't fuse velocity data if GPS doesn't support it
        if (frontend->_fusionModeGPS <= 1) {
            fuseVelData = true;
        } else {
            fuseVelData = false;
        }
        fusePosData = true;

    } else {
        fuseVelData = false;
        fusePosData = false;
    }

    // we have GPS data to fuse and a request to align the yaw using the GPS course
    if (gpsYawResetRequest) {
        realignYawGPS();
    }

    // Select height data to be fused from the available baro, range finder and GPS sources
    //从有效的气压计、rng、GPS 选择高度数据作为融合
    selectHeightForFusion();

    // if we are using GPS, check for a change in receiver and reset position and height
    //如果我们使用GPS，检查接收器的变化和重置位置和高度
    if (gpsDataToFuse && PV_AidingMode == AID_ABSOLUTE && gpsDataDelayed.sensor_idx != last_gps_idx) {
        // record the ID of the GPS that we are using for the reset
        last_gps_idx = gpsDataDelayed.sensor_idx;

        // Store the position before the reset so that we can record the reset delta
        posResetNE.x = stateStruct.position.x;
        posResetNE.y = stateStruct.position.y;

        // Set the position states to the position from the new GPS
        stateStruct.position.x = gpsDataNew.pos.x;
        stateStruct.position.y = gpsDataNew.pos.y;

        // Calculate the position offset due to the reset
        posResetNE.x = stateStruct.position.x - posResetNE.x;
        posResetNE.y = stateStruct.position.y - posResetNE.y;

        // Add the offset to the output observer states
        for (uint8_t i=0; i<imu_buffer_length; i++) {
            storedOutput[i].position.x += posResetNE.x;
            storedOutput[i].position.y += posResetNE.y;
        }
        outputDataNew.position.x += posResetNE.x;
        outputDataNew.position.y += posResetNE.y;
        outputDataDelayed.position.x += posResetNE.x;
        outputDataDelayed.position.y += posResetNE.y;

        // store the time of the reset
        lastPosReset_ms = imuSampleTime_ms;

        // If we are alseo using GPS as the height reference, reset the height
        if (activeHgtSource == HGT_SOURCE_GPS) {
            // Store the position before the reset so that we can record the reset delta
            posResetD = stateStruct.position.z;

            // write to the state vector
            stateStruct.position.z = -hgtMea;

            // Calculate the position jump due to the reset
            posResetD = stateStruct.position.z - posResetD;

            // Add the offset to the output observer states
            outputDataNew.position.z += posResetD;
            outputDataDelayed.position.z += posResetD;
            for (uint8_t i=0; i<imu_buffer_length; i++) {
                storedOutput[i].position.z += posResetD;
            }

            // store the time of the reset
            lastPosResetD_ms = imuSampleTime_ms;
        }
    }

    // If we are operating without any aiding, fuse in the last known position
    // to constrain tilt drift. This assumes a non-manoeuvring vehicle
    // Do this to coincide with the height fusion
    if (fuseHgtData && PV_AidingMode == AID_NONE) {
        gpsDataDelayed.vel.zero();
        gpsDataDelayed.pos.x = lastKnownPositionNE.x;
        gpsDataDelayed.pos.y = lastKnownPositionNE.y;

        fusePosData = true;
        fuseVelData = false;
    }

    // perform fusion
    //执行融合！！速度位置高度
    if (fuseVelData || fusePosData || fuseHgtData) {
        FuseVelPosNED();
        // clear the flags to prevent repeated fusion of the same data
        fuseVelData = false;
        fuseHgtData = false;
        fusePosData = false;
    }
}

// fuse selected position, velocity and height measurements
// 融合位置、速度和高度
void NavEKF2_core::FuseVelPosNED()
{
    // start performance timer
    hal.util->perf_begin(_perf_FuseVelPosNED);

    // health is set bad until test passed
    // 健康值设置坏得直到测试通过
    velHealth = false;
    posHealth = false;
    hgtHealth = false;

    // declare variables used to check measurement errors
    //声明变量用作检查测量误差
    Vector3f velInnov;

    // declare variables used to control access to arrays
    //声明变量用作访问控制数组
    bool fuseData[6] = {false,false,false,false,false,false};
    uint8_t stateIndex;
    uint8_t obsIndex;

    // declare variables used by state and covariance update calculations
    // 声明状态和协方差更新计算所使用的变量
    Vector6 R_OBS; // Measurement variances used for fusion
    			   // 测量方差用作融合
    Vector6 R_OBS_DATA_CHECKS; // Measurement variances used for data checks only
    						   // 测量方差仅仅用作数据检查
    Vector6 observation;
    float SK;

    // perform sequential fusion of GPS measurements. This assumes that the
    // errors in the different velocity and position components are
    // uncorrelated which is not true, however in the absence of covariance
    // data from the GPS receiver it is the only assumption we can make
    // so we might as well take advantage of the computational efficiencies
    // associated with sequential fusion
    //执行GPS测量的顺序融合。
	//这个假设不同速度和位置分量的错误不相关的，这不是真的
	//但是在没有协方差的情况下
	//来自GPS接收器的数据这是我们唯一能做的假设
	//所以我们不妨利用计算效率
	//与顺序融合有关
    if (fuseVelData || fusePosData || fuseHgtData) {
        // form the observation vector
        // 来自观测矢量
        observation[0] = gpsDataDelayed.vel.x;
        observation[1] = gpsDataDelayed.vel.y;
        observation[2] = gpsDataDelayed.vel.z;
        observation[3] = gpsDataDelayed.pos.x;
        observation[4] = gpsDataDelayed.pos.y;
        observation[5] = -hgtMea;

        // calculate additional error in GPS position caused by manoeuvring
        //计算由操纵引起的GPS位置的额外误差
        float posErr = frontend->gpsPosVarAccScale * accNavMag;

        // estimate the GPS Velocity, GPS horiz position and height measurement variances.
        // Use different errors if operating without external aiding using an assumed position or velocity of zero
        //估计GPS速度、GPS定位位置和高度测量差异。
		//使用不同的错误，如果不使用外部辅助操作，使用假定的位置或速度为0
        if (PV_AidingMode == AID_NONE) {
            if (tiltAlignComplete && motorsArmed) {
            // This is a compromise between corrections for gyro errors and reducing effect of manoeuvre accelerations on tilt estimate
            // 这是对陀螺错误校正和减少操纵加速度对倾斜估计的影响之间的折衷。
                R_OBS[0] = sq(constrain_float(frontend->_noaidHorizNoise, 0.5f, 50.0f));
            } else {
                // Use a smaller value to give faster initial alignment
                //使用更小的值来提供更快的初始对齐
                R_OBS[0] = sq(0.5f);
            }
            R_OBS[1] = R_OBS[0];
            R_OBS[2] = R_OBS[0];
            R_OBS[3] = R_OBS[0];
            R_OBS[4] = R_OBS[0];
            for (uint8_t i=0; i<=2; i++) R_OBS_DATA_CHECKS[i] = R_OBS[i];
        } else {
            if (gpsSpdAccuracy > 0.0f) {
                // use GPS receivers reported speed accuracy if available and floor at value set by GPS velocity noise parameter
                //使用GPS接收器报告的速度精度，如果可用，则使用GPS速度噪声参数设定的下限
                R_OBS[0] = sq(constrain_float(gpsSpdAccuracy, frontend->_gpsHorizVelNoise, 50.0f));
                R_OBS[2] = sq(constrain_float(gpsSpdAccuracy, frontend->_gpsVertVelNoise, 50.0f));
            } else {
                // calculate additional error in GPS velocity caused by manoeuvring
                //计算由操纵引起的GPS速度的额外误差
                R_OBS[0] = sq(constrain_float(frontend->_gpsHorizVelNoise, 0.05f, 5.0f)) + sq(frontend->gpsNEVelVarAccScale * accNavMag);
                R_OBS[2] = sq(constrain_float(frontend->_gpsVertVelNoise,  0.05f, 5.0f)) + sq(frontend->gpsDVelVarAccScale  * accNavMag);
            }
            R_OBS[1] = R_OBS[0];
            // Use GPS reported position accuracy if available and floor at value set by GPS position noise parameter
            //使用GPS报告的位置精度，如果可用，则使用GPS定位噪声参数设定的值
            if (gpsPosAccuracy > 0.0f) {
                R_OBS[3] = sq(constrain_float(gpsPosAccuracy, frontend->_gpsHorizPosNoise, 100.0f));
            } else {
                R_OBS[3] = sq(constrain_float(frontend->_gpsHorizPosNoise, 0.1f, 10.0f)) + sq(posErr);
            }
            R_OBS[4] = R_OBS[3];
            // For data integrity checks we use the same measurement variances as used to calculate the Kalman gains for all measurements except GPS horizontal velocity
            // For horizontal GPs velocity we don't want the acceptance radius to increase with reported GPS accuracy so we use a value based on best GPs perfomrance
            // plus a margin for manoeuvres. It is better to reject GPS horizontal velocity errors early
            //对于数据完整性检查，我们使用与计算卡尔曼增益的相同的测量方差，除了GPS水平速度之外的所有测量值
			//对于水平的GPS速度我们不希望接收半径随报告的GPs精度而增加所以我们使用基于最佳GPs性能的值
			//加上机动的余地。最好是尽早拒绝GPS水平速度错误
            for (uint8_t i=0; i<=2; i++) R_OBS_DATA_CHECKS[i] = sq(constrain_float(frontend->_gpsHorizVelNoise, 0.05f, 5.0f)) + sq(frontend->gpsNEVelVarAccScale * accNavMag);
        }
        R_OBS[5] = posDownObsNoise;
        for (uint8_t i=3; i<=5; i++) R_OBS_DATA_CHECKS[i] = R_OBS[i];

        // if vertical GPS velocity data and an independent height source is being used, check to see if the GPS vertical velocity and altimeter
        // innovations have the same sign and are outside limits. If so, then it is likely aliasing is affecting
        // the accelerometers and we should disable the GPS and barometer innovation consistency checks.
        //如果使用垂直的GPS速度数据和独立的高度源，请检查GPS垂直速度和高度计
		//创新有相同的标志，并且是外部限制。
		//如果是这样，那么混叠就会影响到加速计，我们应该禁用GPS和气压计的创新一致性检查。
        if (useGpsVertVel && fuseVelData && (frontend->_altSource != 2)) {
            // calculate innovations for height and vertical GPS vel measurements
            //计算高度和垂直GPS测量值的创新
            float hgtErr  = stateStruct.position.z - observation[5];
            float velDErr = stateStruct.velocity.z - observation[2];
            // check if they are the same sign and both more than 3-sigma out of bounds
            //检查它们是否是相同的符号，并且超过3-sigma的界限
            if ((hgtErr*velDErr > 0.0f) && (sq(hgtErr) > 9.0f * (P[8][8] + R_OBS_DATA_CHECKS[5])) && (sq(velDErr) > 9.0f * (P[5][5] + R_OBS_DATA_CHECKS[2]))) {
                badIMUdata = true;
            } else {
                badIMUdata = false;
            }
        }

        // calculate innovations and check GPS data validity using an innovation consistency check
        // test position measurements
        if (fusePosData) {
            // test horizontal position measurements
            innovVelPos[3] = stateStruct.position.x - observation[3];
            innovVelPos[4] = stateStruct.position.y - observation[4];
            varInnovVelPos[3] = P[6][6] + R_OBS_DATA_CHECKS[3];
            varInnovVelPos[4] = P[7][7] + R_OBS_DATA_CHECKS[4];
            // apply an innovation consistency threshold test, but don't fail if bad IMU data
            float maxPosInnov2 = sq(MAX(0.01f * (float)frontend->_gpsPosInnovGate, 1.0f))*(varInnovVelPos[3] + varInnovVelPos[4]);
            posTestRatio = (sq(innovVelPos[3]) + sq(innovVelPos[4])) / maxPosInnov2;
            posHealth = ((posTestRatio < 1.0f) || badIMUdata);
            // use position data if healthy or timed out
            if (PV_AidingMode == AID_NONE) {
                posHealth = true;
                lastPosPassTime_ms = imuSampleTime_ms;
            } else if (posHealth || posTimeout) {
                posHealth = true;
                lastPosPassTime_ms = imuSampleTime_ms;
                // if timed out or outside the specified uncertainty radius, reset to the GPS
                if (posTimeout || ((P[6][6] + P[7][7]) > sq(float(frontend->_gpsGlitchRadiusMax)))) {
                    // reset the position to the current GPS position
                    ResetPosition();
                    // reset the velocity to the GPS velocity
                    ResetVelocity();
                    // don't fuse GPS data on this time step
                    fusePosData = false;
                    fuseVelData = false;
                    // Reset the position variances and corresponding covariances to a value that will pass the checks
                    //将位置差异和相应的协方差重置为通过检查的值
                    zeroRows(P,6,7);
                    zeroCols(P,6,7);
                    P[6][6] = sq(float(0.5f*frontend->_gpsGlitchRadiusMax));
                    P[7][7] = P[6][6];
                    // Reset the normalised innovation to avoid failing the bad fusion tests
                    posTestRatio = 0.0f;
                    velTestRatio = 0.0f;
                }
            } else {
                posHealth = false;
            }
        }

        // test velocity measurements
        if (fuseVelData) {
            // test velocity measurements
            uint8_t imax = 2;
            // Don't fuse vertical velocity observations if inhibited by the user or if we are using synthetic data
            if (frontend->_fusionModeGPS >= 1 || PV_AidingMode != AID_ABSOLUTE) {
                imax = 1;
            }
            float innovVelSumSq = 0; // sum of squares of velocity innovations
            float varVelSum = 0; // sum of velocity innovation variances
            for (uint8_t i = 0; i<=imax; i++) {
                // velocity states start at index 3
                stateIndex   = i + 3;
                // calculate innovations using blended and single IMU predicted states
                velInnov[i]  = stateStruct.velocity[i] - observation[i]; // blended
                // calculate innovation variance
                varInnovVelPos[i] = P[stateIndex][stateIndex] + R_OBS_DATA_CHECKS[i];
                // sum the innovation and innovation variances
                innovVelSumSq += sq(velInnov[i]);
                varVelSum += varInnovVelPos[i];
            }
            // apply an innovation consistency threshold test, but don't fail if bad IMU data
            // calculate the test ratio
            velTestRatio = innovVelSumSq / (varVelSum * sq(MAX(0.01f * (float)frontend->_gpsVelInnovGate, 1.0f)));
            // fail if the ratio is greater than 1
            velHealth = ((velTestRatio < 1.0f)  || badIMUdata);
            // use velocity data if healthy, timed out, or in constant position mode
            if (velHealth || velTimeout) {
                velHealth = true;
                // restart the timeout count
                lastVelPassTime_ms = imuSampleTime_ms;
                // If we are doing full aiding and velocity fusion times out, reset to the GPS velocity
                if (PV_AidingMode == AID_ABSOLUTE && velTimeout) {
                    // reset the velocity to the GPS velocity
                    ResetVelocity();
                    // don't fuse GPS velocity data on this time step
                    fuseVelData = false;
                    // Reset the normalised innovation to avoid failing the bad fusion tests
                    velTestRatio = 0.0f;
                }
            } else {
                velHealth = false;
            }
        }

        // test height measurements
        // 测试高度测量
        if (fuseHgtData) {
            // calculate height innovations
            // 计算高度创新
            innovVelPos[5] = stateStruct.position.z - observation[5];
            varInnovVelPos[5] = P[8][8] + R_OBS_DATA_CHECKS[5];
            // calculate the innovation consistency test ratio
            //计算创新一致性测试比率
            hgtTestRatio = sq(innovVelPos[5]) / (sq(MAX(0.01f * (float)frontend->_hgtInnovGate, 1.0f)) * varInnovVelPos[5]);
            // fail if the ratio is > 1, but don't fail if bad IMU data
            // 如果比率大于1 并且不是坏的IMU数据，则失败
            hgtHealth = ((hgtTestRatio < 1.0f) || badIMUdata);
            // Fuse height data if healthy or timed out or in constant position mode
            //融合高度数据如果健康或超时或处于固定位置模式
            if (hgtHealth || hgtTimeout || (PV_AidingMode == AID_NONE && onGround)) {
                // Calculate a filtered value to be used by pre-flight health checks
                // We need to filter because wind gusts can generate significant baro noise and we want to be able to detect bias errors in the inertial solution
				//计算飞行前健康检查所使用的过滤值
				//我们需要过滤，因为风的阵风会产生显著的baro噪音我们希望能够在惯性溶液中发现偏差误差
				if (onGround) {
                    float dtBaro = (imuSampleTime_ms - lastHgtPassTime_ms)*1.0e-3f;
                    const float hgtInnovFiltTC = 2.0f;
                    float alpha = constrain_float(dtBaro/(dtBaro+hgtInnovFiltTC),0.0f,1.0f);
                    hgtInnovFiltState += (innovVelPos[5]-hgtInnovFiltState)*alpha;
                } else {
                    hgtInnovFiltState = 0.0f;
                }

                // if timed out, reset the height
                // 超市复位高度
                if (hgtTimeout) {
                    ResetHeight();
                }

                // If we have got this far then declare the height data as healthy and reset the timeout counter
                //如果我们已经到达这里，那么将高度数据声明为健康并重新设置超时计数器
                hgtHealth = true;
                lastHgtPassTime_ms = imuSampleTime_ms;
            }
        }

        // set range for sequential fusion of velocity and position measurements depending on which data is available and its health
		//确定速度和位置测量的顺序融合的范围，这取决于可用的数据和它的健康状况
		if (fuseVelData && velHealth) {
            fuseData[0] = true;
            fuseData[1] = true;
            if (useGpsVertVel) {
                fuseData[2] = true;
            }
            tiltErrVec.zero();
        }
        if (fusePosData && posHealth) {
            fuseData[3] = true;
            fuseData[4] = true;
            tiltErrVec.zero();
        }
        if (fuseHgtData && hgtHealth) {
            fuseData[5] = true;
        }

        // fuse measurements sequentially
        //顺序融合测量
        for (obsIndex=0; obsIndex<=5; obsIndex++) {
            if (fuseData[obsIndex]) {
                stateIndex = 3 + obsIndex;
                // calculate the measurement innovation, using states from a different time coordinate if fusing height data
                // adjust scaling on GPS measurement noise variances if not enough satellites
                //计算测量的创新，使用不同时间坐标的状态，如果融合高度数据
				//调整GPS测量噪音的大小，如果没有足够的卫星
                if (obsIndex <= 2)
                {
                    innovVelPos[obsIndex] = stateStruct.velocity[obsIndex] - observation[obsIndex];
                    R_OBS[obsIndex] *= sq(gpsNoiseScaler);
                }
                else if (obsIndex == 3 || obsIndex == 4) {
                    innovVelPos[obsIndex] = stateStruct.position[obsIndex-3] - observation[obsIndex];
                    R_OBS[obsIndex] *= sq(gpsNoiseScaler);
                } else if (obsIndex == 5) {//高度
                    innovVelPos[obsIndex] = stateStruct.position[obsIndex-3] - observation[obsIndex];
                    const float gndMaxBaroErr = 4.0f;
                    const float gndBaroInnovFloor = -0.5f;

                    if(getTouchdownExpected() && activeHgtSource == HGT_SOURCE_BARO) {
                        // when a touchdown is expected, floor the barometer innovation at gndBaroInnovFloor
                        // constrain the correction between 0 and gndBaroInnovFloor+gndMaxBaroErr
                        // this function looks like this:
                        //当预期触地得分时，在gnd气压创新地板上的气压计创新
						//限制0和gnd气压创新地板+gndmax气压的修正
						//这个函数是这样的：
                        //         |/
                        //---------|---------
                        //    ____/|
                        //   /     |
                        //  /      |
                        innovVelPos[5] += constrain_float(-innovVelPos[5]+gndBaroInnovFloor, 0.0f, gndBaroInnovFloor+gndMaxBaroErr);
                    }
                }

                // calculate the Kalman gain and calculate innovation variances
                //计算卡尔曼增益并计算创新差异
                varInnovVelPos[obsIndex] = P[stateIndex][stateIndex] + R_OBS[obsIndex];
                SK = 1.0f/varInnovVelPos[obsIndex];
                for (uint8_t i= 0; i<=15; i++) {
                    Kfusion[i] = P[i][stateIndex]*SK;
                }

                // inhibit magnetic field state estimation by setting Kalman gains to zero
                //抑制磁场状态估计为零，通过卡尔曼增益
                if (!inhibitMagStates) {
                    for (uint8_t i = 16; i<=21; i++) {
                        Kfusion[i] = P[i][stateIndex]*SK;
                    }
                } else {
                    for (uint8_t i = 16; i<=21; i++) {
                        Kfusion[i] = 0.0f;
                    }
                }

                // inhibit wind state estimation by setting Kalman gains to zero
                // 抑制风状态估计为0，通过卡尔曼增益
                if (!inhibitWindStates) {
                    Kfusion[22] = P[22][stateIndex]*SK;
                    Kfusion[23] = P[23][stateIndex]*SK;
                } else {
                    Kfusion[22] = 0.0f;
                    Kfusion[23] = 0.0f;
                }

                // update the covariance - take advantage of direct observation of a single state at index = stateIndex to reduce computations
                // this is a numerically optimised implementation of standard equation P = (I - K*H)*P;
                //更新协方差——利用在索引=状态索引中的单个状态的直接观察，以减少计算
				//这是标准方程P=（I-K H）P的数字优化实现;
                for (uint8_t i= 0; i<=stateIndexLim; i++) {
                    for (uint8_t j= 0; j<=stateIndexLim; j++)
                    {
                        KHP[i][j] = Kfusion[i] * P[stateIndex][j];
                    }
                }
                // Check that we are not going to drive any variances negative and skip the update if so
                //检查我们不会将任何方差都推到负值，如果有的话，跳过更新
                bool healthyFusion = true;
                for (uint8_t i= 0; i<=stateIndexLim; i++) {
                    if (KHP[i][i] > P[i][i]) {
                        healthyFusion = false;
                    }
                }
                if (healthyFusion) {
                    // update the covariance matrix
                    // 更新协方差矩阵
                    for (uint8_t i= 0; i<=stateIndexLim; i++) {
                        for (uint8_t j= 0; j<=stateIndexLim; j++) {
                            P[i][j] = P[i][j] - KHP[i][j];
                        }
                    }

                    // force the covariance matrix to be symmetrical and limit the variances to prevent ill-condiioning.
                    //强制协方差矩阵是对称的，并限制方差以防止不正当的情况。
                    ForceSymmetry();
                    ConstrainVariances();

                    // update the states
                    // zero the attitude error state - by definition it is assumed to be zero before each observaton fusion
                    //更新状态
					//零态度的高度误差状态——根据定义，在每次观察到的融合之前，它被认为是零。
                    stateStruct.angErr.zero();

                    // calculate state corrections and re-normalise the quaternions for states predicted using the blended IMU data
                    //计算状态修正，并对使用混合IMU数据预测的状态四元数进行重新标准化
                    for (uint8_t i = 0; i<=stateIndexLim; i++) {
                        statesArray[i] = statesArray[i] - Kfusion[i] * innovVelPos[obsIndex];
                    }

                    // the first 3 states represent the angular misalignment vector. This is
                    // is used to correct the estimated quaternion
                    //前3个状态代表了角度误差矢量。这是
					//用于校正估计的四元数
                    stateStruct.quat.rotate(stateStruct.angErr);

                    // sum the attitude error from velocity and position fusion only
                    // used as a metric for convergence monitoring
                    //和速度和位置融合的态度误差之和
					//用作收敛监测的指标
                    if (obsIndex != 5) {
                        tiltErrVec += stateStruct.angErr;
                    }
                    // record good fusion status
                    //记录良好的融合状态
                    if (obsIndex == 0) {
                        faultStatus.bad_nvel = false;
                    } else if (obsIndex == 1) {
                        faultStatus.bad_evel = false;
                    } else if (obsIndex == 2) {
                        faultStatus.bad_dvel = false;
                    } else if (obsIndex == 3) {
                        faultStatus.bad_npos = false;
                    } else if (obsIndex == 4) {
                        faultStatus.bad_epos = false;
                    } else if (obsIndex == 5) {
                        faultStatus.bad_dpos = false;
                    }
                } else {
                    // record bad fusion status
                    // 记录不良的融合状态
                    if (obsIndex == 0) {
                        faultStatus.bad_nvel = true;
                    } else if (obsIndex == 1) {
                        faultStatus.bad_evel = true;
                    } else if (obsIndex == 2) {
                        faultStatus.bad_dvel = true;
                    } else if (obsIndex == 3) {
                        faultStatus.bad_npos = true;
                    } else if (obsIndex == 4) {
                        faultStatus.bad_epos = true;
                    } else if (obsIndex == 5) {
                        faultStatus.bad_dpos = true;
                    }
                }
            }
        }
    }

    // stop performance timer
    hal.util->perf_end(_perf_FuseVelPosNED);
}

/********************************************************
*                   MISC FUNCTIONS                      *
********************************************************/

// select the height measurement to be fused from the available baro, range finder and GPS sources
void NavEKF2_core::selectHeightForFusion()
{
    // Read range finder data and check for new data in the buffer
    // This data is used by both height and optical flow fusion processing
    readRangeFinder();
    rangeDataToFuse = storedRange.recall(rangeDataDelayed,imuDataDelayed.time_ms);

    // correct range data for the body frame position offset relative to the IMU
    // the corrected reading is the reading that would have been taken if the sensor was
    // co-located with the IMU
    if (rangeDataToFuse) {
        Vector3f posOffsetBody = frontend->_rng.get_pos_offset(rangeDataDelayed.sensor_idx) - accelPosOffset;
        if (!posOffsetBody.is_zero()) {
            Vector3f posOffsetEarth = prevTnb.mul_transpose(posOffsetBody);
            rangeDataDelayed.rng += posOffsetEarth.z / prevTnb.c.z;
        }
    }

    // read baro height data from the sensor and check for new data in the buffer
    readBaroData();
    baroDataToFuse = storedBaro.recall(baroDataDelayed, imuDataDelayed.time_ms);

    // select height source
    if (((frontend->_useRngSwHgt > 0) || (frontend->_altSource == 1)) && (imuSampleTime_ms - rngValidMeaTime_ms < 500)) {
        if (frontend->_altSource == 1) {
            // always use range finder
            activeHgtSource = HGT_SOURCE_RNG;
        } else {
            // determine if we are above or below the height switch region
            float rangeMaxUse = 1e-4f * (float)frontend->_rng.max_distance_cm_orient(ROTATION_PITCH_270) * (float)frontend->_useRngSwHgt;
            bool aboveUpperSwHgt = (terrainState - stateStruct.position.z) > rangeMaxUse;
            bool belowLowerSwHgt = (terrainState - stateStruct.position.z) < 0.7f * rangeMaxUse;

            // If the terrain height is consistent and we are moving slowly, then it can be
            // used as a height reference in combination with a range finder
            // apply a hysteresis to the speed check to prevent rapid switching
            float horizSpeed = norm(stateStruct.velocity.x, stateStruct.velocity.y);
            bool dontTrustTerrain = ((horizSpeed > frontend->_useRngSwSpd) && filterStatus.flags.horiz_vel) || !terrainHgtStable;
            float trust_spd_trigger = MAX((frontend->_useRngSwSpd - 1.0f),(frontend->_useRngSwSpd * 0.5f));
            bool trustTerrain = (horizSpeed < trust_spd_trigger) && terrainHgtStable;

            /*
             * Switch between range finder and primary height source using height above ground and speed thresholds with
             * hysteresis to avoid rapid switching. Using range finder for height requires a consistent terrain height
             * which cannot be assumed if the vehicle is moving horizontally.
            */
            if ((aboveUpperSwHgt || dontTrustTerrain) && (activeHgtSource == HGT_SOURCE_RNG)) {
                // cannot trust terrain or range finder so stop using range finder height
                if (frontend->_altSource == 0) {
                    activeHgtSource = HGT_SOURCE_BARO;
                } else if (frontend->_altSource == 2) {
                    activeHgtSource = HGT_SOURCE_GPS;
                }
            } else if (belowLowerSwHgt && trustTerrain && (activeHgtSource != HGT_SOURCE_RNG)) {
                // reliable terrain and range finder so start using range finder height
                activeHgtSource = HGT_SOURCE_RNG;
            }
        }
    } else if ((frontend->_altSource == 2) && ((imuSampleTime_ms - lastTimeGpsReceived_ms) < 500) && validOrigin && gpsAccuracyGood) {
        activeHgtSource = HGT_SOURCE_GPS;
    } else if ((frontend->_altSource == 3) && validOrigin && rngBcnGoodToAlign) {
        activeHgtSource = HGT_SOURCE_BCN;
    } else {
        activeHgtSource = HGT_SOURCE_BARO;
    }

    // Use Baro alt as a fallback if we lose range finder or GPS
    bool lostRngHgt = ((activeHgtSource == HGT_SOURCE_RNG) && ((imuSampleTime_ms - rngValidMeaTime_ms) > 500));
    bool lostGpsHgt = ((activeHgtSource == HGT_SOURCE_GPS) && ((imuSampleTime_ms - lastTimeGpsReceived_ms) > 2000));
    if (lostRngHgt || lostGpsHgt) {
        activeHgtSource = HGT_SOURCE_BARO;
    }

    // if there is new baro data to fuse, calculate filtered baro data required by other processes
    //如果有新的baro数据要融合，计算其他进程所需的过滤后的baro数据
    if (baroDataToFuse) {
        // calculate offset to baro data that enables us to switch to Baro height use during operation
        //计算Baro数据的偏移量，使我们能够在操作过程中切换到Baro高度使用
        if  (activeHgtSource != HGT_SOURCE_BARO) {
            calcFiltBaroOffset();
        }
        // filtered baro data used to provide a reference for takeoff
        // it is is reset to last height measurement on disarming in performArmingChecks()
        //过滤后的baro数据，用于为起飞提供参考。
		//在穿孔检查（）中，它被重置为最后的高度测量（）
        if (!getTakeoffExpected()) {
            const float gndHgtFiltTC = 0.5f;
            const float dtBaro = frontend->hgtAvg_ms*1.0e-3f;
            float alpha = constrain_float(dtBaro / (dtBaro+gndHgtFiltTC),0.0f,1.0f);
            meaHgtAtTakeOff += (baroDataDelayed.hgt-meaHgtAtTakeOff)*alpha;
        }
    }

    // calculate offset to GPS height data that enables us to switch to GPS height during operation
    if (gpsDataToFuse && (activeHgtSource != HGT_SOURCE_GPS)) {
            calcFiltGpsHgtOffset();
    }

    // Select the height measurement source
    if (rangeDataToFuse && (activeHgtSource == HGT_SOURCE_RNG)) {
        // using range finder data
        // correct for tilt using a flat earth model
        if (prevTnb.c.z >= 0.7) {
            // calculate height above ground
            hgtMea  = MAX(rangeDataDelayed.rng * prevTnb.c.z, rngOnGnd);
            // correct for terrain position relative to datum
            hgtMea -= terrainState;
            // enable fusion
            fuseHgtData = true;
            // set the observation noise
            posDownObsNoise = sq(constrain_float(frontend->_rngNoise, 0.1f, 10.0f));
            // add uncertainty created by terrain gradient and vehicle tilt
            posDownObsNoise += sq(rangeDataDelayed.rng * frontend->_terrGradMax) * MAX(0.0f , (1.0f - sq(prevTnb.c.z)));
        } else {
            // disable fusion if tilted too far
            fuseHgtData = false;
        }
    } else if  (gpsDataToFuse && (activeHgtSource == HGT_SOURCE_GPS)) {
        // using GPS data
        hgtMea = gpsDataDelayed.hgt;
        // enable fusion
        fuseHgtData = true;
        // set the observation noise using receiver reported accuracy or the horizontal noise scaled for typical VDOP/HDOP ratio
        if (gpsHgtAccuracy > 0.0f) {
            posDownObsNoise = sq(constrain_float(gpsHgtAccuracy, 1.5f * frontend->_gpsHorizPosNoise, 100.0f));
        } else {
            posDownObsNoise = sq(constrain_float(1.5f * frontend->_gpsHorizPosNoise, 0.1f, 10.0f));
        }
    } else if (baroDataToFuse && (activeHgtSource == HGT_SOURCE_BARO)) {
        // using Baro data
        hgtMea = baroDataDelayed.hgt - baroHgtOffset;
        // enable fusion
        fuseHgtData = true;
        // set the observation noise
        // 设置观测噪声
        posDownObsNoise = sq(constrain_float(frontend->_baroAltNoise, 0.1f, 10.0f));
        // reduce weighting (increase observation noise) on baro if we are likely to be in ground effect
        // 如果有可能在地面效应，则减少气压计的权重
        if (getTakeoffExpected() || getTouchdownExpected()) {
            posDownObsNoise *= frontend->gndEffectBaroScaler;
        }
        // If we are in takeoff mode, the height measurement is limited to be no less than the measurement at start of takeoff
        // This prevents negative baro disturbances due to copter downwash corrupting the EKF altitude during initial ascent
		//如果我们在起飞模式下，高度测量被限制为不小于起飞时的测量值。
		//这可以防止在初始上升过程中由于直升机的下降气流而导致的负气压干扰。
		if (motorsArmed && getTakeoffExpected()) {
            hgtMea = MAX(hgtMea, meaHgtAtTakeOff);
        }
    } else {
        fuseHgtData = false;
    }

    // If we haven't fused height data for a while, then declare the height data as being timed out
    // set timeout period based on whether we have vertical GPS velocity available to constrain drift
    //如果我们没有将高度数据融合一段时间，然后将高度数据声明为超时
	//设定超时时间，这是基于我们是否有垂直的GPS速度来限制漂移
    hgtRetryTime_ms = (useGpsVertVel && !velTimeout) ? frontend->hgtRetryTimeMode0_ms : frontend->hgtRetryTimeMode12_ms;
    if (imuSampleTime_ms - lastHgtPassTime_ms > hgtRetryTime_ms) {
        hgtTimeout = true;
    } else {
        hgtTimeout = false;
    }
}

#endif // HAL_CPU_CLASS
