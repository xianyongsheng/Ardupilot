/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  driver for PX4Flow optical flow sensor
 */

#include <AP_HAL/AP_HAL.h>
#include "AP_OpticalFlow_MAVFlow.h"
#include <AP_Math/edc.h>
#include <AP_AHRS/AP_AHRS.h>
#include <utility>
#include "OpticalFlow.h"
#include <stdio.h>

extern const AP_HAL::HAL& hal;

uint32_t mavrng_last_update_ms = 0;
uint16_t mavrng_dist_cm = 0;

// constructor
AP_OpticalFlow_MAVFlow::AP_OpticalFlow_MAVFlow(OpticalFlow &_frontend) :
    OpticalFlow_backend(_frontend)
{
}


// detect the device
AP_OpticalFlow_MAVFlow *AP_OpticalFlow_MAVFlow::detect(OpticalFlow &_frontend)
{
    AP_OpticalFlow_MAVFlow *sensor = new AP_OpticalFlow_MAVFlow(_frontend);
    if (!sensor) {
        return nullptr;
    }
    return sensor;
}

// setup the device
bool AP_OpticalFlow_MAVFlow::setup_sensor(void)
{

    // read at 10Hz
    //dev->register_periodic_callback(100000, FUNCTOR_BIND_MEMBER(&AP_OpticalFlow_MAVFlow::timer, void));
    return true;
}

// update - read latest values from sensor and fill in x,y and totals.
void AP_OpticalFlow_MAVFlow::update(void)
{
}

void AP_OpticalFlow_MAVFlow::handle_msg(mavlink_message_t *msg)
{
    /* optical flow */
        mavlink_optical_flow_rad_t flow;
        mavlink_msg_optical_flow_rad_decode(msg, &flow);

        struct OpticalFlow::OpticalFlow_state state {};
        state.device_id = get_address();
        if (flow.integration_time_us > 0) {
            const Vector2f flowScaler = _flowScaler();
            float flowScaleFactorX = 1.0f + 0.001f * flowScaler.x;
            float flowScaleFactorY = 1.0f + 0.001f * flowScaler.y;
            float integralToRate = 1.0e6 / flow.integration_time_us;

            state.surface_quality = flow.quality;
            state.flowRate = Vector2f(flow.integrated_y * flowScaleFactorX * -1.0f,
                                      flow.integrated_x * flowScaleFactorY) * integralToRate;
            state.bodyRate = Vector2f(flow.integrated_ygyro * -1.0f, flow.integrated_xgyro) * integralToRate;

            _applyYaw(state.flowRate);
            _applyYaw(state.bodyRate);

            if(flow.distance >= 0.0f){
                mavrng_last_update_ms = AP_HAL::millis();
                mavrng_dist_cm = flow.distance * 100.0f;
            }
        }
        _update_frontend(state);
}
// timer to read sensor
void AP_OpticalFlow_MAVFlow::timer(void)
{

}
