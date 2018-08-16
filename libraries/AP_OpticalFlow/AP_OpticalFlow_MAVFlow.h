#pragma once

#include "OpticalFlow.h"
#include <AP_HAL/utility/OwnPtr.h>

class AP_OpticalFlow_MAVFlow : public OpticalFlow_backend
{
public:
    /// constructor
    AP_OpticalFlow_MAVFlow(OpticalFlow &_frontend);

    // init - initialise the sensor
    void init() override {}

    // update - read latest values from sensor and fill in x,y and totals.
    void update(void) override;

    // detect if the sensor is available
    static AP_OpticalFlow_MAVFlow *detect(OpticalFlow &_frontend);

    // Get update from mavlink
    void handle_msg(mavlink_message_t *msg);

private:
    AP_HAL::OwnPtr<AP_HAL::Device> dev;

    mavlink_optical_flow_rad_t flow;
    bool    have_msg;

    // setup sensor
    bool setup_sensor(void);

    void timer(void);
};
