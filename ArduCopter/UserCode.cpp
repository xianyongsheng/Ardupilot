#include "Copter.h"
#include "ano_com.h"

#define hal_uart hal.uartD//
#define hal_uart_send(x,l) hal_uart->write(x,l)
#define hal_uart_avail() hal_uart->available()
#define hal_uart_read() hal_uart->read()
#define delay_ms(x) hal.scheduler->delay(x)

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
#if 0
    if(g.k_param_optflow_enabled){

        hal_uart->begin(115200);
        hal_uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        for (uint8_t i=0; i<3; i++) {
            delay_ms(1);
            hal_uart->write(0x30);
            hal_uart->write(0x20);
        }
        // since tcdrain() and TCSADRAIN may not be implemented...
        delay_ms(1);

        hal_uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        hal_uart->begin(115200,256,128);
    }
#endif
    delay_ms(10);
}
#endif

#ifdef USERHOOK_FASTLOOP
// receive new packets
void Copter::userhook_FastLoop()
{
    // put your 200Hz code here
    //ano_exchange_run();
#if 0
    if(g.k_param_optflow_enabled){
        uint16_t nbytes = hal_uart_avail();
        while(nbytes--){
            static mavlink_message_t msg;
            static mavlink_status_t status;
            uint8_t c = hal_uart_read();
            if (mavlink_parse_char(1, c, &msg, &status)) {
                copter.optflow.handle_msg(&msg);
            }
        }
    }
#endif
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif
