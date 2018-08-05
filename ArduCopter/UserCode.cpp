#include "Copter.h"
#include "ano_com.h"

#define hal_uart hal.uartC//
#define hal_uart_send(x,l) hal_uart->write(x,l)
#define hal_uart_avail() hal_uart->available()
#define hal_uart_read() hal_uart->read()
#define delay_ms(x) hal.scheduler->delay(x)

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    hal_uart->begin(57600);
    hal_uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    delay_ms(1000);

}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
    ano_exchange_run();
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
