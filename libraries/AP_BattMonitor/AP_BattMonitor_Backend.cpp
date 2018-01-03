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

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"

/*
  base class constructor.
  This incorporates initialisation as well.
*/
AP_BattMonitor_Backend::AP_BattMonitor_Backend(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state) :
        _mon(mon),
        _state(mon_state)
{
}

uint16_t voltage_table_v[28] = {    422,416,415,414,412,410,408,405,403,397,393,390,387,384,381,379,377,376,374,373,372,371,366,365,364,363,361,359};
uint8_t  voltage_table_mah[28] = { 100,100, 99, 97, 95, 92, 90, 87, 85, 80, 75, 70, 65, 60, 55, 50, 45, 42, 35, 30, 25, 15, 12, 10,  8,  5,  3,  1};

/// capacity_remaining_pct - returns the % battery capacity remaining (0 ~ 100)
uint8_t AP_BattMonitor_Backend::capacity_remaining_pct() const
{
    static float mah_init=456.78;
    static uint8_t battery_type;
    if(mah_init==456.78f)
    {
        uint16_t voltage;
        uint8_t i;
        voltage = _state.voltage*100;
        for(i=1;i<50;i++){
            if(voltage < (425*i)){
                battery_type=i;
                break;
            }
            if(i>=49)battery_type=0;
        }
        for(i=0;i<28;i++){
            if(voltage > (voltage_table_v[i]*battery_type)){
                    mah_init = ((100 - voltage_table_mah[i]) * _mon._pack_capacity[_state.instance]) *0.01f;
                    break;
            }else if(i==27){
                mah_init = _mon._pack_capacity[_state.instance];
                break;
            }
        }
    }
    float mah_remaining = _mon._pack_capacity[_state.instance] - _state.current_total_mah - mah_init;
    if(mah_remaining<0)mah_remaining=0;
    if ( _mon._pack_capacity[_state.instance] > 10 ) { // a very very small battery
        return (100 * (mah_remaining) / _mon._pack_capacity[_state.instance]);
    } else {
        return 0;
    }
/*
    float mah_remaining = _mon._pack_capacity[_state.instance] - _state.current_total_mah;
    if ( _mon._pack_capacity[_state.instance] > 10 ) { // a very very small battery
        return (100 * (mah_remaining) / _mon._pack_capacity[_state.instance]);
    } else {
        return 0;
    }
    */
}

/// get capacity for this instance
int32_t AP_BattMonitor_Backend::get_capacity() const
{
    return _mon.pack_capacity_mah(_state.instance);
}
