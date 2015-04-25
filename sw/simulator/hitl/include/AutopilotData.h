/*
 * AutopilotData.h
 *
 *  Created on: Apr 25, 2015
 *      Author: fwmav
 */

#ifndef AUTOPILOTDATA_H_
#define AUTOPILOTDATA_H_

#include "MsgConfig.h"

/*
 * Actual data
 */
class AutopilotData {
public:
    uint32_t msg_cnt; // Pckg counter
    uint8_t actuators_nb; // NB actuators
    int16_t actuators[ACTUATORS_NB] = {0}; // Actuators value, int16

    AutopilotData(){
      msg_cnt = 0;
      actuators_nb = ACTUATORS_NB;
    }
};

#endif /* AUTOPILOTDATA_H_ */
