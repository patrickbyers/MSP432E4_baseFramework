/*
 * adcCtl.h
 *
 *  Created on: Aug 3, 2021
 *      Author: pbyers
 */

#ifndef ADCCTL_H_
#define ADCCTL_H_

#include <stdint.h>
#include <ti/drivers/adc/ADCMSP432E4.h>
#include "baseDevice.h"

//class ADCBuffer;

class ADCChannel : public BaseDevice {
    public:
        ADCChannel(const char* name, uint32_t port, uint8_t pin, ADCMSP432E4_Sequencer seq,
                   uint8_t adcModNum, uint32_t refV);
        ADCChannel(const char* name, uint8_t channel, ADCMSP432E4_Sequencer seq,
                           uint8_t adcModNum, uint32_t refV);

    protected:
        ADCMSP432E4_Sequencer   mSequencer;
        ADCMSP432E4_Module      mAdcModule;
        uint8_t                 mChannel = -1;
        uint32_t                mRefVoltage;
        uint32_t                mPortPinConfigVal;
};


#endif /* ADCCTL_H_ */
