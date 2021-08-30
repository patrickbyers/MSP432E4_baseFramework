/*
 * adcCtl.cpp
 *
 *  Created on: Aug 5, 2021
 *      Author: pbyers
 */

#include <string>

/* Driver configuration */
#include "ti_drivers_config.h"
#include "adcCtl.h"

/* Device configuration and definitions */
#include <ti/devices/msp432e4/inc/msp432.h>

#define MAX_NUM_ADC_CHANNELS (24)

/*
 *  ADC port/pin defines for pin configuration.  Ports B, D, E, and K are
 *  configurable through the port mapping controller.  None of the port
 *  mappings support ADC.
 *  Channel specifies the ADC channel and ranges from 0 to 20.
 *  pin range: 0 - 7, port range: 0 - 15
 *
 *
 *    32 - 21  20 - 16  15 - 8  7 - 0
 *  ---------------------------------
 *  | X X X X | CHANNEL | PORT | PIN |
 *  ---------------------------------#include <stdint.h>
 *
 *  channel = (((config) >> 16) & 0x1F)
 *  port = (((config << 4) & 0x000FF000) | 0x40000000)
 *  pin = ((config) & 0xFF)
 *
 */
#define ADC_CONFIG_PIN(chan, port, pin) ((((chan) << 16) & 0x001F0000) | (((port) >> 4) & 0x0000FF00) | ((pin) & 0x000000FF))
#define ADC_pinConfigChannel(config)  (((config) >> 16) & 0x1F)


static const uint32_t ADCChannelList[MAX_NUM_ADC_CHANNELS] = {
    PORT_E | PIN_3,     // CHANNEL 0
    PORT_E | PIN_2,     // CHANNEL 1
    PORT_E | PIN_1,     // CHANNEL 2
    PORT_E | PIN_0,     // CHANNEL 3
    PORT_D | PIN_7,     // CHANNEL 4
    PORT_D | PIN_6,     // CHANNEL 5
    PORT_D | PIN_5,     // CHANNEL 6
    PORT_D | PIN_4,     // CHANNEL 7
    PORT_E | PIN_5,     // CHANNEL 8
    PORT_E | PIN_4,     // CHANNEL 9
    PORT_B | PIN_4,     // CHANNEL 10
    PORT_B | PIN_5,     // CHANNEL 11
    PORT_D | PIN_3,     // CHANNEL 12
    PORT_D | PIN_2,     // CHANNEL 13
    PORT_D | PIN_1,     // CHANNEL 14
    PORT_D | PIN_0,     // CHANNEL 15
    PORT_K | PIN_0,     // CHANNEL 16
    PORT_K | PIN_1,     // CHANNEL 17
    PORT_K | PIN_2,     // CHANNEL 18
    PORT_K | PIN_3,     // CHANNEL 19
    PORT_E | PIN_6,     // CHANNEL 20
    PORT_E | PIN_7,     // CHANNEL 21
    PORT_P | PIN_6,     // CHANNEL 22
    PORT_P | PIN_7,     // CHANNEL 23
};



ADCChannel::ADCChannel(const char* name, uint32_t port, uint8_t pin, ADCMSP432E4_Sequencer seq,
                   uint8_t  adcModNum, uint32_t refV) :
                   BaseDevice(name, port, pin, "ADC "+std::to_string(adcModNum)),
                   mSequencer(seq)
{

    mAdcModule = adcModNum?((ADCMSP432E4_Module)ADC1_BASE):((ADCMSP432E4_Module)ADC0_BASE);

    for(uint8_t i=0; i<MAX_NUM_ADC_CHANNELS; i++)
    {
        mChannel = (ADCChannelList[i]==(port & pin))?i:mChannel;
        if (mChannel >= 0) { break; }
    }

    mRefVoltage = refV>3600000?3600000:(refV<2400000?2400000:refV);

    mPortPinConfigVal = ADC_CONFIG_PIN(mChannel, mPort, mPin);
}

ADCChannel::ADCChannel(const char* name, uint8_t channel, ADCMSP432E4_Sequencer seq,
                       uint8_t adcModNum, uint32_t refV) :
                BaseDevice(name, (ADCChannelList[channel] & 0x000FF000),
                           (ADCChannelList[channel] & 0x000000FF), "ADC "+std::to_string(adcModNum)),
                mSequencer(seq), mChannel(channel)
{
    mAdcModule = adcModNum?((ADCMSP432E4_Module)ADC1_BASE):((ADCMSP432E4_Module)ADC0_BASE);
    mRefVoltage = refV>3600000?3600000:(refV<2400000?2400000:refV);

    mPortPinConfigVal = ADC_CONFIG_PIN(mChannel, mPort, mPin);
}

