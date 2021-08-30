/*
 * adcBufCtl.h
 *
 *  Created on: Jul 9, 2021
 *      Author: pbyers
 */

#ifndef ADCBUFCTL_H_
#define ADCBUFCTL_H_

/* POSIX Header files */
#include <stdint.h>
#include <mqueue.h>
/* Driver Header files */
#include <ti/drivers/adcbuf/ADCBufMSP432E4.h>
/* Driver configuration Header files*/
#include "adcCtl.h"


#define CONFIG_ADCBUF_0             0


typedef struct {
    uint8_t     bufId;
    uint8_t     channelMaxCount;
    uint8_t     channelConfigCount;
    uint8_t     adcSampleSize;
    uint8_t     sequencersActive;
    uint32_t    messageSize;
    uint16_t*   pTransferBuffer;
    uint16_t*   pProcessBuffer;
    mqd_t       queueReceive;
    mqd_t       queueSend;
    ADCBufMSP432E4_SamplingDuration sampleHold;
    uint32_t    sampleFreq;
} ADCBuf_ConfigParams;


class ADCBufConversion;


class ADCBuffer {
    public:
        ADCBuffer(int moduleId);
        ADCBuffer(int moduleId, int channelCount);
        void    AddChannel(const char* name, uint8_t ch, ADCBufMSP432E4_Sequencer seq, uint32_t refV);
        uint8_t Initialize();
        void    Start();
        void    PostChannelValue(int configId, uint16_t value) { m_pChannelList[configId]->SetValue((int)value); }
        uint16_t GetChannelValue(int configId) { return((uint16_t)m_pChannelList[configId]->GetValue()); }
//        int GetChannelCount() { return mConfigParams.channelConfigCount; }
        ADCBuf_ConfigParams    GetConfigParams() { return (mConfigParams); }
        void    SetSampleHoldTime(ADCBufMSP432E4_SamplingDuration sh) { mConfigParams.sampleHold = sh; }
        void    SetSamplingFreq(uint32_t freq) { mConfigParams.sampleFreq = freq; }
        void    SetSampleSize(uint8_t sampSize) { mConfigParams.adcSampleSize = (sampSize<100?sampSize:100); }

        void    Print();

    private:
        void ConfigParamsInit(uint8_t modId);

        ADCBuf_ConfigParams mConfigParams;

        uint16_t*   m_pSampleBufferOne;
        uint16_t*   m_pSampleBufferTwo;
        uint16_t*   m_pSampleTransferBuffer;
        uint16_t*   m_pSampleProcessBuffer;

        pthread_t           mThread;
        pthread_attr_t      mThreadAttrs;
        struct sched_param  mThreadSchedParam;

        struct mq_attr      mMqAttr;
        //mqd_t       queueReceive;
        //mqd_t       queueSend;
        mqd_t       msgRcv;
        mqd_t       msgSnd;

        ADCBuf_Handle       mAdcBufHandle;
        ADCBuf_Params       mAdcBufParams;
        ADCBufMSP432E4_ParamsExtension  mParamsExt;

        ADCChannel**        m_pChannelList;
        ADCBufMSP432E4_Channels*    m_pChannelConfigList;

        ADCBufConversion*   m_pAdcBufConversion;
        ADCBuf_Conversion*  mBufConversionStruct;

        ADCBufMSP432E4_SequencePriorities   mSequencerPriorities[ADCBufMSP432E4_SEQUENCER_COUNT];
        ADCBufMSP432E4_TriggerSource    mTriggerSource[ADCBufMSP432E4_SEQUENCER_COUNT];
};

class ADCBufConversion {
    public:
        ADCBufConversion(ADCBufMSP432E4_Channels* channels);
        ADCBufConversion(int chanCount);
        void AddChannel(ADCChannel* chan);
        void LockConversionStruct();
    private:
        int mChannelCount = 0;
        ADCBuf_Conversion*          m_pBufConversionStruct;
        ADCBufMSP432E4_Channels*    m_pConversionChannelList;
};

#endif /* ADCBUFCTL_H_ */
