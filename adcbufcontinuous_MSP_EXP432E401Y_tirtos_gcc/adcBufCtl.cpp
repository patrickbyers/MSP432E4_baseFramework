/*
 * adcBufCtl.c
 *
 *  Created on: Jul 9, 2021
 *      Author: pbyers
 */

#include <stdlib.h>
#include <unistd.h>

/* POSIX Header files */
#include <pthread.h>
//#include <mqueue.h>

/* Driver Header files */
#include <ti/drivers/ADCBuf.h>

/* Driver configuration */
#include "ti_drivers_config.h"
#include "adcBufCtl.h"

/* Display Header files */
#include <ti/display/Display.h>

/* Device configuration and definitions */
#include <ti/devices/msp432e4/inc/msp432.h>


//#define ADCSAMPLESIZE    (5)
//#define MAX_QUEUED_ADC_CONVERSIONS (4)
//#define QUEUED_ADC_MESSAGE_SIZE (sizeof(uint32_t) * 5)
#define THREADSTACKSIZE    1024


extern const uint_least8_t ADCBuf_count = CONFIG_ADCBUF_COUNT;




/*
 *  ADCBuf port/pin defines for pin configuration.  Ports B, D, E, and K are
 *  configurable through the port mapping controller.  None of the port
 *  mappings support ADC.
 *  Channel specifies the ADC channel and ranges from 0 to 23.
 *  pin range: 0 - 7, port range: 0 - 15
 *
 *      31-24        23-16      15-8     7-0
 *  -------------------------------------------
 *  |  RESERVED  |  CHANNEL  |  PORT  |  PIN  |
 *  -------------------------------------------
 *
 *  channel = (((config) >> 16) & 0x10F)
 *  port    = (((config << 4) & 0x000FF000) | 0x40000000)
 *  pin     = ((config) & 0xFF)
 *
 */
const int adcBufChannelPins[24] = {
        ADCBufMSP432E4_PE_3_A0, ADCBufMSP432E4_PE_2_A1, ADCBufMSP432E4_PE_1_A2, ADCBufMSP432E4_PE_0_A3,
        ADCBufMSP432E4_PD_7_A4, ADCBufMSP432E4_PD_6_A5, ADCBufMSP432E4_PD_5_A6, ADCBufMSP432E4_PD_4_A7,
        ADCBufMSP432E4_PE_5_A8, ADCBufMSP432E4_PE_4_A9, ADCBufMSP432E4_PB_4_A10, ADCBufMSP432E4_PB_5_A11,
        ADCBufMSP432E4_PD_3_A12, ADCBufMSP432E4_PD_2_A13, ADCBufMSP432E4_PD_1_A14, ADCBufMSP432E4_PD_0_A15,
        ADCBufMSP432E4_PK_0_A16, ADCBufMSP432E4_PK_1_A17, ADCBufMSP432E4_PK_2_A18, ADCBufMSP432E4_PK_3_A19,
        ADCBufMSP432E4_PE_6_A20, ADCBufMSP432E4_PE_7_A21, ADCBufMSP432E4_PP_7_A22, ADCBufMSP432E4_PP_6_A23,
};


ADCBufMSP432E4_HWAttrsV1    adcbufMSP432E4HWAttrs[CONFIG_ADCBUF_COUNT];
ADCBufMSP432E4_Object       adcbufMSP432E4Objects[CONFIG_ADCBUF_COUNT];
ADCBuf_Config               ADCBuf_config[CONFIG_ADCBUF_COUNT];


uint32_t buffersCompletedCounter = 0;

/* Display Driver Handle */
extern Display_Handle displayHandle;


/*
 * This function is called whenever an ADC buffer is full.
 * The content of the buffer is then converted into human-readable format and
 * sent to the main thread.
 */
void adcBufCallback(ADCBuf_Handle handle, ADCBuf_Conversion *conversion,
    void *completedADCBuffer, uint32_t completedChannel, int_fast16_t status)
{
    ADCBuf_ConfigParams *bufConfigParams = (ADCBuf_ConfigParams *) conversion->arg;
    uint16_t    *transferBuffer = bufConfigParams->pTransferBuffer;
    uint16_t    *pCompletedBuffer = (uint16_t *) completedADCBuffer;

    for (int j = 0; j < (bufConfigParams->channelConfigCount * bufConfigParams->adcSampleSize); j++)
    {
        transferBuffer[j] = pCompletedBuffer[j];
    }
    /* Send ADC data to main thread using message queue */
    mq_send(bufConfigParams->queueSend, (char *) transferBuffer, (int)bufConfigParams->messageSize, 0);
}

void *adcBufProcess(void *arg0)
{
    ADCBuffer* m_pBuf = (ADCBuffer *) arg0;

    //uint8_t channelCount = m_pBuf->GetChannelCount();
    ADCBuf_ConfigParams bufConfigParams = m_pBuf->GetConfigParams();

    uint64_t* average =
            (uint64_t *)malloc(bufConfigParams.channelConfigCount * sizeof(uint64_t));

    uint_fast16_t i = 0;
    uint_fast16_t j = 0;
    uint_fast16_t k = 0;
    /*
     * Wait for the message queue to receive ADC data from the callback
     * function. When data is received, calculate the average and print to UART
     */
//    while(buffersCompletedCounter < 4) {
    while(1) {
        if (mq_receive(bufConfigParams.queueReceive, (char *) bufConfigParams.pProcessBuffer,
                       (int) bufConfigParams.messageSize, NULL) == -1)
        {
            Display_printf(displayHandle, 0, 0, "Error receiving ADC message");
            while(1);
        }

        Display_printf(displayHandle, 0, 0, "Buffer %u finished:",
                    (unsigned int)buffersCompletedCounter++);


        /* Calculate average ADC data value in uV */
        k=0;
        for (j=0; j<bufConfigParams.channelConfigCount; j++) { average[j] = 0; }

        for (i=0; i<(bufConfigParams.adcSampleSize); i++) {
            for (j = 0; j < bufConfigParams.channelConfigCount; j++) {
                //Display_printf(displayHandle, 0, 0, "i=%u j=%u k=%u reading=%u",i,j,k,bufConfigParams.pProcessBuffer[k]);
                average[j] += bufConfigParams.pProcessBuffer[k++];
                //Display_printf(displayHandle, 0, 0, "average[%u] = %u",j,average[j]);
            }
        }
        for (j=0; j<bufConfigParams.channelConfigCount; j++)
        {
            average[j] = average[j]/bufConfigParams.adcSampleSize;
            m_pBuf->PostChannelValue(j, average[j]);
        }

        m_pBuf->Print();
    }
    Display_printf(displayHandle, 0, 0, "Leaving ADC Buffer processing thread");
}


/*
 * ADC Buffer constructor
 *
 * Params:
 *  moduleId:       Module # - 0 or 1
 *
 *  max # of channels or device will be implied (24)
 *
 */
ADCBuffer::ADCBuffer(int moduleId)
{
    int seqIndex;

    this->ConfigParamsInit(moduleId);

    for (seqIndex=0; seqIndex<ADCBufMSP432E4_SEQUENCER_COUNT; seqIndex++)
    {
        mSequencerPriorities[seqIndex] = ADCBufMSP432E4_Seq_Disable;
        mTriggerSource[seqIndex] = ADCBufMSP432E4_SOFTWARE_AUTOMATIC_TRIGGER;
    }

    m_pChannelList = (ADCChannel **) malloc(mConfigParams.channelMaxCount * sizeof(ADCChannel*));
    m_pChannelConfigList = (ADCBufMSP432E4_Channels *) malloc(mConfigParams.channelMaxCount * sizeof(ADCBufMSP432E4_Channels));
}


/*
 * ADC Buffer constructor
 *
 * Params:
 *  moduleId:       Module # - 0 or 1
 *  channelCount:   Max # of channels for module
 *
 */
ADCBuffer::ADCBuffer(int moduleId, int channelCount)
{
    int seqIndex;

    mConfigParams.channelMaxCount = channelCount;
    this->ConfigParamsInit(moduleId);

    for (seqIndex=0; seqIndex<ADCBufMSP432E4_SEQUENCER_COUNT; seqIndex++)
    {
        mSequencerPriorities[seqIndex] = ADCBufMSP432E4_Seq_Disable;
        mTriggerSource[seqIndex] = ADCBufMSP432E4_SOFTWARE_AUTOMATIC_TRIGGER;
    }

    m_pChannelList = (ADCChannel **) malloc(mConfigParams.channelMaxCount * sizeof(ADCChannel*));
    m_pChannelConfigList = (ADCBufMSP432E4_Channels *) malloc(mConfigParams.channelMaxCount * sizeof(ADCBufMSP432E4_Channels));
}

/*
 * Initializes ADC Buffer configuration parameters
 */
void ADCBuffer::ConfigParamsInit(uint8_t modId)
{
    mConfigParams.bufId = modId;
    mConfigParams.channelConfigCount = 0;
    mConfigParams.sequencersActive = 0;
    mConfigParams.adcSampleSize = 0;
    mConfigParams.sampleFreq = 0;

    if (mConfigParams.channelMaxCount == 0) mConfigParams.channelMaxCount = MSP432E4_NUM_ADC_CHANNELS;
}




void ADCBuffer::AddChannel(const char* name, uint8_t ch, ADCBufMSP432E4_Sequencer seq, uint32_t refV)
{
    m_pChannelConfigList[mConfigParams.channelConfigCount].adcPin = adcBufChannelPins[ch];
    m_pChannelConfigList[mConfigParams.channelConfigCount].adcSequence = seq;
    m_pChannelConfigList[mConfigParams.channelConfigCount].adcInputMode = ADCBufMSP432E4_SINGLE_ENDED;
    m_pChannelConfigList[mConfigParams.channelConfigCount].adcDifferentialPin = ADCBufMSP432E4_PIN_NONE;
    m_pChannelConfigList[mConfigParams.channelConfigCount].adcInternalSource = ADCBufMSP432E4_INTERNAL_SOURCE_MODE_OFF;
    m_pChannelConfigList[mConfigParams.channelConfigCount].refVoltage = refV;

    m_pChannelList[mConfigParams.channelConfigCount] = new ADCChannel(name, ch, (ADCMSP432E4_Sequencer)seq, mConfigParams.bufId, refV);

    if (mSequencerPriorities[seq] == ADCBufMSP432E4_Seq_Disable) {
        mSequencerPriorities[seq] = ADCBufMSP432E4_Priority_0;
        mTriggerSource[seq] = ADCBufMSP432E4_TIMER_TRIGGER;
        mConfigParams.sequencersActive++;
//        mSequencersActive++;
    }

    mConfigParams.channelConfigCount++;
//    mChannelConfigCount++;
}


/*
 * Initializes ADC Buffer module
 *
 * This should be called only after initializing the config params and adding all channels, just before Start()
 */
uint8_t ADCBuffer::Initialize()
{
    int id = mConfigParams.bufId;

    if(mConfigParams.channelConfigCount <= mConfigParams.channelMaxCount) {

        if (mConfigParams.adcSampleSize == 0) { mConfigParams.adcSampleSize = 5; }
        if (mConfigParams.sampleFreq == 0) { mConfigParams.sampleFreq = 4 * mConfigParams.adcSampleSize * mConfigParams.channelConfigCount; }

        mConfigParams.messageSize = (sizeof(uint16_t) * mConfigParams.adcSampleSize * mConfigParams.channelConfigCount);

        adcbufMSP432E4HWAttrs[id].intPriority = (~0);
        adcbufMSP432E4HWAttrs[id].adcBase = ADC0_BASE;
        adcbufMSP432E4HWAttrs[id].channelSetting = m_pChannelConfigList;
        adcbufMSP432E4HWAttrs[id].sequencePriority = mSequencerPriorities;
        adcbufMSP432E4HWAttrs[id].adcTriggerSource = mTriggerSource;
        adcbufMSP432E4HWAttrs[id].modulePhase = ADCBufMSP432E4_Phase_Delay_0;
        adcbufMSP432E4HWAttrs[id].refSource = ADCBufMSP432E4_VREF_INTERNAL;
        adcbufMSP432E4HWAttrs[id].useDMA = 1;
        adcbufMSP432E4HWAttrs[id].adcTimerSource = TIMER1_BASE;

        ADCBuf_config[id].fxnTablePtr = &ADCBufMSP432E4_fxnTable;
        ADCBuf_config[id].object = &adcbufMSP432E4Objects[id];
        ADCBuf_config[id].hwAttrs = &adcbufMSP432E4HWAttrs[id];

        ADCBuf_init();

        return(0);
    }
    return(1);
}



void ADCBuffer::Start()
{
    int                 retc;

    ADCBufMSP432E4_ParamsExtension paramsExt;
    paramsExt.samplingDuration = mConfigParams.sampleHold;

    /* Create RTOS Queue */
    mMqAttr.mq_flags = 0;
    mMqAttr.mq_maxmsg = 5;
    mMqAttr.mq_msgsize = mConfigParams.messageSize;
    mMqAttr.mq_curmsgs = 0;

    msgRcv = mq_open("ADCBuf", O_RDWR | O_CREAT, 0664, &mMqAttr);
    msgSnd = mq_open("ADCBuf", O_RDWR | O_CREAT | O_NONBLOCK, 0664, &mMqAttr);
    mConfigParams.queueReceive = msgRcv;
    mConfigParams.queueSend = msgSnd;

    /* Set up an ADCBuf peripheral in ADCBuf_RECURRENCE_MODE_CONTINUOUS */
    ADCBuf_Params_init(&mAdcBufParams);
    mAdcBufParams.callbackFxn = adcBufCallback;
    mAdcBufParams.recurrenceMode = ADCBuf_RECURRENCE_MODE_CONTINUOUS;
    mAdcBufParams.returnMode = ADCBuf_RETURN_MODE_CALLBACK;
    mAdcBufParams.samplingFrequency = mConfigParams.sampleFreq;
    mAdcBufParams.custom = &paramsExt;

    mAdcBufHandle = ADCBuf_open(CONFIG_ADCBUF_0, &mAdcBufParams);

    m_pSampleBufferOne = (uint16_t*)malloc(sizeof(uint16_t) * mConfigParams.adcSampleSize * mConfigParams.channelConfigCount);
    m_pSampleBufferTwo = (uint16_t*)malloc(sizeof(uint16_t) * mConfigParams.adcSampleSize * mConfigParams.channelConfigCount);
    m_pSampleTransferBuffer = (uint16_t*)malloc(sizeof(uint16_t) * mConfigParams.adcSampleSize * mConfigParams.channelConfigCount);
    m_pSampleProcessBuffer = (uint16_t*)malloc(sizeof(uint16_t) * mConfigParams.adcSampleSize * mConfigParams.channelConfigCount);

    mConfigParams.pTransferBuffer = m_pSampleTransferBuffer;
    mConfigParams.pProcessBuffer = m_pSampleProcessBuffer;

    m_pAdcBufConversion = (ADCBufConversion *) malloc(sizeof(ADCBufConversion) * mConfigParams.sequencersActive);

    for(int conversionStructIndex=0; conversionStructIndex < mConfigParams.sequencersActive; conversionStructIndex++)
    {
        m_pAdcBufConversion[conversionStructIndex] = ADCBufConversion(m_pChannelConfigList);
    }


    /* Configure the conversion struct */
    mBufConversionStruct = (ADCBuf_Conversion *) malloc(mConfigParams.channelConfigCount * sizeof(ADCBuf_Conversion));

    mBufConversionStruct[0].arg = &mConfigParams;
    mBufConversionStruct[0].adcChannel = 0;
    mBufConversionStruct[0].sampleBuffer = m_pSampleBufferOne;
    mBufConversionStruct[0].sampleBufferTwo = m_pSampleBufferTwo;
    mBufConversionStruct[0].samplesRequestedCount = (mConfigParams.adcSampleSize * mConfigParams.channelConfigCount);

    for (int i=1; i<mConfigParams.channelConfigCount; i++)
    {
        mBufConversionStruct[i].arg = NULL;
        mBufConversionStruct[i].adcChannel = i;
        mBufConversionStruct[i].sampleBuffer = NULL;
        mBufConversionStruct[i].sampleBufferTwo = NULL;
        mBufConversionStruct[i].samplesRequestedCount = mConfigParams.adcSampleSize * mConfigParams.channelConfigCount;
    }

    if (mAdcBufHandle == NULL){
        /* ADCBuf failed to open. */
        while(1);
    }

    /* Start converting. */
    if (ADCBuf_convert(mAdcBufHandle, mBufConversionStruct, mConfigParams.channelConfigCount) !=
        ADCBuf_STATUS_SUCCESS) {
        /* Did not start conversion process correctly. */
        while(1);
    }

    /* Initialize the attributes structure with default values */
    pthread_attr_init(&mThreadAttrs);

    /* Set priority, detach state, and stack size attributes */
    mThreadSchedParam.sched_priority = 1;
    retc = pthread_attr_setschedparam(&mThreadAttrs, &mThreadSchedParam);
    retc |= pthread_attr_setdetachstate(&mThreadAttrs, PTHREAD_CREATE_DETACHED);
    retc |= pthread_attr_setstacksize(&mThreadAttrs, THREADSTACKSIZE);
    if (retc != 0) {
        /* failed to set attributes */
        while (1) {}
    }

    retc = pthread_create(&(this->mThread), &(this->mThreadAttrs), adcBufProcess, (void *)this);
    if (retc != 0) {
        /* pthread_create() failed */
        while (1) {}
    }
}


void ADCBuffer::Print()
{
    for(int i=0; i<mConfigParams.channelConfigCount; i++)
    {
        Display_printf(displayHandle, 0, 0, "  %s - %s (%c%u): %u", this->m_pChannelList[i]->m_pDevName.c_str(),
                           this->m_pChannelList[i]->m_pName.c_str(),
                           this->m_pChannelList[i]->GetPort(), this->m_pChannelList[i]->GetPin(),
                           this->GetChannelValue(i));
    }
}



ADCBufConversion::ADCBufConversion(ADCBufMSP432E4_Channels* channels)
{
    m_pConversionChannelList = channels;
    //mBufConversionStruct.adcChannel=
}

ADCBufConversion::ADCBufConversion(int chanCount)
{
    mChannelCount = chanCount;
    m_pBufConversionStruct = (ADCBuf_Conversion *) malloc(sizeof(ADCBuf_Conversion)*mChannelCount);
}

void ADCBufConversion::AddChannel(ADCChannel* chan)
{
    ;
}

void ADCBufConversion::LockConversionStruct()
{
    ;
}

