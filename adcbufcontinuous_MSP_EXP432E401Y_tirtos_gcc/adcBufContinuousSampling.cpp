/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*
 *  ======== adcBufContinuousSampling.c ========
 */
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
//#include <mqueue.h>

/* RTOS header files */
//#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

/* Driver Header files */
#include <ti/drivers/ADCBuf.h>

/* Display Header files */
#include <ti/display/Display.h>

/* Driver configuration */
#include "ti_drivers_config.h"
#include "adcBufCtl.h"
#include "adcBufContinuousSampling.h"

#define THREADSTACKSIZE2    1024

/* Display Driver Handle */
Display_Handle displayHandle;


/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    Display_Params displayParams;

    Display_init();

    /* Configure & open Display driver */
    Display_Params_init(&displayParams);
    displayParams.lineClearMode = DISPLAY_CLEAR_BOTH;
    displayHandle = Display_open(Display_Type_UART, &displayParams);
    if (displayHandle == NULL) {
        Display_printf(displayHandle, 0, 0, "Error creating displayHandle\n");
        while (1);
    }

    Display_printf(displayHandle, 0, 0, "Starting the ADCBufContinuous example");

    /* Add ADC Buffer */
    ADCBuffer adcBuffer0 = ADCBuffer(0);
//    ADCBuffer adcBuffer1 = ADCBuffer(1);

    /* Add channels to ADC Buffer */
    adcBuffer0.AddChannel("Channel 0", 0, ADCBufMSP432E4_Seq_0, 3300000);
    adcBuffer0.AddChannel("Channel 5", 5, ADCBufMSP432E4_Seq_0, 3300000);
    adcBuffer0.AddChannel("Channel 16", 16, ADCBufMSP432E4_Seq_0, 3300000);

//    adcBuffer1.AddChannel("Channel 2", 2, ADCBufMSP432E4_Seq_1, 3300000);

    /* Adjust Sample and Hold time for the ADC Buffer channels */
    adcBuffer0.SetSampleHoldTime(ADCBufMSP432E4_SamplingDuration_PULSE_WIDTH_256);
    adcBuffer0.SetSampleSize(5);
    adcBuffer0.SetSamplingFreq(6);

//    adcBuffer1.SetSampleHoldTime(ADCBufMSP432E4_SamplingDuration_PULSE_WIDTH_256);
//    adcBuffer1.SetSampleSize(5);
//    adcBuffer1.SetSamplingFreq(6);

    /* Initialize the ADC Buffer */
    if(adcBuffer0.Initialize()) { while (1); }

//    if(adcBuffer1.Initialize()) { while (1); }

    /* Start processing the ADC readings */
    adcBuffer0.Start();

//    adcBuffer1.Start();

    while(1) {
        usleep(100);
    }

    Display_printf(displayHandle, 0, 0, "ADC Buffer error: thread closing");
}
