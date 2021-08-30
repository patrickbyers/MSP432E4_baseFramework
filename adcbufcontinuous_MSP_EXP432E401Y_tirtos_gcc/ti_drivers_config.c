/*
 *  ======== ti_drivers_config.c ========
 *  Configured TI-Drivers module definitions
 *
 *  DO NOT EDIT - This file is generated for the MSP_EXP432E401Y
 *  by the SysConfig tool.
 */

#include <stddef.h>
#include <stdint.h>

#ifndef DeviceFamily_MSP432E401Y
#define DeviceFamily_MSP432E401Y
#endif

#include <ti/devices/DeviceFamily.h>

#include "ti_drivers_config.h"


/*
 * The following (weak) function definition is needed in applications
 * that do *not* use the NDK TCP/IP stack:
 */
void __attribute__((weak)) NDK_hookInit(int32_t id) {}

/*
 *  ============================= Display =============================
 */

#include <ti/display/Display.h>
#include <ti/display/DisplayUart.h>

#define CONFIG_Display_COUNT 1

#define Display_UARTBUFFERSIZE 1024
static char displayUARTBuffer[Display_UARTBUFFERSIZE];

DisplayUart_Object displayUartObject;

const DisplayUart_HWAttrs displayUartHWAttrs = {
    .uartIdx      = CONFIG_UART_0,
    .baudRate     = 115200,
    .mutexTimeout = (unsigned int)(-1),
    .strBuf       = displayUARTBuffer,
    .strBufLen    = Display_UARTBUFFERSIZE
};

const Display_Config Display_config[CONFIG_Display_COUNT] = {
    /* CONFIG_Display_0 */
    /* XDS110 UART */
    {
        .fxnTablePtr = &DisplayUartMin_fxnTable,
        .object      = &displayUartObject,
        .hwAttrs     = &displayUartHWAttrs
    },
};

const uint_least8_t Display_count = CONFIG_Display_COUNT;


/*
 *  =============================== ADCBuf ===============================
 */
/*
#include <ti/drivers/ADCBuf.h>
#include <ti/drivers/adcbuf/ADCBufMSP432E4.h>

#include <ti/devices/msp432e4/inc/msp432.h>

#define CONFIG_ADCBUF_COUNT 1


ADCBufMSP432E4_Object adcbufMSP432E4Objects[CONFIG_ADCBUF_COUNT];


ADCBufMSP432E4_Channels adcBuf0MSP432E4Channels[] = {
    // CONFIG_ADCBUF_0_CHANNEL_0
    {
        .adcPin = ADCBufMSP432E4_PE_3_A0,
        .adcSequence = ADCBufMSP432E4_Seq_0,
        .adcInputMode = ADCBufMSP432E4_SINGLE_ENDED,
        .adcDifferentialPin = ADCBufMSP432E4_PIN_NONE,
        .adcInternalSource = ADCBufMSP432E4_INTERNAL_SOURCE_MODE_OFF,
        .refVoltage = 3300000
    },
};


static ADCBufMSP432E4_SequencePriorities seqPriorities0[ADCBufMSP432E4_SEQUENCER_COUNT] = {
    // Sequencer 0
    ADCBufMSP432E4_Priority_0,
    // Sequencer 1
    ADCBufMSP432E4_Seq_Disable,
    // Sequencer 2
    ADCBufMSP432E4_Seq_Disable,
    // Sequencer 3
    ADCBufMSP432E4_Seq_Disable,
};


static ADCBufMSP432E4_TriggerSource triggerSource0[ADCBufMSP432E4_SEQUENCER_COUNT] = {
    // Sequencer 0 trigger source
    ADCBufMSP432E4_TIMER_TRIGGER,
    // Sequencer 1 trigger source
    ADCBufMSP432E4_SOFTWARE_AUTOMATIC_TRIGGER,
    // Sequencer 2 trigger source
    ADCBufMSP432E4_SOFTWARE_AUTOMATIC_TRIGGER,
    // Sequencer 3 trigger source
    ADCBufMSP432E4_SOFTWARE_AUTOMATIC_TRIGGER,
};


ADCBufMSP432E4_HWAttrsV1 adcbufMSP432E4HWAttrs[CONFIG_ADCBUF_COUNT] = {
    // CONFIG_ADCBUF_0
    {
        .intPriority = (~0),
        .adcBase = ADC0_BASE,
        .channelSetting = adcBuf0MSP432E4Channels,
        .sequencePriority = seqPriorities0,
        .adcTriggerSource = triggerSource0,
        .modulePhase = ADCBufMSP432E4_Phase_Delay_0,
        .refSource = ADCBufMSP432E4_VREF_INTERNAL,
        .useDMA = 1,
        .adcTimerSource = TIMER1_BASE
    },
};


ADCBuf_Config ADCBuf_config[CONFIG_ADCBUF_COUNT] = {
    // CONFIG_ADCBUF_0
    {
        .fxnTablePtr = &ADCBufMSP432E4_fxnTable,
        .object = &adcbufMSP432E4Objects[CONFIG_ADCBUF_0],
        .hwAttrs = &adcbufMSP432E4HWAttrs[CONFIG_ADCBUF_0]
    },
};

const uint_least8_t CONFIG_ADCBUF_0_CONST = CONFIG_ADCBUF_0;
const uint_least8_t CONFIG_ADCBUF_0_CHANNEL_0_CONST = CONFIG_ADCBUF_0_CHANNEL_0;
const uint_least8_t ADCBuf_count = CONFIG_ADCBUF_COUNT;
*/


/*
 *  =============================== DMA ===============================
 */

#include <ti/drivers/dma/UDMAMSP432E4.h>
#include <ti/devices/msp432e4/inc/msp432.h>
#include <ti/devices/msp432e4/driverlib/interrupt.h>
#include <ti/devices/msp432e4/driverlib/udma.h>

/* Ensure DMA control table is aligned as required by the uDMA Hardware */
static tDMAControlTable dmaControlTable[64] __attribute__ ((aligned (1024)));

/* This is the handler for the uDMA error interrupt. */
static void dmaErrorFxn(uintptr_t arg)
{
    int status = uDMAErrorStatusGet();
    uDMAErrorStatusClear();

    /* Suppress unused variable warning */
    (void)status;

    while (1);
}

UDMAMSP432E4_Object udmaMSP432E4Object;

const UDMAMSP432E4_HWAttrs udmaMSP432E4HWAttrs = {
    .controlBaseAddr = (void *)dmaControlTable,
    .dmaErrorFxn     = (UDMAMSP432E4_ErrorFxn)dmaErrorFxn,
    .intNum          = INT_UDMAERR,
    .intPriority     = (~0)
};

const UDMAMSP432E4_Config UDMAMSP432E4_config = {
    .object  = &udmaMSP432E4Object,
    .hwAttrs = &udmaMSP432E4HWAttrs
};


/*
 *  =============================== Power ===============================
 */

#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerMSP432E4.h>
#include <ti/devices/msp432e4/inc/msp432.h>

extern void PowerMSP432E4_sleepPolicy(void);

const PowerMSP432E4_Config PowerMSP432E4_config = {
    .policyFxn             = PowerMSP432E4_sleepPolicy,
    .enablePolicy          = true
};


/*
 *  =============================== UART ===============================
 */

#include <ti/drivers/UART.h>
#include <ti/drivers/uart/UARTMSP432E4.h>
#include <ti/devices/msp432e4/driverlib/interrupt.h>

#define CONFIG_UART_COUNT 1

UARTMSP432E4_Object uartMSP432E4Objects[CONFIG_UART_COUNT];

static unsigned char uartMSP432E4RingBuffer0[32];
static const UARTMSP432E4_HWAttrs uartMSP432E4HWAttrs[CONFIG_UART_COUNT] = {
  {
    .baseAddr           = UART0_BASE,
    .intNum             = INT_UART0,
    .intPriority        = (~0),
    .flowControl        = UARTMSP432E4_FLOWCTRL_NONE,
    .ringBufPtr         = uartMSP432E4RingBuffer0,
    .ringBufSize        = sizeof(uartMSP432E4RingBuffer0),
    .rxPin              = UARTMSP432E4_PA0_U0RX,
    .txPin              = UARTMSP432E4_PA1_U0TX,
    .ctsPin             = UARTMSP432E4_PIN_UNASSIGNED,
    .rtsPin             = UARTMSP432E4_PIN_UNASSIGNED,
    .errorFxn           = NULL
  },
};

const UART_Config UART_config[CONFIG_UART_COUNT] = {
    {   /* CONFIG_UART_0 */
        .fxnTablePtr = &UARTMSP432E4_fxnTable,
        .object      = &uartMSP432E4Objects[CONFIG_UART_0],
        .hwAttrs     = &uartMSP432E4HWAttrs[CONFIG_UART_0]
    },
};

const uint_least8_t CONFIG_UART_0_CONST = CONFIG_UART_0;
const uint_least8_t UART_count = 1;


#include <ti/drivers/Board.h>

/*
 *  ======== Board_initHook ========
 *  Perform any board-specific initialization needed at startup.  This
 *  function is declared weak to allow applications to override it if needed.
 */
void __attribute__((weak)) Board_initHook(void)
{
}

/*
 *  ======== Board_init ========
 *  Perform any initialization needed before using any board APIs
 */
void Board_init(void)
{
    /* ==== /ti/drivers/Power initialization ==== */
    Power_init();

    /* Grant the DMA access to all FLASH memory */
    FLASH_CTRL->PP |= FLASH_PP_DFA;

    /* Region start address - match FLASH start address */
    FLASH_CTRL->DMAST = 0x00000000;

    /*
     * Access to FLASH is granted to the DMA in 2KB regions.  The value
     * assigned to DMASZ is the amount of 2KB regions to which the DMA will
     * have access.  The value can be determined via the following:
     *     2 * (num_regions + 1) KB
     *
     * To grant full access to entire 1MB of FLASH:
     *     2 * (511 + 1) KB = 1024 KB (1 MB)
     */
    FLASH_CTRL->DMASZ = 511;

    Board_initHook();
}
