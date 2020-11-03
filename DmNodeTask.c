/*
 * Copyright (c) 2016, Texas Instruments Incorporated
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

/***** Includes *****/
#include <DmNodeRadioTask.h>
#include <DmNodeTask.h>
#include <xdc/std.h>
#include <xdc/runtime/System.h>

#include <string.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Clock.h>

#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/mw/display/Display.h>
#include <ti/mw/display/DisplayExt.h>

#include "RadioProtocol.h"              //tugce

/* Board Header files */
#include "Board.h"

#include "SceAdc.h"
#include "seb/SEB.h"                    //tugce

#ifdef USE_BIM
#include "bim/BimFactoryReset.h"
#endif


/***** Defines *****/
#define NODE_TASK_STACK_SIZE 1024
#define NODE_TASK_PRIORITY   3

#define NODE_EVENT_ALL                  0xFFFFFFFF
#define NODE_EVENT_NEW_ADC_VALUE    (uint32_t)(1 << 0)
#define NODE_EVENT_UPDATE_LCD       (uint32_t)(1 << 1)

#ifdef USE_BIM
#define NODE_EVENT_FACTORY_RESET    (uint32_t)(1 << 2)
#endif

/* A change mask of 0xFF0 means that changes in the lower 4 bits does not trigger a wakeup. */
#define NODE_ADCTASK_CHANGE_MASK                    0xFF0

/* Minimum slow Report interval is 50s (in units of samplingTime)*/
#define NODE_ADCTASK_REPORTINTERVAL_SLOW                50
/* Minimum fast Report interval is 1s (in units of samplingTime) for 30s*/
#define NODE_ADCTASK_REPORTINTERVAL_FAST                1
#define NODE_ADCTASK_REPORTINTERVAL_FAST_DURIATION_MS   30000

#ifdef USE_BIM
/* BLE Load button check timers - check button ever 100ms for 5s*/
#define NODE_BLE_BOOTLODER_BUTTON_CHECK_TIMER_MS    100
#define NODE_BLE_BOOTLODER_BUTTON_CHECK_DURATION_S  6
#endif


/***** Variable declarations *****/
static Task_Params nodeTaskParams;
Task_Struct nodeTask;    /* not static so you can see in ROV */
static uint8_t nodeTaskStack[NODE_TASK_STACK_SIZE];
Event_Struct nodeEvent;  /* not static so you can see in ROV */
static Event_Handle nodeEventHandle;
static uint16_t latestAdcValue;

/* Clock for the fast report timeout */
Clock_Struct fastReportTimeoutClock;     /* not static so you can see in ROV */
static Clock_Handle fastReportTimeoutClockHandle;

static struct DualModeSensorPacket dmSensorPacket;              //tugce   31102017
static struct AtsaModePacket atsaPacket;                        //tugce   03112017

#ifdef USE_BIM
/* Clock for the fast report timeout */
Clock_Struct bleBootloadBtnCheckClock;     /* not static so you can see in ROV */
static Clock_Handle bleBootloadBtnCheckClockHandle;
#endif

/* Pin driver handle */
static PIN_Handle buttonPinHandle;
PIN_Handle ledPinHandle;

/* Global memory storage for a PIN_Config table */
static PIN_State buttonPinState;
static PIN_State ledPinState;

/* Display driver handles */
static Display_Handle hDisplayLcd;
static Display_Handle hDisplaySerial;

static Node_AdertiserType advertisementType = Node_AdertiserMsUrl;

/*tugce 03112017*/
typedef struct
{
    uint8_t   frameType;      // TLM
    uint8_t   version;        // 0x00 for now
    uint8_t   vBatt[2];       // Battery Voltage, 1mV/bit, Big Endian
    uint8_t   temp[2];        // Temperature. Signed 8.8 fixed point
    uint8_t   advCnt[4];      // Adv count since power-up/reboot
    uint8_t   secCnt[4];      // Time since power-up/reboot
                              // in 0.1 second resolution
    uint32_t   deneme;         //tugce
    char       buf[10];
} print_t;
/*tugce 03112017*/

static print_t   ePrintFrame;      //tugce

/* Enable the 3.3V power domain used by the LCD */
PIN_Config pinTable[] = {
    NODE_SUB1_ACTIVITY_LED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    NODE_BLE_ACTIVITY_LED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_DIO1_RFSW | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_DIO30_SWPWR | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

/*
 * Application button pin configuration table:
 *   - Buttons interrupts are configured to trigger on falling edge.
 */
PIN_Config buttonPinTable[] = {
    Board_BUTTON0  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    Board_BUTTON1  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    PIN_TERMINATE
};

static uint8_t nodeAddress = 0;
static uint8_t frameTypeValue = 0;      //tugce

/***** Prototypes *****/
static void nodeTaskFunction(UArg arg0, UArg arg1);
static void updateLcd(void);
void fastReportTimeoutCallback(UArg arg0);
void adcCallback(uint16_t adcValue);
void buttonCallback(PIN_Handle handle, PIN_Id pinId);

#ifdef USE_BIM
void bleBtnCheckTimerCallback(UArg arg0);
#endif

/***** Function definitions *****/
void NodeTask_init(void)
{

    /* Create event used internally for state changes */
    Event_Params eventParam;
    Event_Params_init(&eventParam);
    Event_construct(&nodeEvent, &eventParam);
    nodeEventHandle = Event_handle(&nodeEvent);

    /* Create clock object which is used for fast report timeout */
    Clock_Params clkParams;
    clkParams.period = 0;
    clkParams.startFlag = FALSE;
    Clock_construct(&fastReportTimeoutClock, fastReportTimeoutCallback, 1, &clkParams);
    fastReportTimeoutClockHandle = Clock_handle(&fastReportTimeoutClock);

#ifdef USE_BIM
    /* Create clock object which is used for ble bootload button check */
    Clock_construct(&bleBootloadBtnCheckClock, bleBtnCheckTimerCallback, 1, &clkParams);
    bleBootloadBtnCheckClockHandle = Clock_handle(&bleBootloadBtnCheckClock);
#endif

    /* Create the node task */
    Task_Params_init(&nodeTaskParams);
    nodeTaskParams.stackSize = NODE_TASK_STACK_SIZE;
    nodeTaskParams.priority = NODE_TASK_PRIORITY;
    nodeTaskParams.stack = &nodeTaskStack;
    Task_construct(&nodeTask, nodeTaskFunction, &nodeTaskParams, NULL);
}

static void nodeTaskFunction(UArg arg0, UArg arg1)
{
    /* Initialize display and try to open both UART and LCD types of display. */
    //Display_Params params;
    //Display_Params_init(&params);
    //params.lineClearMode = DISPLAY_CLEAR_BOTH;

    /* Open both an available LCD display and an UART display.
     * Whether the open call is successful depends on what is present in the
     * Display_config[] array of the board file.
     *
     * Note that for SensorTag evaluation boards combined with the SHARP96x96
     * Watch DevPack, there is a pin conflict with UART such that one must be
     * excluded, and UART is preferred by default. To display on the Watch
     * DevPack, add the precompiler define BOARD_DISPLAY_EXCLUDE_UART.
     */
    //hDisplayLcd = Display_open(Display_Type_LCD, &params);
    //hDisplaySerial = Display_open(Display_Type_UART, &params);

    /* Check if the selected Display type was found and successfully opened */
    /*if (hDisplaySerial)
    {
        Display_print0(hDisplaySerial, 0, 0, "Waiting for SCE ADC reading...");
    }*/

    /* Check if the selected Display type was found and successfully opened */
    /*if (hDisplayLcd)
    {
        Display_print0(hDisplayLcd, 0, 0, "Waiting for ADC...");
    }*/

    /* Open LED pins */
    ledPinHandle = PIN_open(&ledPinState, pinTable);
    if (!ledPinHandle)
    {
        System_abort("Error initializing board 3.3V domain pins\n");
    }


    /* Start the SCE ADC task with 1s sample period and reacting to change in ADC value. */
    SceAdc_init(0x00010000, NODE_ADCTASK_REPORTINTERVAL_FAST, NODE_ADCTASK_CHANGE_MASK);
    SceAdc_registerAdcCallback(adcCallback);
    SceAdc_start();


    /* setup timeout for fast report timeout */
    Clock_setTimeout(fastReportTimeoutClockHandle,
            NODE_ADCTASK_REPORTINTERVAL_FAST_DURIATION_MS * 1000 / Clock_tickPeriod);

    /* start fast report and timeout */
    Clock_start(fastReportTimeoutClockHandle);

#ifdef USE_BIM
    /* setup timer for ble bootload button check */
    Clock_setTimeout(bleBootloadBtnCheckClockHandle,
            NODE_BLE_BOOTLODER_BUTTON_CHECK_TIMER_MS * 1000 / Clock_tickPeriod);
#endif
    //kadir
//    buttonPinHandle = PIN_open(&buttonPinState, buttonPinTable);
//    if (!buttonPinHandle)
//    {
//        System_abort("Error initializing button pins\n");
//    }
//
//    /* Setup callback for button pins */
//    if (PIN_registerIntCb(buttonPinHandle, &buttonCallback) != 0)
//    {
//        System_abort("Error registering button callback function");
//    }

    //Enable power to RF switch to 2.4G antenna
    //PIN_setOutputValue(ledPinHandle, Board_DIO30_SWPWR, 1);

    while (1)
    {
        /* Wait for event */
        uint32_t events = Event_pend(nodeEventHandle, 0, NODE_EVENT_ALL, BIOS_WAIT_FOREVER);

        /* If new ADC value, send this data */
        if (events & NODE_EVENT_NEW_ADC_VALUE) {
            /* Toggle activity LED */
            PIN_setOutputValue(ledPinHandle, NODE_SUB1_ACTIVITY_LED,!PIN_getOutputValue(NODE_SUB1_ACTIVITY_LED));

            /* Send ADC value to concentrator */
            NodeRadioTask_sendAdcData(latestAdcValue);

            /* update display */
            updateLcd();
        }
        /* If new ADC value, send this data */
        if (events & NODE_EVENT_UPDATE_LCD) {
            /* update display */
            updateLcd();
        }

#ifdef USE_BIM
        if (events & NODE_EVENT_FACTORY_RESET) {

            Display_control(hDisplayLcd, DISPLAY_CMD_TRANSPORT_CLOSE, NULL);

            BimFactoryReset_applyFactoryImage();

            /* BIM Factory Reset failed, reenable the LCD */
            Display_control(hDisplayLcd, DISPLAY_CMD_TRANSPORT_OPEN, NULL);

            /* Re-enable interrupts to detect button release. */
            PIN_setConfig(buttonPinHandle, PIN_BM_IRQ, Board_BUTTON0 | PIN_IRQ_NEGEDGE);
            PIN_setConfig(buttonPinHandle, PIN_BM_IRQ, Board_BUTTON1 | PIN_IRQ_NEGEDGE);
        }
#endif
    }

}
static void updateLcd(void)
{
    char advMode[16] = {0};

    /* get node address if not already done */
    if (nodeAddress == 0)
    {
        nodeAddress = nodeRadioTask_getNodeAddr();
    }

    /* print to LCD */
    Display_clear(hDisplayLcd);
    Display_print1(hDisplayLcd, 0, 0, "NodeID: 0x%02x", nodeAddress);
    Display_print1(hDisplayLcd, 1, 0, "ADC: %04d", latestAdcValue);

    /* print to UART clear screen, put cuser to beggining of terminal and print the header */
    Display_print1(hDisplaySerial, 0, 0, "\033[2J \033[0;0HNode ID: 0x%02x", nodeAddress);
    Display_print1(hDisplaySerial, 0, 0, "Node ADC Reading: %04d", latestAdcValue);

    if (advertisementType == Node_AdertiserMsUrl)
    {
         strncpy(advMode, "BLE MS + URL", 12);
    }
    else if (advertisementType == Node_AdertiserMs)
    {
         strncpy(advMode, "BLE MS", 6);
    }
    else if (advertisementType == Node_AdertiserUrl)
    {
         strncpy(advMode, "Eddystone URL", 13);
    }
    else if (advertisementType == Node_AdertiserUid)
    {
         strncpy(advMode, "Eddystone UID", 13);
    }
    else if( advertisementType == Node_AdertiserAtsa )
    {
         strncpy(advMode, "Eddystone ATSA", 14);
    }
    else
    {
         strncpy(advMode, "None", 4);
    }

    frameTypeValue = nodeRadioTask_getNodeFrame();
    dmSensorPacket.adcValue = nodeRadioTask_getNodeAdcValue();
    dmSensorPacket.batt = nodeRadioTask_getNodeBattValue();
    dmSensorPacket.button = nodeRadioTask_getNodeButtonValue();
    dmSensorPacket.time100MiliSec = nodeRadioTask_getNodeTimeValue();
    dmSensorPacket.header.packetType = nodeRadioTask_getPacketTypeValue();
    //atsaPacket.deneme = nodeRadioTask_getDenemeValue();
    /* print to LCD */
    Display_print0(hDisplayLcd, 2, 0, "Adv Mode:");
    Display_print1(hDisplayLcd, 3, 0, "%s", advMode);
    /* print to UART */

    Display_print1(hDisplaySerial, 0, 0, "vBatt: %d", dmSensorPacket.batt);

    // Battery voltage (bit 10:8 - integer, but 7:0 fraction)
    dmSensorPacket.batt = (dmSensorPacket.batt * 125) >> 5; // convert V to mV
    ePrintFrame.vBatt[0] = (dmSensorPacket.batt & 0xFF00) >> 8;
    ePrintFrame.vBatt[1] = dmSensorPacket.batt & 0xFF;

    // Temperature
    ePrintFrame.temp[0] = (dmSensorPacket.adcValue & 0xFF00) >> 8;;
    ePrintFrame.temp[1] = dmSensorPacket.adcValue & 0xFF;

    // running time
    //time100MiliSec = UTC_getClock() * 10; // 1-second resolution for now
    ePrintFrame.secCnt[0] = (dmSensorPacket.time100MiliSec & 0xFF000000) >> 24;
    ePrintFrame.secCnt[1] = (dmSensorPacket.time100MiliSec & 0x00FF0000) >> 16;
    ePrintFrame.secCnt[2] = (dmSensorPacket.time100MiliSec & 0x0000FF00) >> 8;
    ePrintFrame.secCnt[3] = dmSensorPacket.time100MiliSec & 0xFF;

    ePrintFrame.deneme = atsaPacket.time100MiliSec;

    Display_print1(hDisplaySerial, 0, 0, "Packet Type: %d", dmSensorPacket.header.packetType);


    Display_print1(hDisplaySerial, 0, 0, "Advertiser Mode: %s", advMode);
    Display_print1(hDisplaySerial, 0, 0, "Frame Type: %d\n", frameTypeValue);

    Display_print1(hDisplaySerial, 0, 0, "time: %d", dmSensorPacket.time100MiliSec);
    Display_print1(hDisplaySerial, 0, 0, "ADC: %d\n", dmSensorPacket.adcValue);

    Display_print1(hDisplaySerial, 1, 0, "button: %04d", dmSensorPacket.button);        //tugce
    Display_print2(hDisplaySerial, 1, 0, "batt: %d%d", ePrintFrame.vBatt[1], ePrintFrame.vBatt[0]);        //tugce
    Display_print4(hDisplaySerial, 1, 0, "time: %d%d%d5%d", ePrintFrame.secCnt[3], ePrintFrame.secCnt[2], ePrintFrame.secCnt[1], ePrintFrame.secCnt[0]);        //tugce
    Display_print2(hDisplaySerial, 1, 0, "temp: %d%d", ePrintFrame.temp[1], ePrintFrame.temp[0]);        //tugce

    if( advertisementType == Node_AdertiserAtsa )
    {
//        nodeRadioTask_getDenemeValue(dmSensorPacket.deneme);
//        strncpy( atsaPacket.buf, "deneme", 10);
//        Display_print1(hDisplaySerial, 1, 0, "deneme: %d", atsaPacket.deneme);        //tugce
//        Display_print1(hDisplaySerial, 1, 0, "buf: %s", atsaPacket.buf);        //tugce
    }





//    Display_print1(hDisplaySerial, 0, 0, "Advertiser Mode: %s", );      //tugce
}


void adcCallback(uint16_t adcValue)
{
    /* Save latest value */
    latestAdcValue = adcValue;

    /* Post event */
    Event_post(nodeEventHandle, NODE_EVENT_NEW_ADC_VALUE);
}

/*
 *  ======== buttonCallback ========
 *  Pin interrupt Callback function board buttons configured in the pinTable.
 */
void buttonCallback(PIN_Handle handle, PIN_Id pinId)
{
    /* Debounce logic, only toggle if the button is still pushed (low) */
    CPUdelay(8000*50);


    if ((PIN_getInputValue(Board_BUTTON0) == 0) & (PIN_getInputValue(Board_BUTTON1) == 0)) {
#ifdef USE_BIM
        //stop button interupts while checking ble bootload button hold
        PIN_setConfig(buttonPinHandle, PIN_BM_IRQ, Board_BUTTON0 | PIN_IRQ_DIS);
        PIN_setConfig(buttonPinHandle, PIN_BM_IRQ, Board_BUTTON1 | PIN_IRQ_DIS);
        //start BLE bootloader timer to check pins
        Clock_start(bleBootloadBtnCheckClockHandle);
#endif
    }
    else if (PIN_getInputValue(Board_BUTTON0) == 0)
    {
        //start fast report and timeout
        SceAdc_setReportInterval(NODE_ADCTASK_REPORTINTERVAL_FAST, NODE_ADCTASK_CHANGE_MASK);
        Clock_start(fastReportTimeoutClockHandle);
    }
    else if (PIN_getInputValue(Board_BUTTON1) == 0)
    {
        //cycle between url, uid and none
        advertisementType++;
        if (advertisementType == Node_AdertiserTypeEnd)
        {
            advertisementType = Node_AdertiserNone;
        }

        /* update display */
        Event_post(nodeEventHandle, NODE_EVENT_UPDATE_LCD);

        //Set advertiement type
        nodeRadioTask_setAdvertiserType(advertisementType);

        //start fast report and timeout
        SceAdc_setReportInterval(NODE_ADCTASK_REPORTINTERVAL_FAST, NODE_ADCTASK_CHANGE_MASK);
        Clock_start(fastReportTimeoutClockHandle);
    }
}

void fastReportTimeoutCallback(UArg arg0)
{
    //stop fast report
    SceAdc_setReportInterval(NODE_ADCTASK_REPORTINTERVAL_SLOW, NODE_ADCTASK_CHANGE_MASK);
}

#ifdef USE_BIM
void bleBtnCheckTimerCallback(UArg arg0)
{
    static uint32_t btnHoldCnt = 0;

    if ((PIN_getInputValue(Board_BUTTON0) == 0) & (PIN_getInputValue(Board_BUTTON1) == 0))
    {
        if ( (++btnHoldCnt * NODE_BLE_BOOTLODER_BUTTON_CHECK_TIMER_MS) >
            (NODE_BLE_BOOTLODER_BUTTON_CHECK_DURATION_S*1000))
        {
            Event_post(nodeEventHandle, NODE_EVENT_FACTORY_RESET);
            btnHoldCnt = 0;
        }
        else
        {
            Clock_start(bleBootloadBtnCheckClockHandle);
        }
    }
    else
    {
        btnHoldCnt = 0;
        /* Re-enable interrupts to detect button release. */
        PIN_setConfig(buttonPinHandle, PIN_BM_IRQ, Board_BUTTON0 | PIN_IRQ_NEGEDGE);
        PIN_setConfig(buttonPinHandle, PIN_BM_IRQ, Board_BUTTON1 | PIN_IRQ_NEGEDGE);
    }
}
#endif //USE_BIM
