/*
 * SensorTask.c
 *
 *  Created on: 21 Oca 2018
 *      Author: User
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "inv_mpu.h"
#include "Board.h"
#include <SensorTask.h>
#include <xdc/runtime/System.h>
#include <SensorBmp280.h>
#include <SensorMpu9250_DMP.h>

#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Clock.h>

#include <ti/mw/display/Display.h>
#include <ti/mw/display/DisplayExt.h>
#include <ti/drivers/UART.h>

#include "RadioProtocol.h"

/***** Defines *****/
#define SENSOR_TASK_STACK_SIZE 4096 //1024
#define SENSOR_TASK_PRIORITY  2


/***** Variable declarations *****/
static void sensorTaskFunction(UArg arg0, UArg arg1);
static Task_Params sensorTaskParams;
Task_Struct sensorTask;    /* not static so you can see in ROV */
static uint8_t sensorTaskStack[SENSOR_TASK_STACK_SIZE];
//Event_Struct sensorEvent;  /* not static so you can see in ROV */
//static Event_Handle sensorEventHandle;

/* Clock for the fast report timeout */
Clock_Struct fastReportTimeoutClock;     /* not static so you can see in ROV */
static Clock_Handle fastReportTimeoutClockHandle;
extern void fastReportTimeoutCallback(UArg arg0);
extern bool SensorI2C_open(void);
extern int mpu_read_sensor_data();
static Display_Handle hDisplaySerial;
unsigned char orient;

int indexOfPeakLoop = 0;
double steadyPressureValue = 0;
int pressureFilterStep = 5;
double newHeightValue = 0;
uint8_t pullOver = 0;
uint8_t impact = 0;
uint8_t impactDirect = 0;
uint32_t tempData;

#define DMP
#define ORIENT_PORTRAIT          0
#define ORIENT_LANDSCAPE         1
#define ORIENT_REVERSE_PORTRAIT  2
#define ORIENT_REVERSE_LANDSCAPE 3


#define moveTreshold = 400;

#define NODE_ADCTASK_REPORTINTERVAL_FAST                1
#define NODE_ADCTASK_REPORTINTERVAL_FAST_DURIATION_MS   30000

UART_Handle uart;
UART_Params uartParams;

UInt32 time1, time2;

void SensorTask_init(void)
{


    /* Create clock object which is used for fast report timeout */
        Clock_Params clkParams;
        clkParams.period = 0;
        clkParams.startFlag = FALSE;
        Clock_construct(&fastReportTimeoutClock, fastReportTimeoutCallback, 1, &clkParams);
        fastReportTimeoutClockHandle = Clock_handle(&fastReportTimeoutClock);

    /* Create event used internally for state changes */
    /*Event_Params eventParam;
    Event_Params_init(&eventParam);
    Event_construct(&sensorEvent, &eventParam);
    sensorEventHandle = Event_handle(&sensorEvent);*/

    /* Create the node task */
    Task_Params_init(&sensorTaskParams);
    sensorTaskParams.stackSize = SENSOR_TASK_STACK_SIZE;
    sensorTaskParams.priority = SENSOR_TASK_PRIORITY;
    sensorTaskParams.stack = &sensorTaskStack;
    Task_construct(&sensorTask, sensorTaskFunction, &sensorTaskParams, NULL);
}
MPU9250_DMP MPU9250_Dmp_Instance;
static void sensorTaskFunction(UArg arg0, UArg arg1)
{
    UInt32 lTime1, lTime2;

    /* Firat */

    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.baudRate = 9600;
    uart = UART_open(Board_UART, &uartParams);

    /*Firat*/


    /* Initialize display and try to open both UART and LCD types of display. */
//    Display_Params params;
//    Display_Params_init(&params);
//    params.lineClearMode = DISPLAY_CLEAR_BOTH;

    /* Open both an available LCD display and an UART display.
     * Whether the open call is successful depends on what is present in the
     * Display_config[] array of the board file.
     *
     * Note that for SensorTag evaluation boards combined with the SHARP96x96
     * Watch DevPack, there is a pin conflict with UART such that one must be
     * excluded, and UART is preferred by default. To display on the Watch
     * DevPack, add the precompiler define BOARD_DISPLAY_EXCLUDE_UART.
     */
//    Display_Handle hDisplaySerial;
//    hDisplaySerial = Display_open(Display_Type_UART, &params);

    /* Check if the selected Display type was found and successfully opened */
//    Display_print0(hDisplaySerial, 0, 0, "Welcome...");

    /* setup timeout for fast report timeout */
 //      Clock_setTimeout(fastReportTimeoutClockHandle,
 //              NODE_ADCTASK_REPORTINTERVAL_FAST_DURIATION_MS * 1000 / Clock_tickPeriod);

       /* start fast report and timeout */
//       Clock_start(fastReportTimeoutClockHandle);

//  Firat
#ifdef DMP

       MPU9250_DMP_init(&MPU9250_Dmp_Instance);
       if (MPU9250_DMP_begin(&MPU9250_Dmp_Instance))
       {
           return;
       }
       MPU9250_DMP_dmpBegin(DMP_FEATURE_ANDROID_ORIENT,MAX_DMP_SAMPLE_RATE);
       MPU9250_DMP_dmpSetOrientation(defaultOrientation);
       unsigned char lastOrient = 0;
#else
        if (SensorI2C_open())
        {
            if ( !SensorMpu9250_init() )
            {
               return;
            }
        }
#endif
//  Firat

   /*if ( !SensorBmp280_init() )
           {
               return;
           }*/

   //steadyPressValueCalculate();
//    double pressValueArray[5];
    char input;
    while (1)
    {
        lTime1 =  Clock_getTicks();
//      processPressureSensor(pressValueArray);
#ifndef DMP
        if (!mpu_read_sensor_data() )
        {
            Task_sleep( (12500) / Clock_tickPeriod);
//            sendVehicleState(pullOver, impact, impactDirect, tempData);
//            System_printf("Sensor !!!! \n"); //printStr(" sensor data was not read ");
        }
#else
        if (!MPU9250_DMP_fifoAvailable())
        {
//            if(MPU9250_DMP_updateFifo())
//            {
//                    Task_sleep( (12500) / Clock_tickPeriod);
        //            sendVehicleState(pullOver, impact, impactDirect, tempData);
        //            System_printf("Sensor !!!! \n"); //printStr(" sensor data was not read ");
        }
        else
        {
            if(!MPU9250_DMP_dmpUpdateFifo(&MPU9250_Dmp_Instance))
            {
            orient = MPU9250_DMP_dmpGetOrientation();
                if (orient != lastOrient)
                {
                    switch (orient)

                    {
                        case ORIENT_PORTRAIT:
                            printStr("Portrait");
                            printf("Portrait\n");
                            break;
                        case ORIENT_LANDSCAPE:
                            printStr("Landscape");
                            printf("Landscape\n");
                            break;
                        case ORIENT_REVERSE_PORTRAIT:
                            printStr("Portrait (Reverse)");
                            printf("Portrait (Reverse)\n");
                            break;
                        case ORIENT_REVERSE_LANDSCAPE:
                            printStr("Landscape (Reverse)");
                            printf("Landscape (Reverse)\n");
                            break;
                    }
                    lastOrient = orient;

                }
            }
        }
#endif

//        System_printf("Yes\n");
//        Display_print1(hDisplaySerial, 0, 0,"time1 :%d" ,time1);
//        Display_print1(hDisplaySerial, 0, 0,"time2 :%d" ,time2);
////        read_press_data();
//        lTime2 =  Clock_getTicks();
//        if( (lTime2 - lTime1) < 125000 )
//        {
//            Task_sleep( (lTime2 - lTime1) / Clock_tickPeriod);
//        }

        //Task_sleep((125000) / Clock_tickPeriod);
    }
//       int j;
//    char input[canMessageSize] = {'a','a','a','a','a','a','a','a','a','a'};
//    for ( j = 0; j < canMessageSize; j++ )
//    {
//        input[j] = 'a';
//    }
//    UART_Params_init(&uartParams);
//    uartParams.writeDataMode = UART_DATA_BINARY;
//    uartParams.readDataMode = UART_DATA_BINARY;
//    uartParams.readReturnMode = UART_RETURN_FULL;
//    uartParams.readEcho = UART_ECHO_OFF;
//    uartParams.baudRate = 115200;
//    uart = UART_open(Board_UART, &uartParams);
//
//
//    while (1) {
//        UART_read(uart, &input, 120);
//        sendCanMessage(input, 10);
//        sendCanMessage(input);
//        for ( j = 0; j < 100000; j++);
//        for ( j = 0; j < 120; j++ )
//        {
//            printf("input :%c\n" ,input[j]);
//        }
//        //sendCanMessage(input);
//    }
}

/*void processPressureSensor (double *pressValueArray)
{
    double tempVal;
    double pressVal;
    double newPressVal = 0;
    read_press_data( &tempVal, &pressVal );
    int tempdatavalue;
    tempdatavalue = (int)tempVal;
    tempData = tempdatavalue;
    int i = 0;
    //if ( pressureFilterCheck >= pressureFilterStep )
    {
        for ( i = 0; i < pressureFilterStep - 1; i++ )
        {
            pressValueArray[i] = pressValueArray[i + 1];
            newPressVal += pressValueArray[i];
        }
        pressValueArray[pressureFilterStep] = pressVal;
        newPressVal += pressValueArray[pressureFilterStep];
        newPressVal /= pressureFilterStep;
    }
    else
    {
        pressValueArray[pressureFilterCheck] = pressVal;
        newPressVal = steadyPressureValue;
        pressureFilterCheck++;
    }
    newPressVal = pressVal;
    newHeightValue = ( ( tempVal + 273.15 ) / 0.0065) * ( 1 - pow( ( newPressVal / steadyPressureValue ),0.190263 ) );
    return;
    //Display_print1(hDisplaySerial, 0, 0,"new Height :%d;", (int)(newHeightValue*100));
    //Display_print1(hDisplaySerial, 0, 0,"new press: %d", (int)newPressVal);
}

*/
//void steadyPressValueCalculate(void)
//{
//    double tempVal;
//    double pressVal = 0;
//    double pressValArray[20];
//    int i = 0;
//    for ( i = 0; i < 10; i++ )
//    {
//        read_press_data(&tempVal, &pressValArray[0] );
//    }
//    Task_sleep((500000) / Clock_tickPeriod);
//    for ( i = 0; i < 20; i++ )
//    {
//        read_press_data(&tempVal, &pressValArray[i] );
//        Display_print1(hDisplaySerial, 0, 0,"press :%d" , (int)(pressValArray[i]));
//        pressVal += pressValArray[i];
//        Task_sleep((150000) / Clock_tickPeriod);
//    }
//    steadyPressureValue  = pressVal/20;
//    return;
//}

//void read_press_data (double *tempVal, double *pressVal )
//{
//    uint32_t uncomp_temp_data;
//    uint32_t uncomp_press_data;
//    SensorBmp280_read(&uncomp_temp_data, &uncomp_press_data);
//    SensorBmp280_convert(&uncomp_press_data, &uncomp_temp_data, pressVal, tempVal);
//
//    //Display_print1(hDisplaySerial, 0, 0,"temp :%d" , (int)tempVal);
//     //Display_print1(hDisplaySerial, 0, 0,"press :%d" , (int)(*pressVal));
//}

int processMovSensor(short signed int* acc, short signed int* gyro, int length)
{
    movingAvarageFilter(acc, length, 5);
    movingAvarageFilter(gyro, length, 5);

    peakValues accFirstPeakValue = {-10, -10, 10, 10, 960, 960};
    checkMovePeak( &accFirstPeakValue, acc, length );
    if ( ( ( accFirstPeakValue.zAxisPeakP - accFirstPeakValue.zAxisPeakN) > 50 ) && newHeightValue > 0.8 )
    {
        pullOver = 1;
        //System_printf(" Arac cekildi." );
        printf(" Arac cekildi.\n");
        return 1;
    }
    else if ( ( ( accFirstPeakValue.xAxisPeakP - accFirstPeakValue.xAxisPeakN ) > 1000 ) && ( ( accFirstPeakValue.xAxisPeakP - accFirstPeakValue.xAxisPeakN ) > ( accFirstPeakValue.yAxisPeakP - accFirstPeakValue.yAxisPeakN ) ) )
    {
        impact = 1;
        impactDirect = 0;
        //System_printf(" Arac yandan darbe aldi." );
        printf(" Arac yandan darbe aldi.\n" );
        return 1;
    }
    else if ( ( ( accFirstPeakValue.yAxisPeakP - accFirstPeakValue.yAxisPeakN ) > 1000 ) && ( ( accFirstPeakValue.yAxisPeakP - accFirstPeakValue.yAxisPeakN ) > ( accFirstPeakValue.xAxisPeakP - accFirstPeakValue.xAxisPeakN ) ) )
    {
        impact = 1;
        impactDirect = 1;
        //System_printf(" Arac onden darbe aldi." );
        printf(" Arac onden darbe aldi.\n" );
        return 1;
    }

//    time2 =  Clock_getTicks();
//    Display_print3(hDisplaySerial, 0, 0, "Peak P: X = %d; Y = %d; Z = %d;", accPeakValue.xAxisPeakP, accPeakValue.yAxisPeakP, accPeakValue.zAxisPeakP );
//    Display_print3(hDisplaySerial, 0, 0, "Peak N: X = %d; Y = %d; Z = %d;", accPeakValue.xAxisPeakN, accPeakValue.yAxisPeakN, accPeakValue.zAxisPeakN );
//    printSensorData(acc, gyro, length);
//    time1 =  Clock_getTicks();
//    ///////////////////////////////////sendVehicleState(pullOver, impact, impactDirect);
//    time2 =  Clock_getTicks();
//    Display_print1(hDisplaySerial, 0, 0, "Ble time: %d;", time2 - time1);
//    Display_print0(hDisplaySerial, 0, 0, "Ble time\n");
//    printf("Ble time: \n");
    return 0;
}

int movingAvarageFilter(short signed int* rawData, int length, int filterStep)
{
    signed int i = 0;
    int k = 0;
    short signed int step = 0;

    if( !length)
    {
        return 1;
    }
    i = ( length - 1 ) * 3 ;
    while ( i > 0 )
    {
        k = i - (filterStep - 1) * 3;
        if ( k < 0)
        {
            k = 0;
        }
        step = 0;
        while ( k < i  )
        {
            rawData[i + 0] +=  rawData[k + 0];
            rawData[i + 1] +=  rawData[k + 1];
            rawData[i + 2] +=  rawData[k + 2];
            step += 1;
            k += 3;
        }
        rawData[i + 0] = rawData[i + 0]/(step + 1);
        rawData[i + 1] = rawData[i + 1]/(step + 1);
        rawData[i + 2] = rawData[i + 2]/(step + 1);
        i -= 3;
    }
    return 0;
}

void initPeakValues ( peakValues* peakValueParam, short signed int* dataArray, int length  )
{
    int i = 0;
    short signed int xAxisVal = 0;
    short signed int yAxisVal = 0;
    short signed int zAxisVal = 0;
    for ( i = 0; i < length; i++ )
    {
        xAxisVal += dataArray[i*3 + 0];
        yAxisVal += dataArray[i*3 + 1];
        zAxisVal += dataArray[i*3 + 2];
    }
    peakValueParam->xAxisPeakP = xAxisVal /length;
    peakValueParam->xAxisPeakN = xAxisVal /length;
    peakValueParam->yAxisPeakP = yAxisVal /length;
    peakValueParam->yAxisPeakN = yAxisVal /length;
    peakValueParam->zAxisPeakP = zAxisVal /length;
    peakValueParam->zAxisPeakN = zAxisVal /length;
}

void checkMovePeak ( peakValues* peakValueParam, short signed int* dataArray, int length)
{
    int i = 0;
    while( i < length*3 )
    {
        if( dataArray[i + 0] <  peakValueParam->xAxisPeakN )
        {
            peakValueParam->xAxisPeakN = dataArray[i + 0];
        }
        else if ( dataArray[i + 0] >  peakValueParam->xAxisPeakP  )
        {
            peakValueParam->xAxisPeakP = dataArray[i + 0];
        }
        if( dataArray[i + 1] <  peakValueParam->yAxisPeakN )
        {
            peakValueParam->yAxisPeakN = dataArray[i + 1];
        }
        else if ( dataArray[i + 1] >  peakValueParam->yAxisPeakP  )
        {
            peakValueParam->yAxisPeakP = dataArray[i + 1];
        }
        if( dataArray[i + 2] <  peakValueParam->zAxisPeakN )
        {
            peakValueParam->zAxisPeakN = dataArray[i + 2];
        }
        else if ( dataArray[i + 2] >  peakValueParam->zAxisPeakP  )
        {
            peakValueParam->zAxisPeakP = dataArray[i + 2];
        }

        i += 3;
    }
}

void printSensorData(short signed int* acc, short signed int* gyro, int length)
{
    time1 =  Clock_getTicks();
    int i = 0;
    Display_print1(hDisplaySerial, 0, 0, "length: %d;", length);
    for( i = 0; i < length; i++)
    {
        Display_print3(hDisplaySerial, 0, 0, "acc: %d; %d; %d;", acc[0 + i*3], acc[1 + i*3], acc[2 + i*3]);
        Display_print3(hDisplaySerial, 0, 0, "gyro: %d; %d; %d;", gyro[0 + i*3], gyro[1 + i*3], gyro[2 + i*3]);
    }
    time2 =  Clock_getTicks();
}

void printStr (char * printStr1)
{
    UART_write(uart, printStr1, sizeof(printStr1));
}

