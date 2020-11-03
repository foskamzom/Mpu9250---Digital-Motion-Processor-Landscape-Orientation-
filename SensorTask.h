/*
 * SensorTask.h
 *
 *  Created on: 21 Oca 2018
 *      Author: User
 */

#ifndef SENSORTASK_H_
#define SENSORTASK_H_

typedef struct
{
    short signed int   xAxisPeakP;
    short signed int   xAxisPeakN;
    short signed int   yAxisPeakP;
    short signed int   yAxisPeakN;
    short signed int   zAxisPeakP;
    short signed int   zAxisPeakN;
} peakValues;

void SensorTask_init(void);
static void sensorTaskFunction(UArg arg0, UArg arg1);
void checkMovePeak ( peakValues* peakValueParam, short signed int* dataArray, int length);
void printSensorData(short signed int* acc, short signed int* gyro, int length);
void printStr (char * printStr);
void initPeakValues ( peakValues* peakValueParam, short signed int* dataArray, int length );
int movingAvarageFilter(short signed int* rawData, int length, int filterStep);
void processPressureSensor (double *pressValueArray);
void steadyPressValueCalculate(void);
void read_press_data (double *tempVal, double *pressVal );

#endif /* SENSORTASK_H_ */
