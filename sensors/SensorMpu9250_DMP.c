#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "data_builder.h"
#include "MPU9250_RegisterMap.h"

#include "Board.h"
#include "SensorMpu9250.h"
#include "SensorMpu9250_DMP.h"
#include "SensorOpt3001.h" // For reset of I2C bus
#include "SensorUtil.h"
#include "SensorI2C.h"
#include "inv_mpu.h"

#define PI 22/7

static unsigned char mpu9250_orientation;
static unsigned char tap_count;
static unsigned char tap_direction;
static bool _tap_available;
static void MPU9250_DMP_orient_cb(unsigned char orient);
static void MPU9250_DMP_tap_cb(unsigned char direction, unsigned char count);

const signed char defaultOrientation[9] = {
    1, 0, 0,
    0, 1, 0,
    0, 0, 1
};

void MPU9250_DMP_init (MPU9250_DMP* MPU9250_Dmp_Instance)
{
    MPU9250_Dmp_Instance->_mSense = 6.665f; // Constant - 4915 / 32760
    MPU9250_Dmp_Instance->_aSense = 0.0f;   // Updated after accel FSR is set
    MPU9250_Dmp_Instance->_gSense = 0.0f;   // Updated after gyro FSR is set
}

inv_error_t MPU9250_DMP_begin(MPU9250_DMP* MPU9250_Dmp_Instance)
{
    inv_error_t result;
    struct int_param_s int_param;
    SensorI2C_open();

    result = mpu_init(&int_param);

    if (result)
        return result;

    mpu_set_bypass(1); // Place all slaves (including compass) on primary bus

    result = MPU9250_DMP_setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

    MPU9250_Dmp_Instance->_gSense = MPU9250_DMP_getGyroSens();
    MPU9250_Dmp_Instance->_aSense = MPU9250_DMP_getAccelSens();

    return result;
}

/*inv_error_t MPU9250_DMP::enableInterrupt(unsigned char enable)
{
    return set_int_enable(enable);
}*/

inv_error_t MPU9250_DMP_setIntLevel(unsigned char active_low)
{
    return mpu_set_int_level(active_low);
}

inv_error_t MPU9250_DMP_setIntLatched(unsigned char enable)
{
    return mpu_set_int_latched(enable);
}

short MPU9250_DMP_getIntStatus(void)
{
    short status;
    if (mpu_get_int_status(&status) == INV_SUCCESS)
    {
        return status;
    }
    return 0;
}

// Accelerometer Low-Power Mode. Rate options:
// 1.25 (1), 2.5 (2), 5, 10, 20, 40,
// 80, 160, 320, or 640 Hz
// Disables compass and gyro
inv_error_t MPU9250_DMP_lowPowerAccel(unsigned short rate)
{
    return mpu_lp_accel_mode(rate);
}

inv_error_t MPU9250_DMP_setGyroFSR(unsigned short fsr, MPU9250_DMP* MPU9250_Dmp_Instance)
{
    inv_error_t err;
    err = mpu_set_gyro_fsr(fsr);
    if (err == INV_SUCCESS)
    {
        MPU9250_Dmp_Instance->_gSense = MPU9250_DMP_getGyroSens();
    }
    return err;
}

inv_error_t MPU9250_DMP_setAccelFSR(unsigned char fsr, MPU9250_DMP* MPU9250_Dmp_Instance)
{
    inv_error_t err;
    err = mpu_set_accel_fsr(fsr);
    if (err == INV_SUCCESS)
    {
        MPU9250_Dmp_Instance->_aSense = MPU9250_DMP_getAccelSens();
    }
    return err;
}

unsigned short MPU9250_DMP_getGyroFSR(void)
{
    unsigned short tmp;
    if (mpu_get_gyro_fsr(&tmp) == INV_SUCCESS)
    {
        return tmp;
    }
    return 0;
}

unsigned char getAccelFSR(void)
{
    unsigned char tmp;
    if (mpu_get_accel_fsr(&tmp) == INV_SUCCESS)
    {
        return tmp;
    }
    return 0;
}

unsigned short getMagFSR(void)
{
    unsigned short tmp;
    if (mpu_get_compass_fsr(&tmp) == INV_SUCCESS)
    {
        return tmp;
    }
    return 0;
}

inv_error_t setLPF(unsigned short lpf)
{
    return mpu_set_lpf(lpf);
}

unsigned short getLPF(void)
{
    unsigned short tmp;
    if (mpu_get_lpf(&tmp) == INV_SUCCESS)
    {
        return tmp;
    }
    return 0;
}

inv_error_t setSampleRate(unsigned short rate)
{
    return mpu_set_sample_rate(rate);
}

unsigned short getSampleRate(void)
{
    unsigned short tmp;
    if (mpu_get_sample_rate(&tmp) == INV_SUCCESS)
    {
        return tmp;
    }
    return 0;
}

inv_error_t setCompassSampleRate(unsigned short rate)
{
    return mpu_set_compass_sample_rate(rate);
}

unsigned short getCompassSampleRate(void)
{
    unsigned short tmp;
    if (mpu_get_compass_sample_rate(&tmp) == INV_SUCCESS)
    {
        return tmp;
    }

    return 0;
}

float MPU9250_DMP_getGyroSens(void)
{
    float sens;
    if (mpu_get_gyro_sens(&sens) == INV_SUCCESS)
    {
        return sens;
    }
    return 0;
}

unsigned short MPU9250_DMP_getAccelSens(void)
{
    unsigned short sens;
    if (mpu_get_accel_sens(&sens) == INV_SUCCESS)
    {
        return sens;
    }
    return 0;
}

float MPU9250_DMP_getMagSens(void)
{
    return 0.15; // Static, 4915/32760
}

unsigned char MPU9250_DMP_getFifoConfig(void)
{
    unsigned char sensors;
    if (mpu_get_fifo_config(&sensors) == INV_SUCCESS)
    {
        return sensors;
    }
    return 0;
}

inv_error_t MPU9250_DMP_configureFifo(unsigned char sensors)
{
    return mpu_configure_fifo(sensors);
}

inv_error_t MPU9250_DMP_resetFifo(void)
{
    return mpu_reset_fifo();
}

/* 0 - Unsuccessful */

unsigned short MPU9250_DMP_fifoAvailable(void)
{
    unsigned char fifoH, fifoL;
    if (mpu_read_reg(MPU9250_FIFO_COUNTH, &fifoH) != INV_SUCCESS)
        return 0;
    if (mpu_read_reg(MPU9250_FIFO_COUNTL, &fifoL) != INV_SUCCESS)
        return 0;

    return (fifoH << 8 ) | fifoL;
}

inv_error_t MPU9250_DMP_updateFifo(MPU9250_DMP* MPU9250_Dmp_Instance)
{
    short gyro[3], accel[3];
    unsigned long timestamp;
    unsigned char sensors, more;

    if (mpu_read_fifo(gyro, accel, &timestamp, &sensors, &more) != INV_SUCCESS)
        return INV_ERROR;

    if (sensors & INV_XYZ_ACCEL)
    {
        MPU9250_Dmp_Instance->ax = accel[X_AXIS];
        MPU9250_Dmp_Instance->ay = accel[Y_AXIS];
        MPU9250_Dmp_Instance->az = accel[Z_AXIS];
    }
    if (sensors & INV_X_GYRO)
        MPU9250_Dmp_Instance->gx = gyro[X_AXIS];
    if (sensors & INV_Y_GYRO)
        MPU9250_Dmp_Instance->gy = gyro[Y_AXIS];
    if (sensors & INV_Z_GYRO)
        MPU9250_Dmp_Instance->gz = gyro[Z_AXIS];

    MPU9250_Dmp_Instance->time = timestamp;

    return INV_SUCCESS;
}

inv_error_t MPU9250_DMP_setSensors(unsigned char sensors)
{
    return mpu_set_sensors(sensors);
}

bool MPU9250_DMP_dataReady()
{
    unsigned char intStatusReg;

    if (mpu_read_reg(MPU9250_INT_STATUS, &intStatusReg) == INV_SUCCESS)
    {
        return (intStatusReg & (1<<INT_STATUS_RAW_DATA_RDY_INT));
    }
    return false;
}

inv_error_t MPU9250_DMP_update(unsigned char sensors, MPU9250_DMP* MPU9250_Dmp_Instance)
{
    inv_error_t aErr = INV_SUCCESS;
    inv_error_t gErr = INV_SUCCESS;
    inv_error_t mErr = INV_SUCCESS;
    inv_error_t tErr = INV_SUCCESS;

    if (sensors & UPDATE_ACCEL)
        aErr = MPU9250_DMP_updateAccel(MPU9250_Dmp_Instance);
    if (sensors & UPDATE_GYRO)
        gErr = MPU9250_DMP_updateGyro(MPU9250_Dmp_Instance);
    if (sensors & UPDATE_COMPASS)
        mErr = MPU9250_DMP_updateCompass(MPU9250_Dmp_Instance);
    if (sensors & UPDATE_TEMP)
        tErr = MPU9250_DMP_updateTemperature(MPU9250_Dmp_Instance);

    return aErr | gErr | mErr | tErr;
}

int MPU9250_DMP_updateAccel(MPU9250_DMP* MPU9250_Dmp_Instance)
{
    short data[3];

    if (mpu_get_accel_reg(data, &(MPU9250_Dmp_Instance->time)))
    {
        return INV_ERROR;
    }
    MPU9250_Dmp_Instance->ax = data[X_AXIS];
    MPU9250_Dmp_Instance->ay = data[Y_AXIS];
    MPU9250_Dmp_Instance->az = data[Z_AXIS];
    return INV_SUCCESS;
}

int MPU9250_DMP_updateGyro(MPU9250_DMP* MPU9250_Dmp_Instance)
{
    short data[3];

    if (mpu_get_gyro_reg(data, &(MPU9250_Dmp_Instance->time)))
    {
        return INV_ERROR;
    }
    MPU9250_Dmp_Instance->gx = data[X_AXIS];
    MPU9250_Dmp_Instance->gy = data[Y_AXIS];
    MPU9250_Dmp_Instance->gz = data[Z_AXIS];
    return INV_SUCCESS;
}

int MPU9250_DMP_updateCompass(MPU9250_DMP* MPU9250_Dmp_Instance)
{
    short data[3];

    if (mpu_get_compass_reg(data, & (MPU9250_Dmp_Instance->time)))
    {
        return INV_ERROR;
    }
    MPU9250_Dmp_Instance->mx = data[X_AXIS];
    MPU9250_Dmp_Instance->my = data[Y_AXIS];
    MPU9250_Dmp_Instance->mz = data[Z_AXIS];
    return INV_SUCCESS;
}

inv_error_t MPU9250_DMP_updateTemperature(MPU9250_DMP* MPU9250_Dmp_Instance)
{
    return mpu_get_temperature(&(MPU9250_Dmp_Instance->temperature), &(MPU9250_Dmp_Instance->time));
}

int MPU9250_DMP_selfTest(unsigned char debug)
{
    long gyro[3], accel[3];
    return mpu_run_self_test(gyro, accel);
}

inv_error_t MPU9250_DMP_dmpBegin(unsigned short features, unsigned short fifoRate)
{
    unsigned short feat = features;
    unsigned short rate = fifoRate;

    if (MPU9250_DMP_dmpLoad() != INV_SUCCESS)
        return INV_ERROR;

    // 3-axis and 6-axis LP quat are mutually exclusive.
    // If both are selected, default to 3-axis
    if (feat & DMP_FEATURE_LP_QUAT)
    {
        feat &= ~(DMP_FEATURE_6X_LP_QUAT);
        dmp_enable_lp_quat(1);
    }
    else if (feat & DMP_FEATURE_6X_LP_QUAT)
        dmp_enable_6x_lp_quat(1);

    if (feat & DMP_FEATURE_GYRO_CAL)
        dmp_enable_gyro_cal(1);

    if (MPU9250_DMP_dmpEnableFeatures(feat) != INV_SUCCESS)
        return INV_ERROR;

//  rate = constrain(rate, 1, 200);
    if (MPU9250_DMP_dmpSetFifoRate(rate) != INV_SUCCESS)
        return INV_ERROR;

    return mpu_set_dmp_state(1);
}

inv_error_t MPU9250_DMP_dmpLoad(void)
{
    return dmp_load_motion_driver_firmware();
}

unsigned short MPU9250_DMP_dmpGetFifoRate(void)
{
    unsigned short rate;
    if (dmp_get_fifo_rate(&rate) == INV_SUCCESS)
        return rate;

    return 0;
}

inv_error_t MPU9250_DMP_dmpSetFifoRate(unsigned short rate)
{
    if (rate > MAX_DMP_SAMPLE_RATE) rate = MAX_DMP_SAMPLE_RATE;
    return dmp_set_fifo_rate(rate);
}

inv_error_t MPU9250_DMP_dmpUpdateFifo(MPU9250_DMP* MPU9250_Dmp_Instance)
{
    short gyro[3];
    short accel[3];
    long quat[4];
    unsigned long timestamp;
    short sensors;
    unsigned char more;

    if (dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more)!= INV_SUCCESS)
    {
       return INV_ERROR;
    }

//    if (sensors & INV_XYZ_ACCEL)
//    {
        MPU9250_Dmp_Instance->ax = accel[X_AXIS];
        MPU9250_Dmp_Instance->ay = accel[Y_AXIS];
        MPU9250_Dmp_Instance->az = accel[Z_AXIS];
//    }
//    if (sensors & INV_X_GYRO)
        MPU9250_Dmp_Instance->gx = gyro[X_AXIS];
//    if (sensors & INV_Y_GYRO)
        MPU9250_Dmp_Instance->gy = gyro[Y_AXIS];
//    if (sensors & INV_Z_GYRO)
        MPU9250_Dmp_Instance->gz = gyro[Z_AXIS];
//    if (sensors & INV_WXYZ_QUAT)
//    {
        MPU9250_Dmp_Instance->qw = quat[0];
        MPU9250_Dmp_Instance->qx = quat[1];
        MPU9250_Dmp_Instance->qy = quat[2];
        MPU9250_Dmp_Instance->qz = quat[3];
//    }

    MPU9250_Dmp_Instance->time = timestamp;

    return INV_SUCCESS;
}

inv_error_t MPU9250_DMP_dmpEnableFeatures(unsigned short mask)
{
    unsigned short enMask = 0;
    enMask |= mask;
    // Combat known issue where fifo sample rate is incorrect
    // unless tap is enabled in the DMP.
    enMask |= DMP_FEATURE_TAP;
    return dmp_enable_feature(enMask);
}

unsigned short MPU9250_DMP_dmpGetEnabledFeatures(void)
{
    unsigned short mask;
    if (dmp_get_enabled_features(&mask) == INV_SUCCESS)
        return mask;
    return 0;
}

inv_error_t MPU9250_DMP_dmpSetTap(
        unsigned short xThresh, unsigned short yThresh, unsigned short zThresh,
        unsigned char taps, unsigned short tapTime, unsigned short tapMulti)
{
    unsigned char axes = 0;
    if (xThresh > 0)
    {
        axes |= TAP_X;
//      xThresh = constrain(xThresh, 1, 1600);
        if (dmp_set_tap_thresh(1<<X_AXIS, xThresh) != INV_SUCCESS)
            return INV_ERROR;
    }
    if (yThresh > 0)
    {
        axes |= TAP_Y;
//      yThresh = constrain(yThresh, 1, 1600);
        if (dmp_set_tap_thresh(1<<Y_AXIS, yThresh) != INV_SUCCESS)
            return INV_ERROR;
    }
    if (zThresh > 0)
    {
        axes |= TAP_Z;
//      zThresh = constrain(zThresh, 1, 1600);
        if (dmp_set_tap_thresh(1<<Z_AXIS, zThresh) != INV_SUCCESS)
            return INV_ERROR;
    }
    if (dmp_set_tap_axes(axes) != INV_SUCCESS)
        return INV_ERROR;
    if (dmp_set_tap_count(taps) != INV_SUCCESS)
        return INV_ERROR;
    if (dmp_set_tap_time(tapTime) != INV_SUCCESS)
        return INV_ERROR;
    if (dmp_set_tap_time_multi(tapMulti) != INV_SUCCESS)
        return INV_ERROR;

    dmp_register_tap_cb(MPU9250_DMP_tap_cb);

    return INV_SUCCESS;
}

unsigned char MPU9250_DMP_getTapDir(void)
{
    _tap_available = false;
    return tap_direction;
}

unsigned char MPU9250_DMP_getTapCount(void)
{
    _tap_available = false;
    return tap_count;
}

bool MPU9250_DMP_tapAvailable(void)
{
    return _tap_available;
}

inv_error_t MPU9250_DMP_dmpSetOrientation(const signed char * orientationMatrix)
{
    unsigned short scalar;
    scalar = MPU9250_DMP_orientation_row_2_scale(orientationMatrix);
    scalar |= MPU9250_DMP_orientation_row_2_scale(orientationMatrix + 3) << 3;
    scalar |= MPU9250_DMP_orientation_row_2_scale(orientationMatrix + 6) << 6;

    dmp_register_android_orient_cb(MPU9250_DMP_orient_cb);

    return dmp_set_orientation(scalar);
}

unsigned char MPU9250_DMP_dmpGetOrientation(void)
{
    return mpu9250_orientation;
}

inv_error_t MPU9250_DMP_dmpEnable3Quat(void)
{
    unsigned short dmpFeatures;

    // 3-axis and 6-axis quat are mutually exclusive
    dmpFeatures = MPU9250_DMP_dmpGetEnabledFeatures();
    dmpFeatures &= ~(DMP_FEATURE_6X_LP_QUAT);
    dmpFeatures |= DMP_FEATURE_LP_QUAT;

    if (MPU9250_DMP_dmpEnableFeatures(dmpFeatures) != INV_SUCCESS)
        return INV_ERROR;

    return dmp_enable_lp_quat(1);
}

unsigned long MPU9250_DMP_dmpGetPedometerSteps(void)
{
    unsigned long steps;
    if (dmp_get_pedometer_step_count(&steps) == INV_SUCCESS)
    {
        return steps;
    }
    return 0;
}

inv_error_t MPU9250_DMP_dmpSetPedometerSteps(unsigned long steps)
{
    return dmp_set_pedometer_step_count(steps);
}

unsigned long MPU9250_DMP_dmpGetPedometerTime(void)
{
    unsigned long walkTime;
    if (dmp_get_pedometer_walk_time(&walkTime) == INV_SUCCESS)
    {
        return walkTime;
    }
    return 0;
}

inv_error_t MPU9250_DMP_dmpSetPedometerTime(unsigned long time)
{
    return dmp_set_pedometer_walk_time(time);
}

float MPU9250_DMP_calcAccel(int axis, MPU9250_DMP* MPU9250_Dmp_Instance)
{
    return (float) axis / (float) MPU9250_Dmp_Instance->_aSense;
}

float MPU9250_DMP_calcGyro(int axis, MPU9250_DMP* MPU9250_Dmp_Instance)
{
    return (float) axis / (float) MPU9250_Dmp_Instance->_gSense;
}

float MPU9250_DMP_calcMag(int axis, MPU9250_DMP* MPU9250_Dmp_Instance)
{
    return (float) axis / (float) MPU9250_Dmp_Instance->_mSense;
}

float MPU9250_DMP_calcQuat(long axis)
{
    return MPU9250_DMP_qToFloat(axis, 30);
}

float MPU9250_DMP_qToFloat(long number, unsigned char q)
{
    long mask = 0;
    int i;
    for (i=0; i<q; i++)
    {
        mask |= (1<<i);
    }
    return (number >> q) + ((number & mask) / (float) (2<<(q-1)));
}

void MPU9250_DMP_computeEulerAngles(bool degrees, MPU9250_DMP* MPU9250_Dmp_Instance)
{
    float dqw = MPU9250_DMP_qToFloat(MPU9250_Dmp_Instance->qw, 30);
    float dqx = MPU9250_DMP_qToFloat(MPU9250_Dmp_Instance->qx, 30);
    float dqy = MPU9250_DMP_qToFloat(MPU9250_Dmp_Instance->qy, 30);
    float dqz = MPU9250_DMP_qToFloat(MPU9250_Dmp_Instance->qz, 30);

    float ysqr = dqy * dqy;
    float t0 = -2.0f * (ysqr + dqz * dqz) + 1.0f;
    float t1 = +2.0f * (dqx * dqy - dqw * dqz);
    float t2 = -2.0f * (dqx * dqz + dqw * dqy);
    float t3 = +2.0f * (dqy * dqz - dqw * dqx);
    float t4 = -2.0f * (dqx * dqx + ysqr) + 1.0f;

    // Keep t2 within range of asin (-1, 1)
    t2 = t2 > 1.0f ? 1.0f : t2;
    t2 = t2 < -1.0f ? -1.0f : t2;

    MPU9250_Dmp_Instance->pitch = asin(t2) * 2;
    MPU9250_Dmp_Instance->roll = atan2(t3, t4);
    MPU9250_Dmp_Instance->yaw = atan2(t1, t0);

    if (degrees)
    {
        MPU9250_Dmp_Instance->pitch *= (180.0 / PI);
        MPU9250_Dmp_Instance->roll *= (180.0 / PI);
        MPU9250_Dmp_Instance->yaw *= (180.0 / PI);
        if (MPU9250_Dmp_Instance->pitch < 0) MPU9250_Dmp_Instance->pitch = 360.0 + MPU9250_Dmp_Instance->pitch;
        if (MPU9250_Dmp_Instance->roll < 0) MPU9250_Dmp_Instance->roll = 360.0 + MPU9250_Dmp_Instance->roll;
        if (MPU9250_Dmp_Instance->yaw < 0) MPU9250_Dmp_Instance->yaw = 360.0 + MPU9250_Dmp_Instance->yaw;
    }
}

float MPU9250_DMP_computeCompassHeading(MPU9250_DMP* MPU9250_Dmp_Instance)
{
    if (MPU9250_Dmp_Instance->my == 0)
        MPU9250_Dmp_Instance->heading = (MPU9250_Dmp_Instance->mx < 0) ? PI : 0;
    else
        MPU9250_Dmp_Instance->heading = atan2(MPU9250_Dmp_Instance->mx, MPU9250_Dmp_Instance->my);

    if (MPU9250_Dmp_Instance->heading > PI) MPU9250_Dmp_Instance->heading -= (2 * PI);
    else if (MPU9250_Dmp_Instance->heading < -PI) MPU9250_Dmp_Instance->heading += (2 * PI);
    else if (MPU9250_Dmp_Instance->heading < 0) MPU9250_Dmp_Instance->heading += 2 * PI;

    MPU9250_Dmp_Instance->heading*= 180.0 / PI;

    return MPU9250_Dmp_Instance->heading;
}

unsigned short MPU9250_DMP_orientation_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

static void MPU9250_DMP_tap_cb(unsigned char direction, unsigned char count)
{
    _tap_available = true;
    tap_count = count;
    tap_direction = direction;
}

static void MPU9250_DMP_orient_cb(unsigned char orient)
{
    mpu9250_orientation = orient;
}

