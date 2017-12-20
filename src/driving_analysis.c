/*
 * driving_analysis.c
 *
 *  Created on: Nov 28, 2017
 *      Author: Konrad Traczyk
 */


#include "driving_analysis.h"
#include "lsm6dsm.h"
#include <malloc.h>
#include <stdint-gcc.h>
#include <math.h>
#include "parsing_utils.h"

#define WINDOW_SIZE_SAMPLES_CNT          13
#define VARIANCE_SI_TO_TICKS        (float)2789342.6271

#define MEASURE_FREQ    52
#define WINDOW_WIDTH    (MEASURE_FREQ/2)

#define X_ACCEL_THRESHOLD           1670.0
#define Y_ACCEL_THRESHOLD_FWD       4175.0
#define Y_ACCEL_THRESHOLD_BWD       -6681.0
#define X_VARIANCE_THRESHOLD        (float)((float)1000.0 * VARIANCE_SI_TO_TICKS)
#define Y_VARIANCE_THRESHOLD        (float)((float)1100.0 * VARIANCE_SI_TO_TICKS)

#define X_ACC_STABLE_OFFSET         0.271679270817683
#define Y_ACC_STABLE_OFFSET         0.061365195848284
#define X_VAR_STABLE_OFFSET         0.374161921779365
#define Y_VAR_STABLE_OFFSET         0.24351096782251

static int32_t    zrywX[1024];
static int32_t    zrywY[1024];

static uint16_t _CalculateSecondDiffs(int16_t* inBuf, int32_t* outBuf, uint16_t bufSize, float step)
{
    int32_t diff = 0;
    for (int i=0; i<bufSize-1; ++i)
    {
        diff = inBuf[i+1] - inBuf[i];
        outBuf[i] = (int32_t)diff/step;
    }
}

static int64_t CalculateVariance(void* buf, uint16_t bufSize, uint8_t wordSize)
{
    int32_t mean = ImuCalculateMeanValue(buf, bufSize, 4);
    int64_t variance = 0;
    int32_t diff = 0;
//    int64_t power = 0;

    uint8_t* b = (uint8_t*)buf;

    for(int i=0; i<bufSize; ++i)
    {
        memcpy(&diff, &b[i*wordSize], wordSize);
        diff -= mean;
        variance += _pow(diff, 2);
    }

    variance /= bufSize;
    return variance;
}

float CalculateAssessment(int16_t* x, int16_t* y, int16_t* z, uint16_t samplesNumber)
{
    uint16_t windowsCount = 0;

    if ((samplesNumber % WINDOW_WIDTH) == 0)
    {
        windowsCount = samplesNumber / WINDOW_WIDTH;
    }
    else
    {
        windowsCount = (samplesNumber / WINDOW_WIDTH) + 1;
    }

    float*    meanAccXNorm     = malloc(windowsCount * sizeof(float));
    float*    meanAccYNorm     = malloc(windowsCount * sizeof(float));

    _CalculateSecondDiffs(x, zrywX,  samplesNumber-1, 1.0/MEASURE_FREQ);
    _CalculateSecondDiffs(y, zrywY,  samplesNumber-1, 1.0/MEASURE_FREQ);

    float*    varianceZrywXNorm     = malloc(windowsCount * sizeof(float));
    float*    varianceZrywYNorm     = malloc(windowsCount * sizeof(float));

    int16_t meanX = 0;
    int16_t meanY = 0;

    for (uint16_t i=0; i<windowsCount; ++i)
    {
        uint16_t width = WINDOW_WIDTH;

        if ((samplesNumber - i*WINDOW_WIDTH) % WINDOW_WIDTH != 0)
        {
            if (i == (windowsCount - 1))
            {
                width = samplesNumber % WINDOW_WIDTH;
            }
        }
        meanX = (int16_t)ImuCalculateMeanValue(x + i*WINDOW_WIDTH, width, 2);
        meanAccXNorm[i]   = meanX / X_ACCEL_THRESHOLD;

        if (meanAccXNorm[i] < 0)
        {
            meanAccXNorm[i] *= -1;
        }

        meanY = (int16_t)ImuCalculateMeanValue(y + i*WINDOW_WIDTH, width, 2);
        meanAccYNorm[i]   = meanY / Y_ACCEL_THRESHOLD_FWD;

        if (meanAccYNorm[i] < 0)
        {
            meanAccYNorm[i]   *= -1;
        }

//        varianceZrywXNorm[i] = ImuCalculateVariance(&zrywX[i*WINDOW_WIDTH], width, sizeof(uint32_t)) / X_VARIANCE_THRESHOLD;
//        varianceZrywYNorm[i] = ImuCalculateVariance(&zrywY[i*WINDOW_WIDTH], width, sizeof(uint32_t)) / Y_VARIANCE_THRESHOLD;

        varianceZrywXNorm[i] = CalculateVariance(&zrywX[i*WINDOW_WIDTH], width, sizeof(uint32_t)) / X_VARIANCE_THRESHOLD;
        varianceZrywYNorm[i] = CalculateVariance(&zrywY[i*WINDOW_WIDTH], width, sizeof(uint32_t)) / Y_VARIANCE_THRESHOLD;

//        printf("Params: %10d %10d %10d %10lu %10lu\n", i, meanAccX[i], meanAccY[i], varianceZrywX[i], varianceZrywY[i]);
    }

    float _meanXNorm = 0;
    float _meanYNorm = 0;
    float _varXNorm = 0;
    float _varYNorm = 0;

    for (int i=0; i<windowsCount; ++i)
    {
        _meanXNorm += meanAccXNorm[i];
        _meanYNorm += meanAccYNorm[i];
        _varXNorm += varianceZrywXNorm[i];
        _varYNorm += varianceZrywYNorm[i];
    }

    _meanXNorm /= windowsCount;
    _meanYNorm /= windowsCount;
    _varXNorm /= windowsCount;
    _varYNorm /= windowsCount;

    _meanXNorm -= X_ACC_STABLE_OFFSET;
    _meanYNorm -= Y_ACC_STABLE_OFFSET;
    _varXNorm -= X_VAR_STABLE_OFFSET;
    _varYNorm -= Y_VAR_STABLE_OFFSET;

    float combinedMarkX = (_meanXNorm + 2*_varXNorm)/3;
    float combinedMarkY = (_meanYNorm + 2*_varYNorm)/3;

    float finalAssessment = (combinedMarkX + combinedMarkY)/2;

    if (finalAssessment < 0)
    {
        finalAssessment = 0;
    }

    if (finalAssessment > 1)
    {
        finalAssessment = 1;
    }

    free(meanAccXNorm);
    free(meanAccYNorm);

    free(varianceZrywXNorm);
    free(varianceZrywYNorm);


    return finalAssessment;
}
