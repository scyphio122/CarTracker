/*
 * driving_analysis.c
 *
 *  Created on: Nov 28, 2017
 *      Author: Konrad Traczyk
 */

#include "lsm6dsm.h"
#include <stdint-gcc.h>

#define WINDOW_SIZE_SAMPLES_CNT          13

static void _CalculateAccDifferentials(uint16_t samplesCount, int16_t* inBuf, int32_t* outBuf, uint16_t samplingFreqHz)
{
    for (uint16_t i=0; i<samplesCount; ++i)
    {
        outBuf[i] = (inBuf[i+1] - inBuf[i])*samplingFreqHz;
    }
}

static void _DoCalculationsForSubWindows(int16_t* inAcc, int32_t* inAccDiff, uint16_t samplesCount)
{
    uint8_t     windowsCount = samplesCount/WINDOW_SIZE_SAMPLES_CNT;
    int16_t*    accMean     = malloc(sizeof(int16_t) * windowsCount);
    int32_t*    accDiffMean = malloc(sizeof(int32_t) * windowsCount);
    int32_t*    accDiffVariance = malloc(sizeof(int32_t) * windowsCount);
    uint8_t     samplesTaken = 0;

    for (uint8_t i=0; i<windowsCount; ++i)
    {
        if (samplesCount > WINDOW_SIZE_SAMPLES_CNT)
        {
            samplesTaken = WINDOW_SIZE_SAMPLES_CNT;
            samplesCount -= samplesTaken;
        }
        else
        {
            samplesTaken = samplesCount;
            samplesCount = 0;
        }
       accMean[i]           =   ImuCalculateMeanValue(&inAcc[i*WINDOW_SIZE_SAMPLES_CNT], samplesTaken, sizeof(int16_t));
       accDiffMean[i]       =   ImuCalculateMeanValue(&inAccDiff[i*WINDOW_SIZE_SAMPLES_CNT], samplesTaken, sizeof(int32_t));
       accDiffVariance[i]   =   ImuCalculateVariance(&inAccDiff[i*WINDOW_SIZE_SAMPLES_CNT], samplesTaken, sizeof(int32_t));

       if (accMean[i] < 0)
           accMean[i] *= -1;

       if (accDiffMean[i] < 0)
           accDiffMean[i] *= -1;
    }

    free(accMean);
    free(accDiffMean);
    free(accDiffVariance);
}
