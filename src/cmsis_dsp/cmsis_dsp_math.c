/*
 * cmsis_dsp_statistics.c
 *
 *  Created on: Nov 11, 2017
 *      Author: Konrad Traczyk
 */




/* ----------------------------------------------------------------------
* Copyright (C) 2010-2014 ARM Limited. All rights reserved.
*
* $Date:        19. March 2015
* $Revision:    V.1.4.5
*
* Project:      CMSIS DSP Library
* Title:        arm_mean_q7.c
*
* Description:  Mean value of a Q7 vector.
*
* Target Processor: Cortex-M4/Cortex-M3/Cortex-M0
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*   - Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   - Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in
*     the documentation and/or other materials provided with the
*     distribution.
*   - Neither the name of ARM LIMITED nor the names of its contributors
*     may be used to endorse or promote products derived from this
*     software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
* -------------------------------------------------------------------- */

#include "arm_math.h"

/**
 * @ingroup groupStats
 */

/**
 * @addtogroup mean
 * @{
 */

/**
 * @brief Mean value of a Q7 vector.
 * @param[in]       *pSrc points to the input vector
 * @param[in]       blockSize length of the input vector
 * @param[out]      *pResult mean value returned here
 * @return none.
 *
 * @details
 * <b>Scaling and Overflow Behavior:</b>
 * \par
 * The function is implemented using a 32-bit internal accumulator.
 * The input is represented in 1.7 format and is accumulated in a 32-bit
 * accumulator in 25.7 format.
 * There is no risk of internal overflow with this approach, and the
 * full precision of intermediate result is preserved.
 * Finally, the accumulator is truncated to yield a result of 1.7 format.
 *
 */


void arm_mean_q7(
  q7_t * pSrc,
  uint32_t blockSize,
  q7_t * pResult)
{
  q31_t sum = 0;                                 /* Temporary result storage */
  uint32_t blkCnt;                               /* loop counter */

#ifndef ARM_MATH_CM0_FAMILY

  /* Run the below code for Cortex-M4 and Cortex-M3 */
  q31_t in;

  /*loop Unrolling */
  blkCnt = blockSize >> 2u;

  /* First part of the processing with loop unrolling.  Compute 4 outputs at a time.
   ** a second loop below computes the remaining 1 to 3 samples. */
  while(blkCnt > 0u)
  {
    /* C = (A[0] + A[1] + A[2] + ... + A[blockSize-1]) */
    in = *__SIMD32(pSrc)++;

    sum += ((in << 24) >> 24);
    sum += ((in << 16) >> 24);
    sum += ((in << 8) >> 24);
    sum += (in >> 24);

    /* Decrement the loop counter */
    blkCnt--;
  }

  /* If the blockSize is not a multiple of 4, compute any remaining output samples here.
   ** No loop unrolling is used. */
  blkCnt = blockSize % 0x4u;

#else

  /* Run the below code for Cortex-M0 */

  /* Loop over blockSize number of values */
  blkCnt = blockSize;

#endif /* #ifndef ARM_MATH_CM0_FAMILY */

  while(blkCnt > 0u)
  {
    /* C = (A[0] + A[1] + A[2] + ... + A[blockSize-1]) */
    sum += *pSrc++;

    /* Decrement the loop counter */
    blkCnt--;
  }

  /* C = (A[0] + A[1] + A[2] + ... + A[blockSize-1]) / blockSize  */
  /* Store the result to the destination */
  *pResult = (q7_t) (sum / (int32_t) blockSize);
}


void arm_mean_q15(
  q15_t * pSrc,
  uint32_t blockSize,
  q15_t * pResult)
{
  q31_t sum = 0;                                 /* Temporary result storage */
  uint32_t blkCnt;                               /* loop counter */

#ifndef ARM_MATH_CM0_FAMILY

  /* Run the below code for Cortex-M4 and Cortex-M3 */
  q31_t in;

  /*loop Unrolling */
  blkCnt = blockSize >> 2u;

  /* First part of the processing with loop unrolling.  Compute 4 outputs at a time.
   ** a second loop below computes the remaining 1 to 3 samples. */
  while(blkCnt > 0u)
  {
    /* C = (A[0] + A[1] + A[2] + ... + A[blockSize-1]) */
    in = *__SIMD32(pSrc)++;
    sum += ((in << 16) >> 16);
    sum += (in >> 16);
    in = *__SIMD32(pSrc)++;
    sum += ((in << 16) >> 16);
    sum += (in >> 16);

    /* Decrement the loop counter */
    blkCnt--;
  }

  /* If the blockSize is not a multiple of 4, compute any remaining output samples here.
   ** No loop unrolling is used. */
  blkCnt = blockSize % 0x4u;

#else

  /* Run the below code for Cortex-M0 */

  /* Loop over blockSize number of values */
  blkCnt = blockSize;

#endif /* #ifndef ARM_MATH_CM0_FAMILY */

  while(blkCnt > 0u)
  {
    /* C = (A[0] + A[1] + A[2] + ... + A[blockSize-1]) */
    sum += *pSrc++;

    /* Decrement the loop counter */
    blkCnt--;
  }

  /* C = (A[0] + A[1] + A[2] + ... + A[blockSize-1]) / blockSize  */
  /* Store the result to the destination */
  *pResult = (q15_t) (sum / (q31_t)blockSize);
}

/**
 * @brief Mean value of a Q31 vector.
 * @param[in]       *pSrc points to the input vector
 * @param[in]       blockSize length of the input vector
 * @param[out]      *pResult mean value returned here
 * @return none.
 *
 * @details
 * <b>Scaling and Overflow Behavior:</b>
 *\par
 * The function is implemented using a 64-bit internal accumulator.
 * The input is represented in 1.31 format and is accumulated in a 64-bit
 * accumulator in 33.31 format.
 * There is no risk of internal overflow with this approach, and the
 * full precision of intermediate result is preserved.
 * Finally, the accumulator is truncated to yield a result of 1.31 format.
 *
 */


void arm_mean_q31(
  q31_t * pSrc,
  uint32_t blockSize,
  q31_t * pResult)
{
  q63_t sum = 0;                                 /* Temporary result storage */
  uint32_t blkCnt;                               /* loop counter */

#ifndef ARM_MATH_CM0_FAMILY

  /* Run the below code for Cortex-M4 and Cortex-M3 */
  q31_t in1, in2, in3, in4;

  /*loop Unrolling */
  blkCnt = blockSize >> 2u;

  /* First part of the processing with loop unrolling.  Compute 4 outputs at a time.
   ** a second loop below computes the remaining 1 to 3 samples. */
  while(blkCnt > 0u)
  {
    /* C = (A[0] + A[1] + A[2] + ... + A[blockSize-1]) */
    in1 = *pSrc++;
    in2 = *pSrc++;
    in3 = *pSrc++;
    in4 = *pSrc++;

    sum += in1;
    sum += in2;
    sum += in3;
    sum += in4;

    /* Decrement the loop counter */
    blkCnt--;
  }

  /* If the blockSize is not a multiple of 4, compute any remaining output samples here.
   ** No loop unrolling is used. */
  blkCnt = blockSize % 0x4u;

#else

  /* Run the below code for Cortex-M0 */

  /* Loop over blockSize number of values */
  blkCnt = blockSize;

#endif /* #ifndef ARM_MATH_CM0_FAMILY */

  while(blkCnt > 0u)
  {
    /* C = (A[0] + A[1] + A[2] + ... + A[blockSize-1]) */
    sum += *pSrc++;

    /* Decrement the loop counter */
    blkCnt--;
  }

  /* C = (A[0] + A[1] + A[2] + ... + A[blockSize-1]) / blockSize  */
  /* Store the result to the destination */
  *pResult = (q31_t) (sum / (int32_t) blockSize);
}


arm_status arm_sqrt_q31(
  q31_t in,
  q31_t * pOut)
{
  q31_t number, temp1, bits_val1, var1, signBits1, half;
  float32_t temp_float1;
  union
  {
      q31_t fracval;
      float32_t floatval;
  } tempconv;

  number = in;

  /* If the input is a positive number then compute the signBits. */
  if(number > 0)
  {
    signBits1 = __CLZ(number) - 1;

    /* Shift by the number of signBits1 */
    if((signBits1 % 2) == 0)
    {
      number = number << signBits1;
    }
    else
    {
      number = number << (signBits1 - 1);
    }

    /* Calculate half value of the number */
    half = number >> 1;
    /* Store the number for later use */
    temp1 = number;

    /*Convert to float */
    temp_float1 = number * 4.6566128731e-010f;
    /*Store as integer */
    tempconv.floatval = temp_float1;
    bits_val1 = tempconv.fracval;
    /* Subtract the shifted value from the magic number to give intial guess */
    bits_val1 = 0x5f3759df - (bits_val1 >> 1);  /* gives initial guess */
    /* Store as float */
    tempconv.fracval = bits_val1;
    temp_float1 = tempconv.floatval;
    /* Convert to integer format */
    var1 = (q31_t) (temp_float1 * 1073741824);

    /* 1st iteration */
    var1 = ((q31_t) ((q63_t) var1 * (0x30000000 -
                                     ((q31_t)
                                      ((((q31_t)
                                         (((q63_t) var1 * var1) >> 31)) *
                                        (q63_t) half) >> 31))) >> 31)) << 2;
    /* 2nd iteration */
    var1 = ((q31_t) ((q63_t) var1 * (0x30000000 -
                                     ((q31_t)
                                      ((((q31_t)
                                         (((q63_t) var1 * var1) >> 31)) *
                                        (q63_t) half) >> 31))) >> 31)) << 2;
    /* 3rd iteration */
    var1 = ((q31_t) ((q63_t) var1 * (0x30000000 -
                                     ((q31_t)
                                      ((((q31_t)
                                         (((q63_t) var1 * var1) >> 31)) *
                                        (q63_t) half) >> 31))) >> 31)) << 2;

    /* Multiply the inverse square root with the original value */
    var1 = ((q31_t) (((q63_t) temp1 * var1) >> 31)) << 1;

    /* Shift the output down accordingly */
    if((signBits1 % 2) == 0)
    {
      var1 = var1 >> (signBits1 / 2);
    }
    else
    {
      var1 = var1 >> ((signBits1 - 1) / 2);
    }
    *pOut = var1;

    return (ARM_MATH_SUCCESS);
  }
  /* If the number is a negative number then store zero as its square root value */
  else
  {
    *pOut = 0;
    return (ARM_MATH_ARGUMENT_ERROR);
  }
}


/**
   * @brief  Q15 square root function.
   * @param[in]   in     input value.  The range of the input value is [0 +1) or 0x0000 to 0x7FFF.
   * @param[out]  *pOut  square root of input value.
   * @return The function returns ARM_MATH_SUCCESS if the input value is positive
   * and ARM_MATH_ARGUMENT_ERROR if the input is negative.  For
   * negative inputs, the function returns *pOut = 0.
   */

arm_status arm_sqrt_q15(
  q15_t in,
  q15_t * pOut)
{
  q15_t number, temp1, var1, signBits1, half;
  q31_t bits_val1;
  float32_t temp_float1;
  union
  {
    q31_t fracval;
    float32_t floatval;
  } tempconv;

  number = in;

  /* If the input is a positive number then compute the signBits. */
  if(number > 0)
  {
    signBits1 = __CLZ(number) - 17;

    /* Shift by the number of signBits1 */
    if((signBits1 % 2) == 0)
    {
      number = number << signBits1;
    }
    else
    {
      number = number << (signBits1 - 1);
    }

    /* Calculate half value of the number */
    half = number >> 1;
    /* Store the number for later use */
    temp1 = number;

    /* Convert to float */
    temp_float1 = number * 3.051757812500000e-005f;
    /*Store as integer */
    tempconv.floatval = temp_float1;
    bits_val1 = tempconv.fracval;
    /* Subtract the shifted value from the magic number to give intial guess */
    bits_val1 = 0x5f3759df - (bits_val1 >> 1);  /* gives initial guess */
    /* Store as float */
    tempconv.fracval = bits_val1;
    temp_float1 = tempconv.floatval;
    /* Convert to integer format */
    var1 = (q31_t) (temp_float1 * 16384);

    /* 1st iteration */
    var1 = ((q15_t) ((q31_t) var1 * (0x3000 -
                                     ((q15_t)
                                      ((((q15_t)
                                         (((q31_t) var1 * var1) >> 15)) *
                                        (q31_t) half) >> 15))) >> 15)) << 2;
    /* 2nd iteration */
    var1 = ((q15_t) ((q31_t) var1 * (0x3000 -
                                     ((q15_t)
                                      ((((q15_t)
                                         (((q31_t) var1 * var1) >> 15)) *
                                        (q31_t) half) >> 15))) >> 15)) << 2;
    /* 3rd iteration */
    var1 = ((q15_t) ((q31_t) var1 * (0x3000 -
                                     ((q15_t)
                                      ((((q15_t)
                                         (((q31_t) var1 * var1) >> 15)) *
                                        (q31_t) half) >> 15))) >> 15)) << 2;

    /* Multiply the inverse square root with the original value */
    var1 = ((q15_t) (((q31_t) temp1 * var1) >> 15)) << 1;

    /* Shift the output down accordingly */
    if((signBits1 % 2) == 0)
    {
      var1 = var1 >> (signBits1 / 2);
    }
    else
    {
      var1 = var1 >> ((signBits1 - 1) / 2);
    }
    *pOut = var1;

    return (ARM_MATH_SUCCESS);
  }
  /* If the number is a negative number then store zero as its square root value */
  else
  {
    *pOut = 0;
    return (ARM_MATH_ARGUMENT_ERROR);
  }
}



/* ----------------------------------------------------------------------
* Copyright (C) 2010-2014 ARM Limited. All rights reserved.
*
* $Date:        19. March 2015
* $Revision:    V.1.4.5
*
* Project:      CMSIS DSP Library
* Title:        arm_var_q15.c
*
* Description:  Variance of an array of Q15 type.
*
* Target Processor: Cortex-M4/Cortex-M3/Cortex-M0
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*   - Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   - Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in
*     the documentation and/or other materials provided with the
*     distribution.
*   - Neither the name of ARM LIMITED nor the names of its contributors
*     may be used to endorse or promote products derived from this
*     software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
* -------------------------------------------------------------------- */

#include "arm_math.h"

/**
 * @ingroup groupStats
 */

/**
 * @addtogroup variance
 * @{
 */

/**
 * @brief Variance of the elements of a Q15 vector.
 * @param[in]       *pSrc points to the input vector
 * @param[in]       blockSize length of the input vector
 * @param[out]      *pResult variance value returned here
 * @return none.
 *
 * @details
 * <b>Scaling and Overflow Behavior:</b>
 *
 * \par
 * The function is implemented using a 64-bit internal accumulator.
 * The input is represented in 1.15 format.
 * Intermediate multiplication yields a 2.30 format, and this
 * result is added without saturation to a 64-bit accumulator in 34.30 format.
 * With 33 guard bits in the accumulator, there is no risk of overflow, and the
 * full precision of the intermediate multiplication is preserved.
 * Finally, the 34.30 result is truncated to 34.15 format by discarding the lower
 * 15 bits, and then saturated to yield a result in 1.15 format.
 *
 */


void arm_var_q15(
  q15_t * pSrc,
  uint32_t blockSize,
  q15_t * pResult)
{

  q31_t sum = 0;                                 /* Accumulator */
  q31_t meanOfSquares, squareOfMean;             /* square of mean and mean of square */
  uint32_t blkCnt;                               /* loop counter */
  q63_t sumOfSquares = 0;                        /* Accumulator */

#ifndef ARM_MATH_CM0_FAMILY

  /* Run the below code for Cortex-M4 and Cortex-M3 */

  q31_t in;                                      /* input value */
  q15_t in1;                                     /* input value */

    if(blockSize == 1)
    {
        *pResult = 0;
        return;
    }

  /*loop Unrolling */
  blkCnt = blockSize >> 2u;

  /* First part of the processing with loop unrolling.  Compute 4 outputs at a time.
   ** a second loop below computes the remaining 1 to 3 samples. */
  while(blkCnt > 0u)
  {
    /* C = (A[0] * A[0] + A[1] * A[1] + ... + A[blockSize-1] * A[blockSize-1])  */
    /* Compute Sum of squares of the input samples
     * and then store the result in a temporary variable, sum. */
    in = *__SIMD32(pSrc)++;
    sum += ((in << 16) >> 16);
    sum += (in >> 16);
    sumOfSquares = __SMLALD(in, in, sumOfSquares);
    in = *__SIMD32(pSrc)++;
    sum += ((in << 16) >> 16);
    sum += (in >> 16);
    sumOfSquares = __SMLALD(in, in, sumOfSquares);

    /* Decrement the loop counter */
    blkCnt--;
  }

  /* If the blockSize is not a multiple of 4, compute any remaining output samples here.
   ** No loop unrolling is used. */
  blkCnt = blockSize % 0x4u;

  while(blkCnt > 0u)
  {
    /* C = (A[0] * A[0] + A[1] * A[1] + ... + A[blockSize-1] * A[blockSize-1]) */
    /* Compute Sum of squares of the input samples
     * and then store the result in a temporary variable, sum. */
    in1 = *pSrc++;
    sumOfSquares = __SMLALD(in1, in1, sumOfSquares);
    sum += in1;

    /* Decrement the loop counter */
    blkCnt--;
  }

  /* Compute Mean of squares of the input samples
   * and then store the result in a temporary variable, meanOfSquares. */
  meanOfSquares = (q31_t) (sumOfSquares / (q63_t)(blockSize - 1));

  /* Compute square of mean */
  squareOfMean = (q31_t)((q63_t)sum * sum / (q63_t)(blockSize * (blockSize - 1)));

  /* mean of the squares minus the square of the mean. */
  *pResult = (meanOfSquares - squareOfMean) >> 15;

#else

  /* Run the below code for Cortex-M0 */
  q15_t in;                                      /* input value */

    if(blockSize == 1)
    {
        *pResult = 0;
        return;
    }

  /* Loop over blockSize number of values */
  blkCnt = blockSize;

  while(blkCnt > 0u)
  {
    /* C = (A[0] * A[0] + A[1] * A[1] + ... + A[blockSize-1] * A[blockSize-1]) */
    /* Compute Sum of squares of the input samples
     * and then store the result in a temporary variable, sumOfSquares. */
    in = *pSrc++;
    sumOfSquares += (in * in);

    /* C = (A[0] + A[1] + A[2] + ... + A[blockSize-1]) */
    /* Compute sum of all input values and then store the result in a temporary variable, sum. */
    sum += in;

    /* Decrement the loop counter */
    blkCnt--;
  }

  /* Compute Mean of squares of the input samples
   * and then store the result in a temporary variable, meanOfSquares. */
  meanOfSquares = (q31_t) (sumOfSquares / (q63_t)(blockSize - 1));

  /* Compute square of mean */
  squareOfMean = (q31_t)((q63_t)sum * sum / (q63_t)(blockSize * (blockSize - 1)));

  /* mean of the squares minus the square of the mean. */
  *pResult = (meanOfSquares - squareOfMean) >> 15;

#endif /* #ifndef ARM_MATH_CM0_FAMILY */

}


/* ----------------------------------------------------------------------
* Copyright (C) 2010-2014 ARM Limited. All rights reserved.
*
* $Date:        19. March 2015
* $Revision:    V.1.4.5
*
* Project:      CMSIS DSP Library
* Title:        arm_var_q31.c
*
* Description:  Variance of an array of Q31 type.
*
* Target Processor: Cortex-M4/Cortex-M3/Cortex-M0
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*   - Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   - Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in
*     the documentation and/or other materials provided with the
*     distribution.
*   - Neither the name of ARM LIMITED nor the names of its contributors
*     may be used to endorse or promote products derived from this
*     software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
* -------------------------------------------------------------------- */

#include "arm_math.h"

/**
 * @ingroup groupStats
 */

/**
 * @addtogroup variance
 * @{
 */

/**
 * @brief Variance of the elements of a Q31 vector.
 * @param[in]       *pSrc points to the input vector
 * @param[in]       blockSize length of the input vector
 * @param[out]      *pResult variance value returned here
 * @return none.
 *
 * @details
 * <b>Scaling and Overflow Behavior:</b>
 *
 *\par
 * The function is implemented using an internal 64-bit accumulator.
 * The input is represented in 1.31 format, which is then downshifted by 8 bits
 * which yields 1.23, and intermediate multiplication yields a 2.46 format.
 * The accumulator maintains full precision of the intermediate multiplication results,
 * but provides only a 16 guard bits.
 * There is no saturation on intermediate additions.
 * If the accumulator overflows it wraps around and distorts the result.
 * In order to avoid overflows completely the input signal must be scaled down by
 * log2(blockSize)-8 bits, as a total of blockSize additions are performed internally.
 * After division, internal variables should be Q18.46
 * Finally, the 18.46 accumulator is right shifted by 15 bits to yield a 1.31 format value.
 *
 */


void arm_var_q31(
  q31_t * pSrc,
  uint32_t blockSize,
  q31_t * pResult)
{
  q63_t sum = 0;                                 /* Accumulator */
  q63_t meanOfSquares, squareOfMean;             /* square of mean and mean of square */
  q31_t in;                                      /* input value */
  uint32_t blkCnt;                               /* loop counter */
  q63_t sumOfSquares = 0;                        /* Accumulator */

    if(blockSize == 1)
    {
        *pResult = 0;
        return;
    }

#ifndef ARM_MATH_CM0_FAMILY

  /* Run the below code for Cortex-M4 and Cortex-M3 */

  /*loop Unrolling */
  blkCnt = blockSize >> 2u;

  /* First part of the processing with loop unrolling.  Compute 4 outputs at a time.
   ** a second loop below computes the remaining 1 to 3 samples. */
  while(blkCnt > 0u)
  {
    /* C = (A[0] * A[0] + A[1] * A[1] + ... + A[blockSize-1] * A[blockSize-1])  */
    /* Compute Sum of squares of the input samples
     * and then store the result in a temporary variable, sum. */
    in = *pSrc++ >> 8;
    sum += in;
    sumOfSquares += ((q63_t) (in) * (in));
    in = *pSrc++ >> 8;
    sum += in;
    sumOfSquares += ((q63_t) (in) * (in));
    in = *pSrc++ >> 8;
    sum += in;
    sumOfSquares += ((q63_t) (in) * (in));
    in = *pSrc++ >> 8;
    sum += in;
    sumOfSquares += ((q63_t) (in) * (in));

    /* Decrement the loop counter */
    blkCnt--;
  }

  /* If the blockSize is not a multiple of 4, compute any remaining output samples here.
   ** No loop unrolling is used. */
  blkCnt = blockSize % 0x4u;

  while(blkCnt > 0u)
  {
    /* C = (A[0] * A[0] + A[1] * A[1] + ... + A[blockSize-1] * A[blockSize-1]) */
    /* Compute Sum of squares of the input samples
     * and then store the result in a temporary variable, sum. */
    in = *pSrc++ >> 8;
    sum += in;
    sumOfSquares += ((q63_t) (in) * (in));

    /* Decrement the loop counter */
    blkCnt--;
  }

  /* Compute Mean of squares of the input samples
   * and then store the result in a temporary variable, meanOfSquares. */
  meanOfSquares = sumOfSquares / (q63_t)(blockSize - 1);

#else

  /* Run the below code for Cortex-M0 */

  /* Loop over blockSize number of values */
  blkCnt = blockSize;

  while(blkCnt > 0u)
  {
    /* C = (A[0] * A[0] + A[1] * A[1] + ... + A[blockSize-1] * A[blockSize-1]) */
    /* Compute Sum of squares of the input samples
     * and then store the result in a temporary variable, sumOfSquares. */
    in = *pSrc++ >> 8;
    sumOfSquares += ((q63_t) (in) * (in));

    /* C = (A[0] + A[1] + A[2] + ... + A[blockSize-1]) */
    /* Compute sum of all input values and then store the result in a temporary variable, sum. */
    sum += in;

    /* Decrement the loop counter */
    blkCnt--;
  }

  /* Compute Mean of squares of the input samples
   * and then store the result in a temporary variable, meanOfSquares. */
  meanOfSquares = sumOfSquares / (q63_t)(blockSize - 1);

#endif /* #ifndef ARM_MATH_CM0_FAMILY */

  /* Compute square of mean */
  squareOfMean = sum * sum / (q63_t)(blockSize * (blockSize - 1u));


  /* Compute standard deviation and then store the result to the destination */
  *pResult = (meanOfSquares - squareOfMean) >> 15;

}


/* ----------------------------------------------------------------------
* Copyright (C) 2010-2014 ARM Limited. All rights reserved.
*
* $Date:        12. March 2014
* $Revision:    V1.4.4
*
* Project:      CMSIS DSP Library
* Title:        arm_power_q31.c
*
* Description:  Sum of the squares of the elements of a Q31 vector.
*
* Target Processor: Cortex-M4/Cortex-M3/Cortex-M0
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*   - Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   - Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in
*     the documentation and/or other materials provided with the
*     distribution.
*   - Neither the name of ARM LIMITED nor the names of its contributors
*     may be used to endorse or promote products derived from this
*     software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
* -------------------------------------------------------------------- */

#include "arm_math.h"

/**
 * @ingroup groupStats
 */

/**
 * @addtogroup power
 * @{
 */

/**
 * @brief Sum of the squares of the elements of a Q31 vector.
 * @param[in]       *pSrc points to the input vector
 * @param[in]       blockSize length of the input vector
 * @param[out]      *pResult sum of the squares value returned here
 * @return none.
 *
 * @details
 * <b>Scaling and Overflow Behavior:</b>
 *
 * \par
 * The function is implemented using a 64-bit internal accumulator.
 * The input is represented in 1.31 format.
 * Intermediate multiplication yields a 2.62 format, and this
 * result is truncated to 2.48 format by discarding the lower 14 bits.
 * The 2.48 result is then added without saturation to a 64-bit accumulator in 16.48 format.
 * With 15 guard bits in the accumulator, there is no risk of overflow, and the
 * full precision of the intermediate multiplication is preserved.
 * Finally, the return result is in 16.48 format.
 *
 */

void arm_power_q31(
  q31_t * pSrc,
  uint32_t blockSize,
  q63_t * pResult)
{
  q63_t sum = 0;                                 /* Temporary result storage */
  q31_t in;
  uint32_t blkCnt;                               /* loop counter */


#ifndef ARM_MATH_CM0_FAMILY

  /* Run the below code for Cortex-M4 and Cortex-M3 */

  /*loop Unrolling */
  blkCnt = blockSize >> 2u;

  /* First part of the processing with loop unrolling.  Compute 4 outputs at a time.
   ** a second loop below computes the remaining 1 to 3 samples. */
  while(blkCnt > 0u)
  {
    /* C = A[0] * A[0] + A[1] * A[1] + A[2] * A[2] + ... + A[blockSize-1] * A[blockSize-1] */
    /* Compute Power then shift intermediate results by 14 bits to maintain 16.48 format and then store the result in a temporary variable sum, providing 15 guard bits. */
    in = *pSrc++;
    sum += ((q63_t) in * in) >> 14u;

    in = *pSrc++;
    sum += ((q63_t) in * in) >> 14u;

    in = *pSrc++;
    sum += ((q63_t) in * in) >> 14u;

    in = *pSrc++;
    sum += ((q63_t) in * in) >> 14u;

    /* Decrement the loop counter */
    blkCnt--;
  }

  /* If the blockSize is not a multiple of 4, compute any remaining output samples here.
   ** No loop unrolling is used. */
  blkCnt = blockSize % 0x4u;

#else

  /* Run the below code for Cortex-M0 */

  /* Loop over blockSize number of values */
  blkCnt = blockSize;

#endif /* #ifndef ARM_MATH_CM0_FAMILY */

  while(blkCnt > 0u)
  {
    /* C = A[0] * A[0] + A[1] * A[1] + A[2] * A[2] + ... + A[blockSize-1] * A[blockSize-1] */
    /* Compute Power and then store the result in a temporary variable, sum. */
    in = *pSrc++;
    sum += ((q63_t) in * in) >> 14u;

    /* Decrement the loop counter */
    blkCnt--;
  }

  /* Store the results in 16.48 format  */
  *pResult = sum;
}


/**
 * @} end of mean group
 */
