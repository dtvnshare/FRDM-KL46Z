/*
 * Copyright (c) 2014, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

///////////////////////////////////////////////////////////////////////////////
// Includes
///////////////////////////////////////////////////////////////////////////////

// SDK Included Files
#include "adc_hw_trigger.h"
#include "fsl_lptmr_driver.h"

///////////////////////////////////////////////////////////////////////////////
// Variables
///////////////////////////////////////////////////////////////////////////////

extern SIM_Type * gSimBase[];
static lptmr_state_t gLPTMRState;

///////////////////////////////////////////////////////////////////////////////
// Code
///////////////////////////////////////////////////////////////////////////////

/*!
 * @Brief enable the trigger source of LPTimer
 */
void init_trigger_source(uint32_t adcInstance)
{
    uint32_t freqUs;

    lptmr_user_config_t lptmrUserConfig =
    {
        .timerMode = kLptmrTimerModeTimeCounter,
        .freeRunningEnable = false,
        .prescalerEnable = false, // bypass perscaler
#if (CLOCK_INIT_CONFIG == CLOCK_VLPR)
        // use MCGIRCCLK, 4M or 32KHz
        .prescalerClockSource = kClockLptmrSrcMcgIrClk,
#else
#if defined(FRDM_KL02Z) || defined(FRDM_KL03Z) || defined(TWR_KL43Z48M) || defined(FRDM_KL27Z)|| defined(FRDM_KL43Z)
        // Use LPO clock 1KHz
        .prescalerClockSource = kClockLptmrSrcLpoClk,
#else
        .prescalerClockSource = kClockLptmrSrcMcgIrClk,
#endif
#endif
        .isInterruptEnabled = false
    };

    // Init LPTimer driver
    LPTMR_DRV_Init(0, &gLPTMRState, &lptmrUserConfig);

    // Set the LPTimer period
    freqUs = 1000000U/(INPUT_SIGNAL_FREQ*NR_SAMPLES)*2;
    LPTMR_DRV_SetTimerPeriodUs(0, freqUs);

    // Start the LPTimer
    LPTMR_DRV_Start(0);

    // Configure SIM for ADC hw trigger source selection
    SIM_HAL_SetAdcAlternativeTriggerCmd(gSimBase[0], adcInstance, true);
    SIM_HAL_SetAdcPreTriggerMode(gSimBase[0], adcInstance, kSimAdcPretrgselA);
    SIM_HAL_SetAdcTriggerMode(gSimBase[0], adcInstance, kSimAdcTrgSelLptimer);
}

/*!
 * @Brief disable the trigger source
 */
void deinit_trigger_source(uint32_t adcInstance)
{
    LPTMR_DRV_Stop(0);
    LPTMR_DRV_Deinit(0);
}
