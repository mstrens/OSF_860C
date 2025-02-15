/*******************************************************************************
 * File Name: cycfg_system.c
 *
 * Description:
 * System configuration
 * This file was automatically generated and should not be modified.
 * Configurator Backend 3.30.0
 * device-db 4.20.0.7450
 * mtb-xmclib-cat3 4.4.0.4715
 *
 *******************************************************************************
 * Copyright 2025 Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ******************************************************************************/

#include "cycfg_system.h"

#define CLOCK_PCLK_SEL XMC_SCU_CLOCK_PCLKSRC_DOUBLE_MCLK
#define CLOCK_DCO1_SYNC_ENABLED 0
#define CLOCK_FDIV_SEL XMC_SCU_CLOCK_DCLKSRC_DCO1
#define CLOCK_FDIV_I 1U
#define CLOCK_FDIV_F 0U
#define CLOCK_RTCCLK_SEL XMC_SCU_CLOCK_RTCCLKSRC_DCO2

void SystemCoreClockSetup(void)
{
    const XMC_SCU_CLOCK_CONFIG_t clock_config =
  {
    .pclk_src = CLOCK_PCLK_SEL,
    .rtc_src = CLOCK_RTCCLK_SEL,
    .fdiv = CLOCK_FDIV_F,
    .idiv = CLOCK_FDIV_I,
 #if (UC_SERIES == XMC14)
    .dclk_src = CLOCK_FDIV_SEL,
 #if defined(CLOCK_OSCHP_ENABLED)
    .oschp_mode = CLOCK_OSCHP_MODE,
 #else
    .oschp_mode = XMC_SCU_CLOCK_OSCHP_MODE_DISABLED,
 #endif
 #if defined(CLOCK_OSCLP_ENABLED)
    .osclp_mode = XMC_SCU_CLOCK_OSCLP_MODE_OSC,
 #else
    .osclp_mode = XMC_SCU_CLOCK_OSCLP_MODE_DISABLED,
 #endif
 #endif
  };
  
  XMC_SCU_CLOCK_Init(&clock_config);
  
 #if defined(CLOCK_DCO1_SYNC_ENABLED) && CLOCK_DCO1_SYNC_ENABLED
  /* DCO1 Calibration using external oscillator */
  XMC_SCU_CLOCK_EnableDCO1ExtRefCalibration(CLOCK_DCO1_SYNC_SEL, CLOCK_DCO1_SYNC_PRESCALER, CLOCK_DCO1_SYNC_PRELOAD);
  while (XMC_SCU_CLOCK_IsDCO1ExtRefCalibrationReady() == false)
  {
      /* Wait for DCO1 synchronization */
  }
 #endif
}
