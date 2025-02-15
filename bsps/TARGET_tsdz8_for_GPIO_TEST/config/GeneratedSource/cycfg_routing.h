/*******************************************************************************
 * File Name: cycfg_routing.h
 *
 * Description:
 * Establishes all necessary connections between hardware elements.
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

#if !defined(CYCFG_ROUTING_H)
#define CYCFG_ROUTING_H

#include "cycfg_notices.h"

#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */

#define ioss_0_port_0_pin_0_ALT XMC_GPIO_MODE_OUTPUT_ALT5
#define ioss_0_port_0_pin_1_ALT XMC_GPIO_MODE_OUTPUT_ALT5
#define ioss_0_port_0_pin_2_ALT XMC_GPIO_MODE_OUTPUT_ALT7
#define ioss_0_port_0_pin_3_ALT XMC_GPIO_MODE_OUTPUT_ALT7
#define ioss_0_port_0_pin_6_ALT XMC_GPIO_MODE_OUTPUT_ALT7
#define ioss_0_port_0_pin_7_INPUT XMC_GPIO_MODE_INPUT_TRISTATE
#define ioss_0_port_0_pin_8_ALT XMC_GPIO_MODE_OUTPUT_ALT5
#define ioss_0_port_0_pin_9_ALT XMC_GPIO_MODE_OUTPUT_ALT5
#define ioss_0_port_1_pin_0_INPUT XMC_GPIO_MODE_INPUT_TRISTATE
#define ioss_0_port_1_pin_1_INPUT XMC_GPIO_MODE_INPUT_TRISTATE
#define ioss_0_port_1_pin_2_INPUT XMC_GPIO_MODE_INPUT_TRISTATE
#define ioss_0_port_1_pin_3_ALT XMC_GPIO_MODE_OUTPUT_ALT2
#define CCU40_IN0_EV0IS_VALUE XMC_CCU4_SLICE_INPUT_E
#define CCU40_IN1_EV0IS_VALUE XMC_CCU4_SLICE_INPUT_F
#define POSIF0_PCONF_DSEL XMC_POSIF_INPUT_PORT_A
#define POSIF0_PCONF_INSEL0 XMC_POSIF_INPUT_PORT_A
#define POSIF0_PCONF_INSEL1 XMC_POSIF_INPUT_PORT_A
#define POSIF0_PCONF_INSEL2 XMC_POSIF_INPUT_PORT_A
#define USIC0_CH1_DX0CR_DSEL_VALUE 3
#define VADC0_BGGTSEL_VALUE 5
#define VADC0_BGXTSEL_VALUE 9

static inline void init_cycfg_routing(void) {}

#if defined(__cplusplus)
}
#endif /* defined(__cplusplus) */

#endif /* CYCFG_ROUTING_H */
