/*******************************************************************************
 * File Name: cycfg_pins.h
 *
 * Description:
 * Pin configuration
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

#if !defined(CYCFG_PINS_H)
#define CYCFG_PINS_H

#include "cycfg_notices.h"
#include "xmc_gpio.h"
#include "cycfg_routing.h"

#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */

#define PHASE_U_HIGH_ENABLED 1U
#define PHASE_U_HIGH_PORT XMC_GPIO_PORT0
#define PHASE_U_HIGH_PORT_NUM 0U
#define PHASE_U_HIGH_PIN 0U
#ifndef ioss_0_port_0_pin_0_ALT
    #define ioss_0_port_0_pin_0_ALT 0U
#endif
#define PHASE_U_HIGH_MODE (XMC_GPIO_MODE_OUTPUT_PUSH_PULL | ioss_0_port_0_pin_0_ALT)
#ifndef ioss_0_port_0_pin_0_HWO
    #define ioss_0_port_0_pin_0_HWO XMC_GPIO_HWCTRL_DISABLED
#endif
#define PHASE_U_HIGH_HWO ioss_0_port_0_pin_0_HWO
#define IN_BRAKE_ENABLED 1U
#define IN_BRAKE_PORT XMC_GPIO_PORT0
#define IN_BRAKE_PORT_NUM 0U
#define IN_BRAKE_PIN 10U
#ifndef ioss_0_port_0_pin_10_INPUT
    #define ioss_0_port_0_pin_10_INPUT 0U
#endif
#define IN_BRAKE_MODE (XMC_GPIO_MODE_INPUT_PULL_UP | ioss_0_port_0_pin_10_INPUT)
#ifndef ioss_0_port_0_pin_10_HWO
    #define ioss_0_port_0_pin_10_HWO XMC_GPIO_HWCTRL_DISABLED
#endif
#define IN_BRAKE_HWO ioss_0_port_0_pin_10_HWO
#define IN_UNKNOWN_0_11_PORT XMC_GPIO_PORT0
#define IN_UNKNOWN_0_11_PORT_NUM 0U
#define IN_UNKNOWN_0_11_PIN 11U
#define IN_UNKNOWN_0_12_PORT XMC_GPIO_PORT0
#define IN_UNKNOWN_0_12_PORT_NUM 0U
#define IN_UNKNOWN_0_12_PIN 12U
#define IN_PAS2_ENABLED 1U
#define IN_PAS2_PORT XMC_GPIO_PORT0
#define IN_PAS2_PORT_NUM 0U
#define IN_PAS2_PIN 13U
#ifndef ioss_0_port_0_pin_13_INPUT
    #define ioss_0_port_0_pin_13_INPUT 0U
#endif
#define IN_PAS2_MODE (XMC_GPIO_MODE_INPUT_PULL_UP | ioss_0_port_0_pin_13_INPUT)
#ifndef ioss_0_port_0_pin_13_HWO
    #define ioss_0_port_0_pin_13_HWO XMC_GPIO_HWCTRL_DISABLED
#endif
#define IN_PAS2_HWO ioss_0_port_0_pin_13_HWO
#define CYBSP_SWDIO_PORT XMC_GPIO_PORT0
#define CYBSP_SWDIO_PORT_NUM 0U
#define CYBSP_SWDIO_PIN 14U
#define CYBSP_SWDCK_PORT XMC_GPIO_PORT0
#define CYBSP_SWDCK_PORT_NUM 0U
#define CYBSP_SWDCK_PIN 15U
#define PHASE_U_LOW_ENABLED 1U
#define PHASE_U_LOW_PORT XMC_GPIO_PORT0
#define PHASE_U_LOW_PORT_NUM 0U
#define PHASE_U_LOW_PIN 1U
#ifndef ioss_0_port_0_pin_1_ALT
    #define ioss_0_port_0_pin_1_ALT 0U
#endif
#define PHASE_U_LOW_MODE (XMC_GPIO_MODE_OUTPUT_PUSH_PULL | ioss_0_port_0_pin_1_ALT)
#ifndef ioss_0_port_0_pin_1_HWO
    #define ioss_0_port_0_pin_1_HWO XMC_GPIO_HWCTRL_DISABLED
#endif
#define PHASE_U_LOW_HWO ioss_0_port_0_pin_1_HWO
#define PHASE_V_HIGH_ENABLED 1U
#define PHASE_V_HIGH_PORT XMC_GPIO_PORT0
#define PHASE_V_HIGH_PORT_NUM 0U
#define PHASE_V_HIGH_PIN 2U
#ifndef ioss_0_port_0_pin_2_ALT
    #define ioss_0_port_0_pin_2_ALT 0U
#endif
#define PHASE_V_HIGH_MODE (XMC_GPIO_MODE_OUTPUT_PUSH_PULL | ioss_0_port_0_pin_2_ALT)
#ifndef ioss_0_port_0_pin_2_HWO
    #define ioss_0_port_0_pin_2_HWO XMC_GPIO_HWCTRL_DISABLED
#endif
#define PHASE_V_HIGH_HWO ioss_0_port_0_pin_2_HWO
#define PHASE_V_LOW_ENABLED 1U
#define PHASE_V_LOW_PORT XMC_GPIO_PORT0
#define PHASE_V_LOW_PORT_NUM 0U
#define PHASE_V_LOW_PIN 3U
#ifndef ioss_0_port_0_pin_3_ALT
    #define ioss_0_port_0_pin_3_ALT 0U
#endif
#define PHASE_V_LOW_MODE (XMC_GPIO_MODE_OUTPUT_PUSH_PULL | ioss_0_port_0_pin_3_ALT)
#ifndef ioss_0_port_0_pin_3_HWO
    #define ioss_0_port_0_pin_3_HWO XMC_GPIO_HWCTRL_DISABLED
#endif
#define PHASE_V_LOW_HWO ioss_0_port_0_pin_3_HWO
#define IN_SPEED_ENABLED 1U
#define IN_SPEED_PORT XMC_GPIO_PORT0
#define IN_SPEED_PORT_NUM 0U
#define IN_SPEED_PIN 4U
#ifndef ioss_0_port_0_pin_4_INPUT
    #define ioss_0_port_0_pin_4_INPUT 0U
#endif
#define IN_SPEED_MODE (XMC_GPIO_MODE_INPUT_PULL_DOWN | ioss_0_port_0_pin_4_INPUT)
#ifndef ioss_0_port_0_pin_4_HWO
    #define ioss_0_port_0_pin_4_HWO XMC_GPIO_HWCTRL_DISABLED
#endif
#define IN_SPEED_HWO ioss_0_port_0_pin_4_HWO
#define IN_PAS1_ENABLED 1U
#define IN_PAS1_PORT XMC_GPIO_PORT0
#define IN_PAS1_PORT_NUM 0U
#define IN_PAS1_PIN 5U
#ifndef ioss_0_port_0_pin_5_INPUT
    #define ioss_0_port_0_pin_5_INPUT 0U
#endif
#define IN_PAS1_MODE (XMC_GPIO_MODE_INPUT_PULL_UP | ioss_0_port_0_pin_5_INPUT)
#ifndef ioss_0_port_0_pin_5_HWO
    #define ioss_0_port_0_pin_5_HWO XMC_GPIO_HWCTRL_DISABLED
#endif
#define IN_PAS1_HWO ioss_0_port_0_pin_5_HWO
#define CYBSP_DEBUG_UART_TX_ENABLED 1U
#define CYBSP_DEBUG_UART_TX_PORT XMC_GPIO_PORT0
#define CYBSP_DEBUG_UART_TX_PORT_NUM 0U
#define CYBSP_DEBUG_UART_TX_PIN 6U
#ifndef ioss_0_port_0_pin_6_ALT
    #define ioss_0_port_0_pin_6_ALT 0U
#endif
#define CYBSP_DEBUG_UART_TX_MODE (XMC_GPIO_MODE_OUTPUT_PUSH_PULL | ioss_0_port_0_pin_6_ALT)
#ifndef ioss_0_port_0_pin_6_HWO
    #define ioss_0_port_0_pin_6_HWO XMC_GPIO_HWCTRL_DISABLED
#endif
#define CYBSP_DEBUG_UART_TX_HWO ioss_0_port_0_pin_6_HWO
#define CYBSP_DEBUG_UART_RX_ENABLED 1U
#define CYBSP_DEBUG_UART_RX_PORT XMC_GPIO_PORT0
#define CYBSP_DEBUG_UART_RX_PORT_NUM 0U
#define CYBSP_DEBUG_UART_RX_PIN 7U
#ifndef ioss_0_port_0_pin_7_INPUT
    #define ioss_0_port_0_pin_7_INPUT 0U
#endif
#define CYBSP_DEBUG_UART_RX_MODE (XMC_GPIO_MODE_INPUT_PULL_UP | ioss_0_port_0_pin_7_INPUT)
#ifndef ioss_0_port_0_pin_7_HWO
    #define ioss_0_port_0_pin_7_HWO XMC_GPIO_HWCTRL_DISABLED
#endif
#define CYBSP_DEBUG_UART_RX_HWO ioss_0_port_0_pin_7_HWO
#define PHASE_W_HIGH_ENABLED 1U
#define PHASE_W_HIGH_PORT XMC_GPIO_PORT0
#define PHASE_W_HIGH_PORT_NUM 0U
#define PHASE_W_HIGH_PIN 8U
#ifndef ioss_0_port_0_pin_8_ALT
    #define ioss_0_port_0_pin_8_ALT 0U
#endif
#define PHASE_W_HIGH_MODE (XMC_GPIO_MODE_OUTPUT_PUSH_PULL | ioss_0_port_0_pin_8_ALT)
#ifndef ioss_0_port_0_pin_8_HWO
    #define ioss_0_port_0_pin_8_HWO XMC_GPIO_HWCTRL_DISABLED
#endif
#define PHASE_W_HIGH_HWO ioss_0_port_0_pin_8_HWO
#define PHASE_W_LOW_ENABLED 1U
#define PHASE_W_LOW_PORT XMC_GPIO_PORT0
#define PHASE_W_LOW_PORT_NUM 0U
#define PHASE_W_LOW_PIN 9U
#ifndef ioss_0_port_0_pin_9_ALT
    #define ioss_0_port_0_pin_9_ALT 0U
#endif
#define PHASE_W_LOW_MODE (XMC_GPIO_MODE_OUTPUT_PUSH_PULL | ioss_0_port_0_pin_9_ALT)
#ifndef ioss_0_port_0_pin_9_HWO
    #define ioss_0_port_0_pin_9_HWO XMC_GPIO_HWCTRL_DISABLED
#endif
#define PHASE_W_LOW_HWO ioss_0_port_0_pin_9_HWO
#define IN_HALL2_ENABLED 1U
#define IN_HALL2_PORT XMC_GPIO_PORT1
#define IN_HALL2_PORT_NUM 1U
#define IN_HALL2_PIN 0U
#ifndef ioss_0_port_1_pin_0_INPUT
    #define ioss_0_port_1_pin_0_INPUT 0U
#endif
#define IN_HALL2_MODE (XMC_GPIO_MODE_INPUT_PULL_DOWN | ioss_0_port_1_pin_0_INPUT)
#ifndef ioss_0_port_1_pin_0_HWO
    #define ioss_0_port_1_pin_0_HWO XMC_GPIO_HWCTRL_DISABLED
#endif
#define IN_HALL2_HWO ioss_0_port_1_pin_0_HWO
#define IN_HALL1_ENABLED 1U
#define IN_HALL1_PORT XMC_GPIO_PORT1
#define IN_HALL1_PORT_NUM 1U
#define IN_HALL1_PIN 1U
#ifndef ioss_0_port_1_pin_1_INPUT
    #define ioss_0_port_1_pin_1_INPUT 0U
#endif
#define IN_HALL1_MODE (XMC_GPIO_MODE_INPUT_PULL_DOWN | ioss_0_port_1_pin_1_INPUT)
#ifndef ioss_0_port_1_pin_1_HWO
    #define ioss_0_port_1_pin_1_HWO XMC_GPIO_HWCTRL_DISABLED
#endif
#define IN_HALL1_HWO ioss_0_port_1_pin_1_HWO
#define IN_HALL0_ENABLED 1U
#define IN_HALL0_PORT XMC_GPIO_PORT1
#define IN_HALL0_PORT_NUM 1U
#define IN_HALL0_PIN 2U
#ifndef ioss_0_port_1_pin_2_INPUT
    #define ioss_0_port_1_pin_2_INPUT 0U
#endif
#define IN_HALL0_MODE (XMC_GPIO_MODE_INPUT_PULL_DOWN | ioss_0_port_1_pin_2_INPUT)
#ifndef ioss_0_port_1_pin_2_HWO
    #define ioss_0_port_1_pin_2_HWO XMC_GPIO_HWCTRL_DISABLED
#endif
#define IN_HALL0_HWO ioss_0_port_1_pin_2_HWO
#define PWM_TORQUE_ENABLED 1U
#define PWM_TORQUE_PORT XMC_GPIO_PORT1
#define PWM_TORQUE_PORT_NUM 1U
#define PWM_TORQUE_PIN 3U
#ifndef ioss_0_port_1_pin_3_ALT
    #define ioss_0_port_1_pin_3_ALT 0U
#endif
#define PWM_TORQUE_MODE (XMC_GPIO_MODE_OUTPUT_PUSH_PULL | ioss_0_port_1_pin_3_ALT)
#ifndef ioss_0_port_1_pin_3_HWO
    #define ioss_0_port_1_pin_3_HWO XMC_GPIO_HWCTRL_DISABLED
#endif
#define PWM_TORQUE_HWO ioss_0_port_1_pin_3_HWO
#define IN_UNKNOWN_1_4_PORT XMC_GPIO_PORT1
#define IN_UNKNOWN_1_4_PORT_NUM 1U
#define IN_UNKNOWN_1_4_PIN 4U
#define OUT_LIGHT_ENABLED 1U
#define OUT_LIGHT_PORT XMC_GPIO_PORT1
#define OUT_LIGHT_PORT_NUM 1U
#define OUT_LIGHT_PIN 5U
#ifndef ioss_0_port_1_pin_5_ALT
    #define ioss_0_port_1_pin_5_ALT 0U
#endif
#define OUT_LIGHT_MODE (XMC_GPIO_MODE_OUTPUT_PUSH_PULL | ioss_0_port_1_pin_5_ALT)
#ifndef ioss_0_port_1_pin_5_HWO
    #define ioss_0_port_1_pin_5_HWO XMC_GPIO_HWCTRL_DISABLED
#endif
#define OUT_LIGHT_HWO ioss_0_port_1_pin_5_HWO
#define CURRENT_V_P2_10_ENABLED 1U
#define CURRENT_V_P2_10_PORT XMC_GPIO_PORT2
#define CURRENT_V_P2_10_PORT_NUM 2U
#define CURRENT_V_P2_10_PIN 10U
#ifndef ioss_0_port_2_pin_10_INPUT
    #define ioss_0_port_2_pin_10_INPUT 0U
#endif
#define CURRENT_V_P2_10_MODE (XMC_GPIO_MODE_INPUT_TRISTATE | ioss_0_port_2_pin_10_INPUT)
#ifndef ioss_0_port_2_pin_10_HWO
    #define ioss_0_port_2_pin_10_HWO XMC_GPIO_HWCTRL_DISABLED
#endif
#define CURRENT_V_P2_10_HWO ioss_0_port_2_pin_10_HWO
#define CURRENT_W_P2_11_ENABLED 1U
#define CURRENT_W_P2_11_PORT XMC_GPIO_PORT2
#define CURRENT_W_P2_11_PORT_NUM 2U
#define CURRENT_W_P2_11_PIN 11U
#ifndef ioss_0_port_2_pin_11_INPUT
    #define ioss_0_port_2_pin_11_INPUT 0U
#endif
#define CURRENT_W_P2_11_MODE (XMC_GPIO_MODE_INPUT_TRISTATE | ioss_0_port_2_pin_11_INPUT)
#ifndef ioss_0_port_2_pin_11_HWO
    #define ioss_0_port_2_pin_11_HWO XMC_GPIO_HWCTRL_DISABLED
#endif
#define CURRENT_W_P2_11_HWO ioss_0_port_2_pin_11_HWO
#define IN_UNKNOWN_2_1_PORT XMC_GPIO_PORT2
#define IN_UNKNOWN_2_1_PORT_NUM 2U
#define IN_UNKNOWN_2_1_PIN 1U
#define TORQUE_P2_2_ENABLED 1U
#define TORQUE_P2_2_PORT XMC_GPIO_PORT2
#define TORQUE_P2_2_PORT_NUM 2U
#define TORQUE_P2_2_PIN 2U
#ifndef ioss_0_port_2_pin_2_ALT
    #define ioss_0_port_2_pin_2_ALT 0U
#endif
#define TORQUE_P2_2_MODE (XMC_GPIO_MODE_OUTPUT_PUSH_PULL | ioss_0_port_2_pin_2_ALT)
#ifndef ioss_0_port_2_pin_2_HWO
    #define ioss_0_port_2_pin_2_HWO XMC_GPIO_HWCTRL_DISABLED
#endif
#define TORQUE_P2_2_HWO ioss_0_port_2_pin_2_HWO
#define UNKNOWN_P2_3_PORT XMC_GPIO_PORT2
#define UNKNOWN_P2_3_PORT_NUM 2U
#define UNKNOWN_P2_3_PIN 3U
#define BATTERY_P2_4_ENABLED 1U
#define BATTERY_P2_4_PORT XMC_GPIO_PORT2
#define BATTERY_P2_4_PORT_NUM 2U
#define BATTERY_P2_4_PIN 4U
#ifndef ioss_0_port_2_pin_4_ALT
    #define ioss_0_port_2_pin_4_ALT 0U
#endif
#define BATTERY_P2_4_MODE (XMC_GPIO_MODE_OUTPUT_PUSH_PULL | ioss_0_port_2_pin_4_ALT)
#ifndef ioss_0_port_2_pin_4_HWO
    #define ioss_0_port_2_pin_4_HWO XMC_GPIO_HWCTRL_DISABLED
#endif
#define BATTERY_P2_4_HWO ioss_0_port_2_pin_4_HWO
#define THROTTLE_P2_5_ENABLED 1U
#define THROTTLE_P2_5_PORT XMC_GPIO_PORT2
#define THROTTLE_P2_5_PORT_NUM 2U
#define THROTTLE_P2_5_PIN 5U
#ifndef ioss_0_port_2_pin_5_ALT
    #define ioss_0_port_2_pin_5_ALT 0U
#endif
#define THROTTLE_P2_5_MODE (XMC_GPIO_MODE_OUTPUT_PUSH_PULL | ioss_0_port_2_pin_5_ALT)
#ifndef ioss_0_port_2_pin_5_HWO
    #define ioss_0_port_2_pin_5_HWO XMC_GPIO_HWCTRL_DISABLED
#endif
#define THROTTLE_P2_5_HWO ioss_0_port_2_pin_5_HWO
#define VCC_P2_6_ENABLED 1U
#define VCC_P2_6_PORT XMC_GPIO_PORT2
#define VCC_P2_6_PORT_NUM 2U
#define VCC_P2_6_PIN 6U
#ifndef ioss_0_port_2_pin_6_ALT
    #define ioss_0_port_2_pin_6_ALT 0U
#endif
#define VCC_P2_6_MODE (XMC_GPIO_MODE_OUTPUT_PUSH_PULL | ioss_0_port_2_pin_6_ALT)
#ifndef ioss_0_port_2_pin_6_HWO
    #define ioss_0_port_2_pin_6_HWO XMC_GPIO_HWCTRL_DISABLED
#endif
#define VCC_P2_6_HWO ioss_0_port_2_pin_6_HWO
#define UNKNOWN_P2_7_PORT XMC_GPIO_PORT2
#define UNKNOWN_P2_7_PORT_NUM 2U
#define UNKNOWN_P2_7_PIN 7U
#define CURRENT_P2_8_ENABLED 1U
#define CURRENT_P2_8_PORT XMC_GPIO_PORT2
#define CURRENT_P2_8_PORT_NUM 2U
#define CURRENT_P2_8_PIN 8U
#ifndef ioss_0_port_2_pin_8_INPUT
    #define ioss_0_port_2_pin_8_INPUT 0U
#endif
#define CURRENT_P2_8_MODE (XMC_GPIO_MODE_INPUT_TRISTATE | ioss_0_port_2_pin_8_INPUT)
#ifndef ioss_0_port_2_pin_8_HWO
    #define ioss_0_port_2_pin_8_HWO XMC_GPIO_HWCTRL_DISABLED
#endif
#define CURRENT_P2_8_HWO ioss_0_port_2_pin_8_HWO
#define CURRENT_U_P2_9_ENABLED 1U
#define CURRENT_U_P2_9_PORT XMC_GPIO_PORT2
#define CURRENT_U_P2_9_PORT_NUM 2U
#define CURRENT_U_P2_9_PIN 9U
#ifndef ioss_0_port_2_pin_9_INPUT
    #define ioss_0_port_2_pin_9_INPUT 0U
#endif
#define CURRENT_U_P2_9_MODE (XMC_GPIO_MODE_INPUT_TRISTATE | ioss_0_port_2_pin_9_INPUT)
#ifndef ioss_0_port_2_pin_9_HWO
    #define ioss_0_port_2_pin_9_HWO XMC_GPIO_HWCTRL_DISABLED
#endif
#define CURRENT_U_P2_9_HWO ioss_0_port_2_pin_9_HWO

extern const XMC_GPIO_CONFIG_t PHASE_U_HIGH_config;
extern const XMC_GPIO_CONFIG_t IN_BRAKE_config;
extern const XMC_GPIO_CONFIG_t IN_PAS2_config;
extern const XMC_GPIO_CONFIG_t PHASE_U_LOW_config;
extern const XMC_GPIO_CONFIG_t PHASE_V_HIGH_config;
extern const XMC_GPIO_CONFIG_t PHASE_V_LOW_config;
extern const XMC_GPIO_CONFIG_t IN_SPEED_config;
extern const XMC_GPIO_CONFIG_t IN_PAS1_config;
extern const XMC_GPIO_CONFIG_t CYBSP_DEBUG_UART_TX_config;
extern const XMC_GPIO_CONFIG_t CYBSP_DEBUG_UART_RX_config;
extern const XMC_GPIO_CONFIG_t PHASE_W_HIGH_config;
extern const XMC_GPIO_CONFIG_t PHASE_W_LOW_config;
extern const XMC_GPIO_CONFIG_t IN_HALL2_config;
extern const XMC_GPIO_CONFIG_t IN_HALL1_config;
extern const XMC_GPIO_CONFIG_t IN_HALL0_config;
extern const XMC_GPIO_CONFIG_t PWM_TORQUE_config;
extern const XMC_GPIO_CONFIG_t OUT_LIGHT_config;
extern const XMC_GPIO_CONFIG_t CURRENT_V_P2_10_config;
extern const XMC_GPIO_CONFIG_t CURRENT_W_P2_11_config;
extern const XMC_GPIO_CONFIG_t TORQUE_P2_2_config;
extern const XMC_GPIO_CONFIG_t BATTERY_P2_4_config;
extern const XMC_GPIO_CONFIG_t THROTTLE_P2_5_config;
extern const XMC_GPIO_CONFIG_t VCC_P2_6_config;
extern const XMC_GPIO_CONFIG_t CURRENT_P2_8_config;
extern const XMC_GPIO_CONFIG_t CURRENT_U_P2_9_config;

void init_cycfg_pins(void);

#if defined(__cplusplus)
}
#endif /* defined(__cplusplus) */

#endif /* CYCFG_PINS_H */
