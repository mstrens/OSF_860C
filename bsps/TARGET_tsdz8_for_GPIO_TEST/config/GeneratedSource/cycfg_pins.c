/*******************************************************************************
 * File Name: cycfg_pins.c
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

#include "cycfg_pins.h"

const XMC_GPIO_CONFIG_t PHASE_U_HIGH_config =
{
    .mode = (XMC_GPIO_MODE_t)PHASE_U_HIGH_MODE,
    .output_level = XMC_GPIO_OUTPUT_LEVEL_LOW,
};
const XMC_GPIO_CONFIG_t IN_BRAKE_config =
{
    .mode = (XMC_GPIO_MODE_t)IN_BRAKE_MODE,
    .input_hysteresis = XMC_GPIO_INPUT_HYSTERESIS_STANDARD,
};
const XMC_GPIO_CONFIG_t IN_PAS2_config =
{
    .mode = (XMC_GPIO_MODE_t)IN_PAS2_MODE,
    .input_hysteresis = XMC_GPIO_INPUT_HYSTERESIS_STANDARD,
};
const XMC_GPIO_CONFIG_t PHASE_U_LOW_config =
{
    .mode = (XMC_GPIO_MODE_t)PHASE_U_LOW_MODE,
    .output_level = XMC_GPIO_OUTPUT_LEVEL_LOW,
};
const XMC_GPIO_CONFIG_t PHASE_V_HIGH_config =
{
    .mode = (XMC_GPIO_MODE_t)PHASE_V_HIGH_MODE,
    .output_level = XMC_GPIO_OUTPUT_LEVEL_LOW,
};
const XMC_GPIO_CONFIG_t PHASE_V_LOW_config =
{
    .mode = (XMC_GPIO_MODE_t)PHASE_V_LOW_MODE,
    .output_level = XMC_GPIO_OUTPUT_LEVEL_LOW,
};
const XMC_GPIO_CONFIG_t IN_SPEED_config =
{
    .mode = (XMC_GPIO_MODE_t)IN_SPEED_MODE,
    .input_hysteresis = XMC_GPIO_INPUT_HYSTERESIS_STANDARD,
};
const XMC_GPIO_CONFIG_t IN_PAS1_config =
{
    .mode = (XMC_GPIO_MODE_t)IN_PAS1_MODE,
    .input_hysteresis = XMC_GPIO_INPUT_HYSTERESIS_STANDARD,
};
const XMC_GPIO_CONFIG_t CYBSP_DEBUG_UART_TX_config =
{
    .mode = (XMC_GPIO_MODE_t)CYBSP_DEBUG_UART_TX_MODE,
    .output_level = XMC_GPIO_OUTPUT_LEVEL_LOW,
};
const XMC_GPIO_CONFIG_t CYBSP_DEBUG_UART_RX_config =
{
    .mode = (XMC_GPIO_MODE_t)CYBSP_DEBUG_UART_RX_MODE,
    .input_hysteresis = XMC_GPIO_INPUT_HYSTERESIS_STANDARD,
};
const XMC_GPIO_CONFIG_t PHASE_W_HIGH_config =
{
    .mode = (XMC_GPIO_MODE_t)PHASE_W_HIGH_MODE,
    .output_level = XMC_GPIO_OUTPUT_LEVEL_LOW,
};
const XMC_GPIO_CONFIG_t PHASE_W_LOW_config =
{
    .mode = (XMC_GPIO_MODE_t)PHASE_W_LOW_MODE,
    .output_level = XMC_GPIO_OUTPUT_LEVEL_LOW,
};
const XMC_GPIO_CONFIG_t IN_HALL2_config =
{
    .mode = (XMC_GPIO_MODE_t)IN_HALL2_MODE,
    .input_hysteresis = XMC_GPIO_INPUT_HYSTERESIS_STANDARD,
};
const XMC_GPIO_CONFIG_t IN_HALL1_config =
{
    .mode = (XMC_GPIO_MODE_t)IN_HALL1_MODE,
    .input_hysteresis = XMC_GPIO_INPUT_HYSTERESIS_STANDARD,
};
const XMC_GPIO_CONFIG_t IN_HALL0_config =
{
    .mode = (XMC_GPIO_MODE_t)IN_HALL0_MODE,
    .input_hysteresis = XMC_GPIO_INPUT_HYSTERESIS_STANDARD,
};
const XMC_GPIO_CONFIG_t PWM_TORQUE_config =
{
    .mode = (XMC_GPIO_MODE_t)PWM_TORQUE_MODE,
    .output_level = XMC_GPIO_OUTPUT_LEVEL_LOW,
};
const XMC_GPIO_CONFIG_t OUT_LIGHT_config =
{
    .mode = (XMC_GPIO_MODE_t)OUT_LIGHT_MODE,
    .output_level = XMC_GPIO_OUTPUT_LEVEL_LOW,
};
const XMC_GPIO_CONFIG_t CURRENT_V_P2_10_config =
{
    .mode = (XMC_GPIO_MODE_t)CURRENT_V_P2_10_MODE,
    .input_hysteresis = XMC_GPIO_INPUT_HYSTERESIS_STANDARD,
};
const XMC_GPIO_CONFIG_t CURRENT_W_P2_11_config =
{
    .mode = (XMC_GPIO_MODE_t)CURRENT_W_P2_11_MODE,
    .input_hysteresis = XMC_GPIO_INPUT_HYSTERESIS_STANDARD,
};
const XMC_GPIO_CONFIG_t TORQUE_P2_2_config =
{
    .mode = (XMC_GPIO_MODE_t)TORQUE_P2_2_MODE,
    .input_hysteresis = XMC_GPIO_INPUT_HYSTERESIS_STANDARD,
};
const XMC_GPIO_CONFIG_t BATTERY_P2_4_config =
{
    .mode = (XMC_GPIO_MODE_t)BATTERY_P2_4_MODE,
    .input_hysteresis = XMC_GPIO_INPUT_HYSTERESIS_STANDARD,
};
const XMC_GPIO_CONFIG_t THROTTLE_P2_5_config =
{
    .mode = (XMC_GPIO_MODE_t)THROTTLE_P2_5_MODE,
    .input_hysteresis = XMC_GPIO_INPUT_HYSTERESIS_STANDARD,
};
const XMC_GPIO_CONFIG_t VCC_P2_6_config =
{
    .mode = (XMC_GPIO_MODE_t)VCC_P2_6_MODE,
    .output_level = XMC_GPIO_OUTPUT_LEVEL_LOW,
};
const XMC_GPIO_CONFIG_t CURRENT_P2_8_config =
{
    .mode = (XMC_GPIO_MODE_t)CURRENT_P2_8_MODE,
    .input_hysteresis = XMC_GPIO_INPUT_HYSTERESIS_STANDARD,
};
const XMC_GPIO_CONFIG_t CURRENT_U_P2_9_config =
{
    .mode = (XMC_GPIO_MODE_t)CURRENT_U_P2_9_MODE,
    .input_hysteresis = XMC_GPIO_INPUT_HYSTERESIS_STANDARD,
};

void init_cycfg_pins(void)
{
    XMC_GPIO_Init(PHASE_U_HIGH_PORT, PHASE_U_HIGH_PIN, &PHASE_U_HIGH_config);
    XMC_GPIO_SetHardwareControl(PHASE_U_HIGH_PORT, PHASE_U_HIGH_PIN, PHASE_U_HIGH_HWO);
    XMC_GPIO_Init(IN_BRAKE_PORT, IN_BRAKE_PIN, &IN_BRAKE_config);
    XMC_GPIO_SetHardwareControl(IN_BRAKE_PORT, IN_BRAKE_PIN, IN_BRAKE_HWO);
    XMC_GPIO_Init(IN_PAS2_PORT, IN_PAS2_PIN, &IN_PAS2_config);
    XMC_GPIO_SetHardwareControl(IN_PAS2_PORT, IN_PAS2_PIN, IN_PAS2_HWO);
    XMC_GPIO_Init(PHASE_U_LOW_PORT, PHASE_U_LOW_PIN, &PHASE_U_LOW_config);
    XMC_GPIO_SetHardwareControl(PHASE_U_LOW_PORT, PHASE_U_LOW_PIN, PHASE_U_LOW_HWO);
    XMC_GPIO_Init(PHASE_V_HIGH_PORT, PHASE_V_HIGH_PIN, &PHASE_V_HIGH_config);
    XMC_GPIO_SetHardwareControl(PHASE_V_HIGH_PORT, PHASE_V_HIGH_PIN, PHASE_V_HIGH_HWO);
    XMC_GPIO_Init(PHASE_V_LOW_PORT, PHASE_V_LOW_PIN, &PHASE_V_LOW_config);
    XMC_GPIO_SetHardwareControl(PHASE_V_LOW_PORT, PHASE_V_LOW_PIN, PHASE_V_LOW_HWO);
    XMC_GPIO_Init(IN_SPEED_PORT, IN_SPEED_PIN, &IN_SPEED_config);
    XMC_GPIO_SetHardwareControl(IN_SPEED_PORT, IN_SPEED_PIN, IN_SPEED_HWO);
    XMC_GPIO_Init(IN_PAS1_PORT, IN_PAS1_PIN, &IN_PAS1_config);
    XMC_GPIO_SetHardwareControl(IN_PAS1_PORT, IN_PAS1_PIN, IN_PAS1_HWO);
    XMC_GPIO_Init(CYBSP_DEBUG_UART_TX_PORT, CYBSP_DEBUG_UART_TX_PIN, &CYBSP_DEBUG_UART_TX_config);
    XMC_GPIO_SetHardwareControl(CYBSP_DEBUG_UART_TX_PORT, CYBSP_DEBUG_UART_TX_PIN, CYBSP_DEBUG_UART_TX_HWO);
    XMC_GPIO_Init(CYBSP_DEBUG_UART_RX_PORT, CYBSP_DEBUG_UART_RX_PIN, &CYBSP_DEBUG_UART_RX_config);
    XMC_GPIO_SetHardwareControl(CYBSP_DEBUG_UART_RX_PORT, CYBSP_DEBUG_UART_RX_PIN, CYBSP_DEBUG_UART_RX_HWO);
    XMC_GPIO_Init(PHASE_W_HIGH_PORT, PHASE_W_HIGH_PIN, &PHASE_W_HIGH_config);
    XMC_GPIO_SetHardwareControl(PHASE_W_HIGH_PORT, PHASE_W_HIGH_PIN, PHASE_W_HIGH_HWO);
    XMC_GPIO_Init(PHASE_W_LOW_PORT, PHASE_W_LOW_PIN, &PHASE_W_LOW_config);
    XMC_GPIO_SetHardwareControl(PHASE_W_LOW_PORT, PHASE_W_LOW_PIN, PHASE_W_LOW_HWO);
    XMC_GPIO_Init(IN_HALL2_PORT, IN_HALL2_PIN, &IN_HALL2_config);
    XMC_GPIO_SetHardwareControl(IN_HALL2_PORT, IN_HALL2_PIN, IN_HALL2_HWO);
    XMC_GPIO_Init(IN_HALL1_PORT, IN_HALL1_PIN, &IN_HALL1_config);
    XMC_GPIO_SetHardwareControl(IN_HALL1_PORT, IN_HALL1_PIN, IN_HALL1_HWO);
    XMC_GPIO_Init(IN_HALL0_PORT, IN_HALL0_PIN, &IN_HALL0_config);
    XMC_GPIO_SetHardwareControl(IN_HALL0_PORT, IN_HALL0_PIN, IN_HALL0_HWO);
    XMC_GPIO_Init(PWM_TORQUE_PORT, PWM_TORQUE_PIN, &PWM_TORQUE_config);
    XMC_GPIO_SetHardwareControl(PWM_TORQUE_PORT, PWM_TORQUE_PIN, PWM_TORQUE_HWO);
    XMC_GPIO_Init(OUT_LIGHT_PORT, OUT_LIGHT_PIN, &OUT_LIGHT_config);
    XMC_GPIO_SetHardwareControl(OUT_LIGHT_PORT, OUT_LIGHT_PIN, OUT_LIGHT_HWO);
    XMC_GPIO_Init(CURRENT_V_P2_10_PORT, CURRENT_V_P2_10_PIN, &CURRENT_V_P2_10_config);
    XMC_GPIO_SetHardwareControl(CURRENT_V_P2_10_PORT, CURRENT_V_P2_10_PIN, CURRENT_V_P2_10_HWO);
    XMC_GPIO_Init(CURRENT_W_P2_11_PORT, CURRENT_W_P2_11_PIN, &CURRENT_W_P2_11_config);
    XMC_GPIO_SetHardwareControl(CURRENT_W_P2_11_PORT, CURRENT_W_P2_11_PIN, CURRENT_W_P2_11_HWO);
    XMC_GPIO_Init(TORQUE_P2_2_PORT, TORQUE_P2_2_PIN, &TORQUE_P2_2_config);
    XMC_GPIO_SetHardwareControl(TORQUE_P2_2_PORT, TORQUE_P2_2_PIN, TORQUE_P2_2_HWO);
    XMC_GPIO_Init(BATTERY_P2_4_PORT, BATTERY_P2_4_PIN, &BATTERY_P2_4_config);
    XMC_GPIO_SetHardwareControl(BATTERY_P2_4_PORT, BATTERY_P2_4_PIN, BATTERY_P2_4_HWO);
    XMC_GPIO_Init(THROTTLE_P2_5_PORT, THROTTLE_P2_5_PIN, &THROTTLE_P2_5_config);
    XMC_GPIO_SetHardwareControl(THROTTLE_P2_5_PORT, THROTTLE_P2_5_PIN, THROTTLE_P2_5_HWO);
    XMC_GPIO_Init(VCC_P2_6_PORT, VCC_P2_6_PIN, &VCC_P2_6_config);
    XMC_GPIO_SetHardwareControl(VCC_P2_6_PORT, VCC_P2_6_PIN, VCC_P2_6_HWO);
    XMC_GPIO_Init(CURRENT_P2_8_PORT, CURRENT_P2_8_PIN, &CURRENT_P2_8_config);
    XMC_GPIO_SetHardwareControl(CURRENT_P2_8_PORT, CURRENT_P2_8_PIN, CURRENT_P2_8_HWO);
    XMC_GPIO_Init(CURRENT_U_P2_9_PORT, CURRENT_U_P2_9_PIN, &CURRENT_U_P2_9_config);
    XMC_GPIO_SetHardwareControl(CURRENT_U_P2_9_PORT, CURRENT_U_P2_9_PIN, CURRENT_U_P2_9_HWO);
}
