/*******************************************************************************
 * File Name: cycfg_peripherals.h
 *
 * Description:
 * Peripheral Hardware Block configuration
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

#if !defined(CYCFG_PERIPHERALS_H)
#define CYCFG_PERIPHERALS_H

#include "cycfg_notices.h"
#include "xmc_ccu4.h"
#include "cycfg_routing.h"
#include "xmc_ccu8.h"
#include "xmc_posif.h"
#include "xmc_uart.h"
#include "xmc_vadc.h"
#include "xmc_wdt.h"

#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */

#define ccu4_0_ENABLED 1U
#define ccu4_0_NUM 0U
#define ccu4_0_HW CCU40
#define ccu4_0_SR1_INTERRUPT_HANDLER CCU40_1_IRQHandler
#define ccu4_0_SR1_IRQN CCU40_1_IRQn
#define HALL_DELAY_TIMER_ENABLED 1U
#define HALL_DELAY_TIMER_NUM 0U
#define HALL_DELAY_TIMER_HW CCU40_CC40
#define HALL_DELAY_TIMER_TICK_NS 16U
#define HALL_SPEED_TIMER_ENABLED 1U
#define HALL_SPEED_TIMER_NUM 1U
#define HALL_SPEED_TIMER_HW CCU40_CC41
#define HALL_SPEED_TIMER_TICK_NS 4000U
#define PWM_TORQUE_TIMER_ENABLED 1U
#define PWM_TORQUE_TIMER_NUM 3U
#define PWM_TORQUE_TIMER_HW CCU40_CC43
#define PWM_TORQUE_TIMER_TICK_NS 16U
#define ccu8_0_ENABLED 1U
#define ccu8_0_NUM 0U
#define ccu8_0_HW CCU80
#define ccu8_0_SR0_INTERRUPT_HANDLER CCU80_0_IRQHandler
#define ccu8_0_SR1_INTERRUPT_HANDLER CCU80_1_IRQHandler
#define ccu8_0_SR2_INTERRUPT_HANDLER CCU80_2_IRQHandler
#define ccu8_0_SR3_INTERRUPT_HANDLER CCU80_3_IRQHandler
#define ccu8_0_SR0_IRQN CCU80_0_IRQn
#define ccu8_0_SR1_IRQN CCU80_1_IRQn
#define ccu8_0_SR2_IRQN CCU80_2_IRQn
#define ccu8_0_SR3_IRQN CCU80_3_IRQn
#define PHASE_U_TIMER_ENABLED 1U
#define PHASE_U_TIMER_NUM 0U
#define PHASE_U_TIMER_HW CCU80_CC80
#define PHASE_U_TIMER_TICK_NS 16U
#define PHASE_V_TIMER_ENABLED 1U
#define PHASE_V_TIMER_NUM 1U
#define PHASE_V_TIMER_HW CCU80_CC81
#define PHASE_V_TIMER_TICK_NS 16U
#define PHASE_W_TIMER_ENABLED 1U
#define PHASE_W_TIMER_NUM 2U
#define PHASE_W_TIMER_HW CCU80_CC82
#define PHASE_W_TIMER_TICK_NS 16U
#define PWM_IRQ_TIMER_ENABLED 1U
#define PWM_IRQ_TIMER_NUM 3U
#define PWM_IRQ_TIMER_HW CCU80_CC83
#define PWM_IRQ_TIMER_TICK_NS 16U
#define HALL_POSIF_ENABLED 1U
#define HALL_POSIF_NUM 0U
#define HALL_POSIF_HW POSIF0
#define HALL_POSIF_SR0_INTERRUPT_HANDLER POSIF0_0_IRQHandler
#define HALL_POSIF_SR0_IRQN POSIF0_0_IRQn
#define CYBSP_DEBUG_UART_ENABLED 1U
#define CYBSP_DEBUG_UART_NUM 1U
#define CYBSP_DEBUG_UART_HW XMC_UART0_CH1
#define CYBSP_DEBUG_UART_DX0_INPUT USIC0_CH1_DX0CR_DSEL_VALUE
#define CYBSP_DEBUG_UART_RXFIFO_SIZE XMC_USIC_CH_FIFO_SIZE_32WORDS
#define CYBSP_DEBUG_UART_RXFIFO_DPTR 0
#define CYBSP_DEBUG_UART_RXFIFO_LIMIT 31
#define CYBSP_DEBUG_UART_TXFIFO_SIZE XMC_USIC_CH_FIFO_SIZE_32WORDS
#define CYBSP_DEBUG_UART_TXFIFO_DPTR 32
#define CYBSP_DEBUG_UART_TXFIFO_LIMIT 31
#define vadc_0_ENABLED 1U
#define vadc_0_HW VADC
#define vadc_0_SR0_INTERRUPT_HANDLER VADC0_C0_0_IRQHandler
#define vadc_0_SR1_INTERRUPT_HANDLER VADC0_C0_1_IRQHandler
#define vadc_0_SR0_IRQN VADC0_C0_0_IRQn
#define vadc_0_SR1_IRQN VADC0_C0_1_IRQn
#define vadc_0_group_0_ENABLED 1U
#define vadc_0_group_0_HW VADC_G0
#define vadc_0_group_0_NUM 0U
#define vadc_0_group_0_ICLASS_0 0U
#define vadc_0_group_0_ICLASS_1 1U
#define vadc_0_group_0_LOWER_BOUND_VALUE 0U
#define vadc_0_group_0_UPPER_BOUND_VALUE 1000U
#define vadc_0_group_0_SR0_INTERRUPT_HANDLER VADC0_G0_0_IRQHandler
#define vadc_0_group_0_SR1_INTERRUPT_HANDLER VADC0_G0_1_IRQHandler
#define vadc_0_group_0_SR0_IRQN VADC0_G0_0_IRQn
#define vadc_0_group_0_SR1_IRQN VADC0_G0_1_IRQn
#define G0_CH1_CURRENT_P2_8_ENABLED 1U
#define G0_CH2_CURRENT_U_P2_9_ENABLED 1U
#define G0_CH7_TORQUE_P2_2_ENABLED 1U
#define vadc_0_group_1_ENABLED 1U
#define vadc_0_group_1_HW VADC_G1
#define vadc_0_group_1_NUM 1U
#define vadc_0_group_1_ICLASS_0 0U
#define vadc_0_group_1_ICLASS_1 1U
#define vadc_0_group_1_SR0_INTERRUPT_HANDLER VADC0_G1_0_IRQHandler
#define vadc_0_group_1_SR1_INTERRUPT_HANDLER VADC0_G1_1_IRQHandler
#define vadc_0_group_1_SR0_IRQN VADC0_G1_0_IRQn
#define vadc_0_group_1_SR1_IRQN VADC0_G1_1_IRQn
#define G1_CH0_CURRENT_P2_8_ENABLED 1U
#define G1_CH6_BATTERY_P2_4_ENABLED 1U
#define G1_CH7_THROTTLE_P2_5_ENABLED 1U
#define wdt_0_ENABLED 1U
#define WATCHDOG_EVENT_VIA_SCU 0U
#define WATCHDOG_EVENT_VIA_NMI 0U
#define WATCHDOG_PREWARNING_CHECK 0U

extern const XMC_CCU4_SLICE_COMPARE_CONFIG_t HALL_DELAY_TIMER_compare_config;
extern const XMC_CCU4_SLICE_EVENT_CONFIG_t HALL_DELAY_TIMER_event0_config;
extern const XMC_CCU4_SLICE_EVENT_CONFIG_t HALL_DELAY_TIMER_event1_config;
extern const XMC_CCU4_SLICE_EVENT_CONFIG_t HALL_DELAY_TIMER_event2_config;
extern const XMC_CCU4_SLICE_CAPTURE_CONFIG_t HALL_SPEED_TIMER_capture_config;
extern const XMC_CCU4_SLICE_EVENT_CONFIG_t HALL_SPEED_TIMER_event0_config;
extern const XMC_CCU4_SLICE_EVENT_CONFIG_t HALL_SPEED_TIMER_event1_config;
extern const XMC_CCU4_SLICE_EVENT_CONFIG_t HALL_SPEED_TIMER_event2_config;
extern const XMC_CCU4_SLICE_COMPARE_CONFIG_t PWM_TORQUE_TIMER_compare_config;
extern const XMC_CCU4_SLICE_EVENT_CONFIG_t PWM_TORQUE_TIMER_event0_config;
extern const XMC_CCU4_SLICE_EVENT_CONFIG_t PWM_TORQUE_TIMER_event1_config;
extern const XMC_CCU4_SLICE_EVENT_CONFIG_t PWM_TORQUE_TIMER_event2_config;
extern const XMC_CCU8_SLICE_COMPARE_CONFIG_t PHASE_U_TIMER_compare_config;
extern const XMC_CCU8_SLICE_DEAD_TIME_CONFIG_t PHASE_U_TIMER_dead_time_config;
extern const XMC_CCU8_SLICE_EVENT_CONFIG_t PHASE_U_TIMER_event0_config;
extern const XMC_CCU8_SLICE_EVENT_CONFIG_t PHASE_U_TIMER_event1_config;
extern const XMC_CCU8_SLICE_EVENT_CONFIG_t PHASE_U_TIMER_event2_config;
extern const XMC_CCU8_SLICE_COMPARE_CONFIG_t PHASE_V_TIMER_compare_config;
extern const XMC_CCU8_SLICE_DEAD_TIME_CONFIG_t PHASE_V_TIMER_dead_time_config;
extern const XMC_CCU8_SLICE_EVENT_CONFIG_t PHASE_V_TIMER_event0_config;
extern const XMC_CCU8_SLICE_EVENT_CONFIG_t PHASE_V_TIMER_event1_config;
extern const XMC_CCU8_SLICE_EVENT_CONFIG_t PHASE_V_TIMER_event2_config;
extern const XMC_CCU8_SLICE_COMPARE_CONFIG_t PHASE_W_TIMER_compare_config;
extern const XMC_CCU8_SLICE_DEAD_TIME_CONFIG_t PHASE_W_TIMER_dead_time_config;
extern const XMC_CCU8_SLICE_EVENT_CONFIG_t PHASE_W_TIMER_event0_config;
extern const XMC_CCU8_SLICE_EVENT_CONFIG_t PHASE_W_TIMER_event1_config;
extern const XMC_CCU8_SLICE_EVENT_CONFIG_t PHASE_W_TIMER_event2_config;
extern const XMC_CCU8_SLICE_COMPARE_CONFIG_t PWM_IRQ_TIMER_compare_config;
extern const XMC_CCU8_SLICE_EVENT_CONFIG_t PWM_IRQ_TIMER_event0_config;
extern const XMC_CCU8_SLICE_EVENT_CONFIG_t PWM_IRQ_TIMER_event1_config;
extern const XMC_CCU8_SLICE_EVENT_CONFIG_t PWM_IRQ_TIMER_event2_config;
extern const uint32_t HALL_POSIF_Hall_Pattern[];
extern const XMC_POSIF_CONFIG_t HALL_POSIF_config;
extern const XMC_POSIF_HSC_CONFIG_t HALL_POSIF_HSC_InitHandle;
extern const XMC_UART_CH_CONFIG_t CYBSP_DEBUG_UART_config;
extern XMC_VADC_GROUP_CONFIG_t vadc_0_group0_init_config;
extern XMC_VADC_GROUP_CONFIG_t vadc_0_group1_init_config;
extern const XMC_VADC_GLOBAL_CONFIG_t vadc_0_config;
extern const XMC_VADC_BACKGROUND_CONFIG_t vadc_0_background_scan_config;
extern const XMC_VADC_QUEUE_ENTRY_t vadc_0_group_0_queue_entries_0;
extern const XMC_VADC_QUEUE_ENTRY_t vadc_0_group_0_queue_entries_1;
extern const XMC_VADC_QUEUE_ENTRY_t vadc_0_group_0_queue_entries_2;
extern const XMC_VADC_QUEUE_ENTRY_t vadc_0_group_0_queue_entries_3;
extern const XMC_VADC_QUEUE_ENTRY_t vadc_0_group_0_queue_entries_4;
extern const XMC_VADC_QUEUE_ENTRY_t vadc_0_group_0_queue_entries_5;
extern const XMC_VADC_QUEUE_ENTRY_t vadc_0_group_0_queue_entries_6;
extern const XMC_VADC_QUEUE_ENTRY_t vadc_0_group_0_queue_entries_7;
extern const XMC_VADC_RESULT_CONFIG_t vadc_0_group_0_result_15_config;
extern const XMC_VADC_RESULT_CONFIG_t vadc_0_group_0_result_9_config;
extern const XMC_VADC_RESULT_CONFIG_t vadc_0_group_0_result_2_config;
extern const XMC_VADC_QUEUE_CONFIG_t vadc_0_group_0_queue_config;
extern const XMC_VADC_GROUP_CLASS_t vadc_0_0_iclass_0;
extern const XMC_VADC_GROUP_CLASS_t vadc_0_0_iclass_1;
extern const XMC_VADC_CHANNEL_CONFIG_t G0_CH1_CURRENT_P2_8_config;
extern const XMC_VADC_CHANNEL_CONFIG_t G0_CH2_CURRENT_U_P2_9_config;
extern const XMC_VADC_CHANNEL_CONFIG_t G0_CH7_TORQUE_P2_2_config;
extern const XMC_VADC_QUEUE_ENTRY_t vadc_0_group_1_queue_entries_0;
extern const XMC_VADC_QUEUE_ENTRY_t vadc_0_group_1_queue_entries_1;
extern const XMC_VADC_QUEUE_ENTRY_t vadc_0_group_1_queue_entries_2;
extern const XMC_VADC_QUEUE_ENTRY_t vadc_0_group_1_queue_entries_3;
extern const XMC_VADC_QUEUE_ENTRY_t vadc_0_group_1_queue_entries_4;
extern const XMC_VADC_QUEUE_ENTRY_t vadc_0_group_1_queue_entries_5;
extern const XMC_VADC_QUEUE_ENTRY_t vadc_0_group_1_queue_entries_6;
extern const XMC_VADC_QUEUE_ENTRY_t vadc_0_group_1_queue_entries_7;
extern const XMC_VADC_RESULT_CONFIG_t vadc_0_group_1_result_15_config;
extern const XMC_VADC_RESULT_CONFIG_t vadc_0_group_1_result_4_config;
extern const XMC_VADC_RESULT_CONFIG_t vadc_0_group_1_result_5_config;
extern const XMC_VADC_QUEUE_CONFIG_t vadc_0_group_1_queue_config;
extern const XMC_VADC_GROUP_CLASS_t vadc_0_1_iclass_0;
extern const XMC_VADC_GROUP_CLASS_t vadc_0_1_iclass_1;
extern const XMC_VADC_CHANNEL_CONFIG_t G1_CH0_CURRENT_P2_8_config;
extern const XMC_VADC_CHANNEL_CONFIG_t G1_CH6_BATTERY_P2_4_config;
extern const XMC_VADC_CHANNEL_CONFIG_t G1_CH7_THROTTLE_P2_5_config;
extern const XMC_WDT_CONFIG_t wdt_0_config;

void init_cycfg_peripherals(void);

#if defined(__cplusplus)
}
#endif /* defined(__cplusplus) */

#endif /* CYCFG_PERIPHERALS_H */
