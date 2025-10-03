/*
* Description: This is based on the open source firmware for TSDZ2 motor made by mbrusa
* It is adapted for TSDZ8 
*
* Related Document: See README.md
*
* Copyright (C) Casainho, Leon, MSpider65 2020.
*
* Released under the GPL License, Version 3
*/
#include "cybsp.h"
#include "cy_utils.h"

#include "cy_retarget_io.h"

#include "main.h"
#if (DEBUG_ON_JLINK == 1)
#include "SEGGER_RTT.h"
#endif 

#include "adc.h"

#if(uCPROBE_GUI_OSCILLOSCOPE == MY_ENABLED)
#include "ProbeScope/probe_scope.h"
#endif

#include "motor.h"
#include "ebike_app.h"
//#include "eeprom.h"


/*******************************************************************************
* Macros
*******************************************************************************/
/* SysTick timer frequency in Hz  */
#define TICKS_PER_SECOND            1000 // 1 tick = 1msec


/*******************************************************************************
* Global Variables
*******************************************************************************/

// Variable for keeping track of time 
/*
volatile uint32_t system_ticks = 0;
uint32_t loop_25ms_ticks = 0;  
uint32_t start = 0 ; // mainly for debugging ; allow to print some variable every e.g. 5 sec
*/
uint16_t last_clock_ticks = 0;  // used to call a function every 25 ms (ebbike controller at 40Hz)
uint16_t last_foc_pid_ticks = 0;    // used to call a function every 10 msec (update foc pid angle at 100hz)
uint16_t last_foc_optimiser_ticks = 0 ; // used to call a function every 200 msec (update of optimizer at 5 hz)
uint16_t last_system_ticks = 0;
volatile uint32_t system_ticks2 = 0;


// maximum duty cycle
//extern uint8_t ui8_pwm_duty_cycle_max; 

// for debugging only at the beginning
uint32_t count = 0;
uint32_t speed = 0; 
uint32_t pas_1 = 0;
uint32_t uart_rx = 0;
uint32_t brake = 0;
uint32_t unknown = 0;


extern volatile uint8_t ui8_received_package_flag ;
extern volatile uint8_t ui8_tx_buffer[];

// for debugging // probably to remove todo
extern volatile uint32_t hall_print_pos ; 
extern volatile uint32_t hall_print_angle ;
extern volatile uint32_t hall_print_pos2; // current hall pattern (after a sampling delay)
extern volatile uint32_t hall_print_interval ; // interval between 2 correct hall transitions
extern volatile uint32_t posif_SR0; 
extern volatile uint32_t posif_SR1;
extern volatile uint32_t posif_print_current_pattern ;

extern volatile uint8_t current_hall_pattern;                   // current hall pattern
extern uint8_t  previous_hall_pattern; 

extern volatile uint16_t ui16_a ;
extern volatile uint16_t ui16_b ;
extern volatile uint16_t ui16_c ;

// to measure time in irq
extern volatile uint16_t debug_time_ccu8_irq0; // to debug time in irq0 CCU8 (should be less than 25usec; 1 = 4 usec )
extern volatile uint16_t debug_time_ccu8_irq1; // to debug time in irq0 CCU8 (should be less than 25usec; 1 = 4 usec )
extern volatile uint16_t debug_time_ccu8_irq1b; // to debug time in irq0 CCU8 (should be less than 25usec; 1 = 4 usec )
extern volatile uint16_t debug_time_ccu8_irq1c; // to debug time in irq0 CCU8 (should be less than 25usec; 1 = 4 usec )
extern volatile uint16_t debug_time_ccu8_irq1d; // to debug time in irq0 CCU8 (should be less than 25usec; 1 = 4 usec )
extern volatile uint16_t debug_time_ccu8_irq1e; // to debug time in irq0 CCU8 (should be less than 25usec; 1 = 4 usec )

extern volatile uint8_t ui8_adc_battery_current_filtered;
extern uint8_t ui8_battery_current_filtered_x10;
extern uint16_t ui16_display_data_factor; 
extern volatile uint8_t ui8_g_foc_angle;
extern uint8_t ui8_throttle_adc_in;

extern volatile uint8_t ui8_best_ref_angles[8] ;
extern uint32_t best_ref_angles_X16bits[8] ;

extern volatile uint16_t ui16_adc_motor_phase_current;
extern volatile uint16_t ui16_adc_motor_phase_current_max;
extern volatile uint16_t ui16_hall_counter_total;
extern volatile uint16_t ui16_adc_voltage;
extern volatile uint16_t ui16_adc_voltage_cut_off;

extern uint8_t hall_reference_angle;
extern uint8_t ui8_wheel_speed_simulate ;  //added by mstrens to simulate a fixed speed whithout having a speed sensor 

extern uint8_t ui8_m_system_state;

/*
// debug manipulating each ref angle and see impact
uint8_t ui8_best_ref_angles1 ;
uint8_t ui8_best_ref_angles2 ;
uint8_t ui8_best_ref_angles3 ;
uint8_t ui8_best_ref_angles4 ;
uint8_t ui8_best_ref_angles5 ;
uint8_t ui8_best_ref_angles6 ;
*/

//debug interval irq0
extern uint16_t interval_ticks ;
extern uint16_t error_ticks_counter ;
extern uint16_t error_ticks_value;
extern uint16_t error_ticks_prev;
extern uint16_t interval_ticks_min; 
extern uint16_t interval_ticks_max; 
extern uint16_t irq0_min ;
extern uint16_t irq0_max ;
extern uint16_t irq1_min ;
extern uint16_t irq1_max ;


/*******************************************************************************
* Function Name: SysTick_Handler
********************************************************************************
* Summary:
* This is the interrupt handler function for the SysTick timer interrupt.
* It counts the time elapsed in milliseconds since the timer started. 
*******************************************************************************/
/*
void SysTick_Handler(void)
{
    system_ticks++;
}
*/

#define CHANNEL_NUMBER_PIN_2_2              (7U) // Torque
#define CHANNEL_NUMBER_PIN_2_3              (5U) // unknown
#define CHANNEL_NUMBER_PIN_2_4              (6U) // Battery
#define CHANNEL_NUMBER_PIN_2_5              (7U) // Throttle
#define CHANNEL_NUMBER_PIN_2_6              (0U) // Vcc
#define CHANNEL_NUMBER_PIN_2_7              (1U) // Unknown

#define CHANNEL_NUMBER_PIN_2_8              (1U) // Current 1 if group 0
#define CHANNEL_NUMBER_PIN_2_9              (2U) // Current U if group 0
#define CHANNEL_NUMBER_PIN_2_10             (3U) // Current v if group 0
#define CHANNEL_NUMBER_PIN_2_11             (4U) // Current W if group 0


// ************* declaration **********
void jlink_print_system_state();


//*******************************************************************************
// Function Name: main
//********************************************************************************

int main(void)


{
    cy_rslt_t result;

    uint32_t wait_time = 1200000;
    while (wait_time > 0){  // wait a little at power on to let VCC be stable and so get probably better ADC conversions
        wait_time--;
    }
    /* Initialize the device and board peripherals */
    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
    
    


    /*
    // fill table used for knowing the best hall pattern positions
    for (uint8_t i = 0; i<8 ; i++) { 
        ui8_best_ref_angles[i] = ui8_hall_ref_angles[i];
        best_ref_angles_X16bits[i] = ui8_hall_ref_angles[i] << 8;
    } // init best value with reference values.
    // debug when changing ref angle used in motor via uc probe
    ui8_best_ref_angles1 = ui8_hall_ref_angles[1];
    ui8_best_ref_angles2 = ui8_hall_ref_angles[2];
    ui8_best_ref_angles3 = ui8_hall_ref_angles[3];
    ui8_best_ref_angles4 = ui8_hall_ref_angles[4];
    ui8_best_ref_angles5 = ui8_hall_ref_angles[5];
    ui8_best_ref_angles6 = ui8_hall_ref_angles[6];
    */

    #if(uCPROBE_GUI_OSCILLOSCOPE == MY_ENABLED)
    ProbeScope_Init(19000); // freq of PWM
    #endif

    #if (DEBUG_ON_JLINK == 1)
    // init segger to allow kind of printf
    SEGGER_RTT_Init ();     
    SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_TRIM);
    SEGGER_RTT_WriteString(0, RTT_CTRL_CLEAR); // clear the RTT terminal
    #endif

    // set the PWM in such a way that PWM are set to passive levels when processor is halted for debugging (safety)
    XMC_CCU8_SetSuspendMode(ccu8_0_HW, XMC_CCU8_SUSPEND_MODE_SAFE_STOP);
    XMC_CCU4_SetSuspendMode(ccu4_0_HW, XMC_CCU4_SUSPEND_MODE_SAFE_STOP);

    /* Initialize printf retarget  when printf on uart is used*/
    //cy_retarget_io_init(CYBSP_DEBUG_UART_HW);

    /* System timer configuration */
    //SysTick_Config(SystemCoreClock / TICKS_PER_SECOND);
    
    // CCU8 slice 3 (IRQ at mid point) generates a SR3 when period match and this trigger a VADC group 0 for queue
    // CCU8 slice 2 (PWM) is configured in device generator to generate a sr2 when ONE match
    //  but device configurator does not allow to setup a second trigger for vadc queue conversion
    // some setup has to be done manually in the group1 queue
     // we have to connect sr2 to vadc group1 queue, to activate trigering and to disable gating.


    // mstrens : we configure vadc with infineon init instead of with bsp
    pmsm_adc_module_init(); 

    /*
    //Here I overwite the config defined in device manage (and generated in cycfg_peripherals.c)     
    const XMC_VADC_QUEUE_CONFIG_t vadc_0_group_0_queue_config2 =
    {
        .conv_start_mode = (uint32_t) XMC_VADC_STARTMODE_WFS,
        .req_src_priority = (uint32_t) XMC_VADC_GROUP_RS_PRIORITY_3,
        .src_specific_result_reg = (uint32_t) 0,
        .trigger_signal = (uint32_t) XMC_VADC_REQ_TR_J, // XMC_VADC_REQ_TR_J = CCU8 SR3 = mid point , // XMC_VADC_REQ_TR_P, // use gate set up
        .trigger_edge = (uint32_t) XMC_VADC_TRIGGER_EDGE_ANY,
        .gate_signal = (uint32_t) XMC_VADC_REQ_GT_E, //use CCU8_ST3A = when timer is at mid period counting up
        .timer_mode = (uint32_t) false,
        .external_trigger = (uint32_t) true,
    };
    XMC_VADC_GROUP_QueueSetGatingMode(vadc_0_group_0_HW, (XMC_VADC_GATEMODE_t) XMC_VADC_GATEMODE_IGNORE);
    XMC_VADC_GROUP_QueueInit(vadc_0_group_0_HW, &vadc_0_group_0_queue_config2);

    const XMC_VADC_QUEUE_CONFIG_t vadc_0_group_1_queue_config2 = {
            .conv_start_mode = (uint32_t) XMC_VADC_STARTMODE_WFS,
            .req_src_priority = (uint32_t) XMC_VADC_GROUP_RS_PRIORITY_2,
            .src_specific_result_reg = (uint32_t) 0,
            .trigger_signal = (uint32_t) XMC_VADC_REQ_TR_I,  //XMC_VADC_REQ_TR_I = CCU8 SR2 = ONe match   // XMC_VADC_REQ_TR_P,  // use gate set up
            .trigger_edge = (uint32_t) XMC_VADC_TRIGGER_EDGE_ANY,
            .gate_signal = (uint32_t) XMC_VADC_REQ_GT_E, ////use CCU8_ST3A = when timer is at mid period counting up
            .timer_mode = (uint32_t) false,
            .external_trigger = (uint32_t) true,
        };
    XMC_VADC_GROUP_QueueSetGatingMode(vadc_0_group_1_HW, (XMC_VADC_GATEMODE_t) XMC_VADC_GATEMODE_IGNORE);
    XMC_VADC_GROUP_QueueInit(vadc_0_group_1_HW, &vadc_0_group_1_queue_config2);
    */

    /* Start the temperature measurement */
    XMC_SCU_StartTempMeasurement();

    // **** load the config from flash
    //init_extra_fields_config (); // get the user parameters from flash
    // todo : change when eeprom is coded properly add some initialisation (e.g. m_configuration_init() and ebike_app.init)
    // currently it is filled with parameters from user setup + some dummy values (e.g. for soc)
    //m_configuration_init();
    // add some initialisation in ebike_app.init
    //ebike_app_init();
    // added by Mstrens
	hall_reference_angle =  (uint8_t) DEFAULT_HALL_REFERENCE_ANGLE; //to do = add a small offset like in non 860c version
	//hall_reference_angle = 66;
	ui8_wheel_speed_simulate =  WHEEL_SPEED_SIMULATE; // load wheel speed simulate (so allow to change it with uc-probe)

    //XMC_WDT_Service();
    // set initial position of hall sensor and first next expected one in shadow and load immediately in real register
    //posif_init_position();
    get_hall_pattern();
    previous_hall_pattern = 0; // use a different hall pattern to force the angle. 
    XMC_POSIF_Start(HALL_POSIF_HW);
    
    
    //XMC_WDT_Service();
    wait_time = 120000; // on more delay before starting the IRQ
    while (wait_time > 0){  // wait a little at power on to let VCC be stable and so get probably better ADC conversions
        wait_time--;
    }
    //XMC_WDT_Service();
    

        // set interrupt when current exceed 256 (10 bits) = 1024 (12 bits) = boundaries set in modus
    // link event on group 0 channel 1 to group specific interrupt
    // mstrens : this part has not yet been updated for infineon init
    //XMC_VADC_GROUP_ChannelSetEventInterruptNode(vadc_0_group_0_HW, 1, XMC_VADC_SR_GROUP_SR0 );
    //NVIC_SetPriority(VADC0_G0_0_IRQn,0); // interrupt for group specific event G0 SR0
    //NVIC_EnableIRQ(VADC0_G0_0_IRQn);    
    
    // set interrupt 
//    NVIC_SetPriority(CCU40_1_IRQn, 0U); //capture hall pattern and slice 2 time when a hall change occurs
//	NVIC_EnableIRQ(CCU40_1_IRQn);
    // set irq triggered by posif when a pattern changes
    #if (USE_IRQ_FOR_HALL == (1))    
    NVIC_SetPriority(POSIF0_0_IRQn,0);
    NVIC_EnableIRQ(POSIF0_0_IRQn);     
    #endif
    /* CCU80_0_IRQn and CCU80_1_IRQn. slice 3 interrupt on counting up and down. at 19 khz to manage rotating flux*/
	NVIC_SetPriority(CCU80_0_IRQn, 1U);
	NVIC_EnableIRQ(CCU80_0_IRQn);
    NVIC_SetPriority(CCU80_1_IRQn, 1U);
	NVIC_EnableIRQ(CCU80_1_IRQn);

    //added to test the same startup as infineon example
    #define XMC_CCU8_GIDLC_CLOCK_MASK (15U) // start the 4 slice simultanously
	XMC_CCU8_EnableMultipleClocks(ccu8_0_HW , XMC_CCU8_GIDLC_CLOCK_MASK);

    XMC_SCU_SetCcuTriggerHigh(SCU_GENERAL_CCUCON_GSC80_Msk); // I had to use the High instead of LOw
    // Clear idle bit slice 0,1,2,3 for CCU80 Kernel  // MASK = 1111 =the 4 slices
    

    
    // Enable Global Start Control CCU80  in a synchronized way
    //XMC_SCU_SetCcuTriggerHigh(SCU_GENERAL_CCUCON_GSC80_Msk);
    
    //XMC_SCU_SetCcuTriggerLow(SCU_GENERAL_CCUCON_GSC80_Msk);
    /*
    uint32_t retry_start_counter = 10;
    while ((!XMC_CCU8_SLICE_IsTimerRunning(PHASE_U_TIMER_HW)) && (retry_start_counter > 0)){ // to be sure it is running
        XMC_SCU_SetCcuTriggerHigh(SCU_GENERAL_CCUCON_GSC80_Msk);
        XMC_SCU_SetCcuTriggerLow(SCU_GENERAL_CCUCON_GSC80_Msk);
    
        //SEGGER_RTT_printf(0, "Retry CCU8 start; still %u try\r\n", retry_counter);
    }
    */

    //XMC_VADC_GLOBAL_EnablePostCalibration(vadc_0_HW, 0U);
    //XMC_VADC_GLOBAL_EnablePostCalibration(vadc_0_HW, 1U);
    //XMC_VADC_GLOBAL_StartupCalibration(vadc_0_HW);
   
   
   //XMC_WDT_Service();
    wait_time = 120000; // on more delay before starting the IRQ
    while (wait_time > 0){  // wait a little at power on to let VCC be stable and so get probably better ADC conversions
        wait_time--;
    }
    
    //start = system_ticks;
   XMC_WDT_Start();
   XMC_WDT_Service();

   // init the clock timer
   last_clock_ticks = XMC_CCU4_SLICE_GetTimerValue(HALL_SPEED_TIMER_HW) ; 
   last_system_ticks = last_clock_ticks;
//***************************** while ************************************
    while (1) // main loop
    {     
	    // avoid a reset
        XMC_WDT_Service(); // reset if we do not run here within the 0,5 sec
    
        // when there is no frame waiting for process (ui8_received_package_flag == 0), try to get incoming data from UART but do not process them
        // When frame is full, data are processed in ebike_app.c once every 100 msec
        if (ui8_received_package_flag == 0) {
            fillRxBuffer();
        }
        // must be activated for real production
        // Here we should call a funtion every 25 msec (based on systick or on an interrupt based on a CCU4 timer)
        /*
        if ((system_ticks - loop_25ms_ticks) > 25) { 
            loop_25ms_ticks = system_ticks;
            ebike_app_controller();  // this performs some checks and update some variable every 25 msec
        }
        */
       
        // avoid a reset
        XMC_WDT_Service(); // reset if we do not run here within the 0,5 sec
        #if (USE_SPIDER_LOGIC_FOR_TORQUE > (0)) 
        if (ui8_pas_new_transition) {
            new_torque_sample();
        }
        #endif
        // Here we should call a funtion every 25 msec (based on systick or on an interrupt based on a CCU4 timer)
        uint16_t temp_ticks = XMC_CCU4_SLICE_GetTimerValue(HALL_SPEED_TIMER_HW);
        if (temp_ticks < last_system_ticks) { // once every 65536 * 4 usec = about 0,25 sec
            system_ticks2++; // add 1 every 0,25 sec (about)
            last_system_ticks = temp_ticks;
        }
        
        uint16_t temp_interval;
        #if (DYNAMIC_LEAD_ANGLE == (1))
        temp_interval = temp_ticks - last_foc_pid_ticks ; 
        if ( temp_interval > 2500){ // 100hz : interval 10000 usec / 4usec = 2500 ticks
            last_foc_pid_ticks = temp_ticks;
            update_foc_pid();  // this calculate a new FOC angle based on a PI and on the Id current
        }
        #endif

        temp_interval = temp_ticks - last_clock_ticks;
        //if ( (uint16_t) ((uint16_t) temp_ticks - (uint16_t) last_clock_ticks) > (uint16_t) 6250){ // 25000 usec / 4usec = 6250
        if ( temp_interval > 6250){ // 25000 usec / 4usec = 6250
            last_clock_ticks = temp_ticks;
            ebike_app_controller();  // this performs some checks and update some variable every 25 msec
        }
        
        #if (DYNAMIC_LEAD_ANGLE == (1))
        temp_interval = temp_ticks - last_foc_optimiser_ticks;
        //if ( (uint16_t) ((uint16_t) temp_ticks - (uint16_t) last_clock_ticks) > (uint16_t) 6250){ // 25000 usec / 4usec = 6250
        if ( temp_interval > 50000){ // 200000 usec / 4usec = 50000
            last_foc_optimiser_ticks = temp_ticks;
            update_foc_optimiser();  // this performs some checks and update some variable every 25 msec
        }
        #endif        

        
        #if (uCPROBE_GUI_OSCILLOSCOPE == MY_ENABLED)
        //ProbeScope_Sampling(); // this should be moved e.g. in a interrupt that run faster
        #endif
        
        // for debug
        // if (take_action_250ms(1,4))debug_time_ccu8_irq0 = 0;
//        if( take_action(1,1000)) debug_time_ccu8_irq0 = 0;

        #if (DEBUG_ON_JLINK == 1)
        if (take_action_250ms(3,4)){
        //if(take_action(3,1000)){
//            uint32_t vadc_group0_event = XMC_VADC_GROUP_ChannelGetAssertedEvents(vadc_0_group_0_HW );
            //SEGGER_RTT_printf(0, "init_state %x   status %X\r\n", ui8_m_motor_init_state , ui8_m_motor_init_status);
            /*
            uint32_t is_running_0 = XMC_CCU8_SLICE_IsTimerRunning	(	PHASE_U_TIMER_HW	)	;
            uint32_t is_running_1 = XMC_CCU8_SLICE_IsTimerRunning	(	PHASE_V_TIMER_HW	)	;
            uint32_t is_running_2 = XMC_CCU8_SLICE_IsTimerRunning	(	PHASE_W_TIMER_HW	)	;
            uint32_t is_running_3 = XMC_CCU8_SLICE_IsTimerRunning	(	PWM_IRQ_TIMER_HW	)	;  
            uint32_t tim_0 = XMC_CCU8_SLICE_GetTimerValue(PHASE_U_TIMER_HW);
            uint32_t tim_1 = XMC_CCU8_SLICE_GetTimerValue(PHASE_V_TIMER_HW);
            uint32_t tim_2 = XMC_CCU8_SLICE_GetTimerValue(PHASE_W_TIMER_HW);
            uint32_t tim_3 = XMC_CCU8_SLICE_GetTimerValue(PWM_IRQ_TIMER_HW);
            SEGGER_RTT_printf(0, "running %u %u %u %u %u %u %u %u \r\n", is_running_0 , is_running_1 , is_running_2 , is_running_3, 
                tim_0 , tim_1 ,tim_2 ,tim_3);
            */
           /*
            for (uint32_t i = 0; i<=28 ; i++){
                if (NVIC_GetEnableIRQ((IRQn_Type) i)) {
                    SEGGER_RTT_printf (0, "irq on = %u\r\n", i);
                }
           }
           */     
               SEGGER_RTT_printf(0, "error ticks = %u %u %u %u %u %u %u%\r\n", error_ticks_counter, 
                           interval_ticks_min , interval_ticks_max , irq0_min , irq0_max , irq1_min , irq1_max);
                           
        }
        #endif    
        #if (DEBUG_ON_JLINK == 1)
         // do debug if communication with display is working
         //if( take_action_250ms(1, 1)) SEGGER_RTT_printf(0,"Light is= %u\r\n", (unsigned int) ui8_lights_button_flag);
        
         //if( take_action(1, 250)) SEGGER_RTT_printf(0,"Light is= %u\r\n", (unsigned int) ui8_lights_button_flag);
        if (ui8_m_system_state) { // print a message when there is an error detected
            if( take_action_250ms(1,1)) jlink_print_system_state();
            //if( take_action(1,200)) jlink_print_system_state();
        }

//        if( take_action(2, 500)) SEGGER_RTT_printf(0,"Adc current= %u adcX8=%u  current_Ax10=%u  factor=%u\r\n", 
//            (unsigned int) ui8_adc_battery_current_filtered ,
//            (unsigned int) ui16_adc_battery_current_acc_X8 ,
//            (unsigned int) ui8_battery_current_filtered_x10 , 
//            (unsigned int) ui16_display_data_factor
//            );
        // monitor duty cycle, current when motor is running
        /*
        //if( take_action(3, 1000)) SEGGER_RTT_printf(0,"dctarg=%u dc=%u    ctarg=%u cfilt=%u Throttle=%u  erps=%u foc%u\r\n",
        if( take_action_250ms(3, 4)) SEGGER_RTT_printf(0,"dctarg=%u dc=%u    ctarg=%u cfilt=%u Throttle=%u  erps=%u foc%u\r\n",
        
            (unsigned int) ui8_controller_duty_cycle_target,
            (unsigned int) ui8_g_duty_cycle,
            (unsigned int) ui8_controller_adc_battery_current_target,
            (unsigned int) ui8_adc_battery_current_filtered,
            (unsigned int) ui8_throttle_adc_in,
            (unsigned int) ui16_motor_speed_erps,
            (unsigned int) ui8_g_foc_angle
            //(unsigned int) XMC_CCU8_SLICE_IsTimerRunning(PHASE_U_TIMER_HW),
            //(unsigned int) ui8_motor_enabled
        );
        */

        #define DEBUG_HALL_POSITIONS (1)
        #if (DEBUG_HALL_POSITIONS == (1) )
        if( take_action_250ms(6, 20)) {
        //if( take_action(6, 5000)) {
            SEGGER_RTT_printf(0,
            "c10b=%u  dc=%u erps=%u t360=%u  best1=%u best2=%u best3=%u best4=%u best5=%u best6=%u\r\n",
                (unsigned int) ui8_adc_battery_current_filtered,
                (unsigned int) ui8_g_duty_cycle,
                (unsigned int) ui16_motor_speed_erps,
                (unsigned int) ui16_hall_counter_total,
                ui8_best_ref_angles[1], ui8_best_ref_angles[2], ui8_best_ref_angles[3], ui8_best_ref_angles[4], ui8_best_ref_angles[5], ui8_best_ref_angles[6]
            );
        }
        #endif
        #endif    // end DEBUG_ON_JLINK
       
    } // end while main loop
}

// Error state (changed)
// NO_ERROR                                0			// "None"
// ERROR_NOT_INIT                          1			// "Motor not init"
// ERROR_TORQUE_SENSOR                     (1 << 1)	// "Torque Fault"
// ERROR_CADENCE_SENSOR		    		(1 << 2)	// "Cadence fault"
// ERROR_MOTOR_BLOCKED     				(1 << 3)	// "Motor Blocked"
// ERROR_THROTTLE						 	(1 << 4)	// "Throttle Fault"
// ERROR_FATAL                             (1 << 5)	// "Fatal error"
// ERROR_BATTERY_OVERCURRENT               (1 << 6)	// "Overcurrent"
// ERROR_SPEED_SENSOR	                    (1 << 7)	// "Speed fault"

#if (DEBUG_ON_JLINK == 1)
void jlink_print_system_state(){
    if (ui8_m_system_state & ERROR_NOT_INIT) SEGGER_RTT_printf(0,"Error : not init\r\n");
    if (ui8_m_system_state & ERROR_TORQUE_SENSOR) SEGGER_RTT_printf(0,"Error : torque sensor\r\n");
    if (ui8_m_system_state & ERROR_CADENCE_SENSOR) SEGGER_RTT_printf(0,"Error : cadence sensor\r\n");
    if (ui8_m_system_state & ERROR_MOTOR_BLOCKED) SEGGER_RTT_printf(0,"Error : motor blocked\r\n");
    if (ui8_m_system_state & ERROR_THROTTLE) SEGGER_RTT_printf(0,"Error : throttle\r\n");
    if (ui8_m_system_state & ERROR_FATAL) SEGGER_RTT_printf(0,"Error : fatal\r\n");
    if (ui8_m_system_state & ERROR_BATTERY_OVERCURRENT) SEGGER_RTT_printf(0,"Error : battery_overcurrent\r\n");
    if (ui8_m_system_state & ERROR_SPEED_SENSOR) SEGGER_RTT_printf(0,"Error : speed sensor\r\n");
}    
#endif
/* END OF FILE */
