/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, Leon, MSpider65 2021.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _MAIN_H_
#define _MAIN_H_

#include "config.h"
#include "common.h"

#define VERSION "0.0.0"

#define DEBUG_ON_JLINK         (1)  // when 1, messages are generated on jlink; best is to connect only 3 wires (grnd + SWO and S???)


#define NORMAL_OPERATIONS             0
#define DETECT_HALL_SENSORS_POSITIONS 1 // rotate the motor in both directions very slowly and performs some calculations
#define FIND_BEST_GLOBAL_HALL_OFFSET  2 // rotate the motor with progressive values of hall offset to find the best one
#define TEST_WITH_FIXED_DUTY_CYCLE    3 // rotate the motor with fixed duty cycle and a max current target
#define TEST_WITH_THROTTLE            4 // rotate the motor with a duty cycle based on throttle
// this define selects the main way the firmware is used; select one of the 5 options here above
#define PROCESS  NORMAL_OPERATIONS  

#define USE_CONFIG_FROM_COMPILATION (0)  // this should normally be set on 0; set to 1 only if you want to give priority to
                                         // the parameters defined in config.h and used for compilation
                                         // this can be convenient for testing/debugging

// ***************** setup to detect hall sensors positions -- to be done only once for a new motor
    // There may not be a load on the motor
    // we rotate the motor in both directions and report via jlink the angles that provides a change of the hall pattern
    // For each hall pattern (1...6), then we have to calculate (e.g. in xls) the average of the angles from forward and reverse 
    // those averages are then strored in a table
    // Note: after the rotations in both directions the program stops (infinite loop) and PWM are disabled.
    // We never call the main loop during this test
    // during this test, do not forget to enable debug messages on Jlink ; #define DEBUG_ON_JLINK (1)
// 0 is the code for production (run the motor detecting hall pattern changes and adapating flux for rotation)
#define DUTY_CYLE_CAL_HALL_SENSORS_POSITIONS_1 30 // 255 is 100% and is unsecure; max seems to be 35 with delay 5 ; 30 is good for 36V
#define WAIT_TIME_MS_CAL_HALL_SENSORS_POSITIONS_1 5 // could be e.g. up to 10 when duty cycle is lower
#define DUTY_CYLE_CAL_HALL_SENSORS_POSITIONS_2 20 // max was 40 with wait = 1 ; 20 is good with 36V
#define WAIT_TIME_MS_CAL_HALL_SENSORS_POSITIONS_2 5 // it was first tested with 10
#define MAX_CURRENT_A_CAL_HALL_SENSORS_POSITIONS 6  // max current in A that is allowed during this test (6 seems more than eneough)
#define NUMBER_OF_ROTATION_CAL_SENSORS_POSITIONS 16  // number of electrical rotations to do to get average (if magnets are OK there are no big differences)
// Note!!!! using to high duty cycles and/or wait time and or max current or rotation could damage the motor/controller (heating)

// ************* Find best global hall offset
//   this let the motor runs with different hall offset values and display for each the average current
//   this allows to find the best offset and to fill it in global_offset_angle
#define PWM_DUTY_CYCLE_MAX_FIND_BEST_GLOBAL_HALL_OFFSET 20 // max value during this process
#define ADC_BATTERY_CURRENT_TARGET_FIND_BEST_GLOBAL_HALL_OFFSET 6 // 1 ADC step = 0,16A ; so 6 = 1A
#define FIRST_OFFSET_ANGLE_FOR_CALIBRATION (67) // This is the first value used for calibration; it increases every 4 sec up to the max
#define LAST_OFFSET_ANGLE_FOR_CALIBRATION (FIRST_OFFSET_ANGLE_FOR_CALIBRATION+8) // this is the max value to be tested; select a value that avoid increasing to much the current
#define CALIBRATE_OFFSET_STEP 1    // 1// step used when increasing the global_offset_angle (normally 1; could be set to 0 for some kind of test)

// ************* TEST WITH FIXED_DUTY_CYCLE *****************
#define PWM_DUTY_CYCLE_MAX_TEST_WITH_FIXED_DUTY_CYCLE 20 // max is 254 !!!
#define ADC_BATTERY_CURRENT_TARGET_TEST_WITH_FIXED_DUTY_CYCLE (6) // 1 adc step = 0,16A; so 6 = 1A
        
// ************* TEST WITH THROTTLE *****************
// pwm_duty_cycle = 255 (set in ebike_app.c)
#define ADC_BATTERY_CURRENT_TARGET_TEST_WITH_THROTTLE (40)// 1 adc step = 0,16A; so 6 = 1A

// *************** from here we have more general parameters 

// this is the value provided by the find best hall sensor offset process
// it is used for NORMAL_OPERATION, TEST_WITH_FIXED_DUTY_CYCLE and TEST_WITH_TROTTLE
#define CALIBRATED_OFFSET_ANGLE 70   

// for CCU4 slice 2
#define HALL_COUNTER_FREQ                       250000U // 250KHz or 4us

#define PWM_DUTY_CYCLE_MAX						254
#define PWM_DUTY_CYCLE_MAX_NORMAL_OPERATIONS	254   // it is normally 255 but this seems to high because we exceed the max timer value        
#define PWM_DUTY_CYCLE_STARTUP	                30    // Initial PWM Duty Cycle at motor startup


// ----------------------------------------------------------------------------------------------------------------
// PWM related values (for 19 kHz)
#define PWM_COUNTER_MAX					1680 // at 64 Mz // at 16Mhz from TSDZ2, it was 420 // 16MHz / 840 = 19,047 KHz                                        107
#define MIDDLE_SVM_TABLE			(PWM_COUNTER_MAX/2)   // in TSDZ2, it was 107 because svm table was uint8_t and had a max of 215.
                                                                        //TSDZ8 uses a table in uint16_t so it can be 2* higher
// wheel speed parameters
#define OEM_WHEEL_SPEED_DIVISOR			384 // at 19 KHz

#define PWM_CYCLES_SECOND			(64000000/(PWM_COUNTER_MAX*2)) // 55.5us (PWM period) 18 Khz // for TSDZ2, it was 16000000

/*---------------------------------------------------------
 NOTE: regarding duty cycle (PWM) ramping

 Do not change these values if not sure of the effects!

 A lower value of the duty cycle inverse step will mean
 a faster acceleration. Be careful not to choose too
 low values for acceleration.
 ---------------------------------------------------------*/
// ramp up/down PWM cycles count
#define PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT			(uint8_t)(PWM_CYCLES_SECOND/98)  // 194
#define PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN				(uint8_t)(PWM_CYCLES_SECOND/781) // 24
#define PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT		(uint8_t)(PWM_CYCLES_SECOND/260) // 73
#define PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN			(uint8_t)(PWM_CYCLES_SECOND/1953) //9
#define CRUISE_DUTY_CYCLE_RAMP_UP_INVERSE_STEP				(uint8_t)(PWM_CYCLES_SECOND/78)  // 244
#define WALK_ASSIST_DUTY_CYCLE_RAMP_UP_INVERSE_STEP			(uint8_t)(PWM_CYCLES_SECOND/78)  // 244
#define THROTTLE_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT	(uint8_t)(PWM_CYCLES_SECOND/78)  // 244
#define THROTTLE_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN		(uint8_t)(PWM_CYCLES_SECOND/390) //48

// motor max speed | for TSDZ2 29 points for the sinewave at max speed (less than PWM_CYCLES_SECOND/29)
//#define MOTOR_OVER_SPEED_ERPS	((PWM_CYCLES_SECOND/29) < 650 ?  (PWM_CYCLES_SECOND/29) : 650) 
// 19000 cycles/sec /29 = 656 cycle/sec ; 1 cycle is 55 usec ; so 1 electric rotation = 29*55 = 1595 usec
//  so for TSDZ2 one rotation takes 1595 *8 = 12760 usec ; so 78 rps = 4700 rpm
//  for the same rpm, tsdz8 can has 2 more ticks                                            
#define MOTOR_OVER_SPEED_ERPS	1300 

// for TSDZ2
//#define MOTOR_SPEED_FIELD_WEAKENING_MIN			490 // 90 rpm
//#define ERPS_SPEED_OF_MOTOR_REENABLING				320 // 60 rpm
//For TSDZ8, I expect that there must 2 * more ticks to reach the same mecanical speed
#define MOTOR_SPEED_FIELD_WEAKENING_MIN				980 // 90 rpm
#define ERPS_SPEED_OF_MOTOR_REENABLING				640 // 60 rpm


#define MOTOR_SPEED_FIELD_WEAKEANING_MIN				980
// foc angle multiplier
// 48 volt motor
#define FOC_ANGLE_MULTIPLIER					39   // 39 for TSDZ2 48V, still to check for TSDZ8 because it depends on L


// cadence
#define CADENCE_SENSOR_CALC_COUNTER_MIN                         (uint16_t)((uint32_t)PWM_CYCLES_SECOND*100U/446U)  // 3500 at 15.625KHz ; 4270 at 19kHz
#define CADENCE_SENSOR_TICKS_COUNTER_MIN_AT_SPEED               (uint16_t)((uint32_t)PWM_CYCLES_SECOND*10U/558U)   // 280 at 15.625KHz ; 341 at 19 khz
#define CADENCE_TICKS_STARTUP                                   (uint16_t)((uint32_t)PWM_CYCLES_SECOND*10U/25U)  // 7619 ui16_cadence_sensor_ticks value for startup. About 7-8 RPM (6250 at 15.625KHz)
//#define CADENCE_SENSOR_STANDARD_MODE_SCHMITT_TRIGGER_THRESHOLD	(uint16_t)((uint32_t)PWM_CYCLES_SECOND*10U/446U)	// software based Schmitt trigger to stop motor jitter when at resolution limits (350 at 15.625KHz)
#define CADENCE_SENSOR_STANDARD_MODE_SCHMITT_TRIGGER_THRESHOLD	(uint16_t)((uint32_t)PWM_CYCLES_SECOND*10U/892U)	// Quick stop value as version 4.4


// Wheel speed sensor
#define WHEEL_SPEED_SENSOR_TICKS_COUNTER_MAX			(uint16_t)((uint32_t)PWM_CYCLES_SECOND*10U/1157U)   // 164 at 19 khz (135 at 15,625KHz) something like 200 m/h with a 6'' wheel
#define WHEEL_SPEED_SENSOR_TICKS_COUNTER_MIN			(uint16_t)((uint32_t)PWM_CYCLES_SECOND*1000U/477U) // 32767@15625KHz could be a bigger number but will make for a slow detection of stopped wheel speed
#define WHEEL_SPEED_SENSOR_SIMULATION					0



// ----------------------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------

/* Hall Sensors NOTE! - results after Hall sensor calibration experiment
Dai test sulla calibrazione dei sensori Hall risulta che Trise - Tfall = 21 e cioè 84 us
(1 Hall counter step = 4us).
Quindi gli stati 6,3,5 (fronte di salita) vengono rilevati con un ritardo di 84us maggiore
rispetto agli stati 2,1,4.
Quindi per gli stati 6,3,5 va sommato 21 (21x4us=84us) al contatore Hall usato per l'interpolazione,
visto che è partito con 84us di ritardo rispetto agli altri stati.
In questo modo il contatore Hall viene allineato allo stesso modo per tutti gli stati, ma sarà
comunque in ritardo di Tfall per tutti gli stati. Questo ritardo viene gestito con un ulteriore
offset da sommare al contatore per tutti gli stati.
Dai test effettuati risulta che Tfall vale circa 66us (16,5 step) a cui va sommato il ritardo fra							   
la lettura del contatore Hall e la scrittura dei registri PWM che è sempre uguale a mezzo
ciclo PWM (1/(19047*2) = 26,25us o 6,5 step).
Quindi l'offset per gli stati 2,1,4 vale 23 (16,5+6,5) mentre per gli stati 6,3,5
vale 44 (16,5+6,5+21).
I test effettuati hanno inoltre calcolato che il riferimento angolare corretto non è 10 ma 4 step
// traduction du commentaire de TSDZ2
D'après les tests d'étalonnage du capteur Hall, il s'avère que Trise - Tfall = 21 et cela fait 84 µs
(1 pas de compteur Hall = 4us).
Ainsi, les états 6, 3, 5 (front montant) sont détectés avec un délai de 84 µs plus long
par rapport aux états 2,1,4.
Donc pour les états 6, 3, 5, vous devez ajouter 21 (21x4us=84us) au compteur Hall utilisé pour l'interpolation,
car il a commencé 84us plus tard que les autres états.
De cette façon, le compteur Hall est aligné de la même manière pour tous les États, mais il sera
toujours en retard pour tous les états. Ce retard est traité avec un supplément
décalage à ajouter au compteur pour tous les états.
D'après les tests effectués, il apparaît que Tfall est d'environ 66us (16,5 pas) auquel il faut ajouter le délai entre
lire le compteur Hall et écrire les registres PWM qui sont toujours égaux à la moitié
Cycle PWM (1/(19047*2) = 26,25 µs ou 6,5 étapes).
Ainsi, le décalage pour les états 2, 1, 4 est de 23 (16,5+6,5) tandis que pour les états 6, 3, 5
vaut 44 (16,5+6,5+21).
Les tests effectués ont également permis de calculer que la référence angulaire correcte n'est pas de 10 mais de 4 pas
***************************************
Test effettuato il 21/1/2021
MOTOR_ROTOR_OFFSET_ANGLE:  10 -> 4
HALL_COUNTER_OFFSET_DOWN:  8  -> 23
HALL_COUNTER_OFFSET_UP:    29 -> 44
****************************************
*/

#define HALL_COUNTER_OFFSET_DOWN                (HALL_COUNTER_FREQ/PWM_CYCLES_SECOND/2 + 17)
#define HALL_COUNTER_OFFSET_UP                  (HALL_COUNTER_OFFSET_DOWN + 21)
#define FW_HALL_COUNTER_OFFSET_MAX              5 // 5*4=20us max time offset

#define MOTOR_ROTOR_INTERPOLATION_MIN_ERPS      20 // it was 10 for tsdz2 that used 8 poles; tsdz8 uses 4 poles so it takes 2* more ticks

// set on the display with motor type
#define FOC_ANGLE_MULTIPLIER_36V				30 // 36 volt motor
#define FOC_ANGLE_MULTIPLIER_48V				39 // 48 volt motor

// adc torque offset gap value for error
#define ADC_TORQUE_SENSOR_OFFSET_THRESHOLD		30

// Torque sensor values
#define ADC_TORQUE_SENSOR_OFFSET_DEFAULT		150 // from 860c
// adc torque range parameters for remapping
#define ADC_TORQUE_SENSOR_RANGE_TARGET	  		160 // from 860c
#define ADC_TORQUE_SENSOR_RANGE_TARGET_MIN 		133 // from 860c
#
#define ADC_TORQUE_SENSOR_ANGLE_COEFF			11
#define ADC_TORQUE_SENSOR_ANGLE_COEFF_X10		(uint16_t)(ADC_TORQUE_SENSOR_ANGLE_COEFF * 10)



/*
// Torque sensor range values
#define ADC_TORQUE_SENSOR_RANGE				(uint16_t)(PEDAL_TORQUE_ADC_MAX - PEDAL_TORQUE_ADC_OFFSET)
#define ADC_TORQUE_SENSOR_RANGE_TARGET	  		160

// Torque sensor offset values
#if TORQUE_SENSOR_CALIBRATED
#define ADC_TORQUE_SENSOR_CALIBRATION_OFFSET    (uint16_t)(((6 * ADC_TORQUE_SENSOR_RANGE) / ADC_TORQUE_SENSOR_RANGE_TARGET) + 1)
#define ADC_TORQUE_SENSOR_MIDDLE_OFFSET_ADJ		(uint16_t)(((20 * ADC_TORQUE_SENSOR_RANGE) / ADC_TORQUE_SENSOR_RANGE_TARGET) + 1)
#define ADC_TORQUE_SENSOR_OFFSET_ADJ			(uint16_t)(((PEDAL_TORQUE_ADC_OFFSET_ADJ * ADC_TORQUE_SENSOR_RANGE) / ADC_TORQUE_SENSOR_RANGE_TARGET) + 1)
#else
#define ADC_TORQUE_SENSOR_CALIBRATION_OFFSET    6
#define ADC_TORQUE_SENSOR_MIDDLE_OFFSET_ADJ		20
#define ADC_TORQUE_SENSOR_OFFSET_ADJ			PEDAL_TORQUE_ADC_OFFSET_ADJ
#endif

// adc torque range parameters for remapping
#define ADC_TORQUE_SENSOR_DELTA_ADJ			(uint16_t)((ADC_TORQUE_SENSOR_MIDDLE_OFFSET_ADJ * 2) - ADC_TORQUE_SENSOR_CALIBRATION_OFFSET - ADC_TORQUE_SENSOR_OFFSET_ADJ)
#define ADC_TORQUE_SENSOR_RANGE_INGREASE_X100   	(uint16_t)((ADC_TORQUE_SENSOR_RANGE_TARGET * 50) / ADC_TORQUE_SENSOR_RANGE)
#define ADC_TORQUE_SENSOR_ANGLE_COEFF			11
#define ADC_TORQUE_SENSOR_ANGLE_COEFF_X10		(uint16_t)(ADC_TORQUE_SENSOR_ANGLE_COEFF * 10)

#define ADC_TORQUE_SENSOR_RANGE_TARGET_MIN 		(uint16_t)((float)((ADC_TORQUE_SENSOR_RANGE_TARGET / 2) \
* (((ADC_TORQUE_SENSOR_RANGE_TARGET / 2) / ADC_TORQUE_SENSOR_ANGLE_COEFF + ADC_TORQUE_SENSOR_ANGLE_COEFF) / ADC_TORQUE_SENSOR_ANGLE_COEFF)))

#define ADC_TORQUE_SENSOR_RANGE_TARGET_MAX 		(uint16_t)((ADC_TORQUE_SENSOR_RANGE_TARGET_MIN * (100 + PEDAL_TORQUE_ADC_RANGE_ADJ)) / 100)

// parameters of the adc torque step for human power calculation
#define PEDAL_TORQUE_PER_10_BIT_ADC_STEP_BASE_X100	34 // base adc step for remapping
#define WEIGHT_ON_PEDAL_FOR_STEP_CALIBRATION		24 // Kg
#define PERCENT_TORQUE_SENSOR_RANGE_WITH_WEIGHT		75 // % of torque sensor range with weight
#define ADC_TORQUE_SENSOR_TARGET_WITH_WEIGHT		(uint16_t)((ADC_TORQUE_SENSOR_RANGE_TARGET * PERCENT_TORQUE_SENSOR_RANGE_WITH_WEIGHT) / 100)

#define ADC_TORQUE_SENSOR_DELTA_WITH_WEIGHT		(uint16_t)(((((ADC_TORQUE_SENSOR_TARGET_WITH_WEIGHT \
* ADC_TORQUE_SENSOR_RANGE_TARGET_MIN) / ADC_TORQUE_SENSOR_RANGE_TARGET)	* (100 + PEDAL_TORQUE_ADC_RANGE_ADJ) / 100) \
* (ADC_TORQUE_SENSOR_TARGET_WITH_WEIGHT - ADC_TORQUE_SENSOR_CALIBRATION_OFFSET + ADC_TORQUE_SENSOR_OFFSET_ADJ \
- ((ADC_TORQUE_SENSOR_DELTA_ADJ * ADC_TORQUE_SENSOR_TARGET_WITH_WEIGHT) / ADC_TORQUE_SENSOR_RANGE_TARGET))) / ADC_TORQUE_SENSOR_TARGET_WITH_WEIGHT)

#define PEDAL_TORQUE_PER_10_BIT_ADC_STEP_CALC_X100	(uint8_t)((uint16_t)(((WEIGHT_ON_PEDAL_FOR_STEP_CALIBRATION * 167) \
/ ((ADC_TORQUE_SENSOR_DELTA_WITH_WEIGHT * ADC_TORQUE_SENSOR_RANGE_TARGET_MAX) \
/ (ADC_TORQUE_SENSOR_RANGE_TARGET_MAX - (((ADC_TORQUE_SENSOR_RANGE_TARGET_MAX - ADC_TORQUE_SENSOR_DELTA_WITH_WEIGHT) * 10) \
/ PEDAL_TORQUE_ADC_ANGLE_ADJ))) \
* PEDAL_TORQUE_PER_10_BIT_ADC_STEP_ADV_X100) / PEDAL_TORQUE_PER_10_BIT_ADC_STEP_BASE_X100))
*/

// scale the torque assist target current
#define TORQUE_ASSIST_FACTOR_DENOMINATOR		120

// smooth start ramp
#define SMOOTH_START_RAMP_DEFAULT				165 // from 860C 35% (255=0% long ramp)
#define SMOOTH_START_RAMP_MIN					30  // from 860C


// torque step mode Not used in 860C
//#define TORQUE_STEP_DEFAULT				    0 // not calibrated
//#define TORQUE_STEP_ADVANCED				1 // calibrated


// adc current
//#define ADC_10_BIT_BATTERY_EXTRACURRENT		38  //  6 amps
#define ADC_10_BIT_BATTERY_EXTRACURRENT			50//50 // value for TSDZ2=50; should be OK for TSDZ8 too  // 50 = 50*0.16= 8 amps
#define ADC_10_BIT_BATTERY_CURRENT_MAX			112//112 // value for Tsdz2= 112;  should be ok for TSDZ8; 112 = 18 amps // 1 = 0.16 Amp
//#define ADC_10_BIT_BATTERY_CURRENT_MAX		124	// 20 amps // 1 = 0.16 Amp
//#define ADC_10_BIT_BATTERY_CURRENT_MAX		136	// 22 amps // 1 = 0.16 Amp
#define ADC_10_BIT_MOTOR_PHASE_CURRENT_MAX		187//187 // value for tsdz2 = 187 ; :187 = 30 amps // 1 = 0.16 Amp
/*---------------------------------------------------------
 NOTE: regarding ADC battery current max

 This is the maximum current in ADC steps that the motor
 will be able to draw from the battery. A higher value
 will give higher torque figures but the limit of the
 controller is 16 A and it should not be exceeded.
 ---------------------------------------------------------*/

// throttle ADC values
#define ADC_THROTTLE_MIN_VALUE			47 // used in 860c
#define ADC_THROTTLE_MAX_VALUE			176 // used in 860C

/*---------------------------------------------------------
 NOTE: regarding throttle ADC values

 Max voltage value for throttle, in ADC 8 bits step,
 each ADC 8 bits step = (5 V / 256) = 0.0195

 ---------------------------------------------------------*/

// cadence sensor
#define CADENCE_SENSOR_NUMBER_MAGNETS			20U  // is not used in the code (hardcoded 60 min / 20 = 3)

/*---------------------------------------------------------------------------
 NOTE: regarding the cadence sensor

 CADENCE_SENSOR_NUMBER_MAGNETS = 20, this is the number of magnets used for
 the cadence sensor. Was validated on August 2018 by Casainho and jbalat

 Cadence is calculated by counting how much time passes between two
 transitions. Depending on if all transitions are measured or simply
 transitions of the same kind it is important to adjust the calculation of
 pedal cadence.
 --------------------------------------------------------------------------*/


// default values
#define DEFAULT_VALUE_BATTERY_CURRENT_MAX                         10  // 10 amps

/*---------------------------------------------------------

 NOTE: regarding the torque sensor output values

 Torque (force) value needs to be found experimentaly.

 One torque sensor ADC 10 bit step is equal to 0.38 kg

 Force (Nm) = 1 Kg * 9.81 * 0.17 (0.17 = arm cranks size)
 --------------------------------------------------------------------------*/

// ADC battery voltage measurement
#define BATTERY_VOLTAGE_PER_10_BIT_ADC_STEP_X1000		87  // conversion value verified with a cheap power meter = MVolt/adc10bit


// ADC battery voltage to be subtracted from the cut-off
#define DIFFERENCE_CUT_OFF_SHUTDOWN_10_BIT			100  // 9 Volts

/*---------------------------------------------------------
 NOTE: regarding ADC battery voltage measurement

 0.344 per ADC 8 bit step:

 17.9 V -->  ADC 8 bits value  = 52;
 40 V   -->  ADC 8 bits value  = 116;

 This signal is atenuated by the opamp 358.
 ---------------------------------------------------------*/

// ADC battery current measurement
#define BATTERY_CURRENT_PER_10_BIT_ADC_STEP_X100		16  // 0.16A x 10 bit ADC step

// for oem display

// walk assist
#define WALK_ASSIST_WHEEL_SPEED_MIN_DETECT_X10	42
#define WALK_ASSIST_ERPS_THRESHOLD				20
#define WALK_ASSIST_ADJ_DELAY_MIN				4
#define WALK_ASSIST_ADJ_DELAY_STARTUP			10
#define WALK_ASSIST_DUTY_CYCLE_MIN              40
#define WALK_ASSIST_DUTY_CYCLE_STARTUP			50
#define WALK_ASSIST_DUTY_CYCLE_MAX              130
#define WALK_ASSIST_ADC_BATTERY_CURRENT_MAX     40

#endif // _MAIN_H_

#ifdef USE_TSDZ8_VLCD5_VERSION
// next was in TSDZ8 VLCD5 version but not in 860C version

// to be checked come from TSDZ8 vlcd5 version
// UART
#define UART_RX_BUFFER_LEN   		7
#define RX_CHECK_CODE				(UART_RX_BUFFER_LEN - 1)															
#define UART_TX_BUFFER_LEN			9
#define TX_CHECK_CODE				(UART_TX_BUFFER_LEN - 1)
#define TX_STX						0x43
#define RX_STX						0x59

// parameters for display data
#define MILES						1

#define DATA_INDEX_ARRAY_DIM		6

/*
// delay lights function (0.1 sec)
#define DELAY_LIGHTS_ON				 	DELAY_MENU_ON    // 5sec

// delay function status (0.1 sec)
#define DELAY_FUNCTION_STATUS			        (uint8_t) (DELAY_MENU_ON / 2)  //2,5 sec
*/
// delay torque sensor calibration (0.1 sec)
#define DELAY_DISPLAY_TORQUE_CALIBRATION		250  // 25 sec

// display function status
#define FUNCTION_STATUS_OFF				1
#define FUNCTION_STATUS_ON				(uint8_t) (100 + DISPLAY_STATUS_OFFSET)
#define DISPLAY_STATUS_OFFSET			5

// assist level 
#define OFF						    0
#define ECO						    1
#define TOUR						2
#define SPORT						3
#define TURBO						4

// assist pedal level mask
#define ASSIST_PEDAL_LEVEL0				0x10
#define ASSIST_PEDAL_LEVEL01			0x80
#define ASSIST_PEDAL_LEVEL1				0x40
#define ASSIST_PEDAL_LEVEL2				0x02
#define ASSIST_PEDAL_LEVEL3				0x04
#define ASSIST_PEDAL_LEVEL4				0x08

#define ASSIST_PEDAL_LEVEL01_PERCENT			60

// assist mode
#define OFFROAD_MODE				0
#define STREET_MODE					1

// oem display fault & function code
#define CLEAR_DISPLAY				0
#define NO_FUNCTION					0
#define NO_FAULT					0
#define NO_ERROR                    0 

#define ERROR_OVERVOLTAGE				1 // E01 (E06 blinking for XH18)
#define ERROR_TORQUE_SENSOR            	2 // E02
#define ERROR_CADENCE_SENSOR		    3 // E03
#define ERROR_MOTOR_BLOCKED            	4 // E04
#define ERROR_THROTTLE					5 // E05 (E03 blinking for XH18)
#define ERROR_OVERTEMPERATURE			6 // E06  
#define ERROR_BATTERY_OVERCURRENT      	7 // E07 (E04 blinking for XH18)
#define ERROR_SPEED_SENSOR				8 // E08
#define ERROR_WRITE_EEPROM  			9 // E09 shared (E08 blinking for XH18)
#define ERROR_MOTOR_CHECK              	9 // E09 shared (E08 blinking for XH18)

/* // this has been changed in ebike_app.c because it was used only one one place
// optional ADC function
#if ENABLE_TEMPERATURE_LIMIT && ENABLE_THROTTLE
#define OPTIONAL_ADC_FUNCTION                 		NOT_IN_USE
#elif ENABLE_TEMPERATURE_LIMIT
#define OPTIONAL_ADC_FUNCTION                 		TEMPERATURE_CONTROL
#elif ENABLE_THROTTLE && ENABLE_BRAKE_SENSOR
#define OPTIONAL_ADC_FUNCTION                 		THROTTLE_CONTROL
#else
#define OPTIONAL_ADC_FUNCTION                 		NOT_IN_USE
#endif
*/
// temperature sensor type
#define LM35						0
#define TMP36						1

// throttle mode
#define DISABLED					0
#define PEDALING					1
#define W_O_P_6KM_H_ONLY			2
#define W_O_P_6KM_H_AND_PEDALING    3
#define UNCONDITIONAL				4

// wheel perimeter ; it seems this is not used
// #define WHEEL_PERIMETER_0				(uint8_t) (WHEEL_PERIMETER & 0x00FF)
// #define WHEEL_PERIMETER_1				(uint8_t) ((WHEEL_PERIMETER >> 8) & 0x00FF)

// BATTERY PARAMETER
// battery low voltage cut off
// #define BATTERY_LOW_VOLTAGE_CUT_OFF_X10_0		(uint8_t) ((uint16_t)(BATTERY_LOW_VOLTAGE_CUT_OFF * 10) & 0x00FF)
// #define BATTERY_LOW_VOLTAGE_CUT_OFF_X10_1		(uint8_t) (((uint16_t)(BATTERY_LOW_VOLTAGE_CUT_OFF * 10) >> 8) & 0x00FF)
// battery voltage to be subtracted from the cut-off 8bit
#define DIFFERENCE_CUT_OFF_SHUTDOWN_8_BIT		26
// battery voltage for saving battery capacity at shutdown
// #define BATTERY_VOLTAGE_SHUTDOWN_8_BIT			(uint8_t) ((uint16_t)(BATTERY_LOW_VOLTAGE_CUT_OFF * 250 / BATTERY_VOLTAGE_PER_10_BIT_ADC_STEP_X1000)) - ((uint16_t) DIFFERENCE_CUT_OFF_SHUTDOWN_8_BIT)
// #define BATTERY_VOLTAGE_SHUTDOWN_10_BIT			(uint16_t) (BATTERY_VOLTAGE_SHUTDOWN_8_BIT << 2)
// battery voltage reset SOC percentage
// #define BATTERY_VOLTAGE_RESET_SOC_PERCENT_X10   (uint16_t)((float)LI_ION_CELL_RESET_SOC_PERCENT * (float)(BATTERY_CELLS_NUMBER * 10))
//battery SOC eeprom value saved (8 bit)
#define BATTERY_SOC					0
// battery SOC % threshold x10 (volts calc)
#define BATTERY_SOC_PERCENT_THRESHOLD_X10		150
/*
// cell bars
#if ENABLE_VLCD6 || ENABLE_XH18
#define LI_ION_CELL_VOLTS_6_X100		(uint16_t)((float)LI_ION_CELL_OVERVOLT * 100)
#define LI_ION_CELL_VOLTS_5_X100		(uint16_t)((float)LI_ION_CELL_RESET_SOC_PERCENT * 100)
#define LI_ION_CELL_VOLTS_4_X100		(uint16_t)((float)LI_ION_CELL_VOLTS_FULL * 100)
#define LI_ION_CELL_VOLTS_3_X100		(uint16_t)((float)LI_ION_CELL_VOLTS_3_OF_4 * 100)
#define LI_ION_CELL_VOLTS_2_X100		(uint16_t)((float)LI_ION_CELL_VOLTS_2_OF_4 * 100)
#define LI_ION_CELL_VOLTS_1_X100		(uint16_t)((float)LI_ION_CELL_VOLTS_1_OF_4 * 100)
#define LI_ION_CELL_VOLTS_0_X100		(uint16_t)((float)LI_ION_CELL_VOLTS_EMPTY * 100)
#define BATTERY_SOC_VOLTS_6_X10			(uint16_t)(BATTERY_CELLS_NUMBER * ((float)LI_ION_CELL_OVERVOLT * 10))
#define BATTERY_SOC_VOLTS_5_X10			(uint16_t)(BATTERY_CELLS_NUMBER * ((float)LI_ION_CELL_RESET_SOC_PERCENT * 10))
#define BATTERY_SOC_VOLTS_4_X10			(uint16_t)(BATTERY_CELLS_NUMBER * ((float)LI_ION_CELL_VOLTS_FULL * 10))
#define BATTERY_SOC_VOLTS_3_X10			(uint16_t)(BATTERY_CELLS_NUMBER * ((float)LI_ION_CELL_VOLTS_3_OF_4 * 10))
#define BATTERY_SOC_VOLTS_2_X10			(uint16_t)(BATTERY_CELLS_NUMBER * ((float)LI_ION_CELL_VOLTS_2_OF_4 * 10))
#define BATTERY_SOC_VOLTS_1_X10			(uint16_t)(BATTERY_CELLS_NUMBER * ((float)LI_ION_CELL_VOLTS_1_OF_4 * 10))
#define BATTERY_SOC_VOLTS_0_X10			(uint16_t)(BATTERY_CELLS_NUMBER * ((float)LI_ION_CELL_VOLTS_EMPTY * 10))
#else // ENABLE_VLCD5 or 850C
#define LI_ION_CELL_VOLTS_8_X100		(uint16_t)((float)LI_ION_CELL_OVERVOLT * 100)
#define LI_ION_CELL_VOLTS_7_X100		(uint16_t)((float)LI_ION_CELL_RESET_SOC_PERCENT * 100)
#define LI_ION_CELL_VOLTS_6_X100		(uint16_t)((float)LI_ION_CELL_VOLTS_FULL * 100)
#define LI_ION_CELL_VOLTS_5_X100		(uint16_t)((float)LI_ION_CELL_VOLTS_5_OF_6 * 100)
#define LI_ION_CELL_VOLTS_4_X100		(uint16_t)((float)LI_ION_CELL_VOLTS_4_OF_6 * 100)
#define LI_ION_CELL_VOLTS_3_X100		(uint16_t)((float)LI_ION_CELL_VOLTS_3_OF_6 * 100)
#define LI_ION_CELL_VOLTS_2_X100		(uint16_t)((float)LI_ION_CELL_VOLTS_2_OF_6 * 100)
#define LI_ION_CELL_VOLTS_1_X100		(uint16_t)((float)LI_ION_CELL_VOLTS_1_OF_6 * 100)
#define LI_ION_CELL_VOLTS_0_X100		(uint16_t)((float)LI_ION_CELL_VOLTS_EMPTY * 100)
#define BATTERY_SOC_VOLTS_8_X10			(uint16_t)(BATTERY_CELLS_NUMBER * ((float)LI_ION_CELL_OVERVOLT * 10))
#define BATTERY_SOC_VOLTS_7_X10			(uint16_t)(BATTERY_CELLS_NUMBER * ((float)LI_ION_CELL_RESET_SOC_PERCENT * 10))
#define BATTERY_SOC_VOLTS_6_X10			(uint16_t)(BATTERY_CELLS_NUMBER * ((float)LI_ION_CELL_VOLTS_FULL * 10))
#define BATTERY_SOC_VOLTS_5_X10			(uint16_t)(BATTERY_CELLS_NUMBER * ((float)LI_ION_CELL_VOLTS_5_OF_6 * 10))
#define BATTERY_SOC_VOLTS_4_X10			(uint16_t)(BATTERY_CELLS_NUMBER * ((float)LI_ION_CELL_VOLTS_4_OF_6 * 10))
#define BATTERY_SOC_VOLTS_3_X10			(uint16_t)(BATTERY_CELLS_NUMBER * ((float)LI_ION_CELL_VOLTS_3_OF_6 * 10))
#define BATTERY_SOC_VOLTS_2_X10			(uint16_t)(BATTERY_CELLS_NUMBER * ((float)LI_ION_CELL_VOLTS_2_OF_6 * 10))
#define BATTERY_SOC_VOLTS_1_X10			(uint16_t)(BATTERY_CELLS_NUMBER * ((float)LI_ION_CELL_VOLTS_1_OF_6 * 10))
#define BATTERY_SOC_VOLTS_0_X10			(uint16_t)(BATTERY_CELLS_NUMBER * ((float)LI_ION_CELL_VOLTS_EMPTY * 10))
#endif
*/

// assist level 0
#define TORQUE_ASSIST_LEVEL_0        0
#define CADENCE_ASSIST_LEVEL_0       0
#define EMTB_ASSIST_LEVEL_0          0
#define WALK_ASSIST_LEVEL_0          0
#define CRUISE_TARGET_SPEED_LEVEL_0  0


// power assist level
#define POWER_ASSIST_LEVEL_OFF       0
#define POWER_ASSIST_LEVEL_ECO       (uint8_t)(POWER_ASSIST_LEVEL_1 / 2)
#define POWER_ASSIST_LEVEL_TOUR      (uint8_t)(POWER_ASSIST_LEVEL_2 / 2)
#define POWER_ASSIST_LEVEL_SPORT     (uint8_t)(POWER_ASSIST_LEVEL_3 / 2)
#define POWER_ASSIST_LEVEL_TURBO     (uint8_t)(POWER_ASSIST_LEVEL_4 / 2)

// walk assist
//#define WALK_ASSIST_THRESHOLD_SPEED		        (uint8_t)(WALK_ASSIST_THRESHOLD_SPEED_X10 / 10)
#define WALK_ASSIST_WHEEL_SPEED_MIN_DETECT_X10	42
#define WALK_ASSIST_ERPS_THRESHOLD			    20
#define WALK_ASSIST_ADJ_DELAY_MIN			     4
#define WALK_ASSIST_ADJ_DELAY_STARTUP			10
#define WALK_ASSIST_DUTY_CYCLE_MIN              40
#define WALK_ASSIST_DUTY_CYCLE_STARTUP			50
#define WALK_ASSIST_DUTY_CYCLE_MAX              130
#define WALK_ASSIST_ADC_BATTERY_CURRENT_MAX      40

/*
// cruise threshold (speed limit min km/h x10)
#define CRUISE_THRESHOLD_SPEED_X10			(CRUISE_THRESHOLD_SPEED * 10)
#define CRUISE_THRESHOLD_SPEED_X10_DEFAULT		80
#define CRUISE_OFFROAD_THRESHOLD_SPEED_X10		(uint8_t)CRUISE_THRESHOLD_SPEED_X10
#if CRUISE_THRESHOLD_SPEED_X10 < CRUISE_THRESHOLD_SPEED_X10_DEFAULT
#define CRUISE_STREET_THRESHOLD_SPEED_X10		(uint8_t)(CRUISE_THRESHOLD_SPEED_X10_DEFAULT)
#else	
#define CRUISE_STREET_THRESHOLD_SPEED_X10		(uint8_t)(CRUISE_THRESHOLD_SPEED_X10)
#endif	
*/

// odometer compensation for displayed data (eeprom)
#define ODOMETER_COMPENSATION				0
// zero odometer compensation
#define ZERO_ODOMETER_COMPENSATION			100000000

#define ASSISTANCE_WITH_ERROR_ENABLED			0

#define AVAIABLE_FOR_FUTURE_USE				0 // EEPROM

#define ADDRESS_OF_M_CONFIG_FLASH 0x1000F000U // address in flash where the config is strored (must be the same as the adrress set in the xls for config)
    
#endif // _MAIN_H_
