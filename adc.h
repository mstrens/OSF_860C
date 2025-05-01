#ifndef PMSM_FOC_MCUINIT_ADC_H_
#define PMSM_FOC_MCUINIT_ADC_H_

#include "cybsp.h"
#include "cy_utils.h"

/*********************************************************************************************************************
 * HEADER FILES
 ***************************************/

//#include "../Configuration/pmsm_foc_variables_scaling.h"

/* To use ADC calibration. From IFX example project "ADC_Cal_BG_Source_Example_XMC13". */

/*********************************************************************************************************************
 * MACRO
 ***************************************/
#define SHS0_CALOC0 ((uint32_t *)0x480340E0)
#define SHS0_CALOC1 ((uint32_t *)0x480340E4)
#define SHS0_CALCTR ((uint32_t *)0x480340BC)

#define SHS_CALLOC0_CLEAR_OFFSET (0x8000U)
#define REG_RESET (0x00U)
#define GLOBCFG_CLEAR (0x80030000U)
#define CLEAR_OFFSET_CALIB_VALUES         *SHS0_CALOC0 = SHS_CALLOC0_CLEAR_OFFSET;\
                                          *SHS0_CALOC1 = SHS_CALLOC0_CLEAR_OFFSET
/** Delay cycles to complete startup calibration */
#define VADC_CALIBRATION_CYCLE_DELAY  (20U)
/** Trigger dummy conversion for 9* 2000 times.*/
#define VADC_DUMMY_CONVERSION_COUNTER (18000U)

// added by mstrens : copied from infieneon example
/* ADC is configured as three shunt in SYNC*/// other are removed by mstrens
/* Motor Phase U VADC define */
#define VADC_IU_G1_CHANNEL    (4U)        /* P2.9, VADC group1 channel 4 */
#define VADC_IU_G0_CHANNEL    (2U)        /* P2.9, VADC group0 channel 2 */

/* Motor Phase V VADC define */
#define VADC_IV_G1_CHANNEL    (2U)       /* P2.10, VADC group1 channel 2 */
#define VADC_IV_G0_CHANNEL    (3U)       /* P2.10, VADC group0 channel 3 */

/* Motor Phase W VADC define */
#define VADC_IW_G1_CHANNEL    (3U)       /* P2.11, VADC group1 channel 3 */
#define VADC_IW_G0_CHANNEL    (4U)       /* P2.11, VADC group0 channel 4 */


#define VADC_I1_GROUP         VADC_G1
#define VADC_I1_CHANNEL       (0U)
#define VADC_I1_RESULT_REG    (0U)

#define VADC_I3_GROUP         VADC_G1
#define VADC_I3_CHANNEL       (1U)
#define VADC_I3_RESULT_REG    (1U)

#define VADC_I2_GROUP         VADC_G0
#define VADC_I2_CHANNEL       (0U)
#define VADC_I2_RESULT_REG    (0U)

#define VADC_I4_GROUP         VADC_G0
#define VADC_I4_CHANNEL       (1U)
#define VADC_I4_RESULT_REG    (1U)

/* VADC Group 0 Alias channel 0 and channel 1 */
#define VADC_G0_CHANNEL_ALIAS0  VADC_IV_G0_CHANNEL
#define VADC_G0_CHANNEL_ALIAS1  VADC_IDC_CHANNEL

/* VADC Group 1 Alias channel 0 and channel 1 */
#define VADC_G1_CHANNEL_ALIAS0  VADC_IW_G1_CHANNEL
#define VADC_G1_CHANNEL_ALIAS1  VADC_IU_G1_CHANNEL

// DC link voltage VADC define 
#define VADC_VDC_GROUP        VADC_G1
#define VADC_VDC_GROUP_NO     (1U)
#define VADC_VDC_CHANNEL      (6U)      /* P2.4 VADC group1 channel 6 */
#define VADC_VDC_RESULT_REG   (6U)


//TSDZ8 uses pin 2.8 for total current; it is group 0 ch 1 or group 1 ch 0
#define VADC_IDC_GROUP        VADC_G0
#define VADC_IDC_GROUP_NO     (0U)
#define VADC_IDC_CHANNEL      (1U)       /* P2.8 VADC group0 channel 1 */
#define VADC_IDC_RESULT_REG   (1U)


// todo adapt when throttle of tsdz8 should be tested (is enabled in another config file)
// Potentiometer VADC define = throttle
#define VADC_POT_GROUP        VADC_G1
#define VADC_POT_GROUP_NO     (1U)
#define VADC_POT_CHANNEL      (7U)      /* P2.5 VADC group1 channel 7 */
#define VADC_POT_RESULT_REG   (7U)

// added by mstrens for torque on G0_CH7_TORQUE_P2_2_config
#define VADC_TORQUE_GROUP        VADC_G0
#define VADC_TORQUE_GROUP_NO     (0U)
#define VADC_TORQUE_CHANNEL      (7U)      /* P2.2 VADC group0 channel 7 */
#define VADC_TORQUE_RESULT_REG   (7U)



/*********************************************************************************************************************
 * ENUMS
 ***************************************/
typedef enum SHS_GAIN_FACTOR
{
  SHS_GAIN_FACTOR_1 = 0,   /**< Select gain factor 1 */
  SHS_GAIN_FACTOR_3,       /**< Select gain factor 3 */
  SHS_GAIN_FACTOR_6,       /**< Select gain factor 6 */
  SHS_GAIN_FACTOR_12       /**< Select gain factor 12 */
}SHS_GAIN_FACTOR_t;


/*********************************************************************************************************************
 * DATA STRUCTURES
 ***************************************/
typedef struct ADCType
{
  uint16_t ADC_Iu;					 /* For 3-shunt phase current sensing */
  uint16_t ADC_Iv;
  uint16_t ADC_Iw;

  uint16_t ADC_Bias_Iu;			/* Bias of ADC Iu. */
  uint16_t ADC_Bias_Iv;			/* Bias of ADC Iv. */
  uint16_t ADC_Bias_Iw;			/* Bias of ADC Iw. */

	int32_t ADC_Pos_Iu;				/* +Iu. For 2-shunt phase current sensing, no external Op-Amps */
	int32_t ADC_Pos_Iv;				/* +Iv */

	uint16_t ADCTrig_Point;		/* ADC trigger position for 2or3-shunt current sensing */

	int32_t ADC_POT;				  /* ADC Value of potentiometer (POT) */
	int32_t ADC_DCLink;				/* ADC Value of inverter DC link voltage Vdc */
  int32_t ADC_IDCLink;      /* ADC Value of inverter DC link current Idc*/
	int32_t ADC_Bias;				  /* ADC for bias of dc-link current amplifier, or on-chip gain */
	uint32_t ADC_ResultTz1;			/* ADC value (12-bit) of first motor phase current ADC sampling */
	uint32_t ADC_ResultTz2;			/* For single-shunt current sensing. */
	uint32_t ADC_Result3;
	uint32_t ADC_Result4;

	int32_t ADC_Result1;			/* Dedicated RAM for single-shunt 3-Phase Current Reconstruction */
	int32_t ADC_Result2;

	uint16_t ADC3Trig_Point;	/* Trigger position for ADC3, of single-shunt current sensing */
	uint16_t ADC4Trig_Point;	/* Trigger position for ADC4 */

	  uint16_t BEMF_U;        // BEMF for phase U, V and W.
	  uint16_t BEMF_V;
	  uint16_t BEMF_W;
	  uint16_t BEMF_Max;        // Maximum value of BEMF_U and BEMF_V.
	  uint16_t BEMF_UV_Threshold;   // Threshold, above it BEMF_U and BEMF_V are considered high.
	
	 uint16_t current_bias_counter ; // added by mstrens to count that there is enough bias measurement for bias (in DLE state)
}ADCType;

//void pmsm_phasecurrent_init(void);
void pmsm_adc_module_init(void);
void pmsm_adc_pot_init(void);
void pmsm_adc_dclink_init(void);

/**
 * @param None
 * @return
 *    None<BR>
 *
 * \par<b>Description:</b><br>
 * Initializes the VADC channel for Catch Free Running BEMF voltage sensing.  <BR>\n
 *
 * \par<b>Related APIs:</b><br>
 * None.
 */
void ADC_CFR_Init(void);

void pmsm_foc_adc_gaincalib(void);
void pmsm_foc_adc_startupcalibration(void);


#endif /* MCUINIT_ADC_H_ */
