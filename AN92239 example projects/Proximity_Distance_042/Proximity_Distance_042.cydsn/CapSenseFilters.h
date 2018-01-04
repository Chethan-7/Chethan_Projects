/***************************************************************************//**
* @file    CapSenseFilters.h
* @version 1.0
* @authors PRIA
*
* @par Description
*    This file contains the public interface of Advanced Low Pass Filter APIs.
*
* @par Notes
*	This library requires that all proximity sensors should occupy scan order
* 	0 through (n-1) in the CapSense component (Here, n is the total number of 
*	proximity sensors defined in CapSense component).
*
* @par Hardware Dependency
*    None
*
* @par References
*    1. 001-92239: AN92239 "Proximity Sensing with CapSense"
*
* @par Code Tested With
*    1. Silicon: PSoC4100/4200
*    2. IDE: PSoC Creator 3.0 SP1 (3.0.0.3023)
*    3. Compiler: GCC 4.7.3, ARM MDK 4.54.0.0 (armcc 4.1 b894)
*
**//****************************************************************************
* Copyright (2012 - 2014), Cypress Semiconductor Corporation.
********************************************************************************
* All rights reserved. 
* This software, associated documentation and materials (“Software”) is owned
* by Cypress Semiconductor Corporation (“Cypress”) and is protected by and
* subject to worldwide patent protection (United States and foreign), United
* States copyright laws and international treaty provisions. Therefore, you
* may use this Software only as provided in the license agreement accompanying
* the software package from which you obtained this Software (“EULA”).
* If no EULA applies, then any reproduction, modification, translation, 
* compilation, or representation of this Software is prohibited without the 
* express written permission of Cypress. 

* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED 
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress 
* reserves the right to make changes to the Software without notice. Cypress 
* does not assume any liability arising out of the application or use of the 
* Software or any product or circuit described in the Software. Cypress does 
* not authorize its products for use in any products where a malfunction or 
* failure of the Cypress product may reasonably be expected to result in 
* significant property damage, injury or death (“High Risk Product”). By 
* including Cypress’s product in a High Risk Product, the manufacturer of such 
* system or application assumes all risk of such use and in doing 
* so agrees to indemnify Cypress against all liability. 
*
*******************************************************************************/
#ifndef __CAPSENSEFILTERS_H    /* Guard to prevent multiple inclusions */
#define __CAPSENSEFILTERS_H

/*******************************************************************************
*   Included Headers
*******************************************************************************/
#include "CapSense.h"
#include "cytypes.h"	

/*******************************************************************************
*   Macros and #define Constants
*******************************************************************************/
#define CAPSENSEFILTERS_ALP_ENABLE	            (1)

/* This compile time value selects the Average Filter order 
 * Average filter is used internally by ALP filter. This value must 
 * not be modified for the ALP filter to give expected filtered output.
 */    
#define AVERAGE_ORDER	        (0x04u)

/* Size of Average Filter history */
#define AVERAGE_HISTORY_SIZE    (AVERAGE_ORDER - 1)


/*******************************************************************************
*   Data Type Definitions
*******************************************************************************/

/* Following enum type definition is used to set the K-value for ALP Filter. 
 * The K-value of the ALP filter determines the attenuation of noise in the
 * proximity sensor raw counts. Noise attenuation decreases in the order 
 * CAPSENSEFILTERS_IIR_K_64 > CAPSENSEFILTERS_IIR_K_32 > CAPSENSEFILTERS_IIR_K_16. 
 * The CAPSENSEFILTERS_IIR_K_4 and CAPSENSEFILTERS_IIR_K_2 is added for 
 * compatibility purpose and should not be used. Refer to the ALP Filter Tuning 
 * section in AN92239 for more details on this parameter.
 */
typedef enum
{
	CAPSENSEFILTERS_IIR_K_2		= 0x01,
	CAPSENSEFILTERS_IIR_K_4 	= 0x02,
	CAPSENSEFILTERS_IIR_K_16 	= 0x04,
	CAPSENSEFILTERS_IIR_K_32 	= 0x05,
	CAPSENSEFILTERS_IIR_K_64 	= 0x06
} CAPSENSEFILTERS_IIR_K_ENUM;


/* Following enum type definition defines states of ALP filter. 
 * UNINITIALIZED state indicates that the filter history variables are not 
 * initialized.
 * IDLE state indicates the normal operation of a sensor when there is to 
 * target object near the sensor. A slow response filter is applied in this 
 * state.
 * POS_EDGE_TRACK state indicates that there is a target object approaching 
 * the sensor. A fast response filter is applied in this state. 
 * SIGNAL_DETECTED state indicates that the target object is detected. 
 * A slow response filter is applied in this state.
 * NEG_EDGE_TRACK state indicates that the target object is receding away from
 * the sensor. A fast response filter is applied in this state. 
 * NEG_SPIKE state indicates an unexpected negative noise on the sensor 
 * raw count. This state is not expected to occur during normal sensor 
 * operation.
 */
typedef enum
{
	CAPSENSEFILTERS_ALP_STATE_UNINITIALIZED 	= 0x00,
	CAPSENSEFILTERS_ALP_STATE_IDLE 				= 0x01,
	CAPSENSEFILTERS_ALP_STATE_POS_EDGE_TRACK 	= 0x02,
	CAPSENSEFILTERS_ALP_STATE_SIGNAL_DETECTED 	= 0x03,
	CAPSENSEFILTERS_ALP_STATE_NEG_EDGE_TRACK 	= 0x04,
	CAPSENSEFILTERS_ALP_STATE_NEG_SPIKE 		= 0x05
} CAPSENSEFILTERS_ALP_STATE_ENUM;

/*******************************************************************************
*   Extern Variable and Constant Declarations
*******************************************************************************/

/* Following arrays are expected to be defined in user code with the same
 * data type as declared below. The array size should be equal to the
 * macro CapSense_TOTAL_PROX_SENSOR_COUNT defined by CapSense component
 */

/* Thresholds for adjusting ALP filter response */
extern uint8 proxPositiveTh[];
extern uint8 proxNegativeTh[];            
            
/* Average Filter history */
extern uint16 averageHistory[][AVERAGE_HISTORY_SIZE]; 
/* Indicates the oldest data amongst Average Filter histories */
extern uint8 avgOldestDataIndex[];

/* State variable for ALP filter state machine */													
extern CAPSENSEFILTERS_ALP_STATE_ENUM stateAdvancedFilter[];  

/* Average Filter output */													
extern uint16 averageVal[];  

/* Following variables contain input histories for ALP filter */
extern uint16 filterHistory0[];
extern uint16 filterHistory1[];	
extern uint16 filterHistory2[];

/*******************************************************************************
*   Function Declarations
*******************************************************************************/
uint32 CapSenseFilters_GetLibRevision(void);

#if CAPSENSEFILTERS_ALP_ENABLE
    
    /* This API sets the filter coefficient (K-value) for the ALP filter. */
    void CapSenseFilters_SetAdvancedLowPassK(CAPSENSEFILTERS_IIR_K_ENUM k);
    
    /* This API initializes the internal history of the specified sensor to 
     * the specified raw count value.
     */
    void CapSenseFilters_InitializeAdvancedLowPass(
    											   uint32 sensorId,
    											   uint16 initializationRawValue
    											   );
    
    /* This API applies the ALP filter to the raw count of the proximity sensor.
     * Filtered data is stored back in the raw count array.
     */
    void CapSenseFilters_RunAdvancedLowPass(uint32 sensorId);
    
    /* This API resets the ALP filter state for given sensorId. Resetting the  
     * ALP Filter ensures that the filter history is automatically reinitialized 
     * when the next time “CapSenseFilters_RunAdvancedLowPass()” API is called.
     */
    void CapSenseFilters_ResetAdvancedLowPass(uint32 sensorId);
    
    /* This API returns the output of the Average Filter for the specified 
     * sensor (sensorId).
     */
    uint16 CapSenseFilters_GetAverageData(uint32 sensorId);
    
    /* This API returns the current state of ALP filter. */
    CAPSENSEFILTERS_ALP_STATE_ENUM CapSenseFilters_GetALPFilterState(
																uint32 sensorId
																);	
#endif

#endif /* #ifndef __CAPSENSEFILTERS_H */
