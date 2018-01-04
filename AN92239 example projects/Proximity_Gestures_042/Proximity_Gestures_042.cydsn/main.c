/***************************************************************************//**
* @file    main.c
* @version 1.0
* @authors SLAN
*
* @par Description
*    This file contains the main source code for Proximity_Gesture code example
*    provided with Application note AN92239 "Proximity sensing with CapSense".
*	 
*	 Proximity gesture example project shows proximity gesture in X-direction 
*	 and Y- direction. Three LEDs are used to show case gesture in X-direction 
*	 and three in Y- Direction. LEDs are OFF initially and when the gesture is 
*	 completed in any direction the LEDs turn ON and then OFF in the same 
*	 directions.  One LED is used to indicate proximity distance at which gestures
*	 can be performed. 
*
* @par Notes
*    - 
*
* @par Hardware Dependency
*    1. CY8CKIT-042 PSoC4 Pioneer Kit
*    2. CY8CKIT-024 CapSense Proximity Shield
*
* @par References
*    1. 001-92239: AN92239 "Proximity sensing with CapSense"
*
* @par Code Tested With
*    1. PSoC Creator  3.0 SP1 (3.0.0.3023)
*    2. GCC 4.7.3
*	 3. MDK
*
**//****************************************************************************
* Copyright (2014), Cypress Semiconductor Corporation.
********************************************************************************
* All rights reserved. 
* This software, including source code, documentation and related 
* materials (“Software”), is owned by Cypress Semiconductor 
* Corporation (“Cypress”) and is protected by and subject to worldwide 
* patent protection (United States and foreign), United States copyright 
* laws and international treaty provisions. Therefore, you may use this 
* Software only as provided in the license agreement accompanying the 
* software package from which you obtained this Software (“EULA”). 
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive, 
* non-transferable license to copy, modify and compile the Software source code
* solely for your use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.

* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* Cypress reserves the right to make changes to the Software without notice. 
* Cypress does not assume any liability arising out of the application or use 
* of the Software or any product or circuit described in the Software. Cypress
* does not authorize its products for use in any products where a malfunction
* or failure of the Cypress product may reasonably be expected to result in 
* significant property damage, injury or death (“High Risk Product”). By 
* including Cypress’s product in a High Risk Product, the manufacturer of such  
* system or application assumes all risk of such use and in doing so agrees to  
* indemnify Cypress against all liability. 
*******************************************************************************/

/*******************************************************************************
*   Included Headers
*******************************************************************************/
#include <project.h>
#include <stdbool.h>
#include <gestures.h>

/*******************************************************************************
*   Macros and #define Constants
*******************************************************************************/
/* Macro to enable Tx8 data out on P0[1], sends out serial data when macro is 
 * set to '1' 
 */
/* Note the following before enabling Tx8:
 *     In CY8CKIT-042, P0[1] on header J2, has to be manually connected to
 *     P12[6] on header J8. 
 *     P12[6] on header J8 is the Rx pin of onboard USB-UART hardware. Since  
 *     this pin is not hardwired to any of the PSoC4A pins, a manual connection 
 *     is required between PSoC4A Tx8 pin and P12[6] on J8.
 *     Refer AN92239 "Proximity Sensing with CapSense" for more details on 
 *     using Tx8 output to view data on Bridge Control Panel.
 */
#define TX8_ENABLE                  (1u)
/*Enable tuner during tuning*/
#define TUNER_ENABLE                (0u)

/*******************************************************************************
*   Variable and Function Declarations
*******************************************************************************/
#if TX8_ENABLE
    #define NO_OF_SAMPLES(array)	    (sizeof(array)/sizeof(array[0]))    
        
    /* Multichart header and tail */
    const uint8 multichartHeader[] =    {0x0D, 0x0A};      /* Header = CR, LF */
    const uint8 multichartTail[] =      {0x00, 0xFF, 0xFF};  /* Tail = 0x00, 0xFF, 0xFF */

    void SendUint16ToMultichart(uint16 value);
    void SendDataToMultichart(uint8, uint8);
#endif
#if !TUNER_ENABLE
		/*Sensor numbers*/
		SENSOR_NAMES startSensorNumber;
		SENSOR_NAMES endSensorNumber;
#endif

void CapSenseInitialization(void);
void DeviceInit(void);

/*******************************************************************************
* Function Name: main
****************************************************************************//**
* @par Summary
*    This is the main entry point for this application. 
*
* @return
*    None
*
* @param[in] 
*    none
*
* @par Theory of Operation
*    This main code scans all the proximity sensors, and based on the signal
*	 variations on proximity sensors, x and y axis gestures are detected, LEDs
*	 are then driven based on the gesture detection. CapSense raw data
*	 is sent out using serial data
*
* @par Notes
*    None
*
**//***************************************************************************/
int main()
{
	#if !TUNER_ENABLE
		/*Sensor numbers*/
		uint8 sensorNo;
	#endif
	

	/*Initializes system variables, global interrupts and serial interface*/	
	DeviceInit();
	#if TUNER_ENABLE
		/*Only sensors are scanned when tuner is enabled*/
	    CapSenseInitialization();
		CapSense_TunerStart();
	    while(1)
	    {
	        CapSense_TunerComm();
	    }
	#else
	
		/*Initializes capacitive sensing engine*/
		CapSenseInitialization();	
		
		/*Based on the board and gesture direction assign the proper sensor numbers*/
		#if(GESTURE_AXIS)
			startSensorNumber = LEFT_SENSOR;
			endSensorNumber = RIGHT_SENSOR;	
		#else
			startSensorNumber = BOTTOM_SENSOR;
			endSensorNumber = TOP_SENSOR;
		#endif
	    for(;;)
	    {	
			for(sensorNo = startSensorNumber ;sensorNo <= endSensorNumber; sensorNo++)
			{
				/*Scan sensor*/
				CapSense_ScanSensor(sensorNo);
				/*Wait till sensor is scanned*/
		    	while(CapSense_IsBusy())
				{
				} 
		        /*Update the Sensor digital count reference*/
		        CapSense_UpdateSensorBaseline(sensorNo);	
			}			
		
			/*Detailed gesture algorithm is explained in gesture.c file*/
			/*Detect x-axis gestures*/
			#if(GESTURE_AXIS)
				GestureDetection(startSensorNumber,endSensorNumber, &XAxis); 
			#else
				GestureDetection(startSensorNumber,endSensorNumber, &YAxis); 
			#endif	
			
			/*CapSense digital data with reference and signal is sent out serially*/
			#if TX8_ENABLE
	        	SendDataToMultichart(startSensorNumber, endSensorNumber); 
			#endif  			
	    }
	#endif
}


/*******************************************************************************
* Function Name: DeviceInit
****************************************************************************//**
* @par Summary
*    Initializes system variables, global interrupts and serial interface
*
* @return
*    None
*
* @param[in] 
*    none
*
* @par Theory of Operation
*    Initializes system variables, global interrupts and serial interface
*
* @par Notes
*    None
*
**//***************************************************************************/
void DeviceInit(void)
{			
	CyGlobalIntEnable;    
    /*Initialize the variables needed for gestures*/
	GestureVariableInit(&XAxis);
	GestureVariableInit(&YAxis);
	#if TX8_ENABLE
    	Tx8_Start();
	#endif
	
}


/*******************************************************************************
* Function Name: CapSenseInitialization
****************************************************************************//**
* @par Summary
*    Initializes the capacitive sensing engine. 
*
* @return
*    None
*
* @param[in] 
*    none
*
* @par Theory of Operation
*    Enables proximity sensors, Starts the Capacitive sensing hardware block,
*	 Initializes the sensor decision logic reference
*
* @par Notes
*    None
*
**//***************************************************************************/
void CapSenseInitialization(void)
{
	uint8 sensor;
	for(sensor= 0; sensor < CapSense_TOTAL_SENSOR_COUNT; sensor++)
	{
		CapSense_EnableWidget(sensor);
	}
	
	CapSense_Start();        
    /*Set the reference*/
	CapSense_InitializeEnabledBaselines();       

}


/*******************************************************************************
* Function Name: SendDataToMultichart
****************************************************************************//**
* @par Summary
*    This function sends data of either x-axis or y-axis sensor based on macro setting
*    to Tx8 in multichart format.
*
* @return
*    None
*
* @param[in] 
*    startSensorNumber: start sensor number index to send TX8 data
*	 endSensorNumber : TX8 data is sent till this index number
*
* @par Theory of Operation
*    Multichart tool expects a header followed by 3x number of data 
*    words (16 bit, big endian) and a pre-defined tail. This function sends 
*    following data:
*       a. rawcounts
*       b. baseline
*       c. difference counts
*
* @par Notes
*    None
*
**//***************************************************************************/
#if TX8_ENABLE
void SendDataToMultichart(uint8 startSensorNumber, uint8 endSensorNumber)
{
    uint8 sensor;
    
    /* Send header expected by multichart tool */
    Tx8_PutArray(multichartHeader, NO_OF_SAMPLES(multichartHeader));
    
    /* Send 3x words(uint16) of data (as expected by multichart) */
    /* Send Raw Counts, These are labeled as Raw Count 0  
     * to Raw Count (endSensorNumber-startSensorNumber)
     */
    for(sensor= startSensorNumber; sensor <= endSensorNumber; sensor++)
    {
        SendUint16ToMultichart(CapSense_ReadSensorRaw(sensor));
    }

	/* Send Baseline, These are labeled as Baseline 0  
     * to Baseline (endSensorNumber-startSensorNumber)
     */
    for(sensor= startSensorNumber; sensor <= endSensorNumber; sensor++)
    {
        SendUint16ToMultichart(CapSense_GetBaselineData(sensor));
    }
	/* Send Signal, These are labeled  as Signal 0  
     * to Signal (endSensorNumber-startSensorNumber)
     */
	for(sensor= startSensorNumber; sensor <= endSensorNumber; sensor++)
    {
        SendUint16ToMultichart(CapSense_GetDiffCountData(sensor));
    }
    /* Send the Tail as expected by multichart */
    Tx8_PutArray(multichartTail, NO_OF_SAMPLES(multichartTail));
	
}

void SendUint16ToMultichart(uint16 value)
{
    /* Send MSB */
    Tx8_PutChar(HI8(value));
    
    /* Send LSB */
    Tx8_PutChar(LO8(value));
    
}

#endif
/* [] END OF FILE */
