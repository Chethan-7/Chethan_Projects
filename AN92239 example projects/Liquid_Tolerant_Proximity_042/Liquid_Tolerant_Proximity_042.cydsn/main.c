/***************************************************************************//**
* @file    main.c
* @version 1.0
* @authors PRIA
*
* @par Description
*    This file contains the main source code for Liquid_Tolerant_Proximity_042 
*    example project provided with Application note AN92239 "Proximity Sensing  
*    With CapSense". 
*
* @par Notes
*    Liquid tolerance requires a special method of sensor tuning i.e. the 
*    CapSense component resolution and threshold parameters need to be tuned
*    differently compared to a project which does not require liquid tolerance.
*    Refer Section "Tuning for Liquid Tolerance" in AN92239 for details on
*    tuning process. 
*
* @par Hardware Dependency
*    1. CY8CKIT-042 PSoC4 Pioneer Kit
*    2. CY8CKIT-024 CapSense Proximity Shield
*
* @par References
*    1. 001-92239: AN92239 "Proximity Sensing With CapSense" 
*
* @par Code Tested With
*    1. Silicon: PSoC4100/4200
*    2. IDE: PSoC Creator 3.0 SP1 (3.0.0.3023)
*    3. Compiler: GCC 4.7.3, ARM MDK 4.54.0.0 (armcc 4.1 b894)
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

/*******************************************************************************
*   Macros and #define Constants
*******************************************************************************/
/* Macro to enable CapSense Tuner (for tuning the sensors) */
#define TUNER_ENABLE                (0)

/* Macro to enable Tx8 data out on P0[1] */
/* Note that for CY8CKIT-042, P0[1] on header J2, has to be manually connected
 * to P12[6] on header J8. 
 * P12[6] on header J8 is the Rx pin of onboard USB-UART hardware. Since this 
 * pin is not hardwired to any of the PSoC4A pins, a manual connection is
 * required between PSoC4A Tx8 pin and P12[6] on J8.
 * Refer AN92239 "Proximity Sensing With CapSense" for more details on 
 * using Tx8 output to view data on Bridge Control Panel.
 */
#define TX8_ENABLE                  (1)
#define NO_OF_SENSORS_ON_TX8        (1)

#if TX8_ENABLE
    #define NO_OF_SAMPLES(array)	    (sizeof(array)/sizeof(array[0]))   
#endif

/*******************************************************************************
*   Data Type Definitions
*******************************************************************************/
/* Following enum is used by WriteAllLeds function */
typedef enum
{
	LED_ON		= 0x00u,
	LED_OFF 	= 0x01u
} LED_ONOFF;

/*******************************************************************************
*   Module Variables and Constant Declarations with Applicable Initializations
*******************************************************************************/
#if TX8_ENABLE
    /* Tx8 Data Packet header and tail */
    const uint8 header[] =    {0x0Du, 0x0Au};      /* Header = CR, LF */
    const uint8 tail[]   =    {0x00u, 0xFFu, 0xFFu};  /* Tail = 0x00, 0xFF, 0xFF */
#endif

/*******************************************************************************
*   Function Declarations
*******************************************************************************/
void WriteAllLeds(LED_ONOFF onOffValue);

/* Tx8 related functions */
#if TX8_ENABLE
    void SendUint16ToTx8(uint16 value);
    void SendDataToTx8(uint32 noOfSensors);
#endif

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
*    This main code scans a proximity sensor and turns ON LED1-LED5 if the 
*    proximity sensor is active.
*
* @par Notes
*    None
*
**//***************************************************************************/
int main()
{    
    /* Enable global interrupts. Following blocks generate interrupts 
     *      CapSense: Generates interrupt when sensor scan completes
     *      EzI2C:    The EZI2C Slave is a unique implementation of an I2C slave 
     *                in that all communication between the master and slave is 
     *                handled in the ISR (Interrupt Service Routine).
     */
    CyGlobalIntEnable;
   
    /* Proximity sensors are disabled by default, due to their long scan times. 
     * These need to be explicitly enabled by calling following API.
     */
    CapSense_EnableWidget(CapSense_PROX__PROX);
    
    /* In tuner mode, use CapSense tuner APIs to communicate with CapSense
     * Tuner GUI. Tuner GUI will be used to set tuning parameters for CapSense
     * component, as explained in AN92239 "Proximity Sensing With CapSense".
     * No other functionality is enabled in tuner mode. 
     */  
#if TUNER_ENABLE
    
    /* Start Tuner operation */
    CapSense_TunerStart();
    
    /* Communicate with CapSense Tuner*/
    while(1)
    {
        CapSense_TunerComm();
    }
#else  

    /* Start CapSense component */
    CapSense_Start();
    
    /* Scan sensors and initialize baselines */
    CapSense_InitializeSensorBaseline(CapSense_SENSOR_PROX_0__PROX);
  
    for(;;)
    {          
        /* Scan sensor and wait till the scan completes */
        CapSense_ScanSensor(CapSense_SENSOR_PROX_0__PROX);
        while(CapSense_IsBusy())
        {
            /* Wait until scan completes */   
        }
        
       /* Update baselines*/
        CapSense_UpdateEnabledBaselines();
        
        /* Turn ON LEDs if sensor is active, else turn off all LEDs */
        if(CapSense_CheckIsSensorActive(CapSense_SENSOR_PROX_0__PROX))
        {
            WriteAllLeds(LED_ON);
        }
        else
        {
            WriteAllLeds(LED_OFF);
        }    
        
        /* Send data to Tx8 */
#if TX8_ENABLE
        SendDataToTx8(NO_OF_SENSORS_ON_TX8); 
#endif /* #if TX8_ENABLE */            
    }
    
#endif /* #if TUNER_ENABLE */ 

}

/*******************************************************************************
* Function Name: SendDataToTx8
****************************************************************************//**
* @par Summary
*    This function sends data of first "noOfSensors" number of sensors to Tx8
*    in multichart format (AN2397). The same data can be observed on bridge 
*    control panel as well (AN92239).
*
* @return
*    None
*
* @param[in] 
*    noOfSensors: number of sensors for which the data has to be sent to Tx8
*
* @par Theory of Operation
*    Multichart tool expects a header followed by 3x number of data 
*    words (16 bit, big endian) and a pre-defined tail. This function sends 
*    following data:
*       a. header[] array as header
*       b. rawcounts of noOfSensors
*       c. baselines of noOfSensors
*       d. difference counts of noOfSensors
*       e. tail[] array as a tail
*
* @par Notes
*    None
*
**//***************************************************************************/
#if TX8_ENABLE                  
void SendDataToTx8(uint32 noOfSensors)
{
    uint8 sensor;
    
    /* Send data packet header */
    Tx8_PutArray(header, NO_OF_SAMPLES(header));
    
    /* Send 3x words(uint16) of data (as expected by multichart) */
     
    /* Send RawCounts */
    for(sensor= 0; sensor < noOfSensors; sensor++)
    {
        SendUint16ToTx8(CapSense_ReadSensorRaw(sensor));
    }
    
    /* Send Baselines */
    for(sensor= 0; sensor < noOfSensors; sensor++)
    {
        SendUint16ToTx8(CapSense_GetBaselineData(sensor));
    }
   
    /* Send difference counts */
    for(sensor= 0; sensor < noOfSensors; sensor++)
    {
        SendUint16ToTx8(CapSense_GetDiffCountData(sensor));
    }
   
    /* Send the Tail as expected by multichart */
    Tx8_PutArray(tail, NO_OF_SAMPLES(tail));
}

/*******************************************************************************
* Function Name: SendUint16ToTx8
****************************************************************************//**
* @par Summary
*    This function sends a 16 bit value to Tx8 in Big Endian format.
*
* @return
*    None
*
* @param[in] 
*    value: 16 bit integer to be sent to Tx8
*
* @par Theory of Operation
*    None
*
* @par Notes
*    None
*
**//***************************************************************************/
void SendUint16ToTx8(uint16 value)
{
    /* Send MSB */
    Tx8_PutChar(HI8(value));
    
    /* Send LSB */
    Tx8_PutChar(LO8(value));
    
}
#endif /* #if TX8_ENABLE   */

/*******************************************************************************
* Function Name: WriteAllLeds
****************************************************************************//**
* @par Summary
*    This function turns LED1 - LED5 on or off based on input parameter
*
* @return
*    None
*
* @param[in] 
*    onOffValue: enumerated value to indicate whether to turn all LEDsON or OFF
*
* @par Theory of Operation
*    None
*
* @par Notes
*    None
*
**//***************************************************************************/
void WriteAllLeds(LED_ONOFF onOffValue)
{
    LED1_Write(onOffValue);
    LED2_Write(onOffValue);
    LED3_Write(onOffValue);
    LED4_Write(onOffValue);
    LED5_Write(onOffValue);
}

/* [] END OF FILE */
