/***************************************************************************//**
* @file    main.c
* @version 1.0
* @authors PRIA
*
* @par Description
*    This file contains the main source code for Proximity_Distance_042 example
*    project provided with Application note AN92239 "Proximity Sensing with 
*    CapSense".
*
*    This example project showcases the detection of approaching or receding 
*    hand by controlling LED intensities based on amplitude of signal observed  
*    on proximity sensor.
*
* @par Notes
*    - 
*
* @par Hardware Dependency
*    1. CY8CKIT-042 PSoC4 Pioneer Kit
*    2. CY8CKIT-024 CapSense Proximity Shield
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
#include "CapSenseFilters.h"
#include "BrightnessControl.h"

/*******************************************************************************
*   Macros and #define Constants
*******************************************************************************/

/* Macro to enable CapSense Tuner (for tuning the sensors) */
#define TUNER_ENABLE                (0)

/* Macro to enable Tx8 data out on P0[1] */
/* Note the following before enabling Tx8:
 *  1. In CY8CKIT-042, P0[1] on header J2, has to be manually connected to
 *     P12[6] on header J8. 
 *     P12[6] on header J8 is the Rx pin of onboard USB-UART hardware. Since  
 *     this pin is not hardwired to any of the PSoC4A pins, a manual connection 
 *     is required between PSoC4A Tx8 pin and P12[6] on J8.
 *     Refer AN92239 "Proximity Sensing with CapSense" for more details on 
 *     using Tx8 output to view data on Bridge Control Panel.
 *  2. Enabling Tx8 leads to visible flickering in LEDs when LEDs are ON. 
 *     This is because the Tx8 data is sent using software bit banging.
 *     The "SW Tx UART" component disables global interrupts to maintain bit 
 *     timings on the Tx8 bus. This delays the timer ISR, hence leading to 
 *     observable flicker in LEDs.
 */
#define TX8_ENABLE                  (0)

#if TX8_ENABLE     
    /* Macro to calculate number of samples in an array */
    #define NO_OF_SAMPLES(array)	(sizeof(array)/sizeof(array[0]))   
    
    /* Total number of sensors for which the data has to be sent to Tx8 output */
    #define NO_OF_SENSORS_ON_TX8    (1)
#endif /* #if TX8_ENABLE*/

/* Positive and Negative threshold used by ALP filter. 
 * Refer AN92239 for details on how to tune positive and negative thresholds.
 */
#define PROX_POS_THRESHOLD          (90u)
#define PROX_NEG_THRESHOLD          (90u)


/*******************************************************************************
*   Global Variables and Constant Declarations with Applicable Initializations
*******************************************************************************/
#if TX8_ENABLE
    /* Tx8 Data Packet header and tail */
    const uint8 header[] =    {0x0Du, 0x0Au};      /* Header = CR, LF */
    const uint8 tail[]   =    {0x00u, 0xFFu, 0xFFu};  /* Tail = 0x00, 0xFF, 0xFF */
#endif /* #if TX8_ENABLE*/

/* variable that controls LED brightness by controlling PWM duty cycle */   
volatile uint8 brightnessCompare;

/* ALP Filter variables */
/* Thresholds for adjusting ALP filter response */
uint8 proxPositiveTh[CapSense_TOTAL_PROX_SENSOR_COUNT];
uint8 proxNegativeTh[CapSense_TOTAL_PROX_SENSOR_COUNT];            
            
/*Average Filter history */
uint16 averageHistory[CapSense_TOTAL_PROX_SENSOR_COUNT][AVERAGE_HISTORY_SIZE]; 

/* Indicates the oldest data amongst Average Filter histories */
uint8 avgOldestDataIndex[CapSense_TOTAL_PROX_SENSOR_COUNT] = {0};

/* State variable for ALP filter state machine */													
CAPSENSEFILTERS_ALP_STATE_ENUM stateAdvancedFilter[CapSense_TOTAL_PROX_SENSOR_COUNT];  

/* Average Filter output */													
uint16 averageVal[CapSense_TOTAL_PROX_SENSOR_COUNT];  

/* Following variables contain input histories for ALP filter */
uint16 filterHistory0[CapSense_TOTAL_PROX_SENSOR_COUNT];
uint16 filterHistory1[CapSense_TOTAL_PROX_SENSOR_COUNT];	
uint16 filterHistory2[CapSense_TOTAL_PROX_SENSOR_COUNT];

/*******************************************************************************
*   Function Declarations
*******************************************************************************/

#if TX8_ENABLE
    void SendUint16ToTx8Out(uint16 value);
    void SendDataToTx8Out(uint32 noOfSensors);
#endif /* #if TX8_ENABLE*/

/* Declaring Interrupt Service Routine to be used for for Timer ISR */
CY_ISR_PROTO(TimerIsr);

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
*    This main code scans a proximity sensor at 16 bit resolution, applies 
*    the advanced low pass filter on the sensor raw counts and varies
*    the intensities of LEDs LED1 through LED5 between MIN_BRIGHTNESS and 
*    MAX_BRIGHTNESS value in steps of BRIGHTNESS_STEP based on change in signal 
*    observed on proximity sensor.
*
* @par Notes
*    None
*
**//***************************************************************************/
int main()
{
    /* Following variables are not required while using tuner for setting 
     * tuning parameters, hence it is enclosed with "#if !TUNER_ENABLE, #endif".
     */
#if !TUNER_ENABLE    
    /* diffCount variable is used to store the difference counts between
     * rawCounts and baseline, when a signal is detected on the sensor. 
     * LED brightness is controlled based on amplitude of this value.
     */
    uint16 diffCount; 

    /* variable used as an iteration counter for "for loop" that controls LED
     * brightness
     */
    uint8 iteration;
#endif /* #if !TUNER_ENABLE */

    /* Enable global interrupts. Following blocks generate interrupts 
     *      CapSense:   Generates interrupt when sensor scan completes
     *      TimerIsr:   This interrupt is connected to Timer component and gets 
     *                  triggered on each terminal count. Note that timer is 
     *                  used to generate software controlled PWMs for LED 
     *                  brightness control.
     *      EzI2C:      The EZI2C Slave is a unique implementation of an I2C  
     *                  slave in that all communication between the master and
     *                  slave is handled in the ISR (Interrupt Service Routine).
     */
    CyGlobalIntEnable;
    
    /* Set positive and negative thresholds used by ALP filter. 
     * Refer AN92239 for details on positive and negative thresholds.
     */
    proxPositiveTh[CapSense_SENSOR_PROX_0__PROX] = PROX_POS_THRESHOLD;
    proxNegativeTh[CapSense_SENSOR_PROX_0__PROX] = PROX_NEG_THRESHOLD;
   
    /* Proximity sensors are disabled by default, due to their long scan times. 
     * These need to be explicitly enabled by calling following API.
     */
    CapSense_EnableWidget(CapSense_PROX__PROX);

    /* In tuner mode, use CapSense tuner APIs to communicate with CapSense
     * Tuner GUI. Tuner GUI will be used to set tuning parameters for CapSense
     * component, as explaind in AN92239 "Proximity Sensing with CapSense".
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
    
    /* Set resolution of proximity sensor to 16 bit for maximum possible 
     * proximity sensing distance
     */
    CapSense_SetScanResolution(
                                CapSense_PROX__PROX,
                                CapSense_RESOLUTION_16_BITS
                               );
    
    /* Scan sensors and initialize baselines */
    CapSense_InitializeSensorBaseline(CapSense_SENSOR_PROX_0__PROX);
    
    /* Initialize Advanced Low Pass (ALP) filter with current raw count 
     * value, and set the k value for ALP filter. 
     * Note: Refer AN92239 for details on how to select appropriate k value
     * for the filter.
     */ 
    CapSenseFilters_InitializeAdvancedLowPass(
                        CapSense_SENSOR_PROX_0__PROX,
                        CapSense_ReadSensorRaw(CapSense_SENSOR_PROX_0__PROX)
                                              );
    CapSenseFilters_SetAdvancedLowPassK(CAPSENSEFILTERS_IIR_K_32);
    
    /* Start timer and isr components for LED brightness control */
    Timer_Start();
    
    /* Enable Timer Isr and set the interrupt address to TimerIsr function
     * defined in this file.
     */
    TimerIsr_StartEx(&TimerIsr);
  
    for(;;)
    {          
        /* Scan sensor and wait till the scan completes */
        CapSense_ScanSensor(CapSense_SENSOR_PROX_0__PROX);
        while(CapSense_IsBusy())
        {           
            /* Wait until scan is complete */    
        }    
        
        /* Apply ALP filter on proximity sensors */
        CapSenseFilters_RunAdvancedLowPass(CapSense_SENSOR_PROX_0__PROX);
        
        /* Update baselines*/
        CapSense_UpdateEnabledBaselines();
        
        /* Check if sensor is active. If so, find the difference counts 
         * and change LEDs' brightness based on amplitude of difference 
         * counts
         */
        if(CapSense_CheckIsSensorActive(CapSense_SENSOR_PROX_0__PROX))
        {
            diffCount = CapSense_GetDiffCountData(CapSense_SENSOR_PROX_0__PROX);
            for(iteration = 1; iteration <= BRIGHTNESS_LEVELS; iteration++)
            {
                if(diffCount < (INITIAL_SIGNAL + (SIGNAL_CHANGE_STEP * iteration)))
                {
                    /* Load global variable brightness Compare used by TimerIsr
                     * to control LED brightness.
                     */
                    brightnessCompare = BRIGHTNESS_STEP * iteration + MIN_BRIGHTNESS;
                    break;
                }
            }
        }
        else
        {
            /* Load global variable brightness Compare used by TimerIsr to 
             * control LED brightness.
             */
            brightnessCompare = OFF_STATE_BRIGHNTESS;
        }    
        
        /* Send data to Tx8 */
#if TX8_ENABLE
        SendDataToTx8Out(NO_OF_SENSORS_ON_TX8); 
#endif /* #if TX8_ENABLE*/       
         
    }
#endif /* #if TUNER_ENABLE */       
}

/*******************************************************************************
* Function Name: SendDataToTx8Out
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
*       a. rawcounts
*       b. average value (this is calculated by ALP filter)
*       c. baseline
*       d. average value (same as above, sent again to complete 3x words)
*       e. difference counts
*       f. State of ALP filter state machine
*
* @par Notes
*    None
*
**//***************************************************************************/
#if TX8_ENABLE                  
void SendDataToTx8Out(uint32 noOfSensors)
{
    uint8 sensor;
    
    /* Send data packet header */
    Tx8_PutArray(header, NO_OF_SAMPLES(header));
    
    /* Send 3x words(uint16) of data (as expected by multichart) */
     
    /* Send RawCounts and average value. These are labelled as Raw Count 0  
     * to Raw Count (noOfSensors-1) and Raw Count noOfSensors to
     * Raw Count (2*noOfSensors - 1) respectively in multichart
     *
     */
    for(sensor= 0; sensor < noOfSensors; sensor++)
    {
        SendUint16ToTx8Out(CapSense_ReadSensorRaw(sensor));
    }
    for(sensor= 0; sensor < noOfSensors; sensor++)
    {
        SendUint16ToTx8Out(CapSenseFilters_GetAverageData(sensor));
    }
    
    /* Send Baselines and ON/OFF status. These are labelled as Baseline 0  
     * to Baseline (noOfSensors-1) and Baseline noOfSensors to
     * Baseline (2*noOfSensors - 1) respectively in multichart
     */
    for(sensor= 0; sensor < noOfSensors; sensor++)
    {
        SendUint16ToTx8Out(CapSense_GetBaselineData(sensor));
    }
    for(sensor= 0; sensor < noOfSensors; sensor++)
    {
        SendUint16ToTx8Out((uint16)CapSense_CheckIsSensorActive(CapSense_SENSOR_PROX_0__PROX));
    }
   
    /* Send difference counts and ALP state values. These are labelled as 
     * Signal 0  to Signal (noOfSensors-1) and Signal noOfSensors to
     * Signal (2*noOfSensors - 1) respectively in multichart
     */
    for(sensor= 0; sensor < noOfSensors; sensor++)
    {
        SendUint16ToTx8Out(CapSense_GetDiffCountData(sensor));
    }
    for(sensor= 0; sensor < noOfSensors; sensor++)
    {
        SendUint16ToTx8Out(CapSenseFilters_GetALPFilterState(sensor));
    }

    /* Send the Tail as expected by multichart */
    Tx8_PutArray(tail, NO_OF_SAMPLES(tail));
}

void SendUint16ToTx8Out(uint16 value)
{
    /* Send MSB */
    Tx8_PutChar(HI8(value));
    
    /* Send LSB */
    Tx8_PutChar(LO8(value));
    
}
#endif /* #if TX8_ENABLE   */

/*******************************************************************************
* Function Name: TimerIsr
****************************************************************************//**
* @par Summary
*    This function is the Interrupt Service Routine for Timer interrupt.
*    This function drives the LEDs ON for brightnessCompare number of isr cycles
*    amongst every MAX_BRIGHTNESS number of ISR cycles.
*
* @return
*    None
*
* @param[in] 
*    None
*
* @par Theory of Operation
*    A static variable, count is used to keep track of how many times the ISR 
*    has been entered. 
*    LED pins are driven low (for active low LEDs) for brightnessCompare number
*    of times and then driven high for (MAX_BRIGHTNESS - brightnessCompare) 
*    number of times. Count is then reset and above cycle keeps repeating. 
*    Brightness can be varied by changing the value of brightnessCompare 
*    variable in the main code.
*
* @par Notes
*    None
*
**//***************************************************************************/
CY_ISR(TimerIsr)
{
    /* ISR execution time for this project is as follows:
     * GCC 4.7.3: 13.9us
     * MDK 4.54:  9.5us
     * Note that above timings are measured on silicon (using CY8CKIT-042).
	 * These timings may vary across different samples based on IMO variation.
	 *
	 * Also note that ISR execution time will change if CPU clock or compiler 
     * optimization settings are changed. 
     */
	 
    /* count variable is used to control frequency and duty cycle of PWM on 
     * LED pins 
     */
    static uint8 count = 0u;

    /* increment count every time the ISR is entered */
    count++;
    
    /* control LED brightness by driving LEDs ON for brightnessCompare 
     * number of isr cycles amongst every MAX_BRIGHTNESS number of ISR cycles.
     */
    if(count < brightnessCompare)
    {
        /* Note that the cypins i.e. the per pin APIs are used instead of
         * the pin component APIS, since cypins APIs are actually macros
         * rather than function calls as generated by pin component. 
         * Per general coding practices, function calls should be avoided 
         * inside ISRs.
         */
        
        /* Drive LEDs ON */
        CY_SYS_PINS_CLEAR_PIN(LED1__DR, LED1__SHIFT);
        CY_SYS_PINS_CLEAR_PIN(LED2__DR, LED2__SHIFT);
        CY_SYS_PINS_CLEAR_PIN(LED3__DR, LED3__SHIFT);
        CY_SYS_PINS_CLEAR_PIN(LED4__DR, LED4__SHIFT);
        CY_SYS_PINS_CLEAR_PIN(LED5__DR, LED5__SHIFT);
    }
    else
    {
        /* Drive LEDs OFF */
        CY_SYS_PINS_SET_PIN(LED1__DR, LED1__SHIFT);
        CY_SYS_PINS_SET_PIN(LED2__DR, LED2__SHIFT);
        CY_SYS_PINS_SET_PIN(LED3__DR, LED3__SHIFT);
        CY_SYS_PINS_SET_PIN(LED4__DR, LED4__SHIFT);
        CY_SYS_PINS_SET_PIN(LED5__DR, LED5__SHIFT); 
        
        if(count > MAX_BRIGHTNESS)
        {
            /* One PWM Cycle complete, reset count */
            count = 0u;
        }    
    }
    
    /* Interrupts are sticky, thus need to be manually cleared everytime an 
     * isr is entered.
     */
    Timer_ClearInterrupt(Timer_INTR_MASK_TC);
}

/* [] END OF FILE */
