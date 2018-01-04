/***************************************************************************//**
* @file    BrightnessControl.h
* @version 1.0
* @authors PRIA
*
* @par Description
*    This file contains the brightness control code for Proximity_Distance_040 
*    example project provided with Application note AN92239 "Proximity Sensing  
*    with CapSense".
*
* @par Notes
*    - 
*
* @par Hardware Dependency
*    None
*
* @par References
*    1. 001-92239: AN92239 "Proximity Sensing with CapSense" 
*
* @par Code Tested With
*    1. Silicon: PSoC4000
*    2. IDE: PSoC Creator 3.0 SP1 (3.0.0.3023)
*    3. Compiler: GCC 4.7.3, ARM-MDK 4.54.0.0 (armcc 4.1 b894)
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
#ifndef __BRIGHTNESSCONTROL_H    /* Guard to prevent multiple inclusions */
#define __BRIGHTNESSCONTROL_H
    
/*******************************************************************************
*   Included Headers
*******************************************************************************/
#include <cytypes.h>

/*******************************************************************************
*   Macros and #define Constants
*******************************************************************************/
#define MIN_BRIGHTNESS              (3u)
#define MAX_BRIGHTNESS              (200u)
#define OFF_STATE_BRIGHNTESS        (0u)    
#define BRIGHTNESS_STEP             (1u)
#define SIGNAL_CHANGE_STEP          (10u)
#define INITIAL_SIGNAL              (35u)
#define BRIGHTNESS_LEVELS           ((MAX_BRIGHTNESS - MIN_BRIGHTNESS) / BRIGHTNESS_STEP)
    
#endif /* #ifndef __BRIGHTNESSCONTROL_H */
