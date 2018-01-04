/***************************************************************************//**
* @file    CyBitOperations.h
* @version 1.0
* @authors UDYG, PRIA
*
* @par Description
*   This file contains the definition of macro utilities that can be used for
*   bitwise operations.
*
* @par Notes
*    None
*    
* @par Hardware Dependency
*    None
*
* @par Code Tested With
*    1. Silicon: PSoC4000
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

#ifndef __CYBIT_OPERATIONS_H    /* Guard to prevent multiple inclusions */
#define __CYBIT_OPERATIONS_H


/*******************************************************************************
*   Included Headers
*******************************************************************************/
#include <cytypes.h>	

/*******************************************************************************
*   Macro Definitions
*******************************************************************************/

/* Macro to get LSbit from any value */
#define CY_GET_LSBIT(value)         ((value) & 1u)

/* Macro to get MSbit of 8-bit value */
#define CY_GET_MSBIT_8BIT(value)    (((uint8)(value) >> 7u) & 1u)

/* Macro to combine two 8-bit values to form 16-bit value */
#define CY_CONCATENATE_TWO_8BIT(msbyte, lsbyte)        (((uint16)(msbyte) << 8u) | (uint8)(lsbyte))

/* Macro to obtain bit position mask for a given bit */
#define CY_BITPOS_MASK_UINT32(bitPosition) (((uint32) 1) << (bitPosition))

/* Macro to set a particular bit in a given input parameter */
#define CY_SET_BITPOS_UINT32(value, bitPosition) {(value) |= CY_BITPOS_MASK_UINT32(bitPosition);}

/* Macro to clear a particular bit in a given input parameter */
#define CY_CLEAR_BITPOS_UINT32(value, bitPosition) {(value) &= ~CY_BITPOS_MASK_UINT32(bitPosition);}

/* Macro to invert a particular bit in a given input parameter */
#define CY_INVERT_BITPOS_UINT32(value, bitPosition) {(value) ^= CY_BITPOS_MASK_UINT32(bitPosition);}

/* Macro to check if a given bit is set in a given input parameter */
#define CY_IS_BITPOS_SET_UINT32(value, bitPosition) (0 != ((value) & CY_BITPOS_MASK_UINT32(bitPosition)))

/* Macro to check if a given bit is cleared in a given input parameter */
#define CY_IS_BITPOS_CLEAR_UINT32(value, bitPosition) (!CY_IS_BITPOS_SET_UINT32(value, bitPosition))

#endif /* #ifndef __CYBIT_OPERATIONS_H */


/* [] END OF FILE */
