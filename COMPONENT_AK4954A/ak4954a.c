/*******************************************************************************
* File Name: ak4954a.c
*
* Description: This file contains the AK4954a codec control APIs.
*
*******************************************************************************
* Copyright 2023-2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#include "ak4954a.h"
#include "stdbool.h"

#define I2C_WRITE_OPERATION        (0x00)

ak4954a_transmit_callback   ak4954a_transmit;

/*******************************************************************************
* Function Name: ak4954a_init
********************************************************************************
* Summary:
*   Initializes the codec with default settings.
*
* Parameters:  
*    None
*
* Return:
*   uint32_t - I2C master transaction error status 
*
*******************************************************************************/
uint32_t ak4954a_init(ak4954a_transmit_callback callback)
{
    uint32_t ret;
    
    ak4954a_transmit = callback;
   
    /* Clear Power Management 1 register (dummy write) */
    ret = ak4954a_transmit(AK4954A_REG_PWR_MGMT1, 0x00);
    if (ret) return ret;

    /* Clear Power Management 1 register */
    ret = ak4954a_transmit(AK4954A_REG_PWR_MGMT1, 0x00);
    if (ret) return ret;
    
    /* Set the data alignment */
    ret = ak4954a_transmit(AK4954A_REG_MODE_CTRL1, AK4954A_DEF_DATA_ALIGNMENT);
    if (ret) return ret;
    
    /* Set sample rate */
    ret = ak4954a_transmit(AK4954A_REG_MODE_CTRL2, AK4954A_DEF_SAMPLING_RATE |
                                                   AK4954A_MODE_CTRL2_FS_48kHz);
    if (ret) return ret;
    
    /* Set MPWR pin Power Management */
    ret = ak4954a_transmit(AK4954A_REG_SIG_SEL1, AK4954A_SIG_SEL1_PMMP |
                                                 AK4954A_SIG_SEL1_MGAIN_6dB);
    if (ret) return ret;
    
    /* Clear Digital Filter Mode register */
    ret = ak4954a_transmit(AK4954A_REG_DIG_FILT_MODE, 0x00);
    if (ret) return ret;

    /* Mute right channel [Not used] */
    ret = ak4954a_transmit(AK4954A_REG_MODE_CTRL3, 0x00);
    if (ret) return ret;
    ret = ak4954a_transmit(AK4954A_REG_RCH_IN_VOL, 0x00);
    if (ret) return ret;
    ret = ak4954a_transmit(AK4954A_REG_LCH_IN_VOL, 0x00);

    return ret;  
}

/*******************************************************************************
* Function Name: ak4954a_adjust_volume
********************************************************************************
* Summary:
*   This function updates the volume of both the left and right channels of the
*     headphone output.
*
*
* Parameters:  
*    volume - Steps of 0.5dB, where:
*            Minimum volume: -65.5dB (0x8F)
*            Maximum volume:  +6.0dB (0x00)
*            Mute: (0x90~0xFF)
*
* Return:
*   uint32_t - I2C master transaction error status
*
*******************************************************************************/
uint32_t ak4954a_adjust_volume(uint8_t volume)
{
    uint32_t ret;
        
    ret = ak4954a_transmit(AK4954A_REG_LCH_DIG_VOL, volume);
    if (ret) return ret;
    return ak4954a_transmit(AK4954A_REG_RCH_DIG_VOL, volume);
}

/*******************************************************************************
* Function Name: ak4954a_activate
********************************************************************************
* Summary:
*   Activates the codec - This function is called in conjunction with 
*   ak4954A_deactivate API after successful configuration update of the codec.
*
* Parameters:  
*    None
*
* Return:
*   uint32_t - I2C master transaction error status
*
*******************************************************************************/
uint32_t ak4954a_activate(void)
{
    uint32_t ret;
    
    /* Enable Power Management DAC */
    ret = ak4954a_transmit(AK4954A_REG_PWR_MGMT1,
                           AK4954A_PWR_MGMT1_PMDAC | AK4954A_PWR_MGMT1_PMVCM |
                           AK4954A_PWR_MGMT1_PMADL | AK4954A_PWR_MGMT1_PMADR |
                           AK4954A_PWR_MGMT1_PMPFIL);
    if (ret) return ret;
    
    /* Enable Left/Right Channels */
    ret = ak4954a_transmit(AK4954A_REG_PWR_MGMT2, 
                           AK4954A_PWR_MGMT2_PMHPL | AK4954A_PWR_MGMT2_PMHPR);
    return ret;
}

/*******************************************************************************
* Function Name: ak4954a_deactivate
********************************************************************************
* Summary:
*   Deactivates the codec - the configuration is retained, just the codec 
*   input/outputs are disabled. The function should be called before changing 
*   any setting in the codec over I2C
*
* Parameters:  
*    None
*
* Return:
*   uint32_t - I2C master transaction error status
*
*******************************************************************************/
uint32_t ak4954a_deactivate(void)
{
    uint32_t ret;
   
    /* Disable Left/Right Channels */
    ret = ak4954a_transmit(AK4954A_REG_PWR_MGMT2, 0x00);
    
    /* Disable Power Management DAC */
    ret = ak4954a_transmit(AK4954A_REG_PWR_MGMT1, AK4954A_PWR_MGMT1_PMVCM);
    return ret;
}

/* [] END OF FILE */
