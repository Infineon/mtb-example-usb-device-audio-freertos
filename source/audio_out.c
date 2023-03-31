/*******************************************************************************
* File Name: audio_out.c
*
*  Description: This file contains the Audio Out path configuration and
*               processing code
*
*******************************************************************************
* Copyright 2019-2023, Cypress Semiconductor Corporation (an Infineon company) or
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
#include "USB.h"
#include "USB_Audio.h"
#include "audio_out.h"
#include "audio_app.h"
#include "audio.h"
#include "cyhal.h"
#include "cycfg.h"
#include "rtos.h"


/*******************************************************************************
* Local Functions
*******************************************************************************/

/*******************************************************************************
* Audio Out Variables
*******************************************************************************/
/* PCM buffer data (16-bits) */
uint16_t audio_out_pcm_buffer_ping[MAX_AUDIO_OUT_PACKET_SIZE_WORDS];
uint16_t audio_out_pcm_buffer_pong[MAX_AUDIO_OUT_PACKET_SIZE_WORDS];

/* Audio IN flags */
volatile bool audio_out_is_streaming    = false;
volatile bool audio_start_streaming    = false;

TaskHandle_t rtos_audio_out_task = NULL;

/*******************************************************************************
* Function Name: audio_out_init
********************************************************************************
* Summary:
*   Initialize the audio OUT endpoint by setting up the DMAs involved and the
*   I2S block.
*
* Parameters:
*  None
*
* Return:
*  None
*******************************************************************************/
void audio_out_init(void)
{
    BaseType_t rtos_task_status;

    rtos_task_status = xTaskCreate(audio_out_process, "Audio Out Task",
                        RTOS_STACK_DEPTH, NULL, RTOS_TASK_PRIORITY,
                        &rtos_audio_out_task);

    if (pdPASS != rtos_task_status)
    {
        CY_ASSERT(0);
    }
    configASSERT(rtos_audio_out_task);  
}

/*******************************************************************************
* Function Name: audio_out_enable
********************************************************************************
* Summary:
*   Start a playing session.
*
* Parameters:
*  None
*
* Return:
*  None
*******************************************************************************/
void audio_out_enable(void)
{
#ifdef COMPONENT_AK4954A
    cy_rslt_t res;
    /* Run the I2S RX all the time */
    res = cyhal_i2s_start_rx(&i2s);
    
    if(res != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
#endif   
    audio_start_streaming = true;
}

/*******************************************************************************
* Function Name: audio_out_disable
********************************************************************************
* Summary:
*   Stop a playing session.
*
* Parameters:
*  None
*
* Return:
*  None
*******************************************************************************/
void audio_out_disable(void)
{
    cy_rslt_t res;

    /* Stop the I2S TX */
    res = cyhal_i2s_stop_tx(&i2s);
    if(res != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
#ifdef COMPONENT_AK4954A
    if (false == audio_in_is_recording)
    {
        res = cyhal_i2s_stop_rx(&i2s);
        if(res != CY_RSLT_SUCCESS)
        {
            CY_ASSERT(0);
        }
    }
#endif
    audio_out_is_streaming = false;
}

/*******************************************************************************
* Function Name: audio_out_process
********************************************************************************
* Summary:
*   Main task for the audio out endpoint. Check if it should start a playing
*   session.
*
* Parameters:
*  void *arg - arguments for audio input processing function
*
* Return:
*  None
*******************************************************************************/
void audio_out_process(void *arg)
{
    (void) arg;
    int32_t list_stat;

    list_stat = USBD_AUDIO_Start_Listen(usb_audio_context, NULL);
    configASSERT(0 == list_stat);
    USBD_AUDIO_Read_Task();
    while (1)
    {

    }
}

/*******************************************************************************
* Function Name: audio_out_endpoint_callback
********************************************************************************
* Summary:
*   Audio OUT endpoint callback implementation. It enables transfer of
*   audio frame from USB OUT endpoint buffer to I2S TX FIFO buffer.
 * Parameters:
 *  void * pUserContext -
 *  int NumBytesReceived -
 *  U8 ** ppNextBuffer -
 *  U32 * pNextBufferSize -
 *
 * Return:
 *  None
 *
*******************************************************************************/
void audio_out_endpoint_callback(void * pUserContext, int NumBytesReceived, U8 ** ppNextBuffer, U32 * pNextBufferSize)
{
    CY_UNUSED_PARAMETER(pUserContext);
    cy_rslt_t res;

    static uint16_t *audio_out_to_i2s_tx = NULL; 
    uint32_t data_to_write;

    if (audio_start_streaming)
    {
        audio_start_streaming = false;
        audio_out_is_streaming = true;

        /* Clear I2S TX FIFO */
        Cy_I2S_ClearTxFifo(i2s.base);
        /* Start the I2S TX if disabled */
        if (0U == (Cy_I2S_GetCurrentState(i2s.base) & CY_I2S_TX_START))
        {
            res = cyhal_i2s_start_tx(&i2s);
            if(res != CY_RSLT_SUCCESS)
            {
                CY_ASSERT(0);
            }
        }
        /* Clear Audio In buffer */
        memset(audio_out_pcm_buffer_ping, 0, (MAX_AUDIO_OUT_PACKET_SIZE_BYTES));
        audio_out_to_i2s_tx = audio_out_pcm_buffer_ping;
        /* Start a transfer to the Audio OUT endpoint */
        *ppNextBuffer = (uint8_t *) audio_out_to_i2s_tx;
    }
    else if(audio_out_is_streaming)
    {
        if(NumBytesReceived != 0)
        {
            data_to_write = NumBytesReceived / AUDIO_IN_SUB_FRAME_SIZE ;

            /* Write data to I2S Tx */
            res = cyhal_i2s_write(&i2s, audio_out_to_i2s_tx, (size_t *) &data_to_write);
            if(res != CY_RSLT_SUCCESS)
            {
                CY_ASSERT(0);
            }
            if (audio_out_to_i2s_tx == audio_out_pcm_buffer_ping)
            {
                audio_out_to_i2s_tx = audio_out_pcm_buffer_pong;
            }
            else
            {
                audio_out_to_i2s_tx = audio_out_pcm_buffer_ping;
            }
             /* Start a transfer to OUT endpoint */
            *ppNextBuffer = (uint8_t *) audio_out_to_i2s_tx;
        }
    }
}
/* [] END OF FILE */
