/*******************************************************************************
* File Name: audio_in.c
*
*  Description: This file contains the Audio In path configuration and
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
#include "audio.h"
#include "audio_in.h"
#include "rtos.h"
#include "cyhal.h"
#include "audio_app.h"
#include "cybsp.h"

/*******************************************************************************
* Local Functions
*******************************************************************************/

/*******************************************************************************
* Audio In Variables
*******************************************************************************/
/* PCM buffer data (16-bits) */
uint16_t audio_in_pcm_buffer_ping[MAX_AUDIO_IN_PACKET_SIZE_WORDS];
uint16_t audio_in_pcm_buffer_pong[MAX_AUDIO_IN_PACKET_SIZE_WORDS];

/* Audio IN flags */
volatile bool audio_in_is_recording = false;
volatile bool audio_start_recording = false;

TaskHandle_t rtos_audio_in_task = NULL;

/*****************************************************************************
* Static const data
*****************************************************************************/
const unsigned char silent_frame[MAX_AUDIO_IN_PACKET_SIZE_BYTES] = {0};


/*******************************************************************************
* Function Name: audio_in_init
********************************************************************************
* Summary:
*   Schedules task for IN endpoint.
*
* Parameters:
*  None
*
* Return:
*  None
*******************************************************************************/
void audio_in_init(void)
{
    BaseType_t rtos_task_status;

    rtos_task_status = xTaskCreate(audio_in_process, "Audio In Task",
        RTOS_STACK_DEPTH, NULL, RTOS_TASK_PRIORITY,
        &rtos_audio_in_task);
    if (pdPASS != rtos_task_status)
    {
        CY_ASSERT(0);
    }
    configASSERT(rtos_audio_in_task);
}

/*******************************************************************************
* Function Name: audio_in_enable
********************************************************************************
* Summary:
*   Start a recording session.
*
* Parameters:
*  None
*
* Return:
*  None
*******************************************************************************/
void audio_in_enable(void)
{
    cy_rslt_t res;

    /* Run the I2S RX all the time */
    res = cyhal_i2s_start_rx(&i2s);
    if(res != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
    audio_start_recording = true;
    cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_ON);
}


/*******************************************************************************
* Function Name: audio_in_disable
********************************************************************************
* Summary:
*   Stop a recording session.
*
* Parameters:
*  None
*
* Return:
*  None
*******************************************************************************/
void audio_in_disable(void)
{
    cy_rslt_t res;

    if (false == audio_out_is_streaming)
    {
        res = cyhal_i2s_stop_rx(&i2s);
        if(res != CY_RSLT_SUCCESS)
        {
            CY_ASSERT(0);
        }
    }
    audio_in_is_recording = false;
    cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_OFF);
}



/*******************************************************************************
* Function Name: audio_in_process
********************************************************************************
* Summary:
*   Main task for the audio in endpoint.
*
* Parameters:
*  void *arg - arguments for audio input processing function
*
* Return:
*  None
*******************************************************************************/
void audio_in_process(void *arg)
{
    (void) arg;

    USBD_AUDIO_Write_Task();
    while(1)
    {

    }
}


/*******************************************************************************
* Function Name: audio_in_endpoint_callback
********************************************************************************
* Summary:
*   Audio in endpoint callback implementation. It enables the transfer of
*   audio frame from I2S RX FIFIO to USB IN endpoint buffer.
 * Parameters:
 *  void * pUserContext -
 *  U8 ** ppNextBuffer -
 *  U32 * pNextPacketSize -
 *
 * Return:
 *  None
 *
*******************************************************************************/
void audio_in_endpoint_callback(void * pUserContext, const U8 ** ppNextBuffer, U32 * pNextPacketSize)
{
    unsigned int sample_size;
    size_t audio_in_count;
    static uint16_t *audio_in_pcm_buffer = NULL;
    cy_rslt_t res;

    CY_UNUSED_PARAMETER(pUserContext);
    /*
     * Packet size minus the two additional audio frames reserved in PACKET_SIZE_IN_MAX.
     * The application can periodically increase SampleSize to counterbalance differences between the
     * regular sample size and the actual byte rate.
     */
    sample_size = ((MAX_AUDIO_IN_PACKET_SIZE_BYTES) - (ADDITIONAL_AUDIO_IN_SAMPLE_SIZE_BYTES));
    if (audio_start_recording)
    {
        audio_start_recording = false;
        audio_in_is_recording = true;

        /* Clear I2S RX FIFO */
        Cy_I2S_ClearRxFifo(i2s.base);
        /* Clear Audio In buffer */
        memset(audio_in_pcm_buffer_ping, 0, (MAX_AUDIO_IN_PACKET_SIZE_BYTES));

        audio_in_pcm_buffer = audio_in_pcm_buffer_ping;

        /* Start a transfer to the Audio IN endpoint */
        *ppNextBuffer = (uint8_t *) audio_in_pcm_buffer;
        *pNextPacketSize = sample_size;
    }
    else if(audio_in_is_recording)
    {
        if (audio_in_pcm_buffer == audio_in_pcm_buffer_ping)
        {
            audio_in_pcm_buffer = audio_in_pcm_buffer_pong;
        }
        else
        {
            audio_in_pcm_buffer = audio_in_pcm_buffer_ping;
        }
         /* Setup the number of bytes to transfer based on the current FIFO level */
        if (Cy_I2S_GetNumInRxFifo(i2s.base) > (MAX_AUDIO_IN_PACKET_SIZE_WORDS))
        {
            audio_in_count = (MAX_AUDIO_IN_PACKET_SIZE_WORDS);
        }
        else
        {
            audio_in_count = ((MAX_AUDIO_IN_PACKET_SIZE_WORDS) - (ADDITIONAL_AUDIO_IN_SAMPLE_SIZE_WORDS));
        }
        res = cyhal_i2s_read(&i2s, (void *) audio_in_pcm_buffer, &audio_in_count);
        if(res != CY_RSLT_SUCCESS)
        {
            CY_ASSERT(0);
        }
        if (1U == mic_mute)
        {
            /* Send silent frames in case of mute */
            *ppNextBuffer = silent_frame;
        }
        else
        {
            *ppNextBuffer = (uint8_t *) audio_in_pcm_buffer;
        }
        *pNextPacketSize = audio_in_count * (AUDIO_IN_SUB_FRAME_SIZE);
    }
}
