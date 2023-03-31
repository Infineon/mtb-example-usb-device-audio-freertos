/*******************************************************************************
* File Name: touch.c
*
*  Description: This file contains the implementation of the user interface
*   using CapSense.
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

#include "touch.h"


#include "cycfg.h"
#include "cy_sysint.h"

#include "rtos.h"

/*******************************************************************************
* Touch Application Constants
*******************************************************************************/
#define TOUCH_BASELINE_UPDATE           (1u)
#define TOUCH_BASELINE_IDLE             (-1u)

/*******************************************************************************
* Local Functions
*******************************************************************************/
static void capsense_isr(void);
static void capsense_eos(cy_stc_active_scan_sns_t* active_scan_sns_ptr);

/*******************************************************************************
* Global Variables
*******************************************************************************/
static bool     touch_scan_enable = true;
static int32_t  touch_init_baseline = TOUCH_BASELINE_IDLE;
static TaskHandle_t     touch_task;
static touch_callback_t touch_callback = NULL;
static touch_status_t   touch_current_state = {0};
static touch_status_t   touch_previous_state = {0};
static uint32_t         touch_enable_events = 0;
uint8_t  touch_audio_control_status;

/*******************************************************************************
* Function Name: touch_init
********************************************************************************
* Summary:
*   Initialize the CapSense Block
*
* Parameters:
*  None
*
* Return:
*  None
*******************************************************************************/
void touch_init(void)
{
    cy_capsense_status_t capsens_stat;
    cy_en_sysint_status_t sysint_stat;

    /* Get this task handler */
    touch_task = xTaskGetCurrentTaskHandle();

    /* CapSense interrupt configuration */
    const cy_stc_sysint_t CapSense_interrupt_config =
    {
        .intrSrc = CYBSP_CSD_IRQ,
        .intrPriority = CYHAL_ISR_PRIORITY_DEFAULT,
    };

    /* Capture the CSD HW block and initialize it to the default state. */
    capsens_stat = Cy_CapSense_Init(&cy_capsense_context);
    configASSERT(CY_CAPSENSE_STATUS_SUCCESS == capsens_stat);
    /* Initialize CapSense interrupt */
    sysint_stat = Cy_SysInt_Init(&CapSense_interrupt_config, capsense_isr);
    configASSERT(CY_SYSINT_SUCCESS == sysint_stat);
    NVIC_ClearPendingIRQ(CapSense_interrupt_config.intrSrc);
    NVIC_EnableIRQ(CapSense_interrupt_config.intrSrc);

    /* Register end of scan callback */
    capsens_stat = Cy_CapSense_RegisterCallback(CY_CAPSENSE_END_OF_SCAN_E,
                                 capsense_eos, &cy_capsense_context);
    configASSERT(CY_CAPSENSE_STATUS_SUCCESS == capsens_stat);

    /* Initialize the CapSense firmware modules. */
    capsens_stat = Cy_CapSense_Enable(&cy_capsense_context);
    configASSERT(CY_CAPSENSE_STATUS_SUCCESS == capsens_stat);
}

/*******************************************************************************
* Function Name: touch_is_ready
********************************************************************************
* Summary:
*   Check if ready to process any touches
*
* Return:
*   True is ready, otherwise false.
*
*******************************************************************************/
bool touch_is_ready(void)
{
    return (CY_CAPSENSE_NOT_BUSY == Cy_CapSense_IsBusy(&cy_capsense_context));
}

/*******************************************************************************
* Function Name: touch_start_scan
********************************************************************************
* Summary:
*   Start scanning for touches.
*
* Parameters:
*  None
*
* Return:
*  None
*******************************************************************************/
void touch_start_scan(void)
{
    touch_scan_enable = true;
    xTaskNotify(touch_task, 0, eNoAction);
}

/*******************************************************************************
* Function Name: touch_stop_scan
********************************************************************************
* Summary:
*   Stop scanning for touches.
*
* Parameters:
*  None
*
* Return:
*  None
*******************************************************************************/
void touch_stop_scan(void)
{
    touch_scan_enable = false;
}

/*******************************************************************************
* Function Name: touch_get_state
********************************************************************************
* Summary:
*   Return the current state of the sensors (polling method)
*
* Parameters:
*   sensors: structure with sensor state
*
* Return:
*  None
*******************************************************************************/
void touch_get_state(touch_status_t *sensors)
{
    if (sensors != NULL)
    {
        memcpy(sensors, &touch_current_state, sizeof(touch_status_t));
    }
}

/*******************************************************************************
* Function Name: touch_register_callback
********************************************************************************
* Summary:
*   Register a callback to be executed on certain events
*
* Parameters:
*   callback: function pointer to be executed on event
*
* Return:
*  None
*******************************************************************************/
void touch_register_callback(touch_callback_t callback)
{
    touch_callback = callback;
}

/*******************************************************************************
* Function Name: touch_enable_event
********************************************************************************
* Summary:
*   Events to enable on touch.
*
* Parameters:
*   event: Mask of events to enable
*   enable: true to enable, false to disable
*
* Return:
*  None
*******************************************************************************/
void touch_enable_event(touch_event_t event, bool enable)
{
    if (enable)
    {
        touch_enable_events |= event;
    }
    else
    {
        touch_enable_events &= (~event);
    }
}

/*******************************************************************************
* Function Name: touch_update_baseline
********************************************************************************
* Summary:
*   Update the internal baseline of CapSense.
*
* Parameters:
*  None
*
* Return:
*  None
*******************************************************************************/
void touch_update_baseline(void)
{
    touch_init_baseline = TOUCH_BASELINE_UPDATE;
}

/*******************************************************************************
* Function Name: touch_process
********************************************************************************
* Summary:
*   Main process for the touch task. Handles the CapSense touches and issues
*   USB commands.
*
* Parameters:
*  void *arg - arguments for audio input processing function
*
* Return:
*  None
*******************************************************************************/
void touch_process(void *arg)
{
    cy_stc_capsense_touch_t* slider_touch_info;
    cy_capsense_status_t capsens_stat;
    BaseType_t NotifyRet;
    touch_init();
    while (1)
    {
        /* Check if should keeping scanning */
        if (false == touch_scan_enable)
        {
            /* Wait for start of scan */
            NotifyRet = xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
            if(pdPASS != NotifyRet)
            {
                CY_ASSERT(0);
            }
        }
        /* Start a scan */
        capsens_stat = Cy_CapSense_ScanAllWidgets(&cy_capsense_context);
        if(CY_CAPSENSE_STATUS_SUCCESS != capsens_stat)
        {
            CY_ASSERT(0);
        }
        /* Wait till an end of scan */
        NotifyRet = xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
        if(pdPASS != NotifyRet)
        {
            CY_ASSERT(0);
        }
        /* Process all widgets */
        capsens_stat = Cy_CapSense_ProcessAllWidgets(&cy_capsense_context);
        if(CY_CAPSENSE_STATUS_SUCCESS != capsens_stat)
        {
            CY_ASSERT(0);
        }
        /* Update button touch states */
        touch_current_state.button0 = Cy_CapSense_IsSensorActive(CY_CAPSENSE_BUTTON0_WDGT_ID,
                                                                 CY_CAPSENSE_BUTTON0_SNS0_ID,
                                                                 &cy_capsense_context);
        touch_current_state.button1 = Cy_CapSense_IsSensorActive(CY_CAPSENSE_BUTTON1_WDGT_ID,
                                                                 CY_CAPSENSE_BUTTON1_SNS0_ID,
                                                                 &cy_capsense_context);
        /* Get slider status */
        slider_touch_info = Cy_CapSense_GetTouchInfo(
               CY_CAPSENSE_LINEARSLIDER0_WDGT_ID, &cy_capsense_context);

        touch_current_state.slider_pos = slider_touch_info->ptrPosition->x;
        touch_current_state.slider_status = (slider_touch_info->numPosition > 0);

        /* Check if a callback was registered */
        if (touch_callback != NULL)
        {
            /* Check if should handle TOUCH LIFT events */
            if (touch_enable_events & TOUCH_LIFT)
            {
                if (touch_previous_state.button0 && (!touch_current_state.button0))
                {
                    touch_callback(CY_CAPSENSE_BUTTON0_WDGT_ID, TOUCH_LIFT, 0);
                }

                if (touch_previous_state.button1 && (!touch_current_state.button1))
                {
                    touch_callback(CY_CAPSENSE_BUTTON1_WDGT_ID, TOUCH_LIFT, 0);
                }
                if (touch_previous_state.slider_status && (!touch_current_state.slider_status))
                {
                    touch_callback(CY_CAPSENSE_LINEARSLIDER0_WDGT_ID, TOUCH_LIFT, touch_current_state.slider_pos);
                }
            }
            /* Check if should handle TOUCH DOWN events */
            if (touch_enable_events & TOUCH_DOWN)
            {
                if ((!touch_previous_state.button0) && touch_current_state.button0)
                {
                    touch_callback(CY_CAPSENSE_BUTTON0_WDGT_ID, TOUCH_DOWN, 0);
                }
                if ((!touch_previous_state.button1) && touch_current_state.button1)
                {
                    touch_callback(CY_CAPSENSE_BUTTON1_WDGT_ID, TOUCH_DOWN, 0);
                }

                if ((!touch_previous_state.slider_status) && touch_current_state.slider_status)
                {
                    touch_callback(CY_CAPSENSE_LINEARSLIDER0_WDGT_ID, TOUCH_DOWN, touch_current_state.slider_pos);
                }
            }
            /* Check if should handle TOUCH SLIDE RIGHT events */
            if (touch_enable_events & TOUCH_SLIDE_RIGHT)
            {
                if (touch_previous_state.slider_pos < touch_current_state.slider_pos)
                {
                    touch_callback(CY_CAPSENSE_LINEARSLIDER0_WDGT_ID, TOUCH_SLIDE_RIGHT, touch_current_state.slider_pos);
                }
            }
            /* Check if should handle TOUCH SLIDE LEFT events */
            if (touch_enable_events & TOUCH_SLIDE_LEFT)
            {
                if (touch_previous_state.slider_pos > touch_current_state.slider_pos)
                {
                    touch_callback(CY_CAPSENSE_LINEARSLIDER0_WDGT_ID, TOUCH_SLIDE_LEFT, touch_current_state.slider_pos);
                }
            }
        }
        /* Update the previous state */
        touch_get_state(&touch_previous_state);
        /* Check if baseline should be updated */
        if (0 == touch_init_baseline)
        {
            Cy_CapSense_InitializeAllBaselines(&cy_capsense_context);
            touch_init_baseline = TOUCH_BASELINE_IDLE;
        }
        else if (touch_init_baseline > 0)
        {
            touch_init_baseline--;
        }
        else
        {
            /* Do Nothing */
        }
        /* Wait for the touch period */
        vTaskDelay(pdMS_TO_TICKS(TOUCH_PERIOD_MS));
    }
}

/*******************************************************************************
* Function Name: capsense_isr
********************************************************************************
* Summary:
*  Wrapper function for handling interrupts from CapSense block.
*
* Parameters:
*  None
*
* Return:
*  None
*******************************************************************************/
static void capsense_isr(void)
{
    Cy_CapSense_InterruptHandler(CYBSP_CSD_HW, &cy_capsense_context);
}

/*******************************************************************************
* Function Name: capsense_eos
********************************************************************************
* Summary:
*  CapSense end of scan callback function. This function sends a command to
*  CapSense task to process scan.
*
* Parameters:
*  cy_stc_active_scan_sns_t * active_scan_sns_ptr (unused)
*
* Return:
*  None
*******************************************************************************/
void capsense_eos(cy_stc_active_scan_sns_t* active_scan_sns_ptr)
{
    BaseType_t xYieldRequired;

    (void)active_scan_sns_ptr;

    xTaskNotifyFromISR(touch_task, 0, eNoAction, &xYieldRequired);
    portYIELD_FROM_ISR(xYieldRequired);
}


/* [] END OF FILE */

