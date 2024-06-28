/******************************************************************************
* File Name:   main.c
*
* Description: This is the main source file for the audio recorder/playback example
*              using emUSB stack for ModusToolbox.
*
* Related Document: See README.md
*
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

#include "cyhal.h"
#include "cybsp.h"
#include "touch.h"

/***************************************
*    RTOS Constants
***************************************/
#include "FreeRTOS.h"
#include "task.h"
#include "rtos.h"

/*******************************************************************************
* Macros
********************************************************************************/
#define NUM_OF_TASK    (2U)

/*******************************************************************************
* Global Variables
********************************************************************************/

/* RTOS tasks */
TaskHandle_t rtos_audio_app_task = NULL;
TaskHandle_t rtos_touch_task = NULL;

/*******************************************************************************
* Function Prototypes
********************************************************************************/
void audio_app_process(void* arg);


/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CM4 CPU. It creates the necessary tasks for the code example and starts the scheduler
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;
    BaseType_t TaskCreateRet[NUM_OF_TASK];

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Create the RTOS tasks */
    TaskCreateRet[0U] = xTaskCreate(audio_app_process, "Audio App Task",
                                RTOS_STACK_DEPTH, NULL, RTOS_TASK_PRIORITY,
                                &rtos_audio_app_task);
    configASSERT(rtos_audio_app_task);

    TaskCreateRet[1U] = xTaskCreate(touch_process, "Touch Task",
                                RTOS_STACK_DEPTH, NULL, RTOS_TASK_PRIORITY,
                                &rtos_touch_task);
    configASSERT(rtos_touch_task);

    /* Start the RTOS Scheduler */
    if((pdPASS == TaskCreateRet[0U]) && (pdPASS == TaskCreateRet[1U]))
    {      
        vTaskStartScheduler();
    }
    else
    {
        CY_ASSERT(0);
    }
    return 0;
}

/*******************************************************************************
* Function Name: vApplicationIdleHook()
********************************************************************************
* Summary:
*   RTOS Idle task implementation.
*
*******************************************************************************/
void vApplicationIdleHook( void )
{
    /* Go to sleep */
    cyhal_syspm_sleep();
}

/* [] END OF FILE */
