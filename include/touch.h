/*******************************************************************************
* File Name: touch.h
*
*  Description:  This file contains the function prototypes and constants
*   used in touch.c.
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

#ifndef TOUCH_H
#define TOUCH_H

#include "cycfg_capsense.h"
#include <stdbool.h>
#include <stdint.h>

#if defined(__cplusplus)
extern "C" {
#endif
/*******************************************************************************
* Touch Application Constants
*******************************************************************************/
#define TOUCH_PERIOD_MS                 (10u)

typedef enum
{
    TOUCH_DOWN          = 1 << 0,
    TOUCH_LIFT          = 1 << 1,
    TOUCH_SLIDE_RIGHT   = 1 << 2,
    TOUCH_SLIDE_LEFT    = 1 << 3,
    TOUCH_ALL        = 0xFF
} touch_event_t;

typedef struct {
    bool     button0;
    bool     button1;
    bool     slider_status;
    uint16_t slider_pos;
} touch_status_t;

/* Callback for CapSense events */
typedef void (*touch_callback_t)(uint32_t widget, touch_event_t event, uint32_t value);

/*******************************************************************************
* Touch Functions
*******************************************************************************/
void touch_init(void);
bool touch_is_ready(void);
void touch_update_baseline(void);
void touch_process(void *arg);
void touch_start_scan(void);
void touch_stop_scan(void);
void touch_get_state(touch_status_t *sensors);
void touch_register_callback(touch_callback_t callback);
void touch_enable_event(touch_event_t event, bool enable);

#if defined(__cplusplus)
}
#endif
#endif /* TOUCH_H */
/* [] END OF FILE */
