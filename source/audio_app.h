/*****************************************************************************
* File Name: audio_app.h
*
* Description: This file contains the constants used in audio_app.c.
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

#ifndef AUDIO_APP_H
#define AUDIO_APP_H

#include "speaker_mic_configs.h"

#if defined(__cplusplus)
extern "C" {
#endif

/*******************************************************************************
* Macros
********************************************************************************/
#define AUDIO_IN_SAMPLE_FREQ        MICROPHONE_FREQUENCIES
#define MI2C_TIMEOUT_MS     (10u)         /* in ms */
#define MCLK_CODEC_DELAY_MS (10u)         /* in ms */
#define MCLK_FREQ_HZ        ((384U) * (AUDIO_IN_SAMPLE_FREQ))/* in Hz */
#define MCLK_DUTY_CYCLE     (50.0f)       /* in %  */
#define USB_CLK_RESET_HZ    (100000)      /* in Hz */
#define AUDIO_APP_MCLK_PIN  P5_0
#define DELAY_TICKS         (50U)
#define I2C_CLK_FREQ        (400000U)      /* in Hz */

#define AUDIO_SAMPLING_RATE_22KHZ   (22050U)
#define AUDIO_SAMPLING_RATE_16KHZ   (16000U)

/* Audio Subsystem Clock. Typical values depends on the desired sample rate:
 * 8KHz / 16 KHz / 32 KHz / 48 KHz    : 24.576 MHz
 * 22.05 KHz / 44.1 KHz               : 22.579 MHz
 */
#if (AUDIO_SAMPLING_RATE_22KHZ == AUDIO_IN_SAMPLE_FREQ)
#define AUDIO_SYS_CLOCK_HZ                  (45158400U)
#else
#define AUDIO_SYS_CLOCK_HZ                  (49152000U)
#endif /* (AUDIO_SAMPLING_RATE_22KHZ == AUDIO_IN_SAMPLE_FREQ) */

#define AUDIO_VOLUME_SIZE     (2U)

/* Each report consists of 2 bytes:
 * 1. The report ID (0x01) and a
 * 2. bit mask containing 8 control events: */

#define AUDIO_HID_REPORT_VOLUME_UP            (0x01u)
#define AUDIO_HID_REPORT_VOLUME_DOWN          (0x02u)
#define AUDIO_HID_REPORT_PLAY_PAUSE           (0x08u)

#define HID_REPORT_PARAMS                       (35)
extern const U8 hid_report[HID_REPORT_PARAMS];

#if defined(__cplusplus)
}
#endif
#endif /* AUDIO_APP_H */
