/*****************************************************************************
* File Name: audio.h
*
*
* Description: This file contains the constants mapped to the USB descriptor.
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

#ifndef AUDIO_H
#define AUDIO_H

#include "Global.h"

#if defined(__cplusplus)
extern "C" {
#endif
/***********************************************************************
                        Defines
************************************************************************/
#define ENABLE_LARGE_FORMATS        (0)

#define NUM_INTERFACES              (2)

#define BUFFER_SIZE                 (1024 * 32)
#define NUM_BUFFERS                 (4)
#define MAX_TIMEOUT_COUNT           (100)
#define READ_TIMEOUT                (10)
#define WRITE_TIMEOUT               (10)
#define MAX_UNHANDLED_RECEIVE       (30)
#define MAX_SILENT_PACKET           (2000)



#define AUDIO_SAMPLING_RATE_48KHZ   (48000U)
#define AUDIO_SAMPLING_RATE_44KHZ   (44100U)
#define AUDIO_SAMPLING_RATE_32KHZ   (32000U)
#define AUDIO_SAMPLING_RATE_22KHZ   (22050U)
#define AUDIO_SAMPLING_RATE_16KHZ   (16000U)
#define AUDIO_SAMPLING_RATE_8KHZ    (8000U)

#define AUDIO_OUT_NUM_CHANNELS      (2)
/* In bytes */
#define AUDIO_OUT_SUB_FRAME_SIZE    (2)
#define AUDIO_OUT_BIT_RESOLUTION    (16)
#define AUDIO_OUT_SAMPLE_FREQ       AUDIO_SAMPLING_RATE_16KHZ

#define AUDIO_IN_NUM_CHANNELS       (2)
/* In bytes */
#define AUDIO_IN_SUB_FRAME_SIZE     (2)   
#define AUDIO_IN_BIT_RESOLUTION     (16)
#define AUDIO_IN_SAMPLE_FREQ        AUDIO_SAMPLING_RATE_16KHZ


/* Each report consists of 2 bytes:
 * 1. The report ID (0x01) and a
 * 2. bit mask containing 8 control events: */

#define AUDIO_HID_REPORT_VOLUME_UP            (0x01u)
#define AUDIO_HID_REPORT_VOLUME_DOWN          (0x02u)
#define AUDIO_HID_REPORT_PLAY_PAUSE           (0x08u)
#define AUDIO_HID_CTRL_MUTE                   (0x04u)

#define AUDIO_VOLUME_SIZE     (2U)
/**< Volume minimum value MSB */
#define AUDIO_VOLUME_MIN_MSB  (0xF1U)
/**< Volume minimum value LSB */
#define AUDIO_VOLUME_MIN_LSB  (0x00U)
/**< Volume maximum value MSB */
#define AUDIO_VOLUME_MAX_MSB  (0x00U)
/**< Volume maximum value LSB */
#define AUDIO_VOLUME_MAX_LSB  (0x00U)
/**< Volume resolution MSB */
#define AUDIO_VOLUME_RES_MSB  (0x00U)
/**< Volume resolution LSB */
#define AUDIO_VOLUME_RES_LSB  (0x01U)

/*****************************************************************************************************
* Has to match the configured values in Microphone/Speaker Configuration
*
*
*
* Two additional sample sizes are added to make sure we can send odd sized frames if necessary:
* ((16/8) * 2) = 4

 For 16KHZ -    16000 * ((16/8) * 2) / 1000  + (16/8)*2   = 68 bytes     (2 channel)

 For 22KHZ -    22050 * ((16/8) * 2) / 1000  + (16/8)*2   = 92 bytes     (2 channels)

 For 32KHZ -    32000 * ((16/8) * 2) / 1000  + (16/8)*2   = 132 bytes    (2 channels)

 For 44KHZ -    44100 * ((16/8) * 2) / 1000  + (16/8)*2   = 180 bytes    (2 channels)

 For 48KHZ -    48000 * ((16/8) * 2) / 1000  + (16/8)*2   = 196 bytes    (2 channels)

*******************************************************************************************************/

/* IN endpoint macros */
#define ADDITIONAL_AUDIO_IN_SAMPLE_SIZE_BYTES   (((AUDIO_IN_BIT_RESOLUTION) / 8U) * (AUDIO_IN_SUB_FRAME_SIZE)) /* In bytes */

#define MAX_AUDIO_IN_PACKET_SIZE_BYTES          ((((AUDIO_IN_SAMPLE_FREQ) * (((AUDIO_IN_BIT_RESOLUTION) / 8U) * (AUDIO_IN_NUM_CHANNELS))) / 1000U) + (ADDITIONAL_AUDIO_IN_SAMPLE_SIZE_BYTES)) /* In bytes */

#define ADDITIONAL_AUDIO_IN_SAMPLE_SIZE_WORDS   ((ADDITIONAL_AUDIO_IN_SAMPLE_SIZE_BYTES) / (AUDIO_IN_SUB_FRAME_SIZE)) /* In words */

#define MAX_AUDIO_IN_PACKET_SIZE_WORDS          ((MAX_AUDIO_IN_PACKET_SIZE_BYTES) / (AUDIO_IN_SUB_FRAME_SIZE)) /* In words */


/* OUT endpoint macros */
#define ADDITIONAL_AUDIO_OUT_SAMPLE_SIZE_BYTES   (((AUDIO_OUT_BIT_RESOLUTION) / 8U) * (AUDIO_OUT_SUB_FRAME_SIZE)) /* In bytes */

#define MAX_AUDIO_OUT_PACKET_SIZE_BYTES          ((((AUDIO_OUT_SAMPLE_FREQ) * (((AUDIO_OUT_BIT_RESOLUTION) / 8U) * (AUDIO_OUT_NUM_CHANNELS))) / 1000U) + (ADDITIONAL_AUDIO_OUT_SAMPLE_SIZE_BYTES)) /* In bytes */

#define ADDITIONAL_AUDIO_OUT_SAMPLE_SIZE_WORDS   ((ADDITIONAL_AUDIO_OUT_SAMPLE_SIZE_BYTES) / (AUDIO_OUT_SUB_FRAME_SIZE)) /* In words */

#define MAX_AUDIO_OUT_PACKET_SIZE_WORDS          ((MAX_AUDIO_OUT_PACKET_SIZE_BYTES) / (AUDIO_OUT_SUB_FRAME_SIZE)) /* In words */


extern U8 mic_mute;
extern U8 speaker_mute;


#if defined(__cplusplus)
}
#endif
#endif /* AUDIO_H */