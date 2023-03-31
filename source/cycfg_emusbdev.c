/******************************************************************************
* File Name:   cycfg_emusbdev.c
*
* Description: This is the source code for the emUSB device middleware configuration data
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2021-2023, Cypress Semiconductor Corporation (an Infineon company) or
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

#include "cycfg_emusbdev.h"
#include "audio.h"

/*******************************************************************************
* Macros
********************************************************************************/
#define AUDIO_DEVICE_VENDOR_ID                  (0x0584)
#if (AUDIO_SAMPLING_RATE_22KHZ == AUDIO_IN_SAMPLE_FREQ)
#define AUDIO_DEVICE_PRODUCT_ID                 (0x0271)
#else
#define AUDIO_DEVICE_PRODUCT_ID                 (0x0270)
#endif /* (AUDIO_SAMPLING_RATE_22KHZ == AUDIO_IN_SAMPLE_FREQ) */
/*******************************************************************************
* Global Variables
********************************************************************************/

/***********************************************************
 Information that is used during enumeration.
************************************************************/
const USB_DEVICE_INFO usb_device_info = {
  AUDIO_DEVICE_VENDOR_ID,         // VendorId
  AUDIO_DEVICE_PRODUCT_ID,        // ProductId
  "Infineon Technologies",        // VendorName
  "USB Audio Record Playback Device",      // ProductName
  "13245678"                      // SerialNumber
};


/****************************************************************
                Speaker format definitions

 When changing these values check
 https://wiki.segger.com/USB_Audio#Audio_class_issues_on_Windows

 Also update PACKET_SIZE_OUT_MAX accordingly.
*****************************************************************/
/*
static USBD_AUDIO_FORMAT speaker_formats[] =  {
#if ENABLE_LARGE_FORMATS
  {0, 2, 2, 16, 192000},// 2 channels, 2 byte SubFrameSize, 16 bit, 192000 Hz
  {0, 2, 2, 16, 96000}, // 2 channels, 2 byte SubFrameSize, 16 bit, 96000 Hz
  {0, 2, 3, 24, 96000}, // 2 channels, 3 byte SubFrameSize, 24 bit, 96000 Hz
  {0, 2, 3, 24, 48000}, // 2 channels, 3 byte SubFrameSize, 24 bit, 48000 Hz
  {0, 2, 3, 24, 44100}, // 2 channels, 3 byte SubFrameSize, 24 bit, 44100 Hz
#endif
  {0, AUDIO_OUT_NUM_CHANNELS, AUDIO_OUT_SUB_FRAME_SIZE, AUDIO_OUT_BIT_RESOLUTION, AUDIO_SAMPLING_RATE_48KHZ}, // 2 channels, 2 byte SubFrameSize, 16 bit, 48000 Hz
  {0, AUDIO_OUT_NUM_CHANNELS, AUDIO_OUT_SUB_FRAME_SIZE, AUDIO_OUT_BIT_RESOLUTION, AUDIO_SAMPLING_RATE_44KHZ}, // 2 channels, 2 byte SubFrameSize, 16 bit, 44100 Hz
  {0, AUDIO_OUT_NUM_CHANNELS, AUDIO_OUT_SUB_FRAME_SIZE, AUDIO_OUT_BIT_RESOLUTION, AUDIO_SAMPLING_RATE_22KHZ} // 2 channels, 2 byte SubFrameSize, 16 bit, 22050 Hz
  {0, AUDIO_OUT_NUM_CHANNELS, AUDIO_OUT_SUB_FRAME_SIZE, AUDIO_OUT_BIT_RESOLUTION, AUDIO_SAMPLING_RATE_8KHZ},  // 2 channels, 2 byte SubFrameSize, 16 bit, 8000 Hz
};
*/


static USBD_AUDIO_FORMAT speaker_formats[] = {{0, AUDIO_OUT_NUM_CHANNELS, AUDIO_OUT_SUB_FRAME_SIZE, AUDIO_OUT_BIT_RESOLUTION, AUDIO_OUT_SAMPLE_FREQ}};

static USBD_AUDIO_UNITS speaker_units;

/****************************************************************
                Microphone format definitions

 When changing these values check
 https://wiki.segger.com/USB_Audio#Audio_class_issues_on_Windows

 Also update PACKET_SIZE_IN_MAX accordingly.
*****************************************************************/

static const USBD_AUDIO_FORMAT microphone_formats[] = {{0, AUDIO_IN_NUM_CHANNELS, AUDIO_IN_SUB_FRAME_SIZE, AUDIO_IN_BIT_RESOLUTION, AUDIO_IN_SAMPLE_FREQ}};

static USBD_AUDIO_UNITS microphone_units;


/***********************************************************
        Speaker and microphone configurations
************************************************************/
const USBD_AUDIO_IF_CONF audio_interfaces[] = {
  
  /* Speaker config */
  
  {
    0,                                // Flags
    0x03,                             // Controls
    AUDIO_OUT_NUM_CHANNELS,           // TotalNrChannels
    SEGGER_COUNTOF(speaker_formats),  // NumFormats
    speaker_formats,                  // paFormats
    0x0003,                           // bmChannelConfig (0x3: Left Front, Right Front)
    USB_AUDIO_TERMTYPE_OUTPUT_SPEAKER,// TerminalType
    &speaker_units                    // pUnits
  } ,
  
  /* Microphone config */ 
  
  {
    0,                                  // Flags
    0x03,                               // Controls
    AUDIO_IN_NUM_CHANNELS,              // TotalNrChannels
    SEGGER_COUNTOF(microphone_formats), // NumFormats
    microphone_formats,                 // paFormats
    0x0003,                             // bmChannelConfig (0x4: Center Front)
    USB_AUDIO_TERMTYPE_INPUT_MICROPHONE_ARRAY,// TerminalType
    &microphone_units                   // pUnits
  } 
};



/*********************************************************************
*                           hid_report
*
*  This report is generated according to HID spec and
*  HID Usage Tables specifications.
***********************************************************************/
const U8 hid_report[] = {
    0x05, 0x0c,                    // USAGE_PAGE (Consumer Devices)
    0x09, 0x01,                    // USAGE (Consumer Control)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x85, 0x01,                    //   REPORT_ID (1)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x09, 0xe9,                    //   USAGE (Volume Up)
    0x09, 0xea,                    //   USAGE (Volume Down)
    0x09, 0xe2,                    //   USAGE (Mute)
    0x09, 0xcd,                    //   USAGE (Play/Pause)
    0x09, 0xb5,                    //   USAGE (Scan Next Track)
    0x09, 0xb6,                    //   USAGE (Scan Previous Track)
    0x09, 0xbc,                    //   USAGE (Repeat)
    0x09, 0xb9,                    //   USAGE (Random Play)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x08,                    //   REPORT_COUNT (8)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
    0xc0                           // END_COLLECTION
};
