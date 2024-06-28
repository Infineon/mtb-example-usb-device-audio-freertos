/*******************************************************************************
* File Name: audio_app.c
*
* Description: This file contains the audio application code
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

#include <stdio.h>
#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cycfg.h"
#include "cy_i2s.h"
#include "USB.h"
#include "USB_AC.h"
#include "USB_HID.h"
#include "audio_app.h"
#include "touch.h"
#ifdef COMPONENT_AK4954A
    #include "ak4954a.h"
#endif
#include "USB_HW_Cypress_PSoC6.h"

/*********************************************************************
*
*      AUDIO configurations: Audio 1.0 stereo Speaker, stereo Microphone, 16kHz sample frequency
*
**********************************************************************/
#include "speaker_mic_configs.h"

/*********************************************************************
*
*      RTOS #include section
*
**********************************************************************/
#include "FreeRTOS.h"
#include "rtos.h"
#include "task.h"
#include "queue.h"

/*********************************************************************
*
*      Forward declarations
*
**********************************************************************/
#ifdef __cplusplus
extern "C" {     /* Make sure we have C-declarations in C++ programs */
#endif
    void audio_app_process(void* arg);
    void audio_class_init_data(void);
    void audio_app_run(void* arg);
#ifdef __cplusplus
}
#endif

/*********************************************************************
*
*      Macros
*
**********************************************************************/
#define AUDIO_DEVICE_VENDOR_ID               (0x0655)
#if (AUDIO_SAMPLING_RATE_22KHZ == AUDIO_IN_SAMPLE_FREQ)
#define AUDIO_DEVICE_PRODUCT_ID              (0x0281)
#else
#define AUDIO_DEVICE_PRODUCT_ID              (0x0280)
#endif /* (AUDIO_SAMPLING_RATE_22KHZ == AUDIO_IN_SAMPLE_FREQ) */

/*********************************************************************
*
*      Types
*
**********************************************************************/
typedef enum {
    MSG_SPEAKER_ON,
    MSG_SPEAKER_OFF,
    MSG_SPEAKER_DATA,
    MSG_MIC_ON,
    MSG_MIC_OFF,
    MSG_MIC_DATA,
    MSG_SPEAKERAUX_ON,
    MSG_SPEAKERAUX_OFF,
    MSG_SPEAKERAUX_DATA,
} MSG_TYPE;

typedef struct {
    MSG_TYPE   Event;
    U32     NumBtyes;
    void* pBuff;
} MESSAGE;

/*********************************************************************
*
*      Global Variables
*
**********************************************************************/
TaskHandle_t rtos_audio_run_task = NULL;

static QueueHandle_t    Mail_Box;
static StaticQueue_t    Static_Queue;

static USB_HID_INIT_DATA    hid_init_data;
USB_HID_HANDLE            usb_hid_control_context;
static uint8_t  audio_app_control_report[2] = { 0x1, 0x0 };
static cy_rslt_t mi2c_transmit(uint8_t reg_addr, uint8_t data);

static volatile bool usb_suspend_flag = false;

U8     mic_mute;
U8     speaker_mute;

uint8_t usb_comm_cur_volume[AUDIO_VOLUME_SIZE];
uint8_t   audio_app_volume;
uint8_t   audio_app_prev_volume;

/* Information that is used during enumeration. */
static const USB_DEVICE_INFO usb_device_info = {
  AUDIO_DEVICE_VENDOR_ID,        // VendorId
  AUDIO_DEVICE_PRODUCT_ID,      // ProductId
  "Infineon Technologies",      // VendorName
  "emUSB Audio Device",     // ProductName
  "13245678"      // SerialNumber
};

static const U32  speaker_frequencies[] = { SPEAKER_FREQUENCIES };
static const U32  mic_frequencies[] = { MICROPHONE_FREQUENCIES };

USBD_AC_RX_CTX RX;
USBD_AC_TX_CTX TX;

/* Buffers for audio data */
static MESSAGE        Msg_Buff[5];
static U16            mic_audio_buffer[64 / 2];
static U32            speaker_audio_buffer[64 / 4];

static cyhal_clock_t audio_clock;
static cyhal_clock_t hf0_clock;

static const cyhal_i2s_pins_t i2s_tx_pins =
{
    .sck = P5_1,
    .ws = P5_2,
    .data = P5_3,
    .mclk = NC,
};

static const cyhal_i2s_pins_t i2s_rx_pins =
{
    .sck = P5_4,
    .ws = P5_5,
    .data = P5_6,
    .mclk = NC,
};

static const cyhal_i2s_config_t i2s_config =
{
#ifdef COMPONENT_AK4954A
    .is_tx_slave = true,     /* TX is Slave */
#else
    .is_tx_slave = false,   /* TX is Master */
#endif
    .is_rx_slave = false,   /* RX is Master */
    .mclk_hz = 0,       /* External MCLK not used */
    .channel_length = 16,      /* In bits */
    .word_length = 16,     /* In bits */
    .sample_rate_hz = AUDIO_IN_SAMPLE_FREQ, /* In Hz */
};

#ifdef COMPONENT_AK4954A
/* Master I2C variables */
static cyhal_i2c_t mi2c;

static const cyhal_i2c_cfg_t mi2c_cfg =
{
    .is_slave = false,
    .address = 0,
    .frequencyhal_hz = I2C_CLK_FREQ
};
#endif

/* HAL Objects */
cyhal_i2s_t i2s;
cyhal_clock_t pll_clock;
static cyhal_pwm_t mclk_pwm;

/* Tolerance Values */
const cyhal_clock_tolerance_t tolerance_0_p = { CYHAL_TOLERANCE_PERCENT, 0 };
const cyhal_clock_tolerance_t tolerance_1_p = { CYHAL_TOLERANCE_PERCENT, 1 };

struct AC_Global_t {
    U32   CurrSpeakerFreq;
    U32   CurrSpeakerAuxFreq;
    U32   CurrMicFreq;
    U16   SpeakerVolume;
    U8  SpeakerMute[3];
    U8  SpeakerAltSetting;
    U8  SpeakerCurrBuff;
    U8  MicrophoneMute;
    U16   MicrophoneVolume;
    U8  MicrophoneAltSetting;
    U32   MicSoundSize;
    U32   MicDataRate;
    U32   MicSilentPackets;
    U32   MicDataSend;
    U32   MicSilentCount;
    U32   RemData;
    U16   SideToneVolume;
    U8  SideToneMute;
    const U8* pMicSound;
    const U8* pMicData;
    USBD_AC_STREAM_INTF_INFO MicInfo;
    USBD_AC_STREAM_INTF_INFO SpeakerInfo;
};

static struct AC_Global_t AC_Global;

/*********************************************************************
*                          hid_report
*
*  This report is generated according to HID spec and
*  HID Usage Tables specifications.
***********************************************************************/
const U8 hid_report[] = {
    0x05, 0x0c,                 // USAGE_PAGE (Consumer Devices)
    0x09, 0x01,                 // USAGE (Consumer Control)
    0xa1, 0x01,                 // COLLECTION (Application)
    0x85, 0x01,                 //   REPORT_ID (1)
    0x15, 0x00,                 //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                 //   LOGICAL_MAXIMUM (1)
    0x09, 0xe9,                 //   USAGE (Volume Up)
    0x09, 0xea,                 //   USAGE (Volume Down)
    0x09, 0xe2,                 //   USAGE (Mute)
    0x09, 0xcd,                 //   USAGE (Play/Pause)
    0x09, 0xb5,                 //   USAGE (Scan Next Track)
    0x09, 0xb6,                 //   USAGE (Scan Previous Track)
    0x09, 0xbc,                 //   USAGE (Repeat)
    0x09, 0xb9,                 //   USAGE (Random Play)
    0x75, 0x01,                 //   REPORT_SIZE (1)
    0x95, 0x08,                 //   REPORT_COUNT (8)
    0x81, 0x02,                 //   INPUT (Data,Var,Abs)
    0xc0                           // END_COLLECTION
};

/********************************************************************************
* Function Name: vApplicationTickHook
********************************************************************************
* Summary:
*  Tick hook function called at every tick (1ms). It checks for the activity in
*  the USB bus.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void vApplicationTickHook(void)
{
    /* Supervisor of suspend conditions on the bus */
#if defined (COMPONENT_CAT1A)
    USB_DRIVER_Cypress_PSoC6_SysTick();
#endif /* COMPONENT_CAT1A */

    if (USBD_GetState() & USB_STAT_SUSPENDED)
    {
        /* Suspend condition on USB bus is detected */
        usb_suspend_flag = true;
    }
    else
    {
        /* Clear suspend conditions */
        usb_suspend_flag = false;
    }
}

/********************************************************************************
* Function Name: audio_set_interface_control_callback
********************************************************************************
* Summary:
*  Definition of the callback which is called when the hosts sets an alternate 
*  setting on an audio interface.
*
* Parameters:
*  InterfaceNo - Number of the audio streaming interface.
*  NewAltSetting - Alternate setting selected by the host.
*
* Return:
*  None
*
*******************************************************************************/
static void audio_set_interface_control_callback(unsigned InterfaceNo, unsigned NewAltSetting) {
    MESSAGE Msg;
    BaseType_t xHigherPriorityTaskWoken;

    switch (InterfaceNo) {
    case USBD_AC_INTERFACE_Speaker:
#if USBD_AC_AUDIO_VERSION == 1
        /*
         * If an alternate setting has only a single sample frequency,
         * then setting AltInterface also must set the sample frequency.
         * If an alternate setting has multiple sample frequencies
         * this should be commented out.
         */
        if (NewAltSetting > 0) {
            AC_Global.CurrSpeakerFreq = speaker_frequencies[NewAltSetting - 1];
        }
#endif
        AC_Global.SpeakerInfo = *USBD_AC_GetStreamInfo(USBD_AC_INTERFACE_Speaker, NewAltSetting);
        AC_Global.SpeakerAltSetting = NewAltSetting;
        Msg.Event = (NewAltSetting == 0) ? MSG_SPEAKER_OFF : MSG_SPEAKER_ON;

        xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(Mail_Box, &Msg, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken) {
            /* Actual macro used here is port specific. */
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
        break;

    case USBD_AC_INTERFACE_Microphone:
#if USBD_AC_AUDIO_VERSION == 1
        /* Setting AltInterface also sets the sample frequency */
        if (NewAltSetting > 0) {
            AC_Global.CurrMicFreq = mic_frequencies[NewAltSetting - 1];
        }
#endif
        AC_Global.MicInfo = *USBD_AC_GetStreamInfo(USBD_AC_INTERFACE_Microphone, NewAltSetting);
        AC_Global.MicrophoneAltSetting = NewAltSetting;
        Msg.Event = (NewAltSetting == 0) ? MSG_MIC_OFF : MSG_MIC_ON;

        xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(Mail_Box, &Msg, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken) {
            /* Actual macro used here is port specific. */
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
        break;
    }
}

/********************************************************************************
* Function Name: audio_out_callback
********************************************************************************
* Summary:
*  Definition of the callback which is called when audio data was received from the host.
*  pRxData->Numbytes bytes of data were received into pRxData->pBuffer.
*  The function must reinitialize the members pBuffer, NumBytes and PaxPackets before it returns.
*  This callback is called in interrupt context and must not block. The audio data must not be processed
*  inside this function, instead a task should be triggered that does the audio processing and this function
*  should return as fast as possible. After this functions has returned, the next USB transfer is started
*  immediately. Therefore the member 'pBuffer' should be initialized to point to a different buffer to avoid
*  overwriting the data just received (double buffering mechanism is recommended).
*
* Parameters:
*  Event - Event occurred on the audio channel.
*  pRxData - Pointer to a USBD_AC_RX_DATA structure. The contents is valid only, if Event == USBD_AC_EVENT_DATA_RECEIVED.
*
* Return:
*  None
*
*******************************************************************************/
static void audio_out_callback(USBD_AC_EVENT Event, USBD_AC_RX_DATA* pRxData) {
    MESSAGE Msg;

    switch (Event) {
    case USBD_AC_EVENT_DATA_RECEIVED:
        Msg.Event = MSG_SPEAKER_DATA;
        Msg.NumBtyes = pRxData->NumBytes;
        Msg.pBuff = pRxData->pBuffer;

        BaseType_t xHigherPriorityTaskWoken;
        xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(Mail_Box, &Msg, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken) {
            /* Actual macro used here is port specific. */
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }

        AC_Global.SpeakerCurrBuff ^= 1;
        pRxData->pBuffer = speaker_audio_buffer;
        pRxData->NumBytes = sizeof(speaker_audio_buffer);
        pRxData->NumPackets = 0;  /* Maximum possible */
        break;

    default:
        break;
    }
}

/********************************************************************************
* Function Name: audio_in_callback
********************************************************************************
* Summary:
*  Definition of the callback which is called when audio data was sent to the host.
*  The function should initiate to send more data.
*
* Parameters:
*  Event - Event occurred on the audio channel.
*  pData - Pointer to the data send, that was provided to the USBD_AC_Send() function.
*  pContext - Pointer from the USBD_AC_RX_CTX structure.
*
* Return:
*  None
*
*******************************************************************************/
static void audio_in_callback(USBD_AC_EVENT Event, const void* pData, void* pUserContext) {
    MESSAGE Msg;

    USB_USE_PARA(pData);
    USB_USE_PARA(pUserContext);

    switch (Event) {
    case USBD_AC_EVENT_DATA_SEND:
        Msg.Event = MSG_MIC_DATA;

        BaseType_t xHigherPriorityTaskWoken;
        xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(Mail_Box, &Msg, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken) {
            /* Actual macro used here is port specific. */
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
        break;

    default:
        break;
    }
}

/********************************************************************************
* Function Name: audio_feedback_endpoint_callback
********************************************************************************
* Summary:
*  audio_feedback_endpoint_callback
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void audio_feedback_endpoint_callback(void* arg)
{
    uint32_t DataRate;
    uint32_t num_samples;

    num_samples = Cy_I2S_GetNumInTxFifo(i2s.base);

    /* Calculate number of samples per interval as 16.16 fix point number */
    /* CurrSpeakerFreq * (Interval in micro frames) / 8 / 1000 * 2^16    */

    if (num_samples > 128)
    {
        DataRate = (((15000 << AC_Global.SpeakerInfo.IntervalExp) << 13) + 500) / 1000;
    }
    else if (num_samples < 128)
    {
        DataRate = (((17000 << AC_Global.SpeakerInfo.IntervalExp) << 13) + 500) / 1000;
    }
    else
    {
        DataRate = (((AC_Global.CurrSpeakerFreq << AC_Global.SpeakerInfo.IntervalExp) << 13) + 500) / 1000;
    }

    USBD_AC_SetFeedbackDataRate(&RX, DataRate);
}

/********************************************************************************
* Function Name: audio_app_run
********************************************************************************
* Summary:
*  Run audio application.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void audio_app_run(void* arg) {
    I8 SpeakerActive = 0;
    I8 MicActive = 0;
    MESSAGE Msg;
    U32 SpeakerDataCnt = 0;
    cy_rslt_t res;
    uint32_t data_to_write;
    size_t audio_in_count;

    for (;;) {
        USB_MEMSET(&AC_Global, 0, sizeof(AC_Global));
        while ((USBD_GetState() & (USB_STAT_CONFIGURED | USB_STAT_SUSPENDED)) != USB_STAT_CONFIGURED) {
            USB_OS_Delay(50);
        }

        while ((USBD_GetState() & (USB_STAT_CONFIGURED | USB_STAT_SUSPENDED)) == USB_STAT_CONFIGURED) {

            if (xQueueReceive(Mail_Box, &Msg, 100) == pdFALSE) {
                continue;
            }

            switch (Msg.Event) {
            case MSG_SPEAKER_ON:
                if (SpeakerActive != 0) {
                    USBD_AC_CloseRXStream(&RX);
                    SpeakerActive = 0;
                }

                /* Clear I2S TX FIFO */
                Cy_I2S_ClearTxFifo(i2s.base);

                /* Start the I2S TX if disabled */
                if (0U == (Cy_I2S_GetCurrentState(i2s.base) & CY_I2S_TX_START))
                {
                    res = cyhal_i2s_start_tx(&i2s);
                    if (res != CY_RSLT_SUCCESS)
                    {
                        CY_ASSERT(0);
                    }
                }

                memset(&RX, 0, sizeof(RX));
                RX.Interface = USBD_AC_INTERFACE_Speaker;
                RX.RxData.pBuffer = speaker_audio_buffer;
                RX.RxData.NumBytes = sizeof(speaker_audio_buffer);
                RX.RxData.NumPackets = 0;  /* Maximum possible */
                RX.RxData.Timeout = 5000;
                RX.pfCallback = audio_out_callback;
                RX.pfSOFCallback = audio_feedback_endpoint_callback;
                RX.FeedbackInterval = 10;

                AC_Global.SpeakerCurrBuff = 0;

                if (USBD_AC_OpenRXStream(&RX) == 0) {
                    SpeakerActive = 1;
                }

                break;
            case MSG_SPEAKER_OFF:
                if (SpeakerActive != 0) {
                    USBD_AC_CloseRXStream(&RX);
                    SpeakerActive = 0;
                }
                break;
            case MSG_SPEAKER_DATA:
                SpeakerDataCnt = Msg.NumBtyes;

                if (SpeakerDataCnt > 0) {
                    data_to_write = SpeakerDataCnt / 2;
                    /* Write data to I2S Tx */
                    res = cyhal_i2s_write(&i2s, speaker_audio_buffer, (size_t*) &data_to_write);
                    if (res != CY_RSLT_SUCCESS)
                    {
                        CY_ASSERT(0);
                    }
                }

                break;

            case MSG_MIC_ON:
                if (MicActive != 0) {
                    USBD_AC_CloseTXStream(&TX);
                    MicActive = 0;
                }

                /* Clear I2S RX FIFO */
                Cy_I2S_ClearRxFifo(i2s.base);

                /* Run the I2S RX all the time */
                res = cyhal_i2s_start_rx(&i2s);
                if(res != CY_RSLT_SUCCESS)
                {
                    CY_ASSERT(0);
                }

                memset(&TX, 0, sizeof(TX));
                TX.Interface = USBD_AC_INTERFACE_Microphone;
                TX.Timeout = 5000;
                TX.pfCallback = audio_in_callback;

                if (USBD_AC_OpenTXStream(&TX) == 0) {
                    /* Clear Audio In buffer */
                    memset(mic_audio_buffer, 0, sizeof(mic_audio_buffer));
                    MicActive = 1;

                    USBD_AC_Send(&TX, 1, 64, mic_audio_buffer);
                }
                break;

            case MSG_MIC_OFF:
                if (MicActive != 0) {
                    USBD_AC_CloseTXStream(&TX);
                    MicActive = 0;
                }
                break;

            case MSG_MIC_DATA:
                audio_in_count = 32;
                res = cyhal_i2s_read(&i2s, (void*) mic_audio_buffer, &audio_in_count);
                if (res != CY_RSLT_SUCCESS)
                {
                    CY_ASSERT(0);
                }
                USBD_AC_Send(&TX, 1, 64, mic_audio_buffer);
                break;

            default:
                break;
            }
        }
    }
}

/********************************************************************************
* Function Name: audio_control_get_callback
********************************************************************************
* Summary:
*  For audio 1.0 control requests!
*  Definition of the callback which is called when an audio get requests is received.
*  This callback is called in interrupt context and must not block.
*
* Parameters:
*  pReqInfo  : Contains information about the type of the control request.
*  pBuffer    : Pointer to a buffer into which the callback should write the reply (max. 64 bytes).
*
* Return:
*  >= 0:          Audio control request was handled by the callback and response data was put into pBuffer.
*                The callback function must return the length of the response data which will be send to the host.
*  < 0 :          Audio control request was not handled by the callback (i.e. illegal request or parameters).
*                The stack will STALL the request.
*
*******************************************************************************/
#if USBD_AC_AUDIO_VERSION == 1
static int audio_control_get_callback(const USBD_AC_CONTROL_INFO* pReqInfo, U8* pBuffer) {
    U32 Value;

    switch (pReqInfo->ID) {
    case USBD_AC_ID_EP_Speaker + USB_AC_SAMPLING_FREQ_CONTROL:
        switch (pReqInfo->bRequest) {
        case USB_AC_REQ_MIN:
        case USB_AC_REQ_MAX:
        case USB_AC_REQ_CUR:
            Value = AC_Global.CurrSpeakerFreq;
            break;
        case USB_AC_REQ_RES:
            Value = 1;
            break;
        default:
            return -1;
        }
        USBD_StoreU24LE(pBuffer, Value);
        return 3;

    case USBD_AC_ID_EP_Microphone + USB_AC_SAMPLING_FREQ_CONTROL:
        switch (pReqInfo->bRequest) {
        case USB_AC_REQ_MIN:
        case USB_AC_REQ_MAX:
        case USB_AC_REQ_CUR:
            Value = AC_Global.CurrMicFreq;
            break;
        case USB_AC_REQ_RES:
            Value = 1;
            break;
        default:
            return -1;
        }
        USBD_StoreU24LE(pBuffer, Value);
        return 3;

    case USBD_AC_ID_UNIT_SpeakerControl + USB_AC_FU_VOLUME_CONTROL:
        switch (pReqInfo->bRequest) {
        case USB_AC_REQ_CUR:
            Value = AC_Global.SpeakerVolume;
            break;
        case USB_AC_REQ_MIN:
            Value = 0xF100;  /* -15 db */
            break;
        case USB_AC_REQ_MAX:
            Value = 0x0000;   /* 0 db */
            break;
        case USB_AC_REQ_RES:
            Value = 0x0001;   /* 1 db */
            break;
        default:
            return -1;
        }
        USBD_StoreU16LE(pBuffer, Value);
        return 2;

    case USBD_AC_ID_UNIT_MicControl + USB_AC_FU_VOLUME_CONTROL:
        switch (pReqInfo->bRequest) {
        case USB_AC_REQ_CUR:
            Value = AC_Global.MicrophoneVolume;
            break;
        case USB_AC_REQ_MIN:
            Value = 0xF100;  /* -15 db */
            break;
        case USB_AC_REQ_MAX:
            Value = 0x0000;   /* 0 db */
            break;
        case USB_AC_REQ_RES:
            Value = 0x0001;   /* 1 db */
            break;
        default:
            return -1;
        }
        USBD_StoreU16LE(pBuffer, Value);
        return 2;

    case USBD_AC_ID_UNIT_SpeakerControl + USB_AC_FU_MUTE_CONTROL:
        if (pReqInfo->ChannelNumber == 0) {
            pBuffer[0] = AC_Global.SpeakerMute[1];
            pBuffer[1] = AC_Global.SpeakerMute[2];
            return 2;
        }
        if (pReqInfo->ChannelNumber <= 2) {
            pBuffer[0] = AC_Global.SpeakerMute[pReqInfo->ChannelNumber];
            return 1;
        }
        break;

    case USBD_AC_ID_UNIT_MicControl + USB_AC_FU_MUTE_CONTROL:
        pBuffer[0] = AC_Global.MicrophoneMute;
        return 1;

    }
    return -1;
}
#endif

/********************************************************************************
* Function Name: audio_control_set_callback
********************************************************************************
* Summary:
*  For audio 1.0 control requests!
*  Definition of the callback which is called when an audio set requests is received.
*  This callback is called in interrupt context and must not block.
*
* Parameters:
*  pReqInfo  : Contains information about the type of the control request.
*  NumBytes  : Number of bytes in pBuffer.
*  pBuffer    : Pointer to a buffer containing the request data.
*
* Return:
*  == 0:          Audio control request was handled by the callback.
*  != 0:          Audio control request was not handled by the callback (i.e. illegal request or parameters).
*                The stack will STALL the request.
*
*******************************************************************************/
#if USBD_AC_AUDIO_VERSION == 1
static int audio_control_set_callback(const USBD_AC_CONTROL_INFO* pReqInfo, U32 NumBytes, const U8* pBuffer) {

    switch (pReqInfo->ID) {
    case USBD_AC_ID_EP_Speaker + USB_AC_SAMPLING_FREQ_CONTROL:
        return 0;

    case USBD_AC_ID_EP_Microphone + USB_AC_SAMPLING_FREQ_CONTROL:
        return 0;

    case USBD_AC_ID_UNIT_SpeakerControl + USB_AC_FU_VOLUME_CONTROL:
        if (pReqInfo->bRequest == USB_AC_REQ_CUR && NumBytes == 2) {
            memcpy(usb_comm_cur_volume, pBuffer, sizeof(usb_comm_cur_volume));
            AC_Global.SpeakerVolume = USBD_GetU16LE(pBuffer);
        }
        return 0;

    case USBD_AC_ID_UNIT_MicControl + USB_AC_FU_VOLUME_CONTROL:
        if (pReqInfo->bRequest == USB_AC_REQ_CUR && NumBytes == 2) {
            AC_Global.MicrophoneVolume = USBD_GetU16LE(pBuffer);
        }
        return 0;

    case USBD_AC_ID_UNIT_SpeakerControl + USB_AC_FU_MUTE_CONTROL:
        if (pReqInfo->ChannelNumber == 0 && NumBytes == 2) {
            AC_Global.SpeakerMute[1] = pBuffer[0];
            AC_Global.SpeakerMute[2] = pBuffer[1];
            return 0;
        }
        if (pReqInfo->ChannelNumber <= 2) {
            AC_Global.SpeakerMute[pReqInfo->ChannelNumber] = *pBuffer;
            return 0;
        }
        break;

    case USBD_AC_ID_UNIT_MicControl + USB_AC_FU_MUTE_CONTROL:
        AC_Global.MicrophoneMute = *pBuffer;
        return 0;
    }
    return -1;
}
#endif

/*******************************************************************************
* Function Name: audio_app_touch_events
********************************************************************************
* Summary:
*  Handle touch events from CapSense.
*
* Parameters:
*  uint32_t widget: CapSense element ID
*  touch_event_t event: type of event
*  uint32_t value: slider position (if a slider event)
*
*
* Return:
*  None
*******************************************************************************/
static void audio_app_touch_events(uint32_t widget, touch_event_t event, uint32_t value)
{
    uint8_t touch_audio_control_status = 0;

    switch (widget)
    {
    case CY_CAPSENSE_LINEARSLIDER0_WDGT_ID:
    {
        /* Check the direction to change the volume */
        if (TOUCH_SLIDE_RIGHT == event)
        {
            touch_audio_control_status = AUDIO_HID_REPORT_VOLUME_UP;
            audio_app_control_report[0] = 0x01;
            audio_app_control_report[1] = touch_audio_control_status;
        }
        if (TOUCH_SLIDE_LEFT == event)
        {
            touch_audio_control_status = AUDIO_HID_REPORT_VOLUME_DOWN;
            audio_app_control_report[0] = 0x01;
            audio_app_control_report[1] = touch_audio_control_status;
        }
        break;
    }
    case CY_CAPSENSE_BUTTON0_WDGT_ID:
    {
        /* Play/Pause on Button 0 lift */
        if (TOUCH_LIFT == event)
        {
            touch_audio_control_status = AUDIO_HID_REPORT_PLAY_PAUSE;
            audio_app_control_report[0] = 0x01;
            audio_app_control_report[1] = touch_audio_control_status;
        }
        break;
    }
    default:
    {
        break;
    }
    }
}

/********************************************************************************
* Function Name: audio_class_init_data
********************************************************************************
* Summary:
*  Initialization data for the Audio class instance.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void audio_class_init_data(void) {
    USBD_AC_INIT_DATA  InitData;

    USB_MEMSET(&InitData, 0, sizeof(InitData));
    InitData.pACConfig = USB_AC_CONFIGURATION;
    InitData.pfControlGet = audio_control_get_callback;
    InitData.pfControlSet = audio_control_set_callback;
    InitData.pfSetAlternate = audio_set_interface_control_callback;

    USBD_AC_Add(&InitData);

    Mail_Box = xQueueCreateStatic(SEGGER_COUNTOF(Msg_Buff), sizeof(MESSAGE), (uint8_t*)Msg_Buff, &Static_Queue);

}

/*********************************************************************
* Function name: add_hid_control
**********************************************************************
* Summary:
*  Add HID mouse class to USB stack
*
* Parameters:
*  None
*
* Return:
*  USBD_AUDIO_HANDLE - Handle for added hid instance
**********************************************************************/
static USB_HID_HANDLE add_hid_control(void) {

    USB_HID_HANDLE hInst;

    USB_ADD_EP_INFO   EPIntIn;

    memset(&hid_init_data, 0, sizeof(hid_init_data));
    EPIntIn.Flags = 0;                           // Flags not used.
    EPIntIn.InDir = USB_DIR_IN;                 // IN direction (Device to Host)
    EPIntIn.Interval = 8;                            // Interval of 1 ms (125 us * 8)
    EPIntIn.MaxPacketSize = 2;                           // Report size.
    EPIntIn.TransferType = USB_TRANSFER_TYPE_INT;        // Endpoint type - Interrupt.
    hid_init_data.EPIn = USBD_AddEPEx(&EPIntIn, NULL, 0);

    hid_init_data.pReport = hid_report;
    hid_init_data.NumBytesReport = sizeof(hid_report);
    hInst = USBD_HID_Add(&hid_init_data);

    return hInst;
}

#ifdef COMPONENT_AK4954A

/*******************************************************************************
* Function Name: audio_app_update_codec_volume
********************************************************************************
* Summary:
*   Update the audio codec volume by sending an I2C message.
*
* Parameters:
*  None
*
* Return:
*  None
*******************************************************************************/
static void audio_app_update_codec_volume(void)
{
    int16_t  vol_usb = ((int16_t)usb_comm_cur_volume[1]) * 256 +
        ((int16_t)usb_comm_cur_volume[0]);

    audio_app_volume = (-vol_usb) / 64;

    /* Check if the volume changed */
    if (audio_app_volume != audio_app_prev_volume)
    {
        ak4954a_adjust_volume(audio_app_volume);
        audio_app_prev_volume = audio_app_volume;
    }
    /* Check if mute settings changed */
    if (1 == speaker_mute)
    {
        ak4954a_adjust_volume(AK4954A_HP_MUTE_VALUE);
    }
}

/*******************************************************************************
* Function Name: mi2c_transmit
********************************************************************************
* Summary:
*  I2C Master function to transmit data to the given address.
*
* Parameters:
*  uint8_t reg_addr: address to be updated
*  uint8_t data: 8-bit data to be written in the register
*
* Return:
*  cy_rslt_t - I2C master transaction error status.
*             Returns CY_RSLT_SUCCESS if succeeded.
*
*******************************************************************************/
static cy_rslt_t mi2c_transmit(uint8_t reg_addr, uint8_t data)
{
    cy_rslt_t result;
    uint8_t buffer[AK4954A_PACKET_SIZE];

    buffer[0] = reg_addr;
    buffer[1] = data;

    /* Send the data over the I2C */
    result = cyhal_i2c_master_write(&mi2c, AK4954A_I2C_ADDR, buffer, AK4954A_PACKET_SIZE, MI2C_TIMEOUT_MS, true);
    return result;
}

#endif

/*******************************************************************************
* Function Name: audio_clock_init
********************************************************************************
* Summary:
*  Initializes clock for audio subsystem.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void audio_clock_init(void)
{

    cyhal_clock_t clock_pll;
    cyhal_clock_t usb_rst_clock;
    cy_rslt_t result;

    /* Initialize, take ownership of PLL0/PLL */
    result = cyhal_clock_reserve(&clock_pll, &CYHAL_CLOCK_PLL[0]);
    if (CY_RSLT_SUCCESS != result)
    {
        CY_ASSERT(0);
    }

    /* Set the PLL0/PLL frequency to AUDIO_SYS_CLOCK_HZ based on AUDIO_IN_SAMPLE_FREQ */
    result = cyhal_clock_set_frequency(&clock_pll, AUDIO_SYS_CLOCK_HZ, &tolerance_0_p);
    if (CY_RSLT_SUCCESS != result)
    {
        CY_ASSERT(0);
    }

    /* If the PLL0/PLL clock is not already enabled, enable it */
    if (!cyhal_clock_is_enabled(&clock_pll))
    {
        result = cyhal_clock_set_enabled(&clock_pll, true, true);
        if (CY_RSLT_SUCCESS != result)
        {
            CY_ASSERT(0);
        }
    }

    /* Initialize, take ownership of CLK_HF1 */
    result = cyhal_clock_reserve(&audio_clock, &CYHAL_CLOCK_HF[1]);
    if (CY_RSLT_SUCCESS != result)
    {
        CY_ASSERT(0);
    }

    /* Source the audio subsystem clock (CLK_HF1) from PLL0/PLL */
    result = cyhal_clock_set_source(&audio_clock, &clock_pll);
    if (CY_RSLT_SUCCESS != result)
    {
        CY_ASSERT(0);
    }

    /* Set the divider for audio subsystem clock (CLK_HF1) */
    result = cyhal_clock_set_divider(&audio_clock, 2);
    if (CY_RSLT_SUCCESS != result)
    {
        CY_ASSERT(0);
    }

    /* If the audio subsystem clock (CLK_HF1) is not already enabled, enable it */
    if (!cyhal_clock_is_enabled(&audio_clock))
    {
        result = cyhal_clock_set_enabled(&audio_clock, true, true);
        if (CY_RSLT_SUCCESS != result)
        {
            CY_ASSERT(0);
        }
    }
    /* Initialize, take ownership of CLK_HF0 */
    result = cyhal_clock_reserve(&hf0_clock, &CYHAL_CLOCK_HF[0]);
    if (CY_RSLT_SUCCESS != result)
    {
        CY_ASSERT(0);
    }

    /* Source the CLK_HF0 clock from PLL0/PLL */
    result = cyhal_clock_set_source(&hf0_clock, &clock_pll);
    if (CY_RSLT_SUCCESS != result)
    {
        CY_ASSERT(0);
    }

    /* Set the divider for CLK_HF0 clock */
    result = cyhal_clock_set_divider(&hf0_clock, 1);
    if (CY_RSLT_SUCCESS != result)
    {
        CY_ASSERT(0);
    }

    /* If the CLK_HF0 clock (CLK_HF1) is not already enabled, enable it */
    if (!cyhal_clock_is_enabled(&hf0_clock))
    {
        result = cyhal_clock_set_enabled(&hf0_clock, true, true);
        if (CY_RSLT_SUCCESS != result)
        {
            CY_ASSERT(0);
        }
    }
    result = cyhal_clock_get(&usb_rst_clock, &CYBSP_USB_CLK_DIV_obj);
    if (CY_RSLT_SUCCESS != result)
    {
        CY_ASSERT(0);
    }
    result = cyhal_clock_set_frequency(&usb_rst_clock, USB_CLK_RESET_HZ, &tolerance_1_p);
    if (CY_RSLT_SUCCESS != result)
    {
        CY_ASSERT(0);
    }
}

/*********************************************************************
* Function name: audio_app_init
**********************************************************************
* Summary:
*  Initializes hardware peripherals necessary for this audio application
*
* Parameters:
*  None
*
* Return:
*  None
**********************************************************************/
void audio_app_init(void)
{
    cy_rslt_t result;

    /* Initialize the audio clock based on audio sample rate */
    audio_clock_init();

    /* Initialize the User  LED */
    result = cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
    /* Initialize the Master Clock with a PWM */
    result = cyhal_pwm_init(&mclk_pwm, (cyhal_gpio_t)AUDIO_APP_MCLK_PIN, NULL);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
    result = cyhal_pwm_set_duty_cycle(&mclk_pwm, MCLK_DUTY_CYCLE, MCLK_FREQ_HZ);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
    result = cyhal_pwm_start(&mclk_pwm);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
    /* Wait for the MCLK to clock the audio codec */
    result = cyhal_system_delay_ms(MCLK_CODEC_DELAY_MS);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

#ifdef COMPONENT_AK4954A
    result = cyhal_i2c_init(&mi2c, CYBSP_I2C_SDA, CYBSP_I2C_SCL, NULL);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
    result = cyhal_i2c_configure(&mi2c, &mi2c_cfg);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
    result = ak4954a_init(mi2c_transmit);
    if (result != 0)
    {
        NVIC_SystemReset();
    }
    ak4954a_activate();
    result = ak4954a_adjust_volume(AK4954A_HP_DEFAULT_VOLUME);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
}
#endif
    /* Initialize the I2S block */
    result = cyhal_i2s_init(&i2s, &i2s_tx_pins, &i2s_rx_pins, &i2s_config, &audio_clock);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    USBD_Init();
    USBD_EnableIAD();

    /* Initialization data for the Audio class instance. */
    audio_class_init_data();

    USBD_SetLogFilter(USB_MTYPE_AUDIO);

    /* Set device information*/
    USBD_SetDeviceInfo(&usb_device_info);

    /* Add HID Audio control endpoint to the USB stack */
    usb_hid_control_context = add_hid_control();

    /* Register and enable touch events */
    touch_register_callback(audio_app_touch_events);
    touch_enable_event(TOUCH_ALL, true);

    /* Start the USB device stack */
    USBD_Start();

    BaseType_t rtos_task_status;
    rtos_task_status = xTaskCreate(audio_app_run, "RUN Task", RTOS_STACK_DEPTH, NULL, RTOS_TASK_PRIORITY, &rtos_audio_run_task);
    if (pdPASS != rtos_task_status)
    {
        CY_ASSERT(0);
    }
    configASSERT(rtos_audio_run_task);

}

/********************************************************************************
* Function Name: audio_app_process
********************************************************************************
* Summary:
*   Main audio task. Initialize the USB communication and the audio application.
*   In the main loop, process requests to change
*   the volume.
*
* Parameters:
*  void *arg - arguments for audio processing function
*
* Return:
*  None
*
*******************************************************************************/
void audio_app_process(void* arg) {
    volatile bool usb_suspended = false;
    volatile bool usb_connected = false;

    (void)arg;

    audio_app_init();

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(DELAY_TICKS));
        if (usb_suspend_flag)
        {
            if (!usb_suspended)
            {
                usb_suspended = true;
                usb_connected = false;
            }
        }
        else
        {
            if (!usb_connected)
            {
                usb_connected = true;
                usb_suspended = false;
            }
            USBD_HID_Write(usb_hid_control_context, audio_app_control_report, 2, 0);

            audio_app_control_report[0] = 0x01;
            audio_app_control_report[1] = 0;

            /* Send out report data */
            USBD_HID_Write(usb_hid_control_context, audio_app_control_report, 2, 0);
            cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_ON);

#ifdef COMPONENT_AK4954A
            audio_app_update_codec_volume();
#endif
        }
    }
}

/**************************** end of file ***************************/
