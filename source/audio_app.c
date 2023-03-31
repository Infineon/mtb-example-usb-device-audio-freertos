/*******************************************************************************
* File Name: audio_app.c
*
*  Description: This file contains the application code for audio processing
*
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

#include "audio.h"
#include "audio_app.h"
#include "audio_in.h"
#include "audio_out.h"
#include "USB.h"
#include "cycfg_emusbdev.h"
#include "USB_Audio.h"
#include "USB_HID.h"
#include "rtos.h"
#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "touch.h"
#include "cycfg.h"
#ifdef COMPONENT_AK4954A
    #include "ak4954a.h"
#endif



static USBD_AUDIO_INIT_DATA audio_init_data;
static USB_HID_INIT_DATA    hid_init_data;
USBD_AUDIO_HANDLE           usb_audio_context;
USB_HID_HANDLE              usb_hid_control_context;
static USB_ADD_EP_INFO      ep_out;
static USB_ADD_EP_INFO      ep_in;

/*******************************************************************************
* Macros
********************************************************************************/
#define MI2C_TIMEOUT_MS     (10u)         /* in ms */
#define MCLK_CODEC_DELAY_MS (10u)         /* in ms */
#define MCLK_FREQ_HZ        ((384U) * (AUDIO_IN_SAMPLE_FREQ))/* in Hz */
#define MCLK_DUTY_CYCLE     (50.0f)       /* in %  */
#define USB_CLK_RESET_HZ    (100000)      /* in Hz */
#define AUDIO_APP_MCLK_PIN  P5_0
#define DELAY_TICKS         (50U)
#define I2C_CLK_FREQ        (400000U)      /* in Hz */
/* Audio Subsystem Clock. Typical values depends on the desired sample rate:
 * 8KHz / 16 KHz / 32 KHz / 48 KHz    : 24.576 MHz
 * 22.05 KHz / 44.1 KHz               : 22.579 MHz
 */
#if (AUDIO_SAMPLING_RATE_22KHZ == AUDIO_IN_SAMPLE_FREQ)
#define AUDIO_SYS_CLOCK_HZ                  (45158400U)
#else
#define AUDIO_SYS_CLOCK_HZ                  (49152000U)
#endif /* (AUDIO_SAMPLING_RATE_22KHZ == AUDIO_IN_SAMPLE_FREQ) */


/*******************************************************************************
* Global Variables
********************************************************************************/
static uint8_t  audio_app_control_report[2] = {0x1, 0x0};
uint32_t audio_app_current_sample_rate;
uint8_t   audio_app_volume;
uint8_t   audio_app_prev_volume;
static volatile bool usb_suspend_flag = false;
U8       mic_mute;
U8       speaker_mute;

uint8_t usb_comm_cur_volume[AUDIO_VOLUME_SIZE];
uint8_t usb_comm_min_volume[AUDIO_VOLUME_SIZE] = {AUDIO_VOLUME_MIN_LSB, AUDIO_VOLUME_MIN_MSB};
uint8_t usb_comm_max_volume[AUDIO_VOLUME_SIZE] = {AUDIO_VOLUME_MAX_LSB, AUDIO_VOLUME_MAX_MSB};
uint8_t usb_comm_res_volume[AUDIO_VOLUME_SIZE] = {AUDIO_VOLUME_RES_LSB, AUDIO_VOLUME_RES_MSB};

bool                 audio_app_mute;
uint8_t              current_speaker_format_index;
uint8_t              current_microphone_format_index;
static cyhal_clock_t audio_clock;
static cyhal_clock_t hf0_clock;

static const cyhal_i2s_pins_t i2s_tx_pins = 
{
    .sck  = P5_1,
    .ws   = P5_2,
    .data = P5_3,
    .mclk = NC,
};

static const cyhal_i2s_pins_t i2s_rx_pins = 
{
    .sck  = P5_4,
    .ws   = P5_5,
    .data = P5_6,
    .mclk = NC,
};

static const cyhal_i2s_config_t i2s_config = 
{
#ifdef COMPONENT_AK4954A
    .is_tx_slave    = true,     /* TX is Slave */
#else
    .is_tx_slave    = false,    /* TX is Master */
#endif
    .is_rx_slave    = false,    /* RX is Master */
    .mclk_hz        = 0,        /* External MCLK not used */
    .channel_length = 16,       /* In bits */
    .word_length    = 16,       /* In bits */
    .sample_rate_hz = AUDIO_IN_SAMPLE_FREQ,    /* In Hz */
};

#ifdef COMPONENT_AK4954A
/* Master I2C variables */
static cyhal_i2c_t mi2c;

static const cyhal_i2c_cfg_t mi2c_cfg = 
{
    .is_slave        = false,
    .address         = 0,
    .frequencyhal_hz = I2C_CLK_FREQ
};
#endif

/* HAL Objects */
cyhal_i2s_t i2s;
cyhal_clock_t pll_clock;
static cyhal_pwm_t mclk_pwm;

/* Tolerance Values */
const cyhal_clock_tolerance_t tolerance_0_p = {CYHAL_TOLERANCE_PERCENT, 0};
const cyhal_clock_tolerance_t tolerance_1_p = {CYHAL_TOLERANCE_PERCENT, 1};

/* Speaker configurations */
static USBD_AUDIO_IF_CONF * speaker_config = (USBD_AUDIO_IF_CONF *) &audio_interfaces[0];

/* Microphone configurations */
static USBD_AUDIO_IF_CONF  * microphone_config = (USBD_AUDIO_IF_CONF *) &audio_interfaces[1];


/*******************************************************************************
* Function Prototypes
********************************************************************************/
static void audio_app_touch_events(uint32_t widget, touch_event_t event, uint32_t value);
static void audio_clock_init(void);
#ifdef COMPONENT_AK4954A
static void audio_app_update_codec_volume(void);
static cy_rslt_t mi2c_transmit(uint8_t reg_addr, uint8_t data);
#endif /* COMPONENT_AK4954A */



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
 * Function Name: audio_control_callback
 ********************************************************************************
 * Summary:
 *  Callback function on detection of control action from the USB middleware
 *
 * Parameters:
 *  void * pUserContext -
 *  U8 Event -
 *  U8 ControlSelector -
 *  U8 * pBuffer -
 *  U32 NumBytes -
 *  U8 InterfaceNo -
 *  U8 AltSetting -
 *
 * Return:
 *  int - audio event handling status
 *
 *******************************************************************************/
static int audio_control_callback(void * pUserContext, U8 Event, U8 Unit, U8 ControlSelector, U8 * pBuffer, U32 NumBytes, U8 InterfaceNo, U8 AltSetting) {
    int retVal;

    (void)pUserContext;
    (void)InterfaceNo;
    retVal = 0;

    switch (Event)
    {

        case USB_AUDIO_PLAYBACK_START:
        {
            /* Host enabled transmission */
            audio_out_enable();
            break;
        }
        case USB_AUDIO_PLAYBACK_STOP:
        {
            /* Host disabled transmission. Some hosts do not always send this! */
            audio_out_disable();
            break;
        }
        case USB_AUDIO_RECORD_START:
        {
            /* Host enabled reception */
            audio_in_enable();
            break;
        }
        case USB_AUDIO_RECORD_STOP:
        {
            /* Host disabled reception. Some hosts do not always send this! */
            audio_in_disable();
            break;
        }
        case USB_AUDIO_SET_CUR:
        {
            switch (ControlSelector) 
            {
                case USB_AUDIO_MUTE_CONTROL:
                {
                    if(1U == NumBytes)
                    {
                        if(Unit == microphone_config->pUnits->FeatureUnitID)
                        {
                            mic_mute = *pBuffer;
                        }
                        if(Unit == microphone_config->pUnits->FeatureUnitID)
                        {
                            speaker_mute = *pBuffer;
                        }
                    }
                    break;
                }
                case USB_AUDIO_VOLUME_CONTROL:
                {
                    if(2U == NumBytes)
                    {
                        memcpy(usb_comm_cur_volume, pBuffer, sizeof(usb_comm_cur_volume));
                    }
                    break;
                }
                case USB_AUDIO_SAMPLING_FREQ_CONTROL:
                {
                    if (3U == NumBytes)
                    {
                        if (Unit == speaker_config->pUnits->FeatureUnitID)
                        {
                            if ((AltSetting > 0) && (AltSetting < speaker_config->NumFormats))
                            {
                                current_speaker_format_index = AltSetting - 1;
                            }
                        }
                        if (Unit == microphone_config->pUnits->FeatureUnitID)
                        {
                            if ((AltSetting > 0) && (AltSetting < microphone_config->NumFormats))
                            {
                                current_microphone_format_index = AltSetting - 1;
                            }
                        }
                    }
                    break;
                }
                default:
                    retVal = 1;
                break;
            }

            break;
        }
        case USB_AUDIO_GET_CUR:
        {
            switch (ControlSelector)
            {
                case USB_AUDIO_MUTE_CONTROL:
                {
                      pBuffer[0] = 0;
                      break;
                }
                case USB_AUDIO_VOLUME_CONTROL:
                {
                    pBuffer[0] = 0;
                    pBuffer[0] = 0;
                    break;
                }
                case USB_AUDIO_SAMPLING_FREQ_CONTROL:
                {
                    if (Unit == speaker_config->pUnits->FeatureUnitID)
                    {
                        pBuffer[0] =  speaker_config->paFormats[current_speaker_format_index].SamFreq & 0xff;
                        pBuffer[1] = (speaker_config->paFormats[current_speaker_format_index].SamFreq >> 8) & 0xff;
                        pBuffer[2] = (speaker_config->paFormats[current_speaker_format_index].SamFreq >> 16) & 0xff;
                    }

                    if (Unit == microphone_config->pUnits->FeatureUnitID)
                    {
                        pBuffer[0] =  microphone_config->paFormats[current_microphone_format_index].SamFreq & 0xff;
                        pBuffer[1] = (microphone_config->paFormats[current_microphone_format_index].SamFreq >> 8) & 0xff;
                        pBuffer[2] = (microphone_config->paFormats[current_microphone_format_index].SamFreq >> 16) & 0xff;
                    }
                    break;
                }
                default:
                {
                    pBuffer[0] = 0;
                    pBuffer[1] = 0;
                    break;
                }
            }
            break;
        }
        case USB_AUDIO_SET_MIN:
        {
            break;
        }
        case USB_AUDIO_GET_MIN:
        {
            switch (ControlSelector)
            {
                case USB_AUDIO_VOLUME_CONTROL:
                {
                    pBuffer[0] = usb_comm_min_volume[0];
                    pBuffer[1] = usb_comm_min_volume[1];
                    break;
                }
                default:
                {
                    pBuffer[0] = 0;
                    pBuffer[1] = 0;
                    break;
                }
            }
            break;
        }
        case USB_AUDIO_SET_MAX:
        {
            break;
        }
        case USB_AUDIO_GET_MAX:
        {
            switch (ControlSelector)
            {
                case USB_AUDIO_VOLUME_CONTROL:
                {
                    pBuffer[0] = usb_comm_max_volume[0];
                    pBuffer[1] = usb_comm_max_volume[1];
                    break;
                }
                default:
                {
                    pBuffer[0] = 0;
                    pBuffer[1] = 0;
                    break;
                }
            }
            break;
        }
        case USB_AUDIO_SET_RES:
        {
            break;
        }
        case USB_AUDIO_GET_RES:
        {
            switch (ControlSelector)
            {
                case USB_AUDIO_VOLUME_CONTROL:
                {
                    pBuffer[0] = usb_comm_res_volume[0];
                    pBuffer[1] = usb_comm_res_volume[1];
                    break;
                }
                default:
                {
                    pBuffer[0] = 0;
                    pBuffer[1] = 0;
                    break;
                }
            }
            break;
        }

        default:
        {
            retVal = 1;
            break;
        }
    }
    return retVal;
}

/*********************************************************************
* Function name: add_audio
**********************************************************************
* Summary:
*  Add a USB Audio interface to the USB stack.
*
* Parameters:
*  None
*
* Return:
*  USBD_AUDIO_HANDLE - Handle for added audio instance
************************************************************************/
static USBD_AUDIO_HANDLE add_audio(void) {
    
    USBD_AUDIO_HANDLE hInst;
    
    /* OUT endpoint configurations */
    ep_out.Flags             = USB_ADD_EP_FLAG_USE_ISO_SYNC_TYPES;
    ep_out.InDir             = USB_DIR_OUT;
    ep_out.Interval          = 8;
    ep_out.MaxPacketSize     = MAX_AUDIO_OUT_PACKET_SIZE_BYTES;
    ep_out.TransferType      = USB_TRANSFER_TYPE_ISO;
    ep_out.ISO_Type          = USB_ISO_SYNC_TYPE_ASYNCHRONOUS;


    /* IN endpoint configurations */
    ep_in.Flags              = USB_ADD_EP_FLAG_USE_ISO_SYNC_TYPES;
    ep_in.InDir              = USB_DIR_IN;
    ep_in.Interval           = 8;
    ep_in.MaxPacketSize      = MAX_AUDIO_IN_PACKET_SIZE_BYTES;
    ep_in.TransferType       = USB_TRANSFER_TYPE_ISO;
    ep_in.ISO_Type           = USB_ISO_SYNC_TYPE_ASYNCHRONOUS; 


    memset(&audio_init_data, 0, sizeof(audio_init_data));

    audio_init_data.EPOut           = USBD_AddEPEx(&ep_out, NULL, 0);
    audio_init_data.EPIn            = USBD_AddEPEx(&ep_in, NULL, 0);
    audio_init_data.pfOnOut         = &audio_out_endpoint_callback;
    audio_init_data.pfOnIn          = &audio_in_endpoint_callback;
    audio_init_data.OutPacketSize   = MAX_AUDIO_OUT_PACKET_SIZE_BYTES;
    audio_init_data.pfOnControl     = audio_control_callback;
    audio_init_data.NumInterfaces   = SEGGER_COUNTOF(audio_interfaces);
    audio_init_data.paInterfaces    = audio_interfaces;
    audio_init_data.pOutUserContext = NULL;
    audio_init_data.pInUserContext  = NULL;
    audio_init_data.pControlUserContext  = NULL;

    hInst = USBD_AUDIO_Add(&audio_init_data);
    
    return hInst;

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
  EPIntIn.Flags           = 0;                             // Flags not used.
  EPIntIn.InDir           = USB_DIR_IN;                    // IN direction (Device to Host)
  EPIntIn.Interval        = 8;                             // Interval of 1 ms (125 us * 8)
  EPIntIn.MaxPacketSize   = 2;                             // Report size.
  EPIntIn.TransferType    = USB_TRANSFER_TYPE_INT;         // Endpoint type - Interrupt.
  hid_init_data.EPIn      = USBD_AddEPEx(&EPIntIn, NULL, 0);

  hid_init_data.pReport = hid_report;
  hid_init_data.NumBytesReport = sizeof(hid_report);
  hInst = USBD_HID_Add(&hid_init_data);

  return hInst;
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
    if(result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
    /* Initialize the Master Clock with a PWM */
    result = cyhal_pwm_init(&mclk_pwm, (cyhal_gpio_t) AUDIO_APP_MCLK_PIN, NULL);
    if(result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
    result = cyhal_pwm_set_duty_cycle(&mclk_pwm, MCLK_DUTY_CYCLE, MCLK_FREQ_HZ);
    if(result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
    result = cyhal_pwm_start(&mclk_pwm);
    if(result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
    /* Wait for the MCLK to clock the audio codec */
    result = cyhal_system_delay_ms(MCLK_CODEC_DELAY_MS);
    if(result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

#ifdef COMPONENT_AK4954A
    result = cyhal_i2c_init(&mi2c, CYBSP_I2C_SDA, CYBSP_I2C_SCL, NULL);
    if(result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
    result = cyhal_i2c_configure(&mi2c, &mi2c_cfg);
    if(result != CY_RSLT_SUCCESS)
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
    if(result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
#endif
    /* Initialize the I2S block */
    result = cyhal_i2s_init(&i2s, &i2s_tx_pins, &i2s_rx_pins, &i2s_config, &audio_clock);
    if(result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
    USBD_Init();
    /* Set device information*/
    USBD_SetDeviceInfo(&usb_device_info);
    /* Add Audio endpoints to the USB stack */
    usb_audio_context = add_audio();
    /* Add HID Audio control endpoint to the USB stack */
    usb_hid_control_context = add_hid_control();
    /* Set USB read/write timeouts */
    USBD_AUDIO_Set_Timeouts(0, READ_TIMEOUT, WRITE_TIMEOUT);
    /* Register and enable touch events */
    touch_register_callback(audio_app_touch_events);
    touch_enable_event(TOUCH_ALL, true);
    /* Start the USB device stack */
    USBD_Start();
    /* Schedules task for IN endpoint. */
    audio_in_init();
    /* Schedules task for OUT endpoint. */
    audio_out_init();
    cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_OFF);
}


/*******************************************************************************
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
*******************************************************************************/
void audio_app_process(void *arg)
{
    volatile bool usb_suspended = false;
    volatile bool usb_connected = false;
    int32_t usbd_stat;

    (void) arg;

    audio_app_init();
    while(1)
    {
        vTaskDelay(pdMS_TO_TICKS(DELAY_TICKS));
        /* Check if suspend condition is detected on the bus */
        if (usb_suspend_flag)
        {
            if (!usb_suspended)
            {
                usb_suspended = true;
                usb_connected = false;
                /* Stop providing audio data to the host */
                USBD_AUDIO_Stop_Play(usb_audio_context);
            }
        }
        else /* USB device connected */
        {
            if (!usb_connected)
            {
                usb_connected = true;
                usb_suspended = false;
                /* Start providing audio data to the host */
                usbd_stat = USBD_AUDIO_Start_Play(usb_audio_context, NULL);
                configASSERT(0 == usbd_stat);
            }
            usbd_stat = USBD_HID_Write(usb_hid_control_context, audio_app_control_report, 2, 0);
            /* configASSERT(2 == usbd_stat); */
            audio_app_control_report[0] = 0x01;
            audio_app_control_report[1] = 0;
            /* Send out report data */
            usbd_stat = USBD_HID_Write(usb_hid_control_context, audio_app_control_report, 2, 0);
            /* configASSERT(2 == usbd_stat); */
#ifdef COMPONENT_AK4954A
            audio_app_update_codec_volume();
#endif
        }
    }
}

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
    int16_t  vol_usb = ((int16_t) usb_comm_cur_volume[1])*256 + 
                       ((int16_t) usb_comm_cur_volume[0]);

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
*              Returns CY_RSLT_SUCCESS if succeeded.
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
