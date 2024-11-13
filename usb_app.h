/***************************************************************************//**
* \file usb_app.h
* \version 1.0
*
* Defines the interfaces used in the FX2G3 USB Video Class application.
*
*******************************************************************************
* \copyright
* (c) (2024), Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.
*
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#ifndef _CY_USB_APP_H_
#define _CY_USB_APP_H_

#include "cy_pdl.h"
#include "cy_debug.h"
#include "cy_usbhs_dw_wrapper.h"

#if defined(__cplusplus)
extern "C" {
#endif

#define RED                             "\033[0;31m"
#define CYAN                            "\033[0;36m"
#define COLOR_RESET                     "\033[0m"

#define LOG_COLOR(...)                  Cy_Debug_AddToLog(1,CYAN);\
                                        Cy_Debug_AddToLog(1,__VA_ARGS__); \
                                        Cy_Debug_AddToLog(1,COLOR_RESET);

#define LOG_ERROR(...)                  Cy_Debug_AddToLog(1,RED);\
                                        Cy_Debug_AddToLog(1,__VA_ARGS__); \
                                        Cy_Debug_AddToLog(1,COLOR_RESET);

#define LOG_CLR(CLR, ...)               Cy_Debug_AddToLog(1,CLR);\
                                        Cy_Debug_AddToLog(1,__VA_ARGS__); \
                                        Cy_Debug_AddToLog(1,COLOR_RESET);


#define LOG_TRACE()                     LOG_COLOR("-->[%s]:%d\r\n",__func__,__LINE__);


#define DELAY_MICRO(us)                 Cy_SysLib_DelayUs(us)
#define DELAY_MILLI(ms)                 Cy_SysLib_Delay(ms)

#define PHY_TRAINING_PATTERN_BYTE      (0x00)
#define LINK_TRAINING_PATTERN_BYTE     (0x00000000) 
#define FPS_DEFAULT                    (60)

#define    SET_BIT(byte, mask)               (byte) |= (mask)
#define CLR_BIT(byte, mask)               (byte) &= ~(mask)
#define CHK_BIT(byte, mask)               (byte) & (mask)


/* DMA channel and UVC header configuration*/
#if PRE_ADDED_HEADER
#define UVC_HEADER_BY_FX2G3              (0)
#else
#define UVC_HEADER_BY_FX2G3              (1)
#endif

#define UVC_HEADER_BY_FPGA              ((!UVC_HEADER_BY_FX2G3))
#define FPGA_ENABLE                     (1)
#define SOURCE_COLORBAR                 (1)

#define DMA_BUFFER_SIZE                 (CY_USB_UVC_STREAM_BUF_SIZE)
#if UVC_HEADER_BY_FX2G3
#define FPGA_DMA_BUFFER_SIZE            (DMA_BUFFER_SIZE - CY_USB_UVC_MAX_HEADER)
#elif UVC_HEADER_BY_FPGA
#define FPGA_DMA_BUFFER_SIZE            (DMA_BUFFER_SIZE)
#endif


#define ASSERT(condition, value)                            Cy_CheckStatus(__func__, __LINE__, condition, value, true);
#define ASSERT_NON_BLOCK(condition, value)                  Cy_CheckStatus(__func__, __LINE__, condition, value, false);
#define ASSERT_AND_HANDLE(condition, value, failureHandler) Cy_CheckStatusHandleFailure(__func__, __LINE__, condition, value, false, failureHandler);

/* P4.0 is used for VBus detect functionality. */
#define VBUS_DETECT_GPIO_PORT           (P4_0_PORT)
#define VBUS_DETECT_GPIO_PIN            (P4_0_PIN)
#define VBUS_DETECT_GPIO_INTR           (ioss_interrupts_gpio_dpslp_4_IRQn)
#define VBUS_DETECT_STATE               (0u)

/* Number of buffers used for PDM data transfer. */
#define PDM_APP_BUFFER_CNT              (4u)
#define RAM_BUF_SZ_WORDS                (15360)         /* Total size of the RAM based DMA buffers: 15 KB. */
#define GPIF_SM_NUM                     (0u)

typedef struct cy_stc_usb_app_ctxt_ cy_stc_usb_app_ctxt_t;

/* FPGA Configuration mode selection*/
typedef enum cy_en_fpgaConfigMode_t
{
    ACTIVE_SERIAL_MODE,
    PASSIVE_SERIAL_MODE
}cy_en_fpgaConfigMode_t;

typedef enum cy_en_streamControl_t
{
    STOP,
    START
}cy_en_streamControl_t;
/*
 * USB application data structure which is bridge between USB system and device
 * functionality.
 * It maintains some usb system information which comes from USBD and it also
 * maintains info about functionality.
 */
struct cy_stc_usb_app_ctxt_
{
    uint8_t firstInitDone;
    cy_en_usb_device_state_t devState;
    cy_en_usb_device_state_t prevDevState;
    cy_en_usb_speed_t devSpeed;
    uint8_t devAddr;
    uint8_t activeCfgNum;
    cy_en_usb_enum_method_t enumMethod;
    uint8_t prevAltSetting;
    cy_en_usb_speed_t desiredSpeed;

    cy_stc_app_endp_dma_set_t endpInDma[CY_USB_MAX_ENDP_NUMBER];
    cy_stc_app_endp_dma_set_t endpOutDma[CY_USB_MAX_ENDP_NUMBER];
    DMAC_Type *pCpuDmacBase;
    DW_Type *pCpuDw0Base;
    DW_Type *pCpuDw1Base;

    cy_stc_hbdma_mgr_context_t *pHbDmaMgrCtxt;
    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt;
    cy_stc_hbdma_channel_t *hbBulkInChannel;

    /* UVC specific fields. */
    uint8_t uvcInEpNum;                         /** Index of UVC streaming endpoint. */
    uint8_t uvcPendingBufCnt;                   /** Number of data buffers which are pending to be streamed. */
    bool    uvcFlowCtrlFlag;                    /** Flag indicating that UVC channel is in flow control. */

    /* Global Task handles */
    TaskHandle_t uvcDevicetaskHandle;           /** UVC application task. */
    QueueHandle_t uvcMsgQueue;                  /** Message queue used to send messages to the UVC task. */

    bool usbConnectDone;
    bool vbusChangeIntr;                        /** VBus change interrupt received flag. */
    bool vbusPresent;                           /** VBus presence indicator flag. */
    bool usbConnected;                          /** Whether USB connection is enabled. */
    TimerHandle_t vbusDebounceTimer;            /** VBus change debounce timer handle. */
    uint32_t *pUsbEvtLogBuf;
    TimerHandle_t evtLogTimer;                  /** Timer to print eventLog. */

#if AUDIO_IF_EN
    /* UAC interface specific fields. */
    uint8_t *pPDMRxBuffer[PDM_APP_BUFFER_CNT];  /** Pointer of buffers to read PDM data into. */
    uint16_t pdmRxDataLen[PDM_APP_BUFFER_CNT];  /** Amount of data in each PDM RX buffer. */
    uint8_t  pdmRxBufIndex;                     /** Index of current PDM read buffer. */
    uint8_t  nxtAudioTxBufIndex;                /** Index of next audio buffer to be sent on USB EP. */
    uint8_t  pdmRxFreeBufCount;                 /** Number of free PDM RX buffers. */
    bool     pdmInXferPending;                  /** Whether a write has been queued on IN EP. */
    uint8_t  pdmPendingDmaFlag;                 /** Flag indicating whether DMA transfer is pending on
                                                    each PDM RX channel. */
    TaskHandle_t  uacAppTaskHandle;             /** Handle to the UAC application task. */
    QueueHandle_t uacMsgQueue;                  /** Handle to UAC application message queue. */
    TimerHandle_t pdmActivateTimer;             /** Handle of timer used to activate PDM channels. */

    /* DMA RAM buffer location. */
    uint32_t epDmaBufSpace[RAM_BUF_SZ_WORDS];
    /* Index of the next free DMA RAM buffer location. */
    uint32_t dmaBufFreeIdx;
#endif /* AUDIO_IF_EN */

    /* UVC specific fields. */
    TimerHandle_t fpsTimer;
    uint8_t fpgaVersion;
    uint8_t glpassiveSerialMode;

    uint32_t glfps;
    uint32_t glDmaBufCnt;
    uint32_t glDmaBufCnt_prv;
    volatile uint32_t glProd;
    volatile uint32_t glCons;
    uint32_t glProdCount;
    uint32_t glConsCount;
    uint32_t glFrameSizeTransferred;
    uint32_t glFrameSize;
    volatile uint8_t glPrintFlag;
    volatile uint32_t glFrameCount;
    volatile uint32_t glPartialBufSize;
};

/* FPGA  Register map*/
typedef enum cy_en_i2c_fpgaRegMap_t
{
    /*Common Register Info*/
    FPGA_MAJOR_VERSION_ADDRESS             = 0x00,          /* FPGA Major Version*/
    FPGA_MINOR_VERSION_ADDRESS             = 0x00,          /* FPGA Minor Version*/

    FPGA_UVC_SELECTION_ADDRESS             = 0x01,          /* UVC Enable Address */
    FPGA_UVC_ENABLE                        = 1,             /* UVC Enable */

    FPGA_HEADER_CTRL_ADDRESS               = 0x02,          /* Header Addition Control address */
    FPGA_HEADER_ENABLE                     = 1,             /* FPGA adds 32 byte UVC header */
    FPGA_HEADER_DISABLE                    = 0,             /* FX2G3 adds 32 byte UVC header */

    FPGA_ACTIVE_DIVICE_MASK_ADDRESS        = 0x08,          /* Device Selection Mask */


    /*Device related Info*/
    DEVICE0_OFFSET                         = 0x20,          /* FPGA Device 0 Offset */
    DEVICE1_OFFSET                         = 0x3C,          /* FPGA Device 1 Offset */
    DEVICE2_OFFSET                         = 0x58,          /* FPGA Device 2 Offset */
    DEVICE3_OFFSET                         = 0x74,          /* FPGA Device 3 Offset */
    DEVICE4_OFFSET                         = 0x90,          /* FPGA Device 4 Offset */
    DEVICE5_OFFSET                         = 0xAC,          /* FPGA Device 5 Offset */
    DEVICE6_OFFSET                         = 0xC8,          /* FPGA Device 6 Offset */
    DEVICE7_OFFSET                         = 0xE4,          /* FPGA Device 7 Offset */

    FPGA_DEVICE_STREAM_ENABLE_ADDRESS      = 0x00,         /* FPGA Data Stream Enable Address */
    DATA_DISABLE                           = 0x00,         /* Disable Data Stream */
    DMA_CH_RESET                           = 0x01,         /* DMA Channel Reset */
    DATA_ENABLE                            = 0x02,         /* Enable Data Stream */

    FPGA_DEVICE_STREAM_MODE_ADDRESS        = 0x01,         /* Device Sata Stream Mode  */
    NO_CONVERSION                          = 0,            /* RAW Data Stream  */

    DEVICE_IMAGE_HEIGHT_LSB_ADDRESS        = 0x02,         /* Video Image Height (LSB)  */
    DEVICE_IMAGE_HEIGHT_MSB_ADDRESS        = 0x03,         /* Video Image Height (MSB)  */
    DEVICE_IMAGE_WIDTH_LSB_ADDRESS         = 0x04,         /* Video Image Width (LSB)  */
    DEVICE_IMAGE_WIDTH_MSB_ADDRESS         = 0x05,         /* Video Image Width (MSB)  */
    
    DEVICE_FPS_ADDRESS                     = 0x06,         /* Video frames per second*/

    DEVICE_PIXEL_WIDTH_ADDRESS             = 0x07,         /* Video Bits per pixel */
    _8_BIT_PIXEL                           = 8,
    _12BIT_PIXEL                           = 12,
    _16BIT_PIXEL                           = 16,
    _24BIT_PIXEL                           = 24,
    _36BIT_PIXEL                           = 36,

    DEVICE_SOURCE_TYPE_ADDRESS             = 0x08,        /* Data Source Type */
    INTERNAL_COLORBAR                      = 0x00,        /* Internal Colorbar */
    MIPI_SOURCE                            = 0x02,        /* MIPI Source */

    DEVICE_FLAG_STATUS_ADDRESS             = 0x09,        /* DMA Flag Status Register*/

    DEVICE_MIPI_STATUS_ADDRESS             = 0x0A,        /* MIPI Interface Status Register */

    DEVICE_SOURCE_INFO_ADDRESS             = 0x0B,        /* Device  Source Info Register */
    SOURCE_DISCONNECT                      = 0x00,        /* Source Disconnect */


    DEVICE_ACTIVE_TREAD_INFO_ADDRESS       = 0x0F,        /* GPIF Threads Info */
    DEVICE_THREAD1_INFO_ADDRESS            = 0x10,        /* Thread1 Info */
    DEVICE_THREAD2_INFO_ADDRESS            = 0x11,        /* Thread2 Info */
    DEVICE_THREAD1_SOCKET_INFO_ADDRESS     = 0x12,        /* Thread1 - Socket  Info */
    DEVICE_THREAD2_SOCKET_INFO_ADDRESS     = 0x13,        /* Thread2 - Socket  Info */

    DEVICE_FLAG_INFO_ADDRESS               = 0x14,        /* Device Info Address */
    FX2G3_READY_TO_REC_DATA                = 0x08,
    NEW_UVC_PACKET_START                   = 0x02,
    NEW_FRAME_START                        = 0x01,

    DEVICE_BUFFER_SIZE_LSB_ADDRESS         = 0x16,       /* Device DMA Buffer Size (LSB) Address */
    DEVICE_BUFFER_SIZE_MSB_ADDRESS         = 0x17,       /* Device DMA Buffer Size (MSB) Address */

} cy_en_i2c_fpgaRegMap_t;


void Cy_USB_AppInit(cy_stc_usb_app_ctxt_t *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, 
                    DMAC_Type *pCpuDmacBase, DW_Type *pCpuDw0Base, DW_Type *pCpuDw1Base, 
                    cy_stc_hbdma_mgr_context_t *pHbDmaMgrCtxt);

void Cy_USB_AppRegisterCallback(cy_stc_usb_app_ctxt_t *pAppCtxt);

void Cy_USB_AppSetCfgCallback(void *pAppCtxt,
                              cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                              cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppBusResetCallback(void *pAppCtxt,
                                cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppBusResetDoneCallback(void *pAppCtxt,
                                    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                    cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppBusSpeedCallback(void *pAppCtxt,
                                cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppSetupCallback(void *pAppCtxt,
                             cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                             cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppSuspendCallback(void *pAppCtxt,
                               cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                               cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppResumeCallback (void *pAppCtxt,
                               cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                               cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppSetIntfCallback(void *pAppCtxt,
                               cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                               cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppL1SleepCallback(void *pUsbApp,
                               cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                               cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppL1ResumeCallback(void *pUsbApp,
                                cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppZlpCallback(void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                           cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppSlpCallback(void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                           cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppSetFeatureCallback(void *pUsbApp,
                                  cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                  cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppClearFeatureCallback(void *pUsbApp,
                                  cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                  cy_stc_usb_cal_msg_t *pMsg);
/* Function to register all USB descriptors with the stack. */
void Cy_USB_RegisterUsbDescriptors(cy_stc_usb_app_ctxt_t *pAppCtxt,
                                    cy_en_usb_speed_t usbSpeed);
void Cy_USB_AppQueueRead (cy_stc_usb_app_ctxt_t *pAppCtxt,
        uint8_t endpNumber, uint8_t *pBuffer, uint16_t dataSize);
uint16_t Cy_USB_AppReadShortPacket(cy_stc_usb_app_ctxt_t *pAppCtxt,
        uint8_t endpNumber, uint16_t pktSize);
void Cy_USB_AppQueueWrite (cy_stc_usb_app_ctxt_t *pAppCtxt,
        uint8_t endpNumber, uint8_t *pBuffer, uint16_t dataSize);
void Cy_USB_AppInitDmaIntr(uint32_t endpNumber,
        cy_en_usb_endp_dir_t endpDirection,
        cy_israddress userIsr);
void Cy_USB_AppClearDmaInterrupt(cy_stc_usb_app_ctxt_t *pAppCtxt,
        uint32_t endpNumber, cy_en_usb_endp_dir_t endpDirection);
void CyUvcAppHandleSendCompletion(cy_stc_usb_app_ctxt_t *pUsbApp);

void CY_UVC_DataWire_ISR(void);

/* Enable the USB data connection. */
bool Cy_USB_EnableUsbHSConnection(cy_stc_usb_app_ctxt_t *pAppCtxt);

/* Disable the USB data connection. */
void Cy_USB_DisableUsbHSConnection(cy_stc_usb_app_ctxt_t *pAppCtxt);

/* Initialize the LVDS/LVCMOS interface to receive the video data through. */
void Cy_UVC_LvdsInit(void);

/* Configure USB endpoints based on the settings provided in the endpoint descriptor. */
void Cy_USB_AppConfigureEndp(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, uint8_t *pEndpDscr);

/* Configure application level DMA resources corresponding to a USB endpoint. */
void Cy_USB_AppSetupEndpDmaParamsHs(cy_stc_usb_app_ctxt_t *pUsbApp, uint8_t *pEndpDscr);

/* Initialize the PDM IP to receive data from PDM microphone. */
void Cy_UAC_PdmInit(void);

/* De-initialize the PDM IP. */
void Cy_UAC_PdmDeInit(void);

/* Enable or disable the interrupt handlers for the DataWire channels reading
 * data out from the PDM receive FIFOs.
 */
void Cy_USB_App_PDM_InitDmaIntr(bool enable);

/* Interrupt service routine for completion of data transfer on USB 2.x
 * IN endpoint used for audio streaming.
 */
void Cy_PDM_InEpDma_ISR(void);

/* Initialize UAC application context structures. */
void Cy_UAC_AppInit(cy_stc_usb_app_ctxt_t *pAppCtxt);

/* Queue a read using DataWire channels from the PDM receive FIFO. */
void Cy_PDM_QueuePDMRead(cy_stc_usb_app_ctxt_t *pAppCtxt, uint32_t dataLength);

/* Handler for SET_INTERFACE requests addressed to the UAC streaming interface. */
void Cy_App_SetUACIntfHandler(cy_stc_usb_app_ctxt_t *pAppCtxt, uint8_t altSetting);

void CyUvcAppGpifIntr(void *pApp);

void Cy_USB_App_FreeAllDmaBuffers(cy_stc_usb_app_ctxt_t *pAppCtxt);

uint8_t *Cy_USB_App_GetDmaBuffer(cy_stc_usb_app_ctxt_t *pAppCtxt, uint16_t sz_bytes);

void Cy_CheckStatus(const char *function, uint32_t line, uint8_t condition,uint32_t value, uint8_t isBlocking);

void Cy_USB_SendEndp0DataFailHandler(void);

void Cy_CheckStatusHandleFailure(const char *function, uint32_t line, uint8_t condition, uint32_t value, uint8_t isBlocking, void (*failureHandler)());

#if defined(__cplusplus)
}
#endif

#endif /* _CY_USB_APP_H_ */

/* End of File */

