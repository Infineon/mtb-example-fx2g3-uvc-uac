/***************************************************************************//**
* \file usb_app.c
* \version 1.0
*
* \brief Implements the USB data handling part of the USB Video Class application.
*
*******************************************************************************
* \copyright
* (c) (2026), Cypress Semiconductor Corporation (an Infineon company) or
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

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "cy_pdl.h"
#include "cy_device.h"
#include "cy_usb_common.h"
#include "cy_usbhs_cal_drv.h"
#include "cy_hbdma.h"
#include "cy_hbdma_mgr.h"
#include "cy_usb_usbd.h"
#include "cy_usbhs_dw_wrapper.h"
#include "usb_uvc_device.h"
#include "usb_app.h"
#include "cy_debug.h"
#include "cy_lvds.h"
#include "usb_i2c.h"
#include "usb_qspi.h"
#include "usb_imagesensor.h"

/* Sensor configuration status */
extern bool glIsSensorConfigured;
extern uint8_t CyFxUSB20DeviceDscr[];
extern cy_stc_hbdma_buf_mgr_t HBW_BufMgr;

/* GPIF State Machine number */
uint8_t cy_u3v_smNo = 0;

/* Buffer counter for different resolution */
static uint16_t glUvcCommitLength = 0, glUvcBufferCounter = 1;

/* Default Frame Index is  1080p  as 1st index */
static uint16_t glUvcCurrentFrameIndex = CY_USB_UVC_HS_1080P_FRAME_INDEX;
static uint16_t glUvcFullBufferCount = CY_USB_FULL_BUFFER_NO_1920_1080;
#if FPGA_ENABLE
static bool glIsFPGARegConfigured = false;
static bool glIsFPGAConfigureComplete = false;
#endif /* FPGA_ENABLE */
static volatile bool glUvcIsApplnActive = false;
/* Video Probe Commit Control */
static uint8_t glUvcCommitCtrl[CY_USB_UVC_MAX_PROBE_SETTING_ALIGNED];

/* Whether SET_CONFIG is complete or not */
static volatile bool glUvcDevConfigured = false;

/* USBHS: 640*480  @60 fps */
static uint8_t glUvcProbeCtrlVGA[] = {
    0x00, 0x00,                                    /* bmHint : no hit */
    0x01,                                               /* Use 1st Video format index */
    CY_USB_UVC_HS_VGA_FRAME_INDEX,                       /* Use 1st Video frame index */
    0x0A, 0x8B, 0x02, 0x00,              /* Desired frame interval in the unit of 100ns: 60 fps */
    0x00, 0x00,                                      /* Key frame rate in key frame/video frame units*/
    0x00, 0x00,                                  /* PFrame rate in PFrame / key frame units */
    0x00, 0x00,                                  /* Compression quality control */
    0x00, 0x00,                                  /* Window size for average bit rate */
    0x00, 0x00,                                  /* Internal video streaming i/f latency in ms */
    0x00, 0x60, 0x09, 0x00,          /* Max video frame size in bytes: 640*480*2 = 0x00096000 */

#if UVC_INMEM_EN
    CY_USB_DWORD_GET_BYTE0(CY_USB_FULL_FRAME_SIZE_VGA),      /* No. of bytes device can rx in single payload*/
    CY_USB_DWORD_GET_BYTE1(CY_USB_FULL_FRAME_SIZE_VGA),
    CY_USB_DWORD_GET_BYTE2(CY_USB_FULL_FRAME_SIZE_VGA),
    CY_USB_DWORD_GET_BYTE3(CY_USB_FULL_FRAME_SIZE_VGA),

#else
    CY_USB_DWORD_GET_BYTE0(CY_USB_UVC_STREAM_BUF_SIZE), /* No. of bytes device can rx in single payload*/
    CY_USB_DWORD_GET_BYTE1(CY_USB_UVC_STREAM_BUF_SIZE),
    CY_USB_DWORD_GET_BYTE2(CY_USB_UVC_STREAM_BUF_SIZE),
    CY_USB_DWORD_GET_BYTE3(CY_USB_UVC_STREAM_BUF_SIZE),
#endif /* UVC_INMEM_EN */
    0x00, 0x60, 0xE3, 0x16,             /* Device Clock */
    0x00, 0x00, 0x00, 0x00              /* Framing and format information */
};

/* 1920*1080 @15 fps */
static uint8_t glUvcProbeCtrl1080p[]= {
    0x00, 0x00,                                      /* bmHint : no hit */
    0x01,                                                 /* Use 1st Video format index */
    CY_USB_UVC_HS_1080P_FRAME_INDEX,                      /* Use 2nd Video frame index */
    0x15, 0x16, 0x05, 0x00,                /* Desired frame interval in the unit of 100ns: 30 fps */
    0x00, 0x00,                                      /* Key frame rate in key frame/video frame units */
    0x00, 0x00,                                    /* PFrame rate in PFrame / key frame units */
    0x00, 0x00,                                    /* Compression quality control */
    0x00, 0x00,                                    /* Window size for average bit rate*/
    0x00, 0x00,                                    /* Internal video streaming i/f latency in ms */
    0x00, 0x48, 0x3F, 0x00,            /* Max video frame size in bytes: 1920*1080*2 = 0x0003f4800 */
 #if UVC_INMEM_EN
    CY_USB_DWORD_GET_BYTE0(CY_USB_FULL_FRAME_SIZE_1080P),      /* No. of bytes device can rx in single payload*/
    CY_USB_DWORD_GET_BYTE1(CY_USB_FULL_FRAME_SIZE_1080P),
    CY_USB_DWORD_GET_BYTE2(CY_USB_FULL_FRAME_SIZE_1080P),
    CY_USB_DWORD_GET_BYTE3(CY_USB_FULL_FRAME_SIZE_1080P),
#else
    CY_USB_DWORD_GET_BYTE0(CY_USB_UVC_STREAM_BUF_SIZE),  /* No. of bytes device can rx in single payload*/
    CY_USB_DWORD_GET_BYTE1(CY_USB_UVC_STREAM_BUF_SIZE),
    CY_USB_DWORD_GET_BYTE2(CY_USB_UVC_STREAM_BUF_SIZE),
    CY_USB_DWORD_GET_BYTE3(CY_USB_UVC_STREAM_BUF_SIZE),
#endif /* UVC_INMEM_EN */
    0x00, 0x60, 0xE3, 0x16,             /* Device Clock */
    0x00, 0x00, 0x00, 0x00              /* Framing and format information */
};

/* UVC Header */
static uint8_t glUvcHeader[] = {
    32,                                 /* Header Length */
    0x8C,                               /* Bit field header field */
    0x00, 0x00, 0x00, 0x00,             /* Presentation time stamp field */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* Source clock reference field */
    0x00, 0x00, 0x00, 0x00, 0x00,       /* Zero padding for 32 byte align */
    0x00, 0x00, 0x00, 0x00, 0x00,       /* Zero padding for 32 byte align */
    0x00, 0x00, 0x00, 0x00, 0x00,       /* Zero padding for 32 byte align */
    0x00, 0x00, 0x00, 0x00, 0x00        /* Zero padding for 32 byte align */
};

uint32_t Ep0TestBuffer[32U]__attribute__ ((aligned (32)));

#if FPGA_ENABLE
/**
 * \name Cy_UVC_SendResolution
 * \brief Read the i2c
 * \param width
 * \param height
 * \param device_offset
 * \retval 0 for read success, error code for unsuccess.
 */
uint32_t Cy_UVC_SendResolution (uint16_t width, uint16_t height, uint8_t device_offset)
{
    cy_en_scb_i2c_status_t  status = CY_SCB_I2C_SUCCESS;
    
    uint8_t deviceNum = (device_offset - DEVICE0_OFFSET) / FPGA_STREAMING_DEV_REG_COUNT;

    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE_IMAGE_WIDTH_MSB_ADDRESS(deviceNum),CY_USB_GET_MSB(width),
                                              FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE_IMAGE_WIDTH_LSB_ADDRESS(deviceNum),CY_USB_GET_LSB(width),
                                              FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE_IMAGE_HEIGHT_MSB_ADDRESS(deviceNum),CY_USB_GET_MSB(height),
                                              FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE_IMAGE_HEIGHT_LSB_ADDRESS(deviceNum),CY_USB_GET_LSB(height),
                                              FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    status = Cy_I2C_Write(FPGASLAVE_ADDR,FPGA_DEVICE_STREAM_MODE_ADDRESS(deviceNum),NO_CONVERSION,
                                              FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE_FPS_ADDRESS(deviceNum),FPS_DEFAULT,
                                              FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);
    DBG_APP_INFO("UVC: FPS %d \r\n", FPS_DEFAULT);
    return status;
} /* End of Cy_UVC_SendResolution() */

/**
 * \name Cy_UVC_DataStreamStartStop
 * \brief Starts/Stops FPGA data streaming
 * \param device_offset device number (fpga implementation specific)
 * \param IsStreamStart Pass 1 to start streaming from fpga elase 0
 * \retval 0 for read success, error code for unsuccess.
 */
cy_en_scb_i2c_status_t Cy_UVC_DataStreamStartStop(uint8_t device_offset, cy_en_streamControl_t IsStreamStart)
{
    cy_en_scb_i2c_status_t status = CY_SCB_I2C_SUCCESS;

    uint8_t deviceNum = (device_offset - DEVICE0_OFFSET) / FPGA_STREAMING_DEV_REG_COUNT;

    if (device_offset == DEVICE0_OFFSET)
        glUvcIsApplnActive = IsStreamStart?true:false;

    if(IsStreamStart)
        status = Cy_I2C_Write(FPGASLAVE_ADDR,(FPGA_DEVICE_STREAM_ENABLE_ADDRESS(deviceNum)),DATA_ENABLE,
                                            FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    else

        status = Cy_I2C_Write(FPGASLAVE_ADDR,(FPGA_DEVICE_STREAM_ENABLE_ADDRESS(deviceNum)),DATA_DISABLE,
                                            FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);

    if(status == CY_SCB_I2C_SUCCESS)
    {
        DBG_APP_INFO("UVC: Video Start = %d FPGA deviceNum 0x%x\r\n",IsStreamStart, deviceNum);
    }

    return status;
} /* End of Cy_UVC_DataStreamStartStop() */


/**
 * \name Cy_UVC_SetVideoResolution
 * \brief Read the i2c
 * \param format_index Video format Index
 * \param resolution_index Video resolution Index
 * \param device_offset device number (fpga implementation specific)
 * \param devSpeed device USB speed
 * \retval 0 for read success, error code for unsuccess.
 */
uint32_t Cy_UVC_SetVideoResolution (uint8_t format_index, uint8_t resolution_index, uint8_t device_offset, uint8_t devSpeed)
{
    uint32_t status = 0;
    DBG_APP_INFO("UVC: Format_index: %d Resolution_index:%d Device Speed: %d\r\n", format_index,resolution_index, devSpeed);

    switch(resolution_index)
    {
        case CY_USB_UVC_HS_VGA_FRAME_INDEX:
        /*I2C sensor reg writes can be added here*/
        status = Cy_UVC_SendResolution (H_RES_640, V_RES_480, device_offset);
#if MIPI_SOURCE_ENABLE
        /* Set res for image sensor */
        Cy_UVC_ImageSensorSetResolution(H_RES_640, V_RES_480);
#endif /* MIPI_SOURCE_ENABLE */
        break;
        case CY_USB_UVC_HS_1080P_FRAME_INDEX:
        /*I2C sensor reg writes can be added here*/
        status = Cy_UVC_SendResolution (H_RES_1920,  V_RES_1080, device_offset);
#if MIPI_SOURCE_ENABLE
        /* Set res for image sensor */
        Cy_UVC_ImageSensorSetResolution(H_RES_1920, V_RES_1080);
#endif /* MIPI_SOURCE_ENABLE */
        break;
    }

    return status;
} /* End of Cy_UVC_SetVideoResolution() */

/**
 * \name Cy_UVC_ConfigFpgaRegister
 * \brief   FPGA Register Writes. FPGA is configured to send internally generated
 *          colorbar data over FX2G3's SlaveFIFO Interface
 * \retval 0 for read success, error code for unsuccess.
 */
static cy_en_scb_i2c_status_t Cy_UVC_ConfigFpgaRegister (void)
{
    cy_en_scb_i2c_status_t status = CY_SCB_I2C_SUCCESS;
    /* Defalut resolution*/
    uint16_t width = H_RES_1920;
    uint16_t height = V_RES_1080;

    /* Disable camera before configuring FPGA register */
    status = Cy_I2C_Write(FPGASLAVE_ADDR,FPGA_DEVICE_STREAM_ENABLE_ADDRESS(0),DATA_DISABLE,
                                              FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    /* write FPGA register to enable UVC */
    status = Cy_I2C_Write(FPGASLAVE_ADDR,FPGA_UVC_SELECTION_ADDRESS,FPGA_UVC_ENABLE,
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

#if UVC_HEADER_BY_FPGA
        /* Disable adding UVC header by FPGA. UVC header is added by FX2G3 */
    status = Cy_I2C_Write(FPGASLAVE_ADDR,FPGA_HEADER_CTRL_ADDRESS,FPGA_HEADER_ENABLE,
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
#elif UVC_HEADER_BY_FX2G3
        /* Disable adding UVC header by FPGA. UVC header is added by FX2G3 */
    status = Cy_I2C_Write(FPGASLAVE_ADDR,FPGA_HEADER_CTRL_ADDRESS,FPGA_HEADER_DISABLE,
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
#endif /* UVC_HEADER_BY_FPGA */
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);
    Cy_SysLib_DelayUs(500);


    /* Number of active device list*/
    status = Cy_I2C_Write(FPGASLAVE_ADDR,FPGA_ACTIVE_DEVICE_MASK_ADDRESS,0x01,
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);

    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);
    Cy_SysLib_DelayUs(500);

    /* write FPGA register to Disable format converstion */
    status = Cy_I2C_Write(FPGASLAVE_ADDR,FPGA_DEVICE_STREAM_MODE_ADDRESS(0),NO_CONVERSION,
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

#if (MIPI_SOURCE_ENABLE)
    /* Select MIPI source */
    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE_SOURCE_TYPE_ADDRESS(0),MIPI_SOURCE,
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);

#else
    /* Select colorbar mode by default if DYNAMIC_VIDEOSOURCE is true*/
    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE_SOURCE_TYPE_ADDRESS(0),INTERNAL_COLORBAR,
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
#endif /* (MIPI_SOURCE_ENABLE) */
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

#if INTERLEAVE_EN
    /*
     * In Wide link mode, actual socket number each thread is connected to is derived from SSAD[kkk] control byte sent by FPGA.
     * Actual socket number =  (3'bkkk x 2) + tt[0] on adapter tt[1], where tt is the thread number sent in STAD[tt]
     * if FPGA sets kkk as 0 for Thread 0, actual socket number connected to Thread 0 is 0
     * if FPGA sets kkk as 0 for Thread 1, actual socket number connected to Thread 1 is 1
     */
    /* Inform active threads*/
    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE_ACTIVE_TREAD_INFO_ADDRESS(0),2,
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    /* inform active sockets*/
    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE_THREAD2_SOCKET_INFO_ADDRESS(0),CY_LVDS_GPIF_THREAD_1,
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    /* */
    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE_THREAD2_INFO_ADDRESS(0),1,
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

#else /* Single Thread*/
     /* Inform active threads*/
    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE_ACTIVE_THREAD_INFO_ADDRESS(0),1,
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    /* Inform active sockets*/
    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE_THREAD2_SOCKET_INFO_ADDRESS(0),0,
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    /* Thread 2 information*/
    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE_THREAD2_INFO_ADDRESS(0),0,
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);
#endif /* INTERLEAVE_EN */

    /* Thread 1 information*/
    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE_THREAD1_INFO_ADDRESS(0),CY_LVDS_GPIF_THREAD_0,
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    /* Thread 1 socket information*/
    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE_THREAD1_SOCKET_INFO_ADDRESS(0),0,
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    /* Clear FPGA register during power up, this will get update when firmware detects HDMI */
    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE_SOURCE_INFO_ADDRESS(0),SOURCE_DISCONNECT,
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    DBG_APP_INFO("UVC: DMA Buffer Size %d \r\n",FPGA_DMA_BUFFER_SIZE);

    /* Update DMA buffer size used by Firmware */
    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE_BUFFER_SIZE_MSB_ADDRESS(0),CY_GET_MSB(FPGA_DMA_BUFFER_SIZE),
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE_BUFFER_SIZE_LSB_ADDRESS(0),CY_GET_LSB(FPGA_DMA_BUFFER_SIZE),
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    /* Update default resolution width size used by Firmware */
    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE_IMAGE_WIDTH_MSB_ADDRESS(0),CY_GET_MSB(width),
                                              FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE_IMAGE_WIDTH_LSB_ADDRESS(0),CY_GET_LSB(width),
                                              FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

 /* Update default resolution height size used by Firmware */
    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE_IMAGE_HEIGHT_MSB_ADDRESS(0),CY_GET_MSB(height),
                                              FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE_IMAGE_HEIGHT_LSB_ADDRESS(0),CY_GET_LSB(height),
                                              FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    /* default fps*/
    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE_FPS_ADDRESS(0),60,
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    return status;
} /* End of Cy_UVC_ConfigFpgaRegister() */
#endif

/**
 * \name Cy_UVC_AppStart
 * \brief Start the data stream channel/s
 * \param pAppCtxt application layer context pointer
 * \param format_index Video format index
 * \param frame_index Video resolution index
 * \param DeviceIndex device number (fpga implementation specific)
 * \retval None
 */
static cy_en_hbdma_mgr_status_t Cy_UVC_AppStart(cy_stc_usb_app_ctxt_t *pAppCtxt,uint32_t format_index, uint32_t frame_index, uint16_t DeviceIndex)
{
    cy_en_hbdma_mgr_status_t mgrStatus = CY_HBDMA_MGR_SUCCESS;



    if(glUvcIsApplnActive == true)
    {
        DBG_APP_INFO("App already started \r\n");
    }
    else
    {
        DBG_APP_INFO("UVC: App Start\r\n");

        if (DEVICE0_OFFSET == DeviceIndex)
        {
            mgrStatus = Cy_HBDma_Channel_Enable(pAppCtxt->hbBulkInChannel, 0);
            ASSERT_NON_BLOCK(mgrStatus == CY_HBDMA_MGR_SUCCESS, mgrStatus);

            pAppCtxt->glfps                  = 0;
            pAppCtxt->glDmaBufCnt            = 0;
            pAppCtxt->glProd                 = 0;
            pAppCtxt->glCons                 = 0;
            pAppCtxt->glProdCount            = 0;
            pAppCtxt->glConsCount            = 0;
            pAppCtxt->glFrameSizeTransferred = 0;
            pAppCtxt->glFrameSize            = 0;
            pAppCtxt->glPartialBufSize       = 0;
            glUvcIsApplnActive             = true;

        }

#if FPGA_ENABLE
        if (Cy_UVC_SetVideoResolution(format_index, frame_index, DeviceIndex, pAppCtxt->devSpeed) != 0)
        {
            DBG_APP_ERR("UVC: Error Setting Resolution\r\n");
        }

        Cy_LVDS_GpifSMSwitch(LVDSSS_LVDS, 0, 255, 0, 0, 12, 2 );
        DBG_APP_INFO("UVC: GPIF SM Switch line %d\r\n", __LINE__);

        Cy_UVC_DataStreamStartStop(DeviceIndex, STREAM_START);
#endif /* FPGA_ENABLE */

    }
    return mgrStatus;
}

/**
 * \name Cy_UVC_AppStop
 * \brief Stop the data stream channels
 * \param pAppCtxt application layer context pointer
 * \param pUsbdCtxt USBD layer context pointer
 * \param wIndex USB streaming endpoint numbder
 * \retval None
 */
static void Cy_UVC_AppStop(cy_stc_usb_app_ctxt_t *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, uint16_t wIndex)
{
    cy_en_hbdma_mgr_status_t status = CY_HBDMA_MGR_SUCCESS;
    uint32_t epNumber = ((uint32_t)wIndex & 0x7FUL);
#if !UVC_INMEM_EN
    uint8_t index = 0;
    cy_stc_hbdma_sock_t sckStat;
    uint32_t pollCnt = 0;
#endif /* !UVC_INMEM_EN */

    if(glUvcIsApplnActive != true)
    {
        DBG_APP_INFO("UVC: App Already Stopped \r\n");
    }else {
        DBG_APP_INFO("UVC: App Stop \r\n");

        if(pAppCtxt->hbBulkInChannel != NULL)
        {
#if !UVC_INMEM_EN
#if INTERLEAVE_EN
            for(index = 0; index <= CY_USB_UVC_STREAM_BUF_COUNT; index ++)
            {
                pollCnt = 0;
                do {
                    Cy_SysLib_DelayUs(10);
                    Cy_HBDma_GetSocketStatus(pAppCtxt->pHbDmaMgrCtxt->pDrvContext, pAppCtxt->hbBulkInChannel->prodSckId[0], &sckStat);
                } while(
                        ((_FLD2VAL(LVDSSS_LVDS_ADAPTER_DMA_SCK_STATUS_STATE,sckStat.status)) != 0x1) &&
                        (pollCnt++ < 500)
                       );

               pollCnt = 0;

                do {
                    Cy_SysLib_DelayUs(10);
                    Cy_HBDma_GetSocketStatus(pAppCtxt->pHbDmaMgrCtxt->pDrvContext, pAppCtxt->hbBulkInChannel->prodSckId[1], &sckStat);
                } while(
                        ((_FLD2VAL(LVDSSS_LVDS_ADAPTER_DMA_SCK_STATUS_STATE,sckStat.status)) != 0x1) &&
                        (pollCnt++ < 500)
                       );
             }

            DBG_APP_INFO("UVC: DMA Socket %x is stalled (%x)\r\n", pAppCtxt->hbBulkInChannel->prodSckId[0],
                    sckStat.status);
            DBG_APP_INFO("UVC: DMA Socket %x is stalled (%x)\r\n", pAppCtxt->hbBulkInChannel->prodSckId[1],
                    sckStat.status);
#else
            do {
                Cy_SysLib_DelayUs(10);
                Cy_HBDma_GetSocketStatus(pAppCtxt->pHbDmaMgrCtxt->pDrvContext, pAppCtxt->hbBulkInChannel->prodSckId[index], &sckStat);
            } while(
                    ((_FLD2VAL(LVDSSS_LVDS_ADAPTER_DMA_SCK_STATUS_STATE,sckStat.status)) != 0x1) &&
                    (pollCnt++ < 500)
                   );

            DBG_APP_INFO("UVC: DMA Socket %x is stalled (%x)\r\n", pAppCtxt->hbBulkInChannel->prodSckId[index],
                    sckStat.status);
#endif /* INTERLEAVE_EN */
#endif /* !UVC_INMEM_EN */

#if FPGA_ENABLE
        if(UVC_STREAM_ENDPOINT == epNumber){
            Cy_UVC_DataStreamStartStop(DEVICE0_OFFSET, STREAM_STOP);
        }
#endif /* FPGA_ENABLE */

            if (UVC_STREAM_ENDPOINT == epNumber)
            {
                /* Reset the DMA channel through which data is received from the LVDS side */
                status = Cy_HBDma_Channel_Reset(pAppCtxt->hbBulkInChannel);
                ASSERT_NON_BLOCK(CY_HBDMA_MGR_SUCCESS == status,status);

                glUvcIsApplnActive             = false;
                pAppCtxt->glfps                  = 0;
                pAppCtxt->glDmaBufCnt            = 0;
                pAppCtxt->glProd                 = 0;
                pAppCtxt->glCons                 = 0;
                pAppCtxt->glProdCount            = 0;
                pAppCtxt->glConsCount            = 0;
                pAppCtxt->glFrameSizeTransferred = 0;
                pAppCtxt->glFrameSize            = 0;
                pAppCtxt->glPartialBufSize       = 0;
                DBG_APP_INFO("UVC: DMA Reset and Variable Cleared\r\n");
            }
        }

        /* Flush and reset the endpoint and clear the STALL bit */
        Cy_USBD_FlushEndp(pUsbdCtxt, epNumber, CY_USB_ENDP_DIR_IN);

        Cy_USBD_ResetEndp(pUsbdCtxt, epNumber, CY_USB_ENDP_DIR_IN, false);
        Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, (cy_en_usb_endp_dir_t)epNumber, CY_USB_ENDP_DIR_IN, false);
    }
}

/**
 * \name Cy_UVC_AddHeader
 * \brief UVC header addition function
 * \param buffer_p Buffer pointer
 * \param frameInd Video fram ID
 * \retval None
 */
void Cy_UVC_AddHeader(
    uint8_t *buffer_p, /* Buffer pointer */
    uint8_t frameInd   /* EOF or normal frame indication */
)
{
    /* Check if last packet of the frame */
    if (frameInd == CY_USB_UVC_HEADER_EOF)
    {
        /* Indicate End of Frame */
        glUvcHeader[1] |= CY_USB_UVC_HEADER_EOF;

        /* Copy header to buffer */
        memcpy(buffer_p, (uint8_t *)glUvcHeader, CY_USB_UVC_MAX_HEADER);

        /* Toggle frame ID */
        glUvcHeader[1] = (glUvcHeader[1] & ~CY_USB_UVC_HEADER_EOF) ^ CY_USB_UVC_HEADER_FRAME_ID;
    }
    else
    {
        /* Copy header to buffer */
        memcpy(buffer_p, (uint8_t *)glUvcHeader, CY_USB_UVC_MAX_HEADER);
    }
} /* end of function */

/**
 * \name Cy_USB_AppDisableEndpDma
 * \brief This function de-inits all active USB DMA channels as part of USB disconnect process.
 * \param pAppCtxt application layer context pointer
 * \retval None
 */
void
Cy_USB_AppDisableEndpDma (cy_stc_usb_app_ctxt_t *pAppCtxt)
{
    /* Disable and destroy the High BandWidth DMA channels */
    if (pAppCtxt->hbBulkInChannel != NULL) {

#if UVC_INMEM_EN
        Cy_UvcInMem_ClearBufPointers(pAppCtxt, pAppCtxt->hbBulkInChannel);
#endif /* UVC_INMEM_EN */

        Cy_HBDma_Channel_Disable(pAppCtxt->hbBulkInChannel);
        Cy_HBDma_Channel_Destroy(pAppCtxt->hbBulkInChannel);
        pAppCtxt->hbBulkInChannel = NULL;
    }

#if AUDIO_IF_EN
        /* Free up the high bandwidth DMA channel if present */
        if (pAppCtxt->pPDMToUsbChn != NULL) {
            Cy_HBDma_Channel_Disable(pAppCtxt->pPDMToUsbChn);
            Cy_HBDma_Channel_Destroy(pAppCtxt->pPDMToUsbChn);
            pAppCtxt->pPDMToUsbChn = NULL;
        }
#endif /* AUDIO_IF_EN */
}

/**
 * \name Cy_USB_UvcGetCurRqtHandler
 * \brief This function handles the GET request addresses to the UVC video streaming interface.
 * \param pAppCtxt application layer context pointer
 * \param wLength wLength field of control request
 * \param wValue wValue field of control request
 * \retval None
 */
void Cy_USB_UvcGetCurRqtHandler (cy_stc_usb_app_ctxt_t *pAppCtxt,uint8_t bRequest)
{
    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt = pAppCtxt->pUsbdCtxt;
    cy_en_usbd_ret_code_t retStatus = CY_USBD_STATUS_SUCCESS;


    DBG_APP_TRACE("UVC: Get Cur Request Handler\r\n");

    switch (bRequest)
     {
        case CY_USB_UVC_GET_INFO_REQ:
            DBG_APP_TRACE("UVC: Get Info\r\n");
            Ep0TestBuffer[0] = 3; /* GET/SET requests are supported */
            Cy_USB_USBD_SendEndp0Data(pUsbdCtxt, (uint8_t *)Ep0TestBuffer, 0x01);
            break;

        case CY_USB_UVC_GET_LEN_REQ:
            DBG_APP_TRACE("UVC: Get Length\r\n");
            Ep0TestBuffer[0] = CY_USB_UVC_MAX_PROBE_SETTING;
            Cy_USB_USBD_SendEndp0Data(pUsbdCtxt, (uint8_t *)Ep0TestBuffer, 0x01);
            break;

        /* We only have one functional setting. Keep returning the same as current, default
         * minimum and maximum */
        case CY_USB_UVC_GET_CUR_REQ:
        case CY_USB_UVC_GET_DEF_REQ:
        case CY_USB_UVC_GET_MIN_REQ:
        case CY_USB_UVC_GET_MAX_REQ:
            DBG_APP_TRACE("UVC: Get Request Value:%x \r\n", bRequest);

            if (glUvcCurrentFrameIndex == CY_USB_UVC_HS_VGA_FRAME_INDEX)
            {
                retStatus = Cy_USB_USBD_SendEndp0Data(pUsbdCtxt,
                                                        (uint8_t *)glUvcProbeCtrlVGA, CY_USB_UVC_MAX_PROBE_SETTING);
            }
            else if (glUvcCurrentFrameIndex == CY_USB_UVC_HS_1080P_FRAME_INDEX)
            {
                retStatus = Cy_USB_USBD_SendEndp0Data(pUsbdCtxt,
                                                      (uint8_t *)glUvcProbeCtrl1080p, CY_USB_UVC_MAX_PROBE_SETTING);
            }
            else
            {
                DBG_APP_ERR("UVC: Invalid frame Index\r\n");
            }

            if (retStatus != CY_USBD_STATUS_SUCCESS)
            {
                DBG_APP_ERR("UVC: Send Ep0 Failed %x\r\n",retStatus);
            }

            break;
       }
}

/**
 * \name Cy_USB_UvcSetCurRqtHandler
 * \brief This function handles the SET_CUR request addresses to the UVC video streaming interface.
 * \param pAppCtxt application layer context pointer
 * \param wLength wLength field of control request
 * \param wValue wValue field of control request
 * \retval None
 */
void Cy_USB_UvcSetCurRqtHandler (cy_stc_usb_app_ctxt_t *pAppCtxt, uint16_t wLength, uint16_t wValue)
{
    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt = pAppCtxt->pUsbdCtxt;
    cy_en_usbd_ret_code_t retStatus;
    uint16_t loopCnt = 250u;
    cy_stc_usbd_app_msg_t xMsg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    DBG_APP_TRACE("UVC: Set Cur Request Handler\r\n");

    /* Read the data out commitctrl buffer */
    retStatus = Cy_USB_USBD_RecvEndp0Data(pUsbdCtxt, glUvcCommitCtrl, wLength);
    if (retStatus == CY_USBD_STATUS_SUCCESS)
    {
        /* Wait until receive DMA transfer has been completed */
        while ((!Cy_USBD_IsEp0ReceiveDone(pUsbdCtxt)) && (loopCnt--)) {
            Cy_SysLib_DelayUs(10);
        }
        if (!Cy_USBD_IsEp0ReceiveDone(pUsbdCtxt)) {
            DBG_APP_ERR("UVC: Set Cur recv data timed out\r\n");
            Cy_USB_USBD_RetireRecvEndp0Data(pUsbdCtxt);
            Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, 0x00, CY_USB_ENDP_DIR_IN, TRUE);
            return;
        }

        glUvcCurrentFrameIndex = glUvcCommitCtrl[3];
        DBG_APP_TRACE("UVC: Set Cur frame index :%d\r\n", glUvcCurrentFrameIndex);


        if (glUvcCurrentFrameIndex == CY_USB_UVC_HS_VGA_FRAME_INDEX)
        {
            memcpy(&glUvcProbeCtrlVGA[2], &glUvcCommitCtrl[2], CY_USB_UVC_PROBE_CONTROL_UPDATE_SIZE);
            glUvcFullBufferCount = (CY_USB_PARTIAL_BUFFER_640_480 != 0)?CY_USB_FULL_BUFFER_NO_640_480:(CY_USB_FULL_BUFFER_NO_640_480 - 1);

        }
        else if (glUvcCurrentFrameIndex == CY_USB_UVC_HS_1080P_FRAME_INDEX)
        {
            memcpy(&glUvcProbeCtrl1080p[2], &glUvcCommitCtrl[2], CY_USB_UVC_PROBE_CONTROL_UPDATE_SIZE);
            glUvcFullBufferCount = (CY_USB_PARTIAL_BUFFER_1920_1080 != 0)?CY_USB_FULL_BUFFER_NO_1920_1080:(CY_USB_FULL_BUFFER_NO_1920_1080 - 1);
        }
        else
        {
            DBG_APP_ERR("UVC: Invalid Frame Index\r\n");
            glUvcCurrentFrameIndex = CY_USB_UVC_HS_VGA_FRAME_INDEX;
            glUvcFullBufferCount = CY_USB_FULL_BUFFER_NO_640_480;
        }

        if (wValue == CY_USB_UVC_VS_COMMIT_CONTROL)
        {
            /* Reset the Counter */
            glUvcBufferCounter = 1;

            if (Cy_UVC_AppStart(pAppCtxt, glUvcCommitCtrl[2],glUvcCommitCtrl[3], DEVICE0_OFFSET) == CY_HBDMA_MGR_SUCCESS) {
                /* Notify the task that UVC stream has start */
                xMsg.type = CY_USB_UVC_VIDEO_STREAMING_START;
                xQueueSendFromISR(pAppCtxt->uvcMsgQueue, &(xMsg), &(xHigherPriorityTaskWoken));
            }
        }
    }
    else
    {
         DBG_APP_ERR("UVC: Set Cur EP0 Recv Error %x\r\n",retStatus);
    }
}

/**
 * \name Cy_UVC_DeviceTaskHandler
 * \brief This function handles streaming UVC Device.
 * \param pTaskParam task param
 * \note    The actual data forwarding from sensor to USB host is done from the DMA and GPIF callback
 *          functions. The thread is only responsible for checking for streaming start/stop conditions.
 * \retval None
 */
void Cy_UVC_DeviceTaskHandler(void *pTaskParam)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt = (cy_stc_usb_app_ctxt_t *)pTaskParam;
    cy_stc_usbd_app_msg_t queueMsg;
    BaseType_t xStatus;
    uint16_t wIndex;

    vTaskDelay(250);

    /* Reset Frame Id in UVC Header */
    glUvcHeader[1] = CY_USB_UVC_HEADER_DEFAULT_BFH;
    DBG_APP_INFO("UVC: Device Task Handler \r\n");


#if FPGA_ENABLE
#if FPGA_CONFIG_EN
    Cy_FPGAConfigPins(pAppCtxt,FPGA_CONFIG_MODE);
    Cy_QSPI_Start(pAppCtxt);
    Cy_SPI_FlashInit(SPI_FLASH_0, true,false);

    DBG_APP_INFO("FPGA: Configuration Start\r\n");
    glIsFPGAConfigureComplete = Cy_FPGAConfigure(pAppCtxt,FPGA_CONFIG_MODE);
#else
    glIsFPGAConfigureComplete = Cy_IsFPGAConfigured();
#endif /* FPGA_CONFIG_EN */
    if((glIsFPGARegConfigured == false) && (glIsFPGAConfigureComplete == true))
    {
        Cy_APP_GetFPGAVersion(pAppCtxt);
        if(0 == Cy_UVC_ConfigFpgaRegister())
        {
            glIsFPGARegConfigured = true;
            DBG_APP_TRACE("FPGA: Configuration Complete \r\n");
#if MIPI_SOURCE_ENABLE
            Cy_UVC_ConfigureImageSensor();
            if (glIsSensorConfigured == false) {
                /* Select INTERNAL_COLORBAR */

                DBG_APP_TRACE("FPGA: Select Internal Colorbar \r\n");
                cy_en_scb_i2c_status_t status = CY_SCB_I2C_SUCCESS;
                status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE_SOURCE_TYPE_ADDRESS(0), INTERNAL_COLORBAR,
                                                      FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
                ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);
            }
#endif /* MIPI_SOURCE_ENABLE */
        }
        else
        {
            LOG_ERROR("FPGA: Configuration failed \r\n");
        }
    }

#endif /* FPGA_ENABLE */

    /* Initialize the LVDS interface */
    Cy_UVC_LvdsInit();

    vTaskDelay(100);

    /* If VBus is present, enable the USB connection */
    pAppCtxt->vbusPresent =
    (Cy_GPIO_Read(VBUS_DETECT_GPIO_PORT, VBUS_DETECT_GPIO_PIN) == VBUS_DETECT_STATE);
#if USBFS_LOGS_ENABLE
    vTaskDelay(500);
#endif /* USBFS_LOGS_ENABLE */

    if (pAppCtxt->vbusPresent) {
        Cy_USB_EnableUsbHSConnection(pAppCtxt);
    }

    for (;;)
    {
        /*
         * Wait until some data is received from the queue.
         * Timeout after 100 ms.
         */
        xStatus = xQueueReceive(pAppCtxt->uvcMsgQueue, &queueMsg, 100);
        if (xStatus != pdPASS) {
            continue;
        }

        switch (queueMsg.type) {

            case CY_USB_UVC_VBUS_CHANGE_INTR:
                /* Start the debounce timer */
                xTimerStart(pAppCtxt->vbusDebounceTimer, 0);
                break;

            case CY_USB_UVC_VBUS_CHANGE_DEBOUNCED:
                /* Check whether VBus state has changed */
                pAppCtxt->vbusPresent = (Cy_GPIO_Read(VBUS_DETECT_GPIO_PORT, VBUS_DETECT_GPIO_PIN) == VBUS_DETECT_STATE);

                if (pAppCtxt->vbusPresent) {
                    if (!pAppCtxt->usbConnected) {
                        DBG_APP_INFO("USB: Enabling USB connection due to VBus detect\r\n");
                        Cy_USB_EnableUsbHSConnection(pAppCtxt);
                    }
                } else {
                    if (pAppCtxt->usbConnected) {
                        Cy_USB_AppDisableEndpDma(pAppCtxt);
                        DBG_APP_INFO("USB: Disabling USB connection due to VBus removal\r\n");
                        Cy_USB_DisableUsbHSConnection(pAppCtxt);
                    }
                }
                break;

            case CY_USB_UVC_VIDEO_STREAMING_START:
                DBG_APP_INFO("UVC: Stream Start Event\r\n");
#if UVC_INMEM_EN
                Cy_UvcInMem_CommitBuffers(pAppCtxt, pAppCtxt->hbBulkInChannel, glUvcCurrentFrameIndex);
#endif /* UVC_INMEM_EN */
                Cy_USBD_AddEvtToLog(pAppCtxt->pUsbdCtxt, CY_USB_UVC_EVT_VSTREAM_START);
                break;

            case CY_USB_UVC_DEVICE_GET_CUR_RQT:
                Cy_USB_UvcGetCurRqtHandler(pAppCtxt, queueMsg.data[0]);
            break;

            case CY_USB_UVC_DEVICE_SET_CUR_RQT:
                Cy_USB_UvcSetCurRqtHandler(pAppCtxt, queueMsg.data[0], queueMsg.data[1]);
                Cy_USBD_AddEvtToLog(pAppCtxt->pUsbdCtxt, CY_USB_UVC_EVT_SET_CUR_REQ);
                break;
            case CY_USB_UVC_VIDEO_STREAM_STOP_EVENT:
                DBG_APP_INFO("UVC: Stream Stop Event\r\n");
                wIndex = (uint16_t)queueMsg.data[0];
                Cy_UVC_AppStop(pAppCtxt,pAppCtxt->pUsbdCtxt, wIndex);
                break;
            case CY_USB_PRINT_STATUS:
                DBG_APP_INFO("UVC: FPS %d\r\n",queueMsg.data[0]);
                DBG_APP_INFO("UVC: Bytes Transferred %d\r\n",pAppCtxt->glFrameSize);
            break;
            default:
                break;
        }
    }
}

/**
 * \name Cy_USB_PrintFpsCb
 * \brief This Function will be called when timer expires.
 *        This function print event log.
 * \param xTimer
 * \retval None
 */
void
Cy_USB_PrintFpsCb(TimerHandle_t xTimer)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt;

    /* retrieve pAppCtxt */
    pAppCtxt = ( cy_stc_usb_app_ctxt_t *)pvTimerGetTimerID(xTimer);
    if (pAppCtxt->devState == CY_USB_DEVICE_STATE_CONFIGURED) {
        cy_stc_usbd_app_msg_t xMsg;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        if(glUvcIsApplnActive == true)
        {
            xMsg.type = CY_USB_PRINT_STATUS;
            xMsg.data[0] = pAppCtxt->glfps;
            pAppCtxt->glfps = 0;
            xQueueSendFromISR(pAppCtxt->uvcMsgQueue, &(xMsg), &(xHigherPriorityTaskWoken));
        }
    }
}   /* end of function() */

/**
 * \name Cy_USB_VbusDebounceTimerCallback
 * \brief Timer used to do debounce on VBus changed interrupt notification.
 * \param xTimer Timer Handle
 * \retval None
 */
void
Cy_USB_VbusDebounceTimerCallback (TimerHandle_t xTimer)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt = (cy_stc_usb_app_ctxt_t *)pvTimerGetTimerID(xTimer);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    cy_stc_usbd_app_msg_t xMsg;

    DBG_APP_INFO("USB: VBUS Timer CB\r\n");
    if (pAppCtxt->vbusChangeIntr) {
        /* Notify the VCOM task that VBus debounce is complete */
        xMsg.type = CY_USB_UVC_VBUS_CHANGE_DEBOUNCED;
        xQueueSendFromISR(pAppCtxt->uvcMsgQueue, &(xMsg), &(xHigherPriorityTaskWoken));

        /* Clear and re-enable the interrupt */
        pAppCtxt->vbusChangeIntr = false;
        Cy_GPIO_ClearInterrupt(VBUS_DETECT_GPIO_PORT, VBUS_DETECT_GPIO_PIN);
        Cy_GPIO_SetInterruptMask(VBUS_DETECT_GPIO_PORT, VBUS_DETECT_GPIO_PIN, 1);
    }
}   /* end of function  */

/**
 * \name Cy_USB_AppInit
 * \brief   This function Initializes application related data structures, register callback
 *          creates task for device function.
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD layer Context pointer
 * \param pCpuDmacBase DMAC base address
 * \param pCpuDw0Base DataWire 0 base address
 * \param pCpuDw1Base DataWire 1 base address
 * \param pHbDmaMgrCtxt HBDMA Manager Context
 * \retval None
 */
void Cy_USB_AppInit(cy_stc_usb_app_ctxt_t *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                    DMAC_Type *pCpuDmacBase, DW_Type *pCpuDw0Base, DW_Type *pCpuDw1Base,
                    cy_stc_hbdma_mgr_context_t *pHbDmaMgrCtxt)
{
    uint32_t index;
    BaseType_t status = pdFALSE;
    cy_stc_app_endp_dma_set_t *pEndpInDma;
    cy_stc_app_endp_dma_set_t *pEndpOutDma;

    pAppCtxt->devState = CY_USB_DEVICE_STATE_DISABLE;
    pAppCtxt->prevDevState = CY_USB_DEVICE_STATE_DISABLE;
    pAppCtxt->devSpeed = CY_USBD_USB_DEV_FS;
    pAppCtxt->devAddr = 0x00;
    pAppCtxt->activeCfgNum = 0x00;
    pAppCtxt->prevAltSetting = 0x00;
    pAppCtxt->pHbDmaMgrCtxt = pHbDmaMgrCtxt;

    for (index = 0x00; index < CY_USB_MAX_ENDP_NUMBER; index++)
    {
        pEndpInDma = &(pAppCtxt->endpInDma[index]);
        memset((void *)pEndpInDma, 0, sizeof(cy_stc_app_endp_dma_set_t));

        pEndpOutDma = &(pAppCtxt->endpOutDma[index]);
        memset((void *)pEndpOutDma, 0, sizeof(cy_stc_app_endp_dma_set_t));
    }

    pAppCtxt->pCpuDmacBase = pCpuDmacBase;
    pAppCtxt->pCpuDw0Base = pCpuDw0Base;
    pAppCtxt->pCpuDw1Base = pCpuDw1Base;
    pAppCtxt->pUsbdCtxt = pUsbdCtxt;

    /*
     * Callbacks registered with USBD layer. These callbacks will be called
     * based on appropriate event.
     */
    Cy_USB_AppRegisterCallback(pAppCtxt);

    if (!(pAppCtxt->firstInitDone))
    {
        pAppCtxt->vbusChangeIntr = false;
        pAppCtxt->vbusPresent    = false;
        pAppCtxt->usbConnected   = false;

        /* Create the message queue and register it with the kernel */
        pAppCtxt->uvcMsgQueue = xQueueCreate(CY_USB_UVC_DEVICE_MSG_QUEUE_SIZE,
                CY_USB_UVC_DEVICE_MSG_SIZE);
        if (pAppCtxt->uvcMsgQueue == NULL) {
            DBG_APP_ERR("UVC: Queue create failed\r\n");
            return;
        }

        vQueueAddToRegistry(pAppCtxt->uvcMsgQueue, "UVCDeviceMsgQueue");

        /* Create task and check status to confirm task created properly */
        status = xTaskCreate(Cy_UVC_DeviceTaskHandler, "UvcDeviceTask", 2048,
                             (void *)pAppCtxt, 5, &(pAppCtxt->uvcDevicetaskHandle));
        if (status != pdPASS)
        {
            DBG_APP_ERR("UVC: Task create failed \r\n");
            return;
        }

#if AUDIO_IF_EN
        Cy_UAC_AppInit(pAppCtxt);
#endif /* AUDIO_IF_EN */

        pAppCtxt->vbusDebounceTimer = xTimerCreate("VbusDebounceTimer", 200, pdFALSE,
                (void *)pAppCtxt, Cy_USB_VbusDebounceTimerCallback);
        if (pAppCtxt->vbusDebounceTimer == NULL) {
            DBG_APP_ERR("USB: Timer create failed\r\n");
            return;
        }
        DBG_APP_INFO("USB: VBus debounce timer created\r\n");

        pAppCtxt->fpsTimer = xTimerCreate("fpsTimer", 1000, pdTRUE,
                                             (void *)pAppCtxt,
                                             Cy_USB_PrintFpsCb);
        if (pAppCtxt->fpsTimer == NULL) {
            DBG_APP_ERR("UVC: FPS timer create failed\r\n");
            return;
        } else {
            /* Start the debounce timer */
            DBG_APP_INFO("UVC: FPS Timer Started\r\n");
            xTimerStart(pAppCtxt->fpsTimer, 0);
        }


#if UVC_INMEM_EN
        /* Allocate the RAM buffers used for in-memory video streaming */
        if (!Cy_UvcInMem_AllocateBuffers(pAppCtxt)) {
            return;
        }
#endif /* UVC_INMEM_EN */

        pAppCtxt->firstInitDone = 0x01;
    }

    /* Zero out the EP0 test buffer */
    memset((uint8_t *)Ep0TestBuffer, 0, sizeof(Ep0TestBuffer));

    return;
} /* end of function */

/**
 * \name Cy_USB_AppRegisterCallback
 * \brief This function will register all calback with USBD layer.
 * \param pAppCtxt application layer context pointer.
 * \retval None
 */
void Cy_USB_AppRegisterCallback(cy_stc_usb_app_ctxt_t *pAppCtxt)
{
    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt = pAppCtxt->pUsbdCtxt;

    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_RESET, Cy_USB_AppBusResetCallback);

    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_RESET_DONE, Cy_USB_AppBusResetDoneCallback);

    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_BUS_SPEED, Cy_USB_AppBusSpeedCallback);

    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_SETUP, Cy_USB_AppSetupCallback);

    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_SUSPEND, Cy_USB_AppSuspendCallback);

    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_RESUME, Cy_USB_AppResumeCallback);

    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_SET_CONFIG, Cy_USB_AppSetCfgCallback);

    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_SET_INTF, Cy_USB_AppSetIntfCallback);

    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_L1_SLEEP, Cy_USB_AppL1SleepCallback);

    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_L1_RESUME, Cy_USB_AppL1ResumeCallback);

    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_SETADDR, Cy_USB_AppSetAddressCallback);

    return;
} /* end of function */

#if AUDIO_IF_EN
/**
 * \name UAC_HbDma_Cb
 * \brief   HBDMA callback function to manage data transfers on the USB Audio Class endpoint
 * \param handle HBDMA channel handle
 * \param cy_en_hbdma_cb_type_t HBDMA channel type
 * \param pbufStat fHBDMA buffer status
 * \param userCtx user context
 * \retval None
 */
static void UAC_HbDma_Cb (cy_stc_hbdma_channel_t *handle,
                          cy_en_hbdma_cb_type_t type,
                          cy_stc_hbdma_buff_status_t *pbufStat,
                          void *userCtx)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt = (cy_stc_usb_app_ctxt_t *)userCtx;

    /* As this is an IN endpoint, only consume events are expected */
    if (type == CY_HBDMA_CB_CONS_EVENT)
    {
        /* Data has been consumed on USB side and buffer is free now. If all buffers
         * were full previously, we can queue a read operation using the newly freed
         * buffer.
         */
        if (pAppCtxt->pdmRxFreeBufCount == 0)
        {
            /* Queue read from the PDM RX FIFO into the next DMA buffer */
            Cy_PDM_QueuePDMRead(pAppCtxt, PDM_READ_SIZE);
        }

        pAppCtxt->pdmRxFreeBufCount++;
    }
}

#endif /* AUDIO_IF_EN */
/**
 * \name Cy_UVC_HandleProduceEvent
 * \brief   Function that handles a produce event indicating receipt of data through
 *          the LVDS ingress socket.
 * \param pAppCtxt application layer context pointer.
 * \param pChHandle Pointer to DMA channel structure.
 * \retval None
 */
static void
Cy_UVC_HandleProduceEvent (
        cy_stc_usb_app_ctxt_t  *pUsbApp,
        cy_stc_hbdma_channel_t *pChHandle)
{
    cy_en_hbdma_mgr_status_t   status;
    cy_stc_hbdma_buff_status_t buffStat;

    /* Wait for a free buffer */
    status = Cy_HBDma_Channel_GetBuffer(pChHandle, &buffStat);
    if (status != CY_HBDMA_MGR_SUCCESS)
    {
        DBG_APP_ERR("UVC: HB-DMA GetBuffer Error\r\n");
        return;
    }

    pUsbApp->glDmaBufCnt = buffStat.count;
    pUsbApp->glFrameSizeTransferred += pUsbApp->glDmaBufCnt;

    /* Add headers on every frame. Need to check if the EOF bit has to be set */
    if (glUvcBufferCounter <= glUvcFullBufferCount)
    {
#if UVC_HEADER_BY_FX2G3
        /* Not the end of frame */
        Cy_UVC_AddHeader(buffStat.pBuffer - CY_USB_UVC_MAX_HEADER, CY_USB_UVC_HEADER_FRAME);
        glUvcCommitLength = buffStat.count + CY_USB_UVC_MAX_HEADER;
#else
        glUvcCommitLength = buffStat.count ;
#endif /* UVC_HEADER_BY_FX2G3 */
        glUvcBufferCounter++;
    }
    else
    {

#if UVC_HEADER_BY_FX2G3
        /* Short packet: End of frame */
        Cy_UVC_AddHeader(buffStat.pBuffer - CY_USB_UVC_MAX_HEADER, CY_USB_UVC_HEADER_EOF);
        glUvcCommitLength = buffStat.count + CY_USB_UVC_MAX_HEADER;
#else
        glUvcCommitLength = buffStat.count ;
#endif /*  UVC_HEADER_BY_FX2G3 */
        glUvcBufferCounter = 1;
        pUsbApp->glFrameSize = pUsbApp->glFrameSizeTransferred;
        pUsbApp->glFrameSizeTransferred = 0;
        pUsbApp->glPartialBufSize = buffStat.count;
        pUsbApp->glConsCount = pUsbApp->glCons;
        pUsbApp->glProdCount = pUsbApp->glProd;
        pUsbApp->glCons = 0;
        pUsbApp->glProd = 0;
        pUsbApp->glfps++;
    }

#if UVC_HEADER_BY_FX2G3
    buffStat.count   = glUvcCommitLength;
    buffStat.pBuffer = buffStat.pBuffer - CY_USB_UVC_MAX_HEADER;
#endif /* UVC_HEADER_BY_FX2G3 */

    /* Commit buffer  */
    status = Cy_HBDma_Channel_CommitBuffer(pChHandle, &buffStat);
    if (status != CY_HBDMA_MGR_SUCCESS)
    {
        DBG_APP_ERR("UVC: HB-DMA Commit Buffer Error: %x\r\n",status);
        return;
    }

}

/**
 * \name HbDma_Cb
 * \brief   HBDMA callback function to add the UVC header to the frame/buffer coming
 *          from LVDS and commits to USB.
 * \param handle HBDMA channel handle
 * \param cy_en_hbdma_cb_type_t HBDMA channel type
 * \param pbufStat fHBDMA buffer status
 * \param userCtx user context
 * \retval None
 */
void HbDma_Cb(
        cy_stc_hbdma_channel_t *handle,
        cy_en_hbdma_cb_type_t type,
        cy_stc_hbdma_buff_status_t *pbufStat,
        void *userCtx)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt = (cy_stc_usb_app_ctxt_t *)userCtx;

    if (type == CY_HBDMA_CB_PROD_EVENT)
    {
        /* Video streamer application enable */
        if ((glUvcIsApplnActive == true) && (glUvcDevConfigured == true))
        {
            Cy_UVC_HandleProduceEvent(pAppCtxt, handle);
        }
    }
}

/**
 * \name Cy_USB_AppSetupEndpDmaParamsHs
 * \brief Configure and enable HBW DMA channels.
 * \param pAppCtxt application layer context pointer.
 * \param pEndpDscr Endpoint descriptor pointer
 * \retval None
 */
void Cy_USB_AppSetupEndpDmaParamsHs(cy_stc_usb_app_ctxt_t *pUsbApp, uint8_t *pEndpDscr)
{
    cy_stc_hbdma_chn_config_t dmaConfig;
    cy_en_hbdma_mgr_status_t mgrStat;
    uint32_t endpNumber, dir;
    uint16_t maxPktSize;

    Cy_USBD_GetEndpNumMaxPktDir(pEndpDscr, &endpNumber, &maxPktSize, &dir);

    DBG_APP_INFO("USB: DMA Params - endpNum:0x%x maxPktSize:0x%x "
                 "dir:0x%x\r\n",
                 endpNumber, maxPktSize,
                 dir);

    /* Handling UVC streaming endpoint */
    if ((endpNumber == UVC_STREAM_ENDPOINT) && (dir != 0)) {
        pUsbApp->uvcInEpNum       = (uint8_t)endpNumber;

#if UVC_INMEM_EN
        LOG_COLOR("Changing DMA configuration for In-Memory streaming\r\n");
        dmaConfig.chType       = CY_HBDMA_TYPE_MEM_TO_IP;
        dmaConfig.size         = UVC_INMEM_LASTBUF_SIZE;
        dmaConfig.count        = UVC_INMEM_BUF_COUNT;
        dmaConfig.prodHdrSize  = 0;
        dmaConfig.prodBufSize  = UVC_INMEM_LASTBUF_SIZE;
        dmaConfig.prodSckCount = 1;
        dmaConfig.prodSck[0]   = CY_HBDMA_VIRT_SOCKET_WR;
        dmaConfig.prodSck[1]   = (cy_hbdma_socket_id_t)0;
        dmaConfig.consSckCount = 1;
        dmaConfig.consSck[0]    = (cy_hbdma_socket_id_t)(CY_HBDMA_USBHS_IN_EP_00 + endpNumber);
        dmaConfig.usbMaxPktSize = maxPktSize;
        dmaConfig.consSck[1]   = (cy_hbdma_socket_id_t)0;
        dmaConfig.eventEnable  = 0;
        dmaConfig.intrEnable   = LVDSSS_LVDS_ADAPTER_DMA_SCK_INTR_PRODUCE_EVENT_Msk |
            LVDSSS_LVDS_ADAPTER_DMA_SCK_INTR_CONSUME_EVENT_Msk;
        dmaConfig.cb           = Cy_UvcInMem_DmaCallback;
        dmaConfig.userCtx      = (void *)(pUsbApp);
#else

        dmaConfig.consSckCount = 1;                         /* Only one consumer socket per channel */
        dmaConfig.consSck[1] = (cy_hbdma_socket_id_t)0;

#if UVC_HEADER_BY_FX2G3
        dmaConfig.prodHdrSize  = CY_USB_UVC_MAX_HEADER;
#elif UVC_HEADER_BY_FPGA
        dmaConfig.prodHdrSize  = 0;
#endif /* UVC_HEADER_BY_FX2G3 */
        dmaConfig.eventEnable  = 0;          /* Manual channel: Disable event signalling between sockets */
        dmaConfig.intrEnable   = LVDSSS_LVDS_ADAPTER_DMA_SCK_INTR_PRODUCE_EVENT_Msk |
                                 LVDSSS_LVDS_ADAPTER_DMA_SCK_INTR_CONSUME_EVENT_Msk;
        dmaConfig.cb           = HbDma_Cb;   /* HB-DMA callback */
        dmaConfig.prodBufSize  = FPGA_DMA_BUFFER_SIZE;        /* Receive depends on UVC header adds by FX2G3 or FPGA*/
        dmaConfig.size         = DMA_BUFFER_SIZE;             /* DMA Buffer size in bytes */
        dmaConfig.count        = CY_USB_UVC_STREAM_BUF_COUNT; /* DMA Buffer Count */
        dmaConfig.bufferMode   = true;                        /* DMA buffer mode enabled */
        dmaConfig.userCtx      = (void *)(pUsbApp);           /* Pass the application context as user context */


#if INTERLEAVE_EN
        dmaConfig.prodSckCount = 2; /* No. of producer sockets */
        dmaConfig.prodSck[0] = CY_HBDMA_LVDS_SOCKET_00;
        dmaConfig.prodSck[1] = CY_HBDMA_LVDS_SOCKET_01;
#else
        dmaConfig.prodSckCount = 1; /* No. of producer sockets */
        dmaConfig.prodSck[0] = CY_HBDMA_LVDS_SOCKET_00;
        dmaConfig.prodSck[1] = (cy_hbdma_socket_id_t)0; /* Producer Socket ID: None */
#endif /* INTERLEAVE_EN */

        dmaConfig.chType        = CY_HBDMA_TYPE_IP_TO_IP;
        dmaConfig.consSck[0]    = (cy_hbdma_socket_id_t)(CY_HBDMA_USBHS_IN_EP_00 + endpNumber);
        dmaConfig.usbMaxPktSize = maxPktSize;
#endif /* UVC_INMEM_EN */

        if (pUsbApp->hbBulkInChannel != NULL)
        {
            DBG_APP_ERR("UVC: Streaming DMA channel already created\r\n");
            return;
        }

        mgrStat = Cy_HBDma_Channel_Create(pUsbApp->pUsbdCtxt->pHBDmaMgr,
                &(pUsbApp->endpInDma[endpNumber].hbDmaChannel),
                &dmaConfig);
        if (mgrStat != CY_HBDMA_MGR_SUCCESS)
        {
            DBG_APP_ERR("UVC: BulkIn channel create failed 0x%x\r\n", mgrStat);
            return;
        }
        else
        {
            DBG_APP_INFO("UVC: BulkIn channel created- Buffer Size %d , UVC Header size %d\r\n", dmaConfig.size,dmaConfig.prodHdrSize);
        }

        /* Store the DMA channel pointer */
        pUsbApp->hbBulkInChannel = &(pUsbApp->endpInDma[endpNumber].hbDmaChannel);
    }

#if AUDIO_IF_EN
    /* Handle UAC streaming endpoint */
    if ((endpNumber == UAC_IN_ENDPOINT) && (dir != 0)) {

         if (pUsbApp->pPDMToUsbChn != NULL) {
            DBG_APP_ERR("UAC: ISOC-IN channel already created\r\n");
            return;
        }

        dmaConfig.size         = 384;                       /* DMA Buffer size in bytes */
        dmaConfig.count        = PDM_APP_BUFFER_CNT;        /* DMA Buffer Count */
        dmaConfig.prodHdrSize  = 0;                         /* No header being added */
        dmaConfig.prodBufSize  = 384;
        dmaConfig.eventEnable  = 0;
        dmaConfig.intrEnable   = 0x3;
        dmaConfig.bufferMode   = false;                     /* DMA buffer mode disabled */
        dmaConfig.chType       = CY_HBDMA_TYPE_MEM_TO_IP;   /* DMA Channel type: from MEM to USB EG IP */
        dmaConfig.prodSckCount = 1;                         /* No. of producer sockets */
        dmaConfig.prodSck[0]   = CY_HBDMA_VIRT_SOCKET_WR;
        dmaConfig.prodSck[1]   = (cy_hbdma_socket_id_t)0;   /* Producer Socket ID: None */
        dmaConfig.consSckCount = 1;                         /* No. of consumer Sockets */
        dmaConfig.consSck[1]   = (cy_hbdma_socket_id_t)0;   /* Consumer Socket ID: None */
        dmaConfig.cb           = UAC_HbDma_Cb;              /* HB-DMA callback */
        dmaConfig.userCtx      = (void *)(pUsbApp);         /* Pass the application context as user context */
        dmaConfig.consSck[0]    = (cy_hbdma_socket_id_t)(CY_HBDMA_USBHS_IN_EP_00 + endpNumber);
        dmaConfig.usbMaxPktSize = maxPktSize;


        mgrStat = Cy_HBDma_Channel_Create(pUsbApp->pUsbdCtxt->pHBDmaMgr,
                &(pUsbApp->endpInDma[endpNumber].hbDmaChannel),
                &dmaConfig);
        if (mgrStat != CY_HBDMA_MGR_SUCCESS) {
            DBG_APP_ERR("UAC: ISOCIN channel create failed 0x%x\r\n", mgrStat);
            return;
        }

        /* Store the pointer to the DMA channel */
        pUsbApp->pPDMToUsbChn = &(pUsbApp->endpInDma[endpNumber].hbDmaChannel);

        /* Get the list of DMA buffers to be used for PDM to USB transfers */
        mgrStat = Cy_HBDma_Channel_GetBufferInfo(pUsbApp->pPDMToUsbChn, pUsbApp->pPDMRxBuffer, PDM_APP_BUFFER_CNT);
        if (mgrStat != CY_HBDMA_MGR_SUCCESS)
        {
            DBG_APP_ERR("UAC: Get Buffer info failed 0x%x\r\n", mgrStat);
            return;
        }

        pUsbApp->pdmRxBufIndex      = 0;
        pUsbApp->pdmRxFreeBufCount  = PDM_APP_BUFFER_CNT;

        /* Enable the channel for data transfer */
        mgrStat = Cy_HBDma_Channel_Enable(pUsbApp->pPDMToUsbChn, 0);
        DBG_APP_INFO("UAC: PDM to USB channel enable status: %x\r\n", mgrStat);

    }
#endif /* AUDIO_IF_EN */

} /* end of function  */


/**
 * \name Cy_USB_AppConfigureEndp
 * \brief Configure all endpoints used by application (except EP0)
 * \param pUsbdCtxt USBD layer context pointer
 * \param pEndpDscr Endpoint descriptor pointer
 * \retval None
 */
void Cy_USB_AppConfigureEndp(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, uint8_t *pEndpDscr)
{
    cy_stc_usb_endp_config_t endpConfig;
    cy_en_usb_endp_dir_t endpDirection;
    bool valid;
    uint32_t endpType;
    uint32_t endpNumber, dir;
    uint16_t maxPktSize;
    uint32_t isoPkts = 0x00;
    uint8_t burstSize = 0x00;
    uint8_t maxStream = 0x00;
    uint8_t interval = 0x00;
    cy_en_usbd_ret_code_t usbdRetCode;

    /* If it is not endpoint descriptor then return */
    if (!Cy_USBD_EndpDscrValid(pEndpDscr))
    {
        return;
    }

    Cy_USBD_GetEndpNumMaxPktDir(pEndpDscr, &endpNumber, &maxPktSize, &dir);

    if (dir)
    {
        endpDirection = CY_USB_ENDP_DIR_IN;
    }
    else
    {
        endpDirection = CY_USB_ENDP_DIR_OUT;
    }
    Cy_USBD_GetEndpType(pEndpDscr, &endpType);

    if ((CY_USB_ENDP_TYPE_ISO == endpType) || (CY_USB_ENDP_TYPE_INTR == endpType))
    {
        /* The ISOINPKS setting in the USBHS register is the actual packets per microframe value */
        isoPkts = ((*((uint8_t *)(pEndpDscr + CY_USB_ENDP_DSCR_OFFSET_MAX_PKT + 1)) & CY_USB_ENDP_ADDL_XN_MASK) >> CY_USB_ENDP_ADDL_XN_POS) + 1;
    }

    valid = 0x01;
    Cy_USBD_GetEndpInterval(pEndpDscr, &interval);

    /* Prepare endpointConfig parameter */
    endpConfig.endpType = (cy_en_usb_endp_type_t)endpType;
    endpConfig.endpDirection = endpDirection;
    endpConfig.valid = valid;
    endpConfig.endpNumber = endpNumber;
    endpConfig.maxPktSize = (uint32_t)maxPktSize;
    endpConfig.isoPkts = isoPkts;
    endpConfig.burstSize = burstSize;
    endpConfig.streamID = maxStream;
    endpConfig.allowNakTillDmaRdy = true;
    endpConfig.interval = interval;
    usbdRetCode = Cy_USB_USBD_EndpConfig(pUsbdCtxt, endpConfig);

    /* Print status of the endpoint configuration to help debug */
    DBG_APP_INFO("USB: EpNum: %d, status: %x\r\n", endpNumber, usbdRetCode);
    return;
} /* end of function */

/**
 * \name Cy_USB_AppSetCfgCallback
 * \brief Callback function will be invoked by USBD when set configuration is received
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD layer context pointer.
 * \param pMsg USB Message
 * \retval None
 */
void Cy_USB_AppSetCfgCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                              cy_stc_usb_cal_msg_t *pMsg)
{

    glUvcDevConfigured = true;
    cy_stc_usb_app_ctxt_t *pUsbApp;
    uint8_t *pActiveCfg, *pIntfDscr, *pEndpDscr;
    uint8_t index, numOfIntf, numOfEndp;


    DBG_APP_INFO("USB:Set Configuration CB\r\n");

    pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;
    pUsbApp->devSpeed = Cy_USBD_GetDeviceSpeed(pUsbdCtxt);


    /* Disable optional Low Power Mode transitions */
    Cy_USBD_LpmDisable(pUsbdCtxt);

    /*
     * Based on type of application as well as how data flows,
     * data wire can be used so initialize datawire.
     */
    Cy_DMA_Enable(pUsbApp->pCpuDw0Base);
    Cy_DMA_Enable(pUsbApp->pCpuDw1Base);

    pActiveCfg = Cy_USB_USBD_GetActiveCfgDscr(pUsbdCtxt);
    if (!pActiveCfg)
    {
        /* Set config should be called when active config value > 0x00 */
        return;
    }
    numOfIntf = Cy_USBD_FindNumOfIntf(pActiveCfg);
    if (numOfIntf == 0x00)
    {
        return;
    }

    for (index = 0x00; index < numOfIntf; index++)
    {
        /* During Set Config command always altSetting 0 will be active */
        pIntfDscr = Cy_USBD_GetIntfDscr(pUsbdCtxt, index, 0x00);
        if (pIntfDscr == NULL)
        {
            DBG_APP_INFO("USB: Get Intf failed \r\n");
            return;
        }

        numOfEndp = Cy_USBD_FindNumOfEndp(pIntfDscr);
        if (numOfEndp == 0x00)
        {
            DBG_APP_INFO("USB: No. of ep: 0\r\n");
            continue;
        }

        pEndpDscr = Cy_USBD_GetEndpDscr(pUsbdCtxt, pIntfDscr);
        while (numOfEndp != 0x00)
        {
            Cy_USB_AppConfigureEndp(pUsbdCtxt, pEndpDscr);
            Cy_USB_AppSetupEndpDmaParamsHs(pAppCtxt, pEndpDscr);
            numOfEndp--;
            pEndpDscr = (pEndpDscr + (*(pEndpDscr + CY_USB_DSCR_OFFSET_LEN)));

        }
    }

    /* Enable the interrupt for the DataWire channel used for video streaming to USBHS BULK-IN endpoint */
    Cy_USB_AppInitDmaIntr(pUsbApp->uvcInEpNum, CY_USB_ENDP_DIR_IN, CY_UVC_DataWire_ISR);

    pUsbApp->prevDevState = CY_USB_DEVICE_STATE_CONFIGURED;
    pUsbApp->devState = CY_USB_DEVICE_STATE_CONFIGURED;

#if UVC_INMEM_EN
    /* Update the DMA channel and make the descriptors point to the buffers with pre-filled colorbar data */
    Cy_UvcInMem_PrepareBuffers(pUsbApp, pUsbApp->hbBulkInChannel);
#endif /* UVC_INMEM_EN */

    return;
} /* end of function */

/**
 * \name Cy_USB_AppBusResetCallback
 * \brief Callback function will be invoked by USBD when bus detects RESET
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD layer context pointer
 * \param pMsg USB Message
 * \retval None
 */
void Cy_USB_AppBusResetCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pUsbApp;

    pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;

    DBG_APP_INFO("USB: Bus Reset CB\r\n");

    /* Stop and destroy the high bandwidth DMA channel if present. To be done before AppInit is called */
    if (pUsbApp->hbBulkInChannel != NULL)
    {
        DBG_APP_TRACE("Destroying HBDMA channels \r\n");
        Cy_HBDma_Channel_Disable(pUsbApp->hbBulkInChannel);
        Cy_HBDma_Channel_Destroy(pUsbApp->hbBulkInChannel);
        pUsbApp->hbBulkInChannel = NULL;
    }

    /*
     * USBD layer takes care of reseting its own data structure as well as
     * takes care of calling CAL reset APIs. Application needs to take care
     * of reseting its own data structure as well as "device function".
     */
    Cy_USB_AppInit(pUsbApp, pUsbdCtxt, pUsbApp->pCpuDmacBase, pUsbApp->pCpuDw0Base, pUsbApp->pCpuDw1Base, pUsbApp->pHbDmaMgrCtxt);
    pUsbApp->devState = CY_USB_DEVICE_STATE_RESET;
    pUsbApp->prevDevState = CY_USB_DEVICE_STATE_RESET;
    glUvcDevConfigured = false;
    glUvcIsApplnActive = false;
    return;
} /* end of function */

/**
 * \name Cy_USB_AppBusResetDoneCallback
 * \brief Callback function will be invoked by USBD when RESET is completed
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD layer context pointer
 * \param pMsg USB Message
 * \retval None
 */
void Cy_USB_AppBusResetDoneCallback(void *pAppCtxt,
                                    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                    cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pUsbApp;

    DBG_APP_INFO("USB: Bus Reset Done CB \r\n");

    pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;
    pUsbApp->devState = CY_USB_DEVICE_STATE_DEFAULT;
    pUsbApp->prevDevState = pUsbApp->devState;
    return;
} /* end of function */

/**
 * \name Cy_USB_AppBusSpeedCallback
 * \brief   Callback function will be invoked by USBD when speed is identified or
 *          speed change is detected
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD context
 * \param pMsg USB Message
 * \retval None
 */
void Cy_USB_AppBusSpeedCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pUsbApp;

    pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;
    pUsbApp->devState = CY_USB_DEVICE_STATE_DEFAULT;
    pUsbApp->devSpeed = Cy_USBD_GetDeviceSpeed(pUsbdCtxt);
    Cy_USBD_SetDscr(pUsbApp->pUsbdCtxt, CY_USB_SET_HS_DEVICE_DSCR, 0, (uint8_t *)CyFxUSB20DeviceDscr);
    return;
} /* end of function */

/**
 * \name Cy_USB_AppSetupCallback
 * \brief Callback function will be invoked by USBD when SETUP packet is received
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD context
 * \param pMsg USB Message
 * \retval None
 */
void Cy_USB_AppSetupCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                             cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pUsbApp;
    pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;
    uint8_t bRequest, bReqType;
    uint8_t bType, bTarget, temp;
    uint16_t wValue, wIndex, wLength;
    bool isReqHandled = false;
    cy_stc_usbd_app_msg_t xMsg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    cy_en_usb_endp_dir_t epDir = CY_USB_ENDP_DIR_INVALID;
    BaseType_t status = 0;

    DBG_APP_TRACE("USB: Setup CB\r\n");

    /* Fast enumeration is used. Only requests addressed to the interface, class,
     * vendor and unknown control requests are received by this function */

    /* Decode the fields from the setup request */
    bReqType = pUsbdCtxt->setupReq.bmRequest;
    bType = ((bReqType & CY_USB_CTRL_REQ_TYPE_MASK) >> CY_USB_CTRL_REQ_TYPE_POS);
    bTarget = (bReqType & CY_USB_CTRL_REQ_RECIPENT_OTHERS);
    bRequest = pUsbdCtxt->setupReq.bRequest;
    wValue = pUsbdCtxt->setupReq.wValue;
    wIndex = pUsbdCtxt->setupReq.wIndex;
    wLength = pUsbdCtxt->setupReq.wLength;

    if (bType == CY_USB_CTRL_REQ_STD)
    {
        DBG_APP_INFO("USB: Standard request\r\n");
        if (bRequest == CY_USB_SC_SET_FEATURE)
        {
            DBG_APP_INFO("USB: Set Feature request\r\n");
            if ((bTarget == CY_USB_CTRL_REQ_RECIPENT_INTF) && (wValue == 0))
            {
                DBG_APP_TRACE("USB: Set Feature request: Target-interface\r\n");
                Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, 0x00, CY_USB_ENDP_DIR_IN, TRUE);

                isReqHandled = true;
            }

            /* SET-FEATURE(EP-HALT) is only supported to facilitate Chapter 9 compliance tests */
            if ((bTarget == CY_USB_CTRL_REQ_RECIPENT_ENDP) && (wValue == CY_USB_FEATURE_ENDP_HALT))
            {
                DBG_APP_TRACE("USB: Set Feature request : Target-endpoint\r\n");
                epDir = ((wIndex & 0x80UL) ? (CY_USB_ENDP_DIR_IN) : (CY_USB_ENDP_DIR_OUT));
                Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, ((uint32_t)wIndex & 0x7FUL),
                        epDir, true);

                Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
                isReqHandled = true;
            }

            if ((bTarget == CY_USB_CTRL_REQ_RECIPENT_DEVICE) &&
                    ((wValue == CY_USB_FEATURE_U1_ENABLE) || (wValue == CY_USB_FEATURE_U2_ENABLE))) {
                DBG_APP_TRACE("USB: Set Feature request: Target-device\r\n");
                /* Set U1/U2 enable. Just ACK the request */
                Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
                isReqHandled = true;
            }
        }

        if (bRequest == CY_USB_SC_CLEAR_FEATURE)
        {
            DBG_APP_INFO("USB: Clear Feature request\r\n");
            if ((bTarget == CY_USB_CTRL_REQ_RECIPENT_INTF) && (wValue == 0))
            {
                DBG_APP_TRACE("USB: Clear Feature request: Target-interface\r\n");

                Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, 0x00, CY_USB_ENDP_DIR_IN, TRUE);

                isReqHandled = true;
            }

            if ((bTarget == CY_USB_CTRL_REQ_RECIPENT_ENDP) && (wValue == CY_USB_FEATURE_ENDP_HALT))
            {
                DBG_APP_TRACE("USB: Clear Feature request: Target-endpoint\r\n");
                epDir = ((wIndex & 0x80UL) ? (CY_USB_ENDP_DIR_IN) : (CY_USB_ENDP_DIR_OUT));
                if ((epDir == CY_USB_ENDP_DIR_OUT) || ((wIndex & 0x7F) != pUsbApp->uvcInEpNum))
                {
                    /* For any EP other than the UVC streaming endpoint, just clear the STALL bit */
                    Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, ((uint32_t)wIndex & 0x7FUL),
                            epDir, false);
                    Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
                }
                else
                {
                    xMsg.type = CY_USB_UVC_VIDEO_STREAM_STOP_EVENT;
                    xMsg.data[0] = wIndex;
                    status = xQueueSendFromISR(pUsbApp->uvcMsgQueue, &(xMsg), &(xHigherPriorityTaskWoken));
                    ASSERT_NON_BLOCK(pdTRUE == status,status);
                    Cy_USBD_SendACkSetupDataStatusStage(pUsbdCtxt);
                }

                isReqHandled = true;
            }

            if ((bTarget == CY_USB_CTRL_REQ_RECIPENT_DEVICE) &&
                    ((wValue == CY_USB_FEATURE_U1_ENABLE) || (wValue == CY_USB_FEATURE_U2_ENABLE))) {
                DBG_APP_TRACE("USB: Clear Feature request: Target-device\r\n");
                /* Set U1/U2 disable. Just ACK the request */
                Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
                isReqHandled = true;
            }
        }
    }

    /* Check for UVC Class Requests */
    if (bType == CY_USB_CTRL_REQ_CLASS)
    {

        DBG_APP_TRACE("USB: UVC Class Request\r\n");
        /* Handle requests addressed to the Video Control interface */
        if ((bTarget == CY_USB_CTRL_REQ_RECIPENT_INTF) && (CY_GET_LSB(wIndex) == CY_USB_UVC_INTERFACE_VC))
        {
            DBG_APP_TRACE("UVC: Control Interface Request\r\n");
            /* Respond to VC_REQUEST_ERROR_CODE_CONTROL and stall every other request as this example does not support
               any of the Video Control features */
            if ((CY_GET_MSB(wIndex) == 0x00) && (wValue == CY_USB_UVC_VC_RQT_ERROR_CODE_CONTROL))
            {
                temp = CY_USB_UVC_RQT_STAT_INVALID_CTRL;
                isReqHandled = true;
                DBG_APP_ERR("UVC: Invalid Request\r\n");
                Cy_USB_USBD_SendEndp0Data(pUsbdCtxt, &temp, 0x01);
            }
        }

        /* Handle requests addressed to the Video Streaming interface */
        if ((bTarget == CY_USB_CTRL_REQ_RECIPENT_INTF) && (CY_GET_LSB(wIndex) == CY_USB_UVC_INTERFACE_VS))
        {
            isReqHandled = true;
            DBG_APP_TRACE("UVC: Streaming Interface Request\r\n");
            switch (wValue)
            {

            case CY_USB_UVC_VS_PROBE_CONTROL:
            case CY_USB_UVC_VS_COMMIT_CONTROL:
            {
                switch (bRequest)
                {
                case CY_USB_UVC_GET_INFO_REQ:
                case CY_USB_UVC_GET_LEN_REQ:
                case CY_USB_UVC_GET_CUR_REQ:
                case CY_USB_UVC_GET_DEF_REQ:
                case CY_USB_UVC_GET_MIN_REQ:
                case CY_USB_UVC_GET_MAX_REQ:
                    /* Handling request from task */
                    xMsg.type    = CY_USB_UVC_DEVICE_GET_CUR_RQT;
                    xMsg.data[0] = bRequest;
                    xMsg.data[1] = wValue;
                    xQueueSendFromISR(pUsbApp->uvcMsgQueue, &(xMsg), &(xHigherPriorityTaskWoken));
                    isReqHandled = true;
                    break;

                case CY_USB_UVC_SET_CUR_REQ:
                    /* Handling request from task */
                    xMsg.type    = CY_USB_UVC_DEVICE_SET_CUR_RQT;
                    xMsg.data[0] = wLength;
                    xMsg.data[1] = wValue;
                    xQueueSendFromISR(pUsbApp->uvcMsgQueue, &(xMsg), &(xHigherPriorityTaskWoken));
                    isReqHandled = true;
                    break;

                default:
                    isReqHandled = false;
                    break;
                }
            }
            break;

            default:
                isReqHandled = false;
                break;
            }
        }

        /* Don't try to stall the endpoint if we have already attempted data transfer */
    }
    /*
     * If Request is not handled by the callback, Stall the command.
     */
    if (!isReqHandled)
    {
        Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, 0x00,
                                      CY_USB_ENDP_DIR_IN, TRUE);
    }
} /* end of function */

/**
 * \name Cy_USB_AppSuspendCallback
 * \brief Callback function will be invoked by USBD when Suspend signal/message is detected
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD context
 * \param pMsg USB Message
 * \retval None
 */
void Cy_USB_AppSuspendCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                               cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pUsbApp;

    pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;
    pUsbApp->prevDevState = pUsbApp->devState;
    pUsbApp->devState = CY_USB_DEVICE_STATE_SUSPEND;
} /* end of function */

/**
 * \name Cy_USB_AppResumeCallback
 * \brief Callback function will be invoked by USBD when Resume signal/message is detected
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD context
 * \param pMsg USB Message
 * \retval None
 */
void Cy_USB_AppResumeCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                              cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pUsbApp;
    cy_en_usb_device_state_t tempState;

    pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;

    tempState = pUsbApp->devState;
    pUsbApp->devState = pUsbApp->prevDevState;
    pUsbApp->prevDevState = tempState;
    return;
} /* end of function */

/**
 * \name Cy_USB_AppSetIntfCallback
 * \brief Callback function will be invoked by USBD when SET_INTERFACE is  received
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD context
 * \param pMsg USB Message
 * \retval None
 */
void Cy_USB_AppSetIntfCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                               cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_setup_req_t *pSetupReq;
    uint8_t intfNum, altSetting;

    DBG_APP_INFO("USB: Set Interface CB\r\n");
    pSetupReq  = &(pUsbdCtxt->setupReq);
    intfNum    = pSetupReq->wIndex;
    altSetting = pSetupReq->wValue;

#if AUDIO_IF_EN
    if (intfNum == UAC_STREAM_INTF_NUM) {
        Cy_UAC_SetIntfHandler((cy_stc_usb_app_ctxt_t *)pAppCtxt, altSetting);
        return;
    }
#endif /* AUDIO_IF_EN */

    /* Interface does not support multiple alternate settings: Stall the SET_INTERFACE request */
    DBG_APP_INFO("USB: SetIntf(%d) on interface %d: Alt Setting\r\n", intfNum, altSetting);
    Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, 0x00, CY_USB_ENDP_DIR_IN, true);

} /* end of function */

/**
 * \name Cy_USB_AppL1SleepCallback
 * \brief This Function will be called by USBD layer when L1 Sleep message comes.
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD context
 * \param pMsg USB Message
 * \retval None
 */
void Cy_USB_AppL1SleepCallback(void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                               cy_stc_usb_cal_msg_t *pMsg)
{
    DBG_APP_INFO("USB: L1 Sleep CB\r\n");
    return;
} /* end of function */

/**
 * \name Cy_USB_AppL1ResumeCallback
 * \brief This Function will be called by USBD layer when L1 Resume message comes.
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD context
 * \param pMsg USB Message
 * \retval None
 */
void Cy_USB_AppL1ResumeCallback(void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                cy_stc_usb_cal_msg_t *pMsg)
{
    DBG_APP_INFO("USB: L1 Resume CB\r\n");
    return;
} /* end of function */

/**
 * \name Cy_USB_AppSetFeatureCallback
 * \brief This Function will be called by USBD layer when set feature message comes.
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD context
 * \param pMsg USB Message
 * \retval None
 */
void Cy_USB_AppSetFeatureCallback(void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                  cy_stc_usb_cal_msg_t *pMsg)
{
    DBG_APP_INFO("USB Set Feature CB\r\n");
    return;
} /* end of function */

/**
 * \name Cy_USB_AppClearFeatureCallback
 * \brief This Function will be called by USBD layer when clear feature message comes.
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD context
 * \param pMsg USB Message
 * \retval None
 */
void Cy_USB_AppClearFeatureCallback(void *pUsbApp,
                                    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                    cy_stc_usb_cal_msg_t *pMsg)
{
    DBG_APP_INFO("USB: Clear Feature CB\r\n");
    return;
} /* end of function */

/**
 * \name Cy_USB_AppSetAddressCallback
 * \brief This Function will be called by USBD layer when a USB address has been assigned to the device.
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD layer context pointer.
 * \param pMsg USB Message
 * \retval None
 */
void
Cy_USB_AppSetAddressCallback (void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                       cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt = (cy_stc_usb_app_ctxt_t *)pUsbApp;

    /* Update the state variables */
    pAppCtxt->devState     = CY_USB_DEVICE_STATE_ADDRESS;
    pAppCtxt->prevDevState = CY_USB_DEVICE_STATE_DEFAULT;
    pAppCtxt->devAddr      = pUsbdCtxt->devAddr;
    pAppCtxt->devSpeed     = Cy_USBD_GetDeviceSpeed(pUsbdCtxt);

    /* Check the type of USB connection and register appropriate descriptors */
    Cy_USB_RegisterUsbDescriptors(pAppCtxt, pAppCtxt->devSpeed);
}

/**
 * \name Cy_USB_AppInitDmaIntr
 * \brief Function to register an ISR for the DMA channel associated with an endpoint
 * \param endpNumber USB endpoint number
 * \param endpDirection Endpoint direction
 * \param userIsr ISR function pointer. Can be NULL if interrupt is to be disabled.
 * \retval None
 */
void Cy_USB_AppInitDmaIntr(uint32_t endpNumber, cy_en_usb_endp_dir_t endpDirection,
                           cy_israddress userIsr)
{
    cy_stc_sysint_t intrCfg;
    if ((endpNumber > 0) && (endpNumber < CY_USB_MAX_ENDP_NUMBER))
    {
#if (!CY_CPU_CORTEX_M4)

        if (endpDirection == CY_USB_ENDP_DIR_IN)
        {
            intrCfg.intrPriority = 3;
            intrCfg.intrSrc = NvicMux1_IRQn;
            /* DW1 channels 0 onwards are used for IN endpoints */
            intrCfg.cm0pSrc = (cy_en_intr_t)(cpuss_interrupts_dw1_0_IRQn + endpNumber);
        }
        else
        {
             intrCfg.intrPriority = 3;
             intrCfg.intrSrc = NvicMux6_IRQn;
            /* DW0 channels 0 onwards are used for OUT endpoints */
            intrCfg.cm0pSrc = (cy_en_intr_t)(cpuss_interrupts_dw0_0_IRQn + endpNumber);
        }
#else
        intrCfg.intrPriority = 5;
        if (endpDirection == CY_USB_ENDP_DIR_IN)
        {
            /* DW1 channels 0 onwards are used for IN endpoints */
            intrCfg.intrSrc =
                (IRQn_Type)(cpuss_interrupts_dw1_0_IRQn + endpNumber);
        }
        else
        {
            /* DW0 channels 0 onwards are used for OUT endpoints */
            intrCfg.intrSrc =
                (IRQn_Type)(cpuss_interrupts_dw0_0_IRQn + endpNumber);
        }
#endif /* (!CY_CPU_CORTEX_M4) */

        if (userIsr != NULL)
        {
            /* If an ISR is provided, register it and enable the interrupt */
            Cy_SysInt_Init(&intrCfg, userIsr);
            NVIC_EnableIRQ(intrCfg.intrSrc);
        }
        else
        {
            /* ISR is NULL. Disable the interrupt */
            NVIC_DisableIRQ(intrCfg.intrSrc);
        }
    }
} /* end of function */

/**
 * \name Cy_USB_AppClearDmaInterrupt
 * \brief Clear DMA Interrupt
 * \param pAppCtxt application layer context pointer.
 * \param endpNumber Endpoint number
 * \param endpDirection Endpoint direction
 * \retval None
 */
void Cy_USB_AppClearDmaInterrupt(cy_stc_usb_app_ctxt_t *pAppCtxt,
                                 uint32_t endpNumber, cy_en_usb_endp_dir_t endpDirection)
{
    if ((pAppCtxt != NULL) && (endpNumber > 0) &&
            (endpNumber < CY_USB_MAX_ENDP_NUMBER)) {
        if (endpDirection == CY_USB_ENDP_DIR_IN) {
            Cy_USBHS_App_ClearDmaInterrupt(&(pAppCtxt->endpInDma[endpNumber]));
        } else  {
            Cy_USBHS_App_ClearDmaInterrupt(&(pAppCtxt->endpOutDma[endpNumber]));
        }
    }
} /* end of function */

/**
 * \name Cy_CheckStatus
 * \brief Function that handles prints error log
 * \param function Pointer to function
 * \param line Line number where error is seen
 * \param condition condition of failure
 * \param value error code
 * \param isBlocking blocking function
 * \retval None
 */
void Cy_CheckStatus(const char *function, uint32_t line, uint8_t condition, uint32_t value, uint8_t isBlocking)
{
    if (!condition)
    {
        /* Application failed with the error code status */
        Cy_Debug_AddToLog(1, RED);
        Cy_Debug_AddToLog(1, "Function %s failed at line %d with status = 0x%x\r\n", function, line, value);
        Cy_Debug_AddToLog(1, COLOR_RESET);
        if (isBlocking)
        {
            /* Loop indefinitely */
            for (;;)
            {
            }
        }
    }
}

/**
 * \name Cy_CheckStatusHandleFailure
 * \brief Function that handles prints error log
 * \param function Pointer to function
 * \param line LineNumber where error is seen
 * \param condition Line number where error is seen
 * \param value error code
 * \param isBlocking blocking function
 * \param failureHandler failure handler function
 * \retval None
 */
void Cy_CheckStatusHandleFailure(const char *function, uint32_t line, uint8_t condition, uint32_t value, uint8_t isBlocking, void (*failureHandler)(void))
{
    if (!condition)
    {
        /* Application failed with the error code status */
        Cy_Debug_AddToLog(1, RED);
        Cy_Debug_AddToLog(1, "Function %s failed at line %d with status = 0x%x\r\n", function, line, value);
        Cy_Debug_AddToLog(1, COLOR_RESET);

        if(failureHandler != NULL)
        {
            (*failureHandler)();
        }
        if (isBlocking)
        {
            /* Loop indefinitely */
            for (;;)
            {
            }
        }
    }
}

/**
 * \name Cy_FailHandler
 * \brief Error Handler
 * \retval None
 */
void Cy_FailHandler(void)
{
    DBG_APP_ERR("Reset Done\r\n");
}

/* [] END OF FILE */
