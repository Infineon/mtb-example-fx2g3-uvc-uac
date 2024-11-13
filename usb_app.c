/***************************************************************************//**
* \file usb_app.c
* \version 1.0
*
* Implements the USB data handling part of the USB Video Class application.
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

/* Sensor configuraton status */
extern bool glIsSensorConfigured;

/* GPIF State Machine number */
uint8_t cy_u3v_smNo = 0;

/* Buffer counter for different resolution */
static uint16_t cy_uvc_commitLength = 0, cy_uvc_buffer_counter = 1;

/* Default Frame Index is  1080p  as 1st index */
static uint16_t cy_uvc_currentFrameIndex = CY_USB_UVC_HS_1080P_FRAME_INDEX;
static uint16_t cy_usb_full_buffer_no = CY_USB_FULL_BUFFER_NO_1920_1080;
extern cy_stc_hbdma_buf_mgr_t HBW_BufMgr;

bool glIsFPGARegConfigured = false;

static volatile bool cy_uvc_IsApplnActive = false;
extern bool glIsFPGAConfigured;

#if FPGA_ENABLE
/*****************************************************************************
* Function Name: Cy_UVC_SendResolution()
******************************************************************************
* Summary:
*  Read the i2c
*
* Parameters:
* 
*
* Return:
*  0 for read success, error code for unsuccess.
*****************************************************************************/
uint32_t Cy_UVC_SendResolution (uint16_t width, uint16_t height, uint8_t device_offset)
{
    cy_en_scb_i2c_status_t  status = CY_SCB_I2C_SUCCESS;

    status = Cy_I2C_Write(FPGASLAVE_ADDR,device_offset+DEVICE_IMAGE_WIDTH_MSB_ADDRESS,CY_USB_GET_MSB(width),
                                              FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    status = Cy_I2C_Write(FPGASLAVE_ADDR,device_offset+DEVICE_IMAGE_WIDTH_LSB_ADDRESS,CY_USB_GET_LSB(width),
                                              FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    status = Cy_I2C_Write(FPGASLAVE_ADDR,device_offset+DEVICE_IMAGE_HEIGHT_MSB_ADDRESS,CY_USB_GET_MSB(height),
                                              FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    status = Cy_I2C_Write(FPGASLAVE_ADDR,device_offset+DEVICE_IMAGE_HEIGHT_LSB_ADDRESS,CY_USB_GET_LSB(height),
                                              FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    status = Cy_I2C_Write(FPGASLAVE_ADDR,device_offset+FPGA_DEVICE_STREAM_MODE_ADDRESS,NO_CONVERSION,
                                              FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    status = Cy_I2C_Write(FPGASLAVE_ADDR,device_offset+DEVICE_FPS_ADDRESS,FPS_DEFAULT,
                                              FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);
    DBG_APP_INFO(" FPS :%d \n\r", FPS_DEFAULT);
    return status;
} //End of Cy_UVC_SendResolution()

/*****************************************************************************
* Function Name: Cy_UVC_DataStreamStartStop(uint8_t device_offset, uint8_t IsStreamStart)
******************************************************************************
* Summary:
*  Read the i2c
*
* Parameters:
* 
*
* Return:
*  0 for read success, error code for unsuccess.
*****************************************************************************/
cy_en_scb_i2c_status_t Cy_UVC_DataStreamStartStop(uint8_t device_offset, uint8_t IsStreamStart)
{
    cy_en_scb_i2c_status_t status = CY_SCB_I2C_SUCCESS;

    if (device_offset == DEVICE0_OFFSET)
        cy_uvc_IsApplnActive = IsStreamStart?true:false;

    if(IsStreamStart)
        status = Cy_I2C_Write(FPGASLAVE_ADDR,(device_offset+FPGA_DEVICE_STREAM_ENABLE_ADDRESS),DATA_ENABLE,
                                            FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    else

        status = Cy_I2C_Write(FPGASLAVE_ADDR,(device_offset+FPGA_DEVICE_STREAM_ENABLE_ADDRESS),DATA_DISABLE,
                                            FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);

    DBG_APP_INFO("cy_uvc_IsApplnActive = 0x%x\n\r",cy_uvc_IsApplnActive);

    if(false == status)
    {
        DBG_APP_INFO("Video App = %d device_offset 0x%x\n\r",IsStreamStart, device_offset);
    }

    return status;
}//End of Cy_UVC_DataStreamStartStop()


/*****************************************************************************
* Function Name: Cy_UVC_SetVideoResolution()
******************************************************************************
* Summary:
*  Read the i2c
*
* Parameters:
* 
*
* Return:
*  0 for read success, error code for unsuccess.
*****************************************************************************/
uint32_t Cy_UVC_SetVideoResolution (uint8_t format_index, uint8_t resolution_index, uint8_t device_offset, uint8_t devSpeed)
{
    uint32_t status = 0;
    DBG_APP_INFO("format_index %d resolution_index %d devSpeed = %d\r\n", format_index,resolution_index, devSpeed);

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
} //End of Cy_UVC_SetVideoResolution()

/*****************************************************************************
* Function Name: Cy_UVC_ConfigFpgaRegister()
******************************************************************************
* Summary:
*  Read the i2c
*
* Parameters:
* 
*
* Return:
*  0 for read success, error code for unsuccess.
*****************************************************************************/
static cy_en_scb_i2c_status_t Cy_UVC_ConfigFpgaRegister (void)
{
    cy_en_scb_i2c_status_t status = CY_SCB_I2C_SUCCESS;
    /* Defalut resolution*/
    uint16_t width = H_RES_1920;
    uint16_t height = V_RES_1080;

    /* Disable camera before configuring FPGA register */
    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE0_OFFSET+FPGA_DEVICE_STREAM_ENABLE_ADDRESS,DATA_DISABLE,
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
#endif
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);
    Cy_SysLib_DelayUs(500);


    /* Number of active device list*/
    status = Cy_I2C_Write(FPGASLAVE_ADDR,FPGA_ACTIVE_DIVICE_MASK_ADDRESS,0x01,
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);  

    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);
    Cy_SysLib_DelayUs(500);

    /* write FPGA register to Disable format converstion */
    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE0_OFFSET+FPGA_DEVICE_STREAM_MODE_ADDRESS,NO_CONVERSION,
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

#if (MIPI_SOURCE_ENABLE)
    /* Select MIPI source */
    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE0_OFFSET+DEVICE_SOURCE_TYPE_ADDRESS,MIPI_SOURCE,
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);

#else
    /* Select colorbar mode by default if DYNAMIC_VIDEOSOURCE is true*/
    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE0_OFFSET+DEVICE_SOURCE_TYPE_ADDRESS,INTERNAL_COLORBAR,
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
#endif /* (MIPI_ENABLE) */
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

#if INTERLEAVE_EN
    /*
     * In Wide link mode, actual socket number each thread is connected to is derived from SSAD[kkk] control byte sent by FPGA.
     * Actual socket number =  (3'bkkk x 2) + tt[0] on adapter tt[1], where tt is the thread number sent in STAD[tt]
     * if FPGA sets kkk as 0 for Thread 0, actual socket number connected to Thread 0 is 0
     * if FPGA sets kkk as 0 for Thread 1, actual socket number connected to Thread 1 is 1
     */
    /* Inform active threads*/
    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE0_OFFSET+DEVICE_ACTIVE_TREAD_INFO_ADDRESS,2,
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    /* inform active sockets*/
    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE0_OFFSET+DEVICE_THREAD2_SOCKET_INFO_ADDRESS,CY_LVDS_GPIF_THREAD_1,
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    /* */
    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE0_OFFSET+DEVICE_THREAD2_INFO_ADDRESS,1,
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

#else /* Single Thread*/
     /* Inform active threads*/
    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE0_OFFSET+DEVICE_ACTIVE_TREAD_INFO_ADDRESS,1,
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    /* inform active sockets*/
    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE0_OFFSET+DEVICE_THREAD2_SOCKET_INFO_ADDRESS,0,
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    /* Thread 2 information*/
    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE0_OFFSET+DEVICE_THREAD2_INFO_ADDRESS,0,
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);
#endif

    /* Thread 1 information*/
    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE0_OFFSET+DEVICE_THREAD1_INFO_ADDRESS,CY_LVDS_GPIF_THREAD_0,
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    /* Thread 1 socket information*/
    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE0_OFFSET+DEVICE_THREAD1_SOCKET_INFO_ADDRESS,0,
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    /* Clear FPGA register during power up, this will get update when firmware detects HDMI */
    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE0_OFFSET+DEVICE_SOURCE_INFO_ADDRESS,SOURCE_DISCONNECT,
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    DBG_APP_INFO("dma buffer size %d \n\r",FPGA_DMA_BUFFER_SIZE);
 
    /* Update DMA buffer size used by Firmware */
    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE0_OFFSET+DEVICE_BUFFER_SIZE_MSB_ADDRESS,CY_GET_MSB(FPGA_DMA_BUFFER_SIZE),
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE0_OFFSET+DEVICE_BUFFER_SIZE_LSB_ADDRESS,CY_GET_LSB(FPGA_DMA_BUFFER_SIZE),
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH); 
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    /* Update default resolution width size used by Firmware */
    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE0_OFFSET+DEVICE_IMAGE_WIDTH_MSB_ADDRESS,CY_GET_MSB(width),
                                              FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE0_OFFSET+DEVICE_IMAGE_WIDTH_LSB_ADDRESS,CY_GET_LSB(width),
                                              FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

 /* Update default resolution height size used by Firmware */
    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE0_OFFSET+DEVICE_IMAGE_HEIGHT_MSB_ADDRESS,CY_GET_MSB(height),
                                              FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE0_OFFSET+DEVICE_IMAGE_HEIGHT_LSB_ADDRESS,CY_GET_LSB(height),
                                              FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    /* default fps*/
    status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE0_OFFSET+DEVICE_FPS_ADDRESS,60,
                                          FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    return status;
} //End of Cy_UVC_ConfigFpgaRegister()

#endif

static cy_en_hbdma_mgr_status_t CyUVCAppStart(cy_stc_usb_app_ctxt_t *pAppCtxt,uint32_t format_index, uint32_t frame_index, uint16_t DeviceIndex)
{
    cy_en_hbdma_mgr_status_t mgrStatus;
    DBG_APP_INFO("AppStart\r\n");

    if (DEVICE0_OFFSET == DeviceIndex)
    {
        mgrStatus = Cy_HBDma_Channel_Enable(pAppCtxt->hbBulkInChannel, 0);
        ASSERT_NON_BLOCK(mgrStatus == CY_HBDMA_MGR_SUCCESS, mgrStatus);

        pAppCtxt->glfps                  = 0;
        pAppCtxt->glDmaBufCnt            = 0;
        pAppCtxt->glDmaBufCnt_prv        = 0;
        pAppCtxt->glProd                 = 0;
        pAppCtxt->glCons                 = 0;
        pAppCtxt->glProdCount            = 0;
        pAppCtxt->glConsCount            = 0;
        pAppCtxt->glFrameSizeTransferred = 0;
        pAppCtxt->glFrameSize            = 0;
        pAppCtxt->glPrintFlag            = 0;
        pAppCtxt->glFrameCount           = 0;
        pAppCtxt->glPartialBufSize       = 0;
        cy_uvc_IsApplnActive             = true;
    }
#if FPGA_ENABLE
    if (Cy_UVC_SetVideoResolution(format_index, frame_index, DeviceIndex, pAppCtxt->devSpeed) != 0)
    {
        DBG_APP_ERR("Error Sending Resolution\r\n");
    }
    if ((!cy_uvc_IsApplnActive))
    {
        Cy_LVDS_GpifSMSwitch(LVDSSS_LVDS, 0, 255, 0, 0, 12, 2 );
        DBG_APP_INFO("Cy_LVDS_GpifSMSwitch line %d\r\n", __LINE__);
    }
    Cy_UVC_DataStreamStartStop(DeviceIndex, START);
#endif
    return mgrStatus;
}

static void CyUVCAppStop(cy_stc_usb_app_ctxt_t *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, uint16_t wIndex)
{
    cy_en_hbdma_mgr_status_t status = CY_HBDMA_MGR_SUCCESS;
    uint16_t DeviceIndex;
    uint32_t epNumber = ((uint32_t)wIndex & 0x7FUL);
    DBG_APP_INFO("App Stop:windex=0x%x\r\n",wIndex);

#if FPGA_ENABLE
    if(UVC_STREAM_ENDPOINT == epNumber)
        DeviceIndex = DEVICE0_OFFSET;

    Cy_UVC_DataStreamStartStop(DeviceIndex, STOP);
#endif

    if (UVC_STREAM_ENDPOINT == epNumber)
    {
        /* Reset the DMA channel through which data is received from the LVDS side. */
        status = Cy_HBDma_Channel_Reset(pAppCtxt->hbBulkInChannel);
        ASSERT_NON_BLOCK(CY_HBDMA_MGR_SUCCESS == status,status);

        cy_uvc_IsApplnActive             = false;
        pAppCtxt->uvcPendingBufCnt       = 0;
        pAppCtxt->glfps                  = 0;
        pAppCtxt->glDmaBufCnt            = 0;
        pAppCtxt->glDmaBufCnt_prv        = 0;
        pAppCtxt->glProd                 = 0;
        pAppCtxt->glCons                 = 0;
        pAppCtxt->glProdCount            = 0;
        pAppCtxt->glConsCount            = 0;
        pAppCtxt->glFrameSizeTransferred = 0;
        pAppCtxt->glFrameSize            = 0;    
        pAppCtxt->glPrintFlag            = 0;      
        pAppCtxt->glFrameCount           = 0;      
        pAppCtxt->glPartialBufSize       = 0;
        DBG_APP_INFO("UVC DMA Reset and Variable CLeared\r\n");
    }
    /* On USB 2.0 connection, reset the DataWire channel used to send data to the EPM. */

    Cy_USBHS_App_ResetEpDma(&(pAppCtxt->endpInDma[epNumber]));


    /* Flush and reset the endpoint and clear the STALL bit. */
    Cy_USBD_FlushEndp(pUsbdCtxt, epNumber, CY_USB_ENDP_DIR_IN);
    Cy_USBD_ResetEndp(pUsbdCtxt, epNumber, CY_USB_ENDP_DIR_IN, false);
    Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, (cy_en_usb_endp_dir_t)epNumber, CY_USB_ENDP_DIR_IN, false);

}

/* USBHS: 640*480  @60 fps */
USB_DESC_ATTRIBUTES uint8_t cy_uvc_probectrl_HS_VGA[CY_USB_UVC_MAX_PROBE_SETTING];
uint8_t cy_uvc_probectrl_HS_VGA_[] = {
    0x00, 0x00,                         /* bmHint : no hit */
    0x01,                               /* Use 1st Video format index */
    CY_USB_UVC_HS_VGA_FRAME_INDEX,      /* Use 1st Video frame index */
    0x0A, 0x8B, 0x02, 0x00,             /* Desired frame interval in the unit of 100ns: 60 fps */
    0x00, 0x00,                         /* Key frame rate in key frame/video frame units*/
    0x00, 0x00,                         /* PFrame rate in PFrame / key frame units */
    0x00, 0x00,                         /* Compression quality control */
    0x00, 0x00,                         /* Window size for average bit rate */
    0x00, 0x00,                         /* Internal video streaming i/f latency in ms */
    0x00, 0x60, 0x09, 0x00,             /* Max video frame size in bytes: 640*480*2 = 0x00096000 */
    CY_USB_DWORD_GET_BYTE0(CY_USB_UVC_STREAM_BUF_SIZE),
    CY_USB_DWORD_GET_BYTE1(CY_USB_UVC_STREAM_BUF_SIZE),
    CY_USB_DWORD_GET_BYTE2(CY_USB_UVC_STREAM_BUF_SIZE),
    CY_USB_DWORD_GET_BYTE3(CY_USB_UVC_STREAM_BUF_SIZE),
    0x00, 0x60, 0xE3, 0x16,             /* Device Clock */
    0x00, 0x00, 0x00, 0x00              /* Framing and format information. */
};

/* 1920*1080 @15 fps */
USB_DESC_ATTRIBUTES uint8_t cy_uvc_probectrl_HS_1080p[CY_USB_UVC_MAX_PROBE_SETTING] __attribute__((aligned(32)));
uint8_t cy_uvc_probectrl_HS_1080p_[]= {
    0x00, 0x00,                         /* bmHint : no hit */
    0x01,                               /* Use 1st Video format index */
    CY_USB_UVC_HS_1080P_FRAME_INDEX,    /* Use 2nd Video frame index */
    0x15, 0x16, 0x05, 0x00,             /* Desired frame interval in the unit of 100ns: 30 fps */
    0x00, 0x00,                         /* Key frame rate in key frame/video frame units */
    0x00, 0x00,                         /* PFrame rate in PFrame / key frame units */
    0x00, 0x00,                         /* Compression quality control */
    0x00, 0x00,                         /* Window size for average bit rate*/
    0x00, 0x00,                         /* Internal video streaming i/f latency in ms */
    0x00, 0x48, 0x3F, 0x00,             /* Max video frame size in bytes: 1920*1080*2 = 0x0003f4800 */
    CY_USB_DWORD_GET_BYTE0(CY_USB_UVC_STREAM_BUF_SIZE),
    CY_USB_DWORD_GET_BYTE1(CY_USB_UVC_STREAM_BUF_SIZE),
    CY_USB_DWORD_GET_BYTE2(CY_USB_UVC_STREAM_BUF_SIZE),
    CY_USB_DWORD_GET_BYTE3(CY_USB_UVC_STREAM_BUF_SIZE),
    0x00, 0x60, 0xE3, 0x16,             /* Device Clock */
    0x00, 0x00, 0x00, 0x00              /* Framing and format information. */
};

/* Video Probe Commit Control */
USB_DESC_ATTRIBUTES uint8_t cy_uvc_commitctrl[CY_USB_UVC_MAX_PROBE_SETTING_ALIGNED];

/* Whether SET_CONFIG is complete or not. */
static volatile bool cy_uvc_devconfigured = false;


/* UVC Header */
USB_DESC_ATTRIBUTES uint8_t cy_uvc_header[CY_USB_UVC_MAX_HEADER];
uint8_t cy_uvc_header_[] = {
    32,                                 /* Header Length */
    0x8C,                               /* Bit field header field */
    0x00, 0x00, 0x00, 0x00,             /* Presentation time stamp field */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* Source clock reference field */
    0x00, 0x00, 0x00, 0x00, 0x00,       /* Zero padding for 32 byte align */
    0x00, 0x00, 0x00, 0x00, 0x00,       /* Zero padding for 32 byte align */
    0x00, 0x00, 0x00, 0x00, 0x00,       /* Zero padding for 32 byte align */
    0x00, 0x00, 0x00, 0x00, 0x00        /* Zero padding for 32 byte align */
};

HBDMA_BUF_ATTRIBUTES uint32_t Ep0TestBuffer[32U];

#if AUDIO_IF_EN
/*
 * Function: Cy_USB_App_FreeAllDmaBuffers()
 * Description: Clears all DMA buffer allocations made from the application context.
 * Parameter: cy_stc_usb_app_ctxt_t *pAppCtxt
 * return: None
 */
void Cy_USB_App_FreeAllDmaBuffers(cy_stc_usb_app_ctxt_t *pAppCtxt)
{
    if (pAppCtxt != NULL) {
        memset((uint8_t *)pAppCtxt->epDmaBufSpace, 0, RAM_BUF_SZ_WORDS * sizeof(uint32_t));
        pAppCtxt->dmaBufFreeIdx = 0;
    }
}

/*
 * Function: Cy_USB_App_GetDmaBuffer()
 * Description: Obtain a RAM buffer of specified size for DMA data transfers.
 * Parameter: cy_stc_usb_app_ctxt_t *pAppCtxt, uint16_t sz_bytes
 * return: Pointer to RAM buffer, NULL in case of error.
 */
uint8_t *Cy_USB_App_GetDmaBuffer(cy_stc_usb_app_ctxt_t *pAppCtxt, uint16_t sz_bytes)
{
    uint8_t *buf_p = NULL;
    uint16_t sz_words = ((sz_bytes + 3u) >> 2u);

    if (
            (pAppCtxt != NULL) &&
            (sz_bytes != 0) &&
            ((RAM_BUF_SZ_WORDS - pAppCtxt->dmaBufFreeIdx) >= sz_words)
       )
    {
        buf_p = (uint8_t *)(pAppCtxt->epDmaBufSpace + pAppCtxt->dmaBufFreeIdx);
        pAppCtxt->dmaBufFreeIdx += sz_words;
    }

    return buf_p;
}
#endif

void
CyUvcAppGpifIntr(void *pApp)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt;
    pAppCtxt = (cy_stc_usb_app_ctxt_t*)pApp;

    /* Reset the DMA channel through which data is received from the LVDS side. */
    Cy_HBDma_Channel_Reset(pAppCtxt->hbBulkInChannel);

    /* Flush and reset the endpoint and clear the STALL bit. */
    Cy_USBD_FlushEndp(pAppCtxt->pUsbdCtxt, pAppCtxt->uvcInEpNum, CY_USB_ENDP_DIR_IN);
    Cy_USBD_ResetEndp(pAppCtxt->pUsbdCtxt, pAppCtxt->uvcInEpNum, CY_USB_ENDP_DIR_IN, false);
    Cy_USB_USBD_EndpSetClearStall(pAppCtxt->pUsbdCtxt, pAppCtxt->uvcInEpNum,CY_USB_ENDP_DIR_IN, false);
    pAppCtxt->uvcPendingBufCnt = 0;
    Cy_USBD_SendAckSetupDataStatusStage(pAppCtxt->pUsbdCtxt);

    return;
}

/*
 * Function: Cy_USB_UvcAddHeader()
 * Description:  UVC header addition function
 * Parameter: Buffer pointer,Frame ID
 * return: void
 */
void Cy_USB_UvcAddHeader(
    uint8_t *buffer_p, /* Buffer pointer */
    uint8_t frameInd   /* EOF or normal frame indication */
)
{
    /* Copy header to buffer */
    memcpy(buffer_p, (uint8_t *)cy_uvc_header, CY_USB_UVC_MAX_HEADER);

    /* Check if last packet of the frame. */
    if (frameInd == CY_USB_UVC_HEADER_EOF)
    {
        /* Modify UVC header to toggle Frame ID */
        cy_uvc_header[1] ^= CY_USB_UVC_HEADER_FRAME_ID;

        /* Indicate End of Frame in the buffer */
        buffer_p[1] |= CY_USB_UVC_HEADER_EOF;
    }
} /* end of function */

/*
 * Function: Cy_USB_AppDisableEndpDma()
 * Description: This function de-inits all active USB DMA channels as part of USB disconnect process.
 * Parameter: cy_stc_usb_app_ctxt_t *
 * return: void
 */
void
Cy_USB_AppDisableEndpDma (cy_stc_usb_app_ctxt_t *pAppCtxt)
{
    uint8_t i;

    /* On USB 2.x connections, make sure the DataWire channels are disabled and reset. */
 
    for (i = 1; i < CY_USB_MAX_ENDP_NUMBER; i++) {
        if (pAppCtxt->endpInDma[i].valid) {
            /* DeInit the DMA channel and disconnect the triggers. */
            Cy_USBHS_App_DisableEpDmaSet(&(pAppCtxt->endpInDma[i]));
        }

        if (pAppCtxt->endpOutDma[i].valid) {
            /* DeInit the DMA channel and disconnect the triggers. */
            Cy_USBHS_App_DisableEpDmaSet(&(pAppCtxt->endpOutDma[i]));
        }
    }
    

    /* Disable and destroy the High BandWidth DMA channels. */
    if (pAppCtxt->hbBulkInChannel != NULL) {
        Cy_HBDma_Channel_Disable(pAppCtxt->hbBulkInChannel);
        Cy_HBDma_Channel_Destroy(pAppCtxt->hbBulkInChannel);
        pAppCtxt->hbBulkInChannel = NULL;
    }

#if AUDIO_IF_EN
    if (pAppCtxt->dmaBufFreeIdx != 0) {
        Cy_USB_App_FreeAllDmaBuffers(pAppCtxt);
    }
#endif /* AUDIO_IF_EN */
}

/*
 * Function: Cy_USB_UvcSetCurRqtHandler()
 * Description: This function handles the SET_CUR request addresses to the
 * UVC video streaming interface.
 * Parameter: pAppCtxt, wLength
 * return: void
 */
void Cy_USB_UvcSetCurRqtHandler (cy_stc_usb_app_ctxt_t *pAppCtxt, uint16_t wLength, uint16_t wValue)
{
    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt = pAppCtxt->pUsbdCtxt;
    cy_en_usbd_ret_code_t retStatus;
    uint16_t loopCnt = 250u;
    cy_stc_usbd_app_msg_t xMsg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    DBG_APP_INFO("UVC_UVC_SET_CUR_REQ\r\n");

    /* Read the data out commitctrl buffer */
    retStatus = Cy_USB_USBD_RecvEndp0Data(pUsbdCtxt, cy_uvc_commitctrl, wLength);
    if (retStatus == CY_USBD_STATUS_SUCCESS)
    {
        /* Wait until receive DMA transfer has been completed. */
        while ((!Cy_USBD_IsEp0ReceiveDone(pUsbdCtxt)) && (loopCnt--)) {
            Cy_SysLib_DelayUs(10);
        }
        if (!Cy_USBD_IsEp0ReceiveDone(pUsbdCtxt)) {
            DBG_APP_ERR("SET_CUR data timed out\r\n");
            Cy_USB_USBD_RetireRecvEndp0Data(pUsbdCtxt);
            Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, 0x00, CY_USB_ENDP_DIR_IN, TRUE);
            return;
        }

        cy_uvc_currentFrameIndex = cy_uvc_commitctrl[3];
        DBG_APP_INFO("SET_CUR cy_uvc_currentFrameIndex:%x\r\n", cy_uvc_currentFrameIndex);

    
        if (cy_uvc_currentFrameIndex == CY_USB_UVC_HS_VGA_FRAME_INDEX)
        {
            memcpy(&cy_uvc_probectrl_HS_VGA[2], &cy_uvc_commitctrl[2], CY_USB_UVC_PROBE_CONTROL_UPDATE_SIZE);
            cy_usb_full_buffer_no = CY_USB_FULL_BUFFER_NO_640_480;            
        }
        else if (cy_uvc_currentFrameIndex == CY_USB_UVC_HS_1080P_FRAME_INDEX)
        {
            memcpy(&cy_uvc_probectrl_HS_1080p[2], &cy_uvc_commitctrl[2], CY_USB_UVC_PROBE_CONTROL_UPDATE_SIZE);
            cy_usb_full_buffer_no = CY_USB_FULL_BUFFER_NO_1920_1080;            
        }        
        else
        {
            DBG_APP_ERR("Error FrameIndex\r\n");
            cy_uvc_currentFrameIndex = CY_USB_UVC_HS_VGA_FRAME_INDEX;            
            cy_usb_full_buffer_no = CY_USB_FULL_BUFFER_NO_640_480;            
        }

        if (wValue == CY_USB_UVC_VS_COMMIT_CONTROL)
        {
            DBG_APP_INFO("CY_USB_UVC_VS_COMMIT_CONTROL\r\n");
            /* Reset the Counter */
            cy_uvc_buffer_counter = 1;

            if ( CyUVCAppStart(pAppCtxt, cy_uvc_commitctrl[2],cy_uvc_commitctrl[3], DEVICE0_OFFSET) == CY_HBDMA_MGR_SUCCESS) {
                /* Notify the VCOM task that VBus debounce is complete. */
                xMsg.type = CY_USB_UVC_VIDEO_STREAMING_START;
                xQueueSendFromISR(pAppCtxt->uvcMsgQueue, &(xMsg), &(xHigherPriorityTaskWoken));
            }
        }
    }
    else
    {
        DBG_APP_ERR("Error RecvEndp0Data\r\n");
    }
}

/*
 * Function: Cy_UVC_DeviceTaskHandler()
 * Description: This function handles Streaming UVC Device.
 * Parameter: pTaskParam
 * return: void
 *
 * NOTE : The actual data forwarding from sensor to USB host is done from the DMA and GPIF callback
       functions. The thread is only responsible for checking for streaming start/stop conditions.
 */
void Cy_UVC_DeviceTaskHandler(void *pTaskParam)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt = (cy_stc_usb_app_ctxt_t *)pTaskParam;
    cy_stc_usbd_app_msg_t queueMsg;
    BaseType_t xStatus;
    uint16_t wIndex;

    /* Reset Frame Id in UVC Header */
    cy_uvc_header_[1] = CY_USB_UVC_HEADER_DEFAULT_BFH;
    DBG_APP_INFO("UvcDeviceThreadCreated\r\n");
    vTaskDelay(100);

    /*  All UVC control structures copied from flash to the HBW SRAM buffers */
    memcpy (cy_uvc_probectrl_HS_VGA, cy_uvc_probectrl_HS_VGA_, sizeof(cy_uvc_probectrl_HS_VGA_));
    memcpy (cy_uvc_probectrl_HS_1080p, cy_uvc_probectrl_HS_1080p_, sizeof(cy_uvc_probectrl_HS_1080p_));

    memcpy (cy_uvc_header, cy_uvc_header_, sizeof(cy_uvc_header_));

    /* If VBus is present, enable the USB connection. */
    pAppCtxt->vbusPresent = (Cy_GPIO_Read(VBUS_DETECT_GPIO_PORT, VBUS_DETECT_GPIO_PIN) == VBUS_DETECT_STATE);
    if (pAppCtxt->vbusPresent) {
        Cy_USB_EnableUsbHSConnection(pAppCtxt);
    }

#if FPGA_ENABLE
    Cy_FPGAConfigPins(pAppCtxt,FPGA_CONFIG_MODE);
    Cy_QSPI_Start(pAppCtxt,&HBW_BufMgr);
    Cy_SPI_FlashInit(SPI_FLASH_0, false);

    Cy_FPGAConfigure(pAppCtxt,FPGA_CONFIG_MODE);

    DBG_APP_INFO("Configure FPGA\n\r"); 

    if(!glIsFPGARegConfigured)
    {
        Cy_APP_GetFPGAVersion(pAppCtxt);
        if(0 == Cy_UVC_ConfigFpgaRegister())
        {
            glIsFPGARegConfigured = true;
            DBG_APP_INFO("Successfuly configured FPGA via I2C \n\r"); 
#if MIPI_SOURCE_ENABLE            
            Cy_UVC_ConfigureImageSensor();                
            if (glIsSensorConfigured == false) {
                /* Select INTERNAL_COLORBAR */    
            
                DBG_APP_INFO("Select INTERNAL_COLORBAR \n\r"); 
                cy_en_scb_i2c_status_t status = CY_SCB_I2C_SUCCESS;
                status = Cy_I2C_Write(FPGASLAVE_ADDR,DEVICE0_OFFSET+DEVICE_SOURCE_TYPE_ADDRESS, INTERNAL_COLORBAR,
                                                      FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
                ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);
            }    
#endif            
        }
        else
        {
            LOG_ERROR("Failed to configure FPGA via I2C \r\n");
        }
    }

#endif

    vTaskDelay(100);

    /* Initialize the LVDS interface. */
    Cy_UVC_LvdsInit(); 

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
                /* Start the debounce timer. */
                xTimerStart(pAppCtxt->vbusDebounceTimer, 0);
                break;

            case CY_USB_UVC_VBUS_CHANGE_DEBOUNCED:
                /* Check whether VBus state has changed. */
                pAppCtxt->vbusPresent = (Cy_GPIO_Read(VBUS_DETECT_GPIO_PORT, VBUS_DETECT_GPIO_PIN) == VBUS_DETECT_STATE);

                if (pAppCtxt->vbusPresent) {
                    if (!pAppCtxt->usbConnected) {
                        DBG_APP_INFO("Enabling USB connection due to VBus detect\r\n");
                        Cy_USB_EnableUsbHSConnection(pAppCtxt);
                    }
                } else {
                    if (pAppCtxt->usbConnected) {
                        Cy_USB_AppDisableEndpDma(pAppCtxt);
                        DBG_APP_INFO("Disabling USB connection due to VBus removal\r\n");
                        Cy_USB_DisableUsbHSConnection(pAppCtxt);
                    }
                }
                break;

            case CY_USB_UVC_VIDEO_STREAMING_START:
                Cy_USBD_AddEvtToLog(pAppCtxt->pUsbdCtxt, CY_USB_UVC_EVT_VSTREAM_START);
                break;

            case CY_USB_UVC_DEVICE_SET_CUR_RQT:
                Cy_USB_UvcSetCurRqtHandler(pAppCtxt, queueMsg.data[0], queueMsg.data[1]);
                Cy_USBD_AddEvtToLog(pAppCtxt->pUsbdCtxt, CY_USB_UVC_EVT_SET_CUR_REQ);
                break;
            case CY_USB_UVC_VIDEO_STREAM_STOP_EVENT:
                DBG_APP_INFO("CY_USB_UVC_VIDEO_STREAM_STOP_EVENT\r\n");
                wIndex = (uint16_t)queueMsg.data[0];
                CyUVCAppStop(pAppCtxt,pAppCtxt->pUsbdCtxt, wIndex);
                break;
            case CY_USB_PRINT_STATUS:
                DBG_APP_INFO("UVC FPS : %d\n\r",queueMsg.data[0]);
                DBG_APP_INFO("UVC Bytes Transferred : %d\n\r",pAppCtxt->glFrameSize);
            break;
            default:
                break;
        }
    }
}

/*
 * Function: Cy_USB_PrintFpsCb()
 * Description: This Function will be called when timer expires.
 *              This function print event log.
 * Parameter: xTimer
 * return: void
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

        if(cy_uvc_IsApplnActive == true)
        {
            xMsg.type = CY_USB_PRINT_STATUS;
            xMsg.data[0] = pAppCtxt->glfps;
            pAppCtxt->glfps = 0;
            xQueueSendFromISR(pAppCtxt->uvcMsgQueue, &(xMsg), &(xHigherPriorityTaskWoken));
        }
    }
}   /* end of function() */



/*
 * Function: Cy_USB_VbusDebounceTimerCallback()
 * Description: Timer used to do debounce on VBus changed interrupt notification.
 *
 * Parameter:
 *      xTimer: RTOS timer handle.
 * return: void
 */
void
Cy_USB_VbusDebounceTimerCallback (TimerHandle_t xTimer)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt = (cy_stc_usb_app_ctxt_t *)pvTimerGetTimerID(xTimer);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    cy_stc_usbd_app_msg_t xMsg;

    DBG_APP_INFO("VbusDebounce_CB\r\n");
    if (pAppCtxt->vbusChangeIntr) {
        /* Notify the VCOM task that VBus debounce is complete. */
        xMsg.type = CY_USB_UVC_VBUS_CHANGE_DEBOUNCED;
        xQueueSendFromISR(pAppCtxt->uvcMsgQueue, &(xMsg), &(xHigherPriorityTaskWoken));

        /* Clear and re-enable the interrupt. */
        pAppCtxt->vbusChangeIntr = false;
        Cy_GPIO_ClearInterrupt(VBUS_DETECT_GPIO_PORT, VBUS_DETECT_GPIO_PIN);
        Cy_GPIO_SetInterruptMask(VBUS_DETECT_GPIO_PORT, VBUS_DETECT_GPIO_PIN, 1);
    }
}   /* end of function  */

/*
 * Function: Cy_USB_AppInit()
 * Description: This function Initializes application related data structures,
 *              register callback and creates queue and task for device
 *              function.
 * Parameter: cy_stc_usb_app_ctxt_t, cy_stc_usb_usbd_ctxt_t, DMAC_Type
 *            DW_Type, DW_Type, cy_stc_hbdma_mgr_context_t*
 * return: None.
 * Note: This function should be called after USBD_Init()
 */
void Cy_USB_AppInit(cy_stc_usb_app_ctxt_t *pAppCtxt,
                    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, DMAC_Type *pCpuDmacBase,
                    DW_Type *pCpuDw0Base, DW_Type *pCpuDw1Base,
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
    pAppCtxt->enumMethod = CY_USB_ENUM_METHOD_FAST;
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
    pAppCtxt->uvcPendingBufCnt = 0;
    pAppCtxt->uvcFlowCtrlFlag = false;

   
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

        /* Create the message queue and register it with the kernel. */
        pAppCtxt->uvcMsgQueue = xQueueCreate(CY_USB_UVC_DEVICE_MSG_QUEUE_SIZE,
                CY_USB_UVC_DEVICE_MSG_SIZE);
        if (pAppCtxt->uvcMsgQueue == NULL) {
            DBG_APP_ERR("QueuecreateFail\r\n");
            return;
        }

        vQueueAddToRegistry(pAppCtxt->uvcMsgQueue, "UVCDeviceMsgQueue");

        /* Create task and check status to confirm task created properly. */
        status = xTaskCreate(Cy_UVC_DeviceTaskHandler, "UvcDeviceTask", 2048,
                             (void *)pAppCtxt, 5, &(pAppCtxt->uvcDevicetaskHandle));
        if (status != pdPASS)
        {
            DBG_APP_ERR("TaskcreateFail\r\n");
            return;
        }

#if AUDIO_IF_EN
        Cy_UAC_AppInit(pAppCtxt);
#endif /* AUDIO_IF_EN */

        pAppCtxt->vbusDebounceTimer = xTimerCreate("VbusDebounceTimer", 200, pdFALSE,
                (void *)pAppCtxt, Cy_USB_VbusDebounceTimerCallback);
        if (pAppCtxt->vbusDebounceTimer == NULL) {
            DBG_APP_ERR("TimerCreateFail\r\n");
            return;
        }
        DBG_APP_INFO("VBus debounce timer created\r\n");

        pAppCtxt->fpsTimer = xTimerCreate("fpsTimer", 1000, pdTRUE,
                                             (void *)pAppCtxt,
                                             Cy_USB_PrintFpsCb);
        if (pAppCtxt->fpsTimer == NULL) {
            DBG_APP_ERR("FPS timer Create Fail\r\n");
            return;
        } else {
            /* Start the debounce timer. */
            DBG_APP_INFO("FPS Timer Start\r\n");
            xTimerStart(pAppCtxt->fpsTimer, 0);
        }

        pAppCtxt->firstInitDone = 0x01;
    }

    /* Zero out the EP0 test buffer. */
    memset((uint8_t *)Ep0TestBuffer, 0, sizeof(Ep0TestBuffer));

    return;
} /* end of function. */


/*
 * Function: Cy_USB_AppSetAddressCallback()
 * Description: This Function will be called by USBD layer when
 *              a USB address has been assigned to the device.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t, cy_stc_usb_cal_msg_t
 * return: void
 */
void
Cy_USB_AppSetAddressCallback (void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                       cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt = (cy_stc_usb_app_ctxt_t *)pUsbApp;

    /* Update the state variables. */
    pAppCtxt->devState     = CY_USB_DEVICE_STATE_ADDRESS;
    pAppCtxt->prevDevState = CY_USB_DEVICE_STATE_DEFAULT;
    pAppCtxt->devAddr      = pUsbdCtxt->devAddr;
    pAppCtxt->devSpeed     = Cy_USBD_GetDeviceSpeed(pUsbdCtxt);

    /* Check the type of USB connection and register appropriate descriptors. */
    Cy_USB_RegisterUsbDescriptors(pAppCtxt, pAppCtxt->devSpeed);
}


/*
 * Function: Cy_USB_AppRegisterCallback()
 * Description: This function will register all calback with USBD layer.
 * Parameter: cy_stc_usb_app_ctxt_t.
 * return: None.
 */
void Cy_USB_AppRegisterCallback(cy_stc_usb_app_ctxt_t *pAppCtxt)
{
    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt = pAppCtxt->pUsbdCtxt;

    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_RESET,
                             Cy_USB_AppBusResetCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_RESET_DONE,
                             Cy_USB_AppBusResetDoneCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_BUS_SPEED,
                             Cy_USB_AppBusSpeedCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_SETUP,
                             Cy_USB_AppSetupCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_SUSPEND,
                             Cy_USB_AppSuspendCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_RESUME,
                             Cy_USB_AppResumeCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_SET_CONFIG,
                             Cy_USB_AppSetCfgCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_SET_INTF,
                             Cy_USB_AppSetIntfCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_L1_SLEEP,
                             Cy_USB_AppL1SleepCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_L1_RESUME,
                             Cy_USB_AppL1ResumeCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_ZLP,
                             Cy_USB_AppZlpCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_SLP,
                             Cy_USB_AppSlpCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_SETADDR,
                             Cy_USB_AppSetAddressCallback);
    return;
} /* end of function. */

/*
 * Function: CyUvcAppHandleProduceEvent()
 * Description: Function that handles a produce event indicating receipt of data through
 * the LVDS ingress socket.
 * Parameters:
 *      cy_stc_usb_app_ctxt_t *pUsbApp: Pointer to UVC application context structure.
 *      cy_stc_hbdma_channel_t *pChHandle: Pointer to DMA channel structure.
 * Return: None.
 */
static void
CyUvcAppHandleProduceEvent (
        cy_stc_usb_app_ctxt_t  *pUsbApp,
        cy_stc_hbdma_channel_t *pChHandle)
{
    cy_en_hbdma_mgr_status_t   status;
    cy_stc_hbdma_buff_status_t buffStat;

    /* Wait for a free buffer. */
    status = Cy_HBDma_Channel_GetBuffer(pChHandle, &buffStat);
    if (status != CY_HBDMA_MGR_SUCCESS)
    {
        DBG_APP_ERR("HB-DMA GetBuffer Error\r\n");
        return;
    }

    pUsbApp->glDmaBufCnt = buffStat.count;
    pUsbApp->glFrameSizeTransferred += pUsbApp->glDmaBufCnt;

    /* Add headers on every frame. Need to check if the EOF bit has to be set. */
    if (cy_uvc_buffer_counter <= cy_usb_full_buffer_no)
    {
#if UVC_HEADER_BY_FX2G3
        /* Not the end of frame. */
        Cy_USB_UvcAddHeader(buffStat.pBuffer - CY_USB_UVC_MAX_HEADER, CY_USB_UVC_HEADER_FRAME);
        cy_uvc_commitLength = buffStat.count + CY_USB_UVC_MAX_HEADER;
#else
        cy_uvc_commitLength = buffStat.count ;
#endif
        cy_uvc_buffer_counter++;
    }
    else
    {

#if UVC_HEADER_BY_FX2G3
        /* Short packet: End of frame. */
        Cy_USB_UvcAddHeader(buffStat.pBuffer - CY_USB_UVC_MAX_HEADER, CY_USB_UVC_HEADER_EOF);
        cy_uvc_commitLength = buffStat.count + CY_USB_UVC_MAX_HEADER;
#else
        cy_uvc_commitLength = buffStat.count ;
#endif
        cy_uvc_buffer_counter = 1;
        pUsbApp->glFrameSize = pUsbApp->glFrameSizeTransferred;
        pUsbApp->glFrameSizeTransferred = 0;
        pUsbApp->glFrameCount++;
        pUsbApp->glPartialBufSize = buffStat.count;
        pUsbApp->glConsCount = pUsbApp->glCons;
        pUsbApp->glProdCount = pUsbApp->glProd;
        pUsbApp->glPrintFlag = 1;
        pUsbApp->glCons = 0;
        pUsbApp->glProd = 0;
        pUsbApp->glfps++;
    }

#if UVC_HEADER_BY_FX2G3
    buffStat.count   = cy_uvc_commitLength;
    buffStat.pBuffer = buffStat.pBuffer - CY_USB_UVC_MAX_HEADER;
#endif

   
    /* Commit the buffer for transfer */
    Cy_USB_AppQueueWrite(pUsbApp, pUsbApp->uvcInEpNum, buffStat.pBuffer, cy_uvc_commitLength);

}

/*
 * Function: CyUvcAppHandleSendCompletion()
 * Description: Function that handles DMA transfer completion on the USB-HS BULK-IN
 * endpoint. This is equivalent to the receipt of a consume event in the USB-SS use
 * case and we can discard the active data buffer on the LVDS side.
 *
 * Parameters:
 *      cy_stc_usb_app_ctxt_t *pUsbApp: Pointer to UVC application context structure.
 * Return: None.
 */
void
CyUvcAppHandleSendCompletion (
        cy_stc_usb_app_ctxt_t *pUsbApp)
{
    cy_stc_hbdma_buff_status_t buffStat;
    cy_en_hbdma_mgr_status_t   dmaStat;

    /* At least one buffer must be pending. */
    if (pUsbApp->uvcPendingBufCnt == 0)
    {
        DBG_APP_ERR("PendingBufCnt=0 on SendComplete\r\n");
        return;
    }

    /* The buffer which has been sent to the USB host can be discarded. */
    dmaStat = Cy_HBDma_Channel_DiscardBuffer(pUsbApp->hbBulkInChannel, &buffStat);
    if (dmaStat != CY_HBDMA_MGR_SUCCESS)
    {
        DBG_APP_ERR("DiscardBuffer failed with status=%x\r\n", dmaStat);
        return;
    }

    /* If another DMA buffer has already been filled by the producer, go
     * on and send it to the host controller.
     */
    pUsbApp->uvcPendingBufCnt--;
    if (pUsbApp->uvcPendingBufCnt > 0)
    {
        CyUvcAppHandleProduceEvent(pUsbApp, pUsbApp->hbBulkInChannel);
    }
}


/*
 * Function: HbDma_Cb()
 * Description: It adds the UVC header to the frame/buffer coming from LVDS and commits to USB.
 * Parameter: cy_stc_hbdma_channel_t *, cy_en_hbdma_cb_type_t, cy_stc_hbdma_buff_status_t, void* .
 * Return: None.
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
        if ((cy_uvc_IsApplnActive == true) && (cy_uvc_devconfigured == true))
        {   
            pAppCtxt->uvcPendingBufCnt++;
            if (pAppCtxt->uvcPendingBufCnt == 1)
            {
                CyUvcAppHandleProduceEvent(pAppCtxt, handle);
            }   
        }
    }
}

/*
 * Function: Cy_USB_AppSetupEndpDmaParamsHs()
 * Description: This Function will setup Endpoint and DMA related parameters
 *             for high speed device before transfer initiated.
 * Parameter: cy_stc_usb_app_ctxt_t, pEndpDscr
 * return: void
 */
void
Cy_USB_AppSetupEndpDmaParamsHs (cy_stc_usb_app_ctxt_t *pUsbApp,
                                uint8_t *pEndpDscr)
{
    cy_stc_hbdma_chn_config_t dmaConfig;
    cy_en_hbdma_mgr_status_t mgrStat;
    cy_stc_app_endp_dma_set_t *pEndpDmaSet;
    DW_Type *pDW;
    bool stat;
    uint32_t endpNumber, dir;
    uint16_t maxPktSize;
    uint8_t *pCompDscr;
    uint8_t burstSize;
    uint8_t index = 0;

    Cy_USBD_GetEndpNumMaxPktDir(pEndpDscr, &endpNumber, &maxPktSize, &dir);
    pCompDscr = Cy_USBD_GetSsEndpCompDscr(pUsbApp->pUsbdCtxt, pEndpDscr);
    Cy_USBD_GetEndpCompnMaxburst(pCompDscr, &burstSize);

    DBG_APP_INFO("Cy_USB_AppSetupEndpDmaParamsHs: endpNum:0x%x maxPktSize:0x%x "
                 "dir:0x%x burstSize:0x%x\r\n",
                 endpNumber, maxPktSize,
                 dir, burstSize);

    /* Handling UVC streaming endpoint. */
    if ((endpNumber == UVC_STREAM_ENDPOINT) && (dir != 0)) {
        pUsbApp->uvcInEpNum       = (uint8_t)endpNumber;
        pUsbApp->uvcPendingBufCnt = 0;

    
        dmaConfig.consSckCount = 1;                         /* Only one consumer socket per channel. */
        dmaConfig.consSck[1] = (cy_hbdma_socket_id_t)0;
        dmaConfig.chType     = CY_HBDMA_TYPE_IP_TO_MEM;
        dmaConfig.consSck[0] = (cy_hbdma_socket_id_t)0;

#if UVC_HEADER_BY_FX2G3
        dmaConfig.prodHdrSize  = CY_USB_UVC_MAX_HEADER;
#elif UVC_HEADER_BY_FPGA
        dmaConfig.prodHdrSize  = 0;
#endif
        dmaConfig.eventEnable  = 0;          /* Manual channel: Disable event signalling between sockets. */
        dmaConfig.intrEnable   = LVDSSS_LVDS_ADAPTER_DMA_SCK_INTR_PRODUCE_EVENT_Msk |
                                 LVDSSS_LVDS_ADAPTER_DMA_SCK_INTR_CONSUME_EVENT_Msk;
        dmaConfig.cb           = HbDma_Cb;   /* HB-DMA callback */
        dmaConfig.prodBufSize  = FPGA_DMA_BUFFER_SIZE;        /* Receive depends on uvc header adds by FX2G3 or FPGA*/
        dmaConfig.size         = DMA_BUFFER_SIZE;             /* DMA Buffer size in bytes */
        dmaConfig.count        = CY_USB_UVC_STREAM_BUF_COUNT; /* DMA Buffer Count */
        dmaConfig.bufferMode   = true;                        /* DMA buffer mode enabled */
        dmaConfig.userCtx      = (void *)(pUsbApp);           /* Pass the application context as user context. */


#if INTERLEAVE_EN
        dmaConfig.prodSckCount = 2; /* No. of producer sockets */
        dmaConfig.prodSck[0] = CY_HBDMA_LVDS_SOCKET_00;
        dmaConfig.prodSck[1] = CY_HBDMA_LVDS_SOCKET_01;
#else
        dmaConfig.prodSckCount = 1; /* No. of producer sockets */
        dmaConfig.prodSck[0] = CY_HBDMA_LVDS_SOCKET_00;
        dmaConfig.prodSck[1] = (cy_hbdma_socket_id_t)0; /* Producer Socket ID: None */
#endif /* INTERLEAVE_EN */

        if (pUsbApp->hbBulkInChannel != NULL)
        {
            DBG_APP_ERR("Streaming DMA channel already created\r\n");
            return;
        }

        mgrStat = Cy_HBDma_Channel_Create(pUsbApp->pUsbdCtxt->pHBDmaMgr,
                &(pUsbApp->endpInDma[endpNumber].hbDmaChannel),
                &dmaConfig);
        if (mgrStat != CY_HBDMA_MGR_SUCCESS)
        {
            DBG_APP_ERR("BulkIn channel create failed 0x%x\r\n", mgrStat);
            return;
        }
        else
        {
            DBG_APP_INFO("BulkIn channel created BufSize %d , prod header %d\r\n", dmaConfig.size,dmaConfig.prodHdrSize);
        }

        /* Store the DMA channel pointer. */
        pUsbApp->hbBulkInChannel = &(pUsbApp->endpInDma[endpNumber].hbDmaChannel);
    }

#if AUDIO_IF_EN
    /* Handle UAC streaming endpoint. */
    if ((endpNumber == UAC_IN_ENDPOINT) && (dir != 0)) {
        /* Get the RAM buffers*/
        for(index = 0; index < PDM_APP_BUFFER_CNT; index++)
        {
            pUsbApp->pPDMRxBuffer[index] = Cy_USB_App_GetDmaBuffer(pUsbApp,192);
            if(pUsbApp->pPDMRxBuffer[index] == NULL)
            {
                DBG_APP_ERR("UAC DMA buffer NULL\r\n");
                return;
            }
        }

        pUsbApp->pdmRxBufIndex      = 0;
        pUsbApp->pdmRxFreeBufCount  = PDM_APP_BUFFER_CNT;
        pUsbApp->nxtAudioTxBufIndex = 0;
        pUsbApp->pdmInXferPending   = false;

        DBG_APP_INFO("HBDMA ISOCIN endpNum:%x\r\n", endpNumber);
    }
#endif /* AUDIO_IF_EN */

    endpNumber = ((*(pEndpDscr + CY_USB_ENDP_DSCR_OFFSET_ADDRESS)) & 0x7F);
    Cy_USBD_GetEndpMaxPktSize(pEndpDscr, &maxPktSize);

    if (*(pEndpDscr + CY_USB_ENDP_DSCR_OFFSET_ADDRESS) & 0x80)
    {
        dir = CY_USB_ENDP_DIR_IN;
        pEndpDmaSet = &(pUsbApp->endpInDma[endpNumber]);
        pDW = pUsbApp->pCpuDw1Base;
    }
    else
    {
        dir = CY_USB_ENDP_DIR_OUT;
        pEndpDmaSet = &(pUsbApp->endpOutDma[endpNumber]);
        pDW = pUsbApp->pCpuDw0Base;
    }

    stat = Cy_USBHS_App_EnableEpDmaSet(pEndpDmaSet, pDW, endpNumber, endpNumber,(cy_en_usb_endp_dir_t) dir, maxPktSize);
    DBG_APP_INFO("Enable EPDmaSet: endp=%x dir=%x stat=%x\r\n", endpNumber, (cy_en_usb_endp_dir_t)dir, stat);
} /* end of function  */


/*
 * Function: Cy_USB_AppConfigureEndp()
 * Description: This Function is used by application to configure endpoints
 *              after set configuration.  This function should be used for
 *              all endpoints except endp0.
 * Parameter: cy_stc_usb_usbd_ctxt_t, pEndpDscr
 * return: void
 */
void Cy_USB_AppConfigureEndp (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, uint8_t *pEndpDscr)
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
        /* The ISOINPKS setting in the USBHS register is the actual packets per microframe value. */
        isoPkts = ((*((uint8_t *)(pEndpDscr + CY_USB_ENDP_DSCR_OFFSET_MAX_PKT + 1)) & CY_USB_ENDP_ADDL_XN_MASK) >> CY_USB_ENDP_ADDL_XN_POS) + 1;
    }

    valid = 0x01;

    /* Prepare endpointConfig parameter. */
    endpConfig.endpType = (cy_en_usb_endp_type_t)endpType;
    endpConfig.endpDirection = endpDirection;
    endpConfig.valid = valid;
    endpConfig.endpNumber = endpNumber;
    endpConfig.maxPktSize = (uint32_t)maxPktSize;
    endpConfig.isoPkts = isoPkts;
    endpConfig.burstSize = burstSize;
    endpConfig.streamID = maxStream;
    endpConfig.allowNakTillDmaRdy = false;
    usbdRetCode = Cy_USB_USBD_EndpConfig(pUsbdCtxt, endpConfig);

    /* Print status of the endpoint configuration to help debug. */
    DBG_APP_INFO("#ENDPCFG: %d, %d\r\n", endpNumber, usbdRetCode);
    return;
} /* end of function */

/*
 * Function: Cy_USB_AppSetCfgCallback()
 * Description: This Function will be called by USBD  layer when
 *              set configuration command successful. This function
 *              does sanity check and prepare device for function
 *              to take over.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t
 * return: void
 */
void Cy_USB_AppSetCfgCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                              cy_stc_usb_cal_msg_t *pMsg)
{

    cy_uvc_devconfigured = true;
    cy_stc_usb_app_ctxt_t *pUsbApp;
    uint8_t *pActiveCfg, *pIntfDscr, *pEndpDscr;
    uint8_t index, numOfIntf, numOfEndp;


    DBG_APP_INFO("AppSetCfgCbStart\r\n");

    pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;
    pUsbApp->devSpeed = Cy_USBD_GetDeviceSpeed(pUsbdCtxt);


    /* Disable optional Low Power Mode transitions. */
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
        /* Set config should be called when active config value > 0x00. */
        return;
    }
    numOfIntf = Cy_USBD_FindNumOfIntf(pActiveCfg);
    if (numOfIntf == 0x00)
    {
        return;
    }

    for (index = 0x00; index < numOfIntf; index++)
    {
        /* During Set Config command always altSetting 0 will be active. */
        pIntfDscr = Cy_USBD_GetIntfDscr(pUsbdCtxt, index, 0x00);
        if (pIntfDscr == NULL)
        {
            DBG_APP_INFO("pIntfDscrNull\r\n");
            return;
        }

        numOfEndp = Cy_USBD_FindNumOfEndp(pIntfDscr);
        if (numOfEndp == 0x00)
        {
            DBG_APP_INFO("numOfEndp 0\r\n");
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

    /* Enable the interrupt for the DataWire channel used for video streaming to USBHS BULK-IN endpoint. */
    Cy_USB_AppInitDmaIntr(pUsbApp->uvcInEpNum, CY_USB_ENDP_DIR_IN, CY_UVC_DataWire_ISR);
    
    pUsbApp->prevDevState = CY_USB_DEVICE_STATE_CONFIGURED;
    pUsbApp->devState = CY_USB_DEVICE_STATE_CONFIGURED;

    DBG_APP_INFO("AppSetCfgCbEnd Done\r\n");
    return;
} /* end of function */

/*
 * Function: Cy_USB_AppBusResetCallback()
 * Description: This Function will be called by USBD when bus detects RESET.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t
 * return: void
 */
void Cy_USB_AppBusResetCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pUsbApp;

    pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;

    DBG_APP_INFO("AppBusResetCallback\r\n");

    /* Stop and destroy the high bandwidth DMA channel if present. To be done before AppInit is called. */
    if (pUsbApp->hbBulkInChannel != NULL)
    {
        DBG_APP_TRACE("HBDMA destroy\r\n");
        Cy_HBDma_Channel_Disable(pUsbApp->hbBulkInChannel);
        Cy_HBDma_Channel_Destroy(pUsbApp->hbBulkInChannel);
        pUsbApp->hbBulkInChannel = NULL;
    }

    /*
     * USBD layer takes care of reseting its own data structure as well as
     * takes care of calling CAL reset APIs. Application needs to take care
     * of reseting its own data structure as well as "device function".
     */
    Cy_USB_AppInit(pUsbApp, pUsbdCtxt, pUsbApp->pCpuDmacBase,
                   pUsbApp->pCpuDw0Base, pUsbApp->pCpuDw1Base,
                   pUsbApp->pHbDmaMgrCtxt);
    pUsbApp->devState = CY_USB_DEVICE_STATE_RESET;
    pUsbApp->prevDevState = CY_USB_DEVICE_STATE_RESET;
    cy_uvc_devconfigured = false;
    cy_uvc_IsApplnActive = false;
    return;
} /* end of function. */

/*
 * Function: Cy_USB_AppBusResetDoneCallback()
 * Description: This Function will be called by USBD  layer when
 *              set configuration command successful. This function
 *              does sanity check and prepare device for function
 *              to take over.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t
 * return: void
 */
void Cy_USB_AppBusResetDoneCallback(void *pAppCtxt,
                                    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                    cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pUsbApp;

    DBG_APP_INFO("ppBusResetDoneCallback\r\n");

    pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;
    pUsbApp->devState = CY_USB_DEVICE_STATE_DEFAULT;
    pUsbApp->prevDevState = pUsbApp->devState;
    return;
} /* end of function. */

extern uint8_t CyFxUSB20DeviceDscr[];

/*
 * Function: Cy_USB_AppBusSpeedCallback()
 * Description: This Function will be called by USBD  layer when
 *              speed is identified or speed change is detected.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t
 * return: void
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
} /* end of function. */

/*
 * Function: Cy_USB_AppSetupCallback()
 * Description: This Function will be called by USBD  layer when
 *              set configuration command successful. This function
 *              does sanity check and prepare device for function
 *              to take over.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t
 * return: void
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
    cy_en_usbd_ret_code_t retStatus = CY_USBD_STATUS_SUCCESS;
    cy_en_usb_endp_dir_t epDir = CY_USB_ENDP_DIR_INVALID;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    cy_stc_usbd_app_msg_t xMsg;
    BaseType_t status = 0;

    DBG_APP_INFO("AppSetupCallback\r\n");

    /* Fast enumeration is used. Only requests addressed to the interface, class,
     * vendor and unknown control requests are received by this function. */

    /* Decode the fields from the setup request. */
    bReqType = pUsbdCtxt->setupReq.bmRequest;
    bType = ((bReqType & CY_USB_CTRL_REQ_TYPE_MASK) >> CY_USB_CTRL_REQ_TYPE_POS);
    bTarget = (bReqType & CY_USB_CTRL_REQ_RECIPENT_OTHERS);
    bRequest = pUsbdCtxt->setupReq.bRequest;
    wValue = pUsbdCtxt->setupReq.wValue;
    wIndex = pUsbdCtxt->setupReq.wIndex;
    wLength = pUsbdCtxt->setupReq.wLength;

    if (bType == CY_USB_CTRL_REQ_STD)
    {
        DBG_APP_INFO("CY_USB_CTRL_REQ_STD\r\n");
        if (bRequest == CY_USB_SC_SET_FEATURE)
        {
            DBG_APP_INFO("CY_USB_SC_SET_FEATURE\r\n");
            if ((bTarget == CY_USB_CTRL_REQ_RECIPENT_INTF) && (wValue == 0))
            {
                DBG_APP_INFO("CY_USB_CTRL_REQ_RECIPENT_INTF\r\n");
                Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, 0x00, CY_USB_ENDP_DIR_IN, TRUE);

                isReqHandled = true;
            }

            /* SET-FEATURE(EP-HALT) is only supported to facilitate Chapter 9 compliance tests. */
            if ((bTarget == CY_USB_CTRL_REQ_RECIPENT_ENDP) && (wValue == CY_USB_FEATURE_ENDP_HALT))
            {
                DBG_APP_INFO("CY_USB_CTRL_REQ_RECIPENT_ENDP\r\n");
                epDir = ((wIndex & 0x80UL) ? (CY_USB_ENDP_DIR_IN) : (CY_USB_ENDP_DIR_OUT));
                Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, ((uint32_t)wIndex & 0x7FUL),
                        epDir, true);

                Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
                isReqHandled = true;
            }

            if ((bTarget == CY_USB_CTRL_REQ_RECIPENT_DEVICE) &&
                    ((wValue == CY_USB_FEATURE_U1_ENABLE) || (wValue == CY_USB_FEATURE_U2_ENABLE))) {
                /* Set U1/U2 enable. Just ACK the request. */
                Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
                isReqHandled = true;
            }
        }

        if (bRequest == CY_USB_SC_CLEAR_FEATURE)
        {
            DBG_APP_INFO("CY_USB_SC_CLEAR_FEATURE\r\n");
            if ((bTarget == CY_USB_CTRL_REQ_RECIPENT_INTF) && (wValue == 0))
            {
                DBG_APP_INFO("CY_USB_CTRL_REQ_RECIPENT_INTF\r\n");

                Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, 0x00, CY_USB_ENDP_DIR_IN, TRUE);

                isReqHandled = true;
            }

            if ((bTarget == CY_USB_CTRL_REQ_RECIPENT_ENDP) && (wValue == CY_USB_FEATURE_ENDP_HALT))
            {
                DBG_APP_INFO("CY_USB_CTRL_REQ_RECIPENT_ENDP && CY_USB_FEATURE_ENDP_HALT\r\n");
                epDir = ((wIndex & 0x80UL) ? (CY_USB_ENDP_DIR_IN) : (CY_USB_ENDP_DIR_OUT));
                if ((epDir == CY_USB_ENDP_DIR_OUT) || ((wIndex & 0x7F) != pUsbApp->uvcInEpNum))
                {
                    /* For any EP other than the UVC streaming endpoint, just clear the STALL bit. */
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
                /* Set U1/U2 disable. Just ACK the request. */
                Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
                isReqHandled = true;
            }
        }
    }

    /* Check for UVC Class Requests */
    if (bType == CY_USB_CTRL_REQ_CLASS)
    {

        DBG_APP_INFO("UVC Class Requests\r\n");
        /* Handle requests addressed to the Video Control interface. */
        if ((bTarget == CY_USB_CTRL_REQ_RECIPENT_INTF) && (CY_GET_LSB(wIndex) == CY_USB_UVC_INTERFACE_VC))
        {
            DBG_APP_INFO("CY_USB_CTRL_REQ_RECIPENT_INTF && CY_USB_UVC_INTERFACE_VC\r\n");
            /* Respond to VC_REQUEST_ERROR_CODE_CONTROL and stall every other request as this example does not support
               any of the Video Control features */
            if ((CY_GET_MSB(wIndex) == 0x00) && (wValue == CY_USB_UVC_VC_RQT_ERROR_CODE_CONTROL))
            {
                temp = CY_USB_UVC_RQT_STAT_INVALID_CTRL;
                isReqHandled = true;
                DBG_APP_INFO("UVC_RQT_STAT_INVALID_CTRL\r\n");
                Cy_USB_USBD_SendEndp0Data(pUsbdCtxt, &temp, 0x01);
            }
        }

        /* Handle requests addressed to the Video Streaming interface. */
        if ((bTarget == CY_USB_CTRL_REQ_RECIPENT_INTF) && (CY_GET_LSB(wIndex) == CY_USB_UVC_INTERFACE_VS))
        {
            isReqHandled = true;
            DBG_APP_INFO("CY_USB_CTRL_REQ_RECIPENT_INTF && CY_USB_UVC_INTERFACE_VS\r\n");
            switch (wValue)
            {

            case CY_USB_UVC_VS_PROBE_CONTROL:
            case CY_USB_UVC_VS_COMMIT_CONTROL:
            {
                switch (bRequest)
                {
                case CY_USB_UVC_GET_INFO_REQ:
                    DBG_APP_INFO("UVC_GET_INFO_REQ\r\n");
                    Ep0TestBuffer[0] = 3; /* GET/SET requests are supported. */
                    Cy_USB_USBD_SendEndp0Data(pUsbdCtxt, (uint8_t *)Ep0TestBuffer, 0x01);
                    break;

                case CY_USB_UVC_GET_LEN_REQ:
                    DBG_APP_INFO("UVC_GET_LEN_REQ\r\n");
                    Ep0TestBuffer[0] = CY_USB_UVC_MAX_PROBE_SETTING;
                    Cy_USB_USBD_SendEndp0Data(pUsbdCtxt, (uint8_t *)Ep0TestBuffer, 0x01);
                    break;

                /* We only have one functional setting. Keep returning the same as current, default
                 * minimum and maximum. */
                case CY_USB_UVC_GET_CUR_REQ:
                case CY_USB_UVC_GET_DEF_REQ:
                case CY_USB_UVC_GET_MIN_REQ:
                case CY_USB_UVC_GET_MAX_REQ:
                    DBG_APP_INFO("UVC_GET_CUR_DEF_MIN_MAX_REQ:%x \r\n", bRequest);
            
                    if (cy_uvc_currentFrameIndex == CY_USB_UVC_HS_VGA_FRAME_INDEX)
                    {
                        retStatus = Cy_USB_USBD_SendEndp0Data(pUsbdCtxt,
                                                                (uint8_t *)cy_uvc_probectrl_HS_VGA, CY_USB_UVC_MAX_PROBE_SETTING);
                    }
                    else if (cy_uvc_currentFrameIndex == CY_USB_UVC_HS_1080P_FRAME_INDEX)
                    {
                        retStatus = Cy_USB_USBD_SendEndp0Data(pUsbdCtxt,
                                                              (uint8_t *)cy_uvc_probectrl_HS_1080p, CY_USB_UVC_MAX_PROBE_SETTING);
                    }                    
                    else
                    {
                        DBG_APP_ERR("currentFrameIndex\r\n");
                    }

                    if (retStatus != CY_USBD_STATUS_SUCCESS)
                    {
                        DBG_APP_ERR("Error SendEndp0Data\r\n");
                    }
                    break;

                case CY_USB_UVC_SET_CUR_REQ:
                    /* Since this request has OUT data, it should be handled in task context. */
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

        /* Don't try to stall the endpoint if we have already attempted data transfer. */
    }
    /*
     * If Request is not handled by the callback, Stall the command.
     */
    if (!isReqHandled)
    {
        Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, 0x00,
                                      CY_USB_ENDP_DIR_IN, TRUE);
    }
} /* end of function. */

/*
 * Function: Cy_USB_AppSuspendCallback()
 * Description: This Function will be called by USBD  layer when
 *              Suspend signal/message is detected.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t, cy_stc_usb_cal_msg_t
 * return: void
 */
void Cy_USB_AppSuspendCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                               cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pUsbApp;

    pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;
    pUsbApp->prevDevState = pUsbApp->devState;
    pUsbApp->devState = CY_USB_DEVICE_STATE_SUSPEND;
} /* end of function. */

/*
 * Function: Cy_USB_AppResumeCallback()
 * Description: This Function will be called by USBD  layer when
 *              Resume signal/message is detected.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t, cy_stc_usb_cal_msg_t
 * return: void
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
} /* end of function. */

/*
 * Function: Cy_USB_AppSetIntfCallback()
 * Description: This Function will be called by USBD  layer when
 *              set interface is called.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t, cy_stc_usb_cal_msg_t
 * return: void
 */
void Cy_USB_AppSetIntfCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                               cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_setup_req_t *pSetupReq;
    uint8_t intfNum, altSetting;

    DBG_APP_INFO("AppSetIntfCallback Start\r\n");
    pSetupReq  = &(pUsbdCtxt->setupReq);
    intfNum    = pSetupReq->wIndex;
    altSetting = pSetupReq->wValue;

#if AUDIO_IF_EN
    if (intfNum == UAC_STREAM_INTF_NUM) {
        Cy_App_SetUACIntfHandler((cy_stc_usb_app_ctxt_t *)pAppCtxt, altSetting);
        return;
    }
#endif /* AUDIO_IF_EN */

    /* Interface does not support multiple alternate settings: Stall the SET_INTERFACE request. */
    DBG_APP_INFO("SetIntf(%d) on interface %d: Stalled\r\n", altSetting, intfNum);
    Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, 0x00, CY_USB_ENDP_DIR_IN, true);
    DBG_APP_INFO("AppSetIntfCallback done\r\n");
} /* end of function. */

/*
 * Function: Cy_USB_AppZlpCallback()
 * Description: This Function will be called by USBD layer when
 *              ZLP message comes.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t, cy_stc_usb_cal_msg_t
 * return: void
 */
void Cy_USB_AppZlpCallback(void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                           cy_stc_usb_cal_msg_t *pMsg)
{
    DBG_APP_TRACE("AppZlpCb\r\n");

    return;
} /* end of function. */

/*
 * Function: Cy_USB_AppL1SleepCallback()
 * Description: This Function will be called by USBD layer when
 *              L1 Sleep message comes.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t, cy_stc_usb_cal_msg_t
 * return: void
 */
void Cy_USB_AppL1SleepCallback(void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                               cy_stc_usb_cal_msg_t *pMsg)
{
    DBG_APP_INFO("AppL1SleepCb\r\n");
    return;
} /* end of function. */

/*
 * Function: Cy_USB_AppL1ResumeCallback()
 * Description: This Function will be called by USBD layer when
 *              L1 Resume message comes.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t, cy_stc_usb_cal_msg_t
 * return: void
 */
void Cy_USB_AppL1ResumeCallback(void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                cy_stc_usb_cal_msg_t *pMsg)
{
    DBG_APP_INFO("AppL1ResumeCb\r\n");
    return;
} /* end of function. */

/*
 * Function: Cy_USB_AppSlpCallback()
 * Description: This Function will be called by USBD layer when
 *              SLP message comes.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t, cy_stc_usb_cal_msg_t
 * return: void
 */
void Cy_USB_AppSlpCallback(void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                           cy_stc_usb_cal_msg_t *pMsg)
{
    DBG_APP_INFO("AppSlpCb\r\n");
    return;
} /* end of function. */

/*
 * Function: Cy_USB_AppSetFeatureCallback()
 * Description: This Function will be called by USBD layer when
 *              set feature message comes.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t, cy_stc_usb_cal_msg_t
 * return: void
 */
void Cy_USB_AppSetFeatureCallback(void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                  cy_stc_usb_cal_msg_t *pMsg)
{
    DBG_APP_INFO("AppSetFeatureCb\r\n");
    return;
} /* end of function. */

/*
 * Function: Cy_USB_AppClearFeatureCallback()
 * Description: This Function will be called by USBD layer when
 *              clear feature message comes.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t, cy_stc_usb_cal_msg_t
 * return: void
 */
void Cy_USB_AppClearFeatureCallback(void *pUsbApp,
                                    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                    cy_stc_usb_cal_msg_t *pMsg)
{
    DBG_APP_INFO("AppClearFeatureCb\r\n");
    return;
} /* end of function. */

/*
 * Function: Cy_USB_AppQueueWrite()
 * Description: Function to queue write operation on an IN endpoint.
 * Parameter: pAppCtxt, endpNumber, pBuffer, dataSize
 * return: void
 */
void Cy_USB_AppQueueWrite(cy_stc_usb_app_ctxt_t *pAppCtxt, uint8_t endpNumber,
                          uint8_t *pBuffer, uint16_t dataSize)
{
    cy_stc_app_endp_dma_set_t *dmaset_p;

    /* Null pointer checks. */
    if ((pAppCtxt == NULL) || (pAppCtxt->pUsbdCtxt == NULL) ||
            (pAppCtxt->pCpuDw1Base == NULL) || (pBuffer == NULL)) {
        DBG_APP_ERR("QueueWrite Err0\r\n");
        return;
    }

    /*
     * Verify that the selected endpoint is valid and the dataSize
     * is non-zero.
     */
    dmaset_p = &(pAppCtxt->endpInDma[endpNumber]);
    if ((dmaset_p->valid == 0) || (dataSize == 0)) {
        DBG_APP_ERR("QueueWrite Err1\r\n");
        return;
    }

    Cy_USBHS_App_QueueWrite(dmaset_p, pBuffer, dataSize);
} /* end of function */

/*
 * Function: Cy_USB_AppInitDmaIntr()
 * Description: Function to register an ISR for the DMA channel associated
 *              with an endpoint.
 * Parameters:
 *      endpNumber: Endpoint number
 *      endpDirection: Endpoint direction.
 *      userIsr: ISR function pointer. Can be NULL if interrupt is to be
 *               disabled.
 * return: void
 */
void Cy_USB_AppInitDmaIntr(uint32_t endpNumber, cy_en_usb_endp_dir_t endpDirection,
                           cy_israddress userIsr)
{
    cy_stc_sysint_t intrCfg;
    if ((endpNumber > 0) && (endpNumber < CY_USB_MAX_ENDP_NUMBER))
    {
#if (!CY_CPU_CORTEX_M4)
        intrCfg.intrPriority = 3;
        intrCfg.intrSrc = NvicMux7_IRQn;
        if (endpDirection == CY_USB_ENDP_DIR_IN)
        {
            /* DW1 channels 0 onwards are used for IN endpoints. */
            intrCfg.cm0pSrc = (cy_en_intr_t)(cpuss_interrupts_dw1_0_IRQn + endpNumber);
        }
        else
        {
            /* DW0 channels 0 onwards are used for OUT endpoints. */
            intrCfg.cm0pSrc = (cy_en_intr_t)(cpuss_interrupts_dw0_0_IRQn + endpNumber);
        }
#else
        intrCfg.intrPriority = 5;
        if (endpDirection == CY_USB_ENDP_DIR_IN)
        {
            /* DW1 channels 0 onwards are used for IN endpoints. */
            intrCfg.intrSrc =
                (IRQn_Type)(cpuss_interrupts_dw1_0_IRQn + endpNumber);
        }
        else
        {
            /* DW0 channels 0 onwards are used for OUT endpoints. */
            intrCfg.intrSrc =
                (IRQn_Type)(cpuss_interrupts_dw0_0_IRQn + endpNumber);
        }
#endif /* (!CY_CPU_CORTEX_M4) */

        if (userIsr != NULL)
        {
            /* If an ISR is provided, register it and enable the interrupt. */
            Cy_SysInt_Init(&intrCfg, userIsr);
            NVIC_EnableIRQ(intrCfg.intrSrc);
        }
        else
        {
            /* ISR is NULL. Disable the interrupt. */
            NVIC_DisableIRQ(intrCfg.intrSrc);
        }
    }
} /* end of function. */

/*
 * Function: Cy_USB_AppClearDmaInterrupt()
 * Description: Function to clear the pending DMA interrupt associated with an
 *              endpoint.
 * Parameters:
 *      pAppCtxt: Pointer to USB application context structure.
 *      endpNumber: Endpoint number
 *      endpDirection: Endpoint direction.
 * return: void
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
} /* end of function. */

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

void Cy_USB_SendEndp0DataFailHandler(void)
{
    DBG_APP_TRACE("Reset Done\r\n");
}


/*[]*/

