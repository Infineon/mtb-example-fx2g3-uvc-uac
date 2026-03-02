/***************************************************************************//**
* \file usb_inmem.c
* \version 1.0
*
* Implements code to handle streaming of video data from internal RAM buffers.
*
*******************************************************************************
* \copyright
* (c) (2021-2024), Cypress Semiconductor Corporation (an Infineon company) or
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
#include "cy_usbss_cal_drv.h"
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

typedef struct {
    uint8_t *evenFrameBuf1;             /* First buffer for even frames */
    uint8_t *evenFrameBuf2;             /* Second buffer for even frames */

    uint8_t *oddFrameBuf1;              /* First buffer for odd frames */
    uint8_t *oddFrameBuf2;              /* Second buffer for odd frames */

    uint8_t *whiteBandBuf;              /* Buffer containing the white band data */

    uint32_t currentFrameIndex;         /* Selected streaming frame index */
    uint32_t buffersPerFrame;           /* Number of DMA buffers to be sent per frame */
    uint32_t lastBufferSize;            /* Size of last DMA buffer in bytes */
    uint32_t commitBufCnt;              /* Count of buffers committed in current frame */
    uint32_t completeFrameCnt;          /* Number of video frames completed */
    uint32_t whiteBufferIndex;          /* Buffer index where the white band is to be inserted */
    uint32_t whiteBufferFreq;           /* Number of complete frames to send before moving the white band buffer */
} cy_uvc_app_frame_info_t;

#if UVC_INMEM_EN

/*
 * In-Memory Video Streaming:
 *
 * This module implements the logic to send a colorbar pattern with a moving
 * whiteband as part of the UVC video stream.
 *
 * Streaming is performed using three buffers of 60 KB size. The first two buffers
 * are used to send the even video frames and the last two are used to send the
 * odd video frames. The buffers are pre-filled with a 32-byte UVC header followed
 * by a repeating colorbar pattern. Since the first 32 bytes of the first buffer
 * hold the UVC header, we need an additional buffer of 32 bytes to be sent at the
 * end of each frame.
 *
 * Since the same data is used for all video resolutions, the number of bands in the
 * colorbar will vary based on the selected resolution. A single set of 8 colours
 * will be observed when streaming video at 480p resolution. At 720p, two sets of
 * coloured bands are seen and at 2160p, six sets of coloured bands are seen.
 *
 * The appearance of a moving video frame is provided by inserting a white band
 * spanning the entire width of the frame at varying locations.
 */

/* Structure holding all In-MEM streaming info */
static cy_uvc_app_frame_info_t glInMemInfo;

const uint32_t InMemColorInfo[8] = {
    0x80ff80ff,
    0x94ff00ff,
    0x1ac8bfc8,
    0x4aca55ca,
    0xf3969f96,
    0xff4c544c,
    0x9e40d340,
    0x80008000
};

/**
 * \name Cy_UvcInMem_AllocateBuffers
 * \brief Allocate the DMA buffers used to hold the colorbar data which is repeatedly
          sent when UVC streaming from internal memory is enabled.
 * \param pAppCtxt application layer context pointer.eed
 * \return true if allocation is successful, false otherwise
 */
bool
Cy_UvcInMem_AllocateBuffers (
        cy_stc_usb_app_ctxt_t *pAppCtxt)
{
    cy_stc_hbdma_buf_mgr_t *pBufMgr;

    if (pAppCtxt == NULL) {
        DBG_APP_ERR("Unable to allocate video buffers: pAppCtxt=%x\r\n", pAppCtxt);
        return false;
    }

    pBufMgr = pAppCtxt->pHbDmaMgrCtxt->pBufMgr;

    /*
     * First buffer will hold the UVC header + video frame data.
     * Second buffer will hold only video data.
     * Third buffer will hold the last 32 bytes of video data.
     */
    glInMemInfo.evenFrameBuf1 = (uint8_t *)Cy_HBDma_BufMgr_Alloc(pBufMgr, UVC_INMEM_BUF_SIZE);
    glInMemInfo.evenFrameBuf2 = (uint8_t *)Cy_HBDma_BufMgr_Alloc(pBufMgr, UVC_INMEM_BUF_SIZE);

    glInMemInfo.oddFrameBuf1  = (uint8_t *)Cy_HBDma_BufMgr_Alloc(pBufMgr, UVC_INMEM_BUF_SIZE);
    glInMemInfo.oddFrameBuf2  = (uint8_t *)Cy_HBDma_BufMgr_Alloc(pBufMgr, UVC_INMEM_BUF_SIZE);

    glInMemInfo.whiteBandBuf  = (uint8_t *)Cy_HBDma_BufMgr_Alloc(pBufMgr, UVC_INMEM_BUF_SIZE);

    if (
            (glInMemInfo.evenFrameBuf1 == NULL) ||
            (glInMemInfo.evenFrameBuf2 == NULL) ||
            (glInMemInfo.oddFrameBuf1  == NULL) ||
            (glInMemInfo.oddFrameBuf2  == NULL) ||
            (glInMemInfo.whiteBandBuf  == NULL)
       ) {
        DBG_APP_ERR("Failed to allocate buffers to hold video data\r\n");
        if (glInMemInfo.evenFrameBuf1 != NULL)
            Cy_HBDma_BufMgr_Free(pBufMgr, glInMemInfo.evenFrameBuf1);
        if (glInMemInfo.evenFrameBuf2 != NULL)
            Cy_HBDma_BufMgr_Free(pBufMgr, glInMemInfo.evenFrameBuf2);
        if (glInMemInfo.oddFrameBuf1 != NULL)
            Cy_HBDma_BufMgr_Free(pBufMgr, glInMemInfo.oddFrameBuf1);
        if (glInMemInfo.oddFrameBuf2 != NULL)
            Cy_HBDma_BufMgr_Free(pBufMgr, glInMemInfo.oddFrameBuf2);
        if (glInMemInfo.whiteBandBuf != NULL)
            Cy_HBDma_BufMgr_Free(pBufMgr, glInMemInfo.whiteBandBuf);
        return false;
    }

    return true;
}

/* Minimum supported UVC frame width in pixels */
#define CY_UVC_MIN_FRAME_WIDTH          (640)

/* Size of each color band: (MIN_FRAME_WIDTH * bytes per pixel) / (no. of colors * sizeof(uint32_t)) */
#define CY_UVC_COLOR_BAND_SIZE          ((CY_UVC_MIN_FRAME_WIDTH * 2) / (8 * 4))

/* Maximum number of lines per DMA buffer */
#define CY_UVC_MAX_LINES_PER_BUFF       (UVC_INMEM_BUF_SIZE / (CY_UVC_MIN_FRAME_WIDTH * 2))

/*
 * Size of white band in terms of 480p video lines. This needs to be set to a multiple of 6 such that
 * the size spans an integral number of 2160p video lines as well.
 */
#define CY_UVC_WHITEBAND_HEIGHT         (36)

/**
 * \name Cy_UvcInMem_PrepareBuffers
 * \brief Fill the pre-allocated RAM buffers with the UVC header and colorbar video
 * data. Also updates the DMA descriptors in the streaming DMA channel to
 * point to these RAM buffers.
 * \param pAppCtxt application layer context pointer.eed
 * \param pChannel Handle to the UVC streaming channe
 * \return true if buffer updates are successful, false otherwise.
 */
bool
Cy_UvcInMem_PrepareBuffers (
        cy_stc_usb_app_ctxt_t  *pAppCtxt,
        cy_stc_hbdma_channel_t *pChannel)
{
    cy_stc_hbdma_desc_t dscr;
    uint16_t dscrIndex, i;
    uint32_t *pEvnHdr, *pOddHdr;
    uint32_t *pEvnFrame, *pOddFrame;
    uint32_t ln, band, pxl, count;
    uint32_t *pWhiteBand;

    if ((pAppCtxt == NULL) || (pChannel == NULL)) {
        DBG_APP_ERR("Unable to prepare video buffers: pAppCtxt=%x pChannel=%x\r\n", pAppCtxt, pChannel);
        return false;
    }

    /* Pre-fill the buffers with the UVC headers */
    pEvnHdr = (uint32_t *)(glInMemInfo.evenFrameBuf1);
    pOddHdr = (uint32_t *)(glInMemInfo.oddFrameBuf1);

    *pEvnHdr++ = 0x00008E20;              /* bHeaderLength = 32, FrameID = 0, End of frame = 1, End of header = 1 */
    *pOddHdr++ = 0x00008F20;              /* bHeaderLength = 32, FrameID = 1, End of frame = 1, End of header = 1 */
    *pEvnHdr++ = *pOddHdr++ = 0x00000000;
    *pEvnHdr++ = *pOddHdr++ = 0x00000000;
    *pEvnHdr++ = *pOddHdr++ = 0x00000000;
    *pEvnHdr++ = *pOddHdr++ = 0x00000000;
    *pEvnHdr++ = *pOddHdr++ = 0x00000000;
    *pEvnHdr++ = *pOddHdr++ = 0x00000000;
    *pEvnHdr++ = *pOddHdr++ = 0x00000000;

    /* Fill the buffers with the colorbar data */
    pEvnFrame  = (uint32_t *)(glInMemInfo.evenFrameBuf1 + 32);
    pOddFrame  = (uint32_t *)(glInMemInfo.oddFrameBuf1 + 32);

    /*
     * This loop fills two DMA buffers corresponding to each of even and odd frames with a colorbar
     * pattern. The data is formatted so that it forms an colorbar with 8 colours for the minimum
     * resolution of 640*480 pixels.
     */
    count = 32;
    for (ln = 0; ln < (2 * CY_UVC_MAX_LINES_PER_BUFF); ln++) {
        for (band = 0; band < 8; band++) {
            for (pxl = 0; pxl < CY_UVC_COLOR_BAND_SIZE; pxl++) {
                *pEvnFrame++ = InMemColorInfo[band];
                *pOddFrame++ = InMemColorInfo[band];

                count += 4;
                if (count == UVC_INMEM_BUF_SIZE) {
                    pEvnFrame = (uint32_t *)(glInMemInfo.evenFrameBuf2);
                    pOddFrame = (uint32_t *)(glInMemInfo.oddFrameBuf2);
                }
            }
        }
    }

    /*
     * The first 32-bytes of the buffer complete the previous line of video and should be
     * filled with the last colour in the band.
     *
     * The first 36 lines of 480p (6 lines of 2160p) video will be filled with only
     * white color. The rest of the buffer will be filled with normal colorbar pattern.
     */
    pWhiteBand    = (uint32_t *)(glInMemInfo.whiteBandBuf);
    *pWhiteBand++ = InMemColorInfo[7];
    *pWhiteBand++ = InMemColorInfo[7];
    *pWhiteBand++ = InMemColorInfo[7];
    *pWhiteBand++ = InMemColorInfo[7];
    *pWhiteBand++ = InMemColorInfo[7];
    *pWhiteBand++ = InMemColorInfo[7];
    *pWhiteBand++ = InMemColorInfo[7];
    *pWhiteBand++ = InMemColorInfo[7];

    count = 32;
    for (ln = 0; ln < (CY_UVC_MAX_LINES_PER_BUFF); ln++) {
        for (band = 0; band < 8; band++) {
            for (pxl = 0; pxl < CY_UVC_COLOR_BAND_SIZE; pxl++) {
                *pWhiteBand++ = (ln >= CY_UVC_WHITEBAND_HEIGHT) ? InMemColorInfo[band] : InMemColorInfo[0];

                /* Make sure we do not over-run the buffer */
                count += 4;
                if (count >= UVC_INMEM_BUF_SIZE) {
                    break;
                }
            }
        }
    }

    dscrIndex = pChannel->firstProdDscrIndex[0];

    Cy_HBDma_GetDescriptor(dscrIndex, &dscr);
    if (dscr.pBuffer != NULL) {
        Cy_HBDma_BufMgr_Free(pChannel->pContext->pBufMgr, dscr.pBuffer);
    }
    dscr.pBuffer = glInMemInfo.evenFrameBuf1;
    Cy_HBDma_SetDescriptor(dscrIndex, &dscr);
    dscrIndex = CY_HBDMA_CHAIN_TO_NEXT_WR_IDX(dscr.chain);

    /* Free the original memory buffers and point the descriptors to the pre-filled data buffers */
    for (i = 1; i < UVC_INMEM_BUF_COUNT; i++) {
        Cy_HBDma_GetDescriptor(dscrIndex, &dscr);
        if (dscr.pBuffer != NULL) {
            Cy_HBDma_BufMgr_Free(pChannel->pContext->pBufMgr, dscr.pBuffer);
        }
        dscr.pBuffer = glInMemInfo.evenFrameBuf2;
        Cy_HBDma_SetDescriptor(dscrIndex, &dscr);
        dscrIndex = CY_HBDMA_CHAIN_TO_NEXT_WR_IDX(dscr.chain);
    }

    return true;
}

/**
 * \name Cy_UvcInMem_ClearBufPointers
 * \brief Clear the buffer pointers in all descriptors associated with the UVC streaming
 * channel before the channel is destroyed. This ensures that the DMA buffers
 * are not freed when the channel gets destroyed.
 * \param pAppCtxt application layer context pointer.eed
 * \param pChannel Handle to the UVC streaming channel
 * \return true if buffer updates are successful, false otherwise.
 */
bool
Cy_UvcInMem_ClearBufPointers (
        cy_stc_usb_app_ctxt_t  *pAppCtxt,
        cy_stc_hbdma_channel_t *pChannel)
{
    cy_stc_hbdma_desc_t dscr;
    uint16_t dscrIndex, i;

    if ((pAppCtxt == NULL) || (pChannel == NULL)) {
        DBG_APP_ERR("Unable to clear buffer pointers: pAppCtxt=%x pChannel=%x\r\n", pAppCtxt, pChannel);
        return false;
    }

    /* Mark the buffer pointer for all DMA descriptors as NULL so that channel destroy does not free them */
    i         = 0;
    dscrIndex = pChannel->firstProdDscrIndex[0];
    do {
        Cy_HBDma_GetDescriptor(dscrIndex, &dscr);
        dscr.pBuffer = NULL;
        Cy_HBDma_SetDescriptor(dscrIndex, &dscr);
        dscrIndex = CY_HBDMA_CHAIN_TO_NEXT_WR_IDX(dscr.chain);
        i++;
    } while (i < UVC_INMEM_BUF_COUNT);

    return true;
}

/*
 * This look-up table specifies the number of frames to be completed before moving the white
 * band to the next position.
 */
const uint32_t glWhiteBandFreq[4][4] = {
    {0xFFFFFFFF, 1, 10, 30},                    /* For each resolution in Gen1x1 speed */
    {0xFFFFFFFF, 2, 18, 55},                    /* For each resolution in Gen1x2 speed */
    {0xFFFFFFFF, 2, 21, 65},                    /* For each resolution in Gen2x1 speed */
    {0xFFFFFFFF, 5, 42, 127}                    /* For each resolution in Gen2x2 speed */
};

 /**
 * \name Cy_UvcInMem_CommitBuffers
 * \brief Start the video stream from the pre-filled RAM buffers by committing all
 * available descriptors in the DMA channel.
 * \param pAppCtxt application layer context pointer.eed
 * \param pChannel Handle to the UVC video streaming DMA channel.
 * \param frmIndex Index of the currently selected video frame.
 * \return none
 */
void
Cy_UvcInMem_CommitBuffers (
        cy_stc_usb_app_ctxt_t  *pAppCtxt,
        cy_stc_hbdma_channel_t *pChannel,
        uint8_t                 frmIndex)
{
    cy_en_hbdma_mgr_status_t   dmaStat;
    cy_stc_hbdma_buff_status_t bufStat;
    uint32_t intrState;

    /* Place the endpoint in NAK state so that data does not start flowing till we commit all buffers */
    Cy_USB_USBD_EndpSetClearNakNrdy(pAppCtxt->pUsbdCtxt, pAppCtxt->uvcInEpNum, CY_USB_ENDP_DIR_IN, true);

    intrState = Cy_SysLib_EnterCriticalSection();

    /* Commit the first UVC_INMEM_BUF_COUNT with data.
     * We can only queue one buffer in USB-HS case
     */
    dmaStat = Cy_HBDma_Channel_GetBuffer(pChannel, &bufStat);
    if (dmaStat != CY_HBDMA_MGR_SUCCESS) {
        DBG_APP_ERR("UVC: HB-DMA Commit Buffer Error: %x\r\n",dmaStat);
        Cy_USB_USBD_EndpSetClearNakNrdy(pAppCtxt->pUsbdCtxt, pAppCtxt->uvcInEpNum, CY_USB_ENDP_DIR_IN, false);
        Cy_SysLib_ExitCriticalSection(intrState);
        return;
    }

    bufStat.pBuffer = glInMemInfo.evenFrameBuf1;
    bufStat.count   = UVC_INMEM_BUF_SIZE;

    dmaStat = Cy_HBDma_Channel_CommitBuffer(pChannel, &bufStat);
    if (dmaStat != CY_HBDMA_MGR_SUCCESS)
    {
        DBG_APP_ERR("UVC: HB-DMA Commit Buffer Error: %x\r\n",dmaStat);
        return;
    }

    glInMemInfo.currentFrameIndex = 0;
    glInMemInfo.commitBufCnt      = 1;
    glInMemInfo.completeFrameCnt  = 0;

    /* When streaming the video frame, insert a white band into the frame to produce a moving image.
     * Similar logic can be added for other resolutions as required.
     */
    glInMemInfo.currentFrameIndex = frmIndex;

    switch (frmIndex)
    {
        case 1:
            glInMemInfo.buffersPerFrame  = (CY_USB_FULL_FRAME_SIZE_1080P / UVC_INMEM_BUF_SIZE) + 1;
            glInMemInfo.lastBufferSize   = (CY_USB_FULL_FRAME_SIZE_1080P % UVC_INMEM_BUF_SIZE );
            glInMemInfo.whiteBufferIndex = 2;
            glInMemInfo.whiteBufferFreq  = 3;           /* Move the white band to next position after 3 frames */
        break;
        case 2:
            glInMemInfo.buffersPerFrame  = (CY_USB_FULL_FRAME_SIZE_VGA / UVC_INMEM_BUF_SIZE) + 1;
            glInMemInfo.lastBufferSize   = CY_USB_UVC_MAX_HEADER;
            glInMemInfo.whiteBufferIndex = 2;
            glInMemInfo.whiteBufferFreq  = 3;           /* Move the white band to next position after 3 frames */

        break;
        default:
                DBG_APP_ERR("Invalid SS frame index: %d\r\n", frmIndex);
            break;
    }

    Cy_USB_USBD_EndpSetClearNakNrdy(pAppCtxt->pUsbdCtxt, pAppCtxt->uvcInEpNum, CY_USB_ENDP_DIR_IN, false);
    Cy_SysLib_ExitCriticalSection(intrState);
}

 /**
 * \name Cy_UvcInMem_DmaCallback
 * \brief Callback function which gets invoked whenever the device has finished
 * sending one of the RAM buffers with pre-filled video data. This function queues
 * the buffer for transfer again so that we have a continuous pipeline of buffers
 * ready for transfer.
 * \param pAppCtxt application layer context pointer.eed
 * \param type Type of DMA event (can only be a consume event).
 * \param pbufStat Buffer status associated with the event. Not used in this implementation.
 * \param userCtx Application context passed back to the callback as an opaque pointer
 * \return none
 */
void
Cy_UvcInMem_DmaCallback (
        cy_stc_hbdma_channel_t      *handle,
        cy_en_hbdma_cb_type_t       type,
        cy_stc_hbdma_buff_status_t  *pbufStat,
        void                        *userCtx)
{
    cy_stc_hbdma_buff_status_t bufStat;
    cy_en_hbdma_mgr_status_t   status;
    cy_stc_usb_app_ctxt_t *pAppCtxt = (cy_stc_usb_app_ctxt_t *)userCtx;

    status = Cy_HBDma_Channel_GetBuffer(handle, &bufStat);
    if (status != CY_HBDMA_MGR_SUCCESS)
    {
        DBG_APP_ERR("UVC: HB-DMA Get Buffer Error: %x\r\n",status);
        return;
    }

    if (glInMemInfo.commitBufCnt == 0) {
        bufStat.pBuffer = ((glInMemInfo.completeFrameCnt & 1) == 0) ?
            glInMemInfo.evenFrameBuf1 : glInMemInfo.oddFrameBuf1;
        bufStat.count   = UVC_INMEM_BUF_SIZE;
    } else {
        if (glInMemInfo.commitBufCnt == (glInMemInfo.buffersPerFrame - 1)) {
            bufStat.pBuffer = ((glInMemInfo.completeFrameCnt & 1) == 0) ?
                glInMemInfo.evenFrameBuf2 : glInMemInfo.oddFrameBuf2;
            bufStat.count   = glInMemInfo.lastBufferSize;
        } else {
            if (glInMemInfo.commitBufCnt == glInMemInfo.whiteBufferIndex) {
                bufStat.pBuffer = glInMemInfo.whiteBandBuf;
            } else {
                bufStat.pBuffer = ((glInMemInfo.completeFrameCnt & 1) == 0) ?
                    glInMemInfo.evenFrameBuf2 : glInMemInfo.oddFrameBuf2;
            }
            bufStat.count = UVC_INMEM_BUF_SIZE;
        }
    }

    glInMemInfo.commitBufCnt++;
    bufStat.size   = UVC_INMEM_BUF_SIZE;
    if (glInMemInfo.commitBufCnt == glInMemInfo.buffersPerFrame) {
        /* Increment the number of frames completed and clear the buffer count */
        glInMemInfo.commitBufCnt = 0;
        glInMemInfo.completeFrameCnt++;
        pAppCtxt->glfps = pAppCtxt->glfps + 1;
        pAppCtxt->glFrameSize = ((glInMemInfo.buffersPerFrame - 1) * UVC_INMEM_BUF_SIZE) + glInMemInfo.lastBufferSize;
        /* The buffer which provides a white stripe is to be inserted at an incrementing buffer index */
        if (glInMemInfo.completeFrameCnt >= glInMemInfo.whiteBufferFreq) {
            glInMemInfo.completeFrameCnt = 0;
            glInMemInfo.whiteBufferIndex++;
            if (glInMemInfo.whiteBufferIndex >= (glInMemInfo.buffersPerFrame - 1)) {
                glInMemInfo.whiteBufferIndex = 2;
            }
        }
    }

    /* Commit buffer  */
    status = Cy_HBDma_Channel_CommitBuffer(handle, &bufStat);
    if (status != CY_HBDMA_MGR_SUCCESS)
    {
        DBG_APP_ERR("UVC: HB-DMA Commit Buffer Error: %x\r\n",status);
        return;
    }
}

#endif /* UVC_INMEM_EN */

/*[]*/

