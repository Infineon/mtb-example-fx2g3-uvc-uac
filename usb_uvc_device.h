/***************************************************************************//**
* \file usb_uvc_device.h
* \version 1.0
*
* Defines the constants and messages used in the FX2G3 USB Video Class
* implementation.
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

#ifndef _CY_USB_UVC_DEVICE_H_
#define _CY_USB_UVC_DEVICE_H_

#include "cy_hbdma.h"
#include "cy_hbdma_mgr.h"

#if defined(__cplusplus)
extern "C" {
#endif

/* This header file comprises of the UVC application constants and
 * the video frame configurations */

#define H_RES_640                                  (640)
#define V_RES_480                                  (480)
#define BIT_PIXEL                                  (16)

#define H_RES_1920                                   (1920)
#define V_RES_1080                                 (1080)

#define CY_USB_UVC_DEVICE_SET_FEATURE              (0x0C)
#define CY_USB_UVC_DEVICE_CLEAR_FEATURE            (0x0D)

#define CY_USB_UVC_VBUS_CHANGE_INTR                (0x0E)
#define CY_USB_UVC_VBUS_CHANGE_DEBOUNCED           (0x0F)
#define CY_USB_UVC_VIDEO_STREAMING_START           (0x10)
#define CY_USB_UVC_DEVICE_SET_CUR_RQT              (0x11)
#define CY_USB_UVC_VIDEO_STREAM_STOP_EVENT         (0x16)
#define CY_USB_PRINT_STATUS                        (0x17)

#define CY_USB_PDM_MSG_READ_COMPLETE               (0x20)
#define CY_USB_PDM_MSG_WRITE_COMPLETE              (0x21)

#define CY_USB_UVC_DEVICE_MSG_QUEUE_SIZE           (16)
#define CY_USB_UVC_DEVICE_MSG_SIZE                 (sizeof (cy_stc_usbd_app_msg_t))

#define CY_USB_MAX_DATA_BUFFER_SIZE                (4096)
#define CY_IFX_UVC_MAX_QUEUE_SIZE                  (1)

/* UVC descriptor types */
#define CY_USB_INTF_ASSN_DSCR_TYPE                 (11)        /* Interface association descriptor type. */
#define CY_USB_EP_BULK_VIDEO_PKT_SIZE              (0x400)     /* UVC video streaming endpoint packet Size */
#define CY_USB_EP_BULK_VIDEO_PKTS_COUNT            (0x01)      /* UVC video streaming endpoint packet Count */
#define CY_USB_UVC_MAX_VID_FRAMES                  (2)         /* Maximum number of video frames (4) */
#define CY_USB_UVC_STREAM_BUF_SIZE                 (61440)     /* UVC Buffer size  */
#define CY_USB_UVC_STREAM_BUF_COUNT                (4)         /* UVC Buffer count */
#define CY_USB_UVC_MAX_HEADER                      (32)        /* Maximum number of header bytes in UVC */
#define CY_USB_UVC_HEADER_DEFAULT_BFH              (0x8C)      /* Default BFH(Bit Field Header) for the UVC Header */

#define CY_USB_UVC_MAX_PROBE_SETTING               (34)        /* Maximum number of bytes in Probe Control */
#define CY_USB_UVC_MAX_PROBE_SETTING_ALIGNED       (64)        /* Maximum number of bytes in Probe Control aligned to 4 byte */

#define CY_USB_UVC_HEADER_FRAME                    (0)                     /* Normal frame indication */
#define CY_USB_UVC_HEADER_EOF                      (uint8_t)(1 << 1)       /* End of frame indication */
#define CY_USB_UVC_HEADER_FRAME_ID                 (uint8_t)(1 << 0)       /* Frame ID toggle bit */

#define CY_USB_UVC_INTERFACE_VC                    (0)                     /* Video Control interface id. */
#define CY_USB_UVC_INTERFACE_VS                    (1)                     /* Video Streaming interface id. */

#define CY_USB_UVC_SET_REQ_TYPE                    (uint8_t)(0x21)         /* UVC interface SET request type */
#define CY_USB_UVC_GET_REQ_TYPE                    (uint8_t)(0xA1)         /* UVC Interface GET request type */
#define CY_USB_UVC_GET_CUR_REQ                     (uint8_t)(0x81)         /* UVC GET_CUR request */
#define CY_USB_UVC_SET_CUR_REQ                     (uint8_t)(0x01)         /* UVC SET_CUR request */
#define CY_USB_UVC_GET_MIN_REQ                     (uint8_t)(0x82)         /* UVC GET_MIN request */
#define CY_USB_UVC_GET_MAX_REQ                     (uint8_t)(0x83)         /* UVC GET_MAX request */
#define CY_USB_UVC_GET_RES_REQ                     (uint8_t)(0x84)         /* UVC GET_RES Request */
#define CY_USB_UVC_GET_LEN_REQ                     (uint8_t)(0x85)         /* UVC GET_LEN Request */
#define CY_USB_UVC_GET_INFO_REQ                    (uint8_t)(0x86)         /* UVC GET_INFO Request */
#define CY_USB_UVC_GET_DEF_REQ                     (uint8_t)(0x87)         /* UVC GET_DEF request */

#define CY_USB_UVC_VS_PROBE_CONTROL                (0x0100)                /* Control selector for VS_PROBE_CONTROL. */
#define CY_USB_UVC_VS_COMMIT_CONTROL               (0x0200)                /* Control selector for VS_COMMIT_CONTROL. */

#define CY_USB_UVC_VC_RQT_ERROR_CODE_CONTROL       (0x0200)
#define CY_USB_UVC_RQT_STAT_INVALID_CTRL           (0x06)

/* Get the LS byte from a 16-bit number */
#define CY_GET_LSB(w)                              ((uint8_t)((w) & UINT8_MAX))

/* Get the MS byte from a 16-bit number */
#define CY_GET_MSB(w)                              ((uint8_t)((w) >> 8))

#define CY_USB_FULL_BUFFER_NO_640_480              ((H_RES_640*V_RES_480*(BIT_PIXEL/8))/(CY_USB_UVC_STREAM_BUF_SIZE-32))     
#define CY_USB_FULL_BUFFER_NO_1920_1080            ((H_RES_1920*V_RES_1080*(BIT_PIXEL/8) )/((CY_USB_UVC_STREAM_BUF_SIZE)-32))


#define USB_DESC_ATTRIBUTES __attribute__ ((section(".descSection"), used)) __attribute__ ((aligned (32)))
#define HBDMA_BUF_ATTRIBUTES __attribute__ ((section(".hbBufSection"), used)) __attribute__ ((aligned (32)))

#define UVC_STREAM_ENDPOINT                        (0x01)          /* IN endpoint used for UVC video stream. */
#define UVC_CONTROL_ENDPOINT                       (0x02)          /* IN endpoint used for UVC control interface. */
#define UAC_IN_ENDPOINT                            (0x03)          /* IN endpoint used for UAC audio stream. */

#define UVC_CONTROL_INTF_NUM                       (0x00)          /* Index of UVC control interface. */
#define UVC_STREAM_INTF_NUM                        (0x01)          /* Index of UVC streaming interface. */
#define UAC_CONTROL_INTF_NUM                       (0x02)          /* Index of UAC control interface. */
#define UAC_STREAM_INTF_NUM                        (0x03)          /* Index of UAC stream interface. */

#define CY_USB_UVC_HS_1080P_FRAME_INDEX            (1)
#define CY_USB_UVC_HS_VGA_FRAME_INDEX              (2)

#define CY_USB_UVC_PROBE_CONTROL_UPDATE_SIZE       (6) /* Format Index, Frame Index and Frame interval */

void Cy_UVC_DeviceTaskHandler(void *pTaskParam);

#if defined(__cplusplus)
}
#endif

#endif /* _CY_USB_UVC_DEVICE_H_ */

/* End of File */

