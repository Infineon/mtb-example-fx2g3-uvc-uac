/***************************************************************************//**
* \file usb_descriptors.c
* \version 1.0
*
* \brief Defines the USB descriptors used in the FX2G3 USB Device Class implementation.
*
*******************************************************************************
* \copyright
* (c) (2025), Cypress Semiconductor Corporation (an Infineon company) or
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

#include "cy_pdl.h"
#include "usb_uvc_device.h"
#include "cy_usb_common.h"
#include "cy_usbhs_cal_drv.h"
#include "cy_usb_usbd.h"
#include "usb_app.h"


/* Standard device descriptor */
USB_DESC_ATTRIBUTES uint8_t CyFxUSB20DeviceDscr[18];

const uint8_t Usb2DeviceDscr[] = {
    0x12,                           /* Descriptor size */
    0x01,                           /* Device descriptor type */
    0x00,0x02,                      /* USB 2.00 */
    0xEF,                           /* Device class: Miscellaneous. */
    0x02,                           /* Device Sub-class: Interface Association Descriptor. */
    0x01,                           /* Device protocol: Interface Association Descriptor. */
    0x40,                           /* Maxpacket size for EP0 : 64 bytes */
    0xB4,0x04,                      /* Vendor ID */
    0x03,0x49,                      /* Product ID */
    0x00,0x00,                      /* Device release number */
    0x01,                           /* Manufacture string index */
    0x02,                           /* Product string index */
    0x00,                           /* Serial number string index */
    0x01                            /* Number of configurations */
};

/* Binary device object store descriptor */
USB_DESC_ATTRIBUTES uint8_t CyFxUSBBOSDscr[64];

const uint8_t BosDescriptor[] = {
    0x05,                           /* Descriptor size */
    0x0F,                           /* Device descriptor type */
    0xC,0x00,                      /* Length of this descriptor and all sub descriptors */
    0x01,                           /* Number of device capability descriptors */

    /* USB 2.0 Extension */
    0x07,                           /* Descriptor size */
    0x10,                           /* Device capability type descriptor */
    0x02,                           /* USB 2.0 extension capability type */
    0x1E,0x64,0x00,0x00,            /* Supported device level features: LPM support, BESL supported,
                                       Baseline BESL=400 us, Deep BESL=1000 us. */

};

/* Standard device qualifier descriptor */
USB_DESC_ATTRIBUTES uint8_t CyFxUSBDeviceQualDscr[16];

const uint8_t DeviceQualDescriptor[] = {
    0x0A,                           /* descriptor size */
    0x06,                           /* Device qualifier descriptor type */
    0x00,0x02,                      /* USB 2.0 */
    0xEF,                           /* Device class: Miscellaneous. */
    0x02,                           /* Device Sub-class: Interface Association Descriptor. */
    0x01,                           /* Device protocol: Interface Association Descriptor. */
    0x40,                           /* Maxpacket size for EP0 : 64 bytes */
    0x01,                           /* Number of configurations */
    0x00                            /* Reserved */
};

/* Standard High Speed Configuration Descriptor */
USB_DESC_ATTRIBUTES uint8_t CyFxUSBHSConfigDscr[512];

const uint8_t HighSpeedConfigDescr[] = {
    /* Configuration descriptor */
    0x09,                           /* Descriptor size */
    0x02,                           /* Configuration descriptor type */
#if AUDIO_IF_EN
    0x4F,0x01,                      /* Length of this descriptor and all sub descriptors: 0x14F bytes. */
    0x04,                           /* Number of interfaces: 2 */
#else
    0xEC,0x00,                      /* Length of this descriptor and all sub descriptors: 0xEC bytes. */
    0x02,                           /* Number of interfaces: 2 */
#endif /* AUDIO_IF_EN */
    0x01,                           /* Configuration number */
    0x00,                           /* Configuration string index */
    0x80,                           /* Config characteristics - bus powered */
    0xC8,                           /* Max power consumption of device (in 2mA unit) : 400mA */

    /* Interface association descriptor */
    0x08,                           /* Descriptor size */
    0x0B,                           /* Interface association descr type */
    UVC_CONTROL_INTF_NUM,           /* I/f number of first VideoControl i/f */
    0x02,                           /* Number of video i/f */
    0x0E,                           /* CC_VIDEO : Video i/f class code */
    0x03,                           /* SC_VIDEO_INTERFACE_COLLECTION : Subclass code */
    0x00,                           /* Protocol : Not used */
    0x00,                           /* String desc index for interface */

    /* Standard video control interface descriptor */
    0x09,                           /* Descriptor size */
    0x04,                           /* Interface descriptor type */
    UVC_CONTROL_INTF_NUM,           /* Interface number */
    0x00,                           /* Alternate setting number */
    0x01,                           /* Number of end points */
    0x0E,                           /* CC_VIDEO : Interface class */
    0x01,                           /* CC_VIDEOCONTROL : Interface sub class */
    0x00,                           /* Interface protocol code */
    0x00,                           /* Interface descriptor string index */

    /* Class specific VC interface header descriptor */
    0x0D,                           /* Descriptor size */
    0x24,                           /* Class specific i/f header descriptor type */
    0x01,                           /* Descriptor sub type : VC_HEADER */
    0x10,0x01,                      /* Revision of class spec : 1.1 */
    0x51,0x00,                      /* Total size of class specific descriptors (till output terminal) */
    0x00,0x6C,0xDC,0x02,            /* Clock frequency : 48MHz */
    0x01,                           /* Number of streaming interfaces */
    0x01,                           /* Video streaming i/f 1 belongs to VC i/f */

    /* Input (camera) terminal descriptor */
    0x12,                           /* Descriptor size */
    0x24,                           /* Class specific interface desc type */
    0x02,                           /* Input Terminal Descriptor type */
    0x01,                           /* ID of this terminal */
    0x01,0x02,                      /* Camera terminal type */
    0x00,                           /* No association terminal */
    0x00,                           /* String desc index : Not used */
    0x00,0x00,                      /* No optical zoom supported */
    0x00,0x00,                      /* No optical zoom supported */
    0x00,0x00,                      /* No optical zoom supported */
    0x03,                           /* Size of controls field for this terminal : 3 bytes */
    0x00,0x00,0x00,                 /* No controls supported */

    /* Processing unit descriptor */
    0x0D,                           /* Descriptor size */
    0x24,                           /* Class specific interface desc type */
    0x05,                           /* Processing unit descriptor type */
    0x02,                           /* ID of this terminal */
    0x01,                           /* Source ID : 1 : connected to input terminal */
    0x00,0x40,                      /* Digital multiplier */
    0x03,                           /* Size of controls field for this terminal : 3 bytes */
    0x00,0x00,0x00,                 /* No controls supported */
    0x00,                           /* String desc index : Not used */
    0x00,                           /* No analog mode support. */

    /* Extension unit descriptor */
    0x1C,                           /* Descriptor size */
    0x24,                           /* Class specific interface desc type */
    0x06,                           /* Extension unit descriptor type */
    0x03,                           /* ID of this terminal */
    0xFF,0xFF,0xFF,0xFF,            /* 16 byte GUID */
    0xFF,0xFF,0xFF,0xFF,
    0xFF,0xFF,0xFF,0xFF,
    0xFF,0xFF,0xFF,0xFF,
    0x00,                           /* Number of controls in this terminal */
    0x01,                           /* Number of input pins in this terminal */
    0x02,                           /* Source ID : 2 : connected to proc unit */
    0x03,                           /* Size of controls field for this terminal : 3 bytes */
    0x00,0x00,0x00,                 /* No controls supported */
    0x00,                           /* String desc index : Not used */

    /* Output terminal descriptor */
    0x09,                           /* Descriptor size */
    0x24,                           /* Class specific interface desc type */
    0x03,                           /* Output terminal descriptor type */
    0x04,                           /* ID of this terminal */
    0x01,0x01,                      /* USB streaming terminal type */
    0x00,                           /* No association terminal */
    0x03,                           /* Source ID : 3 : Connected to extn Unit */
    0x00,                           /* String desc index : Not used */

    /* Video control status interrupt endpoint descriptor */
    0x07,                           /* Descriptor size */
    0x05,                           /* Endpoint descriptor type */
    0x80 | UVC_CONTROL_ENDPOINT,    /* Endpoint address and description */
    0x03,                           /* Interrupt end point type */
    0x40,0x00,                      /* Max packet size = 64 bytes */
    0x08,                           /* Servicing interval : 8ms */

    /* Class specific interrupt endpoint descriptor */
    0x05,                           /* Descriptor size */
    0x25,                           /* Class specific endpoint descriptor type */
    0x03,                           /* End point sub type */
    0x40,0x00,                      /* Max packet size = 64 */

    /* Standard video streaming interface descriptor (Alternate setting 0) */
    0x09,                           /* Descriptor size */
    0x04,                           /* Interface descriptor type */
    UVC_STREAM_INTF_NUM,            /* Interface number */
    0x00,                           /* Alternate setting number */
    0x01,                           /* Number of end points : One bulk */
    0x0E,                           /* Interface class : CC_VIDEO */
    0x02,                           /* Interface sub class : CC_VIDEOSTREAMING */
    0x00,                           /* Interface protocol code : Undefined */
    0x00,                           /* Interface descriptor string index */

    /* Class-specific video streaming input header descriptor */
    0x0E,                           /* Descriptor size */
    0x24,                           /* Class-specific VS i/f Type */
    0x01,                           /* Descriptor subtype : input header */
    0x01,                           /* 1 format desciptor follows */
    0x65,0x00,                      /* Total size of class specific VS descr = 0x65 bytes */
    0x80 | UVC_STREAM_ENDPOINT,     /* EP address for BULK video data */
    0x00,                           /* No dynamic format change supported */
    0x04,                           /* Output terminal ID : 4 */
    0x00,                           /* No still image capture support. */
    0x00,                           /* No hardware trigger support. */
    0x00,                           /* Hardware to initiate still image capture */
    0x01,                           /* Size of controls field : 1 byte */
    0x00,                           /* D2 : Compression quality supported */

    /* Class specific Uncompressed VS format descriptor */
    0x1B,                           /* Descriptor size */
    0x24,                           /* Class-specific VS i/f type */
    0x04,                           /* Descriptor subtype : VS_FORMAT_Uncompressed */
    0x01,                           /* Format desciptor index */
    0x02,                           /* Number of frame descriptors followed */
    0x59,0x55,0x59,0x32,            /* GUID used to identify streaming-encoding format: YUY2  */
    0x00,0x00,0x10,0x00,
    0x80,0x00,0x00,0xAA,
    0x00,0x38,0x9B,0x71,
    0x10,                           /* Number of bits per pixel */
    0x01,                           /* Optimum Frame Index for this stream: 1 (640*480) */
    0x00,                           /* X dimension of the picture aspect ratio: No interlace */
    0x00,                           /* Y dimension of the pictuer aspect ratio: No interlace */
    0x00,                           /* Interlace Flags: Progressive scanning, No interlace */
    0x00,                           /* duplication of the video stream restriction: 0 - no restriction */

    /* Class specific Uncompressed VS frame descriptor - 1 (1920*1080 p) @15 fps */
    0x1E,                           /* Descriptor size */
    0x24,                           /* Descriptor type*/
    0x05,                           /* Subtype: uncompressed frame I/F */
    CY_USB_UVC_HS_1080P_FRAME_INDEX, /* Frame Descriptor Index */
    0x00,                           /* No Still image capture supported */
    0x80,0x07,                      /* Width in pixel:  1920*/
    0x38,0x04,                      /* Height in pixel: 1080 */
    0x00, 0xC0, 0xA9, 0x1D,         /* Min bit rate (bits/s): 1920 x 1080 x 2 x 8 x 15 = 0x1DA9C000 */
    0x00, 0xC0, 0xA9, 0x1D,         /* Max bit rate (bits/s): 1920 x 1080 x 2 x 8 x 15 = 0x1DA9C000 */
    0x00, 0x48, 0x3F, 0x00,         /* Maximum video or still frame size in bytes(Deprecated): 1920 x 1080 x 2 (No Of Bytes per Pixel) = 0x3F4800*/
    0x2A, 0x2C, 0x0A, 0x00,         /* Default frame interval (in 100ns units): (1/15)x10^7 = 0x000A2C2A */
    0x01,                           /* Frame interval type : No of discrete intervals */
    0x2A, 0x2C, 0x0A, 0x00,         /* Default frame interval (in 100ns units): (1/15)x10^7 = 0x000A2C2A */
    
    /* Class specific Uncompressed VS frame descriptor (640*480 VGA) @60 fps */
    0x1E,                           /* Descriptor size */
    0x24,                           /* Descriptor type*/
    0x05,                           /* Subtype: uncompressed frame I/F */
    CY_USB_UVC_HS_VGA_FRAME_INDEX,  /* Frame Descriptor Index */
    0x00,                           /* Still image capture method not supported */
    0x80,0x02,                      /* Width in pixel:  640 */
    0xE0,0x01,                      /* Height of the frame : 480 */
    0x00,0x00,0x65,0x04,            /* Min bit rate bits/s: 640*480*2*8*60 = 0x04650000 */
    0x00,0x00,0x65,0x04,            /* Max bit rate bits/s: 640*480*2*8*60 = 0x04650000 */
    0x00,0x58,0x02,0x00,            /* Maximum video or still frame size in bytes (Deprecated)*/
    0x0A,0x8B,0x02,0x00,            /* Default frame interval (in 100ns units): (1/60)x10^7 ie 60 fps */
    0x01,                           /* Frame interval type : No of discrete intervals */
    0x0A,0x8B,0x02,0x00,            /* Frame interval = 60fps */

    /* Endpoint descriptor for streaming video data */
    0x07,                           /* Descriptor size */
    0x05,                           /* Endpoint descriptor type */
    0x80 | UVC_STREAM_ENDPOINT,     /* Endpoint address and description */
    0x02,                           /* Bulk Endpoint */
    0x00,0x02,                      /* 512 Bytes Maximum Packet Size. */
    0x00,                           /* Servicing interval for data transfers */

#if AUDIO_IF_EN
    /* Interface association descriptor */
    0x08,                           /* Descriptor size */
    0x0B,                           /* Interface association descr type */
    UAC_CONTROL_INTF_NUM,           /* I/f number of first video control i/f */
    0x02,                           /* Number of interfaces. */
    0x01,                           /* Function class: Audio */
    0x00,                           /* Subclass: Undefined. */
    0x00,                           /* Protocol : not used */
    0x00,                           /* String desc index for interface */

    /* Standard Audio Control Interface Descriptor */
    0x09,                           /* Descriptor Size */
    0x04,                           /* Interface Descriptor Type */
    UAC_CONTROL_INTF_NUM,           /* Interface number */
    0x00,                           /* Alternate setting */
    0x00,                           /* Number of endpoints - 0 endpoints */
    0x01,                           /* Interface Class - Audio */
    0x01,                           /* Interface SubClass - Audio Control */
    0x00,                           /* Interface Protocol - Unused */
    0x00,                           /* Interface string index */

    /* Class Specific Audio Control Interface Descriptor */
    0x09,                           /* Descriptor Size */
    0x24,                           /* Descriptor Type - CS_INTERFACE */
    0x01,                           /* Descriptor SubType - Header */
    0x00, 0x01,                     /* Revision of class specification - 1.0 */
    0x1E, 0x00,                     /* Total size of class specific descriptors */
    0x01,                           /* Number of streaming Interfaces - 1 */
    UAC_STREAM_INTF_NUM,            /* Audio Streaming interface belongs to this AudioControl Interface */

    /* Input terminal descriptor */
    0x0C,                           /* Descriptor size in bytes */
    0x24,                           /* CS Interface Descriptor */
    0x02,                           /* Input Terminal Descriptor subtype */
    0x01,                           /* ID Of the input terminal */
    0x01, 0x02,                     /* Microphone - terminal type */
    0x00,                           /* Association terminal - None */
#if STEREO_ENABLE
    0x02,                           /* Number of channels = 2 */
    0x03, 0x00,                     /* Spatial location of the logical channels - Left Front and Right Front */
#else
    0x01,                           /* Number of channels = 1 */
    0x01, 0x00,                     /* Spatial location of the logical channel - Left Front. */
#endif /* STEREO_ENABLE */
    0x00,                           /* Channel names - Unused */
    0x00,                           /* String index for this descriptor - None */

    /* Output terminal descriptor */
    0x09,                           /* Descriptor size */
    0x24,                           /* Class specific interface desc type */
    0x03,                           /* Output terminal descriptor type */
    0x02,                           /* ID of this terminal */
    0x01, 0x01,                     /* Output terminal type: USB Streaming */
    0x00,                           /* Association terminal - Unused */
    0x01,                           /* Id of the terminal/unit to which this is connected - Input terminal id */
    0x00,                           /* String desc index : Not used */

    /* Standard Audio Streaming Interface Descriptor (Alternate setting 0) */
    0x09,                           /* Descriptor size */
    0x04,                           /* Interface descriptor type */
    UAC_STREAM_INTF_NUM,            /* Interface number */
    0x00,                           /* Alternate setting number */
    0x00,                           /* Number of end points : zero bandwidth */
    0x01,                           /* Interface class : Audio */
    0x02,                           /* Interface sub class : Audio Streaming */
    0x00,                           /* Interface protocol code : Unused */
    0x00,                           /* Interface descriptor string index */

    /* Standard Audio Streaming Interface descriptor (Alternate setting 1) */
    0x09,                           /* Descriptor size */
    0x04,                           /* Interface descriptor type */
    UAC_STREAM_INTF_NUM,            /* Interface number */
    0x01,                           /* Alternate setting number */
    0x01,                           /* Number of end points : 1 ISO EP */
    0x01,                           /* Interface Audio class */
    0x02,                           /* Interface Audio sub class - Audio Streaming */
    0x00,                           /* Interface protocol code : Unused */
    0x00,                           /* Interface descriptor string index */

    /* Class-specific Audio Streaming General Interface descriptor */
    0x07,                           /* Descriptor size */
    0x24,                           /* Class-specific AS i/f Type */
    0x01,                           /* Descriptor subtype : AS General */
    0x02,                           /* Terminal Link - Output terminal id */
    0x01,                           /* Interface delay */
    0x01, 0x00,                     /* Audio data format - PCM */

    /* Class specific AS Format descriptor - Type I Format Descriptor */
    0x0B,                           /* Descriptor size */
    0x24,                           /* Class-specific Interface Descriptor Type */
    0x02,                           /* Format Type Descriptor subtype */
    0x01,                           /* PCM FORMAT_TYPE_I */
#if STEREO_ENABLE
    0x02,                           /* Number of channels = 2 */
#else
    0x01,                           /* Number of channels = 1 */
#endif /* STEREO_ENABLE */
    0x02,                           /* Subframe size - 2 bytes per audio subframe */
    0x10,                           /* Bit resolution - 16 bits */
    0x01,                           /* Number of samping frequencies - 1 */
    0x80, 0xBB, 0x00,               /* Sampling frequency - 48000 Hz */

    /* Endpoint descriptor for ISO streaming Audio data */
    0x09,                           /* Descriptor size */
    0x05,                           /* Endpoint descriptor type */
    0x80 | UAC_IN_ENDPOINT,         /* Endpoint address and description */
    0x05,                           /* ISO End point : Async */
#if STEREO_ENABLE
    0xC0, 0x00,                     /* Transaction size - 192 bytes */
#else
    0x60, 0x00,                     /* Transaction size - 96 bytes */
#endif /* STEREO_ENABLE */
    0x04,                           /* Servicing interval for data transfers: Once in 8 microframes */
    0x00,                           /* bRefresh */
    0x00,                           /* bSynchAddress */

    /* Class Specific AS Isochronous Audio Data Endpoint Descriptor */
    0x07,                           /* Descriptor size in bytes */
    0x25,                           /* CS_ENDPOINT descriptor type */
    0x01,                           /* EP_GENERAL sub-descriptor type */
    0x00,                           /* bmAttributes - None  */
    0x00,                           /* bLockDelayUnits - Unused */
    0x00, 0x00,                     /* wLockDelay - unused */
#endif /* AUDIO_IF_EN */
};

/* Standard device descriptor for full speed (FS) */
USB_DESC_ATTRIBUTES uint8_t CyFxUSBFSDeviceDscr[32];

const uint8_t FullSpeedDeviceDescr[] = {
    0x12,                           /* Descriptor size */
    0x01,                           /* Device descriptor type */
    0x10,0x02,                      /* USB 2.1 */
    0x00,                           /* Device class: Defined in interface */
    0x00,                           /* Device Sub-class:  Defined in interface */
    0x00,                           /* Device protocol: Defined in interface */
    0x40,                           /* Maxpacket size for EP0 */
    0xB4,0x04,                      /* Vendor ID */
    0x22,0x48,                      /* Product ID */
    0x00,0x00,                      /* Device release number */
    0x01,                           /* Manufacture string index */
    0x02,                           /* Product string index */
    0x00,                           /* Serial number string index */
    0x01                            /* Number of configurations */
};

/* Standard full speed configuration descriptor : full speed */
USB_DESC_ATTRIBUTES uint8_t CyFxUSBFSConfigDscr[64];

const uint8_t FullSpeedConfigDescr[] = {
    /* Configuration descriptor */
    0x09,                           /* Descriptor size */
    0x02,                           /* Configuration descriptor type */
    0x12,0x00,                      /* Length of this descriptor and all sub descriptors */
    0x01,                           /* Number of interfaces */
    0x01,                           /* Configuration number */
    0x03,                           /* Configuration string index: CyFxUSBFSDscr */
    0x80,                           /* Config characteristics - bus powered */
    0x32,                           /* Max power consumption of device (in 2mA unit) : 100mA */

    /* interface descriptor */
    0x09,                           /* Descriptor size */
    0x04,                           /* Interface descriptor type */
    0x00,                           /* Interface number */
    0x00,                           /* Alternate setting number */
    0x00,                           /* Number of end points */
    0xFF,                           /* Interface class: Vendor defined */
    0x00,                           /* Interface sub class: Vendor defined */
    0x00,                           /* Interface protocol: Vendor defined */
    0x00                            /* Interface descriptor string index */
};

/* Standard language ID string descriptor */
USB_DESC_ATTRIBUTES uint8_t CyFxUSBStringLangIDDscr[16];

const uint8_t LangStringDescr[] = {
    0x04,                           /* Descriptor size */
    0x03,                           /* Device descriptor type */
    0x09,0x04                       /* Language ID supported */
};

/* Standard manufacturer string descriptor */
USB_DESC_ATTRIBUTES uint8_t CyFxUSBManufactureDscr[32];

const uint8_t MfgStringDescr[] = {
    0x12,        /* Descriptor size */
    0x03,        /* Device descriptor type */
    'I',0x00,
    'N',0x00,
    'F',0x00,
    'I',0x00,
    'N',0x00,
    'E',0x00,
    'O',0x00,
    'N',0x00
};

/* Standard product string descriptor */
USB_DESC_ATTRIBUTES uint8_t CyFxUSBProductDscr[64];

const uint8_t ProdStringDescr[] = {
    0x2A, 0x03,    
    'E',  0x00,
    'Z',  0x00,
    '-',  0x00,
    'U',  0x00,
    'S',  0x00,
    'B',  0x00,
    ' ',  0x00,
    'F',  0x00,
    'X',  0x00,
    '2',  0x00,
    'G',  0x00,
    '3',  0x00,
    ' ',  0x00,
    'U',  0x00,
    'V',  0x00,
    'C',  0x00,
    '-',  0x00,
    'U',  0x00,
    'A',  0x00,
    'C',  0x00
};


/* FS Not Supported string descriptor */
USB_DESC_ATTRIBUTES uint8_t CyFxUSBFSDscr[64];

const uint8_t FullSpeedStringDescr[] = {
    0x22,        /* Descriptor Size */
    0x03,        /* Device descriptor type */
    'F',0x00,
    'S',0x00,
    ' ',0x00,
    'N',0x00,
    'O',0x00,
    'T',0x00,
    ' ',0x00,
    'S',0x00,
    'U',0x00,
    'P',0x00,
    'P',0x00,
    'O',0x00,
    'R',0x00,
    'T',0x00,
    'E',0x00,
    'D',0x00
};


/* Place this buffer as the last buffer so that no other variable / code shares
 * the same cache line. Do not add any other variables / arrays in this file.
 * This will lead to variables sharing the same cache line. */
USB_DESC_ATTRIBUTES uint8_t CyFxUsbDscrAlignBuffer[32];

void CopyDescriptorsToHBRam (void)
{
    memcpy (CyFxUSB20DeviceDscr, Usb2DeviceDscr, sizeof(Usb2DeviceDscr));
    memcpy (CyFxUSBBOSDscr, BosDescriptor, sizeof(BosDescriptor));
    memcpy (CyFxUSBDeviceQualDscr, DeviceQualDescriptor, sizeof(DeviceQualDescriptor));
    memcpy (CyFxUSBHSConfigDscr, HighSpeedConfigDescr, sizeof(HighSpeedConfigDescr));
    memcpy (CyFxUSBFSDeviceDscr, FullSpeedDeviceDescr, sizeof(FullSpeedDeviceDescr));
    memcpy (CyFxUSBFSConfigDscr, FullSpeedConfigDescr, sizeof(FullSpeedConfigDescr));
    memcpy (CyFxUSBStringLangIDDscr, LangStringDescr, sizeof(LangStringDescr));
    memcpy (CyFxUSBManufactureDscr, MfgStringDescr, sizeof(MfgStringDescr));
    memcpy (CyFxUSBProductDscr, ProdStringDescr, sizeof(ProdStringDescr));
    memcpy (CyFxUSBFSDscr, FullSpeedStringDescr, sizeof(FullSpeedStringDescr));
}

/**
 * \name Cy_USB_RegisterUsbDescriptors
 * \brief Function to register USB descriptors to USBD 
 * \param pAppCtxt application layer context pointer.
 * \param usbSpeed USB device Speed
 * \return None
 */
void Cy_USB_RegisterUsbDescriptors(cy_stc_usb_app_ctxt_t *pAppCtxt, cy_en_usb_speed_t usbSpeed)
{
    /* Can be moved so that copy is only done once. */
    CopyDescriptorsToHBRam();

    if ((pAppCtxt != NULL) && (pAppCtxt->pUsbdCtxt != NULL))  {

        /* Register USB descriptors with the stack for USB >=HS */
        if (usbSpeed == CY_USBD_USB_DEV_HS)
        {
            DBG_APP_INFO("Register HS Descriptor\r\n");
            if (pAppCtxt->devState >= CY_USB_DEVICE_STATE_ADDRESS)
                Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_HS_DEVICE_DSCR, 0, (uint8_t *)CyFxUSB20DeviceDscr);
            else
                Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_HS_DEVICE_DSCR, 0, (uint8_t *)CyFxUSBFSDeviceDscr);

            Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_HS_CONFIG_DSCR, 0, (uint8_t *)CyFxUSBHSConfigDscr);
            Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_FS_CONFIG_DSCR, 0, (uint8_t *)CyFxUSBFSConfigDscr);
            Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_STRING_DSCR, 0, (uint8_t *)CyFxUSBStringLangIDDscr);
            Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_STRING_DSCR, 1, (uint8_t *)CyFxUSBManufactureDscr);
            Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_STRING_DSCR, 2, (uint8_t *)CyFxUSBProductDscr);
            Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_DEVICE_QUAL_DSCR, 0, (uint8_t *)CyFxUSBDeviceQualDscr);
            Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_HS_BOS_DSCR, 0, (uint8_t *)CyFxUSBBOSDscr);
        }
        else {
            /* FOR USB FS */
            DBG_APP_INFO("Register FS Descriptor\r\n");
            Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_HS_DEVICE_DSCR, 0, (uint8_t *)CyFxUSBFSDeviceDscr);
            Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_FS_CONFIG_DSCR, 0, (uint8_t *)CyFxUSBFSConfigDscr);
            Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_STRING_DSCR, 0, (uint8_t *)CyFxUSBStringLangIDDscr);
            Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_STRING_DSCR, 1, (uint8_t *)CyFxUSBManufactureDscr);
            Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_STRING_DSCR, 2, (uint8_t *)CyFxUSBProductDscr);
            Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_STRING_DSCR, 3, (uint8_t *)CyFxUSBFSDscr);
            Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_HS_BOS_DSCR, 0, (uint8_t *)CyFxUSBBOSDscr);
        }
    }
}

/*[]*/

