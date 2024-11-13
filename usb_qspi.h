/***************************************************************************//**
* \file qspi.h
* \version 1.0
*
* Provided API declarations for the QSPI implementation. 
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
***************************************************************************/



#ifndef _QSPI_H_
#define _QSPI_H_

#include "cy_pdl.h"
#include "usb_i2c.h"
#include "cy_debug.h"
#include "usb_app.h"


/*****************************************************************************
*                          SMIF enums
******************************************************************************/
typedef enum cy_en_spiFlashType_t
{
    IFX_SFL = 0x80,
    IFX_SFS = 0x81,
    FLASH_UNKNOWN = 0xFF
}cy_en_spiFlashType_t;

typedef enum cy_en_passiveSerialMode_t
{
    PASSIVE_x4,
    PASSIVE_x8
}cy_en_passiveSerialMode_t;

typedef enum cy_en_flash_index_t
{
  SPI_FLASH_0    = 0,
  SPI_FLASH_1    = 1,
  DUAL_SPI_FLASH = 2,
  NUM_SPI_FLASH,
}cy_en_flash_index_t;

typedef struct cy_stc_cfi_erase_block_info_t
{
  uint32_t numSectors;
  uint32_t sectorSize;
  uint32_t startingAddress;
  uint32_t lastAddress;
} cy_stc_cfi_erase_block_info_t ;



/*****************************************************************************
*                          SMIF macros                                       *
******************************************************************************/

#define FPGA_CONFIG_MODE           (PASSIVE_SERIAL_MODE)

#if (FPGA_CONFIG_MODE == PASSIVE_SERIAL_MODE)
/*
 * CDONE is an I/O pin for the FPGA. It can be configurd as a output and pulled low to delay FPGA's User Mode entry.
 * This approach can be used if the FPGA also has a softcore SoC implementation and the SoC has to boot from SPI Flash.
 * This will create a scenario where there will be two SPI masters trying to access the same SPI Flash.
 * After sending the FPGA bin file, FX2G3 will keep CDONE low to hold FPGA in config mode.
 * Then FX2G3 will release control of the SPI pins (High-Z) and make CDONE also High-Z to allow FPGA to enter user mode and proceed to SoC boot.
 *
 * Choose CONFIG_CDONE_AS_INPUT=0 for suppporting FPGA implementations with SoC (CIS_ISP).
 * Choose CONFIG_CDONE_AS_INPUT=1 for only FPGA bin files.
*/
#define EFINIX_FPGA_SOC_MERGED_FILE_SIZE        (3756522)
#define CONFIG_CDONE_AS_INPUT                   (1)
#else
#define CONFIG_CDONE_AS_INPUT                   (1)
#endif


#define SMIF_HW SMIF0

#define SMIF_CLK_HSIOM                          (P6_0_SMIF_SPI_CLK)
#define SMIF_CLK_PORT                           (P6_0_PORT)
#define SMIF_CLK_PIN                            (P6_0_PIN)

#define SMIF_SELECT0_HSIOM                      (P6_1_SMIF_SPI_SELECT0)
#define SMIF_SELECT0_PORT                       (P6_1_PORT)
#define SMIF_SELECT0_PIN                        (P6_1_PIN)

#define SMIF_SELECT1_HSIOM                      (P6_2_SMIF_SPI_SELECT1)
#define SMIF_SELECT1_PORT                       (P6_2_PORT)
#define SMIF_SELECT1_PIN                        (P6_2_PIN)

#define SMIF_DATA0_HSIOM                        (P7_0_SMIF_SPI_DATA0)
#define SMIF_DATA0_PORT                         (P7_0_PORT)
#define SMIF_DATA0_PIN                          (P7_0_PIN)

#define SMIF_DATA1_HSIOM                        (P7_1_SMIF_SPI_DATA1)
#define SMIF_DATA1_PORT                         (P7_1_PORT)
#define SMIF_DATA1_PIN                          (P7_1_PIN)

#define SMIF_DATA2_HSIOM                        (P7_2_SMIF_SPI_DATA2)
#define SMIF_DATA2_PORT                         (P7_2_PORT)
#define SMIF_DATA2_PIN                          (P7_2_PIN)

#define SMIF_DATA3_HSIOM                        (P7_3_SMIF_SPI_DATA3)
#define SMIF_DATA3_PORT                         (P7_3_PORT)
#define SMIF_DATA3_PIN                          (P7_3_PIN)

#define SMIF_DATA4_HSIOM                        (P7_4_SMIF_SPI_DATA4)
#define SMIF_DATA4_PORT                         (P7_4_PORT)
#define SMIF_DATA4_PIN                          (P7_4_PIN)

#define SMIF_DATA5_HSIOM                        (P7_5_SMIF_SPI_DATA5)
#define SMIF_DATA5_PORT                         (P7_5_PORT)
#define SMIF_DATA5_PIN                          (P7_5_PIN)

#define SMIF_DATA6_HSIOM                        (P7_6_SMIF_SPI_DATA6)
#define SMIF_DATA6_PORT                         (P7_6_PORT)
#define SMIF_DATA6_PIN                          (P7_6_PIN)

#define SMIF_DATA7_HSIOM                        (P7_7_SMIF_SPI_DATA7)
#define SMIF_DATA7_PORT                         (P7_7_PORT)
#define SMIF_DATA7_PIN                          (P7_7_PIN)

#define T20_CDONE_PORT                          (1)
#define T20_CDONE_PIN                           (cy_en_lvds_phy_gpio_index_t)(16+9)

#define T20_INIT_RESET_PORT                     (1)
#define T20_INIT_RESET_PIN                      (cy_en_lvds_phy_gpio_index_t)(16+8)

#define T20_PROGRAM_N_PORT                      (1)
#define T20_PROGRAM_N_PIN                       (cy_en_lvds_phy_gpio_index_t)(9)

#define T20_SSN_PORT                            (1)
#define T20_SSN_PIN                             (cy_en_lvds_phy_gpio_index_t)(0)

#define MAX_DUMMY_CYCLES_COUNT                  (8192)
#define EFINIX_MAX_CONFIG_FILE_SIZE             (6262080)

#define FPGA_ADDRESS_OFFSET                     (5)
#define DUMMY_CYCLE                             (150)

extern cy_stc_smif_context_t qspiContext;


/*****************************************************************************
* Function Name:Cy_FPGAConfigPins(cy_stc_usb_app_ctxt_t *pAppCtxt
                                , cy_en_fpgaConfigMode_t mode)
******************************************************************************
* Summary:
* Function to Configure pins for FPGA
*
* Parameters:
*  \param pAppCtxt
*   User Context
*  \param mode
*   FPGA mode
*
* Return:
*  Does not return.
*****************************************************************************/
void Cy_FPGAConfigPins(cy_stc_usb_app_ctxt_t *pAppCtxt, cy_en_fpgaConfigMode_t mode);

/*****************************************************************************
* Function Name:Cy_FPGAConfigure(cy_stc_usb_app_ctxt_t *pAppCtxt
                                , cy_en_fpgaConfigMode_t mode)
******************************************************************************
* Summary:
* Fucntion to configure FPGA
*
* Parameters:
*  \param pAppCtxt
*   User Context
*  \param mode
*   FPGA mode
*
* Return:
*  Does not return.
*****************************************************************************/
bool Cy_FPGAConfigure(cy_stc_usb_app_ctxt_t *pAppCtxt,cy_en_fpgaConfigMode_t mode);


/*****************************************************************************
* Function Name:Cy_QSPI_ConfigureSMIFPins(bool init)
******************************************************************************
* Summary:
* Function to Configure SMIF pins
*
* Parameters:
*  \param pAppCtxt
*   User Context
*  \param init
*   Initialize
*
* Return:
*  Does not return.
*****************************************************************************/
void Cy_QSPI_ConfigureSMIFPins(bool init);


/*****************************************************************************
* Function Name:Cy_QSPI_Start(cy_stc_usb_app_ctxt_t *pUsbApp,
                                cy_stc_hbdma_buf_mgr_t *hbw_bufmgr)
******************************************************************************
* Summary:
* Function to start QSPI block
*
* Parameters:
*  \param pUsbApp
*   User Context
*  \param hbw_bufmgr
*   Pointer to DMA buffer
*
* Return:
*  Does not return.
*****************************************************************************/
void Cy_QSPI_Start(cy_stc_usb_app_ctxt_t *pUsbApp,cy_stc_hbdma_buf_mgr_t *hbw_bufmgr);


/*****************************************************************************
* Function Name:Cy_SPI_AddressToArray(uint32_t value, uint8_t *byteArray, 
                            uint8_t numAddressBytes)
******************************************************************************
* Summary:
* Function to change address to array
*
* Parameters:
*   \param value
*   Value to convert to array
*  \param byteArray
*   Pointer to array
*  \param numAddressBytes
*   Number of bytes in address
*
* Return:
*  returns data
*****************************************************************************/
void Cy_SPI_AddressToArray(uint32_t value, uint8_t *byteArray, uint8_t numAddressBytes);


/*****************************************************************************
* Function Name:Cy_SPI_FlashInit(cy_en_flash_index_t flashIndex, bool qpiEnable)
******************************************************************************
* Summary:
* Function to handler QSPI Vendor commands
*
* Parameters:
*   \param flashIndex
*   Flash Index
*   \param qpiEnable
*   Enable QPI Mode
*
* Return:
*  returns cy_en_smif_status_t
*****************************************************************************/
cy_en_smif_status_t Cy_SPI_FlashInit (cy_en_flash_index_t flashIndex, bool qpiEnable);

#endif /*_QSPI_H_*/
