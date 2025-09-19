/***************************************************************************//**
* \file qspi.c
* \version 1.0
*
* \brief Implements the OSPI data transfers part for FPGA configuration.
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


#include "usb_qspi.h"
#include "cy_smif.h"
#include "cy_lvds.h"
#include "usb_app.h"


cy_stc_smif_context_t qspiContext;
bool glIsFPGAConfigured = false;
cy_en_smif_slave_select_t glSlaveSelectMode = CY_SMIF_SLAVE_SELECT_0;
cy_en_flash_index_t glFlashMode = SPI_FLASH_0;
#if FLASH_AT45D
cy_en_passiveSerialMode_t glpassiveSerialMode = PASSIVE_x1;
#else
cy_en_passiveSerialMode_t glpassiveSerialMode = PASSIVE_x4;
#endif

#if !FLASH_AT45D
cy_en_smif_txfr_width_t   glCommandWidth[NUM_SPI_FLASH]     = {CY_SMIF_WIDTH_SINGLE, CY_SMIF_WIDTH_SINGLE, CY_SMIF_WIDTH_QUAD};
cy_en_smif_txfr_width_t   glReadWriteWidth[NUM_SPI_FLASH]   = {CY_SMIF_WIDTH_SINGLE, CY_SMIF_WIDTH_SINGLE, CY_SMIF_WIDTH_OCTAL};
uint8_t glSlaveSelectIndex[NUM_SPI_FLASH] = {CY_SMIF_SLAVE_SELECT_0, CY_SMIF_SLAVE_SELECT_1, (CY_SMIF_SLAVE_SELECT_0 | CY_SMIF_SLAVE_SELECT_1)};
cy_stc_externalFlashMetadata_t glFpgaFileMetadata;
#endif /* !FLASH_AT45D */

/* QSPI/ SMIF Config*/
static const cy_stc_smif_config_t qspiConfig =
{
    .mode = (uint32_t)CY_SMIF_NORMAL,
    .deselectDelay = 7u,
    .rxClockSel = (uint32_t)CY_SMIF_SEL_INV_INTERNAL_CLK,
    .blockEvent = (uint32_t)CY_SMIF_BUS_ERROR,
};

#if !FLASH_AT45D
/**
 * \name Cy_SPI_FlashReset
 * \brief Function to send reset command to selected flash
 * \param flashIndex SPI Flash Index
 * \retval status
 */
static cy_en_smif_status_t Cy_SPI_FlashReset(cy_en_flash_index_t flashIndex)
{
    cy_en_smif_status_t status = CY_SMIF_SUCCESS;

    status = Cy_SMIF_TransmitCommand(SMIF_HW,
            CY_APP_SPI_RESET_ENABLE_CMD,
            CY_SMIF_WIDTH_QUAD,
            NULL,
            CY_SMIF_CMD_WITHOUT_PARAM,
            CY_SMIF_WIDTH_NA,
            (cy_en_smif_slave_select_t)glSlaveSelectIndex[flashIndex],
            CY_SMIF_TX_LAST_BYTE,
            &qspiContext);
    ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);

    status = Cy_SMIF_TransmitCommand(SMIF_HW,
            CY_APP_SPI_SW_RESET_CMD,
            CY_SMIF_WIDTH_QUAD,
            NULL,
            CY_SMIF_CMD_WITHOUT_PARAM,
            CY_SMIF_WIDTH_NA,
            (cy_en_smif_slave_select_t)glSlaveSelectIndex[flashIndex],
            CY_SMIF_TX_LAST_BYTE,
            &qspiContext);
    ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);

    /*tRPH delay SFS256 Flash*/
    Cy_SysLib_DelayUs(50);

    return status;
}

/**
 * \name Cy_QSPI_WriteEnable
 * \brief   Function to Set write enable latch of the selected flash. 
 *          This is needed before doing program and erase operations.
 * \param flashIndex SPI Flash Index
 * \retval status
 */
static cy_en_smif_status_t Cy_QSPI_WriteEnable(cy_en_flash_index_t flashIndex)
{
    cy_en_smif_status_t status = CY_SMIF_SUCCESS;
    uint8_t statusVal = 0;
    
    if(flashIndex == DUAL_SPI_FLASH)
    {
        DBG_APP_ERR("[%s]Invalid flashIndex. Access both flash memories separately\r\n",__func__);
        return CY_SMIF_BAD_PARAM;
    }

    status = Cy_SMIF_TransmitCommand(SMIF_HW,
            CY_SPI_WRITE_ENABLE_CMD,
            glCommandWidth[flashIndex],
            NULL,
            CY_SMIF_CMD_WITHOUT_PARAM,
            CY_SMIF_WIDTH_NA,
            (cy_en_smif_slave_select_t)glSlaveSelectIndex[flashIndex],
            CY_SMIF_TX_LAST_BYTE,
            &qspiContext);
    ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);
#if !FLASH_AT45D
    /* Check if WRITE_ENABLE LATCH is set */
    status = Cy_SMIF_TransmitCommand(SMIF_HW,
            CY_SPI_STATUS_READ_CMD,
            glCommandWidth[flashIndex],
            NULL,
            CY_SMIF_CMD_WITHOUT_PARAM,
            CY_SMIF_WIDTH_NA,
            (cy_en_smif_slave_select_t)glSlaveSelectIndex[flashIndex],
            CY_SMIF_TX_NOT_LAST_BYTE,
            &qspiContext);
    ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);

    status = Cy_SMIF_ReceiveDataBlocking(SMIF_HW,
            &statusVal,
            1u,
            glReadWriteWidth[flashIndex],
            &qspiContext);
    ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);

    if(statusVal & CY_SPI_WRITE_ENABLE_LATCH_MASK)
    {
        status = CY_SMIF_SUCCESS;
    }
    else
    {
        status = CY_SMIF_BUSY;
        DBG_APP_ERR("Write Enable failed\r\n");
    }
#endif /* !FLASH_AT45D */

    return status;
}

/**
 * \name Cy_QSPI_IsMemBusy
 * \brief Function to check if Write In Progress (WIP) bit of the flash is cleared
 * \param flashIndex SPI Flash Index
 * \retval status
 */
bool Cy_QSPI_IsMemBusy(cy_en_flash_index_t flashIndex)
{
    cy_en_smif_status_t status = CY_SMIF_SUCCESS;
    uint8_t statusVal;
    if(flashIndex == DUAL_SPI_FLASH)
    {
        DBG_APP_ERR("[%s]Invalid flashIndex. Access both flash memories separately\r\n",__func__);
        return CY_SMIF_BAD_PARAM;
    }

    status = Cy_SMIF_TransmitCommand(SMIF_HW,
            CY_SPI_STATUS_READ_CMD,
            glCommandWidth[flashIndex],
            NULL,
            CY_SMIF_CMD_WITHOUT_PARAM,
            CY_SMIF_WIDTH_NA,
            (cy_en_smif_slave_select_t)glSlaveSelectIndex[flashIndex],
            CY_SMIF_TX_NOT_LAST_BYTE,
            &qspiContext);

    ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);

    status = Cy_SMIF_ReceiveDataBlocking(SMIF_HW,
            &statusVal,
            1u,
            glReadWriteWidth[flashIndex],
            &qspiContext);
    ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);

    /* If the Memory is busy, it returns true */
    return ((statusVal & 0x01) == 0x01);
}

/**
 * \name Cy_SPI_WriteConfigRegister
 * \brief Function to Read selected flash's config register
 * \param flashIndex SPI Flash Index
 * \param writeValue Value to write to register address
 * \retval status
 */
static cy_en_smif_status_t Cy_SPI_WriteConfigRegister(cy_en_flash_index_t flashIndex, uint8_t writeValue)
{
    cy_en_smif_status_t status = CY_SMIF_SUCCESS;
    uint8_t dataArray[2] = {0};

    dataArray[0] = 0; // Status Register
    dataArray[1] = writeValue;


    if(flashIndex == DUAL_SPI_FLASH)
    {
        DBG_APP_ERR("[%s]Invalid flashIndex. Access both flash memories separately\r\n",__func__);
        return CY_SMIF_BAD_PARAM;
    }

    status = Cy_QSPI_WriteEnable(flashIndex);
    Cy_SysLib_Delay(200);
    ASSERT_NON_BLOCK(CY_SMIF_SUCCESS == status, status);

    DBG_APP_INFO("Write Config Register..\r\n");
    status = Cy_SMIF_TransmitCommand(SMIF_HW,
            CY_SPI_CONFIG_REG_WRITE_CMD_SFL,
            glCommandWidth[flashIndex],
            NULL,
            CY_SMIF_CMD_WITHOUT_PARAM,
            glCommandWidth[flashIndex],
            (cy_en_smif_slave_select_t)glSlaveSelectIndex[flashIndex],
            CY_SMIF_TX_NOT_LAST_BYTE,
            &qspiContext);
    ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);
    if(CY_SMIF_SUCCESS == status)
    {
        status = Cy_SMIF_TransmitDataBlocking(SMIF0, dataArray, 2, glReadWriteWidth[flashIndex], &qspiContext);
        ASSERT_NON_BLOCK(CY_SMIF_SUCCESS == status, status);
        DBG_APP_INFO("Config Register Write: 0x%x\r\n", writeValue);
    }
    return status;
}

/**
 * \name Cy_SPI_ReadConfigRegister
 * \brief Function to read selected flash's config register
 * \param flashIndex SPI Flash Index
 * \param readValue Pointer to read data
 * \retval status
 */
static cy_en_smif_status_t Cy_SPI_ReadConfigRegister(cy_en_flash_index_t flashIndex, uint8_t *readValue)
{
    cy_en_smif_status_t status = CY_SMIF_SUCCESS;

    if((flashIndex == DUAL_SPI_FLASH) || (NULL == readValue))
    {
        DBG_APP_ERR("[%s]Invalid flashIndex. Access both flash memories separately\r\n",__func__);
        return CY_SMIF_BAD_PARAM;
    }

    DBG_APP_INFO("Read Config Register..\r\n");
    status = Cy_SMIF_TransmitCommand(SMIF_HW,
            CY_SPI_CONFIG_REG_READ_CMD,
            glCommandWidth[flashIndex],
            NULL,
            CY_SMIF_CMD_WITHOUT_PARAM,
            glCommandWidth[flashIndex],
            (cy_en_smif_slave_select_t)glSlaveSelectIndex[flashIndex],
            CY_SMIF_TX_NOT_LAST_BYTE,
            &qspiContext);
    ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);
    if(CY_SMIF_SUCCESS == status)
    {
        status =  Cy_SMIF_ReceiveDataBlocking(SMIF_HW, readValue, 1u, glReadWriteWidth[flashIndex], &qspiContext);
        ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);
        DBG_APP_INFO("Config Register Value: 0x%x\r\n", *readValue);
    }
    return status;
}

/**
 * \name Cy_QSPI_Read
 * \brief Function to perform Read operation from the flash in Register mode.
 * \param address SPI Flash register address
 * \param rxBuffer Pointer to recive data buffer
 * \param flashIndex SPI Flash Index
 * \param length length of data
 * \retval status
 */
cy_en_smif_status_t Cy_QSPI_Read(uint32_t address, uint8_t *rxBuffer, uint32_t length, cy_en_flash_index_t flashIndex)
{
    cy_en_smif_status_t status = CY_SMIF_SUCCESS;
    uint8_t addrArray[CY_APP_QSPI_NUM_ADDRESS_BYTES];
    uint8_t readCommand = CY_SPI_READ_CMD;
    
    if ((glCommandWidth[flashIndex] == CY_SMIF_WIDTH_QUAD) || 
        (glReadWriteWidth[flashIndex] == CY_SMIF_WIDTH_QUAD))
    {
        readCommand = CY_SPI_QPI_READ_CMD;
    }

    DBG_APP_INFO("Cy_QSPI_Read :: %x %d %d \n\r",readCommand,glReadWriteWidth[flashIndex],address);

    Cy_SPI_AddressToArray(address, addrArray, CY_APP_QSPI_NUM_ADDRESS_BYTES);

    status = Cy_SMIF_TransmitCommand(SMIF0,
            CY_APP_QSPI_QREAD_CMD,
            CY_SMIF_WIDTH_SINGLE,
            addrArray,
            CY_APP_QSPI_NUM_ADDRESS_BYTES,
            CY_SMIF_WIDTH_QUAD,
            glSlaveSelectMode,
            CY_SMIF_TX_NOT_LAST_BYTE,
            &qspiContext);
    ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);

    status = Cy_SMIF_TransmitCommand(SMIF0,
            CY_SPI_QREAD_MODE_CMD,
            CY_SMIF_WIDTH_QUAD,
            NULL,
            CY_SMIF_CMD_WITHOUT_PARAM,
            CY_SMIF_WIDTH_NA,
            glSlaveSelectMode,
            CY_SMIF_TX_NOT_LAST_BYTE,
            &qspiContext);
    ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);

    status = Cy_SMIF_SendDummyCycles(SMIF0, CY_APP_QSPI_QREAD_NUM_DUMMY_CYCLES_SFL);
    ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);

    if(status == CY_SMIF_SUCCESS)
    {
        status = Cy_SMIF_ReceiveDataBlocking(SMIF0, rxBuffer, length, CY_SMIF_WIDTH_QUAD, &qspiContext);
        ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);
    }
    return status;
}

#endif /* !FLASH_AT45D */

/**
 * \name Cy_SPI_AddressToArray
 * \brief Function to convert 32-nit value to byte array
 * \param value 32-bit value to be converted
 * \param byteArray Pointer to byte array
 * \param numAddressBytes Number of bytes
 * \retval status
 */
void Cy_SPI_AddressToArray(uint32_t value, uint8_t *byteArray,uint8_t numAddressBytes)
{
    do
    {
        numAddressBytes--;
        byteArray[numAddressBytes] = (uint8_t)(value & 0x000000FF);
        value >>= 8; /* Shift to get the next byte */
    } while (numAddressBytes > 0U);
}

/**
 * \name Cy_SPI_FlashInit
 * \brief Function to initialize SPI Flash
 * \details
 * Quad Mode - Data in x4 mode, Command in x1 mode
 * QPI Mode - Data in x4 mode, Command in x4 mode
 *
 * QPI enabled implies Quad enable.
 *
 * Enable only Quad mode when writes to flash can be in x1 mode and only reads need to be in x4 mode (eg: Passive x4 mode with one x4 flash memory)
 * Enable QPI mode when writes and reads should be in x4 mode (eg: Passive x8 mode with two x4 flash memories)
 *
 * \param flashIndex SPI Flash Index
 * \param quadEnable Quad Mode enable
 * \param qpiEnable QPI mode enable
 * \retval status
 */
cy_en_smif_status_t Cy_SPI_FlashInit (cy_en_flash_index_t flashIndex, bool quadEnable, bool qpiEnable)
{
    cy_en_smif_status_t status = CY_SMIF_SUCCESS;
#if !FLASH_AT45D
    uint8_t readValue = 0;

    if (qpiEnable)
    {
        quadEnable = true;
    }

    Cy_SPI_FlashReset(flashIndex);

    if(quadEnable)
    {

        Cy_SPI_ReadConfigRegister(flashIndex, &readValue);
        DBG_APP_INFO("Cy_SPI_ReadConfigRegister %x \n\r:",readValue);

        Cy_SPI_WriteConfigRegister(flashIndex, readValue | 0x02);

        glReadWriteWidth[flashIndex]  = CY_SMIF_WIDTH_QUAD;

    }
#endif /* !FLASH_AT45D */
    return status;
}

/**
 * \name Cy_QSPI_Start
 * \brief Function to start the QSPI/SMIF block
 * \param pAppCtxt application layer context pointer
 * \param hbw_bufmgr HBDMA buffer manager pointer
 * \retval status
 */
void Cy_QSPI_Start(cy_stc_usb_app_ctxt_t *pAppCtxt,cy_stc_hbdma_buf_mgr_t *hbw_bufmgr)
{
    cy_en_smif_status_t status = CY_SMIF_SUCCESS;

    /* Change QSPI Clock to 150 MHz / <DIVIDER> value */
    Cy_SysClk_ClkHfDisable(1);
    Cy_SysClk_ClkHfSetSource(1, CY_SYSCLK_CLKHF_IN_CLKPATH2);
    Cy_SysClk_ClkHfSetDivider(1, CY_SYSCLK_CLKHF_NO_DIVIDE);
    Cy_SysClk_ClkHfEnable(1);

    /*Initialize SMIF Pins for QSPI*/
    Cy_QSPI_ConfigureSMIFPins(true);

    status = Cy_SMIF_Init(SMIF_HW, &qspiConfig, 10000u, &qspiContext);
    ASSERT(CY_SMIF_SUCCESS == status, status);

    Cy_SMIF_SetDataSelect(SMIF_HW, CY_SMIF_SLAVE_SELECT_0, CY_SMIF_DATA_SEL0);
    Cy_SMIF_SetDataSelect(SMIF_HW, CY_SMIF_SLAVE_SELECT_1, CY_SMIF_DATA_SEL2);

    Cy_SMIF_Enable(SMIF_HW, &qspiContext);

    DBG_APP_INFO("Cy_USB_QSPIEnabled \n\r:");
}

/**
 * \name Cy_QSPI_ConfigureSMIFPins
 * \brief Function to configure SMIF pins
 * \param init initialize SMIF pins if true
 * \retval void
 */
void Cy_QSPI_ConfigureSMIFPins(bool init)
{
    cy_en_gpio_status_t status = CY_GPIO_SUCCESS;
    cy_stc_gpio_pin_config_t pinCfg;

    memset((void *)&pinCfg, 0, sizeof(pinCfg));

    if(init)
    {
        pinCfg.driveMode = CY_GPIO_DM_STRONG_IN_OFF;
        /* Configure P6.0 as QSPI Clock */
        pinCfg.hsiom = SMIF_CLK_HSIOM;
    }
    else
    {
        pinCfg.driveMode = CY_GPIO_DM_HIGHZ;
        /* Configure P6.0 as floating GPIO */
        pinCfg.hsiom = P6_0_GPIO;
    }
    status = Cy_GPIO_Pin_Init(SMIF_CLK_PORT, SMIF_CLK_PIN, &pinCfg);
    ASSERT_NON_BLOCK(CY_GPIO_SUCCESS == status, status);

    if(init)
    {
        /* Configure P6.1 as QSPI Select 0 */
        pinCfg.outVal = 1;
        pinCfg.driveMode = CY_GPIO_DM_STRONG_IN_OFF;
        pinCfg.hsiom = SMIF_SELECT0_HSIOM;
    }
    else
    {
        pinCfg.driveMode = CY_GPIO_DM_HIGHZ;
        /* Configure P6.1 as floating GPIO */
        pinCfg.hsiom = P6_1_GPIO;
    }

    status = Cy_GPIO_Pin_Init(SMIF_SELECT0_PORT, SMIF_SELECT0_PIN, &pinCfg);
    ASSERT_NON_BLOCK(CY_GPIO_SUCCESS == status, status);


    if(init)
    {
        /* Configure P6.2 as QSPI Select 1 */
        pinCfg.outVal = 1;
        pinCfg.driveMode = CY_GPIO_DM_STRONG_IN_OFF;
        pinCfg.hsiom = SMIF_SELECT1_HSIOM;
    }
    else
    {
        pinCfg.driveMode = CY_GPIO_DM_HIGHZ;
        /* Configure P6.2 as floating GPIO */
        pinCfg.hsiom = P6_2_GPIO;
    }
    status = Cy_GPIO_Pin_Init(SMIF_SELECT1_PORT, SMIF_SELECT1_PIN, &pinCfg);
    ASSERT_NON_BLOCK(CY_GPIO_SUCCESS == status, status);

    if(init)
    {
        pinCfg.outVal = 0;
        /* Configure P7.0 as QSPI Data 0 */
        pinCfg.driveMode = CY_GPIO_DM_STRONG;
        pinCfg.hsiom = SMIF_DATA0_HSIOM;
    }
    else
    {
        pinCfg.driveMode = CY_GPIO_DM_HIGHZ;
        /* Configure P7.0 as floating GPIO */
        pinCfg.hsiom = P7_0_GPIO;
    }
    status = Cy_GPIO_Pin_Init(SMIF_DATA0_PORT, SMIF_DATA0_PIN, &pinCfg);
    ASSERT_NON_BLOCK(CY_GPIO_SUCCESS == status, status);


    if(init)
    {
        pinCfg.outVal = 0;
        /* Configure P7.1 as QSPI Data 1 */
        pinCfg.driveMode = CY_GPIO_DM_STRONG;
        pinCfg.hsiom = SMIF_DATA1_HSIOM;
    }
    else
    {
        pinCfg.driveMode = CY_GPIO_DM_HIGHZ;
        /* Configure P7.1 as floating GPIO */
        pinCfg.hsiom = P7_1_GPIO;
    }
    status = Cy_GPIO_Pin_Init(SMIF_DATA1_PORT, SMIF_DATA1_PIN, &pinCfg);
    ASSERT_NON_BLOCK(CY_GPIO_SUCCESS == status, status);


    if(init)
    {
        pinCfg.outVal = 0;
        /* Configure P7.2 as QSPI Data 2*/
        pinCfg.driveMode = CY_GPIO_DM_STRONG;
        pinCfg.hsiom = SMIF_DATA2_HSIOM;
    }
    else
    {
        pinCfg.driveMode = CY_GPIO_DM_HIGHZ;
        /* Configure P7.2 as floating GPIO */
        pinCfg.hsiom = P7_2_GPIO;
    }
    status = Cy_GPIO_Pin_Init(SMIF_DATA2_PORT, SMIF_DATA2_PIN, &pinCfg);
    ASSERT_NON_BLOCK(CY_GPIO_SUCCESS == status, status);


    if(init)
    {
        pinCfg.outVal = 0;
        /* Configure P7.3 as QSPI Data 3*/
        pinCfg.driveMode = CY_GPIO_DM_STRONG;
        pinCfg.hsiom = SMIF_DATA3_HSIOM;
    }
    else
    {
        pinCfg.driveMode = CY_GPIO_DM_HIGHZ;
        /* Configure P7.3 as floating GPIO */
        pinCfg.hsiom = P7_3_GPIO;
    }
    status = Cy_GPIO_Pin_Init(SMIF_DATA3_PORT, SMIF_DATA3_PIN, &pinCfg);
    ASSERT_NON_BLOCK(CY_GPIO_SUCCESS == status, status);


    if(init)
    {
        pinCfg.outVal = 0;
        /* Configure P7.4 as QSPI Data 4 */
        pinCfg.driveMode = CY_GPIO_DM_STRONG;
        pinCfg.hsiom = SMIF_DATA4_HSIOM;
    }
    else
    {
        pinCfg.driveMode = CY_GPIO_DM_HIGHZ;
        /* Configure P7.4 as floating GPIO */
        pinCfg.hsiom = P7_4_GPIO;
    }
    status = Cy_GPIO_Pin_Init(SMIF_DATA4_PORT, SMIF_DATA4_PIN, &pinCfg);
    ASSERT_NON_BLOCK(CY_GPIO_SUCCESS == status, status);


    if(init)
    {
        pinCfg.outVal = 0;
        /* Configure P7.5 as QSPI Data 5 */
        pinCfg.driveMode = CY_GPIO_DM_STRONG;
        pinCfg.hsiom = SMIF_DATA5_HSIOM;
    }
    else
    {
        pinCfg.driveMode = CY_GPIO_DM_HIGHZ;
        /* Configure P7.5 as floating GPIO */
        pinCfg.hsiom = P7_5_GPIO;
    }
    status = Cy_GPIO_Pin_Init(SMIF_DATA5_PORT, SMIF_DATA5_PIN, &pinCfg);
    ASSERT_NON_BLOCK(CY_GPIO_SUCCESS == status, status);


    if(init)
    {
        pinCfg.outVal = 0;
        /* Configure P7.6 as QSPI Data 6 */
        pinCfg.driveMode = CY_GPIO_DM_STRONG;
        pinCfg.hsiom = SMIF_DATA6_HSIOM;
    }
    else
    {
        pinCfg.driveMode = CY_GPIO_DM_HIGHZ;
        /* Configure P7.6 as floating GPIO */
        pinCfg.hsiom = P7_6_GPIO;
    }
    status = Cy_GPIO_Pin_Init(SMIF_DATA6_PORT, SMIF_DATA6_PIN, &pinCfg);
    ASSERT_NON_BLOCK(CY_GPIO_SUCCESS == status, status);


    if(init)
    {
        pinCfg.outVal = 0;
        /* Configure P7.7 as QSPI Data 7 */
        pinCfg.driveMode = CY_GPIO_DM_STRONG;
        pinCfg.hsiom = SMIF_DATA7_HSIOM;
    }
    else
    {
        pinCfg.driveMode = CY_GPIO_DM_HIGHZ;
        /* Configure P7.7 as floating GPIO */
        pinCfg.hsiom = P7_7_GPIO;
    }
    status = Cy_GPIO_Pin_Init(SMIF_DATA7_PORT, SMIF_DATA7_PIN, &pinCfg);
    ASSERT_NON_BLOCK(CY_GPIO_SUCCESS == status, status);

}

/**
 * \name Cy_FPGAConfigPins
 * \brief Function to initialize FPGA configuration pins
 * \param pAppCtxt application layer context pointer
 * \param mode fpga configuration mode
 * \retval void
 */
void Cy_FPGAConfigPins(cy_stc_usb_app_ctxt_t *pAppCtxt, cy_en_fpgaConfigMode_t mode)
{
    cy_en_gpio_status_t status = CY_GPIO_SUCCESS;
    cy_stc_gpio_pin_config_t pinCfg;
    memset((void *)&pinCfg, 0, sizeof(pinCfg));

    /* Configure output GPIOs. */
    pinCfg.driveMode = CY_GPIO_DM_STRONG_IN_OFF;
    pinCfg.hsiom = HSIOM_SEL_GPIO;

    pinCfg.hsiom     = P8_4_GPIO;
    pinCfg.driveMode = CY_GPIO_DM_STRONG_IN_OFF;
    status = Cy_GPIO_Pin_Init(P8_4_PORT, P8_4_PIN, &pinCfg);
    ASSERT_NON_BLOCK(status == CY_GPIO_SUCCESS, status);

    pinCfg.hsiom     = P8_5_GPIO;
    pinCfg.driveMode = CY_GPIO_DM_STRONG_IN_OFF;
    status = Cy_GPIO_Pin_Init(P8_5_PORT, P8_5_PIN, &pinCfg);
    ASSERT_NON_BLOCK(status == CY_GPIO_SUCCESS, status);

    Cy_LVDS_PhyGpioModeEnable(LVDSSS_LVDS, T20_INIT_RESET_PORT,T20_INIT_RESET_PIN,
                                CY_LVDS_PHY_GPIO_OUTPUT, CY_LVDS_PHY_GPIO_NO_INTERRUPT);
    Cy_LVDS_PhyGpioClr(LVDSSS_LVDS, T20_INIT_RESET_PORT, T20_INIT_RESET_PIN);

    Cy_LVDS_PhyGpioModeEnable(LVDSSS_LVDS, T20_SSN_PORT, T20_SSN_PIN,
                                CY_LVDS_PHY_GPIO_OUTPUT, CY_LVDS_PHY_GPIO_NO_INTERRUPT);
    if(mode == ACTIVE_SERIAL_MODE)
    {
        /*FPGA's SSL_N should be HIGH for Active Serial*/
        Cy_LVDS_PhyGpioSet(LVDSSS_LVDS, T20_SSN_PORT, T20_SSN_PIN);
    }
    else
    {
        Cy_LVDS_PhyGpioClr(LVDSSS_LVDS, T20_SSN_PORT, T20_SSN_PIN);
    }

    Cy_LVDS_PhyGpioModeEnable(LVDSSS_LVDS, T20_PROGRAM_N_PORT,T20_PROGRAM_N_PIN,
                                CY_LVDS_PHY_GPIO_OUTPUT, CY_LVDS_PHY_GPIO_NO_INTERRUPT);
    Cy_LVDS_PhyGpioSet(LVDSSS_LVDS, T20_PROGRAM_N_PORT, T20_PROGRAM_N_PIN);

#if CONFIG_CDONE_AS_INPUT
    Cy_LVDS_PhyGpioModeEnable(LVDSSS_LVDS, T20_CDONE_PORT,T20_CDONE_PIN,
                                    CY_LVDS_PHY_GPIO_INPUT, CY_LVDS_PHY_GPIO_NO_INTERRUPT);

#else

    Cy_LVDS_PhyGpioModeEnable(LVDSSS_LVDS, T20_CDONE_PORT,T20_CDONE_PIN,
                                CY_LVDS_PHY_GPIO_OUTPUT, CY_LVDS_PHY_GPIO_NO_INTERRUPT);
    Cy_LVDS_PhyGpioClr(LVDSSS_LVDS, T20_CDONE_PORT, T20_CDONE_PIN);

#endif
}

/**
 * \name Cy_FPGAConfigure
 * \brief Function to initialize initiate FPGA configuration
 * \param pAppCtxt application layer context pointer
 * \param mode fpga configuration mode
 * \retval void
 */
bool Cy_FPGAConfigure(cy_stc_usb_app_ctxt_t *pAppCtxt, cy_en_fpgaConfigMode_t mode)
{
    uint32_t cdoneVal = 0;
    uint32_t maxWait = 1000;

    if(mode == ACTIVE_SERIAL_MODE)
    {
        Cy_QSPI_ConfigureSMIFPins(false);

        DBG_APP_INFO("Starting Active Serial FPGA Configuration\r\n");

        Cy_LVDS_PhyGpioSet(LVDSSS_LVDS, T20_INIT_RESET_PORT, T20_INIT_RESET_PIN);

        Cy_SysLib_Delay(200);

        cdoneVal = false;
        while (cdoneVal == false)
        {
            /*Check if CDONE is LOW*/
            cdoneVal = Cy_LVDS_PhyGpioRead(LVDSSS_LVDS, T20_CDONE_PORT, T20_CDONE_PIN);

            Cy_SysLib_Delay(1);
            maxWait--;
            if (!maxWait)
            {
                break;
            }
        }

        if (cdoneVal)
        {
            glIsFPGAConfigured = true;
            Cy_Debug_AddToLog(3, CYAN);
            Cy_Debug_AddToLog (3,"FPGA Configuration Done \n\r");
            Cy_Debug_AddToLog(3, COLOR_RESET);
        }
        else
        {
            /*Keep the FPGA in reset if the config fails. This is to prevent the FPGA
             *from taking control of the SPI lines which prevents FX2G3's access to QSPI Flash */
            Cy_LVDS_PhyGpioClr(LVDSSS_LVDS, T20_INIT_RESET_PORT, T20_INIT_RESET_PIN);        
            Cy_Debug_AddToLog(1, CYAN);
            Cy_Debug_AddToLog (1,"FPGA Configuration Failed! \n\r");
            Cy_Debug_AddToLog(1, COLOR_RESET);
        }

        /*Re-init SMIF pins. This should not affect the RISC-V boot
         *also as there is sufficient delay of 2 seconds*/
        Cy_QSPI_ConfigureSMIFPins(true);
    }
    else
    {
#if CONFIG_CDONE_AS_INPUT
        uint32_t bitFileSize = EFINIX_MAX_CONFIG_FILE_SIZE;
#else
        uint32_t bitFileSize = EFINIX_FPGA_SOC_MERGED_FILE_SIZE;
#endif
        
        Cy_Debug_AddToLog(3, CYAN);
        Cy_Debug_AddToLog (3," Bit file Size %x \n\r",bitFileSize-FPGA_ADDRESS_OFFSET);
        Cy_Debug_AddToLog(3, COLOR_RESET);

        uint32_t bitFileStartAddress = 0;
        cy_en_smif_status_t status = CY_SMIF_SUCCESS;
        uint8_t dummyBuf[DUMMY_CYCLE] = {0};

        uint8_t addrArray[CY_APP_QSPI_NUM_ADDRESS_BYTES];
        Cy_SPI_AddressToArray(bitFileStartAddress, addrArray, CY_APP_QSPI_NUM_ADDRESS_BYTES);

        DBG_APP_INFO("Starting Passive Serial FPGA Configuration\r\n");

        Cy_LVDS_PhyGpioClr(LVDSSS_LVDS, T20_INIT_RESET_PORT, T20_INIT_RESET_PIN);    


        Cy_SysLib_Delay(2);

#if CONFIG_CDONE_AS_INPUT
        cdoneVal = false;
        maxWait = 1000;
        while (cdoneVal == false)
        {
            /*Check if CDONE is LOW*/
            cdoneVal = Cy_LVDS_PhyGpioRead(LVDSSS_LVDS, T20_CDONE_PORT, T20_CDONE_PIN);
            Cy_SysLib_Delay(1);
            maxWait--;
            if (!maxWait)
            {
                break;
            }
        }
#else
        Cy_LVDS_PhyGpioClr(LVDSSS_LVDS, T20_CDONE_PORT, T20_CDONE_PIN);
#endif

#if FLASH_AT45D
        /* Convert address and add dummy byte */
        Cy_SPI_AddressToArray(bitFileStartAddress, addrArray, 4);

        status = Cy_SMIF_TransmitCommand(SMIF_HW,
                                        CY_APP_SPI_READ_CMD,                  /* Read Data Bytes Low Frequency command */
                                        CY_SMIF_WIDTH_SINGLE,
                                        addrArray,
                                        CY_APP_QSPI_NUM_ADDRESS_BYTES,        /* 3 address bytes + 1 dummy byte */
                                        CY_SMIF_WIDTH_SINGLE,
                                        glSlaveSelectMode,
                                        CY_SMIF_TX_NOT_LAST_BYTE,
                                        &qspiContext);
        
        if (status != CY_SMIF_SUCCESS)
        {
            Cy_Debug_AddToLog(1, "Error: Cy_SPI_ReadAllOperation read cmd 0x%x\r\n", status);
            return status;
        }
#else
        status = Cy_SMIF_TransmitCommand(SMIF0,
                                CY_APP_QSPI_QREAD_CMD,
                                CY_SMIF_WIDTH_SINGLE,
                                addrArray,
                                CY_APP_QSPI_NUM_ADDRESS_BYTES,
                                CY_SMIF_WIDTH_QUAD,
                                glSlaveSelectMode,
                                CY_SMIF_TX_NOT_LAST_BYTE,
                                &qspiContext);
        ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);

        status = Cy_SMIF_TransmitCommand(SMIF0,
                                CY_APP_QSPI_QREAD_MODE_CMD,
                                CY_SMIF_WIDTH_QUAD,
                                NULL,
                                CY_SMIF_CMD_WITHOUT_PARAM,
                                CY_SMIF_WIDTH_NA,
                                glSlaveSelectMode,
                                CY_SMIF_TX_NOT_LAST_BYTE,
                                &qspiContext);
        ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);

        status = Cy_SMIF_SendDummyCycles(SMIF0, CY_APP_QSPI_QREAD_NUM_DUMMY_CYCLES_SFL);
        ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);
#endif /*FLASH_AT45D */

        Cy_LVDS_PhyGpioSet(LVDSSS_LVDS, T20_INIT_RESET_PORT, T20_INIT_RESET_PIN);   

        Cy_SysLib_Delay(6); /*Minimum time between deassertion of CRESET_N to first valid configuration data.*/

        uint32_t cycles = MAX_DUMMY_CYCLES_COUNT;
        uint32_t remainingCycles = (pAppCtxt->glpassiveSerialMode == PASSIVE_x4)? bitFileSize*2 : bitFileSize*8;

        while (remainingCycles > 0)
        {
            cycles = CY_USB_MIN(remainingCycles, MAX_DUMMY_CYCLES_COUNT);
            if (Cy_SMIF_GetCmdFifoStatus(SMIF0) < 3)
            {
                status = Cy_SMIF_SendDummyCycles(SMIF0, cycles);
                ASSERT_NON_BLOCK(status == CY_SMIF_SUCCESS, status);
                remainingCycles -= cycles;
            }
        }  
        /* Extra clocks to get the FPGA into user mode */
        Cy_SMIF_TransmitDataBlocking(SMIF0, dummyBuf, DUMMY_CYCLE, CY_SMIF_WIDTH_SINGLE, &qspiContext);

#if CONFIG_CDONE_AS_INPUT
        cdoneVal = false;
        maxWait = 1000;
        while (cdoneVal == false)
        {
            /* Check if CDONE is LOW */
            cdoneVal = Cy_LVDS_PhyGpioRead(LVDSSS_LVDS, T20_CDONE_PORT, T20_CDONE_PIN);
            Cy_SysLib_Delay(1);
            maxWait--;
            if (!maxWait)
            {
                break;
            }
        }
#else
        cy_stc_gpio_pin_config_t pinCfg;
        memset((void *)&pinCfg, 0, sizeof(pinCfg));
        pinCfg.driveMode = CY_GPIO_DM_HIGHZ;
        pinCfg.hsiom = HSIOM_SEL_GPIO;

        Cy_App_QSPIConfigureSMIFPins(false);
#endif

        maxWait = 1000;
        cdoneVal = false;
        while (cdoneVal == false)
        { 
            /* Check if CDONE is LOW */
            cdoneVal = Cy_LVDS_PhyGpioRead(LVDSSS_LVDS, T20_CDONE_PORT, T20_CDONE_PIN);
            Cy_SysLib_Delay(1);
            maxWait--;
            if (!maxWait)
            {
                break;
            }
        }


        if (cdoneVal)
        {
            glIsFPGAConfigured = true;
            Cy_Debug_AddToLog(3, CYAN);
            Cy_Debug_AddToLog (3,"FPGA Configuration Done \n\r");
            Cy_Debug_AddToLog(3, COLOR_RESET);
            DBG_APP_INFO("FPGA Soft Reset Done\r\n");
        }
        else
        {
            Cy_Debug_AddToLog(1, RED);
            Cy_Debug_AddToLog (1,"FPGA Configuration Failed \n\r");
            Cy_Debug_AddToLog(1, COLOR_RESET);
        }

#if (!CONFIG_CDONE_AS_INPUT)
        DELAY_MILLI(3000);
        /*Re-init SMIF pins. This should not affect the RISC-V boot
         *also as there is sufficient delay of 3 seconds and SOC SPI activity was seen for <2 secs*/
        Cy_App_QSPIConfigureSMIFPins(true);
#endif
    }
    return glIsFPGAConfigured;
}
