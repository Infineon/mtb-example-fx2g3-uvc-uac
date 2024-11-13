/***************************************************************************//**
* \file qspi.c
* \version 1.0
*
*  Implements the OSPI data transfers part for FPGA configuration.
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


#include "usb_qspi.h"
#include "cy_smif.h"
#include "cy_lvds.h"
#include "usb_app.h"


cy_stc_smif_context_t qspiContext;
bool glIsFPGAConfigured = false;
cy_en_smif_slave_select_t glSlaveSelectMode = CY_SMIF_SLAVE_SELECT_0;
cy_en_flash_index_t glFlashMode = SPI_FLASH_0;

/* QSPI/ SMIF Config*/
static const cy_stc_smif_config_t qspiConfig =
{
    .mode = (uint32_t)CY_SMIF_NORMAL,
    .deselectDelay = 7u,
    .rxClockSel = (uint32_t)CY_SMIF_SEL_INV_INTERNAL_CLK,
    .blockEvent = (uint32_t)CY_SMIF_BUS_ERROR,
};

/* Address to array converter*/
void Cy_SPI_AddressToArray(uint32_t value, uint8_t *byteArray,uint8_t numAddressBytes)
{
    do
    {
        numAddressBytes--;
        byteArray[numAddressBytes] = (uint8_t)(value & 0x000000FF);
        value >>= 8; /* Shift to get the next byte */
    } while (numAddressBytes > 0U);
}

/* Initialize the SPI Flash*/
cy_en_smif_status_t Cy_SPI_FlashInit (cy_en_flash_index_t flashIndex, bool qpiEnable)
{
    cy_en_smif_status_t status = CY_SMIF_SUCCESS;

    return status;
}

/* Start the QSPI/SMIF block*/
void Cy_QSPI_Start(cy_stc_usb_app_ctxt_t *pAppCtxt,cy_stc_hbdma_buf_mgr_t *hbw_bufmgr)
{
    cy_en_smif_status_t status = CY_SMIF_SUCCESS;

    /* Change QSPI Clock to 150 MHz / <DIVIDER> value */
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

/* Configure SMIF pins*/
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

/* Initialize FPGA Configuration pins*/
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

/* Configure FPGA*/
bool Cy_FPGAConfigure(cy_stc_usb_app_ctxt_t *pAppCtxt, cy_en_fpgaConfigMode_t mode)
{
    uint32_t cdoneVal = 0;
    uint32_t maxWait = 1000;

    if(mode == ACTIVE_SERIAL_MODE)
    {
        Cy_QSPI_ConfigureSMIFPins(false);

        DBG_APP_INFO("Starting Efinix Active Serial FPGA Configuration...\r\n");

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
        uint32_t bitFileSize = EFINIX_FPGA_SOC_MERGED_FILE_SIZE; /*#TODO: Replace with size read from Flash*/
#endif
        
        Cy_Debug_AddToLog(3, CYAN);
        Cy_Debug_AddToLog (3," Bit file Size %x \n\r",bitFileSize-FPGA_ADDRESS_OFFSET);
        Cy_Debug_AddToLog(3, COLOR_RESET);

        uint32_t bitFileStartAddress = 0;
        cy_en_smif_status_t status = CY_SMIF_SUCCESS;
        uint8_t dummyBuf[DUMMY_CYCLE] = {0};

        uint8_t addrArray[4];

        DBG_APP_INFO("Starting Efinix Passive Serial FPGA Configuration...\r\n");

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

        /* Convert address and add dummy byte */
        Cy_SPI_AddressToArray(bitFileStartAddress, addrArray, 4);

        status = Cy_SMIF_TransmitCommand(SMIF_HW,
                                        0x0B,                  /* Read Data Bytes Low Frequency command */
                                        CY_SMIF_WIDTH_SINGLE,
                                        addrArray,
                                        4,                     /* 3 address bytes + 1 dummy byte */
                                        CY_SMIF_WIDTH_SINGLE,
                                        glSlaveSelectMode,
                                        CY_SMIF_TX_NOT_LAST_BYTE,
                                        &qspiContext);
        
        if (status != CY_SMIF_SUCCESS)
        {
            Cy_Debug_AddToLog(1, "Error: Cy_SPI_ReadAllOperation read cmd 0x%x\r\n", status);
            return status;
        }

        Cy_LVDS_PhyGpioSet(LVDSSS_LVDS, T20_INIT_RESET_PORT, T20_INIT_RESET_PIN);    

        Cy_SysLib_Delay(6); /*Minimum time between deassertion of CRESET_N to first valid configuration data.*/

        /* Passive x1 - One clock cycle shifts out 1 bits.*/
        uint32_t cycles = MAX_DUMMY_CYCLES_COUNT;
        uint32_t remainingCycles = bitFileSize*8;

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
