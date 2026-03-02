/*******************************************************************************
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

#ifndef   _GPIF_HEADER_LVCMOS_H_
#define   _GPIF_HEADER_LVCMOS_H_
#include "cy_lvds.h"


#if defined(__cplusplus)
extern "C" {
#endif

/* Summary
   Number of states in the state machine */
#define CY_NUMBER_OF_STATES 7
#define START 0
#define IDLE 1
#define READ 2
#define WRITE 3
#define SLP 4
#define ZLP 5
#define DSS 6
/* Summary
   Initial value of early outputs from the state machine */
#define ALPHA_START 0x0C


const cy_stc_lvds_gpif_reg_data_t cy_lvds_gpif0_reg_data[] = {
{0x00000000, 0x000400E0}, /* GPIF_CONFIG */
{0x00000004, 0x00002020}, /* GPIF_BUS_CONFIG */
{0x00000008, 0x01040002}, /* GPIF_BUS_CONFIG2 */
{0x0000000C, 0x00600002}, /* GPIF_AD_CONFIG */
{0x00000010, 0x03020100}, /* GPIF_CTL_FUNC0 */
{0x00000014, 0x07060504}, /* GPIF_CTL_FUNC1 */
{0x00000018, 0x0B0A0908}, /* GPIF_CTL_FUNC2 */
{0x0000001C, 0x0F0E0D0C}, /* GPIF_CTL_FUNC3 */
{0x00000020, 0x13121110}, /* GPIF_CTL_FUNC4 */
{0x00000050, 0x00000001}, /* GPIF_CTRL_BUS_DIRECTION_0 */
{0x00000054, 0x00000001}, /* GPIF_CTRL_BUS_DIRECTION_1 */
{0x00000058, 0x00000001}, /* GPIF_CTRL_BUS_DIRECTION_2 */
{0x0000005C, 0x00000001}, /* GPIF_CTRL_BUS_DIRECTION_3 */
{0x00000060, 0x00000001}, /* GPIF_CTRL_BUS_DIRECTION_4 */
{0x00000064, 0x00000002}, /* GPIF_CTRL_BUS_DIRECTION_5 */
{0x00000068, 0x00000000}, /* GPIF_CTRL_BUS_DIRECTION_6 */
{0x0000006C, 0x00000000}, /* GPIF_CTRL_BUS_DIRECTION_7 */
{0x00000070, 0x00000001}, /* GPIF_CTRL_BUS_DIRECTION_8 */
{0x00000074, 0x00000001}, /* GPIF_CTRL_BUS_DIRECTION_9 */
{0x00000078, 0x00000000}, /* GPIF_CTRL_BUS_DIRECTION_10 */
{0x0000007C, 0x00000000}, /* GPIF_CTRL_BUS_DIRECTION_11 */
{0x00000080, 0x00000000}, /* GPIF_CTRL_BUS_DIRECTION_12 */
{0x00000084, 0x00000000}, /* GPIF_CTRL_BUS_DIRECTION_13 */
{0x00000088, 0x00000000}, /* GPIF_CTRL_BUS_DIRECTION_14 */
{0x0000008C, 0x00000000}, /* GPIF_CTRL_BUS_DIRECTION_15 */
{0x00000090, 0x00000000}, /* GPIF_CTRL_BUS_DIRECTION_16 */
{0x00000094, 0x00000000}, /* GPIF_CTRL_BUS_DIRECTION_17 */
{0x00000098, 0x00000000}, /* GPIF_CTRL_BUS_DIRECTION_18 */
{0x0000009C, 0x00000000}, /* GPIF_CTRL_BUS_DIRECTION_19 */
{0x000000B0, 0x0000003F}, /* GPIF_CTRL_BUS_DEFAULT */
{0x000000B4, 0x0000003F}, /* GPIF_CTRL_BUS_POLARITY */
{0x000000B8, 0x00000000}, /* GPIF_CTRL_BUS_TOGGLE */
{0x00000100, 0x00000000}, /* GPIF_CTRL_BUS_SELECT_0 */
{0x00000104, 0x00000000}, /* GPIF_CTRL_BUS_SELECT_1 */
{0x00000108, 0x00000000}, /* GPIF_CTRL_BUS_SELECT_2 */
{0x0000010C, 0x00000000}, /* GPIF_CTRL_BUS_SELECT_3 */
{0x00000110, 0x00000000}, /* GPIF_CTRL_BUS_SELECT_4 */
{0x00000114, 0x00000018}, /* GPIF_CTRL_BUS_SELECT_5 */
{0x00000118, 0x00000000}, /* GPIF_CTRL_BUS_SELECT_6 */
{0x0000011C, 0x00000000}, /* GPIF_CTRL_BUS_SELECT_7 */
{0x00000120, 0x00000000}, /* GPIF_CTRL_BUS_SELECT_8 */
{0x00000124, 0x00000000}, /* GPIF_CTRL_BUS_SELECT_9 */
{0x00000128, 0x00000000}, /* GPIF_CTRL_BUS_SELECT_10 */
{0x0000012C, 0x00000000}, /* GPIF_CTRL_BUS_SELECT_11 */
{0x00000130, 0x00000000}, /* GPIF_CTRL_BUS_SELECT_12 */
{0x00000134, 0x00000000}, /* GPIF_CTRL_BUS_SELECT_13 */
{0x00000138, 0x00000000}, /* GPIF_CTRL_BUS_SELECT_14 */
{0x0000013C, 0x00000000}, /* GPIF_CTRL_BUS_SELECT_15 */
{0x00000140, 0x00000000}, /* GPIF_CTRL_BUS_SELECT_16 */
{0x00000144, 0x00000000}, /* GPIF_CTRL_BUS_SELECT_17 */
{0x00000148, 0x00000000}, /* GPIF_CTRL_BUS_SELECT_18 */
{0x0000014C, 0x00000000}, /* GPIF_CTRL_BUS_SELECT_19 */
{0x00000160, 0x00000000}, /* GPIF_CTRL_COUNT_CONFIG */
{0x00000164, 0x00000000}, /* GPIF_CTRL_COUNT_RESET */
{0x00000168, 0x00000000}, /* GPIF_CTRL_COUNT_LIMIT */
{0x00000170, 0x00000000}, /* GPIF_ADDR_COUNT_CONFIG */
{0x00000174, 0x00000000}, /* GPIF_ADDR_COUNT_RESET */
{0x00000178, 0x00000000}, /* GPIF_ADDR_COUNT_LIMIT */
{0x00000190, 0x00000000}, /* GPIF_DATA_COUNT_CONFIG */
{0x00000194, 0x00000000}, /* GPIF_DATA_COUNT_RESET_LSB */
{0x0000019C, 0x00000000}, /* GPIF_DATA_COUNT_RESET_MSB */
{0x000001A0, 0x00000000}, /* GPIF_DATA_COUNT_LIMIT_LSB */
{0x000001A4, 0x00000000}, /* GPIF_DATA_COUNT_LIMIT_MSB */
{0x000001A8, 0x00000000}, /* GPIF_CTRL_COMP_VALUE */
{0x000001AC, 0x00000000}, /* GPIF_CTRL_COMP_MASK */
{0x000001B0, 0x00000000}, /* GPIF_DATA_COMP_VALUE_WORD */
{0x000001D0, 0x00000000}, /* GPIF_DATA_COMP_MASK_WORD */
{0x000001E0, 0x00000000}, /* GPIF_ADDR_COMP_VALUE */
{0x000001E4, 0x00000000}, /* GPIF_ADDR_COMP_MASK */
{0x00000210, 0x00000000}, /* GPIF_CRC_CALC_CONFIG */
{0x00000218, 0xFFFFFFFF}, /* GPIF_BETA_DEASSERT */
{0x000002B0, 0x00000000}, /* LINK_IDLE_CFG */
{0x000002C0, 0x00000010}, /* LVCMOS_CLK_OUT_CFG */
};



/*  Summary
    Transition function values used in the state machine.
 */
uint16_t cy_lvds_gpif0_transition[] ={
    0x0000, 0x0202, 0x8080, 0x5555, 0x55FF, 0x7F7F, 0x0800
};

/*  Summary
    Table that maps state indices to the descriptor table indices.
 */
uint8_t cy_lvds_gpif0_wavedata_position[] ={
    0, 1, 0, 2, 0, 0, 3, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 0, 4, 0, 2, 0, 0, 4, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 0, 5, 0, 2, 0, 0, 5, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 0, 6, 0, 2, 0, 0, 6
};



/*  Summary
    Table containing the transition information for various states.
    This table has to be stored in the WAVEFORM Registers.
    This array consists of non - replicated waveform descriptors and acts as a
    waveform table.
 */
const cy_stc_lvds_gpif_wavedata_t cy_lvds_gpif0_wavedata[] ={
    {{0x1E0C4001, 0x0002C304, 0x80000000, 0x00000000}, {0x00000000, 0x00000000, 0x00000000, 0x00000000}},
    {{0x1E0C4006, 0x0002C304, 0x80000000, 0x00000000}, {0x5E0C4002, 0x00000300, 0x80000000, 0x00000000}},
    {{0x1E0C4001, 0x0002C304, 0x80000000, 0x00000000}, {0x5E048004, 0x20000300, 0xC0100000, 0x00000000}},
    {{0x1E0C4001, 0x0002C304, 0x80000000, 0x00000000}, {0x5E0C4002, 0x00000300, 0x80000000, 0x00000000}},
    {{0x3E738405, 0x00000300, 0xC0100000, 0x00000000}, {0x00000000, 0x00000000, 0x00000000, 0x00000000}},
    {{0x408C8003, 0x2003030C, 0x80000000, 0x00000000}, {0x00000000, 0x00000000, 0x00000000, 0x00000000}},
    {{0x5E048004, 0x20000300, 0xC0100000, 0x00000000}, {0x00000000, 0x00000000, 0x00000000, 0x00000000}},
    {{0x00000000, 0x00000000, 0x00000000, 0x00000000}, {0x00000000, 0x00000000, 0x00000000, 0x00000000}}
};


const cy_stc_lvds_gpif_config_t cy_lvds_gpif0_config = {
    (uint16_t)(sizeof(cy_lvds_gpif0_wavedata_position) / sizeof(uint8_t)),
    (cy_stc_lvds_gpif_wavedata_t *)cy_lvds_gpif0_wavedata,
    cy_lvds_gpif0_wavedata_position,
    (uint16_t)(sizeof(cy_lvds_gpif0_transition) / sizeof(uint16_t)),
    cy_lvds_gpif0_transition,
    (uint16_t)(sizeof(cy_lvds_gpif0_reg_data) / sizeof(cy_stc_lvds_gpif_reg_data_t)),
    (cy_stc_lvds_gpif_reg_data_t *)cy_lvds_gpif0_reg_data
};

/* LVDS PHY configuration output generated by Device Configurator */
cy_stc_lvds_phy_config_t cy_lvds_phy0_config ={
    .wideLink            = 0u,
    .modeSelect          = CY_LVDS_PHY_MODE_LVCMOS,
    .lvcmosClkMode       = CY_LVDS_LVCMOS_CLK_SLAVE,
#if BUS_WIDTH_16
    .dataBusWidth        = CY_LVDS_PHY_LVCMOS_MODE_NUM_LANE_16,
#else
    .dataBusWidth        = CY_LVDS_PHY_LVCMOS_MODE_NUM_LANE_8,
#endif
    .gearingRatio        = CY_LVDS_PHY_GEAR_RATIO_1_1,
    .clkSrc              = CY_LVDS_GPIF_CLOCK_LVCMOS_IF,
    .clkDivider          = CY_LVDS_GPIF_CLOCK_DIV_INVALID,
    .interfaceClock      = CY_LVDS_PHY_INTERFACE_CLK_100_MHZ,
    .slaveFifoMode       = CY_LVDS_SLAVE_FIFO_2BIT,
    .ctrlBusBitMap       = 0x0000001F,
    .dataBusDirection    = CY_LVDS_PHY_AD_BUS_ALPHA_CONTROL,
    .loopbackModeEn      = false,
    .isPutLoopbackMode   = false,
    .phyTrainingPattern  = 0x00,
    .linkTrainingPattern = 0x00000000,
    .interfaceClock_kHz  = 0x00000000
};

cy_stc_lvds_config_t cy_lvds0_config ={
    .phyConfig           = (cy_stc_lvds_phy_config_t *)&cy_lvds_phy0_config,
    .gpifConfig          = &cy_lvds_gpif0_config
};



#if defined(__cplusplus)
}
#endif

#endif    /* _GPIF_HEADER_LVCMOS_H_ */

/*[EOF]*/
