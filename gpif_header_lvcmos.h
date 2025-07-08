/*******************************************************************************
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

#ifndef _CY_GPIF_HEADER_LVCMOS_H_
#define _CY_GPIF_HEADER_LVCMOS_H_

#if defined(__cplusplus)
extern "C" {
#endif

const cy_stc_lvds_gpif_wavedata_t cy_lvds_gpif0_wavedata[] = {
    {{0x1E086001,0x000302C4,0x80000000,0x00000000},{0x00000000,0x00000000,0x00000000,0x00000000}},
    {{0x4E080302,0x00000300,0x80000000,0x00000000},{0x1E086006,0x000302C4,0x80000000,0x00000000}},
    {{0x1E086001,0x000302C4,0x80000000,0x00000000},{0x4E040404,0x20000300,0xC0100000,0x00000000}},
    {{0x4E080302,0x00000300,0x80000000,0x00000000},{0x1E086001,0x000302C4,0x80000000,0x00000000}},
    {{0x00000000,0x00000000,0x00000000,0x00000000},{0x00000000,0x00000000,0x00000000,0x00000000}},
    {{0x00000000,0x00000000,0x00000000,0x00000000},{0x3E738405,0x00000300,0xC0100000,0x00000000}},
    {{0x00000000,0x00000000,0x00000000,0x00000000},{0x5E002403,0x2003030C,0x80000000,0x00000000}},
    {{0x00000000,0x00000000,0x00000000,0x00000000},{0x4E040404,0x20000300,0xC0100000,0x00000000}}
};

uint16_t cy_lvds_gpif0_transition[] = {0x0000, 0x8080, 0x2222, 0x5555, 0x7F7F, 0x1F1F, 0x8888};

uint8_t cy_lvds_gpif0_wavedata_position[] = {
    0,1,0,2,0,0,3,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,
    4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,
    0,5,0,2,0,0,5,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,
    4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,
    0,6,0,2,0,0,6,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,
    4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,
    0,7,0,2,0,0,7
};

const cy_stc_lvds_gpif_reg_data_t cy_lvds_gpif0_reg_data[] = {
    {0x00000000,    0x00400C0},    /* GPIF_CONFIG */
    {0x00000004,    0x00000620},    /* GPIF_BUS_CONFIG */
    {0x00000008,    0x01040002},    /* GPIF_BUS_CONFIG2 */
    {0x0000000C,    0x00500000},    /* GPIF_AD_CONFIG */
    {0x00000010,    0x03020100},    /* GPIF_CTL_FUNC0 */
    {0x00000014,    0x07060004},    /* GPIF_CTL_FUNC1 */
    {0x00000018,    0x0B0A0908},    /* GPIF_CTL_FUNC2 */
    {0x0000001C,    0x0F0E0D0C},    /* GPIF_CTL_FUNC3 */
    {0x00000020,    0x13121110},    /* GPIF_CTL_FUNC4 */
    {0x00000030,    0xFFFFFFFF},    /* GPIF_INTR */
    {0x00000034,    0x00000000},    /* GPIF_INTR_MASK */
    {0x00000038,    0x00000000},    /* GPIF_INTR_MASKED */
    {0x0000003C,    0x00000000},    /* GPIF_INTR_SET */
    {0x00000050,    0x00000001},    /* GPIF_CTRL_BUS_DIRECTION_0 */
    {0x00000054,    0x00000001},    /* GPIF_CTRL_BUS_DIRECTION_1 */
    {0x00000058,    0x00000001},    /* GPIF_CTRL_BUS_DIRECTION_2 */
    {0x0000005C,    0x00000001},    /* GPIF_CTRL_BUS_DIRECTION_3 */
    {0x00000060,    0x00000001},    /* GPIF_CTRL_BUS_DIRECTION_4 */ 
    {0x00000064,    0x00000002},    /* GPIF_CTRL_BUS_DIRECTION_5 */
    {0x00000068,    0x00000000},    /* GPIF_CTRL_BUS_DIRECTION_6 */
    {0x0000006C,    0x00000000},    /* GPIF_CTRL_BUS_DIRECTION_7 */
    {0x00000070,    0x00000000},    /* GPIF_CTRL_BUS_DIRECTION_8 */
    {0x00000074,    0x00000000},    /* GPIF_CTRL_BUS_DIRECTION_9 */
    {0x00000078,    0x00000000},    /* GPIF_CTRL_BUS_DIRECTION_10 */
    {0x0000007C,    0x00000000},    /* GPIF_CTRL_BUS_DIRECTION_11 */
    {0x00000080,    0x00000000},    /* GPIF_CTRL_BUS_DIRECTION_12 */
    {0x00000084,    0x00000000},    /* GPIF_CTRL_BUS_DIRECTION_13 */
    {0x00000088,    0x00000000},    /* GPIF_CTRL_BUS_DIRECTION_14 */
    {0x0000008C,    0x00000000},    /* GPIF_CTRL_BUS_DIRECTION_15 */
    {0x00000090,    0x00000000},    /* GPIF_CTRL_BUS_DIRECTION_16 */
    {0x00000094,    0x00000000},    /* GPIF_CTRL_BUS_DIRECTION_17 */
    {0x00000098,    0x00000000},    /* GPIF_CTRL_BUS_DIRECTION_18 */
    {0x0000009C,    0x00000000},    /* GPIF_CTRL_BUS_DIRECTION_19 */
    {0x000000B0,    0x0000003F},    /* GPIF_CTRL_BUS_DEFAULT */
    {0x000000B4,    0x0000003F},    /* GPIF_CTRL_BUS_POLARITY */
    {0x000000B8,    0x00000000},    /* GPIF_CTRL_BUS_TOGGLE */
    {0X00000100,    0X00000000},    /* GPIF_CTRL_BUS_SELECT0 */
    {0X00000104,    0X00000000},    /* GPIF_CTRL_BUS_SELECT1 */
    {0X00000108,    0X00000000},    /* GPIF_CTRL_BUS_SELECT2 */
    {0X0000010C,    0X00000000},    /* GPIF_CTRL_BUS_SELECT3 */
    {0X00000110,    0X00000000},    /* GPIF_CTRL_BUS_SELECT4 */
    {0X00000114,    0X00000018},    /* GPIF_CTRL_BUS_SELECT5 */ // FLAG A
    {0X00000118,    0X00000000},    /* GPIF_CTRL_BUS_SELECT6 */
    {0X0000011C,    0X00000000},    /* GPIF_CTRL_BUS_SELECT7 */
    {0X00000120,    0X00000000},    /* GPIF_CTRL_BUS_SELECT8 */
    {0X00000124,    0X00000000},    /* GPIF_CTRL_BUS_SELECT9 */
    {0X00000128,    0X00000000},    /* GPIF_CTRL_BUS_SELECT10 */
    {0X0000012C,    0X00000000},    /* GPIF_CTRL_BUS_SELECT11 */
    {0X00000130,    0X00000000},    /* GPIF_CTRL_BUS_SELECT12 */
    {0X00000134,    0X00000000},    /* GPIF_CTRL_BUS_SELECT13 */
    {0X00000138,    0X00000000},    /* GPIF_CTRL_BUS_SELECT14 */
    {0X0000013C,    0X00000000},    /* GPIF_CTRL_BUS_SELECT15 */
    {0X00000140,    0X00000000},    /* GPIF_CTRL_BUS_SELECT16 */
    {0X00000144,    0X00000000},    /* GPIF_CTRL_BUS_SELECT17 */
    {0X00000148,    0X00000000},    /* GPIF_CTRL_BUS_SELECT18 */
    {0X0000014C,    0X00000000},    /* GPIF_CTRL_BUS_SELECT19 */
    {0x00000160,    0X00000000},    /* GPIF_CTRL_COUNT_CONFIG */
    {0x00000164,    0X00000000},    /* GPIF_CTRL_COUNT_RESET */
    {0x00000168,    0X00000000},    /* GPIF_CTRL_COUNT_LIMIT */
    {0x00000170,    0X00000000},    /* GPIF_ADDR_COUNT_CONFIG */
    {0x00000174,    0x00000000},    /* GPIF_ADDR_COUNT_RESET */
    {0x00000178,    0x00000000},    /* GPIF_ADDR_COUNT_LIMIT */
    {0x00000180,    0x00000000},    /* GPIF_STATE_COUNT_CONFIG */
    {0x00000184,    0x00000000},    /* GPIF_STATE_COUNT_LIMIT */
    {0x00000190,    0x00000000},    /* GPIF_DATA_COUNT_CONFIG */
    {0x00000194,    0x00000000},    /* GPIF_DATA_COUNT_RESET_LSB */
    {0x0000019C,    0x00000000},    /* GPIF_DATA_COUNT_RESET_MSB */
    {0x000001A0,    0x00000000},    /* GPIF_DATA_COUNT_LIMIT_LSB */
    {0x000001A4,    0x00000000},    /* GPIF_DATA_COUNT_LIMIT_MSB */
    {0x000001A8,    0x00000000},    /* GPIF_CTRL_COMP_VALUE */
    {0x000001AC,    0x00000000},    /* GPIF_CTRL_COMP_MASK */
    {0x000001B0,    0x00000000},    /* GPIF_DATA_COMP_VALUE_WORD0 */
    {0x000001B4,    0x00000000},    /* GPIF_DATA_COMP_VALUE_WORD1 */
    {0x000001B8,    0x00000000},    /* GPIF_DATA_COMP_VALUE_WORD2 */
    {0x000001BC,    0x00000000},    /* GPIF_DATA_COMP_VALUE_WORD3 */
    {0x000001D0,    0x00000000},    /* GPIF_DATA_COMP_MASK_WORD0 */
    {0x000001D4,    0x00000000},    /* GPIF_DATA_COMP_MASK_WORD1 */
    {0x000001D8,    0x00000000},    /* GPIF_DATA_COMP_MASK_WORD2 */
    {0x000001DC,    0x00000000},    /* GPIF_DATA_COMP_MASK_WORD3 */
    {0x000001E0,    0x00000000},    /* GPIF_ADDR_COMP_VALUE */
    {0x000001E4,    0x00000000},    /* GPIF_ADDR_COMP_MASK */
    {0x00000200,    0x00000000},    /* GPIF_WAVEFORM_CTRL_STAT */
    {0x00000204,    0x00000000},    /* GPIF_WAVEFORM_SWITCH */
    {0x00000208,    0x00000000},    /* GPIF_WAVEFORM_SWITCH_TIMEOUT */
    {0x00000210,    0x00000000},    /* GPIF_CRC_CONFIG */
    {0x00000218,    0XFFFFFFFF},    /* GPIF_BETA_DEASSERT */
    {0x000002B0,    0x00000000},    /* LINK_IDLE_CONFIG */
};

//Pushing data into Thread 0 Socket
const cy_stc_lvds_gpif_config_t cy_lvds_gpif0_config = {
    (uint16_t)(sizeof(cy_lvds_gpif0_wavedata_position)/sizeof(uint8_t)),
    (cy_stc_lvds_gpif_wavedata_t *) cy_lvds_gpif0_wavedata,
    cy_lvds_gpif0_wavedata_position,
    (uint16_t)(sizeof(cy_lvds_gpif0_transition)/sizeof(uint16_t)),
    cy_lvds_gpif0_transition,
    (uint16_t)(sizeof(cy_lvds_gpif0_reg_data)/sizeof(cy_stc_lvds_gpif_reg_data_t)),
    (cy_stc_lvds_gpif_reg_data_t *) cy_lvds_gpif0_reg_data
};

/* LVCMOS  PHY configuration output generated by Device Configurator */
cy_stc_lvds_phy_config_t cy_lvds_phy0_config = {
    .wideLink = 0u,
#if BUS_WIDTH_16
    .dataBusWidth = CY_LVDS_PHY_LVCMOS_MODE_NUM_LANE_16,
#else
    .dataBusWidth = CY_LVDS_PHY_LVCMOS_MODE_NUM_LANE_8,
#endif
    .modeSelect = CY_LVDS_PHY_MODE_LVCMOS,
    .gearingRatio = CY_LVDS_PHY_GEAR_RATIO_1_1, 
    .clkSrc = CY_LVDS_GPIF_CLOCK_LVCMOS_IF,
    .clkDivider = CY_LVDS_GPIF_CLOCK_DIV_INVALID,
    .interfaceClock = CY_LVDS_PHY_INTERFACE_CLK_100_MHZ,
    .slaveFifoMode = CY_LVDS_SLAVE_FIFO_2BIT,
    .loopbackModeEn = false,
    .isPutLoopbackMode = false,
    .phyTrainingPattern = PHY_TRAINING_PATTERN_BYTE,
    .linkTrainingPattern = LINK_TRAINING_PATTERN_BYTE,
    .ctrlBusBitMap = 0x0000031F
};

cy_stc_lvds_config_t cy_lvds0_config =
{
    .phyConfig  = (cy_stc_lvds_phy_config_t *)&cy_lvds_phy0_config,
    .gpifConfig = &cy_lvds_gpif0_config
};

#if defined(__cplusplus)
}
#endif

#endif /* _CY_GPIF_HEADER_LVCMOS_H_ */

/* End of File */

