/***************************************************************************//**
* \file usb_i2c.h
* \version 1.0
*
* Defines I2C related macros and functions
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

#ifndef _CY_USB_I2C_H_
#define _CY_USB_I2C_H_

#include "usb_app.h"

#define FPGASLAVE_ADDR                   (0x0D)        //FPGA i2c address (Do not change)

/* I2C Related macro */
#define I2C_READ                       (1)
#define I2C_WRITE                      (0)
#define FPGA_I2C_ADDRESS_WIDTH         (2)
#define FPGA_I2C_DATA_WIDTH            (1)
#define I2C_BUFF_SIZE                  (10)
#define I2C_DATARATE                   (100000)
#define I2C_INCLK_TARGET_FREQ          (3200000)

extern cy_stc_scb_i2c_context_t I2C_context;


void Cy_USB_I2CInit (void);
void Cy_I2C_MasterEvent(uint32_t Events);
cy_en_scb_i2c_status_t Cy_I2C_Read (uint16_t slaveAddress,uint16_t registerAddress,
                    uint8_t *data,
                    uint8_t addressWidth,
                    uint8_t dataWidth);
cy_en_scb_i2c_status_t Cy_I2C_Write (uint16_t slaveAddress,uint16_t registerAddress,
                    uint8_t data,
                    uint8_t addressWidth,
                    uint8_t dataWidth);
cy_en_scb_i2c_status_t Cy_APP_GetFPGAVersion(cy_stc_usb_app_ctxt_t *pAppCtxt);

#endif //End _CY_USB_i2C_H_
