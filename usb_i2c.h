/***************************************************************************//**
* \file usb_i2c.h
* \version 1.0
*
* \brief Defines I2C related macros and functions
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

#ifndef _CY_USB_I2C_H_
#define _CY_USB_I2C_H_

#include "usb_app.h"

#define FPGASLAVE_ADDR                 (0x0D)        //FPGA i2c address (Do not change)

/* I2C Related macro */
#define I2C_READ                       (1)
#define I2C_WRITE                      (0)
#define FPGA_I2C_ADDRESS_WIDTH         (2)
#define FPGA_I2C_DATA_WIDTH            (1)
#define I2C_BUFF_SIZE                  (10)
#define I2C_DATARATE                   (100000)
#define I2C_INCLK_TARGET_FREQ          (3200000)

extern cy_stc_scb_i2c_context_t I2C_context;

/**
 * \name Cy_USB_I2CInit
 * \brief Fucntion to initialize I2C master
 * \retval Does not return.
 */
void Cy_USB_I2CInit (void);

/**
 * \name Cy_I2C_MasterEvent
 * \brief  I2C master event callback function, handling various i2c master event
 * \param events i2c master events
 * \retval Does not return.
 */
void Cy_I2C_MasterEvent(uint32_t events);

/**
 * \name Cy_I2C_Read
 * \brief  Function to read data from i2c slave
 * \param slaveAddress 7-bit i2c slave address
 * \param registerAddress i2c slave register address
 * \param data pointer to data receive buffer
 * \param addressWidth address size (in bytes)
 * \param dataWidth data size (in bytes)
 * \retval 0 for read success, error code for unsuccess.
 */
cy_en_scb_i2c_status_t Cy_I2C_Read (uint16_t slaveAddress,uint16_t registerAddress,
                    uint8_t *data,
                    uint8_t addressWidth,
                    uint8_t dataWidth);

/**
 * \name Cy_I2C_Write
 * \brief Function to write data to i2c slave
 * \param slaveAddress 7-bit i2c slave address
 * \param registerAddress i2c slave register address
 * \param data pointer to data buffer
 * \param addressWidth address size (in bytes)
 * \param dataWidth data size (in bytes)
 * \retval 0 for read success, error code for unsuccess.
 */
cy_en_scb_i2c_status_t Cy_I2C_Write (uint16_t slaveAddress,uint16_t registerAddress,
                    uint8_t data,
                    uint8_t addressWidth,
                    uint8_t dataWidth);

/**
 * \name Cy_APP_GetFPGAVersion
 * \brief Function to read FPGA version
 * \param pAppCtxt application layer context pointer
 * \retval i2c status
 */
cy_en_scb_i2c_status_t Cy_APP_GetFPGAVersion(cy_stc_usb_app_ctxt_t *pAppCtxt);

#endif //End _CY_USB_I2C_H_ 
