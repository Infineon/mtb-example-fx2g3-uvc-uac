/***************************************************************************//**
* \file cy_imagesensor.h
* \version 1.0
*
* \brief Defines Image Sensor related macros and functions
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


#ifndef _INCLUDED_USB_IMAGESENSOR_H_
#define _INCLUDED_USB_IMAGESENSOR_H_

#if (MIPI_SOURCE_ENABLE)
#include <stdint.h>

#define PCAMV2_POWER_PORT                      (1)
#define PCAMV2_POWER_PIN                       (cy_en_lvds_phy_gpio_index_t)(7)


typedef struct cy_stc_sensorConfig_t
{
    uint16_t sensorAddr;
    uint8_t sensorVal;
}cy_stc_sensorConfig_t;


/**
 * \name Cy_UVC_ConfigureImageSensor
 * \brief This function configures image sensor
 * \retval None
 */
void Cy_UVC_ConfigureImageSensor(void);

/**
 * \name Cy_UVC_SendI2cTable
 * \brief I2C writes to image sensor
 * \param sensorConfig pointer to sesnofr configuration table
 * \param count number of I2C writes in the configuration table
 * \retval None
 */
void Cy_UVC_SendI2cTable(const cy_stc_sensorConfig_t *sensorConfig, uint32_t count);

/**
 * \name Cy_UVC_ImageSensorSetResolution
 * \brief This function Sets video resolution 
 * \param width Video width (in pixels)
 * \param height Video height (in pixels)
 * \retval None
 */
void Cy_UVC_ImageSensorSetResolution(uint32_t width,uint32_t height);

#endif /* (MIPI_SOURCE_ENABLE) */
#endif /* _INCLUDED_USB_IMAGESENSOR_H_ */
