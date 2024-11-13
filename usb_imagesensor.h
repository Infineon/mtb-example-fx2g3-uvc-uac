/***************************************************************************//**
* \file cy_imagesensor.h
* \version 1.0
*
* Defines Image Sensor related macros and functions
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


#ifndef _INCLUDED_CY_IMAGESENSOR_H_
#define _INCLUDED_CY_IMAGESENSOR_H_

/******************/
/* Include Files  */
/******************/
#if (MIPI_SOURCE_ENABLE)
#include <stdint.h>
/******************/
/* Global Defines */
/******************/
#define PCAMV2_POWER_PORT                      (1)
#define PCAMV2_POWER_PIN                       (cy_en_lvds_phy_gpio_index_t)(7)

/******************/
/*Global Variables*/
/******************/
typedef struct cy_stc_sensorConfig_t
{
    uint16_t sensorAddr;
    uint8_t sensorVal;
}cy_stc_sensorConfig_t;

/********************/
/* Global Functions */
/********************/
void Cy_UVC_ConfigureImageSensor(void);
void Cy_UVC_SendI2cTable(const cy_stc_sensorConfig_t *sensorConfig, uint32_t count);
void Cy_UVC_ImageSensorSetResolution(uint32_t width,uint32_t height);

#endif /* (MIPI_SOURCE_ENABLE) */
#endif /* _INCLUDED_CY_IMAGESENSOR_H_ */
