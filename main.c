/***************************************************************************//**
* \file main.c
* \version 1.0
*
* \details    This is the source code for the USB Video Class + USB Audio Class
*           Application Example for ModusToolbox.
*
* See \ref README.md ["README.md"]
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

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "cy_pdl.h"
#include "cybsp.h"
#include <string.h>
#include "cy_usb_common.h"
#include "usb_uvc_device.h"
#include "cy_usb_usbd.h"
#include "usb_app.h"
#include "cy_debug.h"
#include "cy_usbd_version.h"
#include "cy_hbdma_version.h"
#include "cy_lvds.h"
#include "app_version.h"
#include "timers.h"
#include "usb_i2c.h"
#include "usb_qspi.h"
#include "gpif_header_lvcmos.h"

/* Select SCB interface used for UART based logging. */
#define LOGGING_SCB             (SCB4)
#define LOGGING_SCB_IDX         (4)
#define DEBUG_LEVEL             (3u)

/* Debug log related initilization */
#if DEBUG_INFRA_EN
#define LOGBUF_SIZE (1024u)
uint8_t logBuff[LOGBUF_SIZE];

cy_stc_debug_config_t dbgCfg = {
    .pBuffer         = logBuff,
    .traceLvl        = DEBUG_LEVEL,
    .bufSize         = LOGBUF_SIZE,
#if USBFS_LOGS_ENABLE
    .dbgIntfce       = CY_DEBUG_INTFCE_USBFS_CDC,
#else
    .dbgIntfce       = CY_DEBUG_INTFCE_UART_SCB4,
#endif
    .printNow        = true
};

TaskHandle_t printLogTaskHandle;
#endif /* DEBUG_INFRA_EN */

/* Global variables associated with High BandWidth DMA setup. */
cy_stc_hbdma_context_t HBW_DrvCtxt;     /* High BandWidth DMA driver context. */
cy_stc_hbdma_dscr_list_t HBW_DscrList;  /* High BandWidth DMA descriptor free list. */
cy_stc_hbdma_buf_mgr_t HBW_BufMgr;      /* High BandWidth DMA buffer manager. */
cy_stc_hbdma_mgr_context_t HBW_MgrCtxt; /* High BandWidth DMA manager context. */

/* Global variables associated with LVDS setup. */
cy_stc_lvds_context_t lvdsContext;

/* CPU DMA register pointers. */
DMAC_Type *pCpuDmacBase;
DW_Type   *pCpuDw0Base;
DW_Type   *pCpuDw1Base;

cy_stc_usb_usbd_ctxt_t usbdCtxt;
cy_stc_usb_app_ctxt_t appCtxt;
cy_stc_usb_cal_ctxt_t hsCalCtxt;

extern void xPortPendSVHandler(void);
extern void xPortSysTickHandler(void);
extern void vPortSVCHandler(void);

void vPortSetupTimerInterrupt(void)
{
    /* Register the exception vectors. */
    Cy_SysInt_SetVector(PendSV_IRQn, xPortPendSVHandler);
    Cy_SysInt_SetVector(SVCall_IRQn, vPortSVCHandler);
    Cy_SysInt_SetVector(SysTick_IRQn, xPortSysTickHandler);

    /* Start the SysTick timer with a period of 1 ms. */
    Cy_SysTick_SetClockSource(CY_SYSTICK_CLOCK_SOURCE_CLK_CPU);
    Cy_SysTick_SetReload(Cy_SysClk_ClkFastGetFrequency() / 1000U);
    Cy_SysTick_Clear();
    Cy_SysTick_Enable();
}

#if DEBUG_INFRA_EN
void PrintTaskHandler(void *pTaskParam)
{
    while (1)
    {
        /* Print any pending logs to the output console. */
        Cy_Debug_PrintLog();

        /* Put the thread to sleep for 5 ms */
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
#endif /* DEBUG_INFRA_EN */

/**
 * \name Cy_LVDS_GpifEventCb
 * \brief GPIF error callback function.
 * \param smNo state machine number
 * \param gpifEvent GPIF event
 * \param cntxt app context
 * \retval None
 */
void Cy_LVDS_GpifEventCb(uint8_t smNo, cy_en_lvds_gpif_event_type_t gpifEvent, void *cntxt)
{
    CyUvcAppGpifIntr(&appCtxt);
}

/**
 * \name Cy_LVDS_PhyEventCb
 * \brief GPIF error callback function.
 * \param smNo state machine number
 * \param phyEvent LVCMOS PHY event
 * \param cntxt app context
 * \retval None
 */
void Cy_LVDS_PhyEventCb(uint8_t smNo, cy_en_lvds_phy_events_t phyEvent, void *cntxt)
{
    if(phyEvent == CY_LVDS_PHY_L1_EXIT)
    {
        if(smNo == 0)
        {
           DBG_APP_INFO("P0_L1_Exit\r\n");
        }
    }
    if(phyEvent == CY_LVDS_PHY_L1_ENTRY)
    {
        if(smNo == 0)
        {
            DBG_APP_INFO("P0_L1_Entry\r\n");
        }
    }
    if(phyEvent == CY_LVDS_PHY_L3_ENTRY)
    {
        if(smNo == 0)
        {
            DBG_APP_INFO("P0_L3_Entry\r\n");
        }
    }
}

/**
 * \name Cy_LVDS_GpifErrorCb
 * \brief GPIF error callback function.
 * \param smNo state machine number
 * \param gpifError GPIF error
 * \param cntxt app context
 * \retval None
 */
void Cy_LVDS_GpifErrorCb(uint8_t smNo, cy_en_lvds_gpif_error_t gpifError, void *cntxt)
{
    switch (gpifError)
    {
        case CY_LVDS_GPIF_ERROR_IN_ADDR_OVER_WRITE:
        DBG_APP_ERR("CY_LVDS_GPIF_ERROR_IN_ADDR_OVER_WRITE\n\r");
        break;

        case CY_LVDS_GPIF_ERROR_EG_ADDR_NOT_VALID:
        DBG_APP_ERR("CY_LVDS_GPIF_ERROR_EG_ADDR_NOT_VALID\n\r");
        break;

        case CY_LVDS_GPIF_ERROR_DMA_DATA_RD_ERROR:
        DBG_APP_ERR("CY_LVDS_GPIF_ERROR_DMA_DATA_RD_ERROR\n\r");
        break;

        case CY_LVDS_GPIF_ERROR_DMA_DATA_WR_ERROR:
        DBG_APP_ERR("CY_LVDS_GPIF_ERROR_DMA_DATA_WR_ERROR\n\r");
        break;

        case CY_LVDS_GPIF_ERROR_DMA_ADDR_RD_ERROR:
        DBG_APP_ERR("CY_LVDS_GPIF_ERROR_DMA_DATA_WR_ERROR\n\r");
        break;

        case CY_LVDS_GPIF_ERROR_DMA_ADDR_WR_ERROR:
        DBG_APP_ERR("CY_LVDS_GPIF_ERROR_DMA_ADDR_WR_ERROR\n\r");
        break;

        case CY_LVDS_GPIF_ERROR_INVALID_STATE_ERROR:
        DBG_APP_ERR("CY_LVDS_GPIF_ERROR_INVALID_STATE_ERROR\n\r");
        break;
    }
}

/**
 * \name Cy_LVDS_GpifThreadErrorCb
 * \brief GPIF thread error callback function.
 * \param ThNo thread number
 * \param ThError thread error
 * \param cntxt app context
 * \retval None
 */
void Cy_LVDS_GpifThreadErrorCb (cy_en_lvds_gpif_thread_no_t ThNo, cy_en_lvds_gpif_thread_error_t ThError, void *cntxt)
{
    switch (ThNo)
    {
        case CY_LVDS_GPIF_THREAD_0:
        case CY_LVDS_GPIF_THREAD_1:
        switch (ThError)
        {
            case CY_LVDS_GPIF_THREAD_DIR_ERROR:
            DBG_APP_ERR("CY_LVDS_GPIF_THREAD_DIR_ERROR\n\r");
            break;

            case CY_LVDS_GPIF_THREAD_WR_OVERFLOW:
            DBG_APP_ERR("CY_LVDS_GPIF_THREAD_WR_OVERFLOW\n\r");
            break;

            case CY_LVDS_GPIF_THREAD_RD_UNDERRUN:
            DBG_APP_ERR("CY_LVDS_GPIF_THREAD_RD_UNDERRUN\n\r");
            break;

            case CY_LVDS_GPIF_THREAD_SCK_ACTIVE:
            DBG_APP_ERR("CY_LVDS_GPIF_THREAD_SCK_ACTIVE\n\r");
            break;

            case CY_LVDS_GPIF_THREAD_ADAP_OVERFLOW:
            DBG_APP_ERR("CY_LVDS_GPIF_THREAD_ADAP_OVERFLOW\n\r");
            break;

            case CY_LVDS_GPIF_THREAD_ADAP_UNDERFLOW:
            DBG_APP_ERR("CY_LVDS_GPIF_THREAD_ADAP_UNDERFLOW\n\r");
            break;

            case CY_LVDS_GPIF_THREAD_READ_FORCE_END:
            DBG_APP_ERR("CY_LVDS_GPIF_THREAD_READ_FORCE_END\n\r");
            break;

            case CY_LVDS_GPIF_THREAD_READ_BURST_ERR:
            DBG_APP_ERR("CY_LVDS_GPIF_THREAD_READ_BURST_ERR\n\r");
            break;

            default:
                break;
        }
        break;

        default:
            break;
    }   
}

cy_stc_lvds_app_cb_t cb =
{
    .gpif_events = Cy_LVDS_GpifEventCb,
    .gpif_error = Cy_LVDS_GpifErrorCb,
    .gpif_thread_error = Cy_LVDS_GpifThreadErrorCb,
    .gpif_thread_event = NULL,
    .phy_events = Cy_LVDS_PhyEventCb,
    .low_power_events   = NULL
};

/**
 * \name Cy_Fx2g3_InitPeripheralClocks
 * \brief Function used to enable clocks to different peripherals on the FX2G3 device.
 * \param adcClkEnable Whether to enable clock to the ADC in the USBSS block.
 * \param usbfsClkEnable Whether to enable bus reset detect clock input to the USBFS block.
 * \retval None
 */
void Cy_Fx2g3_InitPeripheralClocks (
        bool adcClkEnable,
        bool usbfsClkEnable)
{
    if (adcClkEnable) {
        /* Divide PERI clock at 75 MHz by 75 to get 1 MHz clock using 16-bit divider #1. */
        Cy_SysClk_PeriphSetDivider(CY_SYSCLK_DIV_16_BIT, 1, 74);
        Cy_SysClk_PeriphEnableDivider(CY_SYSCLK_DIV_16_BIT, 1);
        Cy_SysLib_DelayUs(10U);
        Cy_SysClk_PeriphAssignDivider(PCLK_LVDS2USB32SS_CLOCK_SAR, CY_SYSCLK_DIV_16_BIT, 1);
    }

    if (usbfsClkEnable) {
        /* Divide PERI clock at 75 MHz by 750 to get 100 KHz clock using 16-bit divider #2. */
        Cy_SysClk_PeriphSetDivider(CY_SYSCLK_DIV_16_BIT, 2, 749);
        Cy_SysClk_PeriphEnableDivider(CY_SYSCLK_DIV_16_BIT, 2);
        Cy_SysLib_DelayUs(10U);
        Cy_SysClk_PeriphAssignDivider(PCLK_USB_CLOCK_DEV_BRS, CY_SYSCLK_DIV_16_BIT, 2);
    }
}

/**
 * \name Cy_Fx2g3_OnResetInit
 * \details
 * This function performs initialization that is required to enable scatter
 * loading of data into the High BandWidth RAM during device boot-up. The FX2G3
 * device comes up with the High BandWidth RAM disabled and hence any attempt
 * to read/write the RAM will cause the processor to hang. The RAM needs to
 * be enabled with default clock settings to allow scatter loading to work.
 * This function needs to be called from Cy_OnResetUser.
 * \retval None
 */
void
Cy_Fx2g3_OnResetInit (
        void)
{
    /* Enable clk_hf4 with IMO as input. */
    SRSS->CLK_ROOT_SELECT[4] = SRSS_CLK_ROOT_SELECT_ENABLE_Msk;

    /* Enable LVDS2USB32SS IP and select clk_hf[4] as clock input. */
    MAIN_REG->CTRL = (
            MAIN_REG_CTRL_IP_ENABLED_Msk |
            (1UL << MAIN_REG_CTRL_NUM_FAST_AHB_STALL_CYCLES_Pos) |
            (1UL << MAIN_REG_CTRL_NUM_SLOW_AHB_STALL_CYCLES_Pos) |
            (3UL << MAIN_REG_CTRL_DMA_SRC_SEL_Pos));
}


/**
 * \name Cy_UVC_LvdsInit
 * \brief   Initialize the LVDS interface. Currently, only the SIP #0 is being initialized
 *          and configured to allow transfers into the HBW SRAM through DMA.
 * \retval None
 */
void Cy_UVC_LvdsInit(void)
{
    cy_en_lvds_status_t status = CY_LVDS_SUCCESS;

    Cy_LVDS_SetInterruptMask(LVDSSS_LVDS, LVDSSS_LVDS_LVDS_INTR_WD0_LNK0_TRAINING_DONE_Msk |
                                              LVDSSS_LVDS_LVDS_INTR_WD0_LNK0_TRAINING_BLK_DETECTED_Msk  |
                                              LVDSSS_LVDS_LVDS_INTR_WD0_LNK0_TRAINING_BLK_DET_FAILD_Msk |
                                              LVDSSS_LVDS_LVDS_INTR_WD0_LNK0_L1_ENTRY_Msk  |
                                              LVDSSS_LVDS_LVDS_INTR_WD0_LNK0_L1_EXIT_Msk  |
                                              LVDSSS_LVDS_LVDS_INTR_WD0_LNK0_L3_ENTRY_Msk  | LVDSSS_LVDS_LVDS_INTR_WD0_PHY_LINK0_INTERRUPT_Msk |
                                              LVDSSS_LVDS_LVDS_INTR_WD0_THREAD0_ERR_Msk | LVDSSS_LVDS_LVDS_INTR_WD0_THREAD1_ERR_Msk |
                                              LVDSSS_LVDS_LVDS_INTR_MASK_WD0_GPIF0_INTERRUPT_Msk);
    Cy_LVDS_RegisterCallback(LVDSSS_LVDS, &cb, &lvdsContext,&appCtxt);


    Cy_LVDS_Init(LVDSSS_LVDS, 0, &cy_lvds0_config, &lvdsContext);
    DBG_APP_INFO("LVDS_Init done\r\n");


    /* Set Interrupt Mask for GPIF */
    Cy_LVDS_SetInterruptMask(LVDSSS_LVDS, LVDSSS_LVDS_LVDS_INTR_MASK_WD0_GPIF0_INTERRUPT_Msk);
    DBG_APP_INFO("Set Interrupt Mask for GPIF\r\n");

    Cy_LVDS_RegisterCallback(LVDSSS_LVDS, &cb, &lvdsContext, &appCtxt);
    
    Cy_LVDS_GpifThreadConfig(LVDSSS_LVDS, 0, 0, 0, 0, 0);

#if INTERLEAVE_EN
    Cy_LVDS_GpifThreadConfig(LVDSSS_LVDS, 1, 1, 0, 0, 0);
#endif /* INTERLEAVE_EN */

    Cy_LVDS_Enable(LVDSSS_LVDS);
    DBG_APP_INFO("LVCMOS Enabled\r\n");

    status = Cy_LVDS_GpifSMStart(LVDSSS_LVDS, 0, 0, 0xC);
    ASSERT_NON_BLOCK(CY_LVDS_SUCCESS == status, status);
}

/**
 * \name Cy_LVDS_ISR
 * \brief Handler for LVDS Interrupts.
 * \retval None
 */
void Cy_LVDS_ISR(void)
{
    Cy_LVDS_IrqHandler(LVDSSS_LVDS, &lvdsContext);
}

/**
 * \name Cy_LvdsPortDma_ISR
 * \brief Handler for LVDS Port0.
 * \retval None
 */
void Cy_LvdsPortDma_ISR(void)
{
    /* Call the HBDMA interrupt handler with the appropriate adapter ID. */
    Cy_HBDma_HandleInterrupts(&HBW_DrvCtxt, CY_HBDMA_ADAP_LVDS_0);
    portYIELD_FROM_ISR(true);
}

/**
 * \name Cy_USB_HS_ISR
 * \brief Handler for USB-HS Interrupts.
 * \retval None
 */
void Cy_USB_HS_ISR(void)
{
    if (Cy_USBHS_Cal_IntrHandler(&hsCalCtxt))
    {
        portYIELD_FROM_ISR(true);
    }
}

#if !CY_CPU_CORTEX_M4
/**
 * \name Cy_UVC_DataWire1Combined_ISR
 * \brief Combined Interrupt handler for the Datawire1 Channels.
 * \retval None
 */
void Cy_UVC_DataWire1Combined_ISR (void)
{
    uint32_t chnId;
    for (chnId = 1; chnId < 22; chnId++) {
        if (Cy_DMA_Channel_GetInterruptStatus(DW1, chnId) != 0) {
            switch(chnId)
            {
                case UVC_STREAM_ENDPOINT:
            	    CY_UVC_DataWire_ISR();
                    break;
                case UAC_IN_ENDPOINT:
                    Cy_PDM_InEpDma_ISR();
                    break;
                case PDM_RX_CH0:
                    PDM_CH0_RX_ISR();
                    break;
                case PDM_RX_CH1:
                    PDM_CH1_RX_ISR();
                    break;
            }
        }
    }
}
#endif /* !CY_CPU_CORTEX_M4 */

/**
 * \name CY_UVC_DataWire_ISR
 * \brief Interrupt handler for the UVC Channel.
 * \retval None
 */
void CY_UVC_DataWire_ISR (void)
{
    /* Clear the interrupt first. */
    Cy_USB_AppClearDmaInterrupt(&appCtxt, appCtxt.uvcInEpNum, CY_USB_ENDP_DIR_IN);

    /* The current data buffer has been consumed. Move to another buffer if available. */
    CyUvcAppHandleSendCompletion(&appCtxt);
}

/**
 * \name Cy_PrintVersionInfo
 * \brief Function to print version information to UART console.
 * \param type Type of version string.
 * \param version Version number including major, minor, patch and build number.
 * \retval None
 */
void Cy_PrintVersionInfo(const char *type, uint32_t version)
{
    char tString[32];
    uint16_t vBuild;
    uint8_t vMajor, vMinor, vPatch;
    uint8_t typeLen = strlen(type);

    vMajor = (version >> 28U);
    vMinor = ((version >> 24U) & 0x0FU);
    vPatch = ((version >> 16U) & 0xFFU);
    vBuild = (uint16_t)(version & 0xFFFFUL);

    memcpy(tString, type, typeLen);
    tString[typeLen++] = '0' + (vMajor / 10);
    tString[typeLen++] = '0' + (vMajor % 10);
    tString[typeLen++] = '.';
    tString[typeLen++] = '0' + (vMinor / 10);
    tString[typeLen++] = '0' + (vMinor % 10);
    tString[typeLen++] = '.';
    tString[typeLen++] = '0' + (vPatch / 10);
    tString[typeLen++] = '0' + (vPatch % 10);
    tString[typeLen++] = '.';
    tString[typeLen++] = '0' + (vBuild / 1000);
    tString[typeLen++] = '0' + ((vBuild % 1000) / 100);
    tString[typeLen++] = '0' + ((vBuild % 100) / 10);
    tString[typeLen++] = '0' + (vBuild % 10);
    tString[typeLen++] = '\r';
    tString[typeLen++] = '\n';
    tString[typeLen] = 0;

    Cy_Debug_AddToLog(1,"%s", tString);
}

/**
 * \name Cy_USB_VbusDetGpio_ISR
 * \brief Interrupt handler for the Vbus detect GPIO transition detection.
 * \retval None
 */
static void Cy_USB_VbusDetGpio_ISR(void)
{
    BaseType_t xHigherPriorityTaskWoken;
    cy_stc_usbd_app_msg_t xMsg;

    /* Send VBus changed message to the task thread. */
    xMsg.type = CY_USB_UVC_VBUS_CHANGE_INTR;
    xQueueSendFromISR(appCtxt.uvcMsgQueue, &(xMsg), &(xHigherPriorityTaskWoken));

    /* Remember that VBus change has happened and disable the interrupt. */
    appCtxt.vbusChangeIntr = true;
    Cy_GPIO_SetInterruptMask(VBUS_DETECT_GPIO_PORT, VBUS_DETECT_GPIO_PIN, 0);
}

/**
 * \name Cy_USB_USBHSInit
 * \brief Initialize USBHS block and attempt device enumeration.
 * \retval None
 */
void Cy_USB_USBHSInit(void)
{
    cy_stc_gpio_pin_config_t pinCfg;
    cy_stc_sysint_t intrCfg;
   
    /* Do all the relevant clock configuration */
    Cy_Fx2g3_InitPeripheralClocks(false, true);

    /* Unlock and then disable the watchdog. */
    Cy_WDT_Unlock();
    Cy_WDT_Disable();

    /* Enable interrupts. */
    __enable_irq();

    memset((void *)&pinCfg, 0, sizeof(pinCfg));

    /* Configure VBus detect GPIO. */
    pinCfg.driveMode = CY_GPIO_DM_HIGHZ;
    pinCfg.hsiom     = HSIOM_SEL_GPIO;
    pinCfg.intEdge   = CY_GPIO_INTR_BOTH;
    pinCfg.intMask   = 0x01UL;
    Cy_GPIO_Pin_Init(VBUS_DETECT_GPIO_PORT, VBUS_DETECT_GPIO_PIN, &pinCfg);

    /* CM0+: Interrupt source to NVIC Mux map
       LVDS - NVIC Mux #0
       VBUS(GPIO#4) - NVIC Mux #2
       SCB0(I2C) - NVIC Mux #3
       SIP0 DMA - NVIC Mux #4
       USBHS Active & DeepSleep - NVIC Mux #5
       DataWire 1 - NVIC Mux #1
	   DataWire 0 - NVIC Mux #6
    */
    /* Register edge detect interrupt for Vbus detect GPIO. */
#if CY_CPU_CORTEX_M4
    intrCfg.intrSrc = VBUS_DETECT_GPIO_INTR;
    intrCfg.intrPriority = 7;
#else
    intrCfg.cm0pSrc = VBUS_DETECT_GPIO_INTR;
    intrCfg.intrSrc = NvicMux2_IRQn;
    intrCfg.intrPriority = 3;
#endif /* CY_CPU_CORTEX_M4 */
    Cy_SysInt_Init(&intrCfg, Cy_USB_VbusDetGpio_ISR);
    NVIC_EnableIRQ(intrCfg.intrSrc);

    /*
     * Enable the DMA buffer RAM
     * need to be enabled before we try to do High BandWidth DMA initialization.
     */
    MAIN_REG->CTRL |= MAIN_REG_CTRL_IP_ENABLED_Msk;

    /* Register the LVDS ISR and enable the interrupt for LVDS. */
#if CY_CPU_CORTEX_M4
    intrCfg.intrSrc = lvds2usb32ss_lvds_int_o_IRQn;
    intrCfg.intrPriority = 6;
#else
    intrCfg.intrSrc = NvicMux0_IRQn;
    intrCfg.intrPriority = 2;
    intrCfg.cm0pSrc = lvds2usb32ss_lvds_int_o_IRQn;
#endif /* CY_CPU_CORTEX_M4 */
    Cy_SysInt_Init(&intrCfg, Cy_LVDS_ISR);
    NVIC_EnableIRQ(intrCfg.intrSrc);

    /* Register the ISR and enable the interrupt for SIP0 DMA adapter. */
#if CY_CPU_CORTEX_M4
    intrCfg.intrSrc = lvds2usb32ss_lvds_dma_adap0_int_o_IRQn;
    intrCfg.intrPriority = 3;
#else
    intrCfg.intrSrc = NvicMux4_IRQn;
    intrCfg.intrPriority = 1;
    intrCfg.cm0pSrc = lvds2usb32ss_lvds_dma_adap0_int_o_IRQn;
#endif /* CY_CPU_CORTEX_M4 */
    Cy_SysInt_Init(&intrCfg, &Cy_LvdsPortDma_ISR);
    NVIC_EnableIRQ(intrCfg.intrSrc);

    /* Register ISR for and enable USBHS Interrupt. */
#if CY_CPU_CORTEX_M4
    intrCfg.intrSrc      = usbhsdev_interrupt_u2d_active_o_IRQn;
    intrCfg.intrPriority = 4;
#else
    intrCfg.cm0pSrc      = usbhsdev_interrupt_u2d_active_o_IRQn;
    intrCfg.intrSrc      = NvicMux5_IRQn;
    intrCfg.intrPriority = 2;
#endif /* CY_CPU_CORTEX_M4 */
    Cy_SysInt_Init(&intrCfg, Cy_USB_HS_ISR);
    NVIC_EnableIRQ(intrCfg.intrSrc);

    /* Register ISR for and enable USBHS Interrupt. */
#if CY_CPU_CORTEX_M4
    intrCfg.intrSrc      = usbhsdev_interrupt_u2d_dpslp_o_IRQn;
    intrCfg.intrPriority = 4;
#else
    intrCfg.cm0pSrc      = usbhsdev_interrupt_u2d_dpslp_o_IRQn;
    intrCfg.intrSrc      = NvicMux5_IRQn;
    intrCfg.intrPriority = 2;
#endif /* CY_CPU_CORTEX_M4 */
    Cy_SysInt_Init(&intrCfg, Cy_USB_HS_ISR);
    NVIC_EnableIRQ(intrCfg.intrSrc);
}

/**
 * \name Cy_UVC_HbDmaInit
 * \brief Initialize HBDMA block.
 * \retval  `true` if HBDMA block is initialized successfully
 *          `false` if HBDMA block is not initialized successfully
 */
bool Cy_UVC_HbDmaInit(void)
{
    cy_en_hbdma_status_t drvstat;
    cy_en_hbdma_mgr_status_t mgrstat;

    /* Initialize the HBW DMA driver layer. */
    drvstat = Cy_HBDma_Init(LVDSSS_LVDS, USB32DEV, &HBW_DrvCtxt, 0, 0);
    if (drvstat != CY_HBDMA_SUCCESS)
    {
        return false;
    }

    /* Setup a HBW DMA descriptor list. */
    mgrstat = Cy_HBDma_DscrList_Create(&HBW_DscrList, 256U);
    if (mgrstat != CY_HBDMA_MGR_SUCCESS)
    {
        return false;
    }

    /* Initialize the DMA buffer manager. We will use 512 KB of space from 0x1C030000 onwards. */
    mgrstat = Cy_HBDma_BufMgr_Create(&HBW_BufMgr, (uint32_t *)0x1C030000UL, 0x80000UL);
    if (mgrstat != CY_HBDMA_MGR_SUCCESS)
    {
        return false;
    }

    /* Initialize the HBW DMA channel manager. */
    mgrstat = Cy_HBDma_Mgr_Init(&HBW_MgrCtxt, &HBW_DrvCtxt, &HBW_DscrList, &HBW_BufMgr);
    if (mgrstat != CY_HBDMA_MGR_SUCCESS)
    {
        return false;
    }

    return true;
}

/**
 * \name Cy_USB_DisableUsbBlock
 * \brief   Function to disable the HBDMA IP block after terminating current
 *          connection.
 * \retval None
 */
void Cy_USB_DisableUsbBlock (void)
{
    Cy_HBDma_DeInit(&HBW_DrvCtxt);
    DBG_APP_INFO("Disabled HBWSS DMA adapters\r\n");
}

/**
 * \name Cy_USB_EnableUsbBlock
 * \brief   Function to enable the HBDMA IP block before enabling a new USB
 *          connection.
 * \retval None
 */
void Cy_USB_EnableUsbBlock (void)
{
    /* Enable the USB DMA adapters and respective interrupts. */
    Cy_HBDma_Init(NULL, USB32DEV, &HBW_DrvCtxt, 0, 0);
}

/**
 * \name Cy_USB_EnableUsbHSConnection
 * \brief Enable USBHS connection
 * \param pAppCtxt Pointer to UVC application context structure.
 * \retval None
 */
bool Cy_USB_EnableUsbHSConnection (cy_stc_usb_app_ctxt_t *pAppCtxt)
{

    DBG_APP_INFO("USB_DEV_HS\r\n");
    Cy_USBD_ConnectDevice(pAppCtxt->pUsbdCtxt, CY_USBD_USB_DEV_HS);
    pAppCtxt->usbConnected = true;
    return true;
}

/**
 * \name Cy_USB_DisableUsbHSConnection
 * \brief Diable USBHS connection
 * \retval None
 */
void Cy_USB_DisableUsbHSConnection (cy_stc_usb_app_ctxt_t *pAppCtxt)
{
    Cy_USBD_DisconnectDevice(pAppCtxt->pUsbdCtxt);
    pAppCtxt->usbConnected = false;
    pAppCtxt->devState     = CY_USB_DEVICE_STATE_DISABLE;
    pAppCtxt->prevDevState = CY_USB_DEVICE_STATE_DISABLE;
}

/**
 * \name main
 * \brief Entry to the application.
 * \retval Does not return.
 */
int main(void)
{
    pCpuDmacBase = ((DMAC_Type *)DMAC_BASE);
    pCpuDw0Base = ((DW_Type *)DW0_BASE);
    pCpuDw1Base = ((DW_Type *)DW1_BASE);

    /* Initialize the PDL driver library and set the clock variables. */
    /* Note: All FX devices, share a common configuration structure. */
    Cy_PDL_Init(&cy_deviceIpBlockCfgFX3G2);

    /* Initialize the device and board peripherals */
    cybsp_init();

    /* Initialize the PDL and register ISR for USB block. */
    Cy_USB_USBHSInit();

#if DEBUG_INFRA_EN
#if !USBFS_LOGS_ENABLE
    /* Initialize the UART for logging. */
    InitUart(LOGGING_SCB_IDX);
#endif /* USBFS_LOGS_ENABLE */

    /*
     * Initialize the logger module. We are using a blocking print option which will
     * output the messages immediately without buffering.
     */
    Cy_Debug_LogInit(&dbgCfg);

    /* Create task for printing logs and check status. */
    xTaskCreate(PrintTaskHandler, "PrintLogTask", 512, NULL, 5, &printLogTaskHandle);

    Cy_SysLib_Delay(500);
    Cy_Debug_AddToLog(1, "********** FX2G3: USB Video Class (UVC) Application ********** \r\n");

    /* Print application, USBD stack and HBDMA version information. */
    Cy_PrintVersionInfo("APP_VERSION: ", APP_VERSION_NUM);
    Cy_PrintVersionInfo("USBD_VERSION: ", USBD_VERSION_NUM);
    Cy_PrintVersionInfo("HBDMA_VERSION: ", HBDMA_VERSION_NUM);
#endif /* DEBUG_INFRA_EN */

#if FPGA_ENABLE
    /* Initialize I2C SCB*/
    Cy_USB_I2CInit ();
#endif /* FPGA_ENABLE */

#if AUDIO_IF_EN
    Cy_UAC_PdmInit();
#endif /* AUDIO_IF_EN */

    memset((uint8_t *)&appCtxt, 0, sizeof(appCtxt));
    memset((uint8_t *)&hsCalCtxt, 0, sizeof(hsCalCtxt));
    memset((uint8_t *)&usbdCtxt, 0, sizeof(usbdCtxt));

    /* Store IP base address in CAL context. */
    hsCalCtxt.pCalBase = MXS40USBHSDEV_USBHSDEV;
    hsCalCtxt.pPhyBase = MXS40USBHSDEV_USBHSPHY;

    Cy_SysLib_Delay(500);

    /* Initialize the HbDma IP and DMA Manager */
    Cy_UVC_HbDmaInit();
    DBG_APP_INFO("Cy_UVC_HbDmaInit done \r\n");

    /* Initialize the USBD layer */
    Cy_USB_USBD_Init(&appCtxt, &usbdCtxt, pCpuDmacBase, &hsCalCtxt,NULL, &HBW_MgrCtxt);
    DBG_APP_INFO("USBD_Init done\r\n");

    Cy_USBD_SetDmaClkFreq(&usbdCtxt, CY_HBDMA_CLK_240_MHZ);

    /* Enable stall cycles between back-to-back AHB accesses to high bandwidth RAM. */
    MAIN_REG->CTRL = (MAIN_REG->CTRL & 0xF00FFFFFUL) | 0x09900000UL;

    /* Initialize the application and create echo device thread. */
    Cy_USB_AppInit(&appCtxt, &usbdCtxt, pCpuDmacBase, pCpuDw0Base, pCpuDw1Base, &HBW_MgrCtxt);
    /* Register USB descriptors with the stack. */
    Cy_USB_RegisterUsbDescriptors(&appCtxt, CY_USBD_USB_DEV_HS);

    DBG_APP_INFO("Scheduler start done\r\n");
    /* Invokes scheduler: Not expected to return. */
    vTaskStartScheduler();
    while (1)
    {
    	Cy_SysLib_Delay(10000);
    	DBG_APP_INFO("Task Idle\r\n");
    }

    return 0;
}

/**
 * \name Cy_OnResetUser
 * \brief Init function which is executed before the load regions in RAM are updated.
 * \details
 * The High BandWidth subsystem needs to be enable here to allow variables
 * placed in the High BandWidth SRAM to be updated.
 * \retval None
 */
void Cy_OnResetUser(void)
{
    Cy_Fx2g3_OnResetInit();
}

/* [] END OF FILE */
