/* --------------------------------------------------------------------------
 * Copyright (c) 2013-2020 Arm Limited (or its affiliates). All
 * rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 *
 * $Date:        10. Januar 2020
 * $Revision:    V2.9
 *
 * Driver:       Driver_SPI0, Driver_SPI1, Driver_SPI2
 * Configured:   via RTE_Device.h configuration file
 * Project:      SPI (SSP used for SPI) Driver for NXP LPC17xx
 * --------------------------------------------------------------------------
 * Use the following configuration settings in the middleware component
 * to connect to this driver.
 *
 *   Configuration Setting                 Value   SPI Interface
 *   ---------------------                 -----   -------------
 *   Connect to hardware via Driver_SPI# = 0       use SPI0 (SSP0)
 *   Connect to hardware via Driver_SPI# = 1       use SPI1 (SSP1)
 *   Connect to hardware via Driver_SPI# = 2       use SPI2 (SPI)
 * -------------------------------------------------------------------------- */

/* History:
 *  Version 2.10
 *    - Fixed build issue with LPC1700
 *  Version 2.9
 *    - Removed minor Compiler warnings
 *  Version 2.8
 *    - Removed Arm Compiler warnings
 *    - Made Pin configuration const
 *  Version 2.7
 *    - MISO and MOSI pins can be configured as "not used"
 *  Version 2.6
 *    - Updated Pin configuration
 *    - Driver update to work with GPDMA_LPC17xx ver.: 1.3
 *  Version 2.5
 *    - Corrected ssel pin GPIO mode configuration (for LPC177x_8x)
 *  Version 2.4
 *    - Corrected PowerControl function for conditional Power full (driver must be initialized)
 *    - Corrected Bus Speed configuration
 *    - Updated pin handling
 *  Version 2.3
 *    - PowerControl for Power OFF and Uninitialize functions made unconditional.
 *    - Corrected status bit-field handling, to prevent race conditions.
 *    - Corrected ARM_SPI_EVENT_DATA_LOST event handling in slave mode
 *  Version 2.2
 *    - Changed include for renamed RTE_Device_LPC177x_8x.h to RTE_Device.h
 *    - Updated Control functions
 *    - GPDMA initialization and uninitialization
 *  Version 2.1
 *    - Added DMA support
 *  Version 2.0
 *    - Initial CMSIS Driver API V2.0 release
 */

#include <string.h>

#include "SSP_LPC17xx.h"

#include "GPIO_LPC17xx.h"
#include "GPDMA_LPC17xx.h"
#include "Driver_SPI.h"

void SSP0_GPDMA_Tx_SignalEvent (uint32_t event);
void SSP0_GPDMA_Rx_SignalEvent (uint32_t event);
void SSP1_GPDMA_Tx_SignalEvent (uint32_t event);
void SSP1_GPDMA_Rx_SignalEvent (uint32_t event);
#if defined (LPC177x_8x)
void SSP2_GPDMA_Tx_SignalEvent (uint32_t event);
void SSP2_GPDMA_Rx_SignalEvent (uint32_t event);
#endif

#define ARM_SPI_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(2,10)   // driver version

#if ((defined(RTE_Drivers_SPI0) || defined(RTE_Drivers_SPI1) || defined(RTE_Drivers_SPI2)) && (!RTE_SSP0) && (!RTE_SSP1) && (!RTE_SSP2))
#error "SSP not configured in RTE_Device.h!"
#endif
#if ((RTE_SSP0) &&                                  \
    ((RTE_SSP0_DMA_TX_EN && !RTE_SSP0_DMA_RX_EN) || \
     (RTE_SSP0_DMA_RX_EN && !RTE_SSP0_DMA_TX_EN)))
#error "Both Tx and Rx DMA for SSP0 have to be enabled or disabled in RTE_Device.h!"
#endif
#if ((RTE_SSP1) &&                                  \
    ((RTE_SSP1_DMA_TX_EN && !RTE_SSP1_DMA_RX_EN) || \
     (RTE_SSP1_DMA_RX_EN && !RTE_SSP1_DMA_TX_EN)))
#error "Both Tx and Rx DMA for SSP1 have to be enabled or disabled in RTE_Device.h!"
#endif
#if defined (LPC177x_8x)
#if ((RTE_SSP2) &&                                  \
    ((RTE_SSP2_DMA_TX_EN && !RTE_SSP2_DMA_RX_EN) || \
     (RTE_SSP2_DMA_RX_EN && !RTE_SSP2_DMA_TX_EN)))
#error "Both Tx and Rx DMA for SSP2 have to be enabled or disabled in RTE_Device.h!"
#endif
#endif

#if defined (LPC175x_6x)
  // Peripheral clock divider definitions
  #define PCLKSEL_CCLK_DIV_1   (1U)
  #define PCLKSEL_CCLK_DIV_2   (2U)
  #define PCLKSEL_CCLK_DIV_4   (0U)
  #define PCLKSEL_CCLK_DIV_8   (3U)

  // SSP peripheral clock selection bit position definitions
  #define SSP0_PCLKSEL1_POS    (10U)
  #define SSP1_PCLKSEL0_POS    (20U)
#endif

// Driver Version
static const ARM_DRIVER_VERSION DriverVersion = {
  ARM_SPI_API_VERSION,
  ARM_SPI_DRV_VERSION
};

// Driver Capabilities
static const ARM_SPI_CAPABILITIES DriverCapabilities = {
  0U, // Simplex Mode (Master and Slave)
  1U, // TI Synchronous Serial Interface
  1U, // Microwire Interface
  0U  // Signal Mode Fault event: \ref ARM_SPI_EVENT_MODE_FAULT
#if (defined(ARM_SPI_API_VERSION) && (ARM_SPI_API_VERSION >= 0x202U))
, 0U  /* Reserved bits */
#endif
};

#if (RTE_SSP0)
static SSP_INFO          SSP0_Info = { 0 };
static SSP_TRANSFER_INFO SSP0_Xfer = { 0 };
static const PIN         SSPO_pin_sck  = {RTE_SSP0_SCK_PORT,  RTE_SSP0_SCK_BIT};
#if (RTE_SSP0_MISO_PIN_EN != 0U)
static const PIN         SSPO_pin_miso = {RTE_SSP0_MISO_PORT, RTE_SSP0_MISO_BIT};
#endif
#if (RTE_SSP0_MOSI_PIN_EN != 0U)
static const PIN         SSPO_pin_mosi = {RTE_SSP0_MOSI_PORT, RTE_SSP0_MOSI_BIT};
#endif
#if (RTE_SSP0_SSEL_PIN_EN != 0U)
static const PIN         SSPO_pin_ssel = {RTE_SSP0_SSEL_PORT, RTE_SSP0_SSEL_BIT};
#endif

static SSP_RESOURCES SSP0_Resources = {
     LPC_SSP0,
  {
#if (RTE_SSP0_SSEL_PIN_EN != 0U)
    &SSPO_pin_ssel,
#else
    NULL,
#endif
    &SSPO_pin_sck,
#if (RTE_SSP0_MISO_PIN_EN != 0U)
    &SSPO_pin_miso,
#else
    NULL,
#endif
#if (RTE_SSP0_MOSI_PIN_EN != 0U)
    &SSPO_pin_mosi,
#else
    NULL,
#endif
#if (RTE_SSP0_SSEL_PIN_EN != 0U)
    RTE_SSP0_SSEL_FUNC,
#else
    0U,
#endif
    RTE_SSP0_SCK_FUNC,
#if (RTE_SSP0_MISO_PIN_EN != 0U)
    RTE_SSP0_MISO_FUNC,
#else
    0U,
#endif
#if (RTE_SSP0_MOSI_PIN_EN != 0U)
    RTE_SSP0_MOSI_FUNC,
#else
    0U,
#endif
#if defined (LPC177x_8x)
    0U,
    0U,
    0U,
    0U
#endif
  },
  { (1U << 21),
    &(LPC_SC->PCONP),
#if defined (LPC175x_6x)
    SSP0_PCLKSEL1_POS,
    PCLKSEL_CCLK_DIV_1,
    &(LPC_SC->PCLKSEL1)
#endif
},
  { RTE_SSP0_DMA_TX_EN,
    RTE_SSP0_DMA_TX_CH,
    GPDMA_CONN_SSP0_Tx,
    0U,
    SSP0_GPDMA_Tx_SignalEvent,
    RTE_SSP0_DMA_RX_EN,
    RTE_SSP0_DMA_RX_CH,
    GPDMA_CONN_SSP0_Rx,
    0U,
    SSP0_GPDMA_Rx_SignalEvent },
    SSP0_IRQn,
   &SSP0_Info,
   &SSP0_Xfer
};

#endif /* RTE_SSP0 */

#if (RTE_SSP1)
static SSP_INFO          SSP1_Info = { 0 };
static SSP_TRANSFER_INFO SSP1_Xfer = { 0 };
static const PIN         SSP1_pin_sck  = {RTE_SSP1_SCK_PORT,  RTE_SSP1_SCK_BIT};
#if (RTE_SSP1_MISO_PIN_EN != 0U)
static const PIN         SSP1_pin_miso = {RTE_SSP1_MISO_PORT, RTE_SSP1_MISO_BIT};
#endif
#if (RTE_SSP1_MOSI_PIN_EN != 0U)
static const PIN         SSP1_pin_mosi = {RTE_SSP1_MOSI_PORT, RTE_SSP1_MOSI_BIT};
#endif
#if (RTE_SSP1_SSEL_PIN_EN != 0U)
static const PIN         SSP1_pin_ssel = {RTE_SSP1_SSEL_PORT, RTE_SSP1_SSEL_BIT};
#endif

static SSP_RESOURCES SSP1_Resources = {
     LPC_SSP1,
  {
#if (RTE_SSP1_SSEL_PIN_EN != 0U)
    &SSP1_pin_ssel,
#else
    NULL,
#endif
    &SSP1_pin_sck,
#if (RTE_SSP1_MISO_PIN_EN != 0U)
    &SSP1_pin_miso,
#else
    NULL,
#endif
#if (RTE_SSP1_MOSI_PIN_EN != 0U)
    &SSP1_pin_mosi,
#else
    NULL,
#endif
#if (RTE_SSP1_SSEL_PIN_EN != 0U)
    RTE_SSP1_SSEL_FUNC,
#else
    0U,
#endif
    RTE_SSP1_SCK_FUNC,
#if (RTE_SSP1_MISO_PIN_EN != 0U)
    RTE_SSP1_MISO_FUNC,
#else
    0U,
#endif
#if (RTE_SSP1_MOSI_PIN_EN != 0U)
    RTE_SSP1_MOSI_FUNC,
#else
    0U,
#endif
#if defined (LPC177x_8x)
    RTE_SSP1_SCK_IO_WA,
    RTE_SSP1_MISO_IO_WA,
    RTE_SSP1_MOSI_IO_WA,
    0U
#endif
  },
  { (1U << 10),
    &(LPC_SC->PCONP),
#if defined (LPC175x_6x)
    SSP1_PCLKSEL0_POS,
    PCLKSEL_CCLK_DIV_1,
    &(LPC_SC->PCLKSEL0)
#endif
},
  { RTE_SSP1_DMA_TX_EN,
    RTE_SSP1_DMA_TX_CH,
    GPDMA_CONN_SSP1_Tx,
    0U,
    SSP1_GPDMA_Tx_SignalEvent,
    RTE_SSP1_DMA_RX_EN,
    RTE_SSP1_DMA_RX_CH,
    GPDMA_CONN_SSP1_Rx,
    0U,
    SSP1_GPDMA_Rx_SignalEvent },
  SSP1_IRQn,
 &SSP1_Info,
 &SSP1_Xfer
};
#endif /* RTE_SSP1 */

#if defined(RTE_SSP2) && (RTE_SSP2)
static SSP_INFO          SSP2_Info = { 0 };
static SSP_TRANSFER_INFO SSP2_Xfer = { 0 };
static const PIN         SSP2_pin_sck  = {RTE_SSP2_SCK_PORT,  RTE_SSP2_SCK_BIT};
#if (RTE_SSP2_MISO_PIN_EN != 0U)
static const PIN         SSP2_pin_miso = {RTE_SSP2_MISO_PORT, RTE_SSP2_MISO_BIT};
#endif
#if (RTE_SSP2_MOSI_PIN_EN != 0U)
static const PIN         SSP2_pin_mosi = {RTE_SSP2_MOSI_PORT, RTE_SSP2_MOSI_BIT};
#endif
#if (RTE_SSP2_SSEL_PIN_EN != 0U)
static const PIN          SSP2_pin_ssel = {RTE_SSP2_SSEL_PORT, RTE_SSP2_SSEL_BIT};
#endif

static SSP_RESOURCES     SSP2_Resources = {
     LPC_SSP2,
  {
#if (RTE_SSP2_SSEL_PIN_EN != 0U)
    &SSP2_pin_ssel,
#else
    NULL,
#endif
    &SSP2_pin_sck,
#if (RTE_SSP2_MISO_PIN_EN != 0U)
    &SSP2_pin_miso,
#else
    NULL,
#endif
#if (RTE_SSP2_MOSI_PIN_EN != 0U)
    &SSP2_pin_mosi,
#else
    NULL,
#endif
#if (RTE_SSP2_SSEL_PIN_EN != 0U)
    RTE_SSP2_SSEL_FUNC,
#else
    0U,
#endif
    RTE_SSP2_SCK_FUNC,
#if (RTE_SSP2_MISO_PIN_EN != 0U)
    RTE_SSP2_MISO_FUNC,
#else
    0U,
#endif
#if (RTE_SSP2_MOSI_PIN_EN != 0U)
    RTE_SSP2_MOSI_FUNC,
#else
    0U,
#endif
    0U,
    0U,
    0U,
    0U
  },
  { (1U << 20),
    &(LPC_SC->PCONP),
},
  { RTE_SSP2_DMA_TX_EN,
    RTE_SSP2_DMA_TX_CH,
    GPDMA_CONN_SSP2_Tx,
    0U,
    SSP2_GPDMA_Tx_SignalEvent,
    RTE_SSP2_DMA_RX_EN,
    RTE_SSP2_DMA_RX_CH,
    GPDMA_CONN_SSP2_Rx,
    0U,
    SSP2_GPDMA_Rx_SignalEvent },
    SSP2_IRQn,
   &SSP2_Info,
   &SSP2_Xfer
};
#endif /* RTE_SPI */

// Local Function
/**
  \fn          uint32_t GetSSPClockFreq (SSP_RESOURCES *ssp)
  \brief       Get current value of SSP Peripheral Clock
  \param[in]   ssp       Pointer to SSP resources
  \returns     Value of SSP clock
*/
static uint32_t GetSSPClockFreq (SSP_RESOURCES *ssp) {
#if defined (LPC175x_6x)
  uint32_t div, clk, clksel;

  clksel = (*(ssp->clk.peri_cfg) >> ssp->clk.peri_cfg_pos) & 0x00000003U;
  switch(clksel)
  {
    case 1U:
      div = 1U;
      break;
    case 2U:
      div = 2U;
      break;
    case 3U:
      div = 8U;
      break;
    case 0U:
    default:
      div = 4U;
      break;
  }
  clk = SystemCoreClock / div;
  return(clk);
#elif defined (LPC177x_8x)
  (void)ssp;
  return(PeripheralClock);
#endif
}


/**
  \fn          ARM_DRIVER_VERSION SSP_GetVersion (void)
  \brief       Get SSP driver version.
  \return      \ref ARM_DRV_VERSION
*/
static ARM_DRIVER_VERSION SSP_GetVersion (void) {
  return DriverVersion;
}

/**
  \fn          ARM_SPI_CAPABILITIES SSP_GetCapabilities (void)
  \brief       Get driver capabilities.
  \return      \ref ARM_SPI_CAPABILITIES
*/
static ARM_SPI_CAPABILITIES SSP_GetCapabilities (void) {
  return DriverCapabilities;
}

/**
  \fn          int32_t SSPx_Initialize (ARM_SPI_SignalEvent_t cb_event, SSP_RESOURCES *ssp)
  \brief       Initialize SSP Interface.
  \param[in]   cb_event  Pointer to \ref ARM_SPI_SignalEvent
  \param[in]   ssp       Pointer to SSP resources
  \return      \ref execution_status
*/
static int32_t SSPx_Initialize (ARM_SPI_SignalEvent_t cb_event, SSP_RESOURCES *ssp) {

  if (ssp->info->state & SSP_INITIALIZED) { return ARM_DRIVER_OK; }

  // Initialize SSP Run-Time Resources
  ssp->info->cb_event          = cb_event;
  ssp->info->status.busy       = 0U;
  ssp->info->status.data_lost  = 0U;
  ssp->info->status.mode_fault = 0U;

  // Clear transfer information
  memset(ssp->xfer, 0, sizeof(SSP_TRANSFER_INFO));

#if defined (LPC175x_6x)
  PIN_Configure (ssp->pin.sck->Portnum,  ssp->pin.sck->Pinnum,  ssp->pin.sck_func,  PIN_PINMODE_PULLUP, PIN_PINMODE_NORMAL);
  if (ssp->pin.miso != NULL) {
    PIN_Configure (ssp->pin.miso->Portnum, ssp->pin.miso->Pinnum, ssp->pin.miso_func, PIN_PINMODE_PULLUP, PIN_PINMODE_NORMAL);
  }
  if (ssp->pin.mosi != NULL) {
    PIN_Configure (ssp->pin.mosi->Portnum, ssp->pin.mosi->Pinnum, ssp->pin.mosi_func, PIN_PINMODE_PULLUP, PIN_PINMODE_NORMAL);
  }
#elif defined (LPC177x_8x)
  PIN_Configure (ssp->pin.sck->Portnum,  ssp->pin.sck->Pinnum,  ssp->pin.sck_func  | IOCON_HYS_ENABLE | ((ssp->pin.sck_io_wa)  ?\
                                                       (IOCON_10ns_FILTER_DISABLE | IOCON_DIGITIAL_MODE) : IOCON_MODE_PULLUP));
  if (ssp->pin.miso != NULL) {
    PIN_Configure (ssp->pin.miso->Portnum, ssp->pin.miso->Pinnum, ssp->pin.miso_func | IOCON_HYS_ENABLE | ((ssp->pin.miso_io_wa) ?\
                                                         (IOCON_10ns_FILTER_DISABLE | IOCON_DIGITIAL_MODE) : IOCON_MODE_PULLUP));
  }
  if (ssp->pin.mosi != NULL) {
    PIN_Configure (ssp->pin.mosi->Portnum, ssp->pin.mosi->Pinnum, ssp->pin.mosi_func | IOCON_HYS_ENABLE | ((ssp->pin.mosi_io_wa) ?\
                                                         (IOCON_10ns_FILTER_DISABLE | IOCON_DIGITIAL_MODE) : IOCON_MODE_PULLUP));
  }
#endif

  // Configure DMA if it will be used
  if (ssp->dma.tx_en || ssp->dma.rx_en) { GPDMA_Initialize (); }

  if (ssp->dma.tx_en) { GPDMA_PeripheralSelect (ssp->dma.tx_req, 0U); }
  if (ssp->dma.rx_en) { GPDMA_PeripheralSelect (ssp->dma.rx_req, 0U); }

  ssp->info->state = SSP_INITIALIZED;   // SSP is initialized

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t SSPx_Uninitialize (SSP_RESOURCES *ssp)
  \brief       De-initialize SSP Interface.
  \param[in]   ssp  Pointer to SSP resources
  \return      \ref execution_status
*/
static int32_t SSPx_Uninitialize (SSP_RESOURCES *ssp) {

#if defined (LPC175x_6x)
  /* Unconfigure SSP pins */
  if (ssp->pin.ssel != NULL) { PIN_Configure (ssp->pin.ssel->Portnum, ssp->pin.ssel->Pinnum, 0U, 0U, 0U); }

  PIN_Configure (ssp->pin.sck->Portnum,  ssp->pin.sck->Pinnum,  0U, 0U, 0U);
  if (ssp->pin.miso != NULL) {
    PIN_Configure (ssp->pin.miso->Portnum, ssp->pin.miso->Pinnum, 0U, 0U, 0U);
  }
  if (ssp->pin.mosi != NULL) {
    PIN_Configure (ssp->pin.mosi->Portnum, ssp->pin.mosi->Pinnum, 0U, 0U, 0U);
  }
#elif defined (LPC177x_8x)
  PIN_Configure (ssp->pin.sck->Portnum,  ssp->pin.sck->Pinnum,  IOCON_HYS_ENABLE | ((ssp->pin.sck_io_wa)  ?\
                                                       (IOCON_DIGITIAL_MODE) : IOCON_MODE_PULLUP));
  if (ssp->pin.miso != NULL) {
    PIN_Configure (ssp->pin.miso->Portnum, ssp->pin.miso->Pinnum, IOCON_HYS_ENABLE | ((ssp->pin.miso_io_wa) ?\
                                                         (IOCON_DIGITIAL_MODE) : IOCON_MODE_PULLUP));
  }
  if (ssp->pin.mosi != NULL) {
    PIN_Configure (ssp->pin.mosi->Portnum, ssp->pin.mosi->Pinnum, IOCON_HYS_ENABLE | ((ssp->pin.mosi_io_wa) ?\
                                                         (IOCON_DIGITIAL_MODE) : IOCON_MODE_PULLUP));
  }
#endif

  // Uninitialize DMA
  if (ssp->dma.tx_en || ssp->dma.rx_en) { GPDMA_Uninitialize (); }

  ssp->info->state = 0U;                // SSP is uninitialized

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t SSPx_PowerControl (ARM_POWER_STATE state, SSP_RESOURCES *ssp)
  \brief       Control SSP Interface Power.
  \param[in]   state  Power state
  \param[in]   ssp    Pointer to SSP resources
  \return      \ref execution_status
*/
static int32_t SSPx_PowerControl (ARM_POWER_STATE state, SSP_RESOURCES *ssp) {

  if ((state != ARM_POWER_OFF)  &&
      (state != ARM_POWER_FULL) &&
      (state != ARM_POWER_LOW)) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  switch (state) {
    case ARM_POWER_OFF:
      NVIC_DisableIRQ ((IRQn_Type)ssp->irq_num);   // Disable SSP IRQ in NVIC

      // Enable power to SSPx block
      *(ssp->clk.reg_pwr) |= ssp->clk.reg_pwr_val;

      if (ssp->info->status.busy) {
        // If DMA mode - disable DMA channel
        if (ssp->dma.tx_en) { GPDMA_ChannelDisable (ssp->dma.tx_ch); }
        // If DMA mode - disable DMA channel
        if (ssp->dma.rx_en) { GPDMA_ChannelDisable (ssp->dma.rx_ch); }
      }

      // Reset register values
      ssp->reg->IMSC  = 0U;
      ssp->reg->DMACR = 0U;
      ssp->reg->CR0   = 0U;
      ssp->reg->CR1   = 0U;
      ssp->reg->CPSR  = 0U;
      ssp->reg->ICR   = 3U;

      // Disable power to SSPx block
      *(ssp->clk.reg_pwr) &= ~ssp->clk.reg_pwr_val;

      // Clear pending USART interrupts in NVIC
      NVIC_ClearPendingIRQ((IRQn_Type)ssp->irq_num);

      // Reset SSP Run-Time Resources
      ssp->info->status.busy       = 0U;
      ssp->info->status.data_lost  = 0U;
      ssp->info->status.mode_fault = 0U;

      // Clear transfer information
      memset(ssp->xfer, 0, sizeof(SSP_TRANSFER_INFO));

      ssp->info->state &= ~SSP_POWERED; // SSP is not powered
      break;

    case ARM_POWER_FULL:
      if ((ssp->info->state & SSP_INITIALIZED) == 0U) { return ARM_DRIVER_ERROR; }
      if ((ssp->info->state & SSP_POWERED)     != 0U) { return ARM_DRIVER_OK; }

      // Enable power to SSPx block
      *(ssp->clk.reg_pwr) |= ssp->clk.reg_pwr_val;
#if defined (LPC175x_6x)
      // Configure SSP Clock
      *(ssp->clk.peri_cfg)  &= ~(3U << ssp->clk.peri_cfg_pos);
      *(ssp->clk.peri_cfg)  |=  (ssp->clk.peri_cfg_val << ssp->clk.peri_cfg_pos);
#endif
      ssp->reg->IMSC  = 0U;             // Disable SSP interrupts
      ssp->reg->ICR   = 3U;             // Clear SSP interrupts

      // Reset SSP Run-Time Resources
      ssp->info->status.busy       = 0U;
      ssp->info->status.data_lost  = 0U;
      ssp->info->status.mode_fault = 0U;

      ssp->info->state |=  SSP_POWERED; // SSP is powered

      // Clear RX FIFO
      while ((ssp->reg->SR & SSPx_SR_RNE) != 0U) { ssp->reg->DR; }

      // Enable DMA
      if (ssp->dma.tx_en) { ssp->reg->DMACR |= SSPx_DMACR_TXDMAE; }
      if (ssp->dma.rx_en) { ssp->reg->DMACR |= SSPx_DMACR_RXDMAE; }

      NVIC_ClearPendingIRQ ((IRQn_Type)ssp->irq_num);
      NVIC_EnableIRQ ((IRQn_Type)ssp->irq_num);    // Enable SSP IRQ in NVIC
      break;
    case ARM_POWER_LOW:
      return ARM_DRIVER_ERROR_UNSUPPORTED;
  }

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t SSPx_Send (const void *data, uint32_t num, SSP_RESOURCES *ssp)
  \brief       Start sending data to SSP transmitter.
  \param[in]   data  Pointer to buffer with data to send to SSP transmitter
  \param[in]   num   Number of data items to send
  \param[in]   ssp   Pointer to SSP resources
  \return      \ref execution_status
*/
static int32_t SSPx_Send (const void *data, uint32_t num, SSP_RESOURCES *ssp) {
  static uint32_t dummy_data;

  if ((data == NULL) || (num == 0U))        { return ARM_DRIVER_ERROR_PARAMETER; }
  if (!(ssp->info->state & SSP_CONFIGURED)) { return ARM_DRIVER_ERROR; }
  if (  ssp->info->status.busy)             { return ARM_DRIVER_ERROR_BUSY; }
  ssp->info->status.busy       = 1U;
  ssp->info->status.data_lost  = 0U;
  ssp->info->status.mode_fault = 0U;

  ssp->xfer->rx_buf = NULL;
  ssp->xfer->tx_buf = (uint8_t *)(uint32_t)data;

  ssp->xfer->num    = num;
  ssp->xfer->rx_cnt = 0U;
  ssp->xfer->tx_cnt = 0U;

  if (ssp->dma.tx_en && ssp->dma.rx_en) {
    if (GPDMA_ChannelConfigure (ssp->dma.rx_ch,
                               (uint32_t)&ssp->reg->DR,
                               (uint32_t)&dummy_data,
                                num,
                                GPDMA_CH_CONTROL_SBSIZE(GPDMA_BSIZE_1)                                  |
                                GPDMA_CH_CONTROL_DBSIZE(GPDMA_BSIZE_1)                                  |
                                GPDMA_CH_CONTROL_SWIDTH((uint32_t)((ssp->reg->CR0 & SSPx_CR0_DSS) > 7)) |
                                GPDMA_CH_CONTROL_DWIDTH((uint32_t)((ssp->reg->CR0 & SSPx_CR0_DSS) > 7)) |
                                GPDMA_CH_CONTROL_I,
                                GPDMA_CH_CONFIG_SRC_PERI((uint32_t)ssp->dma.rx_req)                     |
                                GPDMA_CH_CONFIG_FLOWCNTRL(GPDMA_TRANSFER_P2M_CTRL_DMA)                  |
                                GPDMA_CH_CONFIG_IE                                                      |
                                GPDMA_CH_CONFIG_ITC                                                     |
                                GPDMA_CH_CONFIG_E,
                                ssp->dma.rx_callback) == -1) {
      return ARM_DRIVER_ERROR;
    }
    if (GPDMA_ChannelConfigure (ssp->dma.tx_ch,
                               (uint32_t)data,
                               (uint32_t)&ssp->reg->DR,
                                num,
                                GPDMA_CH_CONTROL_SBSIZE(GPDMA_BSIZE_1)                                  |
                                GPDMA_CH_CONTROL_DBSIZE(GPDMA_BSIZE_1)                                  |
                                GPDMA_CH_CONTROL_SWIDTH((uint32_t)((ssp->reg->CR0 & SSPx_CR0_DSS) > 7)) |
                                GPDMA_CH_CONTROL_DWIDTH((uint32_t)((ssp->reg->CR0 & SSPx_CR0_DSS) > 7)) |
                                GPDMA_CH_CONTROL_SI                                                     |
                                GPDMA_CH_CONTROL_I,
                                GPDMA_CH_CONFIG_DEST_PERI(ssp->dma.tx_req)                              |
                                GPDMA_CH_CONFIG_FLOWCNTRL(GPDMA_TRANSFER_M2P_CTRL_DMA)                  |
                                GPDMA_CH_CONFIG_IE                                                      |
                                GPDMA_CH_CONFIG_ITC                                                     |
                                GPDMA_CH_CONFIG_E,
                                ssp->dma.tx_callback) == -1) {
      return ARM_DRIVER_ERROR;
    }
  } else {
    ssp->reg->IMSC = SSPx_IMSC_TXIM | SSPx_IMSC_RXIM | SSPx_IMSC_RTIM | SSPx_IMSC_RORIM;
  }

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t SSPx_Receive (void *data, uint32_t num, SSP_RESOURCES *ssp)
  \brief       Start receiving data from SSP receiver.
  \param[out]  data  Pointer to buffer for data to receive from SSP receiver
  \param[in]   num   Number of data items to receive
  \param[in]   ssp   Pointer to SSP resources
  \return      \ref execution_status
*/
static int32_t SSPx_Receive (void *data, uint32_t num, SSP_RESOURCES *ssp) {
  static uint32_t dummy_data;

  if ((data == NULL) || (num == 0U))        { return ARM_DRIVER_ERROR_PARAMETER; }
  if (!(ssp->info->state & SSP_CONFIGURED)) { return ARM_DRIVER_ERROR; }
  if (  ssp->info->status.busy)             { return ARM_DRIVER_ERROR_BUSY; }
  ssp->info->status.busy       = 1U;
  ssp->info->status.data_lost  = 0U;
  ssp->info->status.mode_fault = 0U;

  dummy_data        = ssp->xfer->def_val;

  ssp->xfer->rx_buf = (uint8_t *)data;
  ssp->xfer->tx_buf = NULL;

  ssp->xfer->num    = num;
  ssp->xfer->rx_cnt = 0U;
  ssp->xfer->tx_cnt = 0U;

  if (ssp->dma.tx_en && ssp->dma.rx_en) {
    if (GPDMA_ChannelConfigure (ssp->dma.rx_ch,
                               (uint32_t)&ssp->reg->DR,
                               (uint32_t)data,
                                num,
                                GPDMA_CH_CONTROL_SBSIZE(GPDMA_BSIZE_1)                                  |
                                GPDMA_CH_CONTROL_DBSIZE(GPDMA_BSIZE_1)                                  |
                                GPDMA_CH_CONTROL_SWIDTH((uint32_t)((ssp->reg->CR0 & SSPx_CR0_DSS) > 7)) |
                                GPDMA_CH_CONTROL_DWIDTH((uint32_t)((ssp->reg->CR0 & SSPx_CR0_DSS) > 7)) |
                                GPDMA_CH_CONTROL_DI                                                     |
                                GPDMA_CH_CONTROL_I,
                                GPDMA_CH_CONFIG_SRC_PERI((uint32_t)ssp->dma.rx_req)                     |
                                GPDMA_CH_CONFIG_FLOWCNTRL(GPDMA_TRANSFER_P2M_CTRL_DMA)                  |
                                GPDMA_CH_CONFIG_IE                                                      |
                                GPDMA_CH_CONFIG_ITC                                                     |
                                GPDMA_CH_CONFIG_E,
                                ssp->dma.rx_callback) == -1) {
      return ARM_DRIVER_ERROR;
    }
    if (GPDMA_ChannelConfigure (ssp->dma.tx_ch,
                               (uint32_t)&dummy_data,
                               (uint32_t)&ssp->reg->DR,
                                num,
                                GPDMA_CH_CONTROL_SBSIZE(GPDMA_BSIZE_1)                                  |
                                GPDMA_CH_CONTROL_DBSIZE(GPDMA_BSIZE_1)                                  |
                                GPDMA_CH_CONTROL_SWIDTH((uint32_t)((ssp->reg->CR0 & SSPx_CR0_DSS) > 7)) |
                                GPDMA_CH_CONTROL_DWIDTH((uint32_t)((ssp->reg->CR0 & SSPx_CR0_DSS) > 7)) |
                                GPDMA_CH_CONTROL_I,
                                GPDMA_CH_CONFIG_DEST_PERI(ssp->dma.tx_req)                              |
                                GPDMA_CH_CONFIG_FLOWCNTRL(GPDMA_TRANSFER_M2P_CTRL_DMA)                  |
                                GPDMA_CH_CONFIG_IE                                                      |
                                GPDMA_CH_CONFIG_ITC                                                     |
                                GPDMA_CH_CONFIG_E,
                                ssp->dma.tx_callback) == -1) {
      return ARM_DRIVER_ERROR;
    }
  } else {
    ssp->reg->IMSC = SSPx_IMSC_TXIM | SSPx_IMSC_RXIM | SSPx_IMSC_RTIM | SSPx_IMSC_RORIM;
  }

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t SSPx_Transfer (const void          *data_out,
                                            void          *data_in,
                                            uint32_t       num,
                                            SSP_RESOURCES *ssp)
  \brief       Start sending/receiving data to/from SSP transmitter/receiver.
  \param[in]   data_out  Pointer to buffer with data to send to SSP transmitter
  \param[out]  data_in   Pointer to buffer for data to receive from SSP receiver
  \param[in]   num       Number of data items to transfer
  \param[in]   ssp       Pointer to SSP resources
  \return      \ref execution_status
*/
static int32_t SSPx_Transfer (const void *data_out, void *data_in, uint32_t num, SSP_RESOURCES *ssp) {

  if ((data_out == NULL) || (data_in == NULL) || (num == 0U)) { return ARM_DRIVER_ERROR_PARAMETER; }
  if (!(ssp->info->state & SSP_CONFIGURED))                   { return ARM_DRIVER_ERROR; }
  if (  ssp->info->status.busy)                               { return ARM_DRIVER_ERROR_BUSY; }
  ssp->info->status.busy       = 1U;
  ssp->info->status.data_lost  = 0U;
  ssp->info->status.mode_fault = 0U;

  ssp->xfer->rx_buf = (uint8_t *)data_in;
  ssp->xfer->tx_buf = (uint8_t *)(uint32_t)data_out;

  ssp->xfer->num    = num;
  ssp->xfer->rx_cnt = 0U;
  ssp->xfer->tx_cnt = 0U;

  if (ssp->dma.tx_en && ssp->dma.rx_en) {
    if (GPDMA_ChannelConfigure (ssp->dma.rx_ch,
                               (uint32_t)&ssp->reg->DR,
                               (uint32_t)data_in,
                                num,
                                GPDMA_CH_CONTROL_SBSIZE(GPDMA_BSIZE_1)                                  |
                                GPDMA_CH_CONTROL_DBSIZE(GPDMA_BSIZE_1)                                  |
                                GPDMA_CH_CONTROL_SWIDTH((uint32_t)((ssp->reg->CR0 & SSPx_CR0_DSS) > 7)) |
                                GPDMA_CH_CONTROL_DWIDTH((uint32_t)((ssp->reg->CR0 & SSPx_CR0_DSS) > 7)) |
                                GPDMA_CH_CONTROL_DI                                                     |
                                GPDMA_CH_CONTROL_I,
                                GPDMA_CH_CONFIG_SRC_PERI((uint32_t)ssp->dma.rx_req)                     |
                                GPDMA_CH_CONFIG_FLOWCNTRL(GPDMA_TRANSFER_P2M_CTRL_DMA)                  |
                                GPDMA_CH_CONFIG_IE                                                      |
                                GPDMA_CH_CONFIG_ITC                                                     |
                                GPDMA_CH_CONFIG_E,
                                ssp->dma.rx_callback) == -1) {
      return ARM_DRIVER_ERROR;
    }

    if (GPDMA_ChannelConfigure (ssp->dma.tx_ch,
                               (uint32_t)data_out,
                               (uint32_t)&ssp->reg->DR,
                                num,
                                GPDMA_CH_CONTROL_SBSIZE(GPDMA_BSIZE_1)                                  |
                                GPDMA_CH_CONTROL_DBSIZE(GPDMA_BSIZE_1)                                  |
                                GPDMA_CH_CONTROL_SWIDTH((uint32_t)((ssp->reg->CR0 & SSPx_CR0_DSS) > 7)) |
                                GPDMA_CH_CONTROL_DWIDTH((uint32_t)((ssp->reg->CR0 & SSPx_CR0_DSS) > 7)) |
                                GPDMA_CH_CONTROL_SI                                                     |
                                GPDMA_CH_CONTROL_I,
                                GPDMA_CH_CONFIG_DEST_PERI(ssp->dma.tx_req)                              |
                                GPDMA_CH_CONFIG_FLOWCNTRL(GPDMA_TRANSFER_M2P_CTRL_DMA)                  |
                                GPDMA_CH_CONFIG_IE                                                      |
                                GPDMA_CH_CONFIG_ITC                                                     |
                                GPDMA_CH_CONFIG_E,
                                ssp->dma.tx_callback) == -1) {
      return ARM_DRIVER_ERROR;
    }
  } else {
    ssp->reg->IMSC = SSPx_IMSC_TXIM | SSPx_IMSC_RXIM | SSPx_IMSC_RTIM | SSPx_IMSC_RORIM;
  }

  return ARM_DRIVER_OK;
}

/**
  \fn          uint32_t SSPx_GetDataCount (SSP_RESOURCES *ssp)
  \brief       Get transferred data count.
  \param[in]   ssp  Pointer to SSP resources
  \return      number of data items transferred
*/
static uint32_t SSPx_GetDataCount (SSP_RESOURCES *ssp) {
  uint32_t cnt;

  if (!(ssp->info->state & SSP_CONFIGURED)) { return 0U; }

  if (ssp->dma.rx_en) {
    cnt = GPDMA_ChannelGetCount (ssp->dma.rx_ch);
  } else {
    cnt = ssp->xfer->rx_cnt;
  }

  return cnt;
}

/**
  \fn          int32_t SSPx_Control (uint32_t control, uint32_t arg, SSP_RESOURCES *ssp)
  \brief       Control SSP Interface.
  \param[in]   control  Operation
  \param[in]   arg      Argument of operation (optional)
  \param[in]   ssp      Pointer to SSP resources
  \return      common \ref execution_status and driver specific \ref spi_execution_status
*/
static int32_t SSPx_Control (uint32_t control, uint32_t arg, SSP_RESOURCES *ssp) {
  uint32_t cpsr, scr, bps = 0U, clk, data_bits;
  uint32_t best_cpsr = 2U, best_scr = 0U, best_bps = 0U;

  if (!(ssp->info->state & SSP_POWERED)) { return ARM_DRIVER_ERROR; }

  if ((control & ARM_SPI_CONTROL_Msk) == ARM_SPI_ABORT_TRANSFER) {
    ssp->reg->CR1 &= ~SSPx_CR1_SSE;         // Disable SSP
    ssp->reg->IMSC =  0U;                   // Disable interrupts
    if (ssp->info->status.busy) {
      // If DMA mode - disable DMA channel
      if (ssp->dma.tx_en) { GPDMA_ChannelDisable (ssp->dma.tx_ch); }
      // If DMA mode - disable DMA channel
      if (ssp->dma.rx_en) { GPDMA_ChannelDisable (ssp->dma.rx_ch); }
    }
    memset(ssp->xfer, 0, sizeof(SSP_TRANSFER_INFO));
    ssp->info->status.busy = 0U;
    ssp->reg->CR1 |=  SSPx_CR1_SSE;         // Enable  SSP
    return ARM_DRIVER_OK;
  }

  if (ssp->info->status.busy)            { return ARM_DRIVER_ERROR_BUSY; }

  switch (control & ARM_SPI_CONTROL_Msk) {
    default:
      return ARM_DRIVER_ERROR_UNSUPPORTED;

    case ARM_SPI_MODE_INACTIVE:             // SPI Inactive
      ssp->reg->CR1    &= ~SSPx_CR1_SSE;    // Disable SSP
      ssp->reg->IMSC    =  0U;              // Disable interrupts
      ssp->info->mode  &= ~ARM_SPI_CONTROL_Msk;
      ssp->info->mode  |=  ARM_SPI_MODE_INACTIVE;
      ssp->info->state &= ~SSP_CONFIGURED;
      return ARM_DRIVER_OK;

    case ARM_SPI_MODE_MASTER:               // SPI Master (Output on MOSI, Input on MISO); arg = Bus Speed in bps
      ssp->reg->CR1    &= ~SSPx_CR1_SSE;    // Disable SSP
      ssp->reg->IMSC    =  0U;              // Disable interrupts
      ssp->reg->CR1    &= ~SSPx_CR1_MS;     // Set master mode
      ssp->info->mode  &= ~ARM_SPI_CONTROL_Msk;
      ssp->info->mode  |=  ARM_SPI_MODE_MASTER;
      ssp->info->state |=  SSP_CONFIGURED;
      ssp->reg->CR1    |=  SSPx_CR1_SSE;    // Enable  SSP
      goto set_speed;

    case ARM_SPI_MODE_SLAVE:                // SPI Slave  (Output on MISO, Input on MOSI)
      ssp->reg->CR1    &= ~SSPx_CR1_SSE;    // Disable SSP
      ssp->reg->CR1    |=  SSPx_CR1_MS;     // Set slave mode
      ssp->reg->IMSC    =  SSPx_IMSC_RORIM; // Enable receive overrun interrupt
      ssp->info->mode  &= ~ARM_SPI_CONTROL_Msk;
      ssp->info->mode  |=  ARM_SPI_MODE_SLAVE;
      ssp->info->state |=  SSP_CONFIGURED;
      ssp->reg->CR1    |=  SSPx_CR1_SSE;    // Enable  SSP
      break;

    case ARM_SPI_MODE_MASTER_SIMPLEX:       // SPI Master (Output/Input on MOSI); arg = Bus Speed in bps
    case ARM_SPI_MODE_SLAVE_SIMPLEX:        // SPI Slave  (Output/Input on MISO)
      return ARM_SPI_ERROR_MODE;

    case ARM_SPI_SET_BUS_SPEED:             // Set Bus Speed in bps; arg = value
set_speed:
      if (arg == 0U) {
        return ARM_DRIVER_ERROR;
      }

      clk = GetSSPClockFreq(ssp) << 4;
      arg = (arg << 4);
      for (cpsr = 2U; cpsr < 255U; cpsr+= 2U) {// Loop through clock prescaler
        for (scr = 0U; scr < 256U; scr++) {    // Loop through bit prescaler
          bps = clk  / (cpsr * (scr + 1U));
          if (arg == bps) {
            best_bps  = bps;
            best_cpsr = cpsr;
            best_scr  = scr;
            goto found_best;
          } else {
            if (arg > bps) {
              if ((arg - best_bps) > (arg - bps)) {
                best_bps  = bps;
                best_cpsr = cpsr;
                best_scr  = scr;
              }
            }
          }
        }
      }
      if (best_bps == 0U) {
        return ARM_DRIVER_ERROR;
      }
found_best:
      ssp->reg->CPSR =  best_cpsr & SSPx_CPSR_CPSDVSR;
      ssp->reg->CR0 &= ~SSPx_CR0_SCR;
      ssp->reg->CR0 |= ((best_scr << 8) & SSPx_CR0_SCR);
      if ((control & ARM_SPI_CONTROL_Msk) == ARM_SPI_SET_BUS_SPEED) {
        return ARM_DRIVER_OK;
      }
      break;

    case ARM_SPI_GET_BUS_SPEED:             // Get Bus Speed in bps
      return (int32_t)((GetSSPClockFreq(ssp)) / ((ssp->reg->CPSR & SSPx_CPSR_CPSDVSR) * (((ssp->reg->CR0 & SSPx_CR0_SCR) >> 8) + 1U)));

    case ARM_SPI_SET_DEFAULT_TX_VALUE:      // Set default Transmit value; arg = value
      ssp->xfer->def_val = (uint16_t)(arg & 0xFFFFU);
      return ARM_DRIVER_OK;

    case ARM_SPI_CONTROL_SS:                // Control Slave Select; arg = 0:inactive, 1:active
      if (((ssp->info->mode & ARM_SPI_CONTROL_Msk)        != ARM_SPI_MODE_MASTER)  ||
          ((ssp->info->mode & ARM_SPI_SS_MASTER_MODE_Msk) != ARM_SPI_SS_MASTER_SW)) {
        return ARM_DRIVER_ERROR;
      }
      if (ssp->pin.ssel == NULL) {
        return ARM_DRIVER_ERROR;
      }
      if (arg == ARM_SPI_SS_INACTIVE) {
        GPIO_PinWrite  (ssp->pin.ssel->Portnum, ssp->pin.ssel->Pinnum, 1U);
      } else {
        GPIO_PinWrite  (ssp->pin.ssel->Portnum, ssp->pin.ssel->Pinnum, 0U);
      }
      return ARM_DRIVER_OK;
  }

  if ((ssp->info->mode & ARM_SPI_CONTROL_Msk) == ARM_SPI_MODE_MASTER) {
    switch (control & ARM_SPI_SS_MASTER_MODE_Msk) {
      case ARM_SPI_SS_MASTER_UNUSED:        // SPI Slave Select when Master: Not used (default)
#if defined (LPC175x_6x)
        if (ssp->pin.ssel != NULL) { PIN_Configure (ssp->pin.ssel->Portnum, ssp->pin.ssel->Pinnum, 0U, 0U, 0U); }
#elif defined (LPC177x_8x)
        if (ssp->pin.ssel != NULL) { PIN_Configure (ssp->pin.ssel->Portnum, ssp->pin.ssel->Pinnum, IOCON_HYS_ENABLE | IOCON_MODE_PULLUP); }
#endif
        ssp->info->mode  &= ~ARM_SPI_SS_MASTER_MODE_Msk;
        ssp->info->mode  |=  ARM_SPI_SS_MASTER_UNUSED;
        break;

      case ARM_SPI_SS_MASTER_HW_INPUT:      // SPI Slave Select when Master: Hardware monitored Input
        ssp->info->mode  &= ~ARM_SPI_SS_MASTER_MODE_Msk;
        return ARM_SPI_ERROR_SS_MODE;

      case ARM_SPI_SS_MASTER_SW:            // SPI Slave Select when Master: Software controlled
        ssp->info->mode  &= ~ARM_SPI_SS_MASTER_MODE_Msk;
        if (ssp->pin.ssel != NULL) {
#if defined (LPC175x_6x)
          PIN_Configure (ssp->pin.ssel->Portnum, ssp->pin.ssel->Pinnum, 0U, PIN_PINMODE_PULLDOWN, 0U);
#elif defined (LPC177x_8x)
          PIN_Configure (ssp->pin.ssel->Portnum, ssp->pin.ssel->Pinnum, IOCON_HYS_ENABLE | IOCON_MODE_PULLDOWN);
#endif
          GPIO_SetDir      (ssp->pin.ssel->Portnum, ssp->pin.ssel->Pinnum, GPIO_DIR_OUTPUT);
          GPIO_PinWrite    (ssp->pin.ssel->Portnum, ssp->pin.ssel->Pinnum, 1U);
          ssp->info->mode |= ARM_SPI_SS_MASTER_SW;
        } else {
          return ARM_SPI_ERROR_SS_MODE;
        }
        break;

      case ARM_SPI_SS_MASTER_HW_OUTPUT:     // SPI Slave Select when Master: Hardware controlled Output
        ssp->info->mode  &= ~ARM_SPI_SS_MASTER_MODE_Msk;
        if (ssp->pin.ssel != NULL) {
#if defined (LPC175x_6x)
          PIN_Configure (ssp->pin.ssel->Portnum, ssp->pin.ssel->Pinnum, ssp->pin.ssel_func, PIN_PINMODE_PULLDOWN, 0U);
#elif defined (LPC177x_8x)
          PIN_Configure (ssp->pin.ssel->Portnum, ssp->pin.ssel->Pinnum, ssp->pin.ssel_func | IOCON_HYS_ENABLE | IOCON_MODE_PULLDOWN);
#endif
          ssp->info->mode |= ARM_SPI_SS_MASTER_HW_OUTPUT;
        } else {
          return ARM_SPI_ERROR_SS_MODE;
        }
        break;
      default:
        break;
    }
  }

  if ((ssp->info->mode & ARM_SPI_CONTROL_Msk) ==  ARM_SPI_MODE_SLAVE) {
    switch (control & ARM_SPI_SS_SLAVE_MODE_Msk) {
      case ARM_SPI_SS_SLAVE_HW:             // SPI Slave Select when Slave: Hardware monitored (default)
        ssp->info->mode  &= ~ARM_SPI_SS_SLAVE_MODE_Msk;
        if (ssp->pin.ssel != NULL) {
#if defined (LPC175x_6x)
          PIN_Configure (ssp->pin.ssel->Portnum, ssp->pin.ssel->Pinnum, ssp->pin.ssel_func, PIN_PINMODE_PULLDOWN, 0U);
#elif defined (LPC177x_8x)
          PIN_Configure (ssp->pin.ssel->Portnum, ssp->pin.ssel->Pinnum, ssp->pin.ssel_func | IOCON_HYS_ENABLE | IOCON_MODE_PULLDOWN);
#endif
          ssp->info->mode |= ARM_SPI_SS_SLAVE_HW;
        } else {
          return ARM_SPI_ERROR_SS_MODE;
        }
        break;

      case ARM_SPI_SS_SLAVE_SW:             // SPI Slave Select when Slave: Software controlled
        ssp->info->mode  &= ~ARM_SPI_SS_SLAVE_MODE_Msk;
        return ARM_SPI_ERROR_SS_MODE;
      default: return ARM_SPI_ERROR_SS_MODE;
    }
  }

  // Configure Frame Format
  switch (control & ARM_SPI_FRAME_FORMAT_Msk) {
    case ARM_SPI_CPOL0_CPHA0:
      ssp->reg->CR0 &=  ~SSPx_CR0_FRF;
      ssp->reg->CR0 &= ~(SSPx_CR0_CPOL | SSPx_CR0_CPHA);
      break;

    case ARM_SPI_CPOL0_CPHA1:
      ssp->reg->CR0 &=  ~SSPx_CR0_FRF;
      ssp->reg->CR0 &=  ~SSPx_CR0_CPOL;
      ssp->reg->CR0 |=   SSPx_CR0_CPHA;
      break;

    case ARM_SPI_CPOL1_CPHA0:
      ssp->reg->CR0 &=  ~SSPx_CR0_FRF;
      ssp->reg->CR0 |=   SSPx_CR0_CPOL;
      ssp->reg->CR0 &=  ~SSPx_CR0_CPHA;
      break;

    case ARM_SPI_CPOL1_CPHA1:
      ssp->reg->CR0 &=  ~SSPx_CR0_FRF;
      ssp->reg->CR0 |=  (SSPx_CR0_CPOL | SSPx_CR0_CPHA);
      break;

    case ARM_SPI_TI_SSI:
      ssp->reg->CR0  =  (ssp->reg->CR0 & (~SSPx_CR0_FRF)) | (1U << 4);
      break;

    case ARM_SPI_MICROWIRE:
      ssp->reg->CR0  =  (ssp->reg->CR0 & (~SSPx_CR0_FRF)) | (2U << 4);
      break;

    default:
      return ARM_SPI_ERROR_FRAME_FORMAT;
  }

  // Configure Number of Data Bits
  data_bits = ((control & ARM_SPI_DATA_BITS_Msk) >> ARM_SPI_DATA_BITS_Pos);
  if ((data_bits >= 4U) && (data_bits <= 16U)) {
    ssp->reg->CR0 = (ssp->reg->CR0 & (~SSPx_CR0_DSS)) | ((data_bits - 1U) << 0);
  } else {
    return ARM_SPI_ERROR_DATA_BITS;
  }

  // Configure Bit Order
  if ((control & ARM_SPI_BIT_ORDER_Msk) == ARM_SPI_LSB_MSB) {
    return ARM_SPI_ERROR_BIT_ORDER;
  }

  return ARM_DRIVER_OK;
}

/**
  \fn          ARM_SPI_STATUS SSPx_GetStatus (SSP_RESOURCES *ssp)
  \brief       Get SSP status.
  \param[in]   ssp  Pointer to SSP resources
  \return      SPI status \ref ARM_SPI_STATUS
*/
static ARM_SPI_STATUS SSPx_GetStatus (SSP_RESOURCES *ssp) {
  ARM_SPI_STATUS status;

  status.busy       = ssp->info->status.busy;
  status.data_lost  = ssp->info->status.data_lost;
  status.mode_fault = ssp->info->status.mode_fault;

  return (status);
}

/**
  \fn          void SSPx_GPDMA_Tx_SignalEvent (uint32_t event, SSP_RESOURCES *ssp)
  \brief       SSP GPDMA Tx Event handler.
  \param[in]   event GPDMA Tx Event
  \param[in]   ssp   Pointer to SSP resources
*/
void SSPx_GPDMA_Tx_SignalEvent (uint32_t event, SSP_RESOURCES *ssp) {

  switch (event) {
    case GPDMA_EVENT_TERMINAL_COUNT_REQUEST:
      ssp->xfer->tx_cnt = ssp->xfer->num;
      break;
    case GPDMA_EVENT_ERROR:
    default:
      break;
  }
}

/**
  \fn          void SSPx_GPDMA_Rx_SignalEvent (uint32_t event, SSP_RESOURCES *ssp)
  \brief       SSP GPDMA Rx Event handler.
  \param[in]   event GPDMA Rx Event
  \param[in]   ssp   Pointer to SSP resources
*/
void SSPx_GPDMA_Rx_SignalEvent (uint32_t event, SSP_RESOURCES *ssp) {

  switch (event) {
    case GPDMA_EVENT_TERMINAL_COUNT_REQUEST:
      ssp->xfer->rx_cnt = ssp->xfer->num;
      ssp->info->status.busy = 0U;
      if (ssp->info->cb_event) {
        ssp->info->cb_event(ARM_SPI_EVENT_TRANSFER_COMPLETE);
      }
      break;
    case GPDMA_EVENT_ERROR:
    default:
      break;
  }
}

/**
  \fn          void SSPx_IRQHandler (SSP_RESOURCES *ssp)
  \brief       SSP Interrupt handler.
  \param[in]   ssp  Pointer to SSP resources
*/
static void SSPx_IRQHandler (SSP_RESOURCES *ssp) {
  uint16_t data;
  uint32_t mis;

  mis = ssp->reg->MIS;
  ssp->reg->ICR = mis & 3U;

                                                  // Handle transfer
  if ((ssp->reg->SR & SSPx_SR_TNF) && (ssp->xfer->num > ssp->xfer->tx_cnt)) {
    if (ssp->xfer->tx_buf) {                      // If data available
      data = *(ssp->xfer->tx_buf++);
      if ((ssp->reg->CR0 & SSPx_CR0_DSS) > 7) {   // If 9..16-bit data frame format
        data |= *(ssp->xfer->tx_buf++) << 8;
      }
    } else {                                      // If default data send
      data = ssp->xfer->def_val;
    }
    ssp->reg->DR = data;                          // Activate send
    ssp->xfer->tx_cnt++;
  }

  if (ssp->reg->SR & SSPx_SR_RNE) {
    data = ssp->reg->DR & 0xFFFFU;               // Read data
    if (ssp->xfer->num > ssp->xfer->rx_cnt) {
      if (ssp->xfer->rx_buf) {
        *(ssp->xfer->rx_buf++) = (uint8_t)data;   // Put data into buffer
        if ((ssp->reg->CR0 & SSPx_CR0_DSS) > 7U) { // If 9..16-bit data frame format
          *(ssp->xfer->rx_buf++) = (uint8_t)(data >> 8);
        }
      }
      ssp->xfer->rx_cnt++;
      if (ssp->xfer->rx_cnt == ssp->xfer->num) {  // If all data received
        ssp->reg->IMSC   &= ~(SSPx_IMSC_TXIM | SSPx_IMSC_RXIM | SSPx_IMSC_RTIM | SSPx_IMSC_RORIM);
        ssp->info->status.busy = 0U;
        if (ssp->info->cb_event) { ssp->info->cb_event(ARM_SPI_EVENT_TRANSFER_COMPLETE); }
      }
    }
  }

  if (mis & SSPx_MIS_RORMIS) {                    // Handle errors
    // Overrun flag is set
    ssp->info->status.data_lost = 1U;
    if (ssp->info->cb_event) { ssp->info->cb_event(ARM_SPI_EVENT_DATA_LOST); }
  }
}


#if (RTE_SSP0)
static int32_t        SSP0_Initialize          (ARM_SPI_SignalEvent_t pSignalEvent)                { return SSPx_Initialize   (pSignalEvent, &SSP0_Resources); }
static int32_t        SSP0_Uninitialize        (void)                                              { return SSPx_Uninitialize (&SSP0_Resources); }
static int32_t        SSP0_PowerControl        (ARM_POWER_STATE state)                             { return SSPx_PowerControl (state, &SSP0_Resources); }
static int32_t        SSP0_Send                (const void *data, uint32_t num)                    { return SSPx_Send         (data, num, &SSP0_Resources); }
static int32_t        SSP0_Receive             (void *data, uint32_t num)                          { return SSPx_Receive      (data, num, &SSP0_Resources); }
static int32_t        SSP0_Transfer            (const void *data_out, void *data_in, uint32_t num) { return SSPx_Transfer     (data_out, data_in, num, &SSP0_Resources); }
static uint32_t       SSP0_GetDataCount        (void)                                              { return SSPx_GetDataCount (&SSP0_Resources); }
static int32_t        SSP0_Control             (uint32_t control, uint32_t arg)                    { return SSPx_Control      (control, arg, &SSP0_Resources); }
static ARM_SPI_STATUS SSP0_GetStatus           (void)                                              { return SSPx_GetStatus    (&SSP0_Resources); }
       void           SSP0_GPDMA_Tx_SignalEvent(uint32_t event)                                    { SSPx_GPDMA_Tx_SignalEvent(event, &SSP0_Resources); }
       void           SSP0_GPDMA_Rx_SignalEvent(uint32_t event)                                    { SSPx_GPDMA_Rx_SignalEvent(event, &SSP0_Resources); }
       void           SSP0_IRQHandler          (void)                                              { SSPx_IRQHandler          (&SSP0_Resources); }

// SPI0 Driver Control Block
ARM_DRIVER_SPI Driver_SPI0 = {
  SSP_GetVersion,
  SSP_GetCapabilities,
  SSP0_Initialize,
  SSP0_Uninitialize,
  SSP0_PowerControl,
  SSP0_Send,
  SSP0_Receive,
  SSP0_Transfer,
  SSP0_GetDataCount,
  SSP0_Control,
  SSP0_GetStatus
};
#endif


#if (RTE_SSP1)
static int32_t        SSP1_Initialize          (ARM_SPI_SignalEvent_t pSignalEvent)                { return SSPx_Initialize   (pSignalEvent, &SSP1_Resources); }
static int32_t        SSP1_Uninitialize        (void)                                              { return SSPx_Uninitialize (&SSP1_Resources); }
static int32_t        SSP1_PowerControl        (ARM_POWER_STATE state)                             { return SSPx_PowerControl (state, &SSP1_Resources); }
static int32_t        SSP1_Send                (const void *data, uint32_t num)                    { return SSPx_Send         (data, num, &SSP1_Resources); }
static int32_t        SSP1_Receive             (void *data, uint32_t num)                          { return SSPx_Receive      (data, num, &SSP1_Resources); }
static int32_t        SSP1_Transfer            (const void *data_out, void *data_in, uint32_t num) { return SSPx_Transfer     (data_out, data_in, num, &SSP1_Resources); }
static uint32_t       SSP1_GetDataCount        (void)                                              { return SSPx_GetDataCount (&SSP1_Resources); }
static int32_t        SSP1_Control             (uint32_t control, uint32_t arg)                    { return SSPx_Control      (control, arg, &SSP1_Resources); }
static ARM_SPI_STATUS SSP1_GetStatus           (void)                                              { return SSPx_GetStatus    (&SSP1_Resources); }
       void           SSP1_GPDMA_Tx_SignalEvent(uint32_t event)                                    { SSPx_GPDMA_Tx_SignalEvent(event, &SSP1_Resources); }
       void           SSP1_GPDMA_Rx_SignalEvent(uint32_t event)                                    { SSPx_GPDMA_Rx_SignalEvent(event, &SSP1_Resources); }
       void           SSP1_IRQHandler          (void)                                              { SSPx_IRQHandler          (&SSP1_Resources); }

// SPI1 Driver Control Block
ARM_DRIVER_SPI Driver_SPI1 = {
  SSP_GetVersion,
  SSP_GetCapabilities,
  SSP1_Initialize,
  SSP1_Uninitialize,
  SSP1_PowerControl,
  SSP1_Send,
  SSP1_Receive,
  SSP1_Transfer,
  SSP1_GetDataCount,
  SSP1_Control,
  SSP1_GetStatus
};
#endif

#if defined (LPC177x_8x)
#if (RTE_SSP2)
static int32_t        SSP2_Initialize          (ARM_SPI_SignalEvent_t pSignalEvent)                { return SSPx_Initialize   (pSignalEvent, &SSP2_Resources); }
static int32_t        SSP2_Uninitialize        (void)                                              { return SSPx_Uninitialize (&SSP2_Resources); }
static int32_t        SSP2_PowerControl        (ARM_POWER_STATE state)                             { return SSPx_PowerControl (state, &SSP2_Resources); }
static int32_t        SSP2_Send                (const void *data, uint32_t num)                    { return SSPx_Send         (data, num, &SSP2_Resources); }
static int32_t        SSP2_Receive             (void *data, uint32_t num)                          { return SSPx_Receive      (data, num, &SSP2_Resources); }
static int32_t        SSP2_Transfer            (const void *data_out, void *data_in, uint32_t num) { return SSPx_Transfer     (data_out, data_in, num, &SSP2_Resources); }
static uint32_t       SSP2_GetDataCount        (void)                                              { return SSPx_GetDataCount (&SSP2_Resources); }
static int32_t        SSP2_Control             (uint32_t control, uint32_t arg)                    { return SSPx_Control      (control, arg, &SSP2_Resources); }
static ARM_SPI_STATUS SSP2_GetStatus           (void)                                              { return SSPx_GetStatus    (&SSP2_Resources); }
       void           SSP2_GPDMA_Tx_SignalEvent(uint32_t event)                                    { SSPx_GPDMA_Tx_SignalEvent(event, &SSP2_Resources); }
       void           SSP2_GPDMA_Rx_SignalEvent(uint32_t event)                                    { SSPx_GPDMA_Rx_SignalEvent(event, &SSP2_Resources); }
       void           SSP2_IRQHandler          (void)                                              { SSPx_IRQHandler          (&SSP2_Resources); }

// SPI2 Driver Control Block
ARM_DRIVER_SPI Driver_SPI2 = {
  SSP_GetVersion,
  SSP_GetCapabilities,
  SSP2_Initialize,
  SSP2_Uninitialize,
  SSP2_PowerControl,
  SSP2_Send,
  SSP2_Receive,
  SSP2_Transfer,
  SSP2_GetDataCount,
  SSP2_Control,
  SSP2_GetStatus
};
#endif
#endif
