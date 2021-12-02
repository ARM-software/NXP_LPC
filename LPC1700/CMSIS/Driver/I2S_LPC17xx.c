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
 * $Revision:    V1.4
 *
 * Driver:       Driver_SAI0
 * Configured:   via RTE_Device.h configuration file
 * Project:      SAI (I2S used for SAI) Driver for NXP LPC17xx
 * --------------------------------------------------------------------------
 * Use the following configuration settings in the middleware component
 * to connect to this driver.
 *
 *   Configuration Setting                 Value   SAI Interface
 *   ---------------------                 -----   -------------
 *   Connect to hardware via Driver_SAI# = 0       use SAI0 (I2S0)
 * -------------------------------------------------------------------------- */

/* History:
 *  Version 1.4
 *    - Removed minor compiler warnings
 *  Version 1.3
 *    - Added support for ARM Compiler 6
 *    - Made Pin configuration const
 *  Version 1.2
 *    - Updated Pin configuration 
 *    - Driver update to work with GPDMA_LPC17xx ver.: 1.3
 *  Version 1.1
 *    - Corrected PowerControl function for conditional Power full (driver must be initialized)
 *  Version 1.0
 *    - Initial release
 */
 
#include <math.h>
 
#include "I2S_LPC17xx.h"

#if (!defined(RTE_I2S0))
#error "I2S missing in RTE_Device.h. Please update RTE_Device.h!"
#endif

#if (defined(RTE_Drivers_SAI0) && !RTE_I2S0)
#error "I2S0 not configured in RTE_Device.h!"
#endif

// Definitions

// Frequency tolerance in percentage
#ifndef I2S_FREQ_TOLERANCE
#define I2S_FREQ_TOLERANCE   (1.)
#endif

#define D2F_DOMAIN           (255UL)

// FIFO level can have value 1 to 7
#ifndef I2S0_TX_FIFO_LEVEL
#define I2S0_TX_FIFO_LEVEL   ( 4U )
#endif

#ifndef I2S0_RX_FIFO_LEVEL
#define I2S0_RX_FIFO_LEVEL   ( 4U )
#endif

#if ((I2S0_TX_FIFO_LEVEL < 1U) || (I2S0_TX_FIFO_LEVEL > 7U))
#error "Invalid FIFO Level value. FIFO Level can be 1 to 7"
#endif
#if ((I2S0_RX_FIFO_LEVEL < 1U) || (I2S0_RX_FIFO_LEVEL > 7U))
#error "Invalid FIFO Level value. FIFO Level can be 1 to 7"
#endif

#define ARM_SAI_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1,4)   // driver version

/* Interrupt Handler Prototypes */
void I2S_IRQHandler (void);

// Driver Version
static const ARM_DRIVER_VERSION DriverVersion = {
  ARM_SAI_API_VERSION,
  ARM_SAI_DRV_VERSION
};


// I2S0
static I2S_INFO I2S0_Info = {0};

#if (RTE_I2S0_RX_SCK_PIN_EN == 1U)
static const PIN I2S0_pin_rx_sck  = { RTE_I2S0_RX_SCK_PORT,  RTE_I2S0_RX_SCK_BIT  };
#endif
#if (RTE_I2S0_RX_WS_PIN_EN  == 1U)
static const PIN I2S0_pin_rx_ws   = { RTE_I2S0_RX_WS_PORT,   RTE_I2S0_RX_WS_BIT   };
#endif
#if (RTE_I2S0_RX_SDA_PIN_EN == 1U)
static const PIN I2S0_pin_rx_sda  = { RTE_I2S0_RX_SDA_PORT,  RTE_I2S0_RX_SDA_BIT  };
#endif
#if (RTE_I2S0_RX_MCLK_PIN_EN == 1U)
static const PIN I2S0_pin_rx_mclk = { RTE_I2S0_RX_MCLK_PORT, RTE_I2S0_RX_MCLK_BIT };
#endif
#if (RTE_I2S0_TX_SCK_PIN_EN  == 1U)
static const PIN I2S0_pin_tx_sck  = { RTE_I2S0_TX_SCK_PORT,  RTE_I2S0_TX_SCK_BIT  };
#endif
#if (RTE_I2S0_TX_WS_PIN_EN   == 1U)
static const PIN I2S0_pin_tx_ws   = { RTE_I2S0_TX_WS_PORT,   RTE_I2S0_TX_WS_BIT   };
#endif
#if (RTE_I2S0_TX_SDA_PIN_EN  == 1U)
static const PIN I2S0_pin_tx_sda  = { RTE_I2S0_TX_SDA_PORT,  RTE_I2S0_TX_SDA_BIT  };
#endif
#if (RTE_I2S0_TX_MCLK_PIN_EN == 1U)
static const PIN I2S0_pin_tx_mclk = { RTE_I2S0_TX_MCLK_PORT, RTE_I2S0_TX_MCLK_BIT };
#endif

#if (RTE_I2S0_DMA_TX_EN == 1U)
void I2S_GPDMA_Tx_Event (uint32_t event);
static I2S_DMA I2S0_DMA_Tx = {I2S_GPDMA_Tx_Event,
                              RTE_I2S0_DMA_TX_CH,
                              GPDMA_CONN_I2S_Channel_0,
                              {0U, 0U}
                             };
#endif
#if (RTE_I2S0_DMA_RX_EN == 1U)
void I2S_GPDMA_Rx_Event (uint32_t event);
static I2S_DMA I2S0_DMA_Rx = {I2S_GPDMA_Rx_Event,
                              RTE_I2S0_DMA_RX_CH,
                              GPDMA_CONN_I2S_Channel_1,
                              {0U, 0U}
                             };
#endif

static I2S_RESOURCES I2S0_Resources = {
  {  // Capabilities
    1U,  ///< supports asynchronous Transmit/Receive
    1U,  ///< supports synchronous Transmit/Receive
    0U,  ///< supports user defined Protocol
    1U,  ///< supports I2S Protocol
    0U,  ///< supports MSB/LSB justified Protocol
    0U,  ///< supports PCM short/long frame Protocol
    0U,  ///< supports AC'97 Protocol
    1U,  ///< supports Mono mode
    0U,  ///< supports Companding
#if ((RTE_I2S0_TX_MCLK_PIN_EN == 1U) && (RTE_I2S0_RX_MCLK_PIN_EN == 1U))
    1U,  ///< supports MCLK (Master Clock) pin
#else
    0U,  ///< supports MCLK (Master Clock) pin
#endif
    0U   ///< supports Frame error event: \ref ARM_SAI_EVENT_FRAME_ERROR
#if (defined(ARM_SAI_API_VERSION) && (ARM_SAI_API_VERSION >= 0x101U))
  , 0U   ///< reserved bits
#endif
  },
  LPC_I2S,
  {  // I2S0 RX Pin configuration
#if (RTE_I2S0_RX_SCK_PIN_EN  == 1U)
    &I2S0_pin_rx_sck,
    RTE_I2S0_RX_SCK_FUNC,
    NULL,
    {0U, 0U},
#else
    NULL,
    0,
    0,
    {0U, 0U},
#endif
#if (RTE_I2S0_RX_WS_PIN_EN   == 1U)
    &I2S0_pin_rx_ws,
    RTE_I2S0_RX_WS_FUNC,
    NULL,
    {0U, 0U},
#else
    NULL,
    0,
    0,
    {0U, 0U},
#endif
#if (RTE_I2S0_RX_SDA_PIN_EN  == 1U)
    &I2S0_pin_rx_sda,
    RTE_I2S0_RX_SDA_FUNC,
    NULL,
    {0U, 0U},
#else
    NULL,
    0,
    0,
    {0U, 0U},
#endif
#if (RTE_I2S0_RX_MCLK_PIN_EN == 1U)
    &I2S0_pin_rx_mclk,
    RTE_I2S0_RX_SCK_FUNC,
    NULL,
    {0U, 0U},
#else
    NULL,
    0,
    0,
    {0U, 0U},
#endif
  },
  {  // I2S0 RX Pin configuration
#if (RTE_I2S0_TX_SCK_PIN_EN  == 1U)
    &I2S0_pin_tx_sck,
    RTE_I2S0_TX_SCK_FUNC,
    NULL,
    {0U, 0U},
#else
    NULL,
    0,
    0,
    {0U, 0U},
#endif
#if (RTE_I2S0_TX_WS_PIN_EN   == 1U)
    &I2S0_pin_tx_ws,
    RTE_I2S0_TX_WS_FUNC,
    NULL,
    {0U, 0U},
#else
    NULL,
    0,
    0,
    {0U, 0U},
#endif
#if (RTE_I2S0_TX_SDA_PIN_EN  == 1U)
    &I2S0_pin_tx_sda,
    RTE_I2S0_TX_SDA_FUNC,
    NULL,
    {0U, 0U},
#else
    NULL,
    0,
    0,
    {0U, 0U},
#endif
#if (RTE_I2S0_TX_MCLK_PIN_EN == 1U)
    &I2S0_pin_tx_mclk,
    RTE_I2S0_TX_MCLK_FUNC,
    NULL,
    {0U, 0U},
#else
    NULL,
    0,
    0,
    {0U, 0U},
#endif
  },
  I2S_IRQn,
#if (RTE_I2S0_DMA_TX_EN == 1U)
  &I2S0_DMA_Tx,
#else
  NULL,
#endif
#if (RTE_I2S0_DMA_RX_EN == 1U)
  &I2S0_DMA_Rx,
#else
  NULL,
#endif
  (uint8_t) I2S0_TX_FIFO_LEVEL,
  (uint8_t) I2S0_RX_FIFO_LEVEL,
  {0U, 0U,},
  &I2S0_Info
};

static const I2S_RESOURCES * const i2s = &I2S0_Resources;

// Extern Function
extern uint32_t GetClockFreq (uint32_t clk_src);

/*
  \fn          static void i2s_dec2fract (double dec, uint8_t* xret, uint8_t* yret)
  \brief       convert a decimal to a fraction for x/y baudrate factor
  \details     Use continued fractions to find matching fraction
               http://en.wikipedia.org/wiki/Generalized_continued_fraction
  \param[in]   dec      Decimal fraction as floating point
  \param[in]   xret     pointer to numerator result
  \param[in]   yret     pointer to denominator result
*/

static void i2s_dec2fract (double dec, uint8_t* xret, uint8_t* yret) {
  int_fast64_t a, tmp, idec;
  int_fast64_t n = 1;

  int_fast64_t f[3] = { 0, 1, 0 };
  int_fast64_t g[3] = { 1, 0, 0 };
  int i;

  //Expand float input
  while (dec != floor(dec)) { n <<= 1; dec *= 2; }
  idec = (int_fast64_t)dec;

  //continue fraction
  for (i = 0; i < 64; i++) {
    a = n ? idec / n : 0;
    if (i && !a) break;
    tmp = idec;
    idec = n; 
    n = tmp % n;
    tmp = a;
    //check denominator
    if (g[1] * a + g[0] >= D2F_DOMAIN) {
      tmp = (D2F_DOMAIN - g[0]) / g[1];
      if (tmp * 2 >= a || g[1] >= D2F_DOMAIN) { i = 65; }
      else                                    { break;  }
    }
    f[2] = tmp * f[1] + f[0]; 
    f[0] = f[1]; 
    f[1] = f[2];
    g[2] = tmp * g[1] + g[0]; 
    g[0] = g[1]; 
    g[1] = g[2];
  }
  *yret = g[1] & 0xFFU;
  *xret = f[1] & 0xFFU;
}

/**
  \fn          ARM_DRIVER_VERSION I2S_GetVersion (void)
  \brief       Get driver version.
  \return      \ref ARM_DRIVER_VERSION
*/
static ARM_DRIVER_VERSION I2S_GetVersion (void) {
  return (DriverVersion);
}

/**
  \fn          ARM_SAI_CAPABILITIES I2Sx_GetCapabilities (void)
  \brief       Get driver capabilities.
  \return      \ref ARM_SAI_CAPABILITIES
*/
static ARM_SAI_CAPABILITIES I2S_GetCapabilities (void) {
  return (i2s->capabilities);
}

/**
  \fn          int32_t I2S0_Initialize (ARM_SAI_SignalEvent_t cb_event)
  \brief       Initialize I2S Interface.
  \param[in]   cb_event  Pointer to \ref ARM_SAI_SignalEvent
  \return      \ref execution_status
*/
static int32_t I2S0_Initialize (ARM_SAI_SignalEvent_t cb_event) {

  if (i2s->info->flags & I2S_FLAG_INITIALIZED) {
    // Driver is already initialized
    return ARM_DRIVER_OK;
  }

  // Initialize I2S Run-Time resources
  i2s->info->cb_event             = cb_event;
  i2s->info->status.frame_error   = 0U;
  i2s->info->status.rx_busy       = 0U;
  i2s->info->status.rx_overflow   = 0U;
  i2s->info->status.tx_busy       = 0U;
  i2s->info->status.tx_underflow  = 0U;

  // Configure RX SCK pin
  if (i2s->rx_pins.sck != NULL) {
#if defined (LPC175x_6x)
    PIN_Configure (i2s->rx_pins.sck->Portnum, i2s->rx_pins.sck->Pinnum, i2s->rx_pins.func_sck, 0U, 0U);
#elif defined (LPC177x_8x)
    PIN_Configure (i2s->rx_pins.sck->Portnum, i2s->rx_pins.sck->Pinnum, (uint32_t)(i2s->rx_pins.func_sck | (i2s->rx_pins.sck_io_wa << 7)));
#endif
  }

  // Configure RX WS pin
  if (i2s->rx_pins.ws != NULL) {
#if defined (LPC175x_6x)
    PIN_Configure (i2s->rx_pins.ws->Portnum, i2s->rx_pins.ws->Pinnum, i2s->rx_pins.func_ws, 0U, 0U);
#elif defined (LPC177x_8x)
    PIN_Configure (i2s->rx_pins.ws->Portnum, i2s->rx_pins.ws->Pinnum, (uint32_t)(i2s->rx_pins.func_ws | (i2s->rx_pins.ws_io_wa << 7)));
#endif
  }

  // Configure RX SDA pin
  if (i2s->rx_pins.sda != NULL) {
#if defined (LPC175x_6x)
    PIN_Configure (i2s->rx_pins.sda->Portnum, i2s->rx_pins.sda->Pinnum, i2s->rx_pins.func_sda, 0U, 0U);
#elif defined (LPC177x_8x)
    PIN_Configure (i2s->rx_pins.sda->Portnum, i2s->rx_pins.sda->Pinnum, (uint32_t)(i2s->rx_pins.func_sda | (i2s->rx_pins.sda_io_wa << 7)));
#endif
  }

  // Configure RX MCLK pin
  if (i2s->rx_pins.mclk != NULL) {
#if defined (LPC175x_6x)
    PIN_Configure (i2s->rx_pins.mclk->Portnum, i2s->rx_pins.mclk->Pinnum, i2s->rx_pins.func_mclk, 0U, 0U);
#elif defined (LPC177x_8x)
    PIN_Configure (i2s->rx_pins.mclk->Portnum, i2s->rx_pins.mclk->Pinnum, (uint32_t)(i2s->rx_pins.func_mclk | (i2s->rx_pins.mclk_io_wa << 7)));
#endif
  }

  // Configure TX SCK pin
  if (i2s->tx_pins.sck != NULL) {
#if defined (LPC175x_6x)
    PIN_Configure (i2s->tx_pins.sck->Portnum, i2s->tx_pins.sck->Pinnum, i2s->tx_pins.func_sck, 0U, 0U);
#elif defined (LPC177x_8x)
    PIN_Configure (i2s->tx_pins.sck->Portnum, i2s->tx_pins.sck->Pinnum, (uint32_t)(i2s->tx_pins.func_sck | (i2s->tx_pins.sck_io_wa << 7)));
#endif
  }

  // Configure TX WS pin
  if (i2s->tx_pins.ws != NULL) {
#if defined (LPC175x_6x)
    PIN_Configure (i2s->tx_pins.ws->Portnum, i2s->tx_pins.ws->Pinnum, i2s->tx_pins.func_ws, 0U, 0U);
#elif defined (LPC177x_8x)
    PIN_Configure (i2s->tx_pins.ws->Portnum, i2s->tx_pins.ws->Pinnum, (uint32_t)(i2s->tx_pins.func_ws | (i2s->tx_pins.ws_io_wa << 7)));
#endif
  }

  // Configure TX SDA pin
  if (i2s->tx_pins.sda != NULL) {
#if defined (LPC175x_6x)
    PIN_Configure (i2s->tx_pins.sda->Portnum, i2s->tx_pins.sda->Pinnum, i2s->tx_pins.func_sda, 0U, 0U);
#elif defined (LPC177x_8x)
    PIN_Configure (i2s->tx_pins.sda->Portnum, i2s->tx_pins.sda->Pinnum, (uint32_t)(i2s->tx_pins.func_sda | (i2s->tx_pins.sda_io_wa << 7)));
#endif
  }

  // Configure TX MCLK pin
  if (i2s->tx_pins.mclk != NULL) {
#if defined (LPC175x_6x)
    PIN_Configure (i2s->tx_pins.mclk->Portnum, i2s->tx_pins.mclk->Pinnum, i2s->tx_pins.func_mclk, 0U, 0U);
#elif defined (LPC177x_8x)
    PIN_Configure (i2s->tx_pins.mclk->Portnum, i2s->tx_pins.mclk->Pinnum, (uint32_t)(i2s->tx_pins.func_mclk | (i2s->tx_pins.mclk_io_wa << 7)));
#endif
  }

 // DMA Initialize
  if ((i2s->dma_tx != NULL) || (i2s->dma_rx != NULL)) {
    GPDMA_Initialize ();
  }

  i2s->info->flags = I2S_FLAG_INITIALIZED;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t I2S0_Uninitialize (void)
  \brief       De-initialize I2S Interface.
  \return      \ref execution_status
*/
static int32_t I2S0_Uninitialize (void) {

  // Reset RX SCK pin configuration
  if (i2s->rx_pins.sck != NULL) {
#if defined (LPC175x_6x)
    PIN_Configure (i2s->rx_pins.sck->Portnum, i2s->rx_pins.sck->Pinnum, 0U, 0U, 0U);
#elif defined (LPC177x_8x)
    PIN_Configure (i2s->rx_pins.sck->Portnum, i2s->rx_pins.sck->Pinnum, (uint32_t)(i2s->rx_pins.sck_io_wa << 7));
#endif
  }

  // Reset RX WS pin configuration
  if (i2s->rx_pins.ws != NULL) {
#if defined (LPC175x_6x)
    PIN_Configure (i2s->rx_pins.ws->Portnum, i2s->rx_pins.ws->Pinnum, 0U, 0U, 0U);
#elif defined (LPC177x_8x)
    PIN_Configure (i2s->rx_pins.ws->Portnum, i2s->rx_pins.ws->Pinnum, (uint32_t)(i2s->rx_pins.ws_io_wa << 7));
#endif
  }

  // Reset RX SDA pin configuration
  if (i2s->rx_pins.sda != NULL) {
#if defined (LPC175x_6x)
    PIN_Configure (i2s->rx_pins.sda->Portnum, i2s->rx_pins.sda->Pinnum, 0U, 0U, 0U);
#elif defined (LPC177x_8x)
    PIN_Configure (i2s->rx_pins.sda->Portnum, i2s->rx_pins.sda->Pinnum, (uint32_t)(i2s->rx_pins.sda_io_wa << 7));
#endif
  }

  // Reset RX MCLK pin configuration
  if (i2s->rx_pins.mclk != NULL) {
#if defined (LPC175x_6x)
    PIN_Configure (i2s->rx_pins.mclk->Portnum, i2s->rx_pins.mclk->Pinnum, 0U, 0U, 0U);
#elif defined (LPC177x_8x)
    PIN_Configure (i2s->rx_pins.mclk->Portnum, i2s->rx_pins.mclk->Pinnum, (uint32_t)(i2s->rx_pins.mclk_io_wa << 7));
#endif
  }

  // Reset TX SCK pin configuration
  if (i2s->tx_pins.sck != NULL) {
#if defined (LPC175x_6x)
    PIN_Configure (i2s->tx_pins.sck->Portnum, i2s->tx_pins.sck->Pinnum, 0U, 0U, 0U);
#elif defined (LPC177x_8x)
    PIN_Configure (i2s->tx_pins.sck->Portnum, i2s->tx_pins.sck->Pinnum, (uint32_t)(i2s->tx_pins.sck_io_wa << 7));
#endif
  }

  // Reset TX WS pin configuration
  if (i2s->tx_pins.ws != NULL) {
#if defined (LPC175x_6x)
    PIN_Configure (i2s->tx_pins.ws->Portnum, i2s->tx_pins.ws->Pinnum, 0U, 0U, 0U);
#elif defined (LPC177x_8x)
    PIN_Configure (i2s->tx_pins.ws->Portnum, i2s->tx_pins.ws->Pinnum, (uint32_t)(i2s->tx_pins.ws_io_wa << 7));
#endif
  }

  // Reset TX SDA pin configuration
  if (i2s->tx_pins.sda != NULL) {
#if defined (LPC175x_6x)
    PIN_Configure (i2s->tx_pins.sda->Portnum, i2s->tx_pins.sda->Pinnum, 0U, 0U, 0U);
#elif defined (LPC177x_8x)
    PIN_Configure (i2s->tx_pins.sda->Portnum, i2s->tx_pins.sda->Pinnum, (uint32_t)(i2s->tx_pins.sda_io_wa << 7));
#endif
  }

  // Reset TX MCLK pin configuration
  if (i2s->tx_pins.mclk != NULL) {
#if defined (LPC175x_6x)
    PIN_Configure (i2s->tx_pins.mclk->Portnum, i2s->tx_pins.mclk->Pinnum, 0U, 0U, 0U);
#elif defined (LPC177x_8x)
    PIN_Configure (i2s->tx_pins.mclk->Portnum, i2s->tx_pins.mclk->Pinnum, (uint32_t)(i2s->tx_pins.mclk_io_wa << 7));
#endif
  }

  // DMA Uninitialize
  if ((i2s->dma_tx != NULL) || (i2s->dma_rx != NULL)) { GPDMA_Uninitialize (); }

  // Reset I2S status flags
  i2s->info->flags = 0U;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t I2S0_PowerControl (ARM_POWER_STATE state)
  \brief       Control I2S Interface Power.
  \param[in]   state  Power state
  \return      \ref execution_status
*/
static int32_t I2S0_PowerControl (ARM_POWER_STATE state) {

  if ((state != ARM_POWER_OFF)  &&
      (state != ARM_POWER_FULL) &&
      (state != ARM_POWER_LOW)) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  switch (state) {
    case ARM_POWER_OFF:
      // Disable I2S IRQ
      NVIC_DisableIRQ((IRQn_Type)i2s->irq_num);

      // Enable power to I2S block
      LPC_SC->PCONP |= (1U << 27);

      if ((i2s->dma_tx == NULL) && (i2s->info->status.tx_busy != 0U)) {
        // Disable DMA channel
        GPDMA_ChannelDisable (i2s->dma_tx->channel);
      }
      if ((i2s->dma_rx == NULL) && (i2s->info->status.rx_busy != 0U)) {
        // Disable DMA channel
        GPDMA_ChannelDisable (i2s->dma_rx->channel);
      }

      // Reset I2S registers
      i2s->reg->IRQ       = 0U;
      i2s->reg->DMA1      = 0U;
      i2s->reg->DMA2      = 0U;
      i2s->reg->DAO       = 0x87E1U;
      i2s->reg->DAI       = 0x07E1U;
      i2s->reg->TXRATE    = 0U;
      i2s->reg->RXRATE    = 0U;
      i2s->reg->TXBITRATE = 0U;
      i2s->reg->RXBITRATE = 0U;
      i2s->reg->TXMODE    = 0U;
      i2s->reg->RXMODE    = 0U;

      // Disable power to I2S block
      LPC_SC->PCONP &= ~(1U << 27);

      // Clear pending I2S interrupts in NVIC
      NVIC_ClearPendingIRQ((IRQn_Type)i2s->irq_num);

      // Clear driver variables
      i2s->info->status.frame_error   = 0U;
      i2s->info->status.rx_busy       = 0U;
      i2s->info->status.rx_overflow   = 0U;
      i2s->info->status.tx_busy       = 0U;
      i2s->info->status.tx_underflow  = 0U;

      i2s->info->flags &= ~I2S_FLAG_POWERED;
      break;

    case ARM_POWER_LOW:
      return ARM_DRIVER_ERROR_UNSUPPORTED;

    case ARM_POWER_FULL:
      if ((i2s->info->flags & I2S_FLAG_INITIALIZED) == 0U) { return ARM_DRIVER_ERROR; }
      if ((i2s->info->flags & I2S_FLAG_POWERED)     != 0U) { return ARM_DRIVER_OK; }

      // Enable power to I2S block
      LPC_SC->PCONP |= (1U << 27);

#if defined (LPC175x_6x)
      // Configure I2S Clock
      LPC_SC->PCLKSEL1 &= ~(3U << 22);
      LPC_SC->PCLKSEL1 |=  (1U << 22);
#endif

      // Disable I2S interrupts
      i2s->reg->IRQ &= ~(I2S_IRQ_RX_IRQ_ENABLE | I2S_IRQ_TX_IRQ_ENABLE);

      // Stop transmitter and receiver
      i2s->reg->DAO |= (I2S_DAO_DAI_STOP | I2S_DAO_DAI_WS_SEL);
      i2s->reg->DAI |= (I2S_DAO_DAI_STOP | I2S_DAO_DAI_WS_SEL);

      // Clear driver variables
      i2s->info->status.frame_error   = 0U;
      i2s->info->status.rx_busy       = 0U;
      i2s->info->status.rx_overflow   = 0U;
      i2s->info->status.tx_busy       = 0U;
      i2s->info->status.tx_underflow  = 0U;

      i2s->info->rx.residue_cnt       = 0U;
      i2s->info->rx.residue_num       = 0U;
      i2s->info->tx.residue_cnt       = 0U;
      i2s->info->tx.residue_num       = 0U;

      i2s->info->flags = I2S_FLAG_POWERED | I2S_FLAG_INITIALIZED;

      // Clear and Enable SAI IRQ
      NVIC_ClearPendingIRQ((IRQn_Type)i2s->irq_num);
      NVIC_EnableIRQ((IRQn_Type)i2s->irq_num);
      break;
  }
  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t I2S0_Send (const void *data, uint32_t num)
  \brief       Start sending data to I2S transmitter.
  \param[in]   data  Pointer to buffer with data to send to I2S transmitter
  \param[in]   num   Number of data items to send
  \return      \ref execution_status
*/
static int32_t I2S0_Send (const void *data, uint32_t num) {
  uint32_t  val;
  int32_t   stat;

  if ((data == NULL) || (num == 0U)) {
    // Invalid parameters
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if ((i2s->info->flags & I2S_FLAG_CONFIGURED) == 0U) {
    // I2S is not configured (mode not selected)
    return ARM_DRIVER_ERROR;
  }

  if (i2s->info->status.tx_busy) {
    // Send is not completed yet
    return ARM_DRIVER_ERROR_BUSY;
  }

  // Set Send active flag
  i2s->info->status.tx_busy = 1U;

  // Clear TX underflow flag
  i2s->info->status.tx_underflow = 0U;

  // Save transmit buffer info
  i2s->info->tx.buf = (uint8_t *)(uint32_t)data;
  i2s->info->tx.cnt = 0U;

  // Convert from number of samples to number of bytes
  num = num * (i2s->info->tx.data_bits / 8U);

  // Only 32-bit value can be written to FIFO. If there is data left from last send,
  // fill residue buffer to 32bits and write to TX FIFO
  if (i2s->info->tx.residue_cnt != 0U) {
    while ((i2s->info->tx.residue_cnt < 4U) && (i2s->info->tx.cnt < num)) {
      i2s->info->tx.residue_buf[i2s->info->tx.residue_cnt++] = i2s->info->tx.buf[i2s->info->tx.cnt++];
    }

    if (i2s->info->tx.residue_cnt == 4U) {
      // Write 32bits to TX FIFO
      i2s->reg->TXFIFO = __UNALIGNED_UINT32_READ(i2s->info->tx.residue_buf);

      // There is no valid data in residue buffer
      i2s->info->tx.residue_cnt = 0U;
    }
  }

  // Disable I2S transmitter interrupt
  i2s->reg->IRQ &= ~I2S_IRQ_TX_IRQ_ENABLE;

  i2s->info->tx.num = num;

  // DMA mode
  if (i2s->dma_tx != NULL) {
    num -= i2s->info->tx.cnt;
    if (num < 4U) {
      // Enable I2S transmitter interrupt
      i2s->reg->IRQ |= I2S_IRQ_TX_IRQ_ENABLE;
    } else {

      // Configure DMA mux
      GPDMA_PeripheralSelect (i2s->dma_tx->request, 1U);

      // Configure DMA channel
      stat = GPDMA_ChannelConfigure (i2s->dma_tx->channel,
                                     (uint32_t)i2s->info->tx.buf + i2s->info->tx.cnt,
                                     (uint32_t)(&(i2s->reg->TXFIFO)),
                                     num / 4U,
                                     GPDMA_CH_CONTROL_SBSIZE(GPDMA_BSIZE_1)                   |
                                     GPDMA_CH_CONTROL_DBSIZE(GPDMA_BSIZE_1)                   |
                                     GPDMA_CH_CONTROL_SWIDTH(GPDMA_WIDTH_WORD)                |
                                     GPDMA_CH_CONTROL_DWIDTH(GPDMA_WIDTH_WORD)                |
                                     GPDMA_CH_CONTROL_I                                       |
                                     GPDMA_CH_CONTROL_SI,
                                     GPDMA_CH_CONFIG_DEST_PERI(i2s->dma_tx->request)          |
                                     GPDMA_CH_CONFIG_FLOWCNTRL(GPDMA_TRANSFER_M2P_CTRL_DMA)   |
                                     GPDMA_CH_CONFIG_IE                                       |
                                     GPDMA_CH_CONFIG_ITC                                      |
                                     GPDMA_CH_CONFIG_E,
                                     i2s->dma_tx->cb_event);
      if (stat == -1) { return ARM_DRIVER_ERROR; }

      // Set FIFO level and enable TX DMA
      i2s->reg->DMA1 = ((((uint32_t)i2s->tx_fifo_level << I2S_DMA_TX_DEPTH_DMA_POS) & I2S_DMA_TX_DEPTH_DMA_MSK) |
                         (I2S_DMA_TX_DMA_ENABLE));
    }
  // Interrupt mode
  } else {
    // Set FIFO level, to trigger TX interrupt
    val  = i2s->reg->IRQ & ~I2S_IRQ_TX_DEPTH_IRQ_MSK;
    val |= ((uint32_t)i2s->tx_fifo_level << I2S_IRQ_TX_DEPTH_IRQ_POS) & I2S_IRQ_TX_DEPTH_IRQ_MSK;
    i2s->reg->IRQ = val;
    // Enable I2S transmitter interrupt
    i2s->reg->IRQ |= I2S_IRQ_TX_IRQ_ENABLE;
  }

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t I2S0_Receive (void *data, uint32_t num)
  \brief       Start receiving data from I2S receiver.
  \param[out]  data  Pointer to buffer for data to receive from I2S receiver
  \param[in]   num   Number of data items to receive
  \return      \ref execution_status
*/
static int32_t I2S0_Receive (void *data, uint32_t num) {
  uint32_t val, offset;
  int32_t  stat;

  if ((data == NULL) || (num == 0U)) {
    // Invalid parameters
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if ((i2s->info->flags & I2S_FLAG_CONFIGURED) == 0U) {
    // I2S is not configured (mode not selected)
    return ARM_DRIVER_ERROR;
  }

  if (i2s->info->status.rx_busy) {
    // Receive is not completed yet
    return ARM_DRIVER_ERROR_BUSY;
  }

  // Set Receive active flag
  i2s->info->status.rx_busy = 1U;

  // Clear RX overflow flag
  i2s->info->status.rx_overflow = 0U;

  // Save receive buffer info
  i2s->info->rx.buf = (uint8_t *)data;
  i2s->info->rx.cnt = 0U;

  // Convert from number of samples to number of bytes
  num = num * (i2s->info->rx.data_bits / 8U);

  while ((i2s->info->rx.cnt < num) && (i2s->info->rx.residue_cnt < i2s->info->rx.residue_num)) {
    // RX Data available in residue buffer
    i2s->info->rx.buf[i2s->info->rx.cnt++] = i2s->info->rx.residue_buf[i2s->info->rx.residue_cnt++];

    if (i2s->info->rx.residue_cnt == i2s->info->rx.residue_num) {
      // Residue buffer empty
      i2s->info->rx.residue_cnt    = 0U;
      i2s->info->rx.residue_num    = 0U;
    }
  }

  // Disable I2S receive interrupt
  i2s->reg->IRQ &= ~I2S_IRQ_RX_IRQ_ENABLE;
  i2s->info->rx.num = num;

  num -= i2s->info->rx.cnt;
  // DMA mode
  if ((i2s->dma_rx != NULL) && (num >= 4U)) {
    // Set offset in RX Buffer for DMA transfer
    offset = i2s->info->rx.cnt;

    // Update RX count
    num /= 4;
    i2s->info->rx.cnt += num * 4;

    // Configure DMA mux
    GPDMA_PeripheralSelect (i2s->dma_rx->request, 1U);

    // Configure DMA channel
    stat = GPDMA_ChannelConfigure (i2s->dma_rx->channel,
                                   (uint32_t)(&(i2s->reg->RXFIFO)),
                                   (uint32_t)i2s->info->rx.buf + offset,
                                   num,
                                   GPDMA_CH_CONTROL_SBSIZE(GPDMA_BSIZE_1)                   |
                                   GPDMA_CH_CONTROL_DBSIZE(GPDMA_BSIZE_1)                   |
                                   GPDMA_CH_CONTROL_SWIDTH(GPDMA_WIDTH_WORD)                |
                                   GPDMA_CH_CONTROL_DWIDTH(GPDMA_WIDTH_WORD)                |
                                   GPDMA_CH_CONTROL_I                                       |
                                   GPDMA_CH_CONTROL_DI,
                                   GPDMA_CH_CONFIG_SRC_PERI(i2s->dma_rx->request)           |
                                   GPDMA_CH_CONFIG_FLOWCNTRL(GPDMA_TRANSFER_P2M_CTRL_DMA)   |
                                   GPDMA_CH_CONFIG_IE                                       |
                                   GPDMA_CH_CONFIG_ITC                                      |
                                   GPDMA_CH_CONFIG_E,
                                   i2s->dma_rx->cb_event);
    if (stat == -1) { return ARM_DRIVER_ERROR; }

    // Set FIFO level and enable RX DMA
    i2s->reg->DMA2 = (((uint32_t)(i2s->rx_fifo_level << I2S_DMA_RX_DEPTH_DMA_POS) & I2S_DMA_RX_DEPTH_DMA_MSK) |
                       (I2S_DMA_RX_DMA_ENABLE));

  // Interrupt mode
  } else {
    // Set FIFO level, to trigger RX interrupt
    val  = i2s->reg->IRQ & ~I2S_IRQ_RX_DEPTH_IRQ_MSK;
    val |= (uint32_t)(i2s->rx_fifo_level << I2S_IRQ_RX_DEPTH_IRQ_POS) & I2S_IRQ_RX_DEPTH_IRQ_MSK;
    i2s->reg->IRQ = val;
    // Enable I2S receive interrupt
    i2s->reg->IRQ |= I2S_IRQ_RX_IRQ_ENABLE;
  }

  return ARM_DRIVER_OK;
}

/**
  \fn          uint32_t I2S0_GetTxCount (void)
  \brief       Get transmitted data count.
  \return      number of data items transmitted
*/
static uint32_t I2S0_GetTxCount (void) {
  uint32_t cnt;

  // Convert count in bytes to count of samples
  cnt = i2s->info->tx.cnt / (i2s->info->tx.data_bits / 8U);

  return (cnt);
}

/**
  \fn          uint32_t I2S0_GetRxCount (void)
  \brief       Get received data count.
  \return      number of data items received
*/
static uint32_t I2S0_GetRxCount (void) {
  uint32_t cnt;

  // Convert count in bytes to count of samples
  cnt = i2s->info->rx.cnt / (i2s->info->rx.data_bits / 8U);

  return (cnt);
}

/**
  \fn          int32_t I2S0_Control (uint32_t control, uint32_t arg1, uint32_t arg2)
  \brief       Control I2S Interface.
  \param[in]   control  Operation
  \param[in]   arg1     Argument 1 of operation (optional)
  \param[in]   arg2     Argument 2 of operation (optional)
  \return      common \ref execution_status and driver specific \ref sai_execution_status
*/
static int32_t I2S0_Control (uint32_t control, uint32_t arg1, uint32_t arg2) {
  uint32_t  val, pclk, mclk, master, data_bits;
  uint32_t  reg_daoi, reg_rate, reg_bitrate, reg_mode;
  uint8_t   x_best, y_best;
  double    div_exact, div, delta;
  I2S_PINS *pins;

  if ((i2s->info->flags & I2S_FLAG_POWERED) == 0U) {
    // I2S not powered
    return ARM_DRIVER_ERROR;
  }

  master      = 0U;
  data_bits   = 0U;

  reg_daoi    = 0U;
  reg_rate    = 0U;
  reg_bitrate = 0U;
  reg_mode    = 0U;

  switch (control & ARM_SAI_CONTROL_Msk) {
    case ARM_SAI_CONFIGURE_TX:
      pins = &(i2s->tx_pins);
      if (pins->sda == NULL) {
        // No Tx_SDA pin
        return ARM_DRIVER_ERROR;
      }
      break;
      
    case ARM_SAI_CONFIGURE_RX:
      pins = &(i2s->rx_pins);
      if (pins->sda == NULL) {
        // No Rx_SDA pin
        return ARM_DRIVER_ERROR;
      }
      break;
      
    case ARM_SAI_CONTROL_TX:
      if ((arg1 & 1U) == 0U) {
        i2s->reg->DAO |= (I2S_DAO_DAI_STOP | I2S_DAO_DAI_WS_SEL );
      } else {
        if (i2s->info->tx.master) { i2s->reg->DAO &= ~I2S_DAO_DAI_WS_SEL; }
        else                      { i2s->reg->DAO |=  I2S_DAO_DAI_WS_SEL; }

        // Set Stop
        i2s->reg->DAO |= I2S_DAO_DAI_STOP;

        if ((i2s->info->status.tx_busy == 0U) || (i2s->dma_tx)) {
          // Set TX level to 0
          val  = i2s->reg->IRQ & ~I2S_IRQ_TX_DEPTH_IRQ_MSK;
          val |= (0U << I2S_IRQ_TX_DEPTH_IRQ_POS) & I2S_IRQ_TX_DEPTH_IRQ_MSK;
          i2s->reg->IRQ = val;

          if (i2s->info->status.tx_busy == 0U) {
            // Ready to detect TX underflow
            i2s->info->tx.num = 0U;
          }
        }

        // Clear stop
        i2s->reg->DAO &= ~I2S_DAO_DAI_STOP;

        // Enable I2S transmit interrupt
        i2s->reg->IRQ |= I2S_IRQ_TX_IRQ_ENABLE;
      }
      // Mute
      if ((arg1 & 2U) != 0U) { i2s->reg->DAO |=  I2S_DAO_MUTE; }
      else                   { i2s->reg->DAO &= ~I2S_DAO_MUTE; }
      return ARM_DRIVER_OK;

    case ARM_SAI_CONTROL_RX:
      if ((arg1 & 1U) == 0U) {
        i2s->reg->DAI |= (I2S_DAO_DAI_STOP | I2S_DAO_DAI_WS_SEL);
      } else {
        if (i2s->info->rx.master) { i2s->reg->DAI &= ~I2S_DAO_DAI_WS_SEL; }
        else                      { i2s->reg->DAI |=  I2S_DAO_DAI_WS_SEL; }

        // Set Stop
        i2s->reg->DAI |= I2S_DAO_DAI_STOP;

        val  = i2s->reg->IRQ & ~I2S_IRQ_RX_DEPTH_IRQ_MSK;
        if (i2s->info->status.rx_busy == 0U) {
          // Set FIFO level to full, to detect RX overflow
          val |= (8U << I2S_IRQ_RX_DEPTH_IRQ_POS) & I2S_IRQ_RX_DEPTH_IRQ_MSK;
          i2s->reg->IRQ = val;

          // Ready to detect RX overflow
          i2s->info->rx.num = 0U;

          // Enable I2S receive interrupt
          i2s->reg->IRQ |= I2S_IRQ_RX_IRQ_ENABLE;
        } else {
          if ((i2s->dma_rx != NULL) && ((i2s->reg->DMA2 & I2S_DMA_RX_DMA_ENABLE) == 0U)) {
            // Set user RX FIFO level
            val |= ((uint32_t)i2s->rx_fifo_level  << I2S_IRQ_RX_DEPTH_IRQ_POS) & I2S_IRQ_RX_DEPTH_IRQ_MSK;

            // Enable I2S receive interrupt
            i2s->reg->IRQ |= I2S_IRQ_RX_IRQ_ENABLE;
          }
        }
        // Clear stop
        i2s->reg->DAI &= ~I2S_DAO_DAI_STOP;
      }
      return ARM_DRIVER_OK;

    case ARM_SAI_MASK_SLOTS_TX:
      return ARM_DRIVER_ERROR;

    case ARM_SAI_MASK_SLOTS_RX:
      return ARM_DRIVER_ERROR;

    case ARM_SAI_ABORT_SEND:
      // Disable TX interrupt
      i2s->reg->IRQ &= ~I2S_IRQ_TX_IRQ_ENABLE;

      if (i2s->dma_tx) {
        if (i2s->info->status.tx_busy != 0U) {
          // Disable DMA channel
          GPDMA_ChannelDisable (i2s->dma_tx->channel);
        }
      }

      // Reset TX FIFO
      i2s->reg->DAO |= I2S_DAO_DAI_RESET;

      // Reset counters
      i2s->info->tx.cnt = 0U;
      i2s->info->tx.num = 0U;
      i2s->info->tx.residue_cnt = 0U;
      i2s->info->tx.residue_num = 0U;

      // Clear reset FIFO bit
      i2s->reg->DAO &= ~I2S_DAO_DAI_RESET;

      // Clear busy flag
      i2s->info->status.tx_busy = 0U;

      if ((i2s->reg->DAO & I2S_DAO_DAI_STOP) == 0U) {
        // Transmitter is enabled
      
        // Set FIFO level to full and enable TX interrupt, to detect RX overflow
        val  = i2s->reg->IRQ & ~I2S_IRQ_RX_DEPTH_IRQ_MSK;
        val |= (8U << I2S_IRQ_RX_DEPTH_IRQ_POS) & I2S_IRQ_RX_DEPTH_IRQ_MSK;
        i2s->reg->IRQ = val | I2S_IRQ_TX_IRQ_ENABLE;
      }
      return ARM_DRIVER_OK;

    case ARM_SAI_ABORT_RECEIVE:
      // Disable RX interrupt
      i2s->reg->IRQ &= ~I2S_IRQ_RX_IRQ_ENABLE;

      if (i2s->dma_rx) {
        if (i2s->info->status.rx_busy != 0U) {
          // Disable DMA channel
          GPDMA_ChannelDisable (i2s->dma_rx->channel);
        }
      }

      // Reset RX FIFO
      i2s->reg->DAI |= I2S_DAO_DAI_RESET;

      // Reset counters
      i2s->info->rx.cnt = 0U;
      i2s->info->rx.num = 0U;
      i2s->info->rx.residue_cnt = 0U;
      i2s->info->rx.residue_num = 0U;

      // Clear reset FIFO bit
      i2s->reg->DAI &= ~I2S_DAO_DAI_RESET;

      // Clear busy flag
      i2s->info->status.rx_busy = 0U;

      if ((i2s->reg->DAI & I2S_DAO_DAI_STOP) == 0U) {
        // Receiver is enabled
      
        // Set FIFO level to full and enable RX interrupt, to detect RX overflow
        val  = i2s->reg->IRQ & ~I2S_IRQ_RX_DEPTH_IRQ_MSK;
        val |= (8U << I2S_IRQ_RX_DEPTH_IRQ_POS) & I2S_IRQ_RX_DEPTH_IRQ_MSK;
        i2s->reg->IRQ = val | I2S_IRQ_RX_IRQ_ENABLE;
      }
      return ARM_DRIVER_OK;

    default: return ARM_DRIVER_ERROR;
  }

  // Mode
  switch (control & ARM_SAI_MODE_Msk) {
    case ARM_SAI_MODE_MASTER:
      master = 1U;
      break;
    case ARM_SAI_MODE_SLAVE:
      break;
    default: return ARM_DRIVER_ERROR;
  }

  // Synchronization
  switch (control & ARM_SAI_SYNCHRONIZATION_Msk) {
    case ARM_SAI_ASYNCHRONOUS:
      if ((pins->sck == NULL) || (pins->ws == NULL)) {
        // Asynchronous mode requires SCK and WS pins
        return ARM_SAI_ERROR_SYNCHRONIZATION;
      }
      break;
    case ARM_SAI_SYNCHRONOUS:
      if (master == 1U) {
        // Only Slave can be synchronous
        return ARM_SAI_ERROR_SYNCHRONIZATION;
      }
      // 4-pin mode: SCK and WS signals are shared between
      // I2S transmit and receive blocks
      reg_daoi |= I2S_TX_RX_MODE_4PIN;
      break;
    default: return ARM_SAI_ERROR_SYNCHRONIZATION;
  }

  // Protocol
  val = (control & ARM_SAI_PROTOCOL_Msk);
  if (val != ARM_SAI_PROTOCOL_I2S) {
    // Only I2S protocol is supported
    return ARM_SAI_ERROR_PROTOCOL;
  }

  // Data size
  switch ((control & ARM_SAI_DATA_SIZE_Msk) >> ARM_SAI_DATA_SIZE_Pos) {
    case 8U-1U:
      data_bits = 8U;
      // 8-Data bit, WS_HALFPERIOD = DataBit-1 = 7
      reg_daoi |= (7U  << I2S_DAO_DAI_WS_HALFPERIOD_POS);
      break;
    case 16U-1U:
      data_bits = 16U;
      // 16-Data bit, WS_HALFPERIOD = DataBit-1 = 15
      reg_daoi |= (1U  << I2S_DAO_DAI_WORDWIDTH_POS);
      reg_daoi |= (15U << I2S_DAO_DAI_WS_HALFPERIOD_POS);
      break;
    case 32U-1U:
      data_bits = 32U;
      // 32-Data bit, WS_HALFPERIOD = DataBit-1 = 31
      reg_daoi |= (3U  << I2S_DAO_DAI_WORDWIDTH_POS);
      reg_daoi |= (31U << I2S_DAO_DAI_WS_HALFPERIOD_POS);
      break;
    default: return ARM_SAI_ERROR_DATA_SIZE;
  }

  // Mono mode
  if (control & ARM_SAI_MONO_MODE) {
    reg_daoi |= I2S_DAO_DAI_MONO;
  }

  // Companding
  val = control & ARM_SAI_COMPANDING_Msk;
  if (val != ARM_SAI_COMPANDING_NONE)  { return ARM_SAI_ERROR_COMPANDING; }

  // Clock polarity
  val = control & ARM_SAI_CLOCK_POLARITY_Msk;
  if (val != ARM_SAI_CLOCK_POLARITY_0) { return ARM_SAI_ERROR_CLOCK_POLARITY; }

  // Master clock pin
  switch (control & ARM_SAI_MCLK_PIN_Msk) {
    case ARM_SAI_MCLK_PIN_INACTIVE:
      break;
    case ARM_SAI_MCLK_PIN_OUTPUT:
      if (pins->mclk == NULL) {
        // MCLK pin is not available
        return ARM_SAI_ERROR_MCLK_PIN;
      }
      // Generate MCLK on MCLK pin
      reg_mode |= I2S_TX_RX_MODE_MCENA;
      break;
    case ARM_SAI_MCLK_PIN_INPUT:
      if (pins->mclk == NULL) {
        // MCLK pin is not available
        return ARM_SAI_ERROR_MCLK_PIN;
      }
      // Audio clock source is MCLK
      reg_mode |= (1U & I2S_TX_RX_MODE_CLKSEL_MSK);
      break;
    default: return ARM_SAI_ERROR_MCLK_PIN;
  }

  // Frame length
  val = ((arg1 & ARM_SAI_FRAME_LENGTH_Msk) >> ARM_SAI_FRAME_LENGTH_Pos) + 1;
  if ((val != 0U) && (val != (data_bits * 2))) { return ARM_SAI_ERROR_FRAME_LENGHT; }

  // Audio Frequency
  if (master == 1U) {
    // WS and SCK are generated only by master

    val = ((arg2 & ARM_SAI_MCLK_PRESCALER_Msk) >> ARM_SAI_MCLK_PRESCALER_Pos) + 1;

    reg_bitrate  = val / (data_bits * 2);
    reg_bitrate--;
    if (reg_bitrate > I2S_TX_RX_BITRATE_BITRATE_MSK) { return ARM_SAI_ERROR_MCLK_PRESCALER; }

    if ((reg_mode & (1U & I2S_TX_RX_MODE_CLKSEL_MSK)) == 0U) {
      // Clock source is not MCLK input

      mclk = val * (arg2 & ARM_SAI_AUDIO_FREQ_Msk);

      pclk = SystemCoreClock;

      // MCLK = pclk * (x/y) /2  ==> (x/y) = 2*MCLK/pclk
      div_exact      = (2.0 * mclk) / pclk;
      i2s_dec2fract (div_exact, &x_best, &y_best);
      div = (double)(x_best) / (double)(y_best);
      if (div_exact > div) { delta = div_exact - div;       }
      else                 { delta = div       - div_exact; }
      if (((delta * 100U) / div_exact) > I2S_FREQ_TOLERANCE) {return ARM_SAI_ERROR_AUDIO_FREQ; }
      reg_rate = ((uint32_t)y_best << I2S_TX_RX_RATE_Y_DIVIDER_POS) | ((uint32_t)x_best << I2S_TX_RX_RATE_X_DIVIDER_POS);
    }
  }

  // Save values to registers and globals
  if ((control & ARM_SAI_CONTROL_Msk) == ARM_SAI_CONFIGURE_TX) {
    i2s->info->tx.data_bits = (uint8_t)data_bits;
    i2s->info->tx.master    = (uint8_t)master;
    i2s->reg->TXRATE        = reg_rate;
    i2s->reg->TXBITRATE     = reg_bitrate;
    i2s->reg->TXMODE        = reg_mode;
    reg_daoi |= (i2s->reg->DAO & (I2S_DAO_DAI_STOP | I2S_DAO_MUTE | I2S_DAO_DAI_WS_SEL));
    if (master == 0U) {reg_daoi |= I2S_DAO_DAI_WS_SEL;}
    i2s->reg->DAO           = reg_daoi;
  } else {
    i2s->info->rx.data_bits = (uint8_t)data_bits;
    i2s->info->rx.master    = (uint8_t)master;
    i2s->reg->RXRATE        = reg_rate;
    i2s->reg->RXBITRATE     = reg_bitrate;
    i2s->reg->RXMODE        = reg_mode;
    reg_daoi |= (i2s->reg->DAI & (I2S_DAO_DAI_STOP | I2S_DAO_DAI_WS_SEL));
    if (master == 0U) {reg_daoi |= I2S_DAO_DAI_WS_SEL;}
    i2s->reg->DAI           = reg_daoi;
  }

  i2s->info->flags |= I2S_FLAG_CONFIGURED;

  return ARM_DRIVER_OK;
}

/**
  \fn          ARM_SAI_STATUS I2S_GetStatus (I2S_RESOURCES *i2s)
  \brief       Get I2S status.
  \return      SAI status \ref ARM_SAI_STATUS
*/
static ARM_SAI_STATUS I2S0_GetStatus (void) {
  ARM_SAI_STATUS status;

  status.frame_error  = i2s->info->status.frame_error;
  status.rx_busy      = i2s->info->status.rx_busy;
  status.rx_overflow  = i2s->info->status.rx_overflow;
  status.tx_busy      = i2s->info->status.tx_busy;
  status.tx_underflow = i2s->info->status.tx_underflow;

  return status;
}

/**
  \fn          void I2S_IRQHandler (I2S_RESOURCES *i2s)
  \brief       I2S Interrupt handler.
*/
void I2S_IRQHandler (void) {
  uint32_t  state, val, event, level;
  uint32_t  i, j;
  uint8_t  *ptr_buf;

  state = i2s->reg->STATE;
  event = 0U;

  if (state & I2S_STATE_IRQ) {

    // Fill TX FIFO if needed
    if (i2s->reg->IRQ & I2S_IRQ_TX_IRQ_ENABLE) {

      // Check for TX underflow
      if (i2s->info->tx.num == 0U) {
        // Set TX underflow event and flag
        i2s->info->status.tx_underflow = 1U;
        event |= ARM_SAI_EVENT_TX_UNDERFLOW;

        // Disable TX interrupt
        i2s->reg->IRQ &= ~I2S_IRQ_TX_IRQ_ENABLE;
      } else {
        // Get TX level
        level = ((i2s->reg->STATE & I2S_STATE_TX_LEVEL_MSK) >> I2S_STATE_TX_LEVEL_POS);
        if (level == 8U) { level = 0U; }
        else             { level = 7U - level; }
        

        while (((i2s->info->tx.cnt + 4U) <= i2s->info->tx.num) && (level != 0U)) {
          // Copy all available 32bit data to FIFO, until FIFO is full
          ptr_buf = i2s->info->tx.buf + i2s->info->tx.cnt;
          i2s->reg->TXFIFO = __UNALIGNED_UINT32_READ(ptr_buf);

          // Update TX buffer info
          i2s->info->tx.cnt += 4U;
          level--;
        }

        if ((i2s->info->tx.cnt < i2s->info->tx.num) && (level != 0U)) {
          while (i2s->info->tx.cnt < i2s->info->tx.num) {
            // Copy remaining data to residue buffer (data < 32bits)
            i2s->info->tx.residue_buf[i2s->info->tx.residue_cnt++] = i2s->info->tx.buf[i2s->info->tx.cnt++];
          }
        }

        if (i2s->info->tx.cnt == i2s->info->tx.num) {
          i2s->info->status.tx_busy = 0U;
          i2s->info->tx.num = 0U;
          event |= ARM_SAI_EVENT_SEND_COMPLETE;

         // Set FIFO level to 0, to detect TX underflow
          val  = i2s->reg->IRQ & ~I2S_IRQ_TX_DEPTH_IRQ_MSK;
          i2s->reg->IRQ = val;
        }
      }
    }

    // RX interrupt
    if (i2s->reg->IRQ & I2S_IRQ_RX_IRQ_ENABLE) {

      // Check for RX overflow
      if (i2s->info->rx.num == 0U) {
        // Flush residue buffer
        i2s->info->rx.residue_cnt = 0U;
        i2s->info->rx.residue_num = 0U;

        i2s->info->status.rx_overflow = 1U;
        event |= ARM_SAI_EVENT_RX_OVERFLOW;
        i2s->reg->IRQ &= ~I2S_IRQ_RX_IRQ_ENABLE;
      } else {
        // Get FIFO level
        level = (i2s->reg->STATE & I2S_STATE_RX_LEVEL_MSK) >> I2S_STATE_RX_LEVEL_POS;

        while (((i2s->info->rx.cnt + 4U) <= i2s->info->rx.num) && (level != 0U)) {
          // Read FIFO
          ptr_buf = i2s->info->rx.buf + i2s->info->rx.cnt;
          __UNALIGNED_UINT32_WRITE(ptr_buf, i2s->reg->RXFIFO);

          i2s->info->rx.cnt += 4U;
          level--;
        }

        if ((i2s->info->rx.cnt < i2s->info->rx.num) && (level != 0U)) {
          // Read FIFO
          val = i2s->reg->RXFIFO;

          j = 0U;
          for (i = 0U; i < 4U; i++) {
            if ((i2s->info->rx.cnt < i2s->info->rx.num)) {
              i2s->info->rx.buf[i2s->info->rx.cnt++] = (uint8_t)(val >> j);
            } else {
              i2s->info->rx.residue_buf[i2s->info->rx.residue_num++] = (uint8_t)(val >> j);
            }
            j += 8U;
          }
        }

        if (i2s->info->rx.cnt == i2s->info->rx.num) {
          i2s->info->status.rx_busy = 0U;
          i2s->info->rx.num = 0U;
          event |= ARM_SAI_EVENT_RECEIVE_COMPLETE;

          // Set FIFO level to full, to detect RX overflow
          val  = i2s->reg->IRQ & ~I2S_IRQ_RX_DEPTH_IRQ_MSK;
          val |= (8U << I2S_IRQ_RX_DEPTH_IRQ_POS) & I2S_IRQ_RX_DEPTH_IRQ_MSK;
          i2s->reg->IRQ = val;
        }
      }
    }
  }

  if ((event != 0U) && (i2s->info->cb_event != NULL)) {
    i2s->info->cb_event (event);
  }
}
#if ((RTE_I2S0 != 0U) &&(RTE_I2S0_DMA_TX_EN == 1U))
void I2S_GPDMA_Tx_Event (uint32_t event) {
  uint32_t evt = 0;

  switch (event) {
    case GPDMA_EVENT_TERMINAL_COUNT_REQUEST:
      // Update TX buffer info
      i2s->info->tx.cnt += (i2s->info->tx.num / 4U) * 4U;

      while (i2s->info->tx.cnt < i2s->info->tx.num) {
        // Copy remaining data to residue buffer (data < 32bits)
        i2s->info->tx.residue_buf[i2s->info->tx.residue_cnt++] = i2s->info->tx.buf[i2s->info->tx.cnt++];
      }

      // Clear TX busy flag
      i2s->info->status.tx_busy = 0U;

      // Clear TX num and enable TX interrupt to detect TX underflow
      i2s->info->tx.num = 0U;
      i2s->reg->IRQ |= I2S_IRQ_TX_IRQ_ENABLE;

      // Set Send complete event
      evt = ARM_SAI_EVENT_SEND_COMPLETE;
      break;
    case GPDMA_EVENT_ERROR:
      break;
  }
  if ((evt != 0U) && (i2s->info->cb_event != NULL)) {
    i2s->info->cb_event (evt);
  }
}
#endif

#if ((RTE_I2S0 != 0U) &&(RTE_I2S0_DMA_RX_EN == 1U))
void I2S_GPDMA_Rx_Event (uint32_t event) {
  uint32_t evt = 0U;
  uint32_t val;

  switch (event) {
    case GPDMA_EVENT_TERMINAL_COUNT_REQUEST:
      if (i2s->info->rx.cnt == i2s->info->rx.num) {
        // Clear RX busy flag
        i2s->info->status.rx_busy = 0U;

        // Set receive complete event
        evt = ARM_SAI_EVENT_RECEIVE_COMPLETE;

        // Set FIFO level to full, to detect RX overflow
        val  = i2s->reg->IRQ & ~I2S_IRQ_RX_DEPTH_IRQ_MSK;
        val |= (8U << I2S_IRQ_RX_DEPTH_IRQ_POS) & I2S_IRQ_RX_DEPTH_IRQ_MSK;
        i2s->reg->IRQ = val;
      } else {
        // Set user defined level, to retrieve remaining requested data
        val  = i2s->reg->IRQ & ~I2S_IRQ_RX_DEPTH_IRQ_MSK;
        val |= ((uint32_t)i2s->rx_fifo_level << I2S_IRQ_RX_DEPTH_IRQ_POS) & I2S_IRQ_RX_DEPTH_IRQ_MSK;
        i2s->reg->IRQ = val;
      
      }

      i2s->reg->IRQ |= I2S_IRQ_RX_IRQ_ENABLE;
      break;
    case GPDMA_EVENT_ERROR:
      break;
  }
  if ((evt != 0U) && (i2s->info->cb_event != NULL)) {
    i2s->info->cb_event (evt);
  }
}
#endif

#if (RTE_I2S0)
// SAI0 Driver Control Block
ARM_DRIVER_SAI Driver_SAI0 = {
    I2S_GetVersion,
    I2S_GetCapabilities,
    I2S0_Initialize,
    I2S0_Uninitialize,
    I2S0_PowerControl,
    I2S0_Send,
    I2S0_Receive,
    I2S0_GetTxCount,
    I2S0_GetRxCount,
    I2S0_Control,
    I2S0_GetStatus
};
#endif
