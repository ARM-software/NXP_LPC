/* -----------------------------------------------------------------------------
 * Copyright (c) 2013-2016 ARM Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * $Date:        02. March 2016
 * $Revision:    V2.5
 *
 * Driver:       Driver_SPI2
 * Configured:   via RTE_Device.h configuration file
 * Project:      SPI Driver for NXP LPC43xx
 * --------------------------------------------------------------------------
 * Use the following configuration settings in the middleware component
 * to connect to this driver.
 *
 *   Configuration Setting                 Value   SPI Interface
 *   ---------------------                 -----   -------------
 *   Connect to hardware via Driver_SPI# = 2       use SPI2
 * -------------------------------------------------------------------------- */

/* History:
 *  Version 2.5
 *    - Corrected Bus Speed Configuration
 *    - Corrected receive function. Updated Send and transmit function
 *    - Corrected GetDataCount function
 *  Version 2.4
 *    - PowerControl for Power OFF and Uninitialize functions made unconditional.
 *    - Corrected status bit-field handling, to prevent race conditions.
 *  Version 2.3
 *    - Added support for Cortex-M0APP and Cortex-M0SUB
 *    - Corrected spi->info->mode and pin handling
 *    - Corrected 9..15-bit data frame format condition
 *  Version 2.2
 *    - Updated Control functions
 *  Version 2.1
 *    - Corrected peripheral reset
 *  Version 2.0
 *    - Initial CMSIS Driver API V2.01 release
 */

#include <string.h>
#include "SPI_LPC43xx.h"

#include "RTE_Device.h"
#include "RTE_Components.h"

#if   (defined(CORE_M0SUB))
#define MX_SPI_IRQn       M0S_SPI_IRQn
#define MX_SPI_IRQHandler M0S_SPI_IRQHandler
#elif (defined(CORE_M0))
#define MX_SPI_IRQn       M0_SPI_OR_DAC_IRQn
#define MX_SPI_IRQHandler M0_SPI_IRQHandler
#else
#define MX_SPI_IRQn       SPI_IRQn
#define MX_SPI_IRQHandler SPI_IRQHandler
#endif

extern uint32_t GetClockFreq   (uint32_t clk_src);

#define ARM_SPI_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(2,5)   // driver version

#if ((defined(RTE_Drivers_SPI2)) && (!RTE_SPI))
#error "SPI not configured in RTE_Device.h!"
#endif

// Driver Version
static const ARM_DRIVER_VERSION DriverVersion = {
  ARM_SPI_API_VERSION,
  ARM_SPI_DRV_VERSION
};

// Driver Capabilities
static const ARM_SPI_CAPABILITIES DriverCapabilities = {
  0,  // Simplex Mode (Master and Slave)
  0,  // TI Synchronous Serial Interface
  0,  // Microwire Interface
  1   // Signal Mode Fault event: \ref ARM_SPI_EVENT_MODE_FAULT
};

#if (RTE_SPI)
static SPI_INFO          SPI_Info = { 0 };
static SPI_TRANSFER_INFO SPI_Xfer;

static PIN_ID  SPI_pin_sck    = { RTE_SPI_SCK_PORT,  RTE_SPI_SCK_BIT,  RTE_SPI_SCK_FUNC };
static PIN_ID  SPI_pin_miso   = { RTE_SPI_MISO_PORT, RTE_SPI_MISO_BIT, RTE_SPI_MISO_FUNC};
static PIN_ID  SPI_pin_mosi   = { RTE_SPI_MOSI_PORT, RTE_SPI_MOSI_BIT, RTE_SPI_MOSI_FUNC};
#if (RTE_SPI_SSEL_PIN_EN == 1U)
static PIN_ID  SPI_pin_ssel   = { RTE_SPI_SSEL_PORT, RTE_SPI_SSEL_BIT, RTE_SPI_SSEL_FUNC };
static GPIO_ID SPI_gpio_ssel  = { RTE_SPI_SSEL_GPIO_PORT, RTE_SPI_SSEL_GPIO_BIT};
#endif

static SPI_RESOURCES     SPI_Resources = {
    LPC_SPI,
  { &SPI_pin_sck,
    &SPI_pin_miso,
    &SPI_pin_mosi,
#if (RTE_SPI_SSEL_PIN_EN == 1U)
    &SPI_pin_ssel,
    &SPI_gpio_ssel,
    RTE_SPI_SSEL_GPIO_FUNC,
#else
    NULL,
    NULL,
    0,
#endif
  },
    MX_SPI_IRQn,
   &SPI_Info,
   &SPI_Xfer
};
static SPI_RESOURCES    *spi = &SPI_Resources;
#endif


/**
  \fn          ARM_DRIVER_VERSION SPI_GetVersion (void)
  \brief       Get SPI driver version.
  \return      \ref ARM_DRV_VERSION
*/
static ARM_DRIVER_VERSION SPI_GetVersion (void) {
  return DriverVersion;
}

/**
  \fn          ARM_SPI_CAPABILITIES SPI_GetCapabilities (void)
  \brief       Get driver capabilities.
  \return      \ref ARM_SPI_CAPABILITIES
*/
static ARM_SPI_CAPABILITIES SPI_GetCapabilities (void) {
  return DriverCapabilities;
}

/**
  \fn          int32_t SPI_Initialize (ARM_SPI_SignalEvent_t cb_event)
  \brief       Initialize SPI Interface.
  \param[in]   cb_event  Pointer to \ref ARM_SPI_SignalEvent
  \return      \ref execution_status
*/
static int32_t SPI_Initialize (ARM_SPI_SignalEvent_t cb_event) {
  uint32_t val;

  if (spi->info->state & SPI_INITIALIZED) return ARM_DRIVER_OK;

  // Initialize SPI Run-Time Resources
  spi->info->cb_event          = cb_event;
  spi->info->status.busy       = 0U;
  spi->info->status.data_lost  = 0U;
  spi->info->status.mode_fault = 0U;

  // Clear transfer information
  memset(spi->xfer, 0, sizeof(SPI_TRANSFER_INFO));

  // Configure pins
  val = SCU_PIN_CFG_PULLUP_DIS | SCU_PIN_CFG_HIGH_SPEED_SLEW_RATE_EN | SCU_PIN_CFG_INPUT_BUFFER_EN | SCU_PIN_CFG_INPUT_FILTER_DIS;
  SCU_PinConfigure (spi->pin.sck->port,  spi->pin.sck->num,  spi->pin.sck->config_val  | val);
  SCU_PinConfigure (spi->pin.miso->port, spi->pin.miso->num, spi->pin.miso->config_val | val);
  SCU_PinConfigure (spi->pin.mosi->port, spi->pin.mosi->num, spi->pin.mosi->config_val | val);

  spi->info->state = SPI_INITIALIZED;   // SPI is initialized

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t SPI_Uninitialize (void)
  \brief       De-initialize SPI Interface.
  \return      \ref execution_status
*/
static int32_t SPI_Uninitialize (void) {

  // Unconfigure pins
  if (spi->pin.ssel != NULL) { SCU_PinConfigure     (spi->pin.ssel->port, spi->pin.ssel->num, 0U); }
                               SCU_PinConfigure     (spi->pin.sck->port,  spi->pin.sck->num,  0U);
                               SCU_PinConfigure     (spi->pin.miso->port, spi->pin.miso->num, 0U);
                               SCU_PinConfigure     (spi->pin.mosi->port, spi->pin.mosi->num, 0U);

  spi->info->state = 0U;                // SPI is uninitialized

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t SPI_PowerControl (ARM_POWER_STATE state)
  \brief       Control SPI Interface Power.
  \param[in]   state  Power state
  \return      \ref execution_status
*/
static int32_t SPI_PowerControl (ARM_POWER_STATE state) {

  switch (state) {
    case ARM_POWER_OFF:

      NVIC_DisableIRQ (spi->irq_num);   // Disable SPI IRQ in NVIC

      // Reset SPI peripheral
      LPC_RGU->RESET_CTRL1 = (RGU_RESET_CTRL1_SPI_RST | (~(LPC_RGU->RESET_ACTIVE_STATUS1)));
      while (!(LPC_RGU->RESET_ACTIVE_STATUS1 & RGU_RESET_ACTIVE_STATUS1_SPI_RST));

      if ((LPC_CGU->BASE_SPI_CLK & 1U) == 0U) {
        LPC_CCU1->CLK_SPI_CFG = ~1U;
        while (LPC_CCU1->CLK_SPI_CFG & 1U);

        // Power down, clock source set to IRC
        LPC_CGU->BASE_SPI_CLK =  1U | (1U << 24) | (1U << 11);
      }

      // Reset SPI Run-Time Resources
      spi->info->status.busy       = 0U;
      spi->info->status.data_lost  = 0U;
      spi->info->status.mode_fault = 0U;

      // Clear pending USART interrupts in NVIC
      NVIC_ClearPendingIRQ(spi->irq_num);

      // Clear transfer information
      memset(spi->xfer, 0, sizeof(SPI_TRANSFER_INFO));

      spi->info->state &= ~SPI_POWERED; // SPI is not powered
      break;

    case ARM_POWER_FULL:
      if ((spi->info->state & SPI_INITIALIZED) ==0U) { return ARM_DRIVER_ERROR; }
      if ((spi->info->state & SPI_POWERED)     !=0U) { return ARM_DRIVER_OK; }

      // Connect SPI BASE clock to PLL1
      LPC_CGU->BASE_SPI_CLK = (CLK_SRC_PLL1 << 24) | CGU_BASE_SPI_CLK_AUTOBLOCK;

      LPC_CCU1->CLK_SPI_CFG = CCU1_CLK_SPI_CFG_RUN | CCU1_CLK_SPI_CFG_AUTO;

      // Reset SPI peripheral
      LPC_RGU->RESET_CTRL1 = (RGU_RESET_CTRL1_SPI_RST | (~(LPC_RGU->RESET_ACTIVE_STATUS1)));
      while (!(LPC_RGU->RESET_ACTIVE_STATUS1 & RGU_RESET_ACTIVE_STATUS1_SPI_RST));

      // Reset SPI Run-Time Resources
      spi->info->status.busy       = 0U;
      spi->info->status.data_lost  = 0U;
      spi->info->status.mode_fault = 0U;

      spi->info->state |= SPI_POWERED;  // SPI is powered

      NVIC_ClearPendingIRQ (spi->irq_num);
      NVIC_EnableIRQ (spi->irq_num);    // Enable SPI IRQ in NVIC
      break;

    default:
      return ARM_DRIVER_ERROR_UNSUPPORTED;
  }

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t SPI_Send (const void *data, uint32_t num)
  \brief       Start sending data to SPI transmitter.
  \param[in]   data  Pointer to buffer with data to send to SPI transmitter
  \param[in]   num   Number of data items to send
  \return      \ref execution_status
*/
static int32_t SPI_Send (const void *data, uint32_t num) {
  uint16_t val;

  if ((data == NULL) || (num == 0U))        { return ARM_DRIVER_ERROR_PARAMETER; }
  if (!(spi->info->state & SPI_CONFIGURED)) { return ARM_DRIVER_ERROR; }
  if (  spi->info->status.busy)             { return ARM_DRIVER_ERROR_BUSY; }
  spi->info->status.busy       = 1U;
  spi->info->status.data_lost  = 0U;
  spi->info->status.mode_fault = 0U;

  spi->xfer->rx_buf = NULL;
  spi->xfer->tx_buf = (uint8_t *)data;

  spi->xfer->num    = num;
  spi->xfer->rx_cnt = 0U;
  spi->xfer->tx_cnt = 0U;

  val = *(spi->xfer->tx_buf++);
  if ((spi->reg->CR & SPI_CR_BITENABLE)   &&  // If data frame format != 8
     (((spi->reg->CR & SPI_CR_BITS) == 0U) || // If 16-bit data frame format
      ((spi->reg->CR & SPI_CR_BITS) >  8U))){ // If 9..15-bit data frame format
    val |= *(spi->xfer->tx_buf++) << 8;
  }
  spi->reg->DR  = val;                        // Activate send
  spi->xfer->tx_cnt++;

  spi->reg->CR |=  SPI_CR_SPIE;               // Enable SPI interrupts

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t SPI_Receive (void *data, uint32_t num)
  \brief       Start receiving data from SPI receiver.
  \param[out]  data  Pointer to buffer for data to receive from SPI receiver
  \param[in]   num   Number of data items to receive
  \return      \ref execution_status
*/
static int32_t SPI_Receive (void *data, uint32_t num) {

  if ((data == NULL) || (num == 0U))        { return ARM_DRIVER_ERROR_PARAMETER; }
  if (!(spi->info->state & SPI_CONFIGURED)) { return ARM_DRIVER_ERROR; }
  if (  spi->info->status.busy)             { return ARM_DRIVER_ERROR_BUSY; }
  spi->info->status.busy       = 1U;
  spi->info->status.data_lost  = 0U;
  spi->info->status.mode_fault = 0U;

  spi->xfer->rx_buf = (uint8_t *)data;
  spi->xfer->tx_buf = NULL;

  spi->xfer->num    = num;
  spi->xfer->rx_cnt = 0U;
  spi->xfer->tx_cnt = 0U;

  spi->reg->DR      = spi->xfer->def_val; // Activate send to generate CLK
  spi->xfer->tx_cnt++;

  spi->reg->CR     |=  SPI_CR_SPIE;       // Enable SPI interrupts

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t SPI_Transfer (const void     *data_out,
                                            void    *data_in,
                                            uint32_t num)
  \brief       Start sending/receiving data to/from SPI transmitter/receiver.
  \param[in]   data_out  Pointer to buffer with data to send to SPI transmitter
  \param[out]  data_in   Pointer to buffer for data to receive from SPI receiver
  \param[in]   num       Number of data items to transfer
  \return      \ref execution_status
*/
static int32_t SPI_Transfer (const void *data_out, void *data_in, uint32_t num) {
  uint16_t val;

  if ((data_out == NULL) || (data_in == NULL) || (num == 0U)) { return ARM_DRIVER_ERROR_PARAMETER; }
  if (!(spi->info->state & SPI_CONFIGURED))                   { return ARM_DRIVER_ERROR; }
  if (  spi->info->status.busy)                               { return ARM_DRIVER_ERROR_BUSY; }
  spi->info->status.busy       = 1U;
  spi->info->status.data_lost  = 0U;
  spi->info->status.mode_fault = 0U;

  spi->xfer->rx_buf = (uint8_t *)data_in;
  spi->xfer->tx_buf = (uint8_t *)data_out;

  spi->xfer->num    = num;
  spi->xfer->rx_cnt = 0U;
  spi->xfer->tx_cnt = 0U;

  val = *(spi->xfer->tx_buf++);
  if ((spi->reg->CR & SPI_CR_BITENABLE)   &&  // If data frame format != 8
     (((spi->reg->CR & SPI_CR_BITS) == 0U) || // If 16-bit data frame format
      ((spi->reg->CR & SPI_CR_BITS) >  8U))){ // If 9..15-bit data frame format
    val |= *(spi->xfer->tx_buf++) << 8;
  }
  spi->reg->DR  = val;                        // Activate send
  spi->xfer->tx_cnt++;

  spi->reg->CR |=  SPI_CR_SPIE;               // Enable SPI interrupts

  return ARM_DRIVER_OK;
}

/**
  \fn          uint32_t SPI_GetDataCount (void)
  \brief       Get transferred data count.
  \return      number of data items transferred
*/
static uint32_t SPI_GetDataCount (void) {

  if (!(spi->info->state & SPI_CONFIGURED)) { return 0U; }

  return spi->xfer->rx_cnt;
}

/**
  \fn          int32_t SPI_Control (uint32_t control, uint32_t arg)
  \brief       Control SPI Interface.
  \param[in]   control  Operation
  \param[in]   arg      Argument of operation (optional)
  \return      common \ref execution_status and driver specific \ref spi_execution_status
*/
static int32_t SPI_Control (uint32_t control, uint32_t arg) {
  uint32_t counter, clk, data_bits;

  if (!(spi->info->state & SPI_POWERED)) { return ARM_DRIVER_ERROR; }

  if ((control & ARM_SPI_CONTROL_Msk) == ARM_SPI_ABORT_TRANSFER) {
    spi->reg->CR &= ~SPI_CR_SPIE;           // Disable SPI interrupts
    memset(spi->xfer, 0, sizeof(SPI_TRANSFER_INFO));
    spi->info->status.busy = 0U;
    return ARM_DRIVER_OK;
  }

  if (spi->info->status.busy) { return ARM_DRIVER_ERROR_BUSY; }

  switch (control & ARM_SPI_CONTROL_Msk) {
    default:
      return ARM_DRIVER_ERROR_UNSUPPORTED;

    case ARM_SPI_MODE_INACTIVE:             // SPI Inactive
      spi->info->mode  &= ~ARM_SPI_CONTROL_Msk;
      spi->info->mode  |=  ARM_SPI_MODE_INACTIVE;
      spi->info->state &= ~SPI_CONFIGURED;
      return ARM_DRIVER_OK;

    case ARM_SPI_MODE_MASTER:               // SPI Master (Output on MOSI, Input on MISO); arg = Bus Speed in bps
      spi->reg->CR     |=  SPI_CR_MSTR;     // Set master mode
      spi->info->mode  &= ~ARM_SPI_CONTROL_Msk;
      spi->info->mode  |=  ARM_SPI_MODE_MASTER;
      spi->info->state |=  SPI_CONFIGURED;
      goto set_speed;

    case ARM_SPI_MODE_SLAVE:                // SPI Slave  (Output on MISO, Input on MOSI)
      spi->reg->CR     &= ~SPI_CR_MSTR;     // Set slave mode
      spi->info->mode  &= ~ARM_SPI_CONTROL_Msk;
      spi->info->mode  |=  ARM_SPI_MODE_SLAVE;
      spi->info->state |=  SPI_CONFIGURED;
      break;

    case ARM_SPI_MODE_MASTER_SIMPLEX:       // SPI Master (Output/Input on MOSI); arg = Bus Speed in bps
    case ARM_SPI_MODE_SLAVE_SIMPLEX:        // SPI Slave  (Output/Input on MISO)
      return ARM_SPI_ERROR_MODE;

    case ARM_SPI_SET_BUS_SPEED:             // Set Bus Speed in bps; arg = value
set_speed:
      if (arg == 0U) {
        return ARM_DRIVER_ERROR;
      }

      clk     = GetClockFreq(CLK_SRC_PLL1);
      counter = clk / arg;
      if ( counter < 8U)   { counter = 8U; }
      if ( counter > 254U) { return ARM_DRIVER_ERROR; }

      if ((counter % 2U) == 1U) {
        counter++;
      } else {
        if ((counter * arg) < clk) {
          counter += 2U;
        }
      }
      spi->reg->CCR = counter & SPI_CCR_COUNTER;
      if ((control & ARM_SPI_CONTROL_Msk) == ARM_SPI_SET_BUS_SPEED) {
        return ARM_DRIVER_OK;
      }
      break;

    case ARM_SPI_GET_BUS_SPEED:             // Get Bus Speed in bps
      return (GetClockFreq(CLK_SRC_PLL1) / (spi->reg->CCR & SPI_CCR_COUNTER));

    case ARM_SPI_SET_DEFAULT_TX_VALUE:      // Set default Transmit value; arg = value
      spi->xfer->def_val = (uint16_t)(arg & 0xFFFF);
      return ARM_DRIVER_OK;

    case ARM_SPI_CONTROL_SS:                // Control Slave Select; arg = 0:inactive, 1:active 
      if (((spi->info->mode & ARM_SPI_CONTROL_Msk)        != ARM_SPI_MODE_MASTER)  ||
          ((spi->info->mode & ARM_SPI_SS_MASTER_MODE_Msk) != ARM_SPI_SS_MASTER_SW)) {
        return ARM_DRIVER_ERROR;
      }
      if (spi->pin.gpio_ssel == NULL) {
        return ARM_DRIVER_ERROR;
      }
      if (arg == ARM_SPI_SS_INACTIVE) {
        GPIO_PinWrite  (spi->pin.gpio_ssel->port, spi->pin.gpio_ssel->num, 1U);
      } else {
        GPIO_PinWrite  (spi->pin.gpio_ssel->port, spi->pin.gpio_ssel->num, 0U);
      }
      return ARM_DRIVER_OK;
  }

  if ((spi->info->mode & ARM_SPI_CONTROL_Msk) ==  ARM_SPI_MODE_MASTER) {
    switch (control & ARM_SPI_SS_MASTER_MODE_Msk) {
      case ARM_SPI_SS_MASTER_UNUSED:        // SPI Slave Select when Master: Not used (default)
        if (spi->pin.ssel != NULL) { SCU_PinConfigure (spi->pin.ssel->port, spi->pin.ssel->num, 0U); }
        spi->info->mode  &= ~ARM_SPI_SS_MASTER_MODE_Msk;
        spi->info->mode  |=  ARM_SPI_SS_MASTER_UNUSED;
        break;

      case ARM_SPI_SS_MASTER_HW_INPUT:      // SPI Slave Select when Master: Hardware monitored Input
        spi->info->mode  &= ~ARM_SPI_SS_MASTER_MODE_Msk;
        return ARM_SPI_ERROR_SS_MODE;

      case ARM_SPI_SS_MASTER_SW:            // SPI Slave Select when Master: Software controlled
        spi->info->mode  &= ~ARM_SPI_SS_MASTER_MODE_Msk;
        if (spi->pin.ssel != NULL) {
          SCU_PinConfigure (spi->pin.ssel->port, spi->pin.ssel->num, spi->pin.gpio_ssel_af               |
                                                                     SCU_PIN_CFG_PULLUP_DIS              |
                                                                     SCU_PIN_CFG_HIGH_SPEED_SLEW_RATE_EN );
          GPIO_SetDir      (spi->pin.gpio_ssel->port, spi->pin.gpio_ssel->num, GPIO_DIR_OUTPUT);
          GPIO_PinWrite    (spi->pin.gpio_ssel->port, spi->pin.gpio_ssel->num, 1U);
          spi->info->mode |= ARM_SPI_SS_MASTER_SW;
        } else {
          return ARM_SPI_ERROR_SS_MODE;
        }
        break;

      case ARM_SPI_SS_MASTER_HW_OUTPUT:     // SPI Slave Select when Master: Hardware controlled Output
        spi->info->mode  &= ~ARM_SPI_SS_MASTER_MODE_Msk;
        if (spi->pin.ssel != NULL) {
          SCU_PinConfigure (spi->pin.ssel->port, spi->pin.ssel->num, spi->pin.ssel->config_val           |
                                                                     SCU_PIN_CFG_PULLUP_DIS              |
                                                                     SCU_PIN_CFG_HIGH_SPEED_SLEW_RATE_EN );
          spi->info->mode |= ARM_SPI_SS_MASTER_HW_OUTPUT;
        } else {
          return ARM_SPI_ERROR_SS_MODE;
        }
        break;
      default:
        break;
    }
  }

  if ((spi->info->mode & ARM_SPI_CONTROL_Msk) ==  ARM_SPI_MODE_SLAVE) {
    switch (control & ARM_SPI_SS_SLAVE_MODE_Msk) {
      case ARM_SPI_SS_SLAVE_HW:             // SPI Slave Select when Slave: Hardware monitored (default)
        spi->info->mode  &= ~ARM_SPI_SS_SLAVE_MODE_Msk;
        if (spi->pin.ssel != NULL) {
          SCU_PinConfigure (spi->pin.ssel->port, spi->pin.ssel->num, spi->pin.ssel->config_val           |
                                                                     SCU_PIN_CFG_PULLUP_DIS              |
                                                                     SCU_PIN_CFG_HIGH_SPEED_SLEW_RATE_EN |
                                                                     SCU_PIN_CFG_INPUT_BUFFER_EN         |
                                                                     SCU_PIN_CFG_INPUT_FILTER_DIS        );
          spi->info->mode |= ARM_SPI_SS_SLAVE_HW;
        } else {
          return ARM_SPI_ERROR_SS_MODE;
        }
        break;

      case ARM_SPI_SS_SLAVE_SW:             // SPI Slave Select when Slave: Software controlled
        spi->info->mode  &= ~ARM_SPI_SS_SLAVE_MODE_Msk;
        return ARM_SPI_ERROR_SS_MODE;
      default:
        break;
    }
  }

  // Configure Frame Format
  switch (control & ARM_SPI_FRAME_FORMAT_Msk) {
    case ARM_SPI_CPOL0_CPHA0:
      spi->reg->CR &= ~(SPI_CR_CPOL | SPI_CR_CPHA);
      break;

    case ARM_SPI_CPOL0_CPHA1:
      spi->reg->CR &=  ~SPI_CR_CPOL;
      spi->reg->CR |=   SPI_CR_CPHA;
      break;

    case ARM_SPI_CPOL1_CPHA0:
      spi->reg->CR |=   SPI_CR_CPOL;
      spi->reg->CR &=  ~SPI_CR_CPHA;
      break;

    case ARM_SPI_CPOL1_CPHA1:
      spi->reg->CR |=  (SPI_CR_CPOL | SPI_CR_CPHA);
      break;

    case ARM_SPI_TI_SSI:
    case ARM_SPI_MICROWIRE:
    default:
      return ARM_SPI_ERROR_FRAME_FORMAT;
  }

  // Configure Number of Data Bits
  data_bits = ((control & ARM_SPI_DATA_BITS_Msk) >> ARM_SPI_DATA_BITS_Pos);
  if         (data_bits ==  8U) {
    spi->reg->CR &= ~SPI_CR_BITENABLE;
  } else if  (data_bits == 16U) {
    spi->reg->CR |=  SPI_CR_BITENABLE;
    spi->reg->CR &= ~SPI_CR_BITS;
  } else if ((data_bits >   8U) &&
             (data_bits <  16U)) {
    spi->reg->CR |=  SPI_CR_BITENABLE;
    spi->reg->CR &= ~SPI_CR_BITS;
    spi->reg->CR |=  data_bits << 8;
  } else {
    return ARM_SPI_ERROR_DATA_BITS;
  }

  // Configure Bit Order
  if ((control & ARM_SPI_BIT_ORDER_Msk) == ARM_SPI_LSB_MSB) {
    spi->reg->CR |=  SPI_CR_LSBF;
  } else {
    spi->reg->CR &= ~SPI_CR_LSBF;
  }

  return ARM_DRIVER_OK;
}

/**
  \fn          ARM_SPI_STATUS SPI_GetStatus (void)
  \brief       Get SPI status.
  \return      SPI status \ref ARM_SPI_STATUS
*/
static ARM_SPI_STATUS SPI_GetStatus (void) {
  ARM_SPI_STATUS status;

  status.busy       = spi->info->status.busy;
  status.data_lost  = spi->info->status.data_lost;
  status.mode_fault = spi->info->status.mode_fault;

  return (status);
}

/**
  \fn          void SPI_IRQHandler (void)
  \brief       SPI Interrupt handler.
*/
void MX_SPI_IRQHandler (void) {
  uint16_t data;
  uint8_t  sr;

  sr            = spi->reg->SR;                       // Read status register
  spi->reg->INT = 1U;                                 // Clear interrupt

  if (sr & SPI_SR_MODF) {                             // Mode fault
    // Overrun flag is set
    spi->info->status.mode_fault = 1U;
    if (spi->info->cb_event) { spi->info->cb_event(ARM_SPI_EVENT_MODE_FAULT); }
  }

  if (sr & SPI_SR_ROVR) {                             // Read overrun
    // Overrun flag is set
    spi->info->status.data_lost = 1U;
    if (spi->info->cb_event) { spi->info->cb_event(ARM_SPI_EVENT_DATA_LOST); }
  }

                                                          // Handle transfer
  if (sr & SPI_SR_SPIF) {
    if (spi->xfer->num > spi->xfer->tx_cnt) {             // If data to send
      if (spi->xfer->tx_buf) {                            // If data available
        data = *(spi->xfer->tx_buf++);
        if ((spi->reg->CR & SPI_CR_BITENABLE )   &&       // If data frame format != 8
           (((spi->reg->CR & SPI_CR_BITS) == 0U)  ||      // If 16-bit data frame format
            ((spi->reg->CR & SPI_CR_BITS) >  8U))) {      // If 9..15-bit data frame format
          data |= *(spi->xfer->tx_buf++) << 8;
        }
      } else {                                            // If default data send
        data = spi->xfer->def_val;
      }
      spi->reg->DR = data;                                // Activate send
      spi->xfer->tx_cnt++;
    }

    if (spi->xfer->num > spi->xfer->rx_cnt) {
      data = spi->reg->DR;                                // Read data
      if (spi->xfer->rx_buf) {
        *(spi->xfer->rx_buf++) = (uint8_t)data;           // Put data into buffer
          if ((spi->reg->CR & SPI_CR_BITENABLE  )  &&     // If data frame format != 8
             (((spi->reg->CR & SPI_CR_BITS) == 0U)  ||    // If 16-bit data frame format
              ((spi->reg->CR & SPI_CR_BITS) >  8U))) {    // If 9..15-bit data frame format
          *(spi->xfer->rx_buf++) = (uint8_t)(data >> 8);
        }
      }
      spi->xfer->rx_cnt++;
      if (spi->xfer->rx_cnt == spi->xfer->num) {      // If all data received
        spi->reg->CR &= ~SPI_CR_SPIE;                 // Disable SPI interrupts
        spi->info->status.busy = 0U;
        if (spi->info->cb_event) { spi->info->cb_event(ARM_SPI_EVENT_TRANSFER_COMPLETE); }
      }
    }
  }
}

#ifdef CORE_M0
__weak void DAC_IRQHandler (void) {}

void M0_SPI_OR_DAC_IRQHandler (void) {
  MX_SPI_IRQHandler();
  DAC_IRQHandler();
}
#endif


// SPI2 Driver Control Block
ARM_DRIVER_SPI Driver_SPI2 = {
  SPI_GetVersion,
  SPI_GetCapabilities,
  SPI_Initialize,
  SPI_Uninitialize,
  SPI_PowerControl,
  SPI_Send,
  SPI_Receive,
  SPI_Transfer,
  SPI_GetDataCount,
  SPI_Control,
  SPI_GetStatus
};
