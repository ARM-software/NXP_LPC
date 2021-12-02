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
 * $Revision:    V1.5
 *
 * Driver:       Driver_MCI0
 * Configured:   via RTE_Device.h configuration file
 * Project:      MCI Driver for NXP LPC177x/8x
 * --------------------------------------------------------------------------
 * Use the following configuration settings in the middleware component
 * to connect to this driver.
 *
 *   Configuration Setting                 Value
 *   ---------------------                 -----
 *   Connect to hardware via Driver_MCI# = 0
 * -------------------------------------------------------------------------- */

/* History:
 *  Version 1.5
 *    - Removed minor compiler warnings
 *  Version 1.4
 *    - Removed Arm Compiler 6 warnings
 *  Version 1.3
 *    - Driver update to work with GPDMA_LPC17xx ver.: 1.3
 *  Version 1.2
 *    Corrected PowerControl function for conditional Power full (driver must be initialized)
 *  Version 1.1
 *    Updated initialization, uninitialization and power procedures
 *    IRQ processing optimized
 *    Data timeout handling corrected
 *  Version 1.0
 *    Initial release
 */

#include "MCI_LPC177x_8x.h"

#define ARM_MCI_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1,5)  /* driver version */

extern ARM_DRIVER_MCI Driver_MCI0;

/* DMA callback function */
void RX_DMA_Complete(uint32_t event);
/* IRQ Handler prototype */
void MCI_IRQHandler (void);

/* PCLK frequency value */
extern uint32_t PeripheralClock;
/* MCI driver state structure */
static MCI_INFO MCI = { 0 };


/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion = {
  ARM_MCI_API_VERSION,
  ARM_MCI_DRV_VERSION
};

/* Driver Capabilities */
static const ARM_MCI_CAPABILITIES DriverCapabilities = {
  MCI_CD_PIN,                                     /* cd_state          */
  0U,                                             /* cd_event          */
  MCI_WP_PIN,                                     /* wp_state          */
  MCI_PWR_PIN,                                    /* vdd               */
  0U,                                             /* vdd_1v8           */
  0U,                                             /* vccq              */
  0U,                                             /* vccq_1v8          */
  0U,                                             /* vccq_1v2          */
  MCI_BUS_WIDTH_4,                                /* data_width_4      */
  0U,                                             /* data_width_8      */
  0U,                                             /* data_width_4_ddr  */
  0U,                                             /* data_width_8_ddr  */
  1U,                                             /* high_speed        */
  0U,                                             /* uhs_signaling     */
  0U,                                             /* uhs_tuning        */
  0U,                                             /* uhs_sdr50         */
  0U,                                             /* uhs_sdr104        */
  0U,                                             /* uhs_ddr50         */
  0U,                                             /* uhs_driver_type_a */
  0U,                                             /* uhs_driver_type_c */
  0U,                                             /* uhs_driver_type_d */
  0U,                                             /* sdio_interrupt    */
  0U,                                             /* read_wait         */
  0U,                                             /* suspend_resume    */
  0U,                                             /* mmc_interrupt     */
  0U,                                             /* mmc_boot          */
  0U,                                             /* rst_n             */
  0U,                                             /* ccs               */
  0U,                                             /* ccs_timeout       */
  0U                                              /* Reserved          */
};


/**
  \fn          ARM_DRV_VERSION GetVersion (void)
  \brief       Get driver version.
  \return      \ref ARM_DRV_VERSION
*/
static ARM_DRIVER_VERSION GetVersion (void) {
  return DriverVersion;
}


/**
  \fn          ARM_MCI_CAPABILITIES GetCapabilities (void)
  \brief       Get driver capabilities.
  \return      \ref ARM_MCI_CAPABILITIES
*/
static ARM_MCI_CAPABILITIES GetCapabilities (void) {
  return DriverCapabilities;
}


/**
  \fn            int32_t Initialize (ARM_MCI_SignalEvent_t cb_event)
  \brief         Initialize the Memory Card Interface
  \param[in]     cb_event  Pointer to \ref ARM_MCI_SignalEvent
  \return        \ref execution_status
*/
static int32_t Initialize (ARM_MCI_SignalEvent_t cb_event) {

  if (MCI.flags & MCI_INIT) { return ARM_DRIVER_OK; }

  SystemCoreClockUpdate();

  /* Configure SD_CMD, SD_CLK and SD_DAT0 pins */
  PIN_Configure (RTE_SD_CMD_PORT,  RTE_SD_CMD_PIN,  2U | IOCON_MODE_PLAIN | IOCON_SLEW_ENABLE);
  PIN_Configure (RTE_SD_CLK_PORT,  RTE_SD_CLK_PIN,  2U | IOCON_MODE_PLAIN | IOCON_SLEW_ENABLE);
  PIN_Configure (RTE_SD_DAT0_PORT, RTE_SD_DAT0_PIN, 2U | IOCON_MODE_PLAIN | IOCON_SLEW_ENABLE);

  #if (MCI_BUS_WIDTH_4 != 0U)
  PIN_Configure (RTE_SD_DAT1_PORT, RTE_SD_DAT1_PIN, 2U | IOCON_MODE_PLAIN | IOCON_SLEW_ENABLE);
  PIN_Configure (RTE_SD_DAT2_PORT, RTE_SD_DAT2_PIN, 2U | IOCON_MODE_PLAIN | IOCON_SLEW_ENABLE);
  PIN_Configure (RTE_SD_DAT3_PORT, RTE_SD_DAT3_PIN, 2U | IOCON_MODE_PLAIN | IOCON_SLEW_ENABLE);
  #endif

  /* Configure CD (Card Detect) Pin */
  #if (MCI_CD_PIN != 0U)
  GPIO_SetDir (RTE_SD_CD_PORT, RTE_SD_CD_PIN, GPIO_DIR_INPUT);
  PIN_Configure (RTE_SD_CD_PORT, RTE_SD_CD_PIN, IOCON_DIGITIAL_MODE | IOCON_MODE_PLAIN);
  #endif

  /* Configure WP (Write Protect) Pin */
  #if (MCI_WP_PIN != 0U)
  GPIO_SetDir (RTE_SD_WP_PORT, RTE_SD_WP_PIN, GPIO_DIR_INPUT);
  PIN_Configure (RTE_SD_WP_PORT, RTE_SD_WP_PIN, IOCON_DIGITIAL_MODE | IOCON_MODE_PLAIN);
  #endif

  /* Configure SD_PWR (Power Supply Enable) Pin */
  #if (MCI_PWR_PIN != 0U)
  PIN_Configure (RTE_SD_PWR_PORT, RTE_SD_PWR_PIN, 2U | IOCON_MODE_PLAIN);
  
  if (RTE_SD_PWR_ACTIVE == 0) {
    /* Configure MCIPWR Active Level in System Control register */
    LPC_SC->SCS &= (uint32_t)~SC_SCS_MCIPWRAL;
  }
  else {
    LPC_SC->SCS |=  SC_SCS_MCIPWRAL;
  }
  #endif

  /* DMA Initialize */
  GPDMA_Initialize ();

  /* Enable DMA request from SD peripheral */
  GPDMA_PeripheralSelect (GPDMA_CONN_SD_CARD, 0U);

  /* Clear control structure */
  memset (&MCI, 0, sizeof (MCI_INFO));

  MCI.cb_event = cb_event;
  MCI.flags    = MCI_INIT;

  return ARM_DRIVER_OK;
}


/**
  \fn            int32_t Uninitialize (void)
  \brief         De-initialize Memory Card Interface.
  \return        \ref execution_status
*/
static int32_t Uninitialize (void) {

  MCI.flags = 0U;

  /* Disable DMA request from SD peripheral */
  GPDMA_PeripheralSelect (GPDMA_CONN_SD_CARD, 0U);

  /* DMA Uninitialize */
  GPDMA_Uninitialize ();

  /* Reset SD_CMD, SD_CLK and SD_DAT0 pins to default state */
  PIN_Configure (RTE_SD_CMD_PORT,  RTE_SD_CMD_PIN,  0U);
  PIN_Configure (RTE_SD_CLK_PORT,  RTE_SD_CLK_PIN,  0U);
  PIN_Configure (RTE_SD_DAT0_PORT, RTE_SD_DAT0_PIN, 0U);

  #if (MCI_BUS_WIDTH_4 != 0U)
  PIN_Configure (RTE_SD_DAT1_PORT, RTE_SD_DAT1_PIN, 0U);
  PIN_Configure (RTE_SD_DAT2_PORT, RTE_SD_DAT2_PIN, 0U);
  PIN_Configure (RTE_SD_DAT3_PORT, RTE_SD_DAT3_PIN, 0U);
  #endif

  /* Configure CD (Card Detect) Pin */
  #if (MCI_CD_PIN != 0U)
  GPIO_SetDir (RTE_SD_CD_PORT, RTE_SD_CD_PIN, GPIO_DIR_INPUT);
  PIN_Configure (RTE_SD_CD_PORT, RTE_SD_CD_PIN, 0U);
  #endif

  /* Configure WP (Write Protect) Pin */
  #if (MCI_WP_PIN != 0U)
  GPIO_SetDir (RTE_SD_WP_PORT, RTE_SD_WP_PIN, GPIO_DIR_INPUT);
  PIN_Configure (RTE_SD_WP_PORT, RTE_SD_WP_PIN, 0U);
  #endif

  /* Configure SD_PWR (Power Supply Enable) Pin */
  #if (MCI_PWR_PIN != 0U)
  PIN_Configure (RTE_SD_PWR_PORT, RTE_SD_PWR_PIN, 0U);
  #endif

  return ARM_DRIVER_OK;
}


/**
  \fn            int32_t PowerControl (ARM_POWER_STATE state)
  \brief         Control Memory Card Interface Power.
  \param[in]     state   Power state \ref ARM_POWER_STATE
  \return        \ref execution_status
*/
static int32_t PowerControl (ARM_POWER_STATE state) {

  if ((state != ARM_POWER_OFF)  &&
      (state != ARM_POWER_FULL) &&
      (state != ARM_POWER_LOW)) {
      return ARM_DRIVER_ERROR_PARAMETER;
  }

  switch (state) {
    case ARM_POWER_OFF:
      /* Disable SDIO interrupts in NVIC */
      NVIC_DisableIRQ(MCI_IRQn);

      /* Disable DMA channel */
      GPDMA_ChannelDisable ((uint32_t)RTE_SD_DMA_CH);

      MCI.flags &= ~MCI_POWER;

      /* Clear status */
      MCI.status.command_active   = 0U;
      MCI.status.command_timeout  = 0U;
      MCI.status.command_error    = 0U;
      MCI.status.transfer_active  = 0U;
      MCI.status.transfer_timeout = 0U;
      MCI.status.transfer_error   = 0U;
      MCI.status.sdio_interrupt   = 0U;
      MCI.status.ccs              = 0U;

      /* Reset/Dereset SDIO peripheral */
      LPC_SC->RSTCON0 |= (1U << 28); /* RSTSDC */
      __NOP(); __NOP(); __NOP(); __NOP(); 
      LPC_SC->RSTCON0 &= ~(1U << 28); /* RSTSDC */

      /* MCI peripheral clock disable */
      LPC_SC->PCONP &= ~(1U << 28); /* PCSDC */
      break;

    case ARM_POWER_FULL:
      if ((MCI.flags & MCI_INIT)  == 0U) {
        return ARM_DRIVER_ERROR;
      }
      if ((MCI.flags & MCI_POWER) != 0U) {
        return ARM_DRIVER_OK;
      }

      /* Clear response and transfer variables */
      MCI.response = NULL;
      MCI.xfer.cnt = NULL;

      /* Enable MCI peripheral clock */
      LPC_SC->PCONP |= (1U << 28); /* PCSDC */

      /* Reset/Dereset SDIO peripheral */
      LPC_SC->RSTCON0 |= (1U << 28); /* RSTSDC */
      __NOP(); __NOP(); __NOP(); __NOP(); 
      LPC_SC->RSTCON0 &= ~(1U << 28); /* RSTSDC */

      /* Enable SDIO peripheral interrupts */
      LPC_MCI->MASK0 = MCI_MASK0_DATAEND     |
                       MCI_MASK0_STARTBITERR |
                       MCI_MASK0_CMDSENT     |
                       MCI_MASK0_CMDRESPEND  |
                       MCI_MASK0_DATATIMEOUT |
                       MCI_MASK0_CMDTIMEOUT  |
                       MCI_MASK0_DATACRCFAIL |
                       MCI_MASK0_CMDCRCFAIL  ;

      /* Set max data timeout */
      LPC_MCI->DATATMR = 0xFFFFFFFFU;

      /* Enable clock to the card (SDIO_CK) */
      LPC_MCI->POWER = MCI_PWR_CTRL_1 | MCI_PWR_CTRL_0;

      /* Enable peripheral interrupts in NVIC */
      NVIC_ClearPendingIRQ(MCI_IRQn);
      NVIC_EnableIRQ(MCI_IRQn);

      MCI.flags |= MCI_POWER;
      break;

    case ARM_POWER_LOW:
      return ARM_DRIVER_ERROR_UNSUPPORTED;
  }
  return ARM_DRIVER_OK;
}


/**
  \fn            int32_t CardPower (uint32_t voltage)
  \brief         Set Memory Card supply voltage.
  \param[in]     voltage  Memory Card supply voltage
  \return        \ref execution_status
*/
static int32_t CardPower (uint32_t voltage) {

  if ((MCI.flags & MCI_POWER) == 0U) { return ARM_DRIVER_ERROR; }

  #if (MCI_PWR_PIN != 0U)
  /* Power on/off is supported */
  switch (voltage & ARM_MCI_POWER_VDD_Msk) {
    case ARM_MCI_POWER_VDD_OFF:
      LPC_MCI->POWER &= (uint32_t)~MCI_PWR_CTRL;
      return ARM_DRIVER_OK;

    case ARM_MCI_POWER_VDD_3V3:
      LPC_MCI->POWER |=  MCI_PWR_CTRL;
      return ARM_DRIVER_OK;
    
    default:
      break;
  }
  #endif
  return ARM_DRIVER_ERROR_UNSUPPORTED;
}


/**
  \fn            int32_t ReadCD (void)
  \brief         Read Card Detect (CD) state.
  \return        1:card detected, 0:card not detected, or error
*/
static int32_t ReadCD (void) {

  if ((MCI.flags & MCI_POWER) == 0U) { return ARM_DRIVER_ERROR; }

  /* Read CD (Card Detect) Pin */
  #if (MCI_CD_PIN != 0U)
  if (GPIO_PinRead (RTE_SD_CD_PORT, RTE_SD_CD_PIN) == RTE_SD_CD_ACTIVE) {
    /* Card Detect switch is active */
    return (1);
  }
  #endif
  return (0);
}


/**
  \fn            int32_t ReadWP (void)
  \brief         Read Write Protect (WP) state.
  \return        1:write protected, 0:not write protected, or error
*/
static int32_t ReadWP (void) {

  if ((MCI.flags & MCI_POWER) == 0U) { return ARM_DRIVER_ERROR; }

  /* Read WP (Write Protect) Pin */
  #if (MCI_WP_PIN != 0U)
  if (GPIO_PinRead (RTE_SD_WP_PORT, RTE_SD_WP_PIN) == RTE_SD_WP_ACTIVE) {
    /* Write protect switch is active */
    return (1);
  }
  #endif
  return (0);
}


/**
  \fn            int32_t SendCommand (uint32_t  cmd,
                                      uint32_t  arg,
                                      uint32_t  flags,
                                      uint32_t *response)
  \brief         Send Command to card and get the response.
  \param[in]     cmd       Memory Card command
  \param[in]     arg       Command argument
  \param[in]     flags     Command flags
  \param[out]    response  Pointer to buffer for response
  \return        \ref execution_status
*/
static int32_t SendCommand (uint32_t cmd, uint32_t arg, uint32_t flags, uint32_t *response) {
  uint32_t i, clock;

  if (((flags & MCI_RESPONSE_EXPECTED_Msk) != 0U) && (response == NULL)) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }
  if ((MCI.flags & MCI_SETUP) == 0U) {
    return ARM_DRIVER_ERROR;
  }
  if (MCI.status.command_active) {
    return ARM_DRIVER_ERROR_BUSY;
  }
  MCI.status.command_active   = 1U;
  MCI.status.command_timeout  = 0U;
  MCI.status.command_error    = 0U;
  MCI.status.transfer_timeout = 0U;
  MCI.status.transfer_error   = 0U;
  MCI.status.ccs              = 0U;

  if (flags & ARM_MCI_CARD_INITIALIZE) {
    clock = LPC_MCI->CLOCK;

    if (((clock & MCI_CLOCK_ENABLE) == 0) || ((clock & MCI_CLOCK_PWRSAVE) != 0)) {
      LPC_MCI->CLOCK = (LPC_MCI->CLOCK & ((uint32_t)~MCI_CLOCK_PWRSAVE)) | MCI_CLOCK_ENABLE;

      for (i = 20000U; i; i--) {
        __NOP(); /* Wait for approximate 500us @ 120MHz */
      }
      LPC_MCI->CLOCK = clock;
    }
  }

  /* Set command register value */
  cmd = MCI_COMMAND_ENABLE | (cmd & 0xFFU);

  MCI.response = response;
  MCI.flags   &= ~(MCI_RESP_CRC | MCI_RESP_LONG);

  switch (flags & ARM_MCI_RESPONSE_Msk) {
    case ARM_MCI_RESPONSE_NONE:
      /* No response expected (wait CMDSENT) */
      break;

    case ARM_MCI_RESPONSE_SHORT:
    case ARM_MCI_RESPONSE_SHORT_BUSY:
      /* Short response expected (wait CMDREND or CCRCFAIL) */
      cmd |= MCI_COMMAND_RESPONSE;
      break;

    case ARM_MCI_RESPONSE_LONG:
      MCI.flags |= MCI_RESP_LONG;
      /* Long response expected (wait CMDREND or CCRCFAIL) */
      cmd |= MCI_COMMAND_RESPONSE | MCI_COMMAND_LONGRSP;
      break;

    default:
      return ARM_DRIVER_ERROR;
  }
  if (flags & ARM_MCI_RESPONSE_CRC) {
    MCI.flags |= MCI_RESP_CRC;
  }
  if (flags & ARM_MCI_TRANSFER_DATA) {
    MCI.flags |= MCI_DATA_XFER;
  }

  /* Clear all interrupt flags */
  LPC_MCI->CLEAR = MCI_CLEAR_Msk;

  /* Send the command */
  LPC_MCI->ARGUMENT = arg;
  LPC_MCI->COMMAND  = cmd;

  return ARM_DRIVER_OK;
}


/**
  \fn            int32_t SetupTransfer (uint8_t *data,
                                        uint32_t block_count,
                                        uint32_t block_size,
                                        uint32_t mode)
  \brief         Setup read or write transfer operation.
  \param[in,out] data         Pointer to data block(s) to be written or read
  \param[in]     block_count  Number of blocks
  \param[in]     block_size   Size of a block in bytes
  \param[in]     mode         Transfer mode
  \return        \ref execution_status
*/
static int32_t SetupTransfer (uint8_t *data, uint32_t block_count, uint32_t block_size, uint32_t mode) {
  uint32_t sz, cnt, dctrl;

  if ((data == NULL) || (block_count == 0U) || (block_size == 0U)) return ARM_DRIVER_ERROR_PARAMETER;

  if (!(MCI.flags & MCI_SETUP)) {
    return ARM_DRIVER_ERROR;
  }
  if (MCI.status.transfer_active) {
    return ARM_DRIVER_ERROR_BUSY;
  }

  MCI.xfer.buf = data;
  MCI.xfer.cnt = block_count * block_size;

  cnt = MCI.xfer.cnt;
  if (cnt > 0xFFFFU) {
    cnt = 0xFFFFU;
  }

  MCI.xfer.cnt -= cnt;
  MCI.xfer.buf += cnt;

  dctrl = 0U;

  if ((mode & ARM_MCI_TRANSFER_WRITE) == 0) {
    /* Direction: From card to controller */
    MCI.flags |= MCI_DATA_READ;
    dctrl |= MCI_DATACTRL_DIRECTION;
  }
  else {
    MCI.flags &= ~MCI_DATA_READ;
  }

  if (mode & ARM_MCI_TRANSFER_STREAM) {
    /* Stream or SDIO multibyte data transfer enable */
    dctrl |= MCI_DATACTRL_MODE;
  }
  
  /* Set data block size */
  if (block_size == 512U) {
    sz = 9U;
  }
  else {
    if (block_size > 16384U) {
      return ARM_DRIVER_ERROR_UNSUPPORTED;
    }
    for (sz = 0U; sz < 14U; sz++) {
      if (block_size & (1UL << sz)) {
        break;
      }
    }
  }

  if (mode & ARM_MCI_TRANSFER_WRITE) {
    /* Configure and enable TX DMA channel */
    GPDMA_ChannelConfigure ((uint32_t)RTE_SD_DMA_CH,
                            (uint32_t)data,
                            (uint32_t)&(LPC_MCI->FIFO),
                            0U,
                            GPDMA_CH_CONTROL_SBSIZE (GPDMA_BSIZE_8)    |
                            GPDMA_CH_CONTROL_DBSIZE (GPDMA_BSIZE_8)    |
                            GPDMA_CH_CONTROL_SWIDTH (GPDMA_WIDTH_WORD) |
                            GPDMA_CH_CONTROL_DWIDTH (GPDMA_WIDTH_WORD) |
                            GPDMA_CH_CONTROL_SI                        |
                            GPDMA_CH_CONTROL_I,
                            GPDMA_CH_CONFIG_DEST_PERI (GPDMA_CONN_SD_CARD)              |
                            GPDMA_CH_CONFIG_FLOWCNTRL (GPDMA_TRANSFER_M2P_CTRL_DST_PER) |
                            GPDMA_CH_CONFIG_IE                                          |
                            GPDMA_CH_CONFIG_ITC                                         |
                            GPDMA_CH_CONFIG_E,
                            NULL);
  }
  else {
    /* Configure and enable RX DMA channel */
    GPDMA_ChannelConfigure ((uint32_t)RTE_SD_DMA_CH,
                            (uint32_t)&(LPC_MCI->FIFO),
                            (uint32_t)data,
                            0U,
                            GPDMA_CH_CONTROL_SBSIZE (GPDMA_BSIZE_8)    |
                            GPDMA_CH_CONTROL_DBSIZE (GPDMA_BSIZE_8)    |
                            GPDMA_CH_CONTROL_SWIDTH (GPDMA_WIDTH_WORD) |
                            GPDMA_CH_CONTROL_DWIDTH (GPDMA_WIDTH_WORD) |
                            GPDMA_CH_CONTROL_DI                        |
                            GPDMA_CH_CONTROL_I,
                            GPDMA_CH_CONFIG_SRC_PERI (GPDMA_CONN_SD_CARD)               |
                            GPDMA_CH_CONFIG_FLOWCNTRL (GPDMA_TRANSFER_P2M_CTRL_SRC_PER) |
                            GPDMA_CH_CONFIG_IE                                          |
                            GPDMA_CH_CONFIG_ITC                                         |
                            GPDMA_CH_CONFIG_E,
                            RX_DMA_Complete);
  }

  MCI.dlen   = cnt;
  MCI.dctrl  = dctrl | (sz << 4) | MCI_DATACTRL_DMAENABLE;

  return ARM_DRIVER_OK;
}


/**
  \fn            int32_t AbortTransfer (void)
  \brief         Abort current read/write data transfer.
  \return        \ref execution_status
*/
static int32_t AbortTransfer (void) {
  int32_t  status;
  uint32_t mask;

  if ((MCI.flags & MCI_SETUP) == 0U) { return ARM_DRIVER_ERROR; }

  status = ARM_DRIVER_OK;

  /* Disable SDIO interrupts */
  mask = LPC_MCI->MASK0;
  LPC_MCI->MASK0 = 0U;

  /* Disable DMA and clear data transfer bit */
  LPC_MCI->DATACTRL &= ~(-MCI_DATACTRL_DMAENABLE | MCI_DATACTRL_ENABLE);

  GPDMA_ChannelDisable ((uint32_t)RTE_SD_DMA_CH);

  /* Clear SDIO FIFO */
  while (LPC_MCI->FIFOCNT) {
    LPC_MCI->FIFO;
  }

  MCI.status.command_active  = 0U;
  MCI.status.transfer_active = 0U;
  MCI.status.sdio_interrupt  = 0U;
  MCI.status.ccs             = 0U;

  /* Clear pending SDIO interrupts */
  LPC_MCI->CLEAR = MCI_CLEAR_Msk;

  /* Enable SDIO interrupts */
  LPC_MCI->MASK0 = mask;

  return status;
}


/**
  \fn            int32_t Control (uint32_t control, uint32_t arg)
  \brief         Control MCI Interface.
  \param[in]     control  Operation
  \param[in]     arg      Argument of operation (optional)
  \return        \ref execution_status
*/
static int32_t Control (uint32_t control, uint32_t arg) {
  uint32_t val, clkdiv, pclk, bps;

  if ((MCI.flags & MCI_POWER) == 0U) { return ARM_DRIVER_ERROR; }

  switch (control) {
    case ARM_MCI_BUS_SPEED:
      /* Determine clock divider and set bus speed */
      pclk = PeripheralClock;
      bps  = arg;
      
      if ((bps == pclk) && (pclk <= 50000000)) {
        LPC_MCI->CLOCK |= MCI_CLOCK_BYPASS | MCI_CLOCK_ENABLE;
      }
      else {
        clkdiv = (pclk/2 + bps - 1) / bps;
        
        if (clkdiv > 0)   { clkdiv -= 1; }
        if (clkdiv > 0xFF){ clkdiv  = 0xFF; }

        LPC_MCI->CLOCK = (LPC_MCI->CLOCK & ((uint32_t)~MCI_CLOCK_CLKDIV))  |
                          MCI_CLOCK_ENABLE | clkdiv;
        bps = pclk / 2 * (clkdiv + 1U);
      }

      for (val = (pclk/5000000U)*20U; val; val--) {
        __NOP(); /* Wait a bit to get stable clock */
      }

      /* Bus speed configured */
      MCI.flags |= MCI_SETUP;
      return ((int32_t)bps);

    case ARM_MCI_BUS_SPEED_MODE:
      switch (arg) {
        case ARM_MCI_BUS_DEFAULT_SPEED:
          /* Speed mode up to 25MHz */
        case ARM_MCI_BUS_HIGH_SPEED:
          /* Speed mode up to 50MHz */
          break;
        default:
          return ARM_DRIVER_ERROR_UNSUPPORTED;
      }
      break;

    case ARM_MCI_BUS_CMD_MODE:
      switch (arg) {
        case ARM_MCI_BUS_CMD_OPEN_DRAIN:
          /* Configure command line in open-drain mode */
          LPC_MCI->POWER |=  MCI_PWR_OPENDRAIN;
          break;
        case ARM_MCI_BUS_CMD_PUSH_PULL:
          /* Configure command line in push-pull mode */
          LPC_MCI->POWER &= (uint32_t)~MCI_PWR_OPENDRAIN;
          break;
        default:
          return ARM_DRIVER_ERROR_UNSUPPORTED;
      }
      break;

    case ARM_MCI_BUS_DATA_WIDTH:
      switch (arg) {
        case ARM_MCI_BUS_DATA_WIDTH_1:
          /* Configure 1-bit data bus width */
          LPC_MCI->CLOCK &= (uint32_t)~MCI_CLOCK_WIDEBUS;
          break;
        case ARM_MCI_BUS_DATA_WIDTH_4:
          /* Configure 4-bit data bus width */
          LPC_MCI->CLOCK |=  MCI_CLOCK_WIDEBUS;
          break;
        case ARM_MCI_BUS_DATA_WIDTH_8:
        default:
          return ARM_DRIVER_ERROR_UNSUPPORTED;
      }
      break;

    case ARM_MCI_CONTROL_CLOCK_IDLE:
      if (arg) {
        /* Clock generation enabled when idle */
        LPC_MCI->CLOCK &= (uint32_t)~MCI_CLOCK_PWRSAVE;
      }
      else {
        /* Clock generation disabled when idle */
        LPC_MCI->CLOCK |=  MCI_CLOCK_PWRSAVE;
      }
      break;

    case ARM_MCI_DATA_TIMEOUT:
      LPC_MCI->DATATMR = arg;
      break;

    case ARM_MCI_MONITOR_SDIO_INTERRUPT:
    case ARM_MCI_CONTROL_READ_WAIT:
    default:
      return ARM_DRIVER_ERROR_UNSUPPORTED;
  }

  return ARM_DRIVER_OK;
}


/**
  \fn            ARM_MCI_STATUS GetStatus (void)
  \brief         Get MCI status.
  \return        MCI status \ref ARM_MCI_STATUS
*/
static ARM_MCI_STATUS GetStatus (void) {
  return MCI.status;
}


/* MCI interrupt handler */
void MCI_IRQHandler (void) {
  uint32_t sta, icr, event;

  event = 0U;
  icr   = 0U;

  /* Read SDIO interrupt status */
  sta = LPC_MCI->STATUS;

  if (sta & MCI_STA_ERROR_Msk) {
    if (sta & MCI_STATUS_CMDCRCFAIL) {
      icr |= MCI_CLEAR_CMDCRCFAILCLR;
      /* Command response CRC check failed */
      if (MCI.flags & MCI_RESP_CRC) {
        MCI.status.command_error = 1U;

        event |= ARM_MCI_EVENT_COMMAND_ERROR;
      }
      else {
        /* Ignore CRC error and read the response */
        sta |= MCI_STATUS_CMDRESPEND;
      }
    }
    if (sta & MCI_STATUS_DATACRCFAIL) {
      icr |= MCI_CLEAR_DATACRCFAILCLR;
      /* Data block CRC check failed */
      MCI.status.transfer_error = 1U;

      event |= ARM_MCI_EVENT_TRANSFER_ERROR;
    }
    if (sta & MCI_STATUS_CMDTIMEOUT) {
      icr |= MCI_CLEAR_CMDTIMEOUTCLR;
      /* Command response timeout */
      MCI.status.command_timeout = 1U;

      event |= ARM_MCI_EVENT_COMMAND_TIMEOUT;
    }
    if (sta & MCI_STATUS_DATATIMEOUT) {
      icr |= MCI_CLEAR_DATATIMEOUTCLR;
      /* Data timeout */
      MCI.status.transfer_timeout = 1U;

      event |= ARM_MCI_EVENT_TRANSFER_TIMEOUT;
    }
    if (sta & MCI_STATUS_STARTBITERR) {
      icr |= MCI_CLEAR_STARTBITERRCLR;
      /* Start bit not detected on all data signals */
      event |= ARM_MCI_EVENT_TRANSFER_ERROR;
    }
  }
  if (sta & MCI_STATUS_CMDRESPEND) {
    icr |= MCI_CLEAR_CMDRESPENDCLR;
    /* Command response received */
    event |= ARM_MCI_EVENT_COMMAND_COMPLETE;

    if (MCI.response) {
      /* Read response registers */
      if (MCI.flags & MCI_RESP_LONG) {
        MCI.response[0] = LPC_MCI->RESP3;
        MCI.response[1] = LPC_MCI->RESP2;
        MCI.response[2] = LPC_MCI->RESP1;
        MCI.response[3] = LPC_MCI->RESP0;
      }
      else {
        MCI.response[0] = LPC_MCI->RESP0;
      }
    }
    if (MCI.flags & MCI_DATA_XFER) {
      MCI.flags &= ~MCI_DATA_XFER;

      /* Start data transfer */
      LPC_MCI->DATALEN  = MCI.dlen;
      LPC_MCI->DATACTRL = MCI.dctrl | MCI_DATACTRL_ENABLE;

      MCI.status.transfer_active = 1U;
    }
  }
  if (sta & MCI_STATUS_CMDSENT) {
    icr |= MCI_CLEAR_CMDSENTCLR;
    /* Command sent (no response required) */
    event |= ARM_MCI_EVENT_COMMAND_COMPLETE;
  }
  if (sta & MCI_STATUS_DATAEND) {
    icr |= MCI_CLEAR_DATAENDCLR;
    /* Data end (DCOUNT is zero) */
    if ((MCI.flags & MCI_DATA_READ) == 0) {
    /* Write transfer */
      LPC_MCI->MASK0 |= MCI_MASK0_DATABLOCKEND;
    }
  }
  if (sta & MCI_STATUS_DATABLOCKEND) {
    icr |= MCI_CLEAR_DATABLOCKENDCLR;
    /* Data block sent/received (CRC check passed) */
    if ((MCI.flags & MCI_DATA_READ) == 0) {
      /* Write transfer */
      if (MCI.xfer.cnt == 0) {
        event |= ARM_MCI_EVENT_TRANSFER_COMPLETE;
      }
    }
    LPC_MCI->MASK0 &= ~MCI_MASK0_DATABLOCKEND;
  }

  /* Clear processed interrupts */
  LPC_MCI->CLEAR = icr;

  if (event & MCI_TRANSFER_EVENT_Msk) {
    MCI.status.transfer_active = 0U;

    if (MCI.cb_event) {
      if (event & ARM_MCI_EVENT_TRANSFER_ERROR) {
        (MCI.cb_event)(ARM_MCI_EVENT_TRANSFER_ERROR);
      }
      else if (event & ARM_MCI_EVENT_TRANSFER_TIMEOUT) {
        (MCI.cb_event)(ARM_MCI_EVENT_TRANSFER_TIMEOUT);
      }
      else {
        (MCI.cb_event)(ARM_MCI_EVENT_TRANSFER_COMPLETE);
      }
    }
  }

  if (event & MCI_COMMAND_EVENT_Msk) {
    MCI.status.command_active = 0U;

    if (MCI.cb_event) {
      if (event & ARM_MCI_EVENT_COMMAND_ERROR) {
        (MCI.cb_event)(ARM_MCI_EVENT_COMMAND_ERROR);
      }
      else if (event & ARM_MCI_EVENT_COMMAND_TIMEOUT) {
        (MCI.cb_event)(ARM_MCI_EVENT_COMMAND_TIMEOUT);
      }
      else {
        (MCI.cb_event)(ARM_MCI_EVENT_COMMAND_COMPLETE);
      }
    }
  }
}

/* Rx DMA Callback */
void RX_DMA_Complete (uint32_t event) {
  (void)event;

  MCI.status.transfer_active = 0U;

  if (MCI.cb_event) {
    (MCI.cb_event)(ARM_MCI_EVENT_TRANSFER_COMPLETE);
  }
}

/* MCI Driver Control Block */
ARM_DRIVER_MCI Driver_MCI0 = {
  GetVersion,
  GetCapabilities,
  Initialize,
  Uninitialize,
  PowerControl,
  CardPower,
  ReadCD,
  ReadWP,
  SendCommand,
  SetupTransfer,
  AbortTransfer,
  Control,
  GetStatus
};
