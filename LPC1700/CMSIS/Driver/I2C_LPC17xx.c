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
 * $Revision:    V2.7
 *
 * Driver:       Driver_I2C0, Driver_I2C1, Driver_I2C2
 * Configured:   via RTE_Device.h configuration file
 * Project:      I2C Driver for NXP LPC17xx
 * --------------------------------------------------------------------------
 * Use the following configuration settings in the middleware component
 * to connect to this driver.
 *
 *   Configuration Setting                 Value   I2C Interface
 *   ---------------------                 -----   -------------
 *   Connect to hardware via Driver_I2C# = 0       use I2C0
 *   Connect to hardware via Driver_I2C# = 1       use I2C1
 *   Connect to hardware via Driver_I2C# = 2       use I2C2
 * -------------------------------------------------------------------------- */

/* History:
 *  Version 2.7
 *    - Removed minor compiler warnings
 *  Version 2.6
 *    - Added support for ARM Compiler 6
 *  Version 2.5
 *    - Corrected pin configuration when I2C0_SCL Pin = P1.31 and I2C0_SDA Pin = P1.30
 *  Version 2.4
 *    - Added Bus Clear Control
 *    - Corrected Pin Configuration for LPC17xx series
 *    - Corrected PowerControl function for conditional Power full (driver must be initialized)
 *  Version 2.3
 *    - Updated initialization, uninitialization and power procedures
 *  Version 2.2
 *    - Changed include for renamed RTE_Device_LPC177x_8x.h to RTE_Device.h
 *  Version 2.1
 *    - Updated to CMSIS Driver API V2.02
 *    - Added Multi-master support
 *  Version 2.0
 *    - Based on API V2.00
 *  Version 1.1
 *    - Based on API V1.10 (namespace prefix ARM_ added)
 *  Version 1.0
 *    - Initial release
 */

#include "I2C_LPC17xx.h"

#include "RTE_Device.h"
#include "RTE_Components.h"

#define ARM_I2C_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(2,7) /* driver version */

/* Interrupt Handler Prototypes */
void I2C0_IRQHandler (void);
void I2C1_IRQHandler (void);
void I2C2_IRQHandler (void);

#if ((defined(RTE_Drivers_I2C0) || \
      defined(RTE_Drivers_I2C1) || \
      defined(RTE_Drivers_I2C2))   \
     && !RTE_I2C0                  \
     && !RTE_I2C1                  \
     && !RTE_I2C2)

#error "I2C not configured in RTE_Device.h!"
#endif

#if defined (LPC175x_6x)
  // Peripheral clock divider definitions
  #define PCLKSEL_CCLK_DIV_1   (1)
  #define PCLKSEL_CCLK_DIV_2   (2)
  #define PCLKSEL_CCLK_DIV_4   (0)
  #define PCLKSEL_CCLK_DIV_8   (3)

  // I2C peripheral clock selection bit position definitions
  #define I2C0_PCLKSEL0_POS    (14)
  #define I2C1_PCLKSEL1_POS    (6)
  #define I2C2_PCLKSEL1_POS    (20)
#endif
/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion = {
  ARM_I2C_API_VERSION,
  ARM_I2C_DRV_VERSION
};

/* Driver Capabilities */
static const ARM_I2C_CAPABILITIES DriverCapabilities = {
  0U            /* supports 10-bit addressing */
};


#if (RTE_I2C0)
/* I2C0 Control Information */
static I2C_CTRL I2C0_Ctrl = { 0 };

static PIN I2C0_PIN[2] = {
  {RTE_I2C0_SCL_PORT, RTE_I2C0_SCL_PIN},
  {RTE_I2C0_SDA_PORT, RTE_I2C0_SDA_PIN}
};

/* I2C0 Resources */
static I2C_RESOURCES I2C0_Resources = {
  (I2C_TypeDef *)LPC_I2C0,
  I2C0_IRQn,  
  &I2C0_PIN[0],
  &I2C0_PIN[1],
#if (defined (LPC177x_8x) && defined (RTE_I2C0_SCL_IO_WA))
  RTE_I2C0_SCL_FUNC | IOCON_DIGITIAL_MODE,
#else
  RTE_I2C0_SCL_FUNC,
#endif
#if (defined (LPC177x_8x) && defined (RTE_I2C0_SDA_IO_WA))
  RTE_I2C0_SDA_FUNC | IOCON_DIGITIAL_MODE,
#else
  RTE_I2C0_SDA_FUNC,
#endif
{0U,0U},
#if defined (LPC177x_8x)
  RTE_I2C0_SCL_I2C_PAD,
  RTE_I2C0_SDA_I2C_PAD,
  (RTE_I2C0_SDA_FAST_PLUS && RTE_I2C0_SCL_FAST_PLUS),
  0U,
#endif
  { (1 << 7),
    &(LPC_SC->PCONP),
#if defined (LPC175x_6x)
    I2C0_PCLKSEL0_POS,
    PCLKSEL_CCLK_DIV_1,
    &(LPC_SC->PCLKSEL0)
#endif
  },
  &I2C0_Ctrl
};
#endif /* RTE_I2C0 */


#if (RTE_I2C1)
/* I2C1 Control Information */
static I2C_CTRL I2C1_Ctrl = { 0 };

static PIN I2C1_PIN[2] = {
  {RTE_I2C1_SCL_PORT, RTE_I2C1_SCL_PIN},
  {RTE_I2C1_SDA_PORT, RTE_I2C1_SDA_PIN}
};

/* I2C1 Resources */
static I2C_RESOURCES I2C1_Resources = {
  (I2C_TypeDef *)LPC_I2C1,
  I2C1_IRQn,
  &I2C1_PIN[0],
  &I2C1_PIN[1],
  RTE_I2C1_SCL_FUNC,
  RTE_I2C1_SDA_FUNC,
  {0U,0U},
#if defined (LPC177x_8x)
  0U,
  0U,
  0U,
  0U,
#endif
  { (1 << 19),
    &(LPC_SC->PCONP),
#if defined (LPC175x_6x)
    I2C1_PCLKSEL1_POS,
    PCLKSEL_CCLK_DIV_1,
    &(LPC_SC->PCLKSEL1)
#endif
  },
  &I2C1_Ctrl
};
#endif /* RTE_I2C1 */

#if (RTE_I2C2)
/* I2C2 Control Information */
static I2C_CTRL I2C2_Ctrl = { 0 };

static PIN I2C2_PIN[2] = {
  {RTE_I2C2_SCL_PORT, RTE_I2C2_SCL_PIN},
  {RTE_I2C2_SDA_PORT, RTE_I2C2_SDA_PIN}
};

/* I2C2 Resources */
static I2C_RESOURCES I2C2_Resources = {
  (I2C_TypeDef *)LPC_I2C2,
  I2C2_IRQn,
  &I2C2_PIN[0],
  &I2C2_PIN[1],
  RTE_I2C2_SCL_FUNC,
  RTE_I2C2_SDA_FUNC,
  {0U,0U},
#if defined (LPC177x_8x)
  0U,
  0U,
  0U,
  0U,
#endif
  { (1 << 26),
    &(LPC_SC->PCONP),
#if defined (LPC175x_6x)
    I2C2_PCLKSEL1_POS,
    PCLKSEL_CCLK_DIV_1,
    &(LPC_SC->PCLKSEL1)
#endif
  },
  &I2C2_Ctrl
};
#endif /* RTE_I2C2 */

// Local Function
/**
  \fn          int32_t GetI2CClockFreq (I2C_RESOURCES *i2c)
  \brief       Get current value of I2C Peripheral Clock
  \param[in]   usart     Pointer to I2C resources)
  \returns     Value of I2C clock
*/
static uint32_t GetI2CClockFreq (I2C_RESOURCES *i2c) {
#if defined (LPC175x_6x)
  uint32_t div, clk, clksel;

  clksel = (*(i2c->clk.peri_cfg) >> i2c->clk.peri_cfg_pos) & 0x00000003U;
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
  (void)i2c;
  return(PeripheralClock);
#endif
}


/**
  \fn          ARM_DRIVER_VERSION I2C_GetVersion (void)
  \brief       Get driver version.
  \return      \ref ARM_DRIVER_VERSION
*/
static ARM_DRIVER_VERSION I2C_GetVersion (void) {
  return DriverVersion;
}

/**
  \fn          ARM_I2C_CAPABILITIES I2C_GetCapabilities (void)
  \brief       Get driver capabilities.
  \return      \ref ARM_I2C_CAPABILITIES
*/
static ARM_I2C_CAPABILITIES I2C_GetCapabilities (void) {
  return DriverCapabilities;
}

/**
  \fn          int32_t I2Cx_Initialize (ARM_I2C_SignalEvent_t cb_event,
                                        I2C_RESOURCES         *i2c)
  \brief       Initialize I2C Interface.
  \param[in]   cb_event  Pointer to \ref ARM_I2C_SignalEvent
  \param[in]   i2c   Pointer to I2C resources
  \return      \ref execution_status
*/
static int32_t I2Cx_Initialize (ARM_I2C_SignalEvent_t cb_event, I2C_RESOURCES *i2c) {

  if (i2c->ctrl->flags & I2C_FLAG_INIT) { return ARM_DRIVER_OK; }

  /* Configure I2C Pins */
#if defined(LPC177x_8x)
  PIN_Configure (i2c->scl->Portnum, i2c->scl->Pinnum, (i2c->func_scl | ((i2c->i2c_pad_scl)\
                                        ? (0U) : (IOCON_MODE_PLAIN | IOCON_OPENDRAIN_MODE))));
  PIN_Configure (i2c->sda->Portnum, i2c->sda->Pinnum, (i2c->func_sda | ((i2c->i2c_pad_sda)\
                                        ? (0U) : (IOCON_MODE_PLAIN | IOCON_OPENDRAIN_MODE))));
#elif defined(LPC175x_6x)
  PIN_Configure (i2c->scl->Portnum, i2c->scl->Pinnum, i2c->func_scl, PIN_PINMODE_TRISTATE, PIN_PINMODE_OPENDRAIN);
  PIN_Configure (i2c->sda->Portnum, i2c->sda->Pinnum, i2c->func_sda, PIN_PINMODE_TRISTATE, PIN_PINMODE_OPENDRAIN);
#endif

  /* Reset Run-Time information structure */
  memset (i2c->ctrl, 0x00, sizeof (I2C_CTRL));

  i2c->ctrl->cb_event = cb_event;
  i2c->ctrl->flags    = I2C_FLAG_INIT;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t I2Cx_Uninitialize (I2C_RESOURCES *i2c)
  \brief       De-initialize I2C Interface.
  \param[in]   i2c   Pointer to I2C resources
  \return      \ref execution_status
*/
static int32_t I2Cx_Uninitialize (I2C_RESOURCES *i2c) {

  i2c->ctrl->flags = 0U;

  /* Unconfigure SCL and SDA pins */
#if defined(LPC177x_8x)
  PIN_Configure (i2c->scl->Portnum, i2c->scl->Pinnum, 0U);
  PIN_Configure (i2c->sda->Portnum, i2c->sda->Pinnum, 0U);
#elif defined(LPC175x_6x)
  PIN_Configure (i2c->scl->Portnum, i2c->scl->Pinnum, i2c->func_scl, 0U, 0U);
  PIN_Configure (i2c->sda->Portnum, i2c->sda->Pinnum, i2c->func_sda, 0U, 0U);
#endif

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t I2Cx_PowerControl (ARM_POWER_STATE state,
                                          I2C_RESOURCES   *i2c)
  \brief       Control I2C Interface Power.
  \param[in]   state  Power state
  \param[in]   i2c    Pointer to I2C resources
  \return      \ref execution_status
*/
static int32_t I2Cx_PowerControl (ARM_POWER_STATE state, I2C_RESOURCES *i2c) {
  uint32_t conset;

  if ((state != ARM_POWER_OFF)  &&
      (state != ARM_POWER_FULL) &&
      (state != ARM_POWER_LOW)) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  switch (state) {
    case ARM_POWER_OFF:

      /* Abort Master/Slave transfer */
      NVIC_DisableIRQ ((IRQn_Type)i2c->i2c_ev_irq);

      /* Enable power to I2C block */
      *(i2c->clk.reg_pwr) |= i2c->clk.reg_pwr_val;

      if (i2c->ctrl->status.busy) {
        /* Master: send STOP to I2C bus           */
        /* Slave:  enter non-addressed Slave mode */
        conset = I2C_CON_STO | i2c->ctrl->con_aa;
        i2c->reg->I2CONSET = conset; 
        i2c->reg->I2CONCLR = conset ^ I2C_CON_FLAGS;
      }

      i2c->ctrl->status.busy             = 0U;
      i2c->ctrl->status.mode             = 0U;
      i2c->ctrl->status.direction        = 0U;
      i2c->ctrl->status.general_call     = 0U;
      i2c->ctrl->status.arbitration_lost = 0U;
      i2c->ctrl->status.bus_error        = 0U;

      i2c->ctrl->stalled = 0U;
      i2c->ctrl->snum    = 0U;

      i2c->ctrl->flags  &= ~I2C_FLAG_POWER;

      /* Disable I2C Operation */
      i2c->reg->I2CONCLR = I2C_CON_AA | I2C_CON_SI | I2C_CON_STA | I2C_CON_I2EN;

      /* Disable power to I2C block */
      *i2c->clk.reg_pwr &= ~i2c->clk.reg_pwr_val;
      break;

    case ARM_POWER_FULL:
      if ((i2c->ctrl->flags & I2C_FLAG_INIT)  == 0U) { return ARM_DRIVER_ERROR; }
      if ((i2c->ctrl->flags & I2C_FLAG_POWER) != 0U) { return ARM_DRIVER_OK; }

      /* Enable power to I2C block */
      *i2c->clk.reg_pwr |= i2c->clk.reg_pwr_val;
#if defined(LPC175x_6x)
      /* Configure I2C Clock*/
      *i2c->clk.peri_cfg &= ~(3U << i2c->clk.peri_cfg_pos);
      *i2c->clk.peri_cfg |=  (i2c->clk.peri_cfg_val << i2c->clk.peri_cfg_pos);
#endif
      /* Enable I2C Operation */
      i2c->reg->I2CONCLR = I2C_CON_FLAGS;
      i2c->reg->I2CONSET = I2C_CON_I2EN;

      i2c->ctrl->stalled = 0U;
      i2c->ctrl->con_aa  = 0U;

      /* Enable I2C interrupts */
      NVIC_ClearPendingIRQ ((IRQn_Type)i2c->i2c_ev_irq);
      NVIC_EnableIRQ ((IRQn_Type)i2c->i2c_ev_irq);

      i2c->ctrl->flags |= I2C_FLAG_POWER;
      break;

    case ARM_POWER_LOW:
      return ARM_DRIVER_ERROR_UNSUPPORTED;
  }

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t I2Cx_MasterTransmit (uint32_t       addr,
                                            const uint8_t *data,
                                            uint32_t       num,
                                            bool           xfer_pending,
                                            I2C_RESOURCES *i2c)
  \brief       Start transmitting data as I2C Master.
  \param[in]   addr          Slave address (7-bit or 10-bit)
  \param[in]   data          Pointer to buffer with data to transmit to I2C Slave
  \param[in]   num           Number of data bytes to transmit
  \param[in]   xfer_pending  Transfer operation is pending - Stop condition will not be generated
  \param[in]   i2c           Pointer to I2C resources
  \return      \ref execution_status
*/
static int32_t I2Cx_MasterTransmit (uint32_t       addr,
                                    const uint8_t *data,
                                    uint32_t       num,
                                    bool           xfer_pending,
                                    I2C_RESOURCES *i2c) {

  if (!data || !num || (addr > 0x7FU)) {
    /* Invalid parameters */
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if (!(i2c->ctrl->flags & I2C_FLAG_SETUP)) {
    /* Driver not yet configured */
    return ARM_DRIVER_ERROR;
  }

  if (i2c->ctrl->status.busy || (i2c->ctrl->stalled & I2C_SLAVE)) {
    /* Transfer operation in progress, or Slave stalled */
    return ARM_DRIVER_ERROR_BUSY;
  }

  NVIC_DisableIRQ ((IRQn_Type)i2c->i2c_ev_irq);

  /* Set control variables */
  i2c->ctrl->sla_rw  = (addr << 1U) & 0xFFU;
  i2c->ctrl->pending = xfer_pending;
  i2c->ctrl->data    = (uint8_t *)((uint32_t)data);
  i2c->ctrl->num     = num;
  i2c->ctrl->cnt     = -1;

  /* Update driver status */
  i2c->ctrl->status.busy             = 1U;
  i2c->ctrl->status.mode             = 1U;
  i2c->ctrl->status.direction        = 0U;
  i2c->ctrl->status.arbitration_lost = 0U;
  i2c->ctrl->status.bus_error        = 0U;
  if (!i2c->ctrl->stalled) {
    i2c->reg->I2CONSET = I2C_CON_STA | i2c->ctrl->con_aa;
  }

  NVIC_EnableIRQ ((IRQn_Type)i2c->i2c_ev_irq);

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t I2Cx_MasterReceive (uint32_t       addr,
                                           uint8_t       *data,
                                           uint32_t       num,
                                           bool           xfer_pending,
                                           I2C_RESOURCES *i2c)
  \brief       Start receiving data as I2C Master.
  \param[in]   addr          Slave address (7-bit or 10-bit)
  \param[out]  data          Pointer to buffer for data to receive from I2C Slave
  \param[in]   num           Number of data bytes to receive
  \param[in]   xfer_pending  Transfer operation is pending - Stop condition will not be generated
  \param[in]   i2c           Pointer to I2C resources
  \return      \ref execution_status
*/
static int32_t I2Cx_MasterReceive (uint32_t       addr,
                                   uint8_t       *data,
                                   uint32_t       num,
                                   bool           xfer_pending,
                                   I2C_RESOURCES *i2c) {

  if (!data || !num || (addr > 0x7FU)) {
    /* Invalid parameters */ 
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if (!(i2c->ctrl->flags & I2C_FLAG_SETUP)) {
    /* Driver not yet configured */
    return ARM_DRIVER_ERROR;
  }

  if (i2c->ctrl->status.busy || (i2c->ctrl->stalled & I2C_SLAVE)) {
    /* Transfer operation in progress, or Slave stalled */
    return ARM_DRIVER_ERROR_BUSY;
  }

  NVIC_DisableIRQ ((IRQn_Type)i2c->i2c_ev_irq);

  /* Set control variables */
  i2c->ctrl->sla_rw  = ((addr << 1) | 0x01U) & 0xFFU;
  i2c->ctrl->pending = xfer_pending;
  i2c->ctrl->data    = data;
  i2c->ctrl->num     = num;
  i2c->ctrl->cnt     = -1;

  /* Update driver status */
  i2c->ctrl->status.busy             = 1U;
  i2c->ctrl->status.mode             = 1U;
  i2c->ctrl->status.direction        = 0U;
  i2c->ctrl->status.arbitration_lost = 0U;
  i2c->ctrl->status.bus_error        = 0U;
  if (!i2c->ctrl->stalled) {
    i2c->reg->I2CONSET = I2C_CON_STA | i2c->ctrl->con_aa;
  }

  NVIC_EnableIRQ ((IRQn_Type)i2c->i2c_ev_irq);

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t I2Cx_SlaveTransmit (const uint8_t *data,
                                           uint32_t       num,
                                           I2C_RESOURCES *i2c)
  \brief       Start transmitting data as I2C Slave.
  \param[in]   data  Pointer to buffer with data to transmit to I2C Master
  \param[in]   num   Number of data bytes to transmit
  \param[in]   i2c   Pointer to I2C resources
  \return      \ref execution_status
*/
static int32_t I2Cx_SlaveTransmit (const uint8_t *data,
                                   uint32_t       num,
                                   I2C_RESOURCES *i2c) {

  if (!data || !num) {
    /* Invalid parameters */
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if (i2c->ctrl->status.busy || (i2c->ctrl->stalled & (I2C_MASTER | I2C_SLAVE_RX))) {
    /* Transfer operation in progress, Master stalled or Slave receive stalled */
    return ARM_DRIVER_ERROR_BUSY;
  }

  NVIC_DisableIRQ ((IRQn_Type)i2c->i2c_ev_irq);

  /* Set control variables */
  i2c->ctrl->flags &= ~I2C_FLAG_SLAVE_RX;
  i2c->ctrl->sdata  = (uint8_t *)((uint32_t)data);
  i2c->ctrl->snum   = num;
  i2c->ctrl->cnt    = -1;

  /* Update driver status */
  i2c->ctrl->status.general_call = 0U;
  i2c->ctrl->status.bus_error    = 0U;

  NVIC_EnableIRQ ((IRQn_Type)i2c->i2c_ev_irq);

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t I2Cx_SlaveReceive (uint8_t       *data,
                                          uint32_t       num,
                                          I2C_RESOURCES *i2c)
  \brief       Start receiving data as I2C Slave.
  \param[out]  data  Pointer to buffer for data to receive from I2C Master
  \param[in]   num   Number of data bytes to receive
  \param[in]   i2c   Pointer to I2C resources
  \return      \ref execution_status
*/
static int32_t I2Cx_SlaveReceive (uint8_t       *data,
                                  uint32_t       num,
                                  I2C_RESOURCES *i2c) {

  if (!data || !num) {
    /* Invalid parameters */ 
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if (i2c->ctrl->status.busy || (i2c->ctrl->stalled & (I2C_MASTER | I2C_SLAVE_TX))) {
    /* Transfer operation in progress, Master stalled or Slave transmit stalled */
    return ARM_DRIVER_ERROR_BUSY;
  }

  NVIC_DisableIRQ ((IRQn_Type)i2c->i2c_ev_irq);

  /* Set control variables */
  i2c->ctrl->flags |= I2C_FLAG_SLAVE_RX;
  i2c->ctrl->sdata  = data;
  i2c->ctrl->snum   = num;
  i2c->ctrl->cnt    = -1;

  /* Update driver status */
  i2c->ctrl->status.general_call = 0U;
  i2c->ctrl->status.bus_error    = 0U;

  NVIC_EnableIRQ ((IRQn_Type)i2c->i2c_ev_irq);

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t I2Cx_GetDataCount (I2C_RESOURCES *i2c)
  \brief       Get transferred data count.
  \return      number of data bytes transferred; -1 when Slave is not addressed by Master
*/
static int32_t I2Cx_GetDataCount (I2C_RESOURCES *i2c) {
  return (i2c->ctrl->cnt);
}

/**
  \fn          int32_t I2Cx_Control (uint32_t       control,
                                     uint32_t       arg,
                                     I2C_RESOURCES *i2c)
  \brief       Control I2C Interface.
  \param[in]   control  operation
  \param[in]   arg      argument of operation (optional)
  \param[in]   i2c      pointer to I2C resources
  \return      \ref execution_status
*/
static int32_t I2Cx_Control (uint32_t control, uint32_t arg, I2C_RESOURCES *i2c) {
  uint32_t val,clk,conset;

  if (!(i2c->ctrl->flags & I2C_FLAG_POWER)) {
    /* Driver not powered */
    return ARM_DRIVER_ERROR;
  }
  switch (control) {
    case ARM_I2C_OWN_ADDRESS:
      /* Set Own Slave Address */
      val = (arg << 1) & 0xFFU;
      if (arg & ARM_I2C_ADDRESS_GC) {
        /* General call enable */
        val |= 0x01U;
      }
      i2c->reg->I2ADR0 = val;

      /* Enable assert acknowledge */
      if (val) val = I2C_CON_AA;
      i2c->ctrl->con_aa  = val& 0xFFU;
      i2c->reg->I2CONSET = val;
      break;

    case ARM_I2C_BUS_SPEED:
      /* Set Bus Speed */
      clk = GetI2CClockFreq(i2c);
      switch (arg) {
        case ARM_I2C_BUS_SPEED_STANDARD:
          /* Standard Speed (100kHz) */
#if defined(LPC175x_6x)
          if (i2c->reg == ((I2C_TypeDef *)LPC_I2C0)) {
            PIN_ConfigureI2C0Pins(PIN_I2C_Normal_Mode, true);
          }
#elif defined(LPC177x_8x)
          if (i2c->i2c_pad_scl) {
            PIN_ConfigureI2C0Pins(i2c->scl->Portnum, i2c->scl->Pinnum, PIN_I2CMODE_FAST_STANDARD);
          }
          if (i2c->i2c_pad_sda) {
            PIN_ConfigureI2C0Pins(i2c->sda->Portnum, i2c->sda->Pinnum, PIN_I2CMODE_FAST_STANDARD);
          }
#endif
          clk /= 100000U;
          break;
        case ARM_I2C_BUS_SPEED_FAST:
          /* Fast Speed     (400kHz) */
#if defined(LPC175x_6x)
          if (i2c->reg == ((I2C_TypeDef *)LPC_I2C0)) {
            PIN_ConfigureI2C0Pins(PIN_I2C_Fast_Mode, true);
          }
#elif defined(LPC177x_8x)
          if (i2c->i2c_pad_scl) {
            PIN_ConfigureI2C0Pins(i2c->scl->Portnum, i2c->scl->Pinnum, PIN_I2CMODE_FAST_STANDARD);
          }
          if (i2c->i2c_pad_sda) {
            PIN_ConfigureI2C0Pins(i2c->sda->Portnum, i2c->sda->Pinnum, PIN_I2CMODE_FAST_STANDARD);
          }
#endif
          clk /= 400000U;
          break;
        case ARM_I2C_BUS_SPEED_FAST_PLUS:
          /* Fast+ Speed    (  1MHz) */
#if defined(LPC175x_6x)
          if (i2c->reg == ((I2C_TypeDef *)LPC_I2C0)) {
            PIN_ConfigureI2C0Pins(PIN_I2C_Fast_Plus_Mode, true);
            clk /= 1000000U;
            break;
          }
#elif defined(LPC177x_8x)
          if (i2c->i2c_fast_plus) {
            PIN_ConfigureI2C0Pins(i2c->scl->Portnum, i2c->scl->Pinnum, PIN_I2CMODE_FASTMODEPLUS);
            PIN_ConfigureI2C0Pins(i2c->sda->Portnum, i2c->sda->Pinnum, PIN_I2CMODE_FASTMODEPLUS);
            clk /= 1000000;
            break;
          }
#endif
          break;
        default:
          return ARM_DRIVER_ERROR_UNSUPPORTED;
      }
      /* Improve accuracy */
      i2c->reg->I2SCLH = clk / 2U;
      i2c->reg->I2SCLL = clk - i2c->reg->I2SCLH;

      /* Speed configured, I2C Master active */
      i2c->ctrl->flags |= I2C_FLAG_SETUP;
      break;

    case ARM_I2C_BUS_CLEAR:
      /* Execute Bus clear */
      NVIC_DisableIRQ ((IRQn_Type)i2c->i2c_ev_irq);

      i2c->reg->I2CONSET = I2C_CON_STA;
      __NOP(); __NOP(); __NOP();
      if ((i2c->reg->I2CONSET & I2C_CON_SI) == 0U) {
        for (val = 0U; val < 2048U; val++) {
          __NOP();
        }
      }
      /* Clear start and interrupt flag */
      i2c->reg->I2CONCLR = I2C_CON_STA | I2C_CON_SI;
      /* Send STOP to end the transaction */
      i2c->reg->I2CONSET = I2C_CON_STO;

      NVIC_ClearPendingIRQ ((IRQn_Type)i2c->i2c_ev_irq);
      NVIC_EnableIRQ ((IRQn_Type)i2c->i2c_ev_irq);
      return ARM_DRIVER_OK;

    case ARM_I2C_ABORT_TRANSFER:
      /* Abort Master/Slave transfer */
      NVIC_DisableIRQ ((IRQn_Type)i2c->i2c_ev_irq);

      i2c->ctrl->status.busy = 0U;
      i2c->ctrl->stalled = 0U;
      i2c->ctrl->snum    = 0U;
      /* Master: send STOP to I2C bus           */
      /* Slave:  enter non-addressed Slave mode */
      conset = I2C_CON_STO | i2c->ctrl->con_aa;
      i2c->reg->I2CONSET = conset; 
      i2c->reg->I2CONCLR = conset ^ I2C_CON_FLAGS;

      NVIC_ClearPendingIRQ ((IRQn_Type)i2c->i2c_ev_irq);
      NVIC_EnableIRQ ((IRQn_Type)i2c->i2c_ev_irq);
      break;

    default:
      return ARM_DRIVER_ERROR_UNSUPPORTED;
  }
  return ARM_DRIVER_OK;
}

/**
  \fn          ARM_I2C_STATUS I2Cx_GetStatus (I2C_RESOURCES *i2c)
  \brief       Get I2C status.
  \param[in]   i2c      pointer to I2C resources
  \return      I2C status \ref ARM_I2C_STATUS
*/
static ARM_I2C_STATUS I2Cx_GetStatus (I2C_RESOURCES *i2c) {
  return (i2c->ctrl->status);
}

/**
  \fn          void I2Cx_MasterHandler (I2C_RESOURCES *i2c)
  \brief       I2C Master state event handler.
  \param[in]   i2c  Pointer to I2C resources
  \return      I2C event notification flags
*/
static uint32_t I2Cx_MasterHandler (I2C_RESOURCES *i2c) {
  uint32_t conset = i2c->ctrl->con_aa;
  uint32_t event  = 0U;

  if (i2c->ctrl->stalled) {
    /* Master resumes with repeated START here */
    /* Stalled states: I2C_STAT_MA_DT_A        */
    /*                 I2C_STAT_MA_DR_NA       */
    i2c->ctrl->stalled = 0U;
    conset |= I2C_CON_STA;
    goto write_con;
  }

  switch (i2c->reg->I2STAT & 0xF8U) {
    case I2C_STAT_BUSERR:
      /* I2C Bus error */
      i2c->ctrl->status.bus_error = 1U;
      i2c->ctrl->status.busy      = 0U;
      i2c->ctrl->status.mode      = 0U;
      event = ARM_I2C_EVENT_BUS_ERROR      |
              ARM_I2C_EVENT_TRANSFER_DONE  |
              ARM_I2C_EVENT_TRANSFER_INCOMPLETE;
      conset |= I2C_CON_STO;
      break;

    case I2C_STAT_MA_START:
      /* START transmitted */
    case I2C_STAT_MA_RSTART:
      /* Repeated START transmitted */
      i2c->reg->I2DAT = i2c->ctrl->sla_rw;
      break;

    case I2C_STAT_MA_SLAW_NA:
      /* SLA+W transmitted, no ACK received */
    case I2C_STAT_MA_SLAR_NA:
      /* SLA+R transmitted, no ACK received */
      i2c->ctrl->status.busy = 0U;
      i2c->ctrl->status.mode = 0U;
      event = ARM_I2C_EVENT_ADDRESS_NACK   |
              ARM_I2C_EVENT_TRANSFER_DONE  |
              ARM_I2C_EVENT_TRANSFER_INCOMPLETE;
      conset |= I2C_CON_STO;
      break;

    case I2C_STAT_MA_SLAW_A:
      /* SLA+W transmitted, ACK received */
      i2c->ctrl->cnt  = 0U;
      i2c->reg->I2DAT = i2c->ctrl->data[0];
      break;

    case I2C_STAT_MA_DT_A:
      /* Data transmitted, ACK received */
      i2c->ctrl->cnt++;
      i2c->ctrl->num--;
      if (!i2c->ctrl->num) {
        goto xfer_done;
      }
      /* Send next byte */
      i2c->reg->I2DAT = i2c->ctrl->data[i2c->ctrl->cnt];
      break;

    case I2C_STAT_MA_DT_NA:
      /* Data transmitted, no ACK received */
      i2c->ctrl->status.busy = 0U;
      i2c->ctrl->status.mode = 0U;
      event = ARM_I2C_EVENT_TRANSFER_DONE  |
              ARM_I2C_EVENT_TRANSFER_INCOMPLETE;
      conset |= I2C_CON_STO;
      break;

    case I2C_STAT_MA_ALOST:
      /* Arbitration lost */
      i2c->ctrl->status.arbitration_lost = 1U;
      i2c->ctrl->status.busy             = 0U;
      i2c->ctrl->status.mode             = 0U;
      event = ARM_I2C_EVENT_ARBITRATION_LOST |
              ARM_I2C_EVENT_TRANSFER_DONE    |
              ARM_I2C_EVENT_TRANSFER_INCOMPLETE;
      break;

    case I2C_STAT_MA_SLAR_A:
      /* SLA+R transmitted, ACK received */
      i2c->ctrl->cnt = 0;
      i2c->ctrl->status.direction = 1U;
      goto upd_conset;

   case I2C_STAT_MA_DR_A:
      /* Data received, ACK returned */
      i2c->ctrl->data[i2c->ctrl->cnt++] = i2c->reg->I2DAT & 0xFFU;
      i2c->ctrl->num--;
upd_conset:
      conset = 0U;
      if (i2c->ctrl->num > 1U) {
        conset = I2C_CON_AA;
      }
      break;

    case I2C_STAT_MA_DR_NA:
      /* Data received, no ACK returned */
      i2c->ctrl->data[i2c->ctrl->cnt++] = i2c->reg->I2DAT & 0xFFU;
      i2c->ctrl->num--;
xfer_done:
      i2c->ctrl->status.busy = 0U;
      event = ARM_I2C_EVENT_TRANSFER_DONE;
      if (i2c->ctrl->pending) {
        /* Stall I2C transaction */
        NVIC_DisableIRQ ((IRQn_Type)i2c->i2c_ev_irq);
        i2c->ctrl->stalled = I2C_MASTER;
        return (event);
      }
      /* Generate STOP */
      conset |= I2C_CON_STO;
      break;
  }
write_con:
  /* Set/clear control flags */
  i2c->reg->I2CONSET = conset;
  i2c->reg->I2CONCLR = conset ^ I2C_CON_FLAGS;
  return (event);
}

/**
  \fn          void I2Cx_SlaveHandler (I2C_RESOURCES *i2c)
  \brief       I2C Slave state event handler.
  \param[in]   i2c  Pointer to I2C resources
  \return      I2C event notification flags
*/
static uint32_t I2Cx_SlaveHandler (I2C_RESOURCES *i2c) {
  uint32_t conset = 0U;
  uint32_t event  = 0U;

  switch (i2c->reg->I2STAT & 0xF8U) {
    case I2C_STAT_SL_ALOST_GC:
      /* Arbitration lost in General call */
      i2c->ctrl->status.arbitration_lost = 1U;
      break;
    
    case I2C_STAT_SL_GCA_A:
      /* General address recvd, ACK returned */
      i2c->ctrl->status.general_call     = 1U;
      goto slaw_a;

    case I2C_STAT_SL_ALOST_MW:
      /* Arbitration lost SLA+W */
      i2c->ctrl->status.arbitration_lost = 1U;
      break;
    
    case I2C_STAT_SL_SLAW_A:
      /* SLA+W received, ACK returned */
slaw_a:
      /* Stalled Slave receiver also resumes here */
      if (!i2c->ctrl->snum || !(i2c->ctrl->flags & I2C_FLAG_SLAVE_RX)) {
        /* Receive buffer unavailable */
        if (i2c->ctrl->stalled) {
          /* Already stalled, abort transaction to prevent dead-loops */
          event = ARM_I2C_EVENT_TRANSFER_DONE |
                  ARM_I2C_EVENT_TRANSFER_INCOMPLETE;
          conset = I2C_CON_STO | i2c->ctrl->con_aa;
          break;
        }
        /* Stall I2C transaction */
        NVIC_DisableIRQ ((IRQn_Type)i2c->i2c_ev_irq);
        i2c->ctrl->stalled = I2C_SLAVE_RX;
        return (ARM_I2C_EVENT_SLAVE_RECEIVE);
      }
      i2c->ctrl->status.direction = 1U;
      i2c->ctrl->status.busy      = 1U;
      i2c->ctrl->cnt     = 0;
      i2c->ctrl->stalled = 0U;
      conset = I2C_CON_AA;
      break;

    case I2C_STAT_SL_DRGC_A:
      /* Data recvd General call, ACK returned */
    case I2C_STAT_SL_DR_A:
      /* Data received, ACK returned */
      i2c->ctrl->sdata[i2c->ctrl->cnt++] = i2c->reg->I2DAT & 0xFFU;
      i2c->ctrl->snum--;
      if (i2c->ctrl->snum) {
        conset = I2C_CON_AA;
      }
      break;

    case I2C_STAT_SL_ALOST_MR:
      /* Arbitration lost SLA+R */
      i2c->ctrl->status.arbitration_lost = 1U;
      break;
    
    case I2C_STAT_SL_SLAR_A:
      /* SLA+R received, ACK returned */
      /* Stalled Slave transmitter also resumes here */
      if (!i2c->ctrl->snum || (i2c->ctrl->flags & I2C_FLAG_SLAVE_RX)) {
        /* Transmit buffer unavailable */
        if (i2c->ctrl->stalled) {
          /* Already stalled, abort transaction to prevent dead-loops */
          event = ARM_I2C_EVENT_TRANSFER_DONE |
                  ARM_I2C_EVENT_TRANSFER_INCOMPLETE;
          conset = I2C_CON_STO | i2c->ctrl->con_aa;
          break;
        }
        NVIC_DisableIRQ ((IRQn_Type)i2c->i2c_ev_irq);
        i2c->ctrl->stalled = I2C_SLAVE_TX;
        return (ARM_I2C_EVENT_SLAVE_TRANSMIT);
      }
      i2c->ctrl->status.direction = 0U;
      i2c->ctrl->status.busy      = 1U;
      i2c->ctrl->cnt     = 0;
      i2c->ctrl->stalled = 0;
      break;

    case I2C_STAT_SL_DT_A:
      /* Data transmitted, ACK received */
      i2c->reg->I2DAT = i2c->ctrl->sdata[i2c->ctrl->cnt++];
      i2c->ctrl->snum--;
      if (i2c->ctrl->snum) {
        conset = I2C_CON_AA;
      }
      break;

    case I2C_STAT_SL_DT_NA:
      /* Data transmitted, no ACK received */
    case I2C_STAT_SL_LDT_A:
      /* Last data transmitted, ACK received */
    case I2C_STAT_SL_DR_NA:
      /* Data received, no ACK returned */
    case I2C_STAT_SL_DRGC_NA:
      /* Data recvd General call, no ACK returned */
    case I2C_STAT_SL_STOP:
      /* STOP received while addressed */
      i2c->ctrl->status.busy = 0U;
      /* Slave operation completed, generate events */
      event = ARM_I2C_EVENT_TRANSFER_DONE;
      if (i2c->ctrl->status.arbitration_lost) {
        event |= ARM_I2C_EVENT_ARBITRATION_LOST;
      }
      if (i2c->ctrl->status.general_call) {
        event |= ARM_I2C_EVENT_GENERAL_CALL;
      }
      if (i2c->ctrl->snum) {
        event |= ARM_I2C_EVENT_TRANSFER_INCOMPLETE;
      }
      conset = i2c->ctrl->con_aa;
      break;

  }
  /* Set/clear control flags */
  i2c->reg->I2CONSET = conset;
  i2c->reg->I2CONCLR = conset ^ I2C_CON_FLAGS;

  return (event);
}

/**
  \fn          void I2Cx_IRQHandler (I2C_RESOURCES *i2c)
  \brief       I2C Event Interrupt handler.
  \param[in]   i2c  Pointer to I2C resources
*/
static void I2Cx_IRQHandler (I2C_RESOURCES *i2c) {
  uint32_t event;

  if (i2c->reg->I2STAT < I2C_STAT_SL_SLAW_A) {
    event = I2Cx_MasterHandler (i2c);
  }
  else {
    event = I2Cx_SlaveHandler (i2c);
  }
  /* Callback event notification */
  if (event && i2c->ctrl->cb_event) {
    i2c->ctrl->cb_event (event);
  }
}

#if (RTE_I2C0)
/* I2C0 Driver wrapper functions */
static int32_t I2C0_Initialize (ARM_I2C_SignalEvent_t cb_event) {
  return (I2Cx_Initialize (cb_event, &I2C0_Resources));
}
static int32_t I2C0_Uninitialize (void) {
  return (I2Cx_Uninitialize (&I2C0_Resources));
}
static int32_t I2C0_PowerControl (ARM_POWER_STATE state) {
  return (I2Cx_PowerControl (state, &I2C0_Resources));
}
static int32_t I2C0_MasterTransmit (uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending) {
  return (I2Cx_MasterTransmit (addr, data, num, xfer_pending, &I2C0_Resources));
}
static int32_t I2C0_MasterReceive (uint32_t addr, uint8_t *data, uint32_t num, bool xfer_pending) {
  return (I2Cx_MasterReceive (addr, data, num, xfer_pending, &I2C0_Resources));
}
static int32_t I2C0_SlaveTransmit (const uint8_t *data, uint32_t num) {
  return (I2Cx_SlaveTransmit (data, num, &I2C0_Resources));
}
static int32_t I2C0_SlaveReceive (uint8_t *data, uint32_t num) {
  return (I2Cx_SlaveReceive (data, num, &I2C0_Resources));
}
static int32_t I2C0_GetDataCount (void) {
  return (I2Cx_GetDataCount (&I2C0_Resources));
}
static int32_t I2C0_Control (uint32_t control, uint32_t arg) {
  return (I2Cx_Control (control, arg, &I2C0_Resources));
}
static ARM_I2C_STATUS I2C0_GetStatus (void) {
  return (I2Cx_GetStatus (&I2C0_Resources));
}
void I2C0_IRQHandler (void) {
  I2Cx_IRQHandler (&I2C0_Resources);
}

/* I2C0 Driver Control Block */
ARM_DRIVER_I2C Driver_I2C0 = {
  I2C_GetVersion,
  I2C_GetCapabilities,
  I2C0_Initialize,
  I2C0_Uninitialize,
  I2C0_PowerControl,
  I2C0_MasterTransmit,
  I2C0_MasterReceive,
  I2C0_SlaveTransmit,
  I2C0_SlaveReceive,
  I2C0_GetDataCount,
  I2C0_Control,
  I2C0_GetStatus
};
#endif

#if (RTE_I2C1)
/* I2C1 Driver wrapper functions */
static int32_t I2C1_Initialize (ARM_I2C_SignalEvent_t cb_event) {
  return (I2Cx_Initialize (cb_event, &I2C1_Resources));
}
static int32_t I2C1_Uninitialize (void) {
  return (I2Cx_Uninitialize (&I2C1_Resources));
}
static int32_t I2C1_PowerControl (ARM_POWER_STATE state) {
  return (I2Cx_PowerControl (state, &I2C1_Resources));
}
static int32_t I2C1_MasterTransmit (uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending) {
  return (I2Cx_MasterTransmit (addr, data, num, xfer_pending, &I2C1_Resources));
}
static int32_t I2C1_MasterReceive (uint32_t addr, uint8_t *data, uint32_t num, bool xfer_pending) {
  return (I2Cx_MasterReceive (addr, data, num, xfer_pending, &I2C1_Resources));
}
static int32_t I2C1_SlaveTransmit (const uint8_t *data, uint32_t num) {
  return (I2Cx_SlaveTransmit (data, num, &I2C1_Resources));
}
static int32_t I2C1_SlaveReceive (uint8_t *data, uint32_t num) {
  return (I2Cx_SlaveReceive (data, num, &I2C1_Resources));
}
static int32_t I2C1_GetDataCount (void) {
  return (I2Cx_GetDataCount (&I2C1_Resources));
}
static int32_t I2C1_Control (uint32_t control, uint32_t arg) {
  return (I2Cx_Control (control, arg, &I2C1_Resources));
}
static ARM_I2C_STATUS I2C1_GetStatus (void) {
  return (I2Cx_GetStatus (&I2C1_Resources));
}
void I2C1_IRQHandler (void) {
  I2Cx_IRQHandler (&I2C1_Resources);
}

/* I2C1 Driver Control Block */
ARM_DRIVER_I2C Driver_I2C1 = {
  I2C_GetVersion,
  I2C_GetCapabilities,
  I2C1_Initialize,
  I2C1_Uninitialize,
  I2C1_PowerControl,
  I2C1_MasterTransmit,
  I2C1_MasterReceive,
  I2C1_SlaveTransmit,
  I2C1_SlaveReceive,
  I2C1_GetDataCount,
  I2C1_Control,
  I2C1_GetStatus
};
#endif

#if (RTE_I2C2)
/* I2C2 Driver wrapper functions */
static int32_t I2C2_Initialize (ARM_I2C_SignalEvent_t cb_event) {
  return (I2Cx_Initialize (cb_event, &I2C2_Resources));
}
static int32_t I2C2_Uninitialize (void) {
  return (I2Cx_Uninitialize (&I2C2_Resources));
}
static int32_t I2C2_PowerControl (ARM_POWER_STATE state) {
  return (I2Cx_PowerControl (state, &I2C2_Resources));
}
static int32_t I2C2_MasterTransmit (uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending) {
  return (I2Cx_MasterTransmit (addr, data, num, xfer_pending, &I2C2_Resources));
}
static int32_t I2C2_MasterReceive (uint32_t addr, uint8_t *data, uint32_t num, bool xfer_pending) {
  return (I2Cx_MasterReceive (addr, data, num, xfer_pending, &I2C2_Resources));
}
static int32_t I2C2_SlaveTransmit (const uint8_t *data, uint32_t num) {
  return (I2Cx_SlaveTransmit (data, num, &I2C2_Resources));
}
static int32_t I2C2_SlaveReceive (uint8_t *data, uint32_t num) {
  return (I2Cx_SlaveReceive (data, num, &I2C2_Resources));
}
static int32_t I2C2_GetDataCount (void) {
  return (I2Cx_GetDataCount (&I2C2_Resources));
}
static int32_t I2C2_Control (uint32_t control, uint32_t arg) {
  return (I2Cx_Control (control, arg, &I2C2_Resources));
}
static ARM_I2C_STATUS I2C2_GetStatus (void) {
  return (I2Cx_GetStatus (&I2C2_Resources));
}
void I2C2_IRQHandler (void) {
  I2Cx_IRQHandler (&I2C2_Resources);
}

/* I2C1 Driver Control Block */
ARM_DRIVER_I2C Driver_I2C2 = {
  I2C_GetVersion,
  I2C_GetCapabilities,
  I2C2_Initialize,
  I2C2_Uninitialize,
  I2C2_PowerControl,
  I2C2_MasterTransmit,
  I2C2_MasterReceive,
  I2C2_SlaveTransmit,
  I2C2_SlaveReceive,
  I2C2_GetDataCount,
  I2C2_Control,
  I2C2_GetStatus
};
#endif
