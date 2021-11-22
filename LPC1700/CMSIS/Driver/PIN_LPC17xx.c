/* -------------------------------------------------------------------------- 
 * Copyright (c) 2013-2019 Arm Limited (or its affiliates). All 
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
 * $Date:        01. August 2019
 * $Revision:    V1.1
 *
 * Project:      PIN Driver for NXP LPC17xx
 * -------------------------------------------------------------------------- */

#include "LPC17xx.h"
#include "PIN_LPC17xx.h"

#define PIN_Func(port,regidx) (*((volatile uint32_t *) (&(LPC_PINCON->PINSEL0)     + 2U*port + regidx)))
#define PIN_Mode(port,regidx) (*((volatile uint32_t *) (&(LPC_PINCON->PINMODE0)    + 2U*port + regidx)))
#define PIN_ModeOp(port)      (*((volatile uint32_t *) (&(LPC_PINCON->PINMODE_OD0) + port)))

// I2C Pin Configuration register bit description
#define PIN_I2CPADCFG_SDADRV0   ((uint32_t)(1 << 0))
#define PIN_I2CPADCFG_SDAI2C0   ((uint32_t)(1 << 1))
#define PIN_I2CPADCFG_SCLDRV0   ((uint32_t)(1 << 2))
#define PIN_I2CPADCFG_SCLI2C0   ((uint32_t)(1 << 3))

/**
  \fn          int32_t PIN_Configure (uint32_t function) {
  \brief       Set pin function and electrical characteristics
  \param[in]   port       port number (0..3)
  \param[in]   pin        pin number  (0..31)
  \param[in]   function   port pin function
  \param[in]   mode       port pin input mode
  \param[in]   open_drain port pin open drain mode
  \returns
   - \b  0: function succeeded
   - \b -1: function failed
*/
int32_t PIN_Configure (uint8_t port, uint8_t pin, uint8_t function, uint8_t mode, uint8_t open_drain) {
  uint32_t regidx = 0;
  uint8_t pinnum = pin;

  if (pin >= 16) {
    pinnum   = pin - 16U;
    regidx = 1U;
  }
  // Configure Pin function
  PIN_Func(port, regidx) &= ~(0x03UL << (pinnum * 2U));
  PIN_Func(port, regidx) |= ((uint32_t)function) << (pinnum * 2U);

  // Configure Register mode
  PIN_Mode(port, regidx) &= ~(0x03UL << (pinnum * 2U));
  PIN_Mode(port, regidx) |= ((uint32_t)mode) << (pinnum * 2U);
  
  // Configure Open drain mode
  if (open_drain == PIN_PINMODE_OPENDRAIN) {
    PIN_ModeOp(port) |=  (0x01UL << pin);
  }
  else {
    PIN_ModeOp(port) &= ~(0x01UL << pin);
  }
  return(0);  
}

/**
  \fn          int32_t PIN_ConfigureTPIU (bool enable)
  \brief       Configure trace function
  \param[in]   enable Enable or disable
                - true (1): enable
                - false(0): disable
  \returns
   - \b  0: function succeeded
   - \b -1: function failed
*/
int32_t PIN_ConfigureTPIU (bool enable)
{
  if (enable == true) {
    LPC_PINCON->PINSEL10 |=  (0x01UL << 3);
  } else {
    LPC_PINCON->PINSEL10 &= ~(0x01UL << 3);
  }
  return(0);
}

/**
  \fn          PIN_ConfigureI2C0Pins(uint8_t i2cPinMode, bool enableFilterSlewRate)
  \brief       Configure I2C0 pins
  \param[in]   i2cPinMode           I2C pin mode
                - PIN_I2C_Normal_Mode
                - PIN_I2C_Fast_Mode
                - PIN_I2C_Normal_Mode
  \param[in]   enableFilterSlewRate Enable or disable filter and slew rate
                - true (1): enable
                - false(0): disable
  \returns
   - \b  0: function succeeded
   - \b -1: function failed
*/
int32_t PIN_ConfigureI2C0Pins (uint8_t i2cPinMode, bool enableFilterSlewRate)
{
  uint32_t regVal = 0U;

  if (i2cPinMode == PIN_I2C_Fast_Plus_Mode){
    regVal |= PIN_I2CPADCFG_SCLDRV0 | PIN_I2CPADCFG_SDADRV0;
  }

  if (enableFilterSlewRate == false){
    regVal |= PIN_I2CPADCFG_SCLI2C0 | PIN_I2CPADCFG_SDAI2C0;
  }
  LPC_PINCON->I2CPADCFG = regVal;
  return(0);
}
