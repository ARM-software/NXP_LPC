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
 * $Revision:    V1.2
 *
 * Project:      PIN Driver for NXP LPC177x_8x
 * -------------------------------------------------------------------------- */

#include "PIN_LPC177x_8x.h"

#include "LPC177x_8x.h"

#define PIN_Cfg(port,pin)     (*((volatile uint32_t *) (LPC_IOCON_BASE + ((port * 32U + pin)*sizeof(uint32_t)))))

/**
  \fn          int32_t PIN_Configure (uint32_t function) {
  \brief       Set pin function and electrical characteristics
  \param[in]   port       port number (0..3)
  \param[in]   pin        pin number  (0..31)
  \param[in]   pin_cfg    pin_cfg configuration bit mask
  \returns
   - \b  0: function succeeded
   - \b -1: function failed
*/
int32_t PIN_Configure (uint8_t port, uint8_t pin, uint32_t pin_cfg) {

  if ((port > 5) || (pin > 31)) return -1;
  PIN_Cfg(port, pin) = pin_cfg;
  return(0);  
}

/**
  \fn          PIN_ConfigureI2C0Pins (uint8_t port, uint8_t pin, uint32_t i2cMode)
  \brief       Configure I2C pins
  \param[in]   port       port number (0..3)
  \param[in]   pin        pin number  (0..31)
  \param[in]   i2cMode    I2C mode
                - PIN_I2CMODE_FAST_STANDARD
                - PIN_I2CMODE_OPENDRAINIO
                - PIN_I2CMODE_FASTMODEPLUS
  \returns
   - \b  0: function succeeded
   - \b -1: function failed
*/
int32_t PIN_ConfigureI2C0Pins (uint8_t port, uint8_t pin, uint32_t i2cMode)
{

  switch(port)
  {
    case 0U:
      if ((pin == 27) || (pin == 28)) break;
      else return (-1);
    case 5U:
      if ((pin == 2 ) || (pin ==  3)) break;
      else return (-1);
    default:
      return (-1);
  }

  switch(i2cMode)
  {
    case PIN_I2CMODE_FAST_STANDARD:
      PIN_Cfg(port, pin) &= ~(IOCON_I2C_FILTER_DISABLE);
      PIN_Cfg(port, pin) &= ~(IOCON_I2CMODE_FASTPLUS);
      break;
    case PIN_I2CMODE_OPENDRAINIO:
      PIN_Cfg(port, pin) |=  (IOCON_I2C_FILTER_DISABLE);
      PIN_Cfg(port, pin) &= ~(IOCON_I2CMODE_FASTPLUS);
      break;
    case PIN_I2CMODE_FASTMODEPLUS:
      PIN_Cfg(port, pin) &= ~(IOCON_I2C_FILTER_DISABLE);
      PIN_Cfg(port, pin) |=  (IOCON_I2CMODE_FASTPLUS);
      break;
    default:
      return (-1);
  }

  return(0);
}
