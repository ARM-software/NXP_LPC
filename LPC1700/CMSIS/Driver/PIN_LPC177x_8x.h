/* -------------------------------------------------------------------------- 
 * Copyright (c) 2013-2014 Arm Limited (or its affiliates). All 
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
 * $Date:        10. July 2014
 * $Revision:    V1.0
 *
 * Project:      PIN Driver Definitions for NXP LPC177x_8x
 * -------------------------------------------------------------------------- */

#ifndef __PIN_LPC177X_8X_H
#define __PIN_LPC177X_8X_H

#include <stdint.h>
#include <stdbool.h>

// Pin Configuration

typedef struct _PIN
{
  uint8_t Portnum;   // Port Number
  uint8_t Pinnum;    // Pin Number
} PIN;


//------------------------------------------------------------------------------
// IOCON REGISTER BIT DEFINITIONS
//------------------------------------------------------------------------------

// Function
#define IOCON_FUNC_POS             ((uint32_t)(0))
#define IOCON_FUNC_MASK            ((uint32_t)(0x07 << IOCON_FUNC_POS))

// Function mode (on-chip pull-up/pull-down resistor control
#define IOCON_MODE_POS             ((uint32_t)(3))
#define IOCON_MODE_PLAIN           ((uint32_t)(0 << IOCON_MODE_POS))
#define IOCON_MODE_PULLDOWN        ((uint32_t)(1 << IOCON_MODE_POS))
#define IOCON_MODE_PULLUP          ((uint32_t)(2 << IOCON_MODE_POS))
#define IOCON_MODE_REPEATER        ((uint32_t)(3 << IOCON_MODE_POS))

// Hysteresis
#define IOCON_HYS_POS              ((uint32_t)(5))
#define IOCON_HYS_ENABLE           ((uint32_t)(1 << IOCON_HYS_POS))

// Input polarity
#define IOCON_INVERT_POS           ((uint32_t)(6))
#define IOCON_INVERT_INPUT         ((uint32_t)(1 << IOCON_INVERT_POS))

// Selects Analog/Digital mode
#define IOCON_ADMODE_POS           ((uint32_t)(7))
#define IOCON_ANALOG_MODE          ((uint32_t)(0 << IOCON_ADMODE_POS))
#define IOCON_DIGITIAL_MODE        ((uint32_t)(1 << IOCON_ADMODE_POS))

// Controls Glitch Filter
#define IOCON_FILTER_POS           ((uint32_t)(8))
#define IOCON_10ns_FILTER_ENABLE   ((uint32_t)(0 << IOCON_FILTER_POS))
#define IOCON_10ns_FILTER_DISABLE  ((uint32_t)(1 << IOCON_FILTER_POS))

// I2C 50ns glitch filter and slew rate control
#define IOCON_HS_POS               ((uint32_t)(8))
#define IOCON_I2C_FILTER_ENABLE    ((uint32_t)(0 << IOCON_HS_POS))
#define IOCON_I2C_FILTER_DISABLE   ((uint32_t)(1 << IOCON_HS_POS))

// Driver Output Slew Rate Control
#define IOCON_SLEW_POS             ((uint32_t)(9))
#define IOCON_SLEW_ENABLE          ((uint32_t)(1 << IOCON_SLEW_POS))

// Controls sink current capability of the pin
#define IOCON_HIDRIVE_POS          ((uint32_t)(9))
#define IOCON_I2CMODE_FASTPLUS     ((uint32_t)(1 << IOCON_HIDRIVE_POS))

// Controls open-drain mode
#define IOCON_OD_POS               ((uint32_t)(10))
#define IOCON_OPENDRAIN_MODE       ((uint32_t)(1 << IOCON_OD_POS))

// DAC enable control
#define IOCON_DACEN_POS            ((uint32_t)(16))
#define IOCON_DAC_ENABLE           ((uint32_t)(1 << IOCON_DACEN_POS))

// I2C mode
#define PIN_I2CMODE_FAST_STANDARD  ((uint32_t)(0))
#define PIN_I2CMODE_OPENDRAINIO    ((uint32_t)(1))
#define PIN_I2CMODE_FASTMODEPLUS   ((uint32_t)(2))

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
extern int32_t PIN_Configure (uint8_t port, uint8_t pin, uint32_t pin_cfg);

/**
  \fn          PIN_ConfigureI2CPins (uint8_t port, uint8_t pin, uint32_t i2cMode)
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
extern int32_t PIN_ConfigureI2C0Pins (uint8_t port, uint8_t pin, uint32_t i2cMode);

#endif // __PIN_LPC177X_8X_H
