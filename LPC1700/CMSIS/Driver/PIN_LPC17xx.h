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
 * $Date:        13. June 2014
 * $Revision:    V1.0
 *
 * Project:      PIN Driver Definitions for NXP LPC17xx
 * -------------------------------------------------------------------------- */

#ifndef __PIN_LPC17XX_H
#define __PIN_LPC17XX_H

#include <stdint.h>
#include <stdbool.h>

// Pin Configuration

typedef struct _PIN
{
  uint8_t Portnum;   // Port Number
  uint8_t Pinnum;    // Pin Number
} PIN;


//------------------------------------------------------------------------------
// PINSEL REGISTER BIT DEFINITIONS
//------------------------------------------------------------------------------

// Pin Function selection
#define PIN_FUNC_0              ((uint32_t)(0))
#define PIN_FUNC_1              ((uint32_t)(1))
#define PIN_FUNC_2              ((uint32_t)(2))
#define PIN_FUNC_3              ((uint32_t)(3))

// Pin mode
#define PIN_PINMODE_PULLUP      ((uint32_t)(0))
#define PIN_PINMODE_REPEATER    ((uint32_t)(1))
#define PIN_PINMODE_TRISTATE    ((uint32_t)(2))
#define PIN_PINMODE_PULLDOWN    ((uint32_t)(3))

// Pin mode (normal/open drain)
#define	PIN_PINMODE_NORMAL      ((uint32_t)(0))
#define	PIN_PINMODE_OPENDRAIN   ((uint32_t)(1))

// I2C mode
#define PIN_I2C_Normal_Mode     ((uint32_t)(0))
#define PIN_I2C_Fast_Mode       ((uint32_t)(1))
#define PIN_I2C_Fast_Plus_Mode  ((uint32_t)(2))

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
extern int32_t PIN_Configure (uint8_t port, uint8_t pin, uint8_t function, uint8_t mode, uint8_t open_drain);

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
extern int32_t PIN_ConfigureTPIU (bool enable);

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
extern int32_t PIN_ConfigureI2C0Pins (uint8_t i2cPinMode, bool enableFilterSlewRate);

#endif // __PIN_LPC17XX_H
