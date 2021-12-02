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
 * Project:      GPIO Driver for LPC17xx
 * -------------------------------------------------------------------------- */

#include "GPIO_LPC17xx.h"

#if defined (LPC175x_6x)
  #include "LPC17xx.h"
#elif defined (LPC177x_8x)
  #include "LPC177x_8x.h"
#endif

#define LPC_GPIO(n)             ((LPC_GPIO_TypeDef *)(LPC_GPIO0_BASE + 0x00020*n))

#if defined (LPC175x_6x)
#define DIR  FIODIR
#define SET  FIOSET
#define CLR  FIOCLR
#define PIN  FIOPIN
#define MASK FIOMASK
#endif

/**
  \fn          void GPIO_PortClock (uint32_t clock)
  \brief       Port Clock Control
  \param[in]   clock  Enable or disable clock
*/
void GPIO_PortClock (uint32_t clock) {

  if (clock) {
    LPC_SC->PCONP |=  (1U << 15);
  }
  else {
    LPC_SC->PCONP &= ~(1U << 15);
  }
}

/**
  \fn          void GPIO_SetDir (uint32_t port_num,
                                 uint32_t pin_num,
                                 uint32_t dir)
  \brief       Configure GPIO pin direction
  \param[in]   port_num   GPIO number (0..4)
  \param[in]   pin_num    Port pin number
  \param[in]   dir        GPIO_DIR_INPUT, GPIO_DIR_OUTPUT
*/
void GPIO_SetDir (uint32_t port_num, uint32_t pin_num, uint32_t dir) {

  dir  ? (LPC_GPIO(port_num)->DIR |=  (1UL << pin_num)) : \
         (LPC_GPIO(port_num)->DIR &= ~(1UL << pin_num));
}

/**
  \fn          void GPIO_PinWrite (uint32_t port_num,
                                   uint32_t pin_num,
                                   uint32_t val);
  \brief       Write port pin
  \param[in]   port_num   GPIO number (0..4)
  \param[in]   pin_num    Port pin number
  \param[in]   val        Port pin value (0 or 1)
*/
void GPIO_PinWrite (uint32_t port_num, uint32_t pin_num, uint32_t val) {

  val ? (LPC_GPIO(port_num)->SET = (1UL << pin_num)) : \
        (LPC_GPIO(port_num)->CLR = (1UL << pin_num));
}

/**
  \fn          uint32_t  GPIO_PinRead (uint32_t port_num, uint32_t pin_num,)
  \brief       Read port pin
  \param[in]   port_num   GPIO number (0..4)
  \param[in]   pin_num    Port pin number
  \return      pin value (0 or 1)
*/
uint32_t GPIO_PinRead (uint32_t port_num, uint32_t pin_num) {
  return ((LPC_GPIO(port_num)->PIN & (1UL << pin_num)) ? (1U) : (0U));
}

/**
  \fn          void GPIO_PortWrite (uint32_t port_num,
                                    uint32_t mask,
                                    uint32_t val)
  \brief       Write port pins
  \param[in]   port_num   GPIO number (0..4)
  \param[in]   mask       Selected pins
  \param[in]   val        Pin values
*/
void GPIO_PortWrite (uint32_t port_num, uint32_t mask, uint32_t val) {
  LPC_GPIO(port_num)->MASK = ~mask;
  LPC_GPIO(port_num)->PIN  =  val;
}

/**
  \fn          uint32_t  GPIO_PortRead (uint32_t port_num)
  \brief       Read port pins
  \param[in]   port_num   GPIO number (0..4)
  \return      port pin inputs
*/
uint32_t GPIO_PortRead (uint32_t port_num) {
  return (LPC_GPIO(port_num)->PIN);
}
