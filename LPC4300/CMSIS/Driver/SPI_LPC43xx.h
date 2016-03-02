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
 * $Revision:    V2.2
 *
 * Project:      SPI Driver Definitions for NXP LPC43xx
 * -------------------------------------------------------------------------- */

#ifndef __SPI_LPC43XX_H
#define __SPI_LPC43XX_H

#include "LPC43xx.h"
#include "SCU_LPC43xx.h"
#include "GPIO_LPC43xx.h"
#include "GPDMA_LPC43xx.h"

#include "Driver_SPI.h"

/* SPI Register Interface Definitions */
#define CGU_BASE_SPI_CLK_PD               (0x01 << 0)       /*!< CGU BASE_SPI_CLK: PD Mask               */
#define CGU_BASE_SPI_CLK_AUTOBLOCK        (0x01 << 11)      /*!< CGU BASE_SPI_CLK: AUTOBLOCK Mask        */
#define CGU_BASE_SPI_CLK_CLK_SEL          (0x1F << 24)      /*!< CGU BASE_SPI_CLK: CLK_SEL Mask          */
#define CCU1_CLK_SPI_CFG_RUN              (0x01 << 0)       /*!< CCU1 CLK SPI CFG: RUN Mask              */
#define CCU1_CLK_SPI_CFG_AUTO             (0x01 << 1)       /*!< CCU1 CLK SPI CFG: AUTO Mask             */
#define CCU1_CLK_SPI_CFG_WAKEUP           (0x01 << 2)       /*!< CCU1 CLK SPI CFG: WAKEUP Mask           */
#define CCU1_CLK_SPI_STAT_RUN             (0x01 << 0)       /*!< CCU1 CLK SPI STAT: RUN Mask             */
#define CCU1_CLK_SPI_STAT_AUTO            (0x01 << 1)       /*!< CCU1 CLK SPI STAT: AUTO Mask            */
#define CCU1_CLK_SPI_STAT_WAKEUP          (0x01 << 2)       /*!< CCU1 CLK SPI STAT: WAKEUP Mask          */
#define RGU_RESET_CTRL1_SPI_RST           (0x01 << 26)      /*!< RGU RESET_CTRL1: SPI_RST Mask           */
#define RGU_RESET_STATUS3_SPI_RST         (0x03 << 20)      /*!< RGU RESET_STATUS3: SPI_RST Mask         */
#define RGU_RESET_ACTIVE_STATUS1_SPI_RST  (0x01 << 26)      /*!< RGU RESET_ACTIVE_STATUS1: SPI_RST Mask  */

#define SPI_CR_BITENABLE                  (0x01 << 2)       /*!< SPI CR: BITENABLE Mask                  */
#define SPI_CR_CPHA                       (0x01 << 3)       /*!< SPI CR: CPHA Mask                       */
#define SPI_CR_CPOL                       (0x01 << 4)       /*!< SPI CR: CPOL Mask                       */
#define SPI_CR_MSTR                       (0x01 << 5)       /*!< SPI CR: MSTR Mask                       */
#define SPI_CR_LSBF                       (0x01 << 6)       /*!< SPI CR: LSBF Mask                       */
#define SPI_CR_SPIE                       (0x01 << 7)       /*!< SPI CR: SPIE Mask                       */
#define SPI_CR_BITS                       (0x0f << 8)       /*!< SPI CR: BITS Mask                       */
#define SPI_SR_ABRT                       (0x01 << 3)       /*!< SPI SR: ABRT Mask                       */
#define SPI_SR_MODF                       (0x01 << 4)       /*!< SPI SR: MODF Mask                       */
#define SPI_SR_ROVR                       (0x01 << 5)       /*!< SPI SR: ROVR Mask                       */
#define SPI_SR_WCOL                       (0x01 << 6)       /*!< SPI SR: WCOL Mask                       */
#define SPI_SR_SPIF                       (0x01 << 7)       /*!< SPI SR: SPIF Mask                       */
#define SPI_DR_DATALOW                    (0xFF << 0)       /*!< SPI DR: DATALOW Mask                    */
#define SPI_DR_DATAHIGH                   (0xFF << 8)       /*!< SPI DR: DATAHIGH Mask                   */
#define SPI_CCR_COUNTER                   (0xFF << 0)       /*!< SPI CCR: COUNTER Mask                   */
#define SPI_TCR_TEST                      (0x7F << 1)       /*!< SPI TCR: TEST Mask                      */
#define SPI_TSR_ABRT                      (0x01 << 3)       /*!< SPI TSR: ABRT Mask                      */
#define SPI_TSR_MODF                      (0x01 << 4)       /*!< SPI TSR: MODF Mask                      */
#define SPI_TSR_ROVR                      (0x01 << 5)       /*!< SPI TSR: ROVR Mask                      */
#define SPI_TSR_WCOL                      (0x01 << 6)       /*!< SPI TSR: WCOL Mask                      */
#define SPI_TSR_SPIF                      (0x01 << 7)       /*!< SPI TSR: SPIF Mask                      */
#define SPI_INT_SPIF                      (0x01 << 0)       /*!< SPI INT: SPIF Mask                      */

#define CLK_SRC_PLL1                       0x09             // SPI clock source

/* Current driver status flag definition */
#define SPI_INITIALIZED                   (1    << 0)       // SPI initialized
#define SPI_POWERED                       (1    << 1)       // SPI powered on
#define SPI_CONFIGURED                    (1    << 2)       // SPI configured
#define SPI_DATA_LOST                     (1    << 3)       // SPI data lost occurred
#define SPI_MODE_FAULT                    (1    << 4)       // SPI mode fault occurred

/* SPI Pins Configuration */
typedef const struct _SPI_PINS {
  PIN_ID              *sck;              // SCK pin
  PIN_ID              *miso;             // MISO pin
  PIN_ID              *mosi;             // MOSI pin
  PIN_ID              *ssel;             // SSEL pin
  GPIO_ID             *gpio_ssel;        // SSEL gpio
  uint8_t              gpio_ssel_af;     // SSEL gpio alternate function
} SPI_PINS;

/* SPI status */
typedef struct _SPI_STATUS {
  uint8_t busy;                         // Transmitter/Receiver busy flag
  uint8_t data_lost;                    // Data lost: Receive overflow / Transmit underflow (cleared on start of transfer operation)
  uint8_t mode_fault;                   // Mode fault detected; optional (cleared on start of transfer operation)
} SPI_STATUS;

/* SPI Information (Run-time) */
typedef struct _SPI_INFO {
  ARM_SPI_SignalEvent_t cb_event;       // Event Callback
  SPI_STATUS            status;         // Status flags
  uint8_t               state;          // Current SPI state
  uint32_t              mode;           // Current SPI mode
} SPI_INFO;

/* SPI Transfer Information (Run-Time) */
typedef struct _SPI_TRANSFER_INFO {
  uint32_t              num;            // Total number of transfers
  uint8_t              *rx_buf;         // Pointer to in data buffer
  uint8_t              *tx_buf;         // Pointer to out data buffer
  uint32_t              rx_cnt;         // Number of data received
  uint32_t              tx_cnt;         // Number of data sent
  uint32_t              dump_val;       // Variable for dumping DMA data
  uint16_t              def_val;        // Default transfer value
} SPI_TRANSFER_INFO;

/* SPI Resources */
typedef struct {
  LPC_SPI_Type         *reg;            // SPI peripheral register interface
  SPI_PINS              pin;            // SPI pins configuration
  IRQn_Type             irq_num;        // SPI IRQ number
  SPI_INFO             *info;           // SPI Run-time information
  SPI_TRANSFER_INFO    *xfer;           // SPI transfer information
} const SPI_RESOURCES;

#endif /* __SPI_LPC43XX_H */
