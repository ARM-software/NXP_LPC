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
 * $Date:        01. August 2019
 * $Revision:    V2.2
 *
 * Project:      SPI Driver Definitions for NXP LPC17xx
 * -------------------------------------------------------------------------- */

#ifndef __SPI_LPC17XX_H
#define __SPI_LPC17XX_H

#if defined (LPC175x_6x)
  #include "LPC17xx.h"
  #include "PIN_LPC17xx.h"
#else
#error "SPI peripheral not available for selected Device."
#endif

#include "Driver_SPI.h"

#include "RTE_Device.h"
#include "RTE_Components.h"

/* SPI Register Interface Definitions */
#define CGU_BASE_SPI_CLK_PD               (0x01U << 0)      /*!< CGU BASE_SPI_CLK: PD Mask               */
#define CGU_BASE_SPI_CLK_AUTOBLOCK        (0x01U << 11)     /*!< CGU BASE_SPI_CLK: AUTOBLOCK Mask        */
#define CGU_BASE_SPI_CLK_CLK_SEL          (0x1FU << 24)     /*!< CGU BASE_SPI_CLK: CLK_SEL Mask          */
#define CCU1_CLK_SPI_CFG_RUN              (0x01U << 0)      /*!< CCU1 CLK SPI CFG: RUN Mask              */
#define CCU1_CLK_SPI_CFG_AUTO             (0x01U << 1)      /*!< CCU1 CLK SPI CFG: AUTO Mask             */
#define CCU1_CLK_SPI_CFG_WAKEUP           (0x01U << 2)      /*!< CCU1 CLK SPI CFG: WAKEUP Mask           */
#define CCU1_CLK_SPI_STAT_RUN             (0x01U << 0)      /*!< CCU1 CLK SPI STAT: RUN Mask             */
#define CCU1_CLK_SPI_STAT_AUTO            (0x01U << 1)      /*!< CCU1 CLK SPI STAT: AUTO Mask            */
#define CCU1_CLK_SPI_STAT_WAKEUP          (0x01U << 2)      /*!< CCU1 CLK SPI STAT: WAKEUP Mask          */
#define RGU_RESET_CTRL1_SPI_RST           (0x01U << 26)     /*!< RGU RESET_CTRL1: SPI_RST Mask           */
#define RGU_RESET_STATUS3_SPI_RST         (0x03U << 20)     /*!< RGU RESET_STATUS3: SPI_RST Mask         */
#define RGU_RESET_ACTIVE_STATUS1_SPI_RST  (0x01U << 26)     /*!< RGU RESET_ACTIVE_STATUS1: SPI_RST Mask  */

#define SPI_CR_BITENABLE                  (0x01U << 2)      /*!< SPI CR: BITENABLE Mask                  */
#define SPI_CR_CPHA                       (0x01U << 3)      /*!< SPI CR: CPHA Mask                       */
#define SPI_CR_CPOL                       (0x01U << 4)      /*!< SPI CR: CPOL Mask                       */
#define SPI_CR_MSTR                       (0x01U << 5)      /*!< SPI CR: MSTR Mask                       */
#define SPI_CR_LSBF                       (0x01U << 6)      /*!< SPI CR: LSBF Mask                       */
#define SPI_CR_SPIE                       (0x01U << 7)      /*!< SPI CR: SPIE Mask                       */
#define SPI_CR_BITS                       (0x0fU << 8)      /*!< SPI CR: BITS Mask                       */
#define SPI_SR_ABRT                       (0x01U << 3)      /*!< SPI SR: ABRT Mask                       */
#define SPI_SR_MODF                       (0x01U << 4)      /*!< SPI SR: MODF Mask                       */
#define SPI_SR_ROVR                       (0x01U << 5)      /*!< SPI SR: ROVR Mask                       */
#define SPI_SR_WCOL                       (0x01U << 6)      /*!< SPI SR: WCOL Mask                       */
#define SPI_SR_SPIF                       (0x01U << 7)      /*!< SPI SR: SPIF Mask                       */
#define SPI_DR_DATALOW                    (0xFFU << 0)      /*!< SPI DR: DATALOW Mask                    */
#define SPI_DR_DATAHIGH                   (0xFFU << 8)      /*!< SPI DR: DATAHIGH Mask                   */
#define SPI_CCR_COUNTER                   (0xFFU << 0)      /*!< SPI CCR: COUNTER Mask                   */
#define SPI_TCR_TEST                      (0x7FU << 1)      /*!< SPI TCR: TEST Mask                      */
#define SPI_TSR_ABRT                      (0x01U << 3)      /*!< SPI TSR: ABRT Mask                      */
#define SPI_TSR_MODF                      (0x01U << 4)      /*!< SPI TSR: MODF Mask                      */
#define SPI_TSR_ROVR                      (0x01U << 5)      /*!< SPI TSR: ROVR Mask                      */
#define SPI_TSR_WCOL                      (0x01U << 6)      /*!< SPI TSR: WCOL Mask                      */
#define SPI_TSR_SPIF                      (0x01U << 7)      /*!< SPI TSR: SPIF Mask                      */
#define SPI_INT_SPIF                      (0x01U << 0)      /*!< SPI INT: SPIF Mask                      */

// Peripheral clock divider definitions
#define PCLKSEL_CCLK_DIV_1   (1U)
#define PCLKSEL_CCLK_DIV_2   (2U)
#define PCLKSEL_CCLK_DIV_4   (0U)
#define PCLKSEL_CCLK_DIV_8   (3U)

// SSP peripheral clock selection bit position definitions
#define SPI_PCLKSEL0_POS     (16U)

/* Current driver status flag definition */
#define SPI_INITIALIZED                   (1U    << 0)      // SPI initialized
#define SPI_POWERED                       (1U    << 1)      // SPI powered on
#define SPI_CONFIGURED                    (1U    << 2)      // SPI configured
#define SPI_DATA_LOST                     (1U    << 3)      // SPI data lost occurred
#define SPI_MODE_FAULT                    (1U    << 4)      // SPI mode fault occurred

/* SPI Pins Configuration */
typedef const struct {
  const PIN            *ssel;           // SSEL pin
  const PIN            *sck;            // SCK pin
  const PIN            *miso;           // MISO pin
  const PIN            *mosi;           // MOSI pin
  uint8_t               ssel_func;      // SSEL pin alternate function
  uint8_t               sck_func;       // SCK pin alternate function
  uint8_t               miso_func;      // MISO pin alternate function
  uint8_t               mosi_func;      // MOSI pin alternate function
} SPI_PINS;

/* Clocks Configuration */
typedef const struct {
  uint32_t              reg_pwr_val;    // SPI block power control register value
  volatile uint32_t    *reg_pwr;        // SPI block power control  register
  uint32_t              peri_cfg_pos;   // SPI peripheral clock configuration position
  uint32_t              peri_cfg_val;   // SPI peripheral clock configuration value
  volatile uint32_t    *peri_cfg;       // SPI peripheral clock configuration register
} SPI_CLOCKS;

/* SPI status */
typedef struct _SPI_STATUS {
  uint8_t               busy;           // Transmitter/Receiver busy flag
  uint8_t               data_lost;      // Data lost: Receive overflow / Transmit underflow (cleared on start of transfer operation)
  uint8_t               mode_fault;     // Mode fault detected; optional (cleared on start of transfer operation)
  uint8_t               reserved;
} SPI_STATUS;

/* SPI Information (Run-time) */
typedef struct {
  ARM_SPI_SignalEvent_t cb_event;       // Event Callback
  SPI_STATUS            status;         // Status flags
  uint32_t              mode;           // Current SPI mode
  uint8_t               state;          // Current SPI state
  uint8_t               reserved[3];
} SPI_INFO;

/* SPI Transfer Information (Run-Time) */
typedef struct {
  uint32_t              num;            // Total number of transfers
  uint8_t              *rx_buf;         // Pointer to in data buffer
  uint8_t              *tx_buf;         // Pointer to out data buffer
  uint32_t              rx_cnt;         // Number of data received
  uint32_t              tx_cnt;         // Number of data sent
  uint32_t              dump_val;       // Variable for dumping DMA data
  uint16_t              def_val;        // Default transfer value
  uint8_t               reserved[2];
} SPI_TRANSFER_INFO;

/* SPI Resources */
typedef struct {
  LPC_SPI_TypeDef      *reg;            // SPI peripheral register interface
  SPI_PINS              pin;            // SPI pins configuration
  SPI_CLOCKS            clk;            // SPI clocks configuration
  int32_t               irq_num;        // SPI IRQ number
  SPI_INFO             *info;           // SPI Run-time information
  SPI_TRANSFER_INFO    *xfer;           // SPI transfer information
} const SPI_RESOURCES;

// Global functions and variables exported by driver .c module */
#if (RTE_SPI)
extern ARM_DRIVER_SPI Driver_SPI2;
#endif

#endif /* __SPI_LPC17XX_H */
