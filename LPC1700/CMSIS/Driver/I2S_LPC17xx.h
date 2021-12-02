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
 * $Revision:    V1.4
 *
 * Project:      I2S Driver Definitions for NXP LPC17xx
 * -------------------------------------------------------------------------- */

#ifndef __I2S_LPC17XX_H
#define __I2S_LPC17XX_H

#if defined (LPC175x_6x)
  #include "LPC17xx.h"
#elif defined (LPC177x_8x)
  #include "LPC177x_8x.h"
#endif

#include "Driver_SAI.h"

#if defined (LPC175x_6x)
  #include "PIN_LPC17xx.h"
  #include "GPDMA_LPC17xx.h"
#elif defined (LPC177x_8x)
  #include "PIN_LPC177x_8x.h"
  #include "GPDMA_LPC17xx.h"
#endif

#include "RTE_Device.h"
#include "RTE_Components.h"

#if defined (LPC175x_6x)
#define DAO         I2SDAO
#define DAI         I2SDAI
#define TXFIFO      I2STXFIFO
#define RXFIFO      I2SRXFIFO
#define STATE       I2SSTATE
#define DMA1        I2SDMA1
#define DMA2        I2SDMA2
#define IRQ         I2SIRQ
#define TXRATE      I2STXRATE
#define RXRATE      I2SRXRATE
#define TXBITRATE   I2STXBITRATE
#define RXBITRATE   I2SRXBITRATE
#define TXMODE      I2STXMODE
#define RXMODE      I2SRXMODE
#endif

// Clock Control Unit register
#define CCU_CLK_CFG_RUN     (1U << 0)
#define CCU_CLK_CFG_AUTO    (1U << 1)
#define CCU_CLK_STAT_RUN    (1U << 0)

// I2S Register interface definitions
// I2S Digital audio output/input register
#define I2S_DAO_DAI_WORDWIDTH_POS       (          0U)
#define I2S_DAO_DAI_WORDWIDTH_MSK       (3U    << I2S_DAO_DAI_WORDWIDTH_POS)
#define I2S_DAO_DAI_MONO                (1U    <<   2)
#define I2S_DAO_DAI_STOP                (1U    <<   3)
#define I2S_DAO_DAI_RESET               (1U    <<   4)
#define I2S_DAO_DAI_WS_SEL              (1U    <<   5)
#define I2S_DAO_DAI_WS_HALFPERIOD_POS   (          6U)
#define I2S_DAO_DAI_WS_HALFPERIOD_MSK   (0x1FF << I2S_DAO_DAI_WS_HALFPERIOD_POS)
#define I2S_DAO_MUTE                    (1U    <<  15)

// I2S Transmit fifo register
#define I2S_TXFIFO_I2STXFIFO_MSK        (0xFFFFFFFFU)

// I2S Receive fifo register
#define I2S_RXFIFO_I2SrXFIFO_MSK        (0xFFFFFFFFU)

// I2S Status feedback register
#define I2S_STATE_IRQ                   (          1U)
#define I2S_STATE_DMAREQ1               (1U    <<   1)
#define I2S_STATE_DMAREQ2               (1U    <<   2)
#define I2S_STATE_RX_LEVEL_POS          (          8U)
#define I2S_STATE_RX_LEVEL_MSK          (0xFU  << I2S_STATE_RX_LEVEL_POS)
#define I2S_STATE_TX_LEVEL_POS          (         16U)
#define I2S_STATE_TX_LEVEL_MSK          (0xFU  << I2S_STATE_TX_LEVEL_POS)

// I2S DMA configuration register
#define I2S_DMA_RX_DMA_ENABLE           (          1U)
#define I2S_DMA_TX_DMA_ENABLE           (1U    <<   1)
#define I2S_DMA_RX_DEPTH_DMA_POS        (          8U)
#define I2S_DMA_RX_DEPTH_DMA_MSK        (0xFU  << I2S_DMA_RX_DEPTH_DMA_POS)
#define I2S_DMA_TX_DEPTH_DMA_POS        (         16U)
#define I2S_DMA_TX_DEPTH_DMA_MSK        (0xFU  << I2S_DMA_TX_DEPTH_DMA_POS)

// I2S Interrupt request control register
#define I2S_IRQ_RX_IRQ_ENABLE           (          1U)
#define I2S_IRQ_TX_IRQ_ENABLE           (1U    <<   1)
#define I2S_IRQ_RX_DEPTH_IRQ_POS        (          8U)
#define I2S_IRQ_RX_DEPTH_IRQ_MSK        (0xFU  << I2S_IRQ_RX_DEPTH_IRQ_POS)
#define I2S_IRQ_TX_DEPTH_IRQ_POS        (         16U)
#define I2S_IRQ_TX_DEPTH_IRQ_MSK        (0xFU  << I2S_IRQ_TX_DEPTH_IRQ_POS)

// I2S Transmit/Recevice clock rate register
#define I2S_TX_RX_RATE_Y_DIVIDER_POS    (          0U)
#define I2S_TX_RX_RATE_Y_DIVIDER_MSK    (0xFFU << I2S_TX_RX_RATE_Y_DIVIDER_POS)
#define I2S_TX_RX_RATE_X_DIVIDER_POS    (          8U)
#define I2S_TX_RX_RATE_X_DIVIDER_MSK    (0xFFU << I2S_TX_RX_RATE_X_DIVIDER_POS)

// I2S Transmit/Receive clock bit rate register
#define I2S_TX_RX_BITRATE_BITRATE_POS   (          0U)
#define I2S_TX_RX_BITRATE_BITRATE_MSK   (0x3FU << I2S_TX_RX_BITRATE_BITRATE_POS)

// I2S Transmit/Receive mode control register
#define I2S_TX_RX_MODE_CLKSEL_POS       (          0U)
#define I2S_TX_RX_MODE_CLKSEL_MSK       (3U    << I2S_TX_RX_MODE_CLKSEL_POS)
#define I2S_TX_RX_MODE_4PIN             (1U    <<  2U)
#define I2S_TX_RX_MODE_MCENA            (1U    <<  3U)

// I2S flags
#define I2S_FLAG_INITIALIZED            (     1U)
#define I2S_FLAG_POWERED                (1U << 1)
#define I2S_FLAG_CONFIGURED             (1U << 2)

// I2S Stream Information (Run-Time)
typedef struct {
  uint32_t                num;           // Total number of data to be transmited/received
  uint8_t                *buf;           // Pointer to data buffer
  uint32_t                cnt;           // Number of data transmited/receive
  uint8_t                 data_bits;     // Number of data bits
  uint8_t                 master;        // Master flag
  uint8_t                 residue_num;
  uint8_t                 residue_cnt;
  uint8_t                 residue_buf[4];
} I2S_STREAM_INFO;

typedef struct {
  uint8_t tx_busy;                       // Transmitter busy flag
  uint8_t rx_busy;                       // Receiver busy flag
  uint8_t tx_underflow;                  // Transmit data underflow detected (cleared on start of next send operation)
  uint8_t rx_overflow;                   // Receive data overflow detected (cleared on start of next receive operation)
  uint8_t frame_error;                   // Sync Frame error detected (cleared on start of next send/receive operation)
  uint8_t reserved[3];
} I2S_STATUS;

// I2S Information (Run-Time)
typedef struct {
  ARM_SAI_SignalEvent_t   cb_event;      // Event callback
  I2S_STATUS              status;        // Status flags
  I2S_STREAM_INFO         tx;            // Transmit information
  I2S_STREAM_INFO         rx;            // Receive information
  uint8_t                 flags;         // I2S driver flags
  uint8_t                 reserved[3];
} I2S_INFO;

// I2S DMA
typedef const struct {
  GPDMA_SignalEvent_t     cb_event;      // DMA Event callback
  uint8_t                 channel;       // DMA Channel number
  uint8_t                 request;       // DMA Request number
  uint8_t                 reserved[2];
} I2S_DMA;

// I2S Pin Configuration
typedef const struct {
  const PIN              *sck;           // Clock pin identifier
  uint8_t                 func_sck;      // Clock pin function
  uint8_t                 sck_io_wa;     // Clock pin io w/a type
  uint8_t                 reserved0[2];  // Reserved
  const PIN              *ws;            // Word select pin identifier
  uint8_t                 func_ws;       // Word select pin function
  uint8_t                 ws_io_wa;      // Word select pin io w/a type
  uint8_t                 reserved1[2];  // Reserved
  const PIN              *sda;           // Data pin identifier
  uint8_t                 func_sda;      // Data pin function
  uint8_t                 sda_io_wa;     // Data pin io w/a type
  uint8_t                 reserved2[2];  // Reserved
  const PIN              *mclk;          // Master clock pin identifier
  uint8_t                 func_mclk;     // Master clock pin function
  uint8_t                 mclk_io_wa;    // Master clock pin io w/a type
  uint8_t                 reserved3[2];  // Reserved
} I2S_PINS;

// I2S Reseurces definitions
typedef const struct {
  ARM_SAI_CAPABILITIES    capabilities;  // Capabilities
  LPC_I2S_TypeDef        *reg;           // Pointer to I2S peripheral
  I2S_PINS                rx_pins;       // I2S receive pins configuration
  I2S_PINS                tx_pins;       // I2S transmit pins configuration
  int32_t                 irq_num;       // I2S IRQ Number
  I2S_DMA                *dma_tx;        // I2S TX DMA configuration
  I2S_DMA                *dma_rx;        // I2S RX DMA configuration
  uint8_t                 tx_fifo_level; // I2S transmit fifo level
  uint8_t                 rx_fifo_level; // I2S receive fifo level
  uint8_t                 reserved[2];
  I2S_INFO               *info;          // Run-Time information
} I2S_RESOURCES;

// Global functions and variables exported by driver .c module */
#if (RTE_I2S0)
extern ARM_DRIVER_SAI Driver_SAI0;
#endif

#endif // __I2S_LPC17XX_H
