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
 * $Revision:    V2.4
 *
 * Project:      SSP Driver Definitions for NXP LPC17xx
 * -------------------------------------------------------------------------- */

#ifndef __SSP_LPC17XX_H
#define __SSP_LPC17XX_H

#if defined (LPC175x_6x)
  #include "LPC17xx.h"
  #include "PIN_LPC17xx.h"
#elif defined (LPC177x_8x)
  #include "LPC177x_8x.h"
  #include "PIN_LPC177x_8x.h"
#endif

#include "Driver_SPI.h"

#include "RTE_Device.h"
#include "RTE_Components.h"

/* SSP Register Interface Definitions */
#define CGU_BASE_SSPx_CLK_PD              (0x01U << 0)      /*!< CGU BASE_SSPx_CLK: PD Mask              */
#define CGU_BASE_SSPx_CLK_AUTOBLOCK       (0x01U << 11)     /*!< CGU BASE_SSPx_CLK: AUTOBLOCK Mask       */
#define CGU_BASE_SSPx_CLK_CLK_SEL         (0x1FU << 24)     /*!< CGU BASE_SSPx_CLK: CLK_SEL Mask         */
#define CCU1_CLK_M3_SSPx_CFG_RUN          (0x01U << 0)      /*!< CCU1 CLK_M3_SSPx_CFG: RUN Mask          */
#define CCU1_CLK_M3_SSPx_CFG_AUTO         (0x01U << 1)      /*!< CCU1 CLK_M3_SSPx_CFG: AUTO Mask         */
#define CCU1_CLK_M3_SSPx_CFG_WAKEUP       (0x01U << 2)      /*!< CCU1 CLK_M3_SSPx_CFG: WAKEUP Mask       */
#define CCU1_CLK_M3_SSPx_STAT_RUN         (0x01U << 0)      /*!< CCU1 CLK_M3_SSPx_STAT: RUN Mask         */
#define CCU1_CLK_M3_SSPx_STAT_AUTO        (0x01U << 1)      /*!< CCU1 CLK_M3_SSPx_STAT: AUTO Mask        */
#define CCU1_CLK_M3_SSPx_STAT_WAKEUP      (0x01U << 2)      /*!< CCU1 CLK_M3_SSPx_STAT: WAKEUP Mask      */
#define CCU2_BASE_STAT_BASE_SSP1_CLK      (0x01U << 5)      /*!< CCU2 BASE_STAT: BASE_SSP1_CLK Mask      */
#define CCU2_BASE_STAT_BASE_SSP0_CLK      (0x01U << 6)      /*!< CCU2 BASE_STAT: BASE_SSP0_CLK Mask      */
#define CCU2_CLK_APBn_SSPx_CFG_RUN        (0x01U << 0)      /*!< CCU2 CLK_APBn_SSPx_CFG: RUN Mask        */
#define CCU2_CLK_APBn_SSPx_CFG_AUTO       (0x01U << 1)      /*!< CCU2 CLK_APBn_SSPx_CFG: AUTO Mask       */
#define CCU2_CLK_APBn_SSPx_CFG_WAKEUP     (0x01U << 2)      /*!< CCU2 CLK_APBn_SSPx_CFG: WAKEUP Mask     */
#define CCU2_CLK_APBn_SSPx_STAT_RUN       (0x01U << 0)      /*!< CCU2 CLK_APBn_SSPx_STAT: RUN Mask       */
#define CCU2_CLK_APBn_SSPx_STAT_AUTO      (0x01U << 1)      /*!< CCU2 CLK_APBn_SSPx_STAT: AUTO Mask      */
#define CCU2_CLK_APBn_SSPx_STAT_WAKEUP    (0x01U << 2)      /*!< CCU2 CLK_APBn_SSPx_STAT: WAKEUP Mask    */
#define RGU_RESET_CTRL1_SSP0_RST          (0x01U << 18)     /*!< RGU RESET_CTRL1: SSP0_RST Mask          */
#define RGU_RESET_CTRL1_SSP1_RST          (0x01U << 19)     /*!< RGU RESET_CTRL1: SSP1_RST Mask          */
#define RGU_RESET_STATUS3_SSP0_RST        (0x03U << 4)      /*!< RGU RESET_STATUS3: SSP0_RST Mask        */
#define RGU_RESET_STATUS3_SSP1_RST        (0x03U << 6)      /*!< RGU RESET_STATUS3: SSP1_RST Mask        */
#define RGU_RESET_ACTIVE_STATUS1_SSP0_RST (0x01U << 18)     /*!< RGU RESET_ACTIVE_STATUS1: SSP0_RST Mask */
#define RGU_RESET_ACTIVE_STATUS1_SSP1_RST (0x01U << 19)     /*!< RGU RESET_ACTIVE_STATUS1: SSP1_RST Mask */

#define SSPx_CR0_DSS                      (0x0FU << 0)      /*!< SSPx CR0: DSS Mask                      */
#define SSPx_CR0_FRF                      (0x03U << 4)      /*!< SSPx CR0: FRF Mask                      */
#define SSPx_CR0_CPOL                     (0x01U << 6)      /*!< SSPx CR0: CPOL Mask                     */
#define SSPx_CR0_CPHA                     (0x01U << 7)      /*!< SSPx CR0: CPHA Mask                     */
#define SSPx_CR0_SCR                      (0xFFU << 8)      /*!< SSPx CR0: SCR Mask                      */
#define SSPx_CR1_LBM                      (0x01U << 0)      /*!< SSPx CR1: LBM Mask                      */
#define SSPx_CR1_SSE                      (0x01U << 1)      /*!< SSPx CR1: SSE Mask                      */
#define SSPx_CR1_MS                       (0x01U << 2)      /*!< SSPx CR1: MS Mask                       */
#define SSPx_CR1_SOD                      (0x01U << 3)      /*!< SSPx CR1: SOD Mask                      */
#define SSPx_DR_DATA                      (0xFFFFU<<0)      /*!< SSPx DR: DATA Mask                      */
#define SSPx_SR_TFE                       (0x01U << 0)      /*!< SSPx SR: TFE Mask                       */
#define SSPx_SR_TNF                       (0x01U << 1)      /*!< SSPx SR: TNF Mask                       */
#define SSPx_SR_RNE                       (0x01U << 2)      /*!< SSPx SR: RNE Mask                       */
#define SSPx_SR_RFF                       (0x01U << 3)      /*!< SSPx SR: RFF Mask                       */
#define SSPx_SR_BSY                       (0x01U << 4)      /*!< SSPx SR: BSY Mask                       */
#define SSPx_CPSR_CPSDVSR                 (0xFFU << 0)      /*!< SSPx CPSR: CPSDVSR Mask                 */
#define SSPx_IMSC_RORIM                   (0x01U << 0)      /*!< SSPx IMSC: RORIM Mask                   */
#define SSPx_IMSC_RTIM                    (0x01U << 1)      /*!< SSPx IMSC: RTIM Mask                    */
#define SSPx_IMSC_RXIM                    (0x01U << 2)      /*!< SSPx IMSC: RXIM Mask                    */
#define SSPx_IMSC_TXIM                    (0x01U << 3)      /*!< SSPx IMSC: TXIM Mask                    */
#define SSPx_RIS_RORRIS                   (0x01U << 0)      /*!< SSPx RIS: RORRIS Mask                   */
#define SSPx_RIS_RTRIS                    (0x01U << 1)      /*!< SSPx RIS: RTRIS Mask                    */
#define SSPx_RIS_RXRIS                    (0x01U << 2)      /*!< SSPx RIS: RXRIS Mask                    */
#define SSPx_RIS_TXRIS                    (0x01U << 3)      /*!< SSPx RIS: TXRIS Mask                    */
#define SSPx_MIS_RORMIS                   (0x01U << 0)      /*!< SSPx MIS: RORMIS Mask                   */
#define SSPx_MIS_RTMIS                    (0x01U << 1)      /*!< SSPx MIS: RTMIS Mask                    */
#define SSPx_MIS_RXMIS                    (0x01U << 2)      /*!< SSPx MIS: RXMIS Mask                    */
#define SSPx_MIS_TXMIS                    (0x01U << 3)      /*!< SSPx MIS: TXMIS Mask                    */
#define SSPx_ICR_RORIC                    (0x01U << 0)      /*!< SSPx ICR: RORIC Mask                    */
#define SSPx_ICR_RTIC                     (0x01U << 1)      /*!< SSPx ICR: RTIC Mask                     */
#define SSPx_DMACR_RXDMAE                 (0x01U << 0)      /*!< SSPx DMACR: RXDMAE Mask                 */
#define SSPx_DMACR_TXDMAE                 (0x01U << 1)      /*!< SSPx DMACR: TXDMAE Mask                 */

#define CLK_SRC_PLL1                       0x09U            // SSP clock source

/* Current driver status flag definition */
#define SSP_INITIALIZED                   (1U    << 0)      // SSP initialized
#define SSP_POWERED                       (1U    << 1)      // SSP powered on
#define SSP_CONFIGURED                    (1U    << 2)      // SSP configured
#define SSP_DATA_LOST                     (1U    << 3)      // SSP data lost occurred
#define SSP_MODE_FAULT                    (1U    << 4)      // SSP mode fault occurred

/* SSP Pins Configuration */
typedef const struct {
  const PIN            *ssel;
  const PIN            *sck;
  const PIN            *miso;
  const PIN            *mosi;
  uint8_t               ssel_func;      // SSEL pin alternate function
  uint8_t               sck_func;       // SCK pin alternate function
  uint8_t               miso_func;      // MISO pin alternate function
  uint8_t               mosi_func;      // MOSI pin alternate function
#if defined (LPC177x_8x)
  uint8_t               sck_io_wa;      // SCK pin io w/a type
  uint8_t               miso_io_wa;     // MISO pin io w/a type
  uint8_t               mosi_io_wa;     // MOSI pin io w/a type
  uint8_t               reserved;
#endif
} SSP_PINS;

/* Clocks Configuration */
typedef const struct {
  uint32_t              reg_pwr_val;    // SSP block power control register value
  volatile uint32_t    *reg_pwr;        // SSP block power control  register
#if defined (LPC175x_6x)
  uint32_t              peri_cfg_pos;   // SSP peripheral clock configuration position
  uint32_t              peri_cfg_val;   // SSP peripheral clock configuration value
  volatile uint32_t    *peri_cfg;       // SSP peripheral clock configuration register
#endif
} SSP_CLOCKS;


/* DMA Configuration */
typedef const struct {
  uint8_t               tx_en;          // Transmit channel enabled
  uint8_t               tx_ch;          // Transmit channel number
  uint8_t               tx_req;         // Transmit DMA request number
  uint8_t               reserved0;
  void                (*tx_callback)(uint32_t event); // Transmit callback
  uint8_t               rx_en;          // Receive channel enabled
  uint8_t               rx_ch;          // Receive channel number
  uint8_t               rx_req;         // Receive DMA request number
  uint8_t               reserved1;
  void                (*rx_callback)(uint32_t event); // Receive callback
} SSP_DMA;

/* SSP status */
typedef struct {
  uint8_t               busy;           // Transmitter/Receiver busy flag
  uint8_t               data_lost;      // Data lost: Receive overflow / Transmit underflow (cleared on start of transfer operation)
  uint8_t               mode_fault;     // Mode fault detected; optional (cleared on start of transfer operation)
  uint8_t               reserved;
} SSP_STATUS;

/* SSP Information (Run-time) */
typedef struct {
  ARM_SPI_SignalEvent_t cb_event;       // Event Callback
  SSP_STATUS            status;         // Status flags
  uint32_t              mode;           // Current SSP mode
  uint8_t               state;          // Current SSP state
  uint8_t               reserved[3];
} SSP_INFO;

/* SSP Transfer Information (Run-Time) */
typedef struct {
  uint32_t              num;            // Total number of transfers
  uint8_t              *rx_buf;         // Pointer to in data buffer
  uint8_t              *tx_buf;         // Pointer to out data buffer
  uint32_t              rx_cnt;         // Number of data received
  uint32_t              tx_cnt;         // Number of data sent
  uint32_t              dump_val;       // Variable for dumping DMA data
  uint16_t              def_val;        // Default transfer value
  uint8_t               reserved[2];    // Reserved
} SSP_TRANSFER_INFO;

/* SSP Resources */
typedef const struct {
  LPC_SSP_TypeDef      *reg;            // SSP peripheral register interface
  SSP_PINS              pin;            // SSP pins configuration
  SSP_CLOCKS            clk;            // SSP clocks configuration
  SSP_DMA               dma;            // SSP DMA configuration
  uint32_t              irq_num;        // SSP IRQ number
  SSP_INFO             *info;           // SSP Run-time information
  SSP_TRANSFER_INFO    *xfer;           // SSP transfer information
} SSP_RESOURCES;

// Function prototypes
void SSPx_GPDMA_Tx_SignalEvent (uint32_t event, SSP_RESOURCES *ssp);
void SSPx_GPDMA_Rx_SignalEvent (uint32_t event, SSP_RESOURCES *ssp);
void SSP0_GPDMA_Tx_SignalEvent (uint32_t event);
void SSP0_GPDMA_Rx_SignalEvent (uint32_t event);
void SSP1_GPDMA_Tx_SignalEvent (uint32_t event);
void SSP1_GPDMA_Rx_SignalEvent (uint32_t event);
void SSP0_IRQHandler (void);
void SSP1_IRQHandler (void);

// Global functions and variables exported by driver .c module */
#if defined(RTE_SPI)
extern ARM_DRIVER_SPI Driver_SPI2;
#endif
#if (RTE_SSP0)
extern ARM_DRIVER_SPI Driver_SPI0;
#endif
#if (RTE_SSP1)
extern ARM_DRIVER_SPI Driver_SPI1;
#endif

#endif /* __SSP_LPC17XX_H */
