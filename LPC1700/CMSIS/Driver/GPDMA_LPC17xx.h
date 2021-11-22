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
 * Project:      GPDMA Driver Definitions for NXP LPC17xx
 * -------------------------------------------------------------------------- */

#ifndef __GPDMA_LPC17XX_H
#define __GPDMA_LPC17XX_H

#if defined (LPC175x_6x)
  #include "LPC17xx.h"
#elif defined (LPC177x_8x)
  #include "LPC177x_8x.h"
#endif

// Number of GPDMA channels
#define GPDMA_NUMBER_OF_CHANNELS           ((uint8_t) 8)

// GPDMA Events
#define GPDMA_EVENT_TERMINAL_COUNT_REQUEST (1U)
#define GPDMA_EVENT_ERROR                  (2U)

// GPDMA Burst size in Source and Destination definitions
#define GPDMA_BSIZE_1                      (0U) // Burst size = 1
#define GPDMA_BSIZE_4                      (1U) // Burst size = 4
#define GPDMA_BSIZE_8                      (2U) // Burst size = 8
#define GPDMA_BSIZE_16                     (3U) // Burst size = 16
#define GPDMA_BSIZE_32                     (4U) // Burst size = 32
#define GPDMA_BSIZE_64                     (5U) // Burst size = 64
#define GPDMA_BSIZE_128                    (6U) // Burst size = 128
#define GPDMA_BSIZE_256                    (7U) // Burst size = 256

// Width in Source transfer width and Destination transfer width definitions
#define GPDMA_WIDTH_BYTE                   (0U) // Width = 1 byte
#define GPDMA_WIDTH_HALFWORD               (1U) // Width = 2 bytes
#define GPDMA_WIDTH_WORD                   (2U) // Width = 4 bytes

// GPDMA Transfer type and flow control
#define GPDMA_TRANSFER_M2M_CTRL_DMA        (0U) // Memory to memory - DMA control
#define GPDMA_TRANSFER_M2P_CTRL_DMA        (1U) // Memory to peripheral - DMA control
#define GPDMA_TRANSFER_P2M_CTRL_DMA        (2U) // Peripheral to memory - DMA control
#define GPDMA_TRANSFER_P2P_CTRL_DMA        (3U) // Source peripheral to destination peripheral - DMA control
#define GPDMA_TRANSFER_P2P_CTRL_DST_PER    (4U) // Source peripheral to destination peripheral - Destination peripheral control
#define GPDMA_TRANSFER_M2P_CTRL_DST_PER    (5U) // Memory to peripheral - Destination peripheral control
#define GPDMA_TRANSFER_P2M_CTRL_SRC_PER    (6U) // Peripheral to memory - Source peripheral control
#define GPDMA_TRANSFER_P2P_CTRL_SRC_PER    (7U) // Source peripheral to destination peripheral - Source peripheral control

//  GPDMA Configuration register definitions
#define GPDMA_CONFIG_E                     (1U <<  0)
#define GPDMA_CONFIG_M0                    (1U <<  1)
#define GPDMA_CONFIG_M1                    (1U <<  2)

// GPDMA Channel Configuration registers definitions
#define GPDMA_CH_CONFIG_E                  (1U <<  0)
#define GPDMA_CH_CONFIG_SRC_PERI_POS       (      1U)
#define GPDMA_CH_CONFIG_SRC_PERI_MSK       (0x1FU          << GPDMA_CH_CONFIG_SRC_PERI_POS)
#define GPDMA_CH_CONFIG_SRC_PERI(n)        ((((uint32_t)n) << GPDMA_CH_CONFIG_SRC_PERI_POS) & GPDMA_CH_CONFIG_SRC_PERI_MSK)
#define GPDMA_CH_CONFIG_DEST_PERI_POS      (      6U)
#define GPDMA_CH_CONFIG_DEST_PERI_MSK      (0x1F           << GPDMA_CH_CONFIG_DEST_PERI_POS)
#define GPDMA_CH_CONFIG_DEST_PERI(n)       (((n)           << GPDMA_CH_CONFIG_DEST_PERI_POS) & GPDMA_CH_CONFIG_DEST_PERI_MSK)
#define GPDMA_CH_CONFIG_FLOWCNTRL_POS      (     11U)
#define GPDMA_CH_CONFIG_FLOWCNTRL_MSK      (0x07U          << GPDMA_CH_CONFIG_FLOWCNTRL_POS)
#define GPDMA_CH_CONFIG_FLOWCNTRL(n)       (((n)           << GPDMA_CH_CONFIG_FLOWCNTRL_POS) & GPDMA_CH_CONFIG_FLOWCNTRL_MSK)
#define GPDMA_CH_CONFIG_IE                 (1U << 14)
#define GPDMA_CH_CONFIG_ITC                (1U << 15)
#define GPDMA_CH_CONFIG_L                  (1U << 16)
#define GPDMA_CH_CONFIG_A                  (1U << 17)
#define GPDMA_CH_CONFIG_H                  (1U << 18)

// GPDMA Channel Control register definition
#define GPDMA_CH_CONTROL_TRANSFERSIZE_POS  (         0U)
#define GPDMA_CH_CONTROL_TRANSFERSIZE_MSK  (0xFFFU << GPDMA_CH_CONTROL_TRANSFERSIZE_POS)
#define GPDMA_CH_CONTROL_TRANSFERSIZE(n)   (((n)  << GPDMA_CH_CONTROL_TRANSFERSIZE_POS) & GPDMA_CH_CONTROL_TRANSFERSIZE_MSK)
#define GPDMA_CH_CONTROL_SBSIZE_POS        (        12U)
#define GPDMA_CH_CONTROL_SBSIZE_MSK        (0x03U << GPDMA_CH_CONTROL_SBSIZE_POS)
#define GPDMA_CH_CONTROL_SBSIZE(n)         (((n)  << GPDMA_CH_CONTROL_SBSIZE_POS) & GPDMA_CH_CONTROL_SBSIZE_MSK)
#define GPDMA_CH_CONTROL_DBSIZE_POS        (        15U)
#define GPDMA_CH_CONTROL_DBSIZE_MSK        (0x03U << GPDMA_CH_CONTROL_DBSIZE_POS)
#define GPDMA_CH_CONTROL_DBSIZE(n)         (((n)  << GPDMA_CH_CONTROL_DBSIZE_POS) & GPDMA_CH_CONTROL_DBSIZE_MSK)
#define GPDMA_CH_CONTROL_SWIDTH_POS        (        18U)
#define GPDMA_CH_CONTROL_SWIDTH_MSK        (0x03U << GPDMA_CH_CONTROL_SWIDTH_POS)
#define GPDMA_CH_CONTROL_SWIDTH(n)         (((n)  << GPDMA_CH_CONTROL_SWIDTH_POS) & GPDMA_CH_CONTROL_SWIDTH_MSK)
#define GPDMA_CH_CONTROL_DWIDTH_POS        (        21U)
#define GPDMA_CH_CONTROL_DWIDTH_MSK        (0x03U << GPDMA_CH_CONTROL_DWIDTH_POS)
#define GPDMA_CH_CONTROL_DWIDTH(n)         (((n)  << GPDMA_CH_CONTROL_DWIDTH_POS) & GPDMA_CH_CONTROL_DWIDTH_MSK)
#define GPDMA_CH_CONTROL_SI                (1U    << 26)
#define GPDMA_CH_CONTROL_DI                (1U    << 27)
#define GPDMA_CH_CONTROL_PROT1             (1U    << 28)
#define GPDMA_CH_CONTROL_PROT2             (1U    << 29)
#define GPDMA_CH_CONTROL_PROT3             (1U    << 30)
#define GPDMA_CH_CONTROL_I                 (1UL   << 31)

// GPDMA Connection number definitions
#if defined (LPC175x_6x)
  #define GPDMA_CONN_SSP0_Tx               ((0UL))     // SSP0 Tx
  #define GPDMA_CONN_SSP0_Rx               ((1UL))     // SSP0 Rx
  #define GPDMA_CONN_SSP1_Tx               ((2UL))     // SSP1 Tx
  #define GPDMA_CONN_SSP1_Rx               ((3UL))     // SSP1 Rx
  #define GPDMA_CONN_ADC                   ((4UL))     // ADC
  #define GPDMA_CONN_I2S_Channel_0         ((5UL))     // I2S channel 0
  #define GPDMA_CONN_I2S_Channel_1         ((6UL))     // I2S channel 1
  #define GPDMA_CONN_DAC                   ((7UL))     // DAC
  #define GPDMA_CONN_UART0_Tx              ((8UL))     // UART0 Tx
  #define GPDMA_CONN_UART0_Rx              ((9UL))     // UART0 Rx
  #define GPDMA_CONN_UART1_Tx              ((10UL))    // UART1 Tx
  #define GPDMA_CONN_UART1_Rx              ((11UL))    // UART1 Rx
  #define GPDMA_CONN_UART2_Tx              ((12UL))    // UART2 Tx
  #define GPDMA_CONN_UART2_Rx              ((13UL))    // UART2 Rx
  #define GPDMA_CONN_UART3_Tx              ((14UL))    // UART3 Tx
  #define GPDMA_CONN_UART3_Rx              ((15UL))    // UART3 Rx
  #define GPDMA_CONN_MAT0_0                ((8UL))     // MAT0.0
  #define GPDMA_CONN_MAT0_1                ((9UL))     // MAT0.1
  #define GPDMA_CONN_MAT1_0                ((10UL))    // MAT1.0
  #define GPDMA_CONN_MAT1_1                ((11UL))    // MAT1.1
  #define GPDMA_CONN_MAT2_0                ((12UL))    // MAT2.0
  #define GPDMA_CONN_MAT2_1                ((13UL))    // MAT2.1
  #define GPDMA_CONN_MAT3_0                ((14UL))    // MAT3.0
  #define GPDMA_CONN_MAT3_1                ((15UL))    // MAT3.1
#elif defined (LPC177x_8x)
  #define GPDMA_CONN_SD_CARD               ((1UL))     // SD Card
  #define GPDMA_CONN_SSP0_Tx               ((2UL))     // SSP0 Tx
  #define GPDMA_CONN_SSP0_Rx               ((3UL))     // SSP0 Rx
  #define GPDMA_CONN_SSP1_Tx               ((4UL))     // SSP1 Tx
  #define GPDMA_CONN_SSP1_Rx               ((5UL))     // SSP1 Rx
  #define GPDMA_CONN_SSP2_Tx               ((6UL))     // SSP1 Tx
  #define GPDMA_CONN_SSP2_Rx               ((7UL))     // SSP1 Rx
  #define GPDMA_CONN_UART0_Tx              ((10UL))    // UART0 Tx
  #define GPDMA_CONN_UART0_Rx              ((11UL))    // UART0 Rx
  #define GPDMA_CONN_UART1_Tx              ((12UL))    // UART1 Tx
  #define GPDMA_CONN_UART1_Rx              ((13UL))    // UART1 Rx
  #define GPDMA_CONN_UART2_Tx              ((14UL))    // UART2 Tx
  #define GPDMA_CONN_UART2_Rx              ((15UL))    // UART2 Rx
  #define GPDMA_CONN_MAT0_0                ((0UL))     // MAT0.0
  #define GPDMA_CONN_MAT0_1                ((1UL))     // MAT0.1
  #define GPDMA_CONN_MAT1_0                ((2UL))     // MAT1.0
  #define GPDMA_CONN_MAT1_1                ((3UL))     // MAT1.1
  #define GPDMA_CONN_MAT2_0                ((4UL))     // MAT2.0
  #define GPDMA_CONN_MAT2_1                ((5UL))     // MAT2.1
  #define GPDMA_CONN_I2S_Channel_0         ((6UL))     // I2S channel 0
  #define GPDMA_CONN_I2S_Channel_1         ((7UL))     // I2S channel 1
  #define GPDMA_CONN_UART3_Tx              ((10UL))    // UART3 Tx
  #define GPDMA_CONN_UART3_Rx              ((11UL))    // UART3 Rx
  #define GPDMA_CONN_UART4_Tx              ((12UL))    // UART4 Tx
  #define GPDMA_CONN_UART4_Rx              ((13UL))    // UART5 Rx
  #define GPDMA_CONN_MAT3_0                ((14UL))    // MAT3.0
  #define GPDMA_CONN_MAT3_1                ((15UL))    // MAT3.1
#endif


/**
  \fn          void GPDMA_SignalEvent_t (uint32_t event)
  \brief       Signal GPDMA Events.
  \param[in]   event  GPDMA Event mask
  \return      none
*/
typedef void (*GPDMA_SignalEvent_t) (uint32_t event);

/**
  \fn          int32_t GPDMA_Initialize (void)
  \brief       Initialize GPDMA peripheral
  \returns
   - \b  0: function succeeded
   - \b -1: function failed
*/
extern int32_t GPDMA_Initialize (void);

/**
  \fn          int32_t GPDMA_Uninitialize (void)
  \brief       De-initialize GPDMA peripheral
  \returns
   - \b  0: function succeeded
   - \b -1: function failed
*/
extern int32_t GPDMA_Uninitialize (void);

/**
  \fn          int32_t GPDMA_PeripheralSelect (uint8_t peri, uint8_t sel)
  \brief       Selects GPDMA requests
  \param[in]   peri GPDMA peripheral (8..15)
  \param[in]   sel  Selects the DMA request for GPDMA input (0..1)
  \returns
   - \b  0: function succeeded
   - \b -1: function failed
*/
extern int32_t GPDMA_PeripheralSelect (uint8_t peri, uint8_t sel);

/**
  \fn          int32_t GPDMA_ChannelConfigure (uint8_t              ch,
                                               uint32_t             src_addr,
                                               uint32_t             dest_addr,
                                               uint32_t             size,
                                               uint32_t             control,
                                               uint32_t             config,
                                               GPDMA_SignalEvent_t  cb_event)
  \brief       Configure GPDMA channel for next transfer
  \param[in]   ch        Channel number (0..7)
  \param[in]   src_addr  Source address
  \param[in]   dest_addr Destination address
  \param[in]   size      Amount of data to transfer
  \param[in]   control   Channel control
  \param[in]   config    Channel configuration
  \param[in]   cb_event  Channel callback pointer
  \returns
   - \b  0: function succeeded
   - \b -1: function failed
*/
extern int32_t GPDMA_ChannelConfigure (uint8_t              ch,
                                       uint32_t             src_addr,
                                       uint32_t             dest_addr,
                                       uint32_t             size,
                                       uint32_t             control,
                                       uint32_t             config,
                                       GPDMA_SignalEvent_t  cb_event);

/**
  \fn          int32_t GPDMA_ChannelEnable (uint8_t ch)
  \brief       Enable GPDMA channel
  \param[in]   ch Channel number (0..7)
  \returns
   - \b  0: function succeeded
   - \b -1: function failed
*/
extern int32_t GPDMA_ChannelEnable (uint8_t ch);

/**
  \fn          int32_t GPDMA_ChannelDisable (uint8_t ch)
  \brief       Disable GPDMA channel
  \param[in]   ch Channel number (0..7)
  \returns
   - \b  0: function succeeded
   - \b -1: function failed
*/
extern int32_t GPDMA_ChannelDisable (uint8_t ch);

/**
  \fn          uint32_t GPDMA_ChannelGetStatus (uint8_t ch)
  \brief       Check if GPDMA channel is enabled or disabled
  \param[in]   ch Channel number (0..7)
  \returns     Channel status
   - \b  1: channel enabled
   - \b  0: channel disabled
*/
extern uint32_t GPDMA_ChannelGetStatus (uint8_t ch);

/**
  \fn          uint32_t GPDMA_ChannelGetCount (uint8_t ch)
  \brief       Get number of transferred data
  \param[in]   ch Channel number (0..7)
  \returns     Number of transferred data
*/
extern uint32_t GPDMA_ChannelGetCount (uint8_t ch);

#endif /* __GPDMA_LPC17XX_H */
