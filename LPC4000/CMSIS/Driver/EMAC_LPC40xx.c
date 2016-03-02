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
 * $Revision:    V1.0
 *
 * Driver:       Driver_ETH_MAC0
 * Configured:   via RTE_Device.h configuration file
 * Project:      Ethernet Media Access (MAC) Driver for NXP LPC40xx
 * --------------------------------------------------------------------------
 * Use the following configuration settings in the middleware component
 * to connect to this driver.
 *
 *   Configuration Setting                     Value
 *   ---------------------                     -----
 *   Connect to hardware via Driver_ETH_MAC# = 0
 * -------------------------------------------------------------------------- */

/* History:
 *  Version 1.0
 *    - Initial release
 */

#include "EMAC_LPC40xx.h"

#define ARM_ETH_MAC_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1,0) /* driver version */

/* Timeouts */
#define PHY_TIMEOUT         500         /* PHY Register access timeout in us  */

/* EMAC Memory Buffer configuration for 16K Ethernet RAM */
#define NUM_RX_BUF          4           /* 0x1800 for Rx (4*1536=6.0K)        */
#define NUM_TX_BUF          3           /* 0x1200 for Tx (3*1536=4.6K)        */
#define ETH_BUF_SIZE        1536        /* ETH Receive/Transmit buffer size   */

/* Ethernet Pin definitions */
static const ETH_PIN eth_pins[] = {
  { ETH_MDC_PORT,    ETH_MDC_PIN,    ETH_MDC_CFG    },    /* MII, RMII */
  { ETH_MDIO_PORT,   ETH_MDIO_PIN,   ETH_MDIO_CFG   },    /* MII, RMII */
  { ETH_TXD0_PORT,   ETH_TXD0_PIN,   ETH_TXD0_CFG   },    /* MII, RMII */
  { ETH_TXD1_PORT,   ETH_TXD1_PIN,   ETH_TXD1_CFG   },    /* MII, RMII */
  { ETH_RXD0_PORT,   ETH_RXD0_PIN,   ETH_RXD0_CFG   },    /* MII, RMII */
  { ETH_RXD1_PORT,   ETH_RXD1_PIN,   ETH_RXD1_CFG   },    /* MII, RMII */
  { ETH_TX_EN_PORT,  ETH_TX_EN_PIN,  ETH_TX_EN_CFG  },    /* MII, RMII */
  { ETH_RX_ER_PORT,  ETH_RX_ER_PIN,  ETH_RX_ER_CFG  },    /* MII, RMII */
  { ETH_CRS_PORT,    ETH_CRS_PIN,    ETH_CRS_CFG    },    /* MII, RMII */
#if (ETH_MII)
  { ETH_TXD2_PORT,   ETH_TXD2_PIN    ETH_TXD2_CFG   },    /* MII, ---- */
  { ETH_TXD3_PORT,   ETH_TXD3_PIN    ETH_TXD3_CFG   },    /* MII, ---- */
  { ETH_RXD2_PORT,   ETH_RXD2_PIN    ETH_RXD2_CFG   },    /* MII, ---- */
  { ETH_RXD3_PORT,   ETH_RXD3_PIN,   ETH_RXD3_CFG   },    /* MII, ---- */
  { ETH_TX_CLK_PORT, ETH_TX_CLK_PIN, ETH_TX_CLK_CFG },    /* MII, ---- */
  { ETH_RX_CLK_PORT, ETH_RX_CLK_PIN, ETH_RX_CLK_CFG },    /* MII, ---- */
  { ETH_COL_PORT,    ETH_COL_PIN,    ETH_COL_CFG    },    /* MII, ---- */
  { ETH_RX_DV_PORT,  ETH_RX_DV_PIN,  ETH_RX_DV_CFG  },    /* MII, ---- */
  #if defined(ETH_TX_ER_PORT)
  { ETH_TX_ER_PORT,  ETH_TX_ER_PIN,  ETH_TX_ER_CFG  },    /* MII, ---- */
  #endif
#else
  { ETH_REF_CLK_PORT, ETH_REF_CLK_PIN, ETH_REF_CLK_CFG }, /* ---, RMII */
#endif
};


/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion = {
  ARM_ETH_MAC_API_VERSION,
  ARM_ETH_MAC_DRV_VERSION
};

/* Driver Capabilities */
static const ARM_ETH_MAC_CAPABILITIES DriverCapabilities = {
  0,                                /* checksum_offload_rx_ip4  */
  0,                                /* checksum_offload_rx_ip6  */
  0,                                /* checksum_offload_rx_udp  */
  0,                                /* checksum_offload_rx_tcp  */
  0,                                /* checksum_offload_rx_icmp */
  0,                                /* checksum_offload_tx_ip4  */
  0,                                /* checksum_offload_tx_ip6  */
  0,                                /* checksum_offload_tx_udp  */
  0,                                /* checksum_offload_tx_tcp  */
  0,                                /* checksum_offload_tx_icmp */
  ARM_ETH_INTERFACE_RMII,           /* media_interface          */
  0,                                /* mac_address              */
  1,                                /* event_rx_frame           */
  1,                                /* event_tx_frame           */
  1,                                /* event_wakeup             */
  0                                 /* precision_timer          */
};

/* EMAC local DMA Descriptors. */
static            RX_Desc Rx_Desc[NUM_RX_BUF];
static __align(8) RX_Stat Rx_Stat[NUM_RX_BUF]; /* Must be 8-Byte aligned   */
static            TX_Desc Tx_Desc[NUM_TX_BUF];
static            TX_Stat Tx_Stat[NUM_TX_BUF];

/* EMAC local DMA buffers. */
static uint32_t rx_buf[NUM_RX_BUF][ETH_BUF_SIZE>>2];
static uint32_t tx_buf[NUM_TX_BUF][ETH_BUF_SIZE>>2];

/* Local variables */
static EMAC_CTRL  Emac = { 0 };

/* Local functions */
static void init_rx_desc (void);
static void init_tx_desc (void);
static uint32_t crc32_8bit_rev (uint32_t crc32, uint8_t val);
static uint32_t crc32_data (const uint8_t *data, uint32_t len);


/**
  \fn          void init_rx_desc (void)
  \brief       Initialize Rx DMA descriptors.
  \return      none.
*/
static void init_rx_desc (void) {
  uint32_t i;

  for (i = 0; i < NUM_RX_BUF; i++) {
    Rx_Desc[i].Packet  = (uint8_t *)&rx_buf[i];
    Rx_Desc[i].Ctrl    = RCTRL_INT | (ETH_BUF_SIZE-1);
    Rx_Stat[i].Info    = 0;
    Rx_Stat[i].HashCRC = 0;
  }

  /* Set EMAC Receive Descriptor Registers */
  LPC_EMAC->RxDescriptor       = (uint32_t)&Rx_Desc[0];
  LPC_EMAC->RxStatus           = (uint32_t)&Rx_Stat[0];
  LPC_EMAC->RxDescriptorNumber = NUM_RX_BUF-1;

  /* Rx Descriptors Point to 0 */
  LPC_EMAC->RxConsumeIndex  = 0;
}

/**
  \fn          void init_tx_desc (void)
  \brief       Initialize Tx DMA descriptors.
  \return      none.
*/
static void init_tx_desc (void) {
  uint32_t i;

  for (i = 0; i < NUM_TX_BUF; i++) {
    Tx_Desc[i].Packet = (uint8_t *)&tx_buf[i];
    Tx_Desc[i].Ctrl   = 0;
    Tx_Stat[i].Info   = 0;
  }

  /* Set EMAC Transmit Descriptor Registers */
  LPC_EMAC->TxDescriptor       = (uint32_t)&Tx_Desc[0];
  LPC_EMAC->TxStatus           = (uint32_t)&Tx_Stat[0];
  LPC_EMAC->TxDescriptorNumber = NUM_TX_BUF-1;

  /* Tx Descriptors Point to 0 */
  LPC_EMAC->TxProduceIndex  = 0;
}

/**
  \fn          uint32_t crc32_8bit_rev (uint32_t crc32, uint8_t val)
  \brief       Calculate 32-bit CRC (Polynom: 0x04C11DB7, data bit-reversed).
  \param[in]   crc32  CRC initial value
  \param[in]   val    Input value
  \return      Calculated CRC value
*/
static uint32_t crc32_8bit_rev (uint32_t crc32, uint8_t val) {
  uint32_t n;

  crc32 ^= __RBIT(val);
  for (n = 8; n; n--) {
    if (crc32 & 0x80000000) {
      crc32 <<= 1;
      crc32  ^= 0x04C11DB7;
    } else {
      crc32 <<= 1;
    }
  }
  return (crc32);
}

/**
  \fn          uint32_t crc32_data (const uint8_t *data, uint32_t len)
  \brief       Calculate standard 32-bit Ethernet CRC.
  \param[in]   data  Pointer to buffer containing the data
  \param[in]   len   Data length in bytes
  \return      Calculated CRC value
*/
static uint32_t crc32_data (const uint8_t *data, uint32_t len) {
  uint32_t crc;

  for (crc = 0xFFFFFFFF; len; len--) {
    crc = crc32_8bit_rev (crc, *data++);
  }
  return (crc);
}

/* Ethernet Driver functions */

/**
  \fn          ARM_DRIVER_VERSION GetVersion (void)
  \brief       Get driver version.
  \return      \ref ARM_DRIVER_VERSION
*/
static ARM_DRIVER_VERSION GetVersion (void) {
  return DriverVersion;
}


/**
  \fn          ARM_ETH_MAC_CAPABILITIES GetCapabilities (void)
  \brief       Get driver capabilities.
  \return      \ref ARM_ETH_MAC_CAPABILITIES
*/
static ARM_ETH_MAC_CAPABILITIES GetCapabilities (void) {
  return DriverCapabilities;
}

/**
  \fn          int32_t Initialize (ARM_ETH_MAC_SignalEvent_t cb_event)
  \brief       Initialize Ethernet MAC Device.
  \param[in]   cb_event  Pointer to \ref ARM_ETH_MAC_SignalEvent
  \return      \ref execution_status
*/
static int32_t Initialize (ARM_ETH_MAC_SignalEvent_t cb_event) {
  const ETH_PIN *io;

  if (Emac.flags & EMAC_FLAG_INIT) { return ARM_DRIVER_OK; }

  /* Configure Ethernet pins */
  for (io = eth_pins; io != &eth_pins[sizeof(eth_pins)/sizeof(ETH_PIN)]; io++) {
    PIN_Configure (io->port, io->pin, io->cfg);
  }

  /* Clear control structure */
  memset (&Emac, 0, sizeof (EMAC_CTRL));

  Emac.cb_event = cb_event;
  Emac.flags    = EMAC_FLAG_INIT;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t Uninitialize (void)
  \brief       De-initialize Ethernet MAC Device.
  \return      \ref execution_status
*/
static int32_t Uninitialize (void) {
  uint32_t cfg_val;
  const ETH_PIN *io;

  Emac.flags = 0;

  /* Reset ethernet pins */
  cfg_val = IOCON_MODE_PULLUP | IOCON_HYS_ENABLE;

  for (io = eth_pins; io != &eth_pins[sizeof(eth_pins)/sizeof(ETH_PIN)]; io++) {
    PIN_Configure (io->port, io->pin, cfg_val);
  }

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t PowerControl (ARM_POWER_STATE state)
  \brief       Control Ethernet MAC Device Power.
  \param[in]   state  Power state
  \return      \ref execution_status
*/
static int32_t PowerControl (ARM_POWER_STATE state) {
  uint32_t tout,hclk,div;

  switch (state) {
    case ARM_POWER_OFF:
      /* Disable EMAC interrupts */
      NVIC_DisableIRQ(ENET_IRQn);

      /* Power Up the EMAC controller. */
      LPC_SC->PCONP |= 0x40000000;

      /* Reset all EMAC internal modules. */
      LPC_EMAC->MAC1 = MAC1_SOFT_RES;

      /* A short delay after reset. */
      for (tout = 10; tout; tout--);

      /* Power Off the EMAC controller. */
      LPC_SC->PCONP &= ~(0x40000000);

      Emac.flags &= ~EMAC_FLAG_POWER;
      break;

    case ARM_POWER_LOW:
      return ARM_DRIVER_ERROR_UNSUPPORTED;

    case ARM_POWER_FULL:
      if ((Emac.flags & EMAC_FLAG_INIT)  == 0U) {
        return ARM_DRIVER_ERROR;
      }
      if ((Emac.flags & EMAC_FLAG_POWER) != 0U) {
        return ARM_DRIVER_OK;
      }

      /* Power Up the EMAC controller. */
      LPC_SC->PCONP |= 0x40000000;

      /* Reset all EMAC internal modules. */
      LPC_EMAC->MAC1    = MAC1_RES_TX     | MAC1_RES_MCS_TX | MAC1_RES_RX   |
                          MAC1_RES_MCS_RX | MAC1_SIM_RES    | MAC1_SOFT_RES;

      #if (ETH_MII == 0)
      LPC_EMAC->Command = CR_REG_RES | CR_TX_RES | CR_RX_RES | CR_PASS_RUNT_FRM | CR_RMII;
      #else
      LPC_EMAC->Command = CR_REG_RES | CR_TX_RES | CR_RX_RES | CR_PASS_RUNT_FRM;
      #endif

      /* A short delay after reset. */
      for (tout = 10; tout; tout--);

      /* Initialize MAC control registers. */
      LPC_EMAC->MAC1 = MAC1_PASS_ALL;
      LPC_EMAC->MAC2 = MAC2_CRC_EN | MAC2_PAD_EN | MAC2_ADET_PAD_EN;
      LPC_EMAC->MAXF = ETH_BUF_SIZE;
      LPC_EMAC->CLRT = CLRT_DEF;
      LPC_EMAC->IPGR = IPGR_DEF;

      LPC_EMAC->MCFG = MCFG_CLK_SEL | MCFG_RES_MII;
      for (tout = 10; tout; tout--);

      /* MDC clock range selection */
      hclk = SystemCoreClock;
      if      (hclk > 150000000) div = MCFG_CS_Div64;
      else if (hclk > 100000000) div = MCFG_CS_Div60;
      else if (hclk >  60000000) div = MCFG_CS_Div40;
      else if (hclk >  35000000) div = MCFG_CS_Div28;
      else if (hclk >  20000000) div = MCFG_CS_Div14;
      else if (hclk >  10000000) div = MCFG_CS_Div8;
      else                       div = MCFG_CS_Div4;
      LPC_EMAC->MCFG = div;

      /* Enable Reduced MII interface. */
      LPC_EMAC->Command = CR_RMII | CR_PASS_RUNT_FRM;
      
      /* Reset Reduced MII Logic. */
      LPC_EMAC->SUPP = SUPP_RES_RMII;
      for (tout = 10; tout; tout--);
      LPC_EMAC->SUPP = 0;

      /* Initialize Ethernet MAC Address registers */
      LPC_EMAC->SA0 = 0x00000000;
      LPC_EMAC->SA1 = 0x00000000;
      LPC_EMAC->SA2 = 0x00000000;

      /* Initialize Tx and Rx DMA Descriptors */
      init_rx_desc ();
      init_tx_desc ();
  
      /* Receive Perfect Match Packets */
      LPC_EMAC->RxFilterCtrl = RFC_PERFECT_EN;

      /* Enable EMAC interrupts */
      LPC_EMAC->IntClear  = 0xFFFF;
      LPC_EMAC->IntEnable = INT_RX_DONE | INT_TX_DONE;

      /* Enable ethernet interrupts */
      NVIC_ClearPendingIRQ(ENET_IRQn);
      NVIC_EnableIRQ(ENET_IRQn);

      Emac.frame_end = NULL;
      Emac.flags    |= EMAC_FLAG_POWER;
      break;

    default:
      return ARM_DRIVER_ERROR_UNSUPPORTED;
  }
  
  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t GetMacAddress (ARM_ETH_MAC_ADDR *ptr_addr)
  \brief       Get Ethernet MAC Address.
  \param[in]   ptr_addr  Pointer to address
  \return      \ref execution_status
*/
static int32_t GetMacAddress (ARM_ETH_MAC_ADDR *ptr_addr) {
  uint32_t val;

  if (ptr_addr == NULL) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if ((Emac.flags & EMAC_FLAG_POWER) == 0U) {
    return ARM_DRIVER_ERROR;
  }

  val = LPC_EMAC->SA0;
  ptr_addr->b[5] = (uint8_t)(val >> 8);
  ptr_addr->b[4] = (uint8_t)(val);
  val = LPC_EMAC->SA1;
  ptr_addr->b[3] = (uint8_t)(val >> 8);
  ptr_addr->b[2] = (uint8_t)(val);
  val = LPC_EMAC->SA2;
  ptr_addr->b[1] = (uint8_t)(val >> 8);
  ptr_addr->b[0] = (uint8_t)(val);

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t SetMacAddress (const ARM_ETH_MAC_ADDR *ptr_addr)
  \brief       Set Ethernet MAC Address.
  \param[in]   ptr_addr  Pointer to address
  \return      \ref execution_status
*/
static int32_t SetMacAddress (const ARM_ETH_MAC_ADDR *ptr_addr) {

  if (ptr_addr == NULL) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if ((Emac.flags & EMAC_FLAG_POWER) == 0U) {
    return ARM_DRIVER_ERROR;
  }

  /* Set Ethernet MAC Address registers */
  LPC_EMAC->SA0 = (ptr_addr->b[5] << 8) | ptr_addr->b[4];
  LPC_EMAC->SA1 = (ptr_addr->b[3] << 8) | ptr_addr->b[2];
  LPC_EMAC->SA2 = (ptr_addr->b[1] << 8) | ptr_addr->b[0];

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t SetAddressFilter (const ARM_ETH_MAC_ADDR *ptr_addr,
                                               uint32_t          num_addr)
  \brief       Configure Address Filter.
  \param[in]   ptr_addr  Pointer to addresses
  \param[in]   num_addr  Number of addresses to configure
  \return      \ref execution_status
*/
static int32_t SetAddressFilter (const ARM_ETH_MAC_ADDR *ptr_addr, uint32_t num_addr) {
  uint32_t crc;

  if ((ptr_addr == NULL) && (num_addr != 0)) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if ((Emac.flags & EMAC_FLAG_POWER) == 0U) {
    return ARM_DRIVER_ERROR;
  }

  LPC_EMAC->RxFilterCtrl &= ~(RFC_UCAST_HASH_EN | RFC_MCAST_HASH_EN);
  LPC_EMAC->HashFilterH = 0x00000000;
  LPC_EMAC->HashFilterL = 0x00000000;
  
  if (num_addr == 0) {
    return ARM_DRIVER_OK;
  }

  /* Calculate 64-bit Hash table for MAC addresses */
  for ( ; num_addr; ptr_addr++, num_addr--) {
    crc = crc32_data(&ptr_addr->b[0], 6) >> 23;
    if (crc & 0x20) {
      LPC_EMAC->HashFilterH |= (1 << (crc & 0x1F));
    }
    else {
      LPC_EMAC->HashFilterL |= (1 << (crc & 0x1F));
    }
  }

  /* Enable Rx Filter */
  LPC_EMAC->RxFilterCtrl |= (RFC_UCAST_HASH_EN | RFC_MCAST_HASH_EN);

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t SendFrame (const uint8_t *frame, uint32_t len, uint32_t flags)
  \brief       Send Ethernet frame.
  \param[in]   frame  Pointer to frame buffer with data to send
  \param[in]   len    Frame buffer length in bytes
  \param[in]   flags  Frame transmit flags (see ARM_ETH_MAC_TX_FRAME_...)
  \return      \ref execution_status
*/
static int32_t SendFrame (const uint8_t *frame, uint32_t len, uint32_t flags) {
  uint8_t *dst;
  uint32_t idx;

  if ((frame == NULL) || (len == 0U)) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if ((Emac.flags & EMAC_FLAG_POWER) == 0U) {
    return ARM_DRIVER_ERROR;
  }

  dst = Emac.frame_end;
  idx = LPC_EMAC->TxProduceIndex;
  if (dst == NULL) {
    dst = Tx_Desc[idx].Packet;
    Emac.frame_len = len;
  }
  else {
    /* Sending data fragments in progress */
    Emac.frame_len += len;
  }
  /* Fast-copy data fragments to EMAC-DMA buffer */
  for ( ; len > 7; dst += 8, frame += 8, len -= 8) {
    ((__packed uint32_t *)dst)[0] = ((__packed uint32_t *)frame)[0];
    ((__packed uint32_t *)dst)[1] = ((__packed uint32_t *)frame)[1];
  }
  /* Copy remaining 7 bytes */
  for ( ; len > 1; dst += 2, frame += 2, len -= 2) {
    ((__packed uint16_t *)dst)[0] = ((__packed uint16_t *)frame)[0];
  }
  if (len > 0) dst++[0] = frame++[0];

  if (flags & ARM_ETH_MAC_TX_FRAME_FRAGMENT) {
    /* More data to come, remember current write position */
    Emac.frame_end = dst;
    return ARM_DRIVER_OK;
  }

  Tx_Desc[idx].Ctrl = (Emac.frame_len-1) | (TCTRL_INT | TCTRL_LAST);

  Emac.frame_end = NULL;
  Emac.frame_len = 0;

  /* Start frame transmission. */
  if (++idx == NUM_TX_BUF) idx = 0;
  LPC_EMAC->TxProduceIndex = idx;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t ReadFrame (uint8_t *frame, uint32_t len)
  \brief       Read data of received Ethernet frame.
  \param[in]   frame  Pointer to frame buffer for data to read into
  \param[in]   len    Frame buffer length in bytes
  \return      number of data bytes read or execution status
                 - value >= 0: number of data bytes read
                 - value < 0: error occurred, value is execution status as defined with \ref execution_status 
*/
static int32_t ReadFrame (uint8_t *frame, uint32_t len) {
  uint8_t const *src;
  uint32_t idx;
  int32_t cnt = (int32_t)len;

  if ((frame == NULL) && (len != 0U)) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if ((Emac.flags & EMAC_FLAG_POWER) == 0U) {
    return ARM_DRIVER_ERROR;
  }

  idx = LPC_EMAC->RxConsumeIndex;
  src = (uint8_t const *)Rx_Desc[idx].Packet;
  /* Fast-copy data to packet buffer */
  for ( ; len > 7; frame += 8, src += 8, len -= 8) {
    ((__packed uint32_t *)frame)[0] = ((uint32_t *)src)[0];
    ((__packed uint32_t *)frame)[1] = ((uint32_t *)src)[1];
  }
  /* Copy remaining 7 bytes */
  for ( ; len > 1; frame += 2, src += 2, len -= 2) {
    ((__packed uint16_t *)frame)[0] = ((uint16_t *)src)[0];
  }
  if (len > 0) frame[0] = src[0];

  if (++idx == NUM_RX_BUF) idx = 0;
  /* Release frame from EMAC buffer */
  LPC_EMAC->RxConsumeIndex = idx;

  return (cnt);
}

/**
  \fn          uint32_t GetRxFrameSize (void)
  \brief       Get size of received Ethernet frame.
  \return      number of bytes in received frame
*/
static uint32_t GetRxFrameSize (void) {
  uint32_t info,idx;

  if ((Emac.flags & EMAC_FLAG_POWER) == 0U) {
    return (0U);
  }

  idx = LPC_EMAC->RxConsumeIndex;
  if (idx == LPC_EMAC->RxProduceIndex) {
    /* No packet received */
    return (0);
  }

  info = Rx_Stat[idx].Info;
  if (!(info & RINFO_LAST_FLAG) || (info & RINFO_ERR_MASK)) {
    /* Error, this block is invalid */
    return (0xFFFFFFFF);
  }

  return ((info & RINFO_SIZE) - 3);
}


/**
  \fn          int32_t GetRxFrameTime (ARM_ETH_MAC_TIME *time)
  \brief       Get time of received Ethernet frame.
  \param[in]   time  Pointer to time structure for data to read into
  \return      \ref execution_status
*/
static int32_t GetRxFrameTime (ARM_ETH_MAC_TIME *time) {
  return ARM_DRIVER_ERROR_UNSUPPORTED;
}

/**
  \fn          int32_t GetTxFrameTime (ARM_ETH_MAC_TIME *time)
  \brief       Get time of transmitted Ethernet frame.
  \param[in]   time  Pointer to time structure for data to read into
  \return      \ref execution_status
*/
static int32_t GetTxFrameTime (ARM_ETH_MAC_TIME *time) {
  return ARM_DRIVER_ERROR_UNSUPPORTED;
}

/**
  \fn          int32_t Control (uint32_t control, uint32_t arg)
  \brief       Control Ethernet Interface.
  \param[in]   control  operation
  \param[in]   arg      argument of operation (optional)
  \return      \ref execution_status
*/
static int32_t Control (uint32_t control, uint32_t arg) {
  uint32_t rxfilterctrl;
  uint32_t command, mac1, mac2, igpt, supp;

  if ((Emac.flags & EMAC_FLAG_POWER) == 0U) {
    return ARM_DRIVER_ERROR;
  }

  switch (control) {
    case ARM_ETH_MAC_CONFIGURE:
      mac2    = LPC_EMAC->MAC2    & ~(MAC2_FULL_DUP | MAC2_FULL_DUP);
      command = LPC_EMAC->Command & ~CR_FULL_DUP;
      igpt    = LPC_EMAC->IPGT    & ~IPGT_FULL_DUP;
      mac1    = LPC_EMAC->MAC1    & ~MAC1_LOOPB;
      supp    = LPC_EMAC->SUPP    & ~SUPP_SPEED;
      /* Configure 100MBit/10MBit mode */
      switch (arg & ARM_ETH_MAC_SPEED_Msk) {
        case ARM_ETH_MAC_SPEED_10M:
          break;
        case ARM_ETH_SPEED_100M:
          #if (ETH_MII == 0)
          supp |= SUPP_SPEED;
          #endif
          break;
        default:
          return ARM_DRIVER_ERROR_UNSUPPORTED;
      }

      /* Configure Half/Full duplex mode */
      switch (arg & ARM_ETH_MAC_DUPLEX_Msk) {
        case ARM_ETH_MAC_DUPLEX_FULL:
          mac2    |= MAC2_FULL_DUP;
          command |= CR_FULL_DUP;        
          igpt    |= IPGT_FULL_DUP;        
          break;
      }

      /* Configure loopback mode */
      if (arg & ARM_ETH_MAC_LOOPBACK) {
        mac1 |= MAC1_LOOPB;
      }

      if ((arg & ARM_ETH_MAC_CHECKSUM_OFFLOAD_RX) ||
          (arg & ARM_ETH_MAC_CHECKSUM_OFFLOAD_TX)) {
        /* Checksum offload is disabled in the driver */
        return ARM_DRIVER_ERROR_UNSUPPORTED;
      }

      LPC_EMAC->SUPP    = supp;
      LPC_EMAC->MAC2    = mac2;
      LPC_EMAC->Command = command;
      LPC_EMAC->IPGT    = igpt;
      LPC_EMAC->MAC1    = mac1;

      rxfilterctrl = LPC_EMAC->RxFilterCtrl & ~(RFC_UCAST_EN | RFC_BCAST_EN | RFC_MCAST_EN);
      /* Enable broadcast frame receive */
      if (arg & ARM_ETH_MAC_ADDRESS_BROADCAST) {
        rxfilterctrl |= RFC_BCAST_EN;
      }

      /* Enable all multicast frame receive */
      if (arg & ARM_ETH_MAC_ADDRESS_MULTICAST) {
        rxfilterctrl |= RFC_MCAST_EN;
      }

      /* Enable promiscuous mode (no filtering) */
      if (arg & ARM_ETH_MAC_ADDRESS_ALL) {
        rxfilterctrl |= (RFC_BCAST_EN | RFC_UCAST_EN | RFC_MCAST_EN);
      }
      LPC_EMAC->RxFilterCtrl = rxfilterctrl;
      break;

    case ARM_ETH_MAC_CONTROL_TX:
      /* Enable/disable MAC transmitter */
      command = LPC_EMAC->Command & ~CR_TX_EN;

      if (arg != 0) {
        command |= CR_TX_EN;
      }
      LPC_EMAC->Command = command;
      break;

    case ARM_ETH_MAC_CONTROL_RX:
      /* Enable/disable MAC receiver */
      command = LPC_EMAC->Command & ~CR_RX_EN;
      mac1    = LPC_EMAC->MAC1    & ~MAC1_REC_EN;
      if (arg != 0) {
        command |= CR_RX_EN;
        mac1    |= MAC1_REC_EN;
      }
      LPC_EMAC->Command = command;
      LPC_EMAC->MAC1    = mac1;
      break;

    case ARM_ETH_MAC_FLUSH:
      /* Flush Tx and Rx buffers */
      if (arg & ARM_ETH_MAC_FLUSH_RX) {
        /* Stop/Start DMA Receive */
        command = LPC_EMAC->Command;
        LPC_EMAC->Command &= ~CR_RX_EN;
        init_rx_desc ();
        LPC_EMAC->Command = command;
      }
      if (arg & ARM_ETH_MAC_FLUSH_TX) {
        /* Stop/Start DMA Transmit */
        command = LPC_EMAC->Command;
        LPC_EMAC->Command &= ~CR_TX_EN;
        init_tx_desc ();
        LPC_EMAC->Command = command;
      }
      break;

    case ARM_ETH_MAC_VLAN_FILTER:
      /* Configure VLAN filter */
    default:
      return ARM_DRIVER_ERROR_UNSUPPORTED;
  }
  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t ControlTimer (uint32_t control, ARM_ETH_MAC_TIME *time)
  \brief       Control Precision Timer.
  \param[in]   control  operation
  \param[in]   time     Pointer to time structure
  \return      \ref execution_status
*/
static int32_t ControlTimer (uint32_t control, ARM_ETH_MAC_TIME *time) {

  return ARM_DRIVER_ERROR_UNSUPPORTED;
}

/**
  \fn          int32_t PHY_Read (uint8_t phy_addr, uint8_t reg_addr, uint16_t *data)
  \brief       Read Ethernet PHY Register through Management Interface.
  \param[in]   phy_addr  5-bit device address
  \param[in]   reg_addr  5-bit register address
  \param[out]  data      Pointer where the result is written to
  \return      \ref execution_status
*/
static int32_t PHY_Read (uint8_t phy_addr, uint8_t reg_addr, uint16_t *data) {
  uint32_t tick;

  if (data == NULL) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if ((Emac.flags & EMAC_FLAG_POWER) == 0U) {
    return ARM_DRIVER_ERROR;
  }

  LPC_EMAC->MADR = (phy_addr << 8) | reg_addr;
  LPC_EMAC->MCMD = MCMD_READ;

  /* Wait until operation completed */
  tick = osKernelSysTick();
  do {
    if ((LPC_EMAC->MIND & MIND_BUSY) == 0) break;
  } while ((osKernelSysTick() - tick) < osKernelSysTickMicroSec(PHY_TIMEOUT));

  if ((LPC_EMAC->MIND & MIND_BUSY) == 0) {
    LPC_EMAC->MCMD = 0;
    *data = LPC_EMAC->MRDD;
    return ARM_DRIVER_OK;
  }

  return ARM_DRIVER_ERROR_TIMEOUT;
}

/**
  \fn          int32_t PHY_Write (uint8_t phy_addr, uint8_t reg_addr, uint16_t data)
  \brief       Write Ethernet PHY Register through Management Interface.
  \param[in]   phy_addr  5-bit device address
  \param[in]   reg_addr  5-bit register address
  \param[in]   data      16-bit data to write
  \return      \ref execution_status
*/
static int32_t PHY_Write (uint8_t phy_addr, uint8_t reg_addr, uint16_t data) {
  uint32_t tick;

  if ((Emac.flags & EMAC_FLAG_POWER) == 0U) {
    return ARM_DRIVER_ERROR;
  }

  LPC_EMAC->MADR = (phy_addr << 8) | reg_addr;
  LPC_EMAC->MWTD = data;

  /* Wait until operation completed */
  tick = osKernelSysTick();
  do {
    if ((LPC_EMAC->MIND & MIND_BUSY) == 0) break;
  } while ((osKernelSysTick() - tick) < osKernelSysTickMicroSec(PHY_TIMEOUT));
  
  if ((LPC_EMAC->MIND & MIND_BUSY) == 0) {
    return ARM_DRIVER_OK;
  }

  return ARM_DRIVER_ERROR_TIMEOUT;
}


/* Ethernet IRQ Handler */
void ENET_IRQHandler (void) {
  uint32_t int_stat;
  uint32_t event = 0;

  int_stat = (LPC_EMAC->IntStatus & LPC_EMAC->IntEnable);
  LPC_EMAC->IntClear = int_stat;
  
  if (int_stat & INT_RX_DONE) {
    /* Packet received, check if packet is valid. */
    event |= ARM_ETH_MAC_EVENT_RX_FRAME;
  }
  if (int_stat & INT_TX_DONE) {
    /* Frame transmit completed. */
    event |= ARM_ETH_MAC_EVENT_TX_FRAME;
  }
  /* Callback event notification */
  if (event && Emac.cb_event) {
    Emac.cb_event (event);
  }
}


/* MAC Driver Control Block */
ARM_DRIVER_ETH_MAC Driver_ETH_MAC0 = {
  GetVersion,
  GetCapabilities,
  Initialize,
  Uninitialize,
  PowerControl,
  GetMacAddress,
  SetMacAddress,
  SetAddressFilter,
  SendFrame,
  ReadFrame,
  GetRxFrameSize,
  GetRxFrameTime,
  GetTxFrameTime,
  ControlTimer,
  Control,
  PHY_Read,
  PHY_Write
};
