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
 * $Revision:    V1.3
 *
 * Project:      USB common (Device and Host) module for NXP LPC17xx
 * -------------------------------------------------------------------------- */

/* History:
 *  Version 1.2
 *    Added dependency on RTE_Device.h pin enable for LPC175x/6x
 *  Version 1.1
 *    Added U1 and U2 support for LPC177x/8x
 *    Changed include for renamed RTE_Device_LPC177x_8x.h to RTE_Device.h
 *  Version 1.0
 *    Initial release
 */

#if defined (LPC175x_6x)
  #include "LPC17xx.h"
  #include "PIN_LPC17xx.h"
#elif defined (LPC177x_8x)
  #include "LPC177x_8x.h"
  #include "PIN_LPC177x_8x.h"
#endif

#include "Driver_USB.h"

#include "RTE_Device.h"
#include "RTE_Components.h"

#if defined (LPC177x_8x)
#define IOCON_D_RST         (IOCON_MODE_PULLUP | IOCON_HYS_ENABLE)
#define IOCON_A_RST         (IOCON_MODE_PULLUP | IOCON_DIGITIAL_MODE | IOCON_10ns_FILTER_DISABLE)
#define IOCON_U_RST         (0)
#define IOCON_I_RST         (0)
#define IOCON_W_RST         (IOCON_HYS_ENABLE  | IOCON_DIGITIAL_MODE)
#define IOCON_D_USB         (IOCON_MODE_PULLUP | IOCON_HYS_ENABLE)
#define IOCON_A_USB         (IOCON_MODE_PULLUP | IOCON_DIGITIAL_MODE)
#define IOCON_U_USB         (0)
#define IOCON_I_USB         (0)
#define IOCON_W_USB         (IOCON_MODE_PULLUP | IOCON_HYS_ENABLE | IOCON_DIGITIAL_MODE)

#ifndef USB_I2C_OTG_TRANSCEIVER_ADDR
#define USB_I2C_OTG_TRANSCEIVER_ADDR   (0x2D)
#endif
#endif

//Function Prototypes
void USB_IRQHandler (void);
int32_t USB_I2C_RegisterRead (uint8_t i2c_addr, uint8_t reg_addr);
int32_t USB_I2C_RegisterWrite (uint8_t i2c_addr, uint8_t reg_addr, uint8_t reg_val);
int32_t USB_I2C_Initialize (void);
int32_t USB_I2C_Uninitialize (void);
int32_t USB_I2C_DpPullUp (bool enable);
int32_t USB_I2C_ControlDmPullUp (bool enable);
int32_t USB_I2C_ControlPullDowns (bool enable);

//External functions
extern void SystemCoreClockUpdate (void);
extern int32_t USB_PinsUnconfigure (void);
extern int32_t USB_PinsConfigure (void);

extern uint8_t usb_role;
extern uint8_t usb_state;
extern uint8_t usb_pin_cfg;

uint8_t usb_role = ARM_USB_ROLE_NONE;
uint8_t usb_state = 0U;
uint8_t usb_pin_cfg = 0U;

#if defined (LPC177x_8x)
static uint8_t  usb_i2c_init = 0U;
//static uint16_t usb_i2c_id = 0U;
#endif

#ifdef RTE_Drivers_USBH0
extern void USBH_IRQ (void);
#endif
#ifdef RTE_Drivers_USBD0
extern void USBD_IRQ (void);
#endif


#if defined (LPC177x_8x)
/**
  \fn          int32_t USB_I2C_RegisterRead (void)
  \brief       Read I2C Device Register.
  \param[in]   i2c_addr  I2C Address
  \param[in]   reg_addr  Register Address
  \return                      register value
               - value 0..255: register value
               - value -1:     register read has failed
*/
int32_t USB_I2C_RegisterRead (uint8_t i2c_addr, uint8_t reg_addr) {

  if (usb_i2c_init == 0U) { return -1; }

  LPC_USB->I2C_TX = (      1U << 8) |           // START bit on transmit start
                    ((uint32_t)i2c_addr << 1)   // I2C Address
                                    ;           // Write request
  while ((LPC_USB->I2C_STS & (1U << 11)) == 0U);// Wait for Tx FIFO empty
  LPC_USB->I2C_TX =  reg_addr;                  // Register address to read from
  while ((LPC_USB->I2C_STS & (1U << 11)) == 0U);// Wait for Tx FIFO empty
  LPC_USB->I2C_TX = (      1U << 8) |           // START bit on transmit start
                    ((uint32_t)i2c_addr << 1) | // I2C Address
                    (      1U     ) ;           // Read request
  LPC_USB->I2C_TX = (      1U << 9) |           // STOP bit on end
                        0x55U       ;           // Dummy data transmit to receive
  while ((LPC_USB->I2C_STS & 1U) == 0U);        // Wait for transaction done
  while ((LPC_USB->I2C_STS & (1U << 5)) != 0U); // Wait for STOP condition

  return (LPC_USB->I2C_RX & 0xFFU);
}

/**
  \fn          int32_t USB_I2C_RegisterWrite (void)
  \brief       Write I2C Device Register.
  \param[in]   i2c_addr  I2C Address
  \param[in]   reg_addr  Register Address
  \param[in]   reg_val   Register Value
  \return                      result of operation
               - value 0:      register written successfully
               - value -1:     register write has failed
*/
int32_t USB_I2C_RegisterWrite (uint8_t i2c_addr, uint8_t reg_addr, uint8_t reg_val) {

  if (usb_i2c_init == 0U) { return -1; }

  LPC_USB->I2C_TX = (      1U << 8) |           // START bit on transmit start
                    ((uint32_t)i2c_addr << 1)   // I2C Address
                                    ;           // Write request
  while ((LPC_USB->I2C_STS & (1U << 11)) == 0U);// Wait for Tx FIFO empty
  LPC_USB->I2C_TX =  reg_addr;                  // Register address to write to
  while ((LPC_USB->I2C_STS & (1U << 11)) == 0U);// Wait for Tx FIFO empty
  LPC_USB->I2C_TX = (      1U << 9) |           // STOP bit on end
                     reg_val        ;           // Register value to write
  while ((LPC_USB->I2C_STS & (1U << 5)) != 0U); // Wait for STOP condition

  return 0;
}

/**
  \fn          int32_t USB_I2C_Initialize (void)
  \brief       Initialize USB I2C Interface.
  \return                  result of operation
               - value 0:  I2C initialized successfully
               - value -1: I2C initialization has failed
*/
int32_t USB_I2C_Initialize (void) {

  if (usb_i2c_init != 0U) { return 0U; }

  SystemCoreClockUpdate ();

  LPC_USB->OTGClkCtrl |= 0x04U;                 // Enable I2C clock
  while ((LPC_USB->OTGClkSt & 0x04U) == 0U);

  LPC_USB->I2C_CTL = 1U << 8;                   // I2C reset
  while ((LPC_USB->I2C_CTL & (1U << 8)) != 0U); // Wait reset end

                                                // Set I2C clock to 100kHz
  LPC_USB->I2C_CLKHI = USBClock/200000U;        // High clock period
  LPC_USB->I2C_CLKLO = USBClock/200000U;        // Low  clock period

  usb_i2c_init = 1U;

  // Get Transceiver ID:
  //   - MIC2555 Transceiver ID: 0x058D
  // usb_i2c_id = USB_I2C_RegisterRead (USB_I2C_OTG_TRANSCEIVER_ADDR, 0x00) | (USB_I2C_RegisterRead (USB_I2C_OTG_TRANSCEIVER_ADDR, 0x01) << 8);

  return 0U;
}

/**
  \fn          int32_t USB_I2C_Uninitialize (void)
  \brief       De-initialize USB I2C Interface.
  \return                  result of operation
               - value 0:  I2C de-initialized successfully
               - value -1: I2C de-initialization has failed
*/
int32_t USB_I2C_Uninitialize (void) {

  // Return I2C to state after reset
  LPC_USB->I2C_CLKHI = 0xB9U;
  LPC_USB->I2C_CLKLO = 0xB9U;

  LPC_USB->OTGClkCtrl &= ~0x04U;                // Disable I2C clock
  while ((LPC_USB->OTGClkSt & 0x04U) != 0U);

  usb_i2c_init = 0U;

  return 0U;
}

/**
  \fn          int32_t USB_I2C_ControlDpPullUp (void)
  \brief       Enable/disable Pull-up on D+ line.
  \param[in]   enable          Requested action
               - true:         enable
               - false:        disable
  \return                      result of operation
               - value 0:      operation finished successfully
               - value -1:     operation has failed
*/
int32_t USB_I2C_DpPullUp (bool enable) {

  // Control Register 2: dp_pull_up bit
  return (USB_I2C_RegisterWrite (USB_I2C_OTG_TRANSCEIVER_ADDR, 0x06U + (enable == 0U), 0x01));
}

/**
  \fn          int32_t USB_I2C_ControlDmPullUp (void)
  \brief       Enable/disable Pull-up on D- line.
  \param[in]   enable          Requested action
               - true:         enable
               - false:        disable
  \return                      result of operation
               - value 0:      operation finished successfully
               - value -1:     operation has failed
*/
int32_t USB_I2C_ControlDmPullUp (bool enable) {

  // Control Register 2: dm_pull_up bit
  return (USB_I2C_RegisterWrite (USB_I2C_OTG_TRANSCEIVER_ADDR, 0x06U + (enable == 0U), 0x02));
}

/**
  \fn          int32_t USB_I2C_ControlPullDowns (void)
  \brief       Enable/disable Pull-down on D+ and D- lines.
  \param[in]   enable          Requested action
               - true:         enable
               - false:        disable
  \return                      result of operation
               - value 0:      operation finished successfully
               - value -1:     operation has failed
*/
int32_t USB_I2C_ControlPullDowns (bool enable) {

  // Control Register 2: dm_pull_down and dp_pull_down bits
  return (USB_I2C_RegisterWrite (USB_I2C_OTG_TRANSCEIVER_ADDR, 0x06U + (enable == 0U), 0x0C));
}
#endif

/**
  \fn          void USB_IRQHandler (void)
  \brief       USB Interrupt Routine (IRQ).
*/
void USB_IRQHandler (void) {

#if  (defined(RTE_Drivers_USBH0) && defined(RTE_Drivers_USBD0))
#if   defined (LPC175x_6x)
  switch (usb_role) {
    case ARM_USB_ROLE_HOST:
      USBH0_IRQ ();
      break;
    case ARM_USB_ROLE_DEVICE:
      USBD0_IRQ ();
      break;
    default:
      break;
  }
#elif defined (LPC177x_8x)
  uint32_t USBIntSt;

  USBIntSt = LPC_SC->USBIntSt;
  if ((USBIntSt & 0x08U) != 0U) {       // Host IRQ
    USBH_IRQ ();
  }
  if ((USBIntSt & 0x07U) != 0U) {       // Device IRQ
    USBD_IRQ ();
  }
#endif
#else
#ifdef RTE_Drivers_USBH0
  USBH_IRQ ();
#else
  USBD_IRQ ();
#endif
#endif
}

/**
  \fn          int32_t USB_PinsConfigure (void)
  \brief       Configure USB pins
  \return                  result of pin configuration
               - value 0:  pins configured correctly
               - value -1: pins configuration has failed
*/
int32_t USB_PinsConfigure (void) {
#if defined (LPC177x_8x)
  volatile uint8_t U1_role = 0U;
  volatile uint8_t U2_role = 0U;
#endif

  if (usb_pin_cfg == 1U) { return 0U; }
  usb_pin_cfg = 0U;

#if   defined (LPC175x_6x)
  switch (usb_role) {
    // Host pins
    case ARM_USB_ROLE_HOST:
#if   (RTE_USB_PPWR_PIN_EN == 1)
      // P1.19 - USB_PPWR
      if (PIN_Configure (1, 19, PIN_FUNC_2, PIN_PINMODE_PULLUP,   PIN_PINMODE_NORMAL) != 0) { return -1; }
#endif
#if   (RTE_USB_PWRD_PIN_EN == 1)
      // P1.22 - USB_PWRD
      if (PIN_Configure (1, 22, PIN_FUNC_2, PIN_PINMODE_TRISTATE, PIN_PINMODE_NORMAL) != 0) { return -1; }
#endif
#if   (RTE_USB_OVRCR_PIN_EN == 1)
      // P1.27 - USB_OVRCR
      if (PIN_Configure (1, 27, PIN_FUNC_2, PIN_PINMODE_TRISTATE, PIN_PINMODE_NORMAL) != 0) { return -1; }
#endif
      break;

    // Device pins
    case ARM_USB_ROLE_DEVICE:
#if   (RTE_USB_CONNECT_PIN_EN == 1)
      // P2.9 SoftConnect
      if (PIN_Configure (2, 9,  PIN_FUNC_1, PIN_PINMODE_PULLDOWN, PIN_PINMODE_NORMAL) != 0) { return -1; }
#endif
#if   (RTE_USB_VBUS_PIN_EN == 1)
      // P1.30 VBUS
      if (PIN_Configure (1, 30, PIN_FUNC_2, PIN_PINMODE_TRISTATE, PIN_PINMODE_NORMAL) != 0) { return -1; }
#endif
      break;

    default:
      return -1;
  }

  // Common pins configuration
#if   (RTE_USB_UP_LED_PIN_EN == 1)
  // P1.18 GoodLink
  if (PIN_Configure (1, 18, PIN_FUNC_1, PIN_PINMODE_PULLUP,   PIN_PINMODE_NORMAL) != 0) { return -1; }
#endif
  // P0.29 D+
  if (PIN_Configure (0, 29, PIN_FUNC_1, PIN_PINMODE_TRISTATE, PIN_PINMODE_NORMAL) != 0) { return -1; }
  // P0.30 D-
  if (PIN_Configure (0, 30, PIN_FUNC_1, PIN_PINMODE_TRISTATE, PIN_PINMODE_NORMAL) != 0) { return -1; }

  usb_pin_cfg = 1U;
  return 0U;

#elif defined (LPC177x_8x)

#if   (RTE_USB_PORT_CFG == 0)
  U1_role = ARM_USB_ROLE_DEVICE;
  U2_role = ARM_USB_ROLE_HOST;
#elif (RTE_USB_PORT_CFG == 1)
  U1_role = ARM_USB_ROLE_HOST;
  U2_role = ARM_USB_ROLE_HOST;
#elif (RTE_USB_PORT_CFG == 2)
  U1_role = ARM_USB_ROLE_NONE;
  U2_role = ARM_USB_ROLE_NONE;
#elif (RTE_USB_PORT_CFG == 3)
  U1_role = ARM_USB_ROLE_HOST;
  U2_role = ARM_USB_ROLE_DEVICE;
#endif

#if   (RTE_USB_PORT1_EN == 1)
  switch (U1_role) {
    // Host pins
    case ARM_USB_ROLE_HOST:
#if   (RTE_USB_PPWR1_PIN_EN == 1)
      // USB_PPWR1
      if (PIN_Configure (RTE_USB_PPWR1_PORT,  RTE_USB_PPWR1_BIT,  RTE_USB_PPWR1_FUNC  | IOCON_D_USB) != 0) { return -1; }
#endif
#if   (RTE_USB_PWRD1_PIN_EN == 1)
      // USB_PWRD1
      if (PIN_Configure (RTE_USB_PWRD1_PORT,  RTE_USB_PWRD1_BIT,  RTE_USB_PWRD1_FUNC  | IOCON_D_USB) != 0) { return -1; }
#endif
#if   (RTE_USB_OVRCR1_PIN_EN == 1)
      // USB_OVRCR1
      if (PIN_Configure (RTE_USB_OVRCR1_PORT, RTE_USB_OVRCR1_BIT, RTE_USB_OVRCR1_FUNC | IOCON_D_USB) != 0) { return -1; }
#endif
      break;

    // Device pins
    case ARM_USB_ROLE_DEVICE:
#if   (RTE_USB_CONNECT1_PIN_EN == 1)
      // SoftConnect1
      if (PIN_Configure (RTE_USB_CONNECT1_PORT, RTE_USB_CONNECT1_BIT,  RTE_USB_CONNECT1_FUNC | IOCON_D_USB) != 0) { return -1; }
#endif
#if   (RTE_USB_VBUS_PIN_EN == 1)
      // VBUS
      if (PIN_Configure (RTE_USB_VBUS_PORT,     RTE_USB_VBUS_BIT,      RTE_USB_VBUS_FUNC     | IOCON_A_USB) != 0) { return -1; }
#endif
      break;

    default:
      return -1;
  }

  // Common pins configuration
#if (RTE_USB_UP_LED1_PIN_EN == 1)
  // USB_UP_LED1
  if (PIN_Configure (RTE_USB_UP_LED1_PORT, RTE_USB_UP_LED1_BIT,  RTE_USB_UP_LED1_FUNC | IOCON_D_USB) != 0) { return -1; }
#endif
#if (RTE_USB_DP1_PIN_EN == 1)
  // USB_D+1
  if (PIN_Configure (RTE_USB_DP1_PORT,     RTE_USB_DP1_BIT,      RTE_USB_DP1_FUNC     | IOCON_U_USB) != 0) { return -1; }
#endif
#if (RTE_USB_DM1_PIN_EN == 1)
  // USB_D-1
  if (PIN_Configure (RTE_USB_DM1_PORT,     RTE_USB_DM1_BIT,      RTE_USB_DM1_FUNC     | IOCON_U_USB) != 0) { return -1; }
#endif
#if (RTE_USB_PORT1_OTG_EN == 1)
#if (RTE_USB_INT1_PIN_EN == 1)
  // USB_INT1
  if (PIN_Configure (RTE_USB_INT1_PORT,    RTE_USB_INT1_BIT,     RTE_USB_INT1_FUNC    | IOCON_D_USB) != 0) { return -1; }
#endif
#if (RTE_USB_SCL1_PIN_EN == 1)
  // USB_SCL1
#if   ((RTE_USB_SCL1_PORT == 0) && (RTE_USB_SCL1_BIT == 28))
  if (PIN_Configure (RTE_USB_SCL1_PORT,    RTE_USB_SCL1_BIT,     RTE_USB_SCL1_FUNC    | IOCON_I_USB) != 0) { return -1; }
#elif ((RTE_USB_SCL1_PORT == 1) && (RTE_USB_SCL1_BIT == 28))
  if (PIN_Configure (RTE_USB_SCL1_PORT,    RTE_USB_SCL1_BIT,     RTE_USB_SCL1_FUNC    | IOCON_D_USB) != 0) { return -1; }
#endif
#endif
#if (RTE_USB_SDA1_PIN_EN == 1)
  // USB_SDA1
#if   ((RTE_USB_SDA1_PORT == 0) && (RTE_USB_SDA1_BIT == 27))
  if (PIN_Configure (RTE_USB_SDA1_PORT,    RTE_USB_SDA1_BIT,     RTE_USB_SDA1_FUNC    | IOCON_I_USB) != 0) { return -1; }
#elif ((RTE_USB_SDA1_PORT == 1) && (RTE_USB_SDA1_BIT == 29))
  if (PIN_Configure (RTE_USB_SDA1_PORT,    RTE_USB_SDA1_BIT,     RTE_USB_SDA1_FUNC    | IOCON_D_USB) != 0) { return -1; }
#endif
#endif
#if (RTE_USB_TX_E1_PIN_EN == 1)
  // USB_TX_E1
  if (PIN_Configure (RTE_USB_TX_E1_PORT,   RTE_USB_TX_E1_BIT,    RTE_USB_TX_E1_FUNC   | IOCON_D_USB) != 0) { return -1; }
#endif
#if (RTE_USB_TX_DP1_PIN_EN == 1)
  // USB_TX_DP1
  if (PIN_Configure (RTE_USB_TX_DP1_PORT,  RTE_USB_TX_DP1_BIT,   RTE_USB_TX_DP1_FUNC  | IOCON_D_USB) != 0) { return -1; }
#endif
#if (RTE_USB_TX_DM1_PIN_EN == 1)
  // USB_TX_DM1
  if (PIN_Configure (RTE_USB_TX_DM1_PORT,  RTE_USB_TX_DM1_BIT,   RTE_USB_TX_DM1_FUNC  | IOCON_D_USB) != 0) { return -1; }
#endif
#if (RTE_USB_RCV1_PIN_EN == 1)
  // USB_RCV1
  if (PIN_Configure (RTE_USB_RCV1_PORT,    RTE_USB_RCV1_BIT,     RTE_USB_RCV1_FUNC    | IOCON_D_USB) != 0) { return -1; }
#endif
#if (RTE_USB_RX_DP1_PIN_EN == 1)
  // USB_RX_DP1
  if (PIN_Configure (RTE_USB_RX_DP1_PORT,  RTE_USB_RX_DP1_BIT,   RTE_USB_RX_DP1_FUNC  | IOCON_D_USB) != 0) { return -1; }
#endif
#if (RTE_USB_RX_DM1_PIN_EN == 1)
  // USB_RX_DM1
  if (PIN_Configure (RTE_USB_RX_DM1_PORT,  RTE_USB_RX_DM1_BIT,   RTE_USB_RX_DM1_FUNC  | IOCON_D_USB) != 0) { return -1; }
#endif
#if (RTE_USB_LS1_PIN_EN == 1)
  // USB_LS1
  if (PIN_Configure (RTE_USB_LS1_PORT,     RTE_USB_LS1_BIT,      RTE_USB_LS1_FUNC     | IOCON_D_USB) != 0) { return -1; }
#endif
#if (RTE_USB_SSPND1_PIN_EN == 1)
  // USB_SSPND1
  if (PIN_Configure (RTE_USB_SSPND1_PORT,  RTE_USB_SSPND1_BIT,   RTE_USB_SSPND1_FUNC  | IOCON_D_USB) != 0) { return -1; }
#endif
#endif
#endif

#if   (RTE_USB_PORT2_EN == 1)
  switch (U2_role) {
    // Host pins
    case ARM_USB_ROLE_HOST:
#if   (RTE_USB_PPWR2_PIN_EN == 1)
      // USB_PPWR2
      if (PIN_Configure (RTE_USB_PPWR2_PORT,  RTE_USB_PPWR2_BIT,  RTE_USB_PPWR2_FUNC  | IOCON_A_USB) != 0) { return -1; }
#endif
#if   (RTE_USB_PWRD2_PIN_EN == 1)
      // USB_PWRD2
      if (PIN_Configure (RTE_USB_PWRD2_PORT,  RTE_USB_PWRD2_BIT,  RTE_USB_PWRD2_FUNC  | IOCON_A_USB) != 0) { return -1; }
#endif
#if  (RTE_USB_OVRCR2_PIN_EN == 1)
      // USB_OVRCR2
      if (PIN_Configure (RTE_USB_OVRCR2_PORT, RTE_USB_OVRCR2_BIT, RTE_USB_OVRCR2_FUNC | IOCON_A_USB) != 0) { return -1; }
#endif
      break;

    // Device pins
    case ARM_USB_ROLE_DEVICE:
#if   (RTE_USB_CONNECT2_PIN_EN == 1)
      // SoftConnect2
      if (PIN_Configure (RTE_USB_CONNECT2_PORT, RTE_USB_CONNECT2_BIT,  RTE_USB_CONNECT2_FUNC | IOCON_D_USB) != 0) { return -1; }
#endif
#if   (RTE_USB_VBUS_PIN_EN == 1)
      // VBUS
      if (PIN_Configure (RTE_USB_VBUS_PORT,     RTE_USB_VBUS_BIT,      RTE_USB_VBUS_FUNC     | IOCON_A_USB) != 0) { return -1; }
#endif
      break;

    default:
      return -1;
  }

  // Common pins configuration
#if (RTE_USB_UP_LED2_PIN_EN == 1)
  // USB_UP_LED2
  if (PIN_Configure (RTE_USB_UP_LED2_PORT, RTE_USB_UP_LED2_BIT,  RTE_USB_UP_LED2_FUNC | IOCON_A_USB) != 0) { return -1; }
#endif
#if (RTE_USB_DP2_PIN_EN == 1)
  // USB_D+2   (USB_D-2: Dedicated pin)
  if (PIN_Configure (RTE_USB_DP2_PORT,     RTE_USB_DP2_BIT,      RTE_USB_DP2_FUNC     | IOCON_U_USB) != 0) { return -1; }
#endif
#endif

  usb_pin_cfg = 1;
  return 0;
#else

  return -1;
#endif
}

/**
  \fn          int32_t USB_PinsUnconfigure (void)
  \brief       De-configure USB pins
  \return                  result of pin de-configuration
               - value 0:  pins de-configured correctly
               - value -1: pins de-configuration has failed
*/
int32_t USB_PinsUnconfigure (void) {
#if defined (LPC177x_8x)
  volatile uint8_t U1_role = 0U;
  volatile uint8_t U2_role = 0U;
#endif

  if (usb_pin_cfg == 0U) { return 0U; }

#if   defined (LPC175x_6x)
  switch (usb_role) {
    // Host pins
    case ARM_USB_ROLE_HOST:
#if   (RTE_USB_PPWR_PIN_EN == 1)
      // P1.19 - USB_PPWR
      if (PIN_Configure (1, 19, PIN_FUNC_0, PIN_PINMODE_PULLUP, PIN_PINMODE_NORMAL) != 0) { return -1; }
#endif
#if   (RTE_USB_PWRD_PIN_EN == 1)
      // P1.22 - USB_PWRD
      if (PIN_Configure (1, 22, PIN_FUNC_0, PIN_PINMODE_PULLUP, PIN_PINMODE_NORMAL) != 0) { return -1; }
#endif
#if   (RTE_USB_OVRCR_PIN_EN == 1)
      // P1.27 - USB_OVRCR
      if (PIN_Configure (1, 27, PIN_FUNC_0, PIN_PINMODE_PULLUP, PIN_PINMODE_NORMAL) != 0) { return -1; }
#endif
      break;

    // Device pins
    case ARM_USB_ROLE_DEVICE:
#if   (RTE_USB_CONNECT_PIN_EN == 1)
      // P2.9 SoftConnect
      if (PIN_Configure (2, 9,  PIN_FUNC_0, PIN_PINMODE_PULLUP, PIN_PINMODE_NORMAL) != 0) { return -1; }
#endif
#if   (RTE_USB_VBUS_PIN_EN == 1)
      // P1.30 VBUS
      if (PIN_Configure (1, 30, PIN_FUNC_0, PIN_PINMODE_PULLUP, PIN_PINMODE_NORMAL) != 0) { return -1; }
#endif
      break;

    default:
      return -1;
  }

  // Common pins configuration
#if   (RTE_USB_UP_LED_PIN_EN == 1)
  // P1.18 GoodLink
  if (PIN_Configure (1, 18, PIN_FUNC_0, PIN_PINMODE_PULLUP, PIN_PINMODE_NORMAL) != 0) { return -1; }
#endif
  // P0.29 D+
  if (PIN_Configure (0, 29, PIN_FUNC_0, PIN_PINMODE_PULLUP, PIN_PINMODE_NORMAL) != 0) { return -1; }
  // P0.30 D-
  if (PIN_Configure (0, 30, PIN_FUNC_0, PIN_PINMODE_PULLUP, PIN_PINMODE_NORMAL) != 0) { return -1; }

  usb_pin_cfg = 0;
  return 0;

#elif defined (LPC177x_8x)

#if   (RTE_USB_PORT_CFG == 0)
  U1_role = ARM_USB_ROLE_DEVICE;
  U2_role = ARM_USB_ROLE_HOST;
#elif (RTE_USB_PORT_CFG == 1)
  U1_role = ARM_USB_ROLE_HOST;
  U2_role = ARM_USB_ROLE_HOST;
#elif (RTE_USB_PORT_CFG == 2)
  U1_role = ARM_USB_ROLE_NONE;
  U2_role = ARM_USB_ROLE_NONE;
#elif (RTE_USB_PORT_CFG == 3)
  U1_role = ARM_USB_ROLE_HOST;
  U2_role = ARM_USB_ROLE_DEVICE;
#endif

#if   (RTE_USB_PORT1_EN == 1)
  switch (U1_role) {
    // Host pins
    case ARM_USB_ROLE_HOST:
#if   (RTE_USB_PPWR1_PIN_EN == 1)
      // USB_PPWR1
      if (PIN_Configure (RTE_USB_PPWR1_PORT,  RTE_USB_PPWR1_BIT,  IOCON_D_RST) != 0) { return -1; }
#endif
#if   (RTE_USB_PWRD1_PIN_EN == 1)
      // USB_PWRD1
      if (PIN_Configure (RTE_USB_PWRD1_PORT,  RTE_USB_PWRD1_BIT,  IOCON_D_RST) != 0) { return -1; }
#endif
#if   (RTE_USB_OVRCR1_PIN_EN == 1)
      // USB_OVRCR1
      if (PIN_Configure (RTE_USB_OVRCR1_PORT, RTE_USB_OVRCR1_BIT, IOCON_D_RST) != 0) { return -1; }
#endif
      break;

    // Device pins
    case ARM_USB_ROLE_DEVICE:
#if   (RTE_USB_CONNECT1_PIN_EN == 1)
      // SoftConnect1
      if (PIN_Configure (RTE_USB_CONNECT1_PORT, RTE_USB_CONNECT1_BIT,  IOCON_D_RST) != 0) { return -1; }
#endif
#if   (RTE_USB_VBUS_PIN_EN == 1)
      // VBUS
      if (PIN_Configure (RTE_USB_VBUS_PORT,     RTE_USB_VBUS_BIT,      IOCON_A_RST) != 0) { return -1; }
#endif
      break;

    default:
      return -1;
  }

  // Common pins configuration
#if (RTE_USB_UP_LED1_PIN_EN == 1)
  // USB_UP_LED1
  if (PIN_Configure (RTE_USB_UP_LED1_PORT, RTE_USB_UP_LED1_BIT,  IOCON_D_RST) != 0) { return -1; }
#endif
#if (RTE_USB_DP1_PIN_EN == 1)
  // USB_D+1
  if (PIN_Configure (RTE_USB_DP1_PORT,     RTE_USB_DP1_BIT,      IOCON_U_RST) != 0) { return -1; }
#endif
#if (RTE_USB_DM1_PIN_EN == 1)
  // USB_D-1
  if (PIN_Configure (RTE_USB_DM1_PORT,     RTE_USB_DM1_BIT,      IOCON_U_RST) != 0) { return -1; }
#endif
#if (RTE_USB_PORT1_OTG_EN == 1)
#if (RTE_USB_INT1_PIN_EN == 1)
  // USB_INT1
  if (PIN_Configure (RTE_USB_INT1_PORT,    RTE_USB_INT1_BIT,     IOCON_D_RST) != 0) { return -1; }
#endif
#if (RTE_USB_SCL1_PIN_EN == 1)
  // USB_SCL1
#if   ((RTE_USB_SCL1_PORT == 0) && (RTE_USB_SCL1_BIT == 28))
  if (PIN_Configure (RTE_USB_SCL1_PORT,    RTE_USB_SCL1_BIT,     IOCON_I_RST) != 0) { return -1; }
#elif ((RTE_USB_SCL1_PORT == 1) && (RTE_USB_SCL1_BIT == 28))
  if (PIN_Configure (RTE_USB_SCL1_PORT,    RTE_USB_SCL1_BIT,     IOCON_D_RST) != 0) { return -1; }
#endif
#endif
#if (RTE_USB_SDA1_PIN_EN == 1)
  // USB_SDA1
#if   ((RTE_USB_SDA1_PORT == 0) && (RTE_USB_SDA1_BIT == 27))
  if (PIN_Configure (RTE_USB_SDA1_PORT,    RTE_USB_SDA1_BIT,     IOCON_I_RST) != 0) { return -1; }
#elif ((RTE_USB_SDA1_PORT == 1) && (RTE_USB_SDA1_BIT == 29))
  if (PIN_Configure (RTE_USB_SDA1_PORT,    RTE_USB_SDA1_BIT,     IOCON_D_RST) != 0) { return -1; }
#endif
#endif
#if (RTE_USB_TX_E1_PIN_EN == 1)
  // USB_TX_E1
  if (PIN_Configure (RTE_USB_TX_E1_PORT,   RTE_USB_TX_E1_BIT,    IOCON_D_RST) != 0) { return -1; }
#endif
#if (RTE_USB_TX_DP1_PIN_EN == 1)
  // USB_TX_DP1
  if (PIN_Configure (RTE_USB_TX_DP1_PORT,  RTE_USB_TX_DP1_BIT,   IOCON_D_RST) != 0) { return -1; }
#endif
#if (RTE_USB_TX_DM1_PIN_EN == 1)
  // USB_TX_DM1
  if (PIN_Configure (RTE_USB_TX_DM1_PORT,  RTE_USB_TX_DM1_BIT,   IOCON_D_RST) != 0) { return -1; }
#endif
#if (RTE_USB_RCV1_PIN_EN == 1)
  // USB_RCV1
  if (PIN_Configure (RTE_USB_RCV1_PORT,    RTE_USB_RCV1_BIT,     IOCON_D_RST) != 0) { return -1; }
#endif
#if (RTE_USB_RX_DP1_PIN_EN == 1)
  // USB_RX_DP1
  if (PIN_Configure (RTE_USB_RX_DP1_PORT,  RTE_USB_RX_DP1_BIT,   IOCON_D_RST) != 0) { return -1; }
#endif
#if (RTE_USB_RX_DM1_PIN_EN == 1)
  // USB_RX_DM1
  if (PIN_Configure (RTE_USB_RX_DM1_PORT,  RTE_USB_RX_DM1_BIT,   IOCON_D_RST) != 0) { return -1; }
#endif
#if (RTE_USB_LS1_PIN_EN == 1)
  // USB_LS1
  if (PIN_Configure (RTE_USB_LS1_PORT,     RTE_USB_LS1_BIT,      IOCON_D_RST) != 0) { return -1; }
#endif
#if (RTE_USB_SSPND1_PIN_EN == 1)
  // USB_SSPND1
  if (PIN_Configure (RTE_USB_SSPND1_PORT,  RTE_USB_SSPND1_BIT,   IOCON_D_RST) != 0) { return -1; }
#endif
#endif
#endif

#if   (RTE_USB_PORT2_EN == 1)
  switch (U2_role) {
    // Host pins
    case ARM_USB_ROLE_HOST:
#if   (RTE_USB_PPWR2_PIN_EN == 1)
      // USB_PPWR2
      if (PIN_Configure (RTE_USB_PPWR2_PORT,  RTE_USB_PPWR2_BIT,  IOCON_A_RST) != 0) { return -1; }
#endif
#if   (RTE_USB_PWRD2_PIN_EN == 1)
      // USB_PWRD2
      if (PIN_Configure (RTE_USB_PWRD2_PORT,  RTE_USB_PWRD2_BIT,  IOCON_A_RST) != 0) { return -1; }
#endif
#if  (RTE_USB_OVRCR2_PIN_EN == 1)
      // USB_OVRCR2
      if (PIN_Configure (RTE_USB_OVRCR2_PORT, RTE_USB_OVRCR2_BIT, IOCON_A_RST) != 0) { return -1; }
#endif
      break;

    // Device pins
    case ARM_USB_ROLE_DEVICE:
#if   (RTE_USB_CONNECT2_PIN_EN == 1)
      // SoftConnect2
      if (PIN_Configure (RTE_USB_CONNECT2_PORT, RTE_USB_CONNECT2_BIT,  IOCON_D_RST) != 0) { return -1; }
#endif
#if   (RTE_USB_VBUS_PIN_EN == 1)
      // VBUS
      if (PIN_Configure (RTE_USB_VBUS_PORT,     RTE_USB_VBUS_BIT,      IOCON_A_RST) != 0) { return -1; }
#endif
      break;

    default:
      return -1;
  }

  // Common pins configuration
#if (RTE_USB_UP_LED2_PIN_EN == 1)
  // USB_UP_LED2
  if (PIN_Configure (RTE_USB_UP_LED2_PORT, RTE_USB_UP_LED2_BIT,  IOCON_A_RST) != 0) { return -1; }
#endif
#if (RTE_USB_DP2_PIN_EN == 1)
  // USB_D+2   (USB_D-2: Dedicated pin)
  if (PIN_Configure (RTE_USB_DP2_PORT,     RTE_USB_DP2_BIT,      IOCON_U_RST) != 0) { return -1; }
#endif
#endif

  usb_pin_cfg = 1U;
  return 0U;
#else

  return -1;
#endif
}
