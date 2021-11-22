/* -------------------------------------------------------------------------- 
 * Copyright (c) 2013-2017 Arm Limited (or its affiliates). All 
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
 * $Date:        20. November 2017
 * $Revision:    V2.0.0
 *
 * Project:      RTE Device Configuration for NXP LPC4000
 * -------------------------------------------------------------------------- */

//-------- <<< Use Configuration Wizard in Context Menu >>> --------------------

#ifndef __RTE_DEVICE_H
#define __RTE_DEVICE_H


// <e> USB Controller [Driver_USBD and Driver_USBH]
// <i> Configuration settings for Driver_USBD in component ::Drivers:USB Device
// <i> Configuration settings for Driver_USBH in component ::Drivers:USB Host
#define   RTE_USB_USB0                  0

//  <o> Port Functionality Configuration
//   <0=> U1 = device, U2 = host
//   <1=> U1 = host,   U2 = host
//   <3=> U1 = host,   U2 = device
#define   RTE_USB_PORT_CFG              0
#if      (RTE_USB_PORT_CFG == 2)
  #error "Invalid Port Functionality Configuration!"
#endif

//  <h> Pin Configuration
//   <e> U1 Port Pin Configuration
#define   RTE_USB_PORT1_EN              1

//     <o> USB_PPWR1 (Host) <0=>Not used <1=>P1_19
//     <i> VBUS drive signal (active low) (towards external charge pump or power management unit).
#define   RTE_USB_PPWR1_ID              0
#if      (RTE_USB_PPWR1_ID == 0)
  #define RTE_USB_PPWR1_PIN_EN          0
#elif    (RTE_USB_PPWR1_ID == 1)
  #define RTE_USB_PPWR1_PORT            1
  #define RTE_USB_PPWR1_BIT             19
  #define RTE_USB_PPWR1_FUNC            2
#else
  #error "Invalid USB_PPWR1 Pin Configuration in RTE_Device.h file!"
#endif
#ifndef   RTE_USB_PPWR1_PIN_EN
  #define RTE_USB_PPWR1_PIN_EN          1
#endif

//     <o> USB_PWRD1 (Host) <0=>Not used <1=>P1_22
//     <i> Power Status for USB port.
#define   RTE_USB_PWRD1_ID              0
#if      (RTE_USB_PWRD1_ID == 0)
  #define RTE_USB_PWRD1_PIN_EN          0
#elif    (RTE_USB_PWRD1_ID == 1)
  #define RTE_USB_PWRD1_PORT            1
  #define RTE_USB_PWRD1_BIT             22
  #define RTE_USB_PWRD1_FUNC            2
#else
  #error "Invalid USB_PWRD1 Pin Configuration in RTE_Device.h file!"
#endif
#ifndef   RTE_USB_PWRD1_PIN_EN
  #define RTE_USB_PWRD1_PIN_EN          1
#endif

//     <o> USB_OVRCR1 (Host) <0=>Not used <1=>P1_27
//     <i> Port power fault signal indicating overcurrent condition.
//     <i> This signal monitors over-current on the USB bus (active low)
//        (external circuitry required to detect over-current condition).
#define   RTE_USB_OVRCR1_ID             0
#if      (RTE_USB_OVRCR1_ID == 0)
  #define RTE_USB_OVRCR1_PIN_EN         0
#elif    (RTE_USB_OVRCR1_ID == 1)
  #define RTE_USB_OVRCR1_PORT           1
  #define RTE_USB_OVRCR1_BIT            27
  #define RTE_USB_OVRCR1_FUNC           2
#else
  #error "Invalid USB_OVRCR1 Pin Configuration in RTE_Device.h file!"
#endif
#ifndef   RTE_USB_OVRCR1_PIN_EN
  #define RTE_USB_OVRCR1_PIN_EN         1
#endif

//     <o> USB_CONNECT1 (Device) <0=>Not used <1=>P2_9
//     <i> SoftConnect control signal.
#define   RTE_USB_CONNECT1_ID           0
#if      (RTE_USB_CONNECT1_ID == 0)
  #define RTE_USB_CONNECT1_PIN_EN       0
#elif    (RTE_USB_CONNECT1_ID == 1)
  #define RTE_USB_CONNECT1_PORT         2
  #define RTE_USB_CONNECT1_BIT          9
  #define RTE_USB_CONNECT1_FUNC         1
#else
  #error "Invalid USB_CONNECT1 Pin Configuration in RTE_Device.h file!"
#endif
#ifndef   RTE_USB_CONNECT1_PIN_EN
  #define RTE_USB_CONNECT1_PIN_EN       1
#endif

//     <o> USB_UP_LED1 <0=>Not used <1=>P1_18
//     <i> GoodLink LED control signal.
#define   RTE_USB_UP_LED1_ID            1
#if      (RTE_USB_UP_LED1_ID == 0)
  #define RTE_USB_UP_LED1_PIN_EN        0
#elif    (RTE_USB_UP_LED1_ID == 1)
  #define RTE_USB_UP_LED1_PORT          1
  #define RTE_USB_UP_LED1_BIT           18
  #define RTE_USB_UP_LED1_FUNC          1
#else
  #error "Invalid USB_UP_LED1 Pin Configuration in RTE_Device.h file!"
#endif
#ifndef   RTE_USB_UP_LED1_PIN_EN
  #define RTE_USB_UP_LED1_PIN_EN        1
#endif

//     <o> USB_D+1 <0=>Not used <1=>P0_29
//     <i> USB port 1 bidirectional D+ line.
#define   RTE_USB_DP1_ID                1
#if      (RTE_USB_DP1_ID == 0)
  #define RTE_USB_DP1_PIN_EN            0
#elif    (RTE_USB_DP1_ID == 1)
  #define RTE_USB_DP1_PORT              0
  #define RTE_USB_DP1_BIT               29
  #define RTE_USB_DP1_FUNC              1
#else
  #error "Invalid USB_D+1 Pin Configuration in RTE_Device.h file!"
#endif
#ifndef   RTE_USB_DP1_PIN_EN
  #define RTE_USB_DP1_PIN_EN            1
#endif

//     <o> USB_D-1 <0=>Not used <1=>P0_30
//     <i> USB port 1 bidirectional D- line.
#define   RTE_USB_DM1_ID                1
#if      (RTE_USB_DM1_ID == 0)
  #define RTE_USB_DM1_PIN_EN            0
#elif    (RTE_USB_DM1_ID == 1)
  #define RTE_USB_DM1_PORT              0
  #define RTE_USB_DM1_BIT               30
  #define RTE_USB_DM1_FUNC              1
#else
  #error "Invalid USB_D-1 Pin Configuration in RTE_Device.h file!"
#endif
#ifndef   RTE_USB_DM1_PIN_EN
  #define RTE_USB_DM1_PIN_EN            1
#endif

//     <e> OTG Transceiver
#define   RTE_USB_PORT1_OTG_EN          1

//       <o> USB_INT1 <0=>Not used <1=>P1_27
//       <i> Interrupt (active low).
#define   RTE_USB_INT1_ID               1
#if      (RTE_USB_INT1_ID == 0)
  #define RTE_USB_INT1_PIN_EN           0
#elif    (RTE_USB_INT1_ID == 1)
  #define RTE_USB_INT1_PORT             1
  #define RTE_USB_INT1_BIT              27
  #define RTE_USB_INT1_FUNC             1
#else
  #error "Invalid USB_INT1 Pin Configuration in RTE_Device.h file!"
#endif
#ifndef   RTE_USB_INT1_PIN_EN
  #define RTE_USB_INT1_PIN_EN           1
#endif

//       <o> USB_SCL1 <0=>Not used <1=>P0_28 <2=>P1_28
//       <i> I2C serial clock.
#define   RTE_USB_SCL1_ID               2
#if      (RTE_USB_SCL1_ID == 0)
  #define RTE_USB_SCL1_PIN_EN           0
#elif    (RTE_USB_SCL1_ID == 1)
  #define RTE_USB_SCL1_PORT             0
  #define RTE_USB_SCL1_BIT              28
  #define RTE_USB_SCL1_FUNC             2
#elif    (RTE_USB_SCL1_ID == 2)
  #define RTE_USB_SCL1_PORT             1
  #define RTE_USB_SCL1_BIT              28
  #define RTE_USB_SCL1_FUNC             1
#else
  #error "Invalid USB_SCL1 Pin Configuration in RTE_Device.h file!"
#endif
#ifndef   RTE_USB_SCL1_PIN_EN
  #define RTE_USB_SCL1_PIN_EN           1
#endif

//       <o> USB_SDA1 <0=>Not used <1=>P0_27 <2=>P1_29
//       <i> I2C serial data.
#define   RTE_USB_SDA1_ID               2
#if      (RTE_USB_SDA1_ID == 0)
  #define RTE_USB_SDA1_PIN_EN           0
#elif    (RTE_USB_SDA1_ID == 1)
  #define RTE_USB_SDA1_PORT             0
  #define RTE_USB_SDA1_BIT              27
  #define RTE_USB_SDA1_FUNC             2
#elif    (RTE_USB_SDA1_ID == 2)
  #define RTE_USB_SDA1_PORT             1
  #define RTE_USB_SDA1_BIT              29
  #define RTE_USB_SDA1_FUNC             1
#else
  #error "Invalid USB_SDA1 Pin Configuration in RTE_Device.h file!"
#endif
#ifndef   RTE_USB_SDA1_PIN_EN
  #define RTE_USB_SDA1_PIN_EN           1
#endif

//       <o> USB_TX_E1 <0=>Not used <1=>P1_19
//       <i> Transmit enable (active low).
#define   RTE_USB_TX_E1_ID              0
#if      (RTE_USB_TX_E1_ID == 0)
  #define RTE_USB_TX_E1_PIN_EN          0
#elif    (RTE_USB_TX_E1_ID == 1)
  #define RTE_USB_TX_E1_PORT            1
  #define RTE_USB_TX_E1_BIT             19
  #define RTE_USB_TX_E1_FUNC            1
#else
  #error "Invalid USB_TX_E1 Pin Configuration in RTE_Device.h file!"
#endif
#ifndef   RTE_USB_TX_E1_PIN_EN
  #define RTE_USB_TX_E1_PIN_EN          1
#endif

//       <o> USB_TX_DP1 <0=>Not used <1=>P1_20
//       <i> D+ transmit data.
#define   RTE_USB_TX_DP1_ID             0
#if      (RTE_USB_TX_DP1_ID == 0)
  #define RTE_USB_TX_DP1_PIN_EN         0
#elif    (RTE_USB_TX_DP1_ID == 1)
  #define RTE_USB_TX_DP1_PORT           1
  #define RTE_USB_TX_DP1_BIT            20
  #define RTE_USB_TX_DP1_FUNC           1
#else
  #error "Invalid USB_TX_DP1 Pin Configuration in RTE_Device.h file!"
#endif
#ifndef   RTE_USB_TX_DP1_PIN_EN
  #define RTE_USB_TX_DP1_PIN_EN         1
#endif

//       <o> USB_TX_DM1 <0=>Not used <1=>P1_21
//       <i> D- transmit data.
#define   RTE_USB_TX_DM1_ID             0
#if      (RTE_USB_TX_DM1_ID == 0)
  #define RTE_USB_TX_DM1_PIN_EN         0
#elif    (RTE_USB_TX_DM1_ID == 1)
  #define RTE_USB_TX_DM1_PORT           1
  #define RTE_USB_TX_DM1_BIT            21
  #define RTE_USB_TX_DM1_FUNC           1
#else
  #error "Invalid USB_TX_DM1 Pin Configuration in RTE_Device.h file!"
#endif
#ifndef   RTE_USB_TX_DM1_PIN_EN
  #define RTE_USB_TX_DM1_PIN_EN         1
#endif

//       <o> USB_RCV1 <0=>Not used <1=>P1_22
//       <i> Differential receive data.
#define   RTE_USB_RCV1_ID               0
#if      (RTE_USB_RCV1_ID == 0)
  #define RTE_USB_RCV1_PIN_EN           0
#elif    (RTE_USB_RCV1_ID == 1)
  #define RTE_USB_RCV1_PORT             1
  #define RTE_USB_RCV1_BIT              22
  #define RTE_USB_RCV1_FUNC             1
#else
  #error "Invalid USB_RCV1 Pin Configuration in RTE_Device.h file!"
#endif
#ifndef   RTE_USB_RCV1_PIN_EN
  #define RTE_USB_RCV1_PIN_EN           1
#endif

//       <o> USB_RX_DP1 <0=>Not used <1=>P1_23
//       <i> D+ receive data.
#define   RTE_USB_RX_DP1_ID             0
#if      (RTE_USB_RX_DP1_ID == 0)
  #define RTE_USB_RX_DP1_PIN_EN         0
#elif    (RTE_USB_RX_DP1_ID == 1)
  #define RTE_USB_RX_DP1_PORT           1
  #define RTE_USB_RX_DP1_BIT            23
  #define RTE_USB_RX_DP1_FUNC           1
#else
  #error "Invalid USB_RX_DP1 Pin Configuration in RTE_Device.h file!"
#endif
#ifndef   RTE_USB_RX_DP1_PIN_EN
  #define RTE_USB_RX_DP1_PIN_EN         1
#endif

//       <o> USB_RX_DM1 <0=>Not used <1=>P1_24
//       <i> D- receive data.
#define   RTE_USB_RX_DM1_ID             0
#if      (RTE_USB_RX_DM1_ID == 0)
  #define RTE_USB_RX_DM1_PIN_EN         0
#elif    (RTE_USB_RX_DM1_ID == 1)
  #define RTE_USB_RX_DM1_PORT           1
  #define RTE_USB_RX_DM1_BIT            24
  #define RTE_USB_RX_DM1_FUNC           1
#else
  #error "Invalid USB_RX_DM1 Pin Configuration in RTE_Device.h file!"
#endif
#ifndef   RTE_USB_RX_DM1_PIN_EN
  #define RTE_USB_RX_DM1_PIN_EN         1
#endif

//       <o> USB_LS1 (Host) <0=>Not used <1=>P1_25
//       <i> Low speed status (active low).
#define   RTE_USB_LS1_ID                0
#if      (RTE_USB_LS1_ID == 0)
  #define RTE_USB_LS1_PIN_EN            0
#elif    (RTE_USB_LS1_ID == 1)
  #define RTE_USB_LS1_PORT              1
  #define RTE_USB_LS1_BIT               25
  #define RTE_USB_LS1_FUNC              1
#else
  #error "Invalid USB_LS1 Pin Configuration in RTE_Device.h file!"
#endif
#ifndef   RTE_USB_LS1_PIN_EN
  #define RTE_USB_LS1_PIN_EN            1
#endif

//       <o> USB_SSPND1 <0=>Not used <1=>P1_26
//       <i> Bus suspend status (active low).
#define   RTE_USB_SSPND1_ID             0
#if      (RTE_USB_SSPND1_ID == 0)
  #define RTE_USB_SSPND1_PIN_EN         0
#elif    (RTE_USB_SSPND1_ID == 1)
  #define RTE_USB_SSPND1_PORT           1
  #define RTE_USB_SSPND1_BIT            26
  #define RTE_USB_SSPND1_FUNC           1
#else
  #error "Invalid USB_SSPND1 Pin Configuration in RTE_Device.h file!"
#endif
#ifndef   RTE_USB_SSPND1_PIN_EN
  #define RTE_USB_SSPND1_PIN_EN         1
#endif

//     </e> OTG Transceiver
//   </e> U1 Port Pin Configuration

//   <e> U2 Port Pin Configuration
#define   RTE_USB_PORT2_EN              1

//     <o> USB_PPWR2 (Host) <0=>Not used <1=>P0_12
//     <i> VBUS drive signal (active low) (towards external charge pump or power management unit).
#define   RTE_USB_PPWR2_ID              1
#if      (RTE_USB_PPWR2_ID == 0)
  #define RTE_USB_PPWR2_PIN_EN          0
#elif    (RTE_USB_PPWR2_ID == 1)
  #define RTE_USB_PPWR2_PORT            0
  #define RTE_USB_PPWR2_BIT             12
  #define RTE_USB_PPWR2_FUNC            1
#else
  #error "Invalid USB_PPWR2 Pin Configuration in RTE_Device.h file!"
#endif
#ifndef   RTE_USB_PPWR2_PIN_EN
  #define RTE_USB_PPWR2_PIN_EN          1
#endif

//     <o> USB_PWRD2 (Host) <0=>Not used <1=>P1_30
//     <i> Power Status for USB port.
#define   RTE_USB_PWRD2_ID              1
#if      (RTE_USB_PWRD2_ID == 0)
  #define RTE_USB_PWRD2_PIN_EN          0
#elif    (RTE_USB_PWRD2_ID == 1)
  #define RTE_USB_PWRD2_PORT            1
  #define RTE_USB_PWRD2_BIT             30
  #define RTE_USB_PWRD2_FUNC            1
#else
  #error "Invalid USB_PWRD2 Pin Configuration in RTE_Device.h file!"
#endif
#ifndef   RTE_USB_PWRD2_PIN_EN
  #define RTE_USB_PWRD2_PIN_EN          1
#endif

//     <o> USB_OVRCR2 (Host) <0=>Not used <1=>P1_31
//     <i> Port power fault signal indicating overcurrent condition.
//     <i> This signal monitors over-current on the USB bus (active low)
//        (external circuitry required to detect over-current condition).
#define   RTE_USB_OVRCR2_ID             0
#if      (RTE_USB_OVRCR2_ID == 0)
  #define RTE_USB_OVRCR2_PIN_EN         0
#elif    (RTE_USB_OVRCR2_ID == 1)
  #define RTE_USB_OVRCR2_PORT           1
  #define RTE_USB_OVRCR2_BIT            31
  #define RTE_USB_OVRCR2_FUNC           1
#else
  #error "Invalid USB_OVRCR2 Pin Configuration in RTE_Device.h file!"
#endif
#ifndef   RTE_USB_OVRCR2_PIN_EN
  #define RTE_USB_OVRCR2_PIN_EN         1
#endif

//     <o> USB_CONNECT2 (Device) <0=>Not used <1=>P0_14
//     <i> SoftConnect control signal
#define   RTE_USB_CONNECT2_ID           1
#if      (RTE_USB_CONNECT2_ID == 0)
  #define RTE_USB_CONNECT2_PIN_EN       0
#elif    (RTE_USB_CONNECT2_ID == 1)
  #define RTE_USB_CONNECT2_PORT         0
  #define RTE_USB_CONNECT2_BIT          14
  #define RTE_USB_CONNECT2_FUNC         3
#else
  #error "Invalid USB_CONNECT2 Pin Configuration in RTE_Device.h file!"
#endif
#ifndef   RTE_USB_CONNECT2_PIN_EN
  #define RTE_USB_CONNECT2_PIN_EN       1
#endif

//     <o> USB_UP_LED2 <0=>Not used <1=>P0_13
//     <i> GoodLink LED control signal.
#define   RTE_USB_UP_LED2_ID            1
#if      (RTE_USB_UP_LED2_ID == 0)
  #define RTE_USB_UP_LED2_PIN_EN        0
#elif    (RTE_USB_UP_LED2_ID == 1)
  #define RTE_USB_UP_LED2_PORT          0
  #define RTE_USB_UP_LED2_BIT           13
  #define RTE_USB_UP_LED2_FUNC          1
#else
  #error "Invalid USB_UP_LED2 Pin Configuration in RTE_Device.h file!"
#endif
#ifndef   RTE_USB_UP_LED2_PIN_EN
  #define RTE_USB_UP_LED2_PIN_EN        1
#endif

//     <o> USB_D+2 <0=>Not used <1=>P0_31
//     <i> USB port 2 bidirectional D+ line.
//     <i> Dedicated pin assinged for "USB port 2 bidirectional D- line".
#define   RTE_USB_DP2_ID                1
#if      (RTE_USB_DP2_ID == 0)
  #define RTE_USB_DP2_PIN_EN            0
#elif    (RTE_USB_DP2_ID == 1)
  #define RTE_USB_DP2_PORT              0
  #define RTE_USB_DP2_BIT               31
  #define RTE_USB_DP2_FUNC              1
#else
  #error "Invalid USB_D+2 Pin Configuration in RTE_Device.h file!"
#endif
#ifndef   RTE_USB_DP2_PIN_EN
  #define RTE_USB_DP2_PIN_EN            1
#endif

//   </e> U2 Port Pin Configuration

//     <o> USB_VBUS (Device) <0=>Not used <1=>P1_30
//     <i> VBUS status input.
//     <i> When this function is not enabled via its corresponding PINSEL register, it is driven HIGH internally.
#define   RTE_USB_VBUS_ID               1
#if      (RTE_USB_VBUS_ID == 0)
  #define RTE_USB_VBUS_PIN_EN           0
#elif    (RTE_USB_VBUS_ID == 1)
  #define RTE_USB_VBUS_PORT             1
  #define RTE_USB_VBUS_BIT              30
  #define RTE_USB_VBUS_FUNC             2
#else
  #error "Invalid USB_VBUS Pin Configuration in RTE_Device.h file!"
#endif
#ifndef   RTE_USB_VBUS_PIN_EN
  #define RTE_USB_VBUS_PIN_EN           1
#endif

//  </h> Pin Configuration
// </e> USB Controller [Driver_USBD and Driver_USBH]


// <e> ENET (Ethernet Interface) [Driver_ETH_MAC0]
// <i> Configuration settings for Driver_ETH_MAC0 in component ::Drivers:Ethernet MAC
#define RTE_ENET                        0


//   <e> MII (Media Independent Interface)
#define RTE_ENET_MII                    0

//     <o> ENET_TXD0 Pin <0=>P1_0
#define RTE_ENET_MII_TXD0_PORT_ID       0
#if    (RTE_ENET_MII_TXD0_PORT_ID == 0)
#define RTE_ENET_MII_TXD0_PORT          1
#define RTE_ENET_MII_TXD0_PIN           0
#define RTE_ENET_MII_TXD0_FUNC          1
#else
#error "Invalid ENET_TXD0 Pin Configuration!"
#endif
//     <o> ENET_TXD1 Pin <0=>P1_1
#define RTE_ENET_MII_TXD1_PORT_ID       0
#if    (RTE_ENET_MII_TXD1_PORT_ID == 0)
#define RTE_ENET_MII_TXD1_PORT          1
#define RTE_ENET_MII_TXD1_PIN           1
#define RTE_ENET_MII_TXD1_FUNC          1
#else
#error "Invalid ENET_TXD1 Pin Configuration!"
#endif
//     <o> ENET_TXD2 Pin <0=>P1_2
#define RTE_ENET_MII_TXD2_PORT_ID       0
#if    (RTE_ENET_MII_TXD2_PORT_ID == 0)
#define RTE_ENET_MII_TXD2_PORT          1
#define RTE_ENET_MII_TXD2_PIN           2
#define RTE_ENET_MII_TXD2_FUNC          1
#else
#error "Invalid ENET_TXD2 Pin Configuration!"
#endif
//     <o> ENET_TXD3 Pin <0=>P1_3
#define RTE_ENET_MII_TXD3_PORT_ID       0
#if    (RTE_ENET_MII_TXD3_PORT_ID == 0)
#define RTE_ENET_MII_TXD3_PORT          1
#define RTE_ENET_MII_TXD3_PIN           3
#define RTE_ENET_MII_TXD3_FUNC          1
#else
#error "Invalid ENET_TXD3 Pin Configuration!"
#endif
//     <o> ENET_TX_ER Pin <0=>Not used <1=>P1_5
//     <i> Optional signal, rarely used
#define RTE_ENET_MII_TX_ER_PORT_ID      0
#if    (RTE_ENET_MII_TX_ER_PORT_ID == 0)
#define RTE_ENET_MII_TX_ER_PIN_EN       0
#elif  (RTE_ENET_MII_TX_ER_PORT_ID == 1)
#define RTE_ENET_MII_TX_ER_PIN_EN       1
#define RTE_ENET_MII_TX_ER_PORT         1
#define RTE_ENET_MII_TX_ER_PIN          5
#define RTE_ENET_MII_TX_ER_FUNC         1
#else
#error "Invalid ENET_TX_ER Pin Configuration!"
#endif
//     <o> ENET_TX_CLK Pin <0=>P1_6
#define RTE_ENET_MII_TX_CLK_PORT_ID     0
#if    (RTE_ENET_MII_TX_CLK_PORT_ID == 0)
#define RTE_ENET_MII_TX_CLK_PORT        1
#define RTE_ENET_MII_TX_CLK_PIN         6
#define RTE_ENET_MII_TX_CLK_FUNC        1
#else
#error "Invalid ENET_RX_CLK Pin Configuration!"
#endif
//     <o> ENET_TX_EN Pin <0=>P1_4
#define RTE_ENET_MII_TX_EN_PORT_ID      0
#if    (RTE_ENET_MII_TX_EN_PORT_ID == 0)
#define RTE_ENET_MII_TX_EN_PORT         1
#define RTE_ENET_MII_TX_EN_PIN          4
#define RTE_ENET_MII_TX_EN_FUNC         1
#else
#error "Invalid ENET_TX_EN Pin Configuration!"
#endif
//     <o> ENET_RXD0 Pin <0=>P1_9
#define RTE_ENET_MII_RXD0_PORT_ID       0
#if    (RTE_ENET_MII_RXD0_PORT_ID == 0)
#define RTE_ENET_MII_RXD0_PORT          1
#define RTE_ENET_MII_RXD0_PIN           9
#define RTE_ENET_MII_RXD0_FUNC          1
#else
#error "Invalid ENET_RXD0 Pin Configuration!"
#endif
//     <o> ENET_RXD1 Pin <0=>P1_10
#define RTE_ENET_MII_RXD1_PORT_ID       0
#if    (RTE_ENET_MII_RXD1_PORT_ID == 0)
#define RTE_ENET_MII_RXD1_PORT          1
#define RTE_ENET_MII_RXD1_PIN           10
#define RTE_ENET_MII_RXD1_FUNC          1
#else
#error "Invalid ENET_RXD1 Pin Configuration!"
#endif
//     <o> ENET_RXD2 Pin <0=>P1_11
#define RTE_ENET_MII_RXD2_PORT_ID       0
#if    (RTE_ENET_MII_RXD2_PORT_ID == 0)
#define RTE_ENET_MII_RXD2_PORT          1
#define RTE_ENET_MII_RXD2_PIN           11
#define RTE_ENET_MII_RXD2_FUNC          1
#else
#error "Invalid ENET_RXD2 Pin Configuration!"
#endif
//     <o> ENET_RXD3 Pin <0=>P1_12
#define RTE_ENET_MII_RXD3_PORT_ID       0
#if    (RTE_ENET_MII_RXD3_PORT_ID == 0)
#define RTE_ENET_MII_RXD3_PORT          1
#define RTE_ENET_MII_RXD3_PIN           12
#define RTE_ENET_MII_RXD3_FUNC          1
#else
#error "Invalid ENET_RXD3 Pin Configuration!"
#endif
//     <o> ENET_RX_DV Pin <0=>P1_13
#define RTE_ENET_MII_RX_DV_PORT_ID      0
#if    (RTE_ENET_MII_RX_DV_PORT_ID == 0)
#define RTE_ENET_MII_RX_DV_PORT         1
#define RTE_ENET_MII_RX_DV_PIN          13
#define RTE_ENET_MII_RX_DV_FUNC         1
#else
#error "Invalid ENET_RX_DV Pin Configuration!"
#endif
//     <o> ENET_RX_ER Pin <0=>P1_14
#define RTE_ENET_MII_RX_ER_PORT_ID      0
#if    (RTE_ENET_MII_RX_ER_PORT_ID == 0)
#define RTE_ENET_MII_RX_ER_PORT         1
#define RTE_ENET_MII_RX_ER_PIN          14
#define RTE_ENET_MII_RX_ER_FUNC         1
#else
#error "Invalid ENET_RX_ER Pin Configuration!"
#endif
//     <o> ENET_RX_CLK Pin <0=>P1_15
#define RTE_ENET_MII_RX_CLK_PORT_ID     0
#if    (RTE_ENET_MII_RX_CLK_PORT_ID == 0)
#define RTE_ENET_MII_RX_CLK_PORT        1
#define RTE_ENET_MII_RX_CLK_PIN         15
#define RTE_ENET_MII_RX_CLK_FUNC        1
#else
#error "Invalid ENET_RX_CLK Pin Configuration!"
#endif
//     <o> ENET_COL Pin <0=>P1_7
#define RTE_ENET_MII_COL_PORT_ID        0
#if    (RTE_ENET_MII_COL_PORT_ID == 0)
#define RTE_ENET_MII_COL_PORT           1
#define RTE_ENET_MII_COL_PIN            7
#define RTE_ENET_MII_COL_FUNC           1
#else
#error "Invalid ENET_COL Pin Configuration!"
#endif
//     <o> ENET_CRS Pin <0=>P1_8
#define RTE_ENET_MII_CRS_PORT_ID        0
#if    (RTE_ENET_MII_CRS_PORT_ID == 0)
#define RTE_ENET_MII_CRS_PORT           1
#define RTE_ENET_MII_CRS_PIN            8
#define RTE_ENET_MII_CRS_FUNC           1
#else
#error "Invalid ENET_CRS Pin Configuration!"
#endif
//   </e>

//   <e> RMII (Reduced Media Independent Interface)
#define RTE_ENET_RMII                   1

//     <o> ENET_TXD0 Pin <0=>P1_0
#define RTE_ENET_RMII_TXD0_PORT_ID      0
#if    (RTE_ENET_RMII_TXD0_PORT_ID == 0)
#define RTE_ENET_RMII_TXD0_PORT         1
#define RTE_ENET_RMII_TXD0_PIN          0
#define RTE_ENET_RMII_TXD0_FUNC         1
#else
#error "Invalid ENET_TXD0 Pin Configuration!"
#endif
//     <o> ENET_TXD1 Pin <0=>P1_1
#define RTE_ENET_RMII_TXD1_PORT_ID      0
#if    (RTE_ENET_RMII_TXD1_PORT_ID == 0)
#define RTE_ENET_RMII_TXD1_PORT         1
#define RTE_ENET_RMII_TXD1_PIN          1
#define RTE_ENET_RMII_TXD1_FUNC         1
#else
#error "Invalid ENET_TXD1 Pin Configuration!"
#endif
//     <o> ENET_REF_CLK Pin <0=>P1_15
#define RTE_ENET_RMII_REF_CLK_PORT_ID   0
#if    (RTE_ENET_RMII_REF_CLK_PORT_ID == 0)
#define RTE_ENET_RMII_REF_CLK_PORT      1
#define RTE_ENET_RMII_REF_CLK_PIN       15
#define RTE_ENET_RMII_REF_CLK_FUNC      1
#else
#error "Invalid ENET_REF_CLK Pin Configuration!"
#endif
//     <o> ENET_TX_EN Pin <0=>P1_4
#define RTE_ENET_RMII_TX_EN_PORT_ID     0
#if    (RTE_ENET_RMII_TX_EN_PORT_ID == 0)
#define RTE_ENET_RMII_TX_EN_PORT        1
#define RTE_ENET_RMII_TX_EN_PIN         4
#define RTE_ENET_RMII_TX_EN_FUNC        1
#else
#error "Invalid ENET_TX_EN Pin Configuration!"
#endif
//     <o> ENET_CRS Pin <0=>P1_8
#define RTE_ENET_RMII_CRS_PORT_ID       0
#if    (RTE_ENET_RMII_CRS_PORT_ID == 0)
#define RTE_ENET_RMII_CRS_PORT          1
#define RTE_ENET_RMII_CRS_PIN           8
#define RTE_ENET_RMII_CRS_FUNC          1
#else
#error "Invalid ENET_CRS Pin Configuration!"
#endif
//     <o> ENET_RXD0 Pin <0=>P1_9
#define RTE_ENET_RMII_RXD0_PORT_ID      0
#if    (RTE_ENET_RMII_RXD0_PORT_ID == 0)
#define RTE_ENET_RMII_RXD0_PORT         1
#define RTE_ENET_RMII_RXD0_PIN          9
#define RTE_ENET_RMII_RXD0_FUNC         1
#else
#error "Invalid ENET_RXD0 Pin Configuration!"
#endif
//     <o> ENET_RXD1 Pin <0=>P1_10
#define RTE_ENET_RMII_RXD1_PORT_ID      0
#if    (RTE_ENET_RMII_RXD1_PORT_ID == 0)
#define RTE_ENET_RMII_RXD1_PORT         1
#define RTE_ENET_RMII_RXD1_PIN          10
#define RTE_ENET_RMII_RXD1_FUNC         1
#else
#error "Invalid ENET_RXD1 Pin Configuration!"
#endif
//     <o> ENET_RX_ER Pin <0=>P1_14
#define RTE_ENET_RMII_RX_ER_PORT_ID     0
#if    (RTE_ENET_RMII_RX_ER_PORT_ID == 0)
#define RTE_ENET_RMII_RX_ER_PORT        1
#define RTE_ENET_RMII_RX_ER_PIN         14
#define RTE_ENET_RMII_RX_ER_FUNC        1
#else
#error "Invalid ENET_REF_CLK Pin Configuration!"
#endif
//   </e>

//   <h> MIIM (Management Data Interface)
//     <o> ENET_MDC Pin <0=>P1_16 <1=>P2_8
#define RTE_ENET_MDI_MDC_PORT_ID        0
#if    (RTE_ENET_MDI_MDC_PORT_ID == 0)
#define RTE_ENET_MDI_MDC_PORT           1
#define RTE_ENET_MDI_MDC_PIN            16
#define RTE_ENET_MDI_MDC_FUNC           1
#define RTE_ENET_MDI_MDC_IO_WA          1
#elif  (RTE_ENET_MDI_MDC_PORT_ID == 1)  
#define RTE_ENET_MDI_MDC_PORT           2
#define RTE_ENET_MDI_MDC_PIN            8
#define RTE_ENET_MDI_MDC_FUNC           4
#else
#error "Invalid ENET_MDC Pin Configuration!"
#endif
//     <o> ENET_MDIO Pin <0=>P1_17 <1=>P2_9
#define RTE_ENET_MDI_MDIO_PORT_ID       0
#if    (RTE_ENET_MDI_MDIO_PORT_ID == 0)
#define RTE_ENET_MDI_MDIO_PORT          1
#define RTE_ENET_MDI_MDIO_PIN           17
#define RTE_ENET_MDI_MDIO_FUNC          1
#define RTE_ENET_MDI_MDIO_IO_WA         1
#elif  (RTE_ENET_MDI_MDIO_PORT_ID == 1)
#define RTE_ENET_MDI_MDIO_PORT          2
#define RTE_ENET_MDI_MDIO_PIN           9
#define RTE_ENET_MDI_MDIO_FUNC          4
#else
#error "Invalid ENET_MDIO Pin Configuration!"
#endif
//   </h>

// </e>


// <e> I2C0 (Inter-integrated Circuit Interface 0) [Driver_I2C0]
// <i> Configuration settings for Driver_I2C0 in component ::Drivers:I2C
#define RTE_I2C0                        0

//   <o> I2C0_SCL Pin <0=>P0_28 <1=>P1_31 <2=>P5_3
#define RTE_I2C0_SCL_PORT_ID            0
#if    (RTE_I2C0_SCL_PORT_ID == 0)
#define RTE_I2C0_SCL_PORT               0
#define RTE_I2C0_SCL_PIN                28
#define RTE_I2C0_SCL_FUNC               1
#define RTE_I2C0_SCL_I2C_PAD            1
#elif  (RTE_I2C0_SCL_PORT_ID == 1)
#define RTE_I2C0_SCL_PORT               1
#define RTE_I2C0_SCL_PIN                31
#define RTE_I2C0_SCL_FUNC               4
#define RTE_I2C0_SCL_IO_WA              1
#elif  (RTE_I2C0_SCL_PORT_ID == 2)
#define RTE_I2C0_SCL_PORT               5
#define RTE_I2C0_SCL_PIN                3
#define RTE_I2C0_SCL_FUNC               5
#define RTE_I2C0_SCL_I2C_PAD            1
#else
#error "Invalid I2C0_SCL Pin Configuration!"
#endif

//   <o> I2C0_SDA Pin <0=>P0_27 <1=>P1_30 <2=>P5_2
#define RTE_I2C0_SDA_PORT_ID            0
#if    (RTE_I2C0_SDA_PORT_ID == 0)
#define RTE_I2C0_SDA_PORT               0
#define RTE_I2C0_SDA_PIN                27
#define RTE_I2C0_SDA_FUNC               1
#define RTE_I2C0_SDA_I2C_PAD            1
#elif  (RTE_I2C0_SDA_PORT_ID == 1)
#define RTE_I2C0_SDA_PORT               1
#define RTE_I2C0_SDA_PIN                30
#define RTE_I2C0_SDA_FUNC               4
#define RTE_I2C0_SDA_IO_WA              1
#elif  (RTE_I2C0_SDA_PORT_ID == 2)
#define RTE_I2C0_SDA_PORT               5
#define RTE_I2C0_SDA_PIN                2
#define RTE_I2C0_SDA_FUNC               5
#define RTE_I2C0_SDA_I2C_PAD            1
#else
#error "Invalid I2C0_SDA Pin Configuration!"
#endif

// </e>


// <e> I2C1 (Inter-integrated Circuit Interface 1) [Driver_I2C1]
// <i> Configuration settings for Driver_I2C1 in component ::Drivers:I2C
#define RTE_I2C1                        0

//   <o> I2C1_SCL Pin <0=>P0_1 <1=>P0_20 <2=>P2_15
#define RTE_I2C1_SCL_PORT_ID            0
#if    (RTE_I2C1_SCL_PORT_ID == 0)
#define RTE_I2C1_SCL_PORT               0
#define RTE_I2C1_SCL_PIN                1
#define RTE_I2C1_SCL_FUNC               3
#elif  (RTE_I2C1_SCL_PORT_ID == 1)
#define RTE_I2C1_SCL_PORT               0
#define RTE_I2C1_SCL_PIN                20
#define RTE_I2C1_SCL_FUNC               3
#elif  (RTE_I2C1_SCL_PORT_ID == 2)
#define RTE_I2C1_SCL_PORT               2
#define RTE_I2C1_SCL_PIN                15
#define RTE_I2C1_SCL_FUNC               2
#else
#error "Invalid I2C1_SCL Pin Configuration!"
#endif

//   <o> I2C1_SDA Pin <0=>P0_0 <1=>P0_19 <2=>P2_14
#define RTE_I2C1_SDA_PORT_ID            0
#if    (RTE_I2C1_SDA_PORT_ID == 0)
#define RTE_I2C1_SDA_PORT               0
#define RTE_I2C1_SDA_PIN                0
#define RTE_I2C1_SDA_FUNC               3
#elif  (RTE_I2C1_SDA_PORT_ID == 1)
#define RTE_I2C1_SDA_PORT               0
#define RTE_I2C1_SDA_PIN                19
#define RTE_I2C1_SDA_FUNC               3
#elif  (RTE_I2C1_SDA_PORT_ID == 2)
#define RTE_I2C1_SDA_PORT               2
#define RTE_I2C1_SDA_PIN                14
#define RTE_I2C1_SDA_FUNC               2
#else
#error "Invalid I2C1_SDA Pin Configuration!"
#endif

// </e>


// <e> I2C2 (Inter-integrated Circuit Interface 2) [Driver_I2C2]
// <i> Configuration settings for Driver_I2C2 in component ::Drivers:I2C
#define RTE_I2C2                        0

//   <o> I2C2_SCL Pin <0=>P0_11 <1=>P2_31 <2=>P4_21 <3=>P4_29
#define RTE_I2C2_SCL_PORT_ID            0
#if    (RTE_I2C2_SCL_PORT_ID == 0)
#define RTE_I2C2_SCL_PORT               0
#define RTE_I2C2_SCL_PIN                11
#define RTE_I2C2_SCL_FUNC               2
#elif  (RTE_I2C2_SCL_PORT_ID == 1)
#define RTE_I2C2_SCL_PORT               2
#define RTE_I2C2_SCL_PIN                31
#define RTE_I2C2_SCL_FUNC               2
#elif  (RTE_I2C2_SCL_PORT_ID == 2)
#define RTE_I2C2_SCL_PORT               4
#define RTE_I2C2_SCL_PIN                21
#define RTE_I2C2_SCL_FUNC               2
#elif  (RTE_I2C2_SCL_PORT_ID == 3)
#define RTE_I2C2_SCL_PORT               4
#define RTE_I2C2_SCL_PIN                29
#define RTE_I2C2_SCL_FUNC               4
#else
#error "Invalid I2C2_SCL Pin Configuration!"
#endif

//   <o> I2C2_SDA Pin <0=>P0_10 <1=>P1_15 <2=>P2_30 <3=>P4_20
#define RTE_I2C2_SDA_PORT_ID            0
#if    (RTE_I2C2_SDA_PORT_ID == 0)
#define RTE_I2C2_SDA_PORT               0
#define RTE_I2C2_SDA_PIN                10
#define RTE_I2C2_SDA_FUNC               2
#elif  (RTE_I2C2_SDA_PORT_ID == 1)
#define RTE_I2C2_SDA_PORT               1
#define RTE_I2C2_SDA_PIN                15
#define RTE_I2C2_SDA_FUNC               3
#elif  (RTE_I2C2_SDA_PORT_ID == 2)
#define RTE_I2C2_SDA_PORT               2
#define RTE_I2C2_SDA_PIN                30
#define RTE_I2C2_SDA_FUNC               2
#elif  (RTE_I2C2_SDA_PORT_ID == 3)
#define RTE_I2C2_SDA_PORT               4
#define RTE_I2C2_SDA_PIN                20
#define RTE_I2C2_SDA_FUNC               2
#else
#error "Invalid I2C2_SDA Pin Configuration!"
#endif

// </e>


// <e> MCI (SD card interface) [Driver_MCI0]
// <i> Configuration settings for Driver_MCI0 in component ::CMSIS Driver:MCI
#define RTE_SD                          0

//   <h> SD Peripheral Bus
//     <o> SD_CLK Pin <0=>P0[19] <1=>P1[2]
#define   RTE_SD_CLK_PORT_ID            0
#if      (RTE_SD_CLK_PORT_ID == 0)
  #define RTE_SD_CLK_PORT               0
  #define RTE_SD_CLK_PIN                19
#elif    (RTE_SD_CLK_PORT_ID == 1)
  #define RTE_SD_CLK_PORT               1
  #define RTE_SD_CLK_PIN                2
#else
  #error "Invalid SD_CLK Pin Configuration!"
#endif
//     <o> SD_CMD Pin <0=>P0[20] <1=>P1[3]
#define   RTE_SD_CMD_PORT_ID            0
#if      (RTE_SD_CMD_PORT_ID == 0)
  #define RTE_SD_CMD_PORT               0
  #define RTE_SD_CMD_PIN                20
#elif    (RTE_SD_CMD_PORT_ID == 1)
  #define RTE_SD_CMD_PORT               1
  #define RTE_SD_CMD_PIN                3
#else
  #error "Invalid SD_CMD Pin Configuration!"
#endif
//     <o> SD_DAT0 Pin <0=>P0[22] <1=>P1[6]
#define   RTE_SD_DAT0_PORT_ID           0
#if      (RTE_SD_DAT0_PORT_ID == 0)
  #define RTE_SD_DAT0_PORT              0
  #define RTE_SD_DAT0_PIN               22
#elif    (RTE_SD_DAT0_PORT_ID == 1)
  #define RTE_SD_DAT0_PORT              1
  #define RTE_SD_DAT0_PIN               6
  #define RTE_SD_DAT0_IO_WA             1
#else
  #error "Invalid SD_DAT0 Pin Configuration!"
#endif
//     <e> SD_DAT[1 .. 3]
#define   RTE_SD_BUS_WIDTH_4            1
//       <o> SD_DAT1 Pin <0=>P1[7] <1=>P2[11]
#define   RTE_SD_DAT1_PORT_ID           0
#if      (RTE_SD_DAT1_PORT_ID == 0)
  #define RTE_SD_DAT1_PORT              1
  #define RTE_SD_DAT1_PIN               7
  #define RTE_SD_DAT1_IO_WA             1
#elif    (RTE_SD_DAT1_PORT_ID == 1)
  #define RTE_SD_DAT1_PORT              2
  #define RTE_SD_DAT1_PIN               11
#else
  #error "Invalid SD_DAT1 Pin Configuration!"
#endif
//       <o> SD_DAT2 Pin <0=>P1[11] <1=>P2[12]
#define   RTE_SD_DAT2_PORT_ID           0
#if      (RTE_SD_DAT2_PORT_ID == 0)
  #define RTE_SD_DAT2_PORT              1
  #define RTE_SD_DAT2_PIN               11
#elif    (RTE_SD_DAT2_PORT_ID == 1)
  #define RTE_SD_DAT2_PORT              2
  #define RTE_SD_DAT2_PIN               12
#else
  #error "Invalid SD_DAT2 Pin Configuration!"
#endif
//       <o> SD_DAT3 Pin <0=>P1[12] <1=>P2[13]
#define   RTE_SD_DAT3_PORT_ID           0
#if      (RTE_SD_DAT3_PORT_ID == 0)
  #define RTE_SD_DAT3_PORT              1
  #define RTE_SD_DAT3_PIN               12
#elif    (RTE_SD_DAT3_PORT_ID == 1)
  #define RTE_SD_DAT3_PORT              2
  #define RTE_SD_DAT3_PIN               13
#else
  #error "Invalid SD_DAT3 Pin Configuration!"
#endif
//     </e> SD_DAT[1 .. 3]
//   </h> SD Peripheral Bus

//   <e> Card Detect Pin
//   <i> Configure Pin if exists
//   <i> Port Px[y] (x = 0..5, y = 0..31)
//     <o1> Port <0=>0 <1=>1 <2=>2 <3=>3 <4=>4 <5=>5
//     <i>  Selects Port Number
//     <o2> Pin <0-31>
//     <i>  Selects Port Pin
//     <o3> Active State <0=>Low <1=>High
//     <i>  Selects Active State Logical Level
//   </e>
#define RTE_SD_CD_PIN_EN                0
#define RTE_SD_CD_PORT                  0
#define RTE_SD_CD_PIN                   0
#define RTE_SD_CD_ACTIVE                0

//   <e> Write Protect Pin
//   <i> Port Px[y] (x = 0..5, y = 0..31)
//     <o1> Port <0=>0 <1=>1 <2=>2 <3=>3 <4=>4 <5=>5
//     <i>  Selects Port Number
//     <o2> Bit <0-31>
//     <i>  Selects Port Pin
//     <o3> Active State <0=>Low <1=>High
//     <i>  Selects Active State Logical Level
//   </e>
#define RTE_SD_WP_PIN_EN                0
#define RTE_SD_WP_PORT                  0
#define RTE_SD_WP_PIN                   1
#define RTE_SD_WP_ACTIVE                1

//   <e> Power Supply Enable Pin
//   <i> Configure SD_PWR Pin if exists
//     <o1> SD_PWR Pin <0=>P0[21] <1=>P1[5]
//     <o2> Active State <0=>Low <1=>High
//     <i>  Selects Active State Logical Level
//   </e>
#define RTE_SD_PWR_PIN_EN               0
#define RTE_SD_PWR_PORT_ID              0
#define RTE_SD_PWR_ACTIVE               0

#if      (RTE_SD_PWR_PORT_ID == 0)
  #define RTE_SD_PWR_PORT               0
  #define RTE_SD_PWR_PIN                21
#elif    (RTE_SD_PWR_PORT_ID == 1)
  #define RTE_SD_PWR_PORT               1
  #define RTE_SD_PWR_PIN                5
  #define RTE_SD_PWR_IO_WA              1
#else
  #error "Invalid SD_PWR Pin Configuration!"
#endif

//   <o> DMA Channel <0=>0 <1=>1 <2=>2 <3=>3 <4=>4 <5=>5 <6=>6 <7=>7
//   <i> Bidirectional DMA channel is mandatory for SD card data transfers.
#define   RTE_SD_DMA_CH                 0

// </e>

// <e> UART0 (Universal asynchronous receiver transmitter)
#define RTE_UART0                       0

//   <o> U0_TXD Pin <0=>Not used <1=>P0_0 <2=>P0_2
//   <i> UART0 Serial Output pin
#define RTE_UART0_TX_ID                 0
#if    (RTE_UART0_TX_ID == 0)
#define RTE_UART0_TX_PIN_EN             0
#elif  (RTE_UART0_TX_ID == 1)
#define RTE_UART0_TX_PORT               0
#define RTE_UART0_TX_BIT                0
#define RTE_UART0_TX_FUNC               4
#elif  (RTE_UART0_TX_ID == 2)
#define RTE_UART0_TX_PORT               0
#define RTE_UART0_TX_BIT                2
#define RTE_UART0_TX_FUNC               1
#else
#error "Invalid UART0_TX Pin Configuration!"
#endif
#ifndef RTE_UART0_TX_PIN_EN
#define RTE_UART0_TX_PIN_EN             1
#endif

//   <o> U0_RXD Pin <0=>Not used <1=>P0_1 <2=>P0_3
//   <i> UART0 Serial Input pin
#define RTE_UART0_RX_ID                 0
#if    (RTE_UART0_RX_ID == 0)
#define RTE_UART0_RX_PIN_EN             0
#elif  (RTE_UART0_RX_ID == 1)
#define RTE_UART0_RX_PORT               0
#define RTE_UART0_RX_BIT                1
#define RTE_UART0_RX_FUNC               4
#elif  (RTE_UART0_RX_ID == 2)
#define RTE_UART0_RX_PORT               0
#define RTE_UART0_RX_BIT                3
#define RTE_UART0_RX_FUNC               1
#else
#error "Invalid UART0_RX Pin Configuration!"
#endif
#ifndef RTE_UART0_RX_PIN_EN
#define RTE_UART0_RX_PIN_EN             1
#endif

//   <h> DMA
//     <e> Tx
//       <o1> Channel     <0=>0 <1=>1 <2=>2 <3=>3 <4=>4 <5=>5 <6=>6 <7=>7
//     </e>
#define   RTE_UART0_DMA_TX_EN           1
#define   RTE_UART0_DMA_TX_CH           0
//     <e> Rx
//       <o1> Channel    <0=>0 <1=>1 <2=>2 <3=>3 <4=>4 <5=>5 <6=>6 <7=>7
//     </e>
#define   RTE_UART0_DMA_RX_EN           1
#define   RTE_UART0_DMA_RX_CH           1
//   </h> DMA

//     </e>
// <e> UART1 (Universal asynchronous receiver transmitter)
#define RTE_UART1                       0

//   <o> U1_TXD Pin <0=>Not used <1=>P0_15 <2=>P2_0 <3=>P3_16
//   <i> UART1 Serial Output pin
#define RTE_UART1_TX_ID                 1
#if    (RTE_UART1_TX_ID == 0)
#define RTE_UART1_TX_PIN_EN             0
#elif  (RTE_UART1_TX_ID == 1)
#define RTE_UART1_TX_PORT               0
#define RTE_UART1_TX_BIT                15
#define RTE_UART1_TX_FUNC               1
#elif  (RTE_UART1_TX_ID == 2)
#define RTE_UART1_TX_PORT               2
#define RTE_UART1_TX_BIT                0
#define RTE_UART1_TX_FUNC               2
#elif  (RTE_UART1_TX_ID == 3)
#define RTE_UART1_TX_PORT               3
#define RTE_UART1_TX_BIT                16
#define RTE_UART1_TX_FUNC               3
#else
#error "Invalid U1_TX Pin Configuration!"
#endif
#ifndef RTE_UART1_TX_PIN_EN
#define RTE_UART1_TX_PIN_EN             1
#endif

//   <o> U1_RXD Pin <0=>Not used <1=>P0_16 <2=>P2_1 <3=>P3_17
//   <i> UART1 Serial Input pin
#define RTE_UART1_RX_ID                 1
#if    (RTE_UART1_RX_ID == 0)
#define RTE_UART1_RX_PIN_EN             0
#elif  (RTE_UART1_RX_ID == 1)
#define RTE_UART1_RX_PORT               0
#define RTE_UART1_RX_BIT                16
#define RTE_UART1_RX_FUNC               1
#elif  (RTE_UART1_RX_ID == 2)
#define RTE_UART1_RX_PORT               2
#define RTE_UART1_RX_BIT                1
#define RTE_UART1_RX_FUNC               2
#elif  (RTE_UART1_RX_ID == 3)
#define RTE_UART1_RX_PORT               3
#define RTE_UART1_RX_BIT                17
#define RTE_UART1_RX_FUNC               3
#else
#error "Invalid U1_RX Pin Configuration!"
#endif
#ifndef RTE_UART1_RX_PIN_EN
#define RTE_UART1_RX_PIN_EN             1
#endif

//     <h> Modem Lines
//      <o> U1_CTS <0=>Not used <1=>P0_17 <2=>P2_2 <3=>P2_8 <4=>P3_18
#define RTE_UART1_CTS_ID                0
#if    (RTE_UART1_CTS_ID == 0)
#define RTE_UART1_CTS_PIN_EN            0
#elif  (RTE_UART1_CTS_ID == 1)
#define RTE_UART1_CTS_PORT              0
#define RTE_UART1_CTS_BIT               17
#define RTE_UART1_CTS_FUNC              1
#elif  (RTE_UART1_CTS_ID == 2)
#define RTE_UART1_CTS_PORT              2
#define RTE_UART1_CTS_BIT               2
#define RTE_UART1_CTS_FUNC              2
#elif  (RTE_UART1_CTS_ID == 3)
#define RTE_UART1_CTS_PORT              2
#define RTE_UART1_CTS_BIT               8
#define RTE_UART1_CTS_FUNC              3
#elif  (RTE_UART1_CTS_ID == 4)
#define RTE_UART1_CTS_PORT              3
#define RTE_UART1_CTS_BIT               18
#define RTE_UART1_CTS_FUNC              3
#else
#error "Invalid U1_CTS Pin Configuration!"
#endif
#ifndef RTE_UART1_CTS_PIN_EN
#define RTE_UART1_CTS_PIN_EN            1
#endif

//       
//      <o> U1_DCD <0=>Not used <1=>P0_18 <2=>P2_3 <3=>P3_19
#define RTE_UART1_DCD_ID                0
#if    (RTE_UART1_DCD_ID == 0)
#define RTE_UART1_DCD_PIN_EN            0
#elif  (RTE_UART1_DCD_ID == 1)
#define RTE_UART1_DCD_PORT              0
#define RTE_UART1_DCD_BIT               18
#define RTE_UART1_DCD_FUNC              1
#elif  (RTE_UART1_DCD_ID == 2)
#define RTE_UART1_DCD_PORT              2
#define RTE_UART1_DCD_BIT               3
#define RTE_UART1_DCD_FUNC              2
#elif  (RTE_UART1_DCD_ID == 3)
#define RTE_UART1_DCD_PORT              3
#define RTE_UART1_DCD_BIT               19
#define RTE_UART1_DCD_FUNC              3
#else
#error "Invalid U1_DCD Pin Configuration!"
#endif
#ifndef RTE_UART1_DCD_PIN_EN
#define RTE_UART1_DCD_PIN_EN            1
#endif

//      <o> DSR <0=>Not used <1=>P0_19  <2=>P2_4  <3=>P3_20
#define RTE_UART1_DSR_ID                0
#if    (RTE_UART1_DSR_ID == 0)
#define RTE_UART1_DSR_PIN_EN            0
#elif  (RTE_UART1_DSR_ID == 1)
#define RTE_UART1_DSR_PORT              0
#define RTE_UART1_DSR_BIT               19
#define RTE_UART1_DSR_FUNC              1
#elif  (RTE_UART1_DSR_ID == 2)
#define RTE_UART1_DSR_PORT              2
#define RTE_UART1_DSR_BIT               4
#define RTE_UART1_DSR_FUNC              2
#elif  (RTE_UART1_DSR_ID == 3)
#define RTE_UART1_DSR_PORT              3
#define RTE_UART1_DSR_BIT               20
#define RTE_UART1_DSR_FUNC              3
#else
#error "Invalid U1_DSR Pin Configuration!"
#endif
#ifndef RTE_UART1_DSR_PIN_EN
#define RTE_UART1_DSR_PIN_EN            1
#endif

//      <o> U1_DTR <0=>Not used <1=>P0_20  <2=>P2_5  <3=>P3_21
#define RTE_UART1_DTR_ID                0
#if    (RTE_UART1_DTR_ID == 0)
#define RTE_UART1_DTR_PIN_EN            0
#elif  (RTE_UART1_DTR_ID == 1)
#define RTE_UART1_DTR_PORT              0
#define RTE_UART1_DTR_BIT               20
#define RTE_UART1_DTR_FUNC              1
#elif  (RTE_UART1_DTR_ID == 2)
#define RTE_UART1_DTR_PORT              2
#define RTE_UART1_DTR_BIT               5
#define RTE_UART1_DTR_FUNC              2
#elif  (RTE_UART1_DTR_ID == 3)
#define RTE_UART1_DTR_PORT              3
#define RTE_UART1_DTR_BIT               21
#define RTE_UART1_DTR_FUNC              3
#else
#error "Invalid U1_DTR Pin Configuration!"
#endif
#ifndef RTE_UART1_DTR_PIN_EN
#define RTE_UART1_DTR_PIN_EN            1
#endif

//       <o> U1_RI <0=>Not used <1=>P0_21 <2=>P2_6 <3=>P3_22
#define RTE_UART1_RI_ID                 0
#if    (RTE_UART1_RI_ID == 0)
#define RTE_UART1_RI_PIN_EN             0
#elif  (RTE_UART1_RI_ID == 1)
#define RTE_UART1_RI_PORT               0
#define RTE_UART1_RI_BIT                21
#define RTE_UART1_RI_FUNC               1
#elif  (RTE_UART1_RI_ID == 2)
#define RTE_UART1_RI_PORT               2
#define RTE_UART1_RI_BIT                6
#define RTE_UART1_RI_FUNC               2
#elif  (RTE_UART1_RI_ID == 3)
#define RTE_UART1_RI_PORT               3
#define RTE_UART1_RI_BIT                22
#define RTE_UART1_RI_FUNC               3
#else
#error "Invalid U1_RI Pin Configuration!"
#endif
#ifndef RTE_UART1_RI_PIN_EN
#define RTE_UART1_RI_PIN_EN             1
#endif

//       <o> U1_RTS <0=>Not used <1=>P0_6 <2=>P0_22 <3=>P2_7 <4=>P3_30
#define RTE_UART1_RTS_ID                0
#if    (RTE_UART1_RTS_ID == 0)
#define RTE_UART1_RTS_PIN_EN            0
#elif  (RTE_UART1_RTS_ID == 1)
#define RTE_UART1_RTS_PORT              0
#define RTE_UART1_RTS_BIT               6
#define RTE_UART1_RTS_FUNC              4
#elif  (RTE_UART1_RTS_ID == 2)
#define RTE_UART1_RTS_PORT              0
#define RTE_UART1_RTS_BIT               22
#define RTE_UART1_RTS_FUNC              1
#elif  (RTE_UART1_RTS_ID == 3)
#define RTE_UART1_RTS_PORT              2
#define RTE_UART1_RTS_BIT               7
#define RTE_UART1_RTS_FUNC              2
#elif  (RTE_UART1_RTS_ID == 4)
#define RTE_UART1_RTS_PORT              3
#define RTE_UART1_RTS_BIT               30
#define RTE_UART1_RTS_FUNC              2
#else
#error "Invalid U1_RTS Pin Configuration!"
#endif
#ifndef RTE_UART1_RTS_PIN_EN
#define RTE_UART1_RTS_PIN_EN            1
#endif

//     </h>

//   <h> DMA
//     <e> Tx
//       <o1> Channel     <0=>0 <1=>1 <2=>2 <3=>3 <4=>4 <5=>5 <6=>6 <7=>7
//     </e>
#define   RTE_UART1_DMA_TX_EN           1
#define   RTE_UART1_DMA_TX_CH           0
//     <e> Rx
//       <o1> Channel    <0=>0 <1=>1 <2=>2 <3=>3 <4=>4 <5=>5 <6=>6 <7=>7
//     </e>
#define   RTE_UART1_DMA_RX_EN           1
#define   RTE_UART1_DMA_RX_CH           1
//   </h> DMA

// </e>

// <e> UART2 (Universal asynchronous receiver transmitter)
#define RTE_UART2                       0

//   <o> U2_TXD Pin <0=>Not used <1=>P0_10 <2=>P2_8 <3=>P4_22
//   <i> UART2 Serial Output pin
#define RTE_UART2_TX_ID                 0
#if    (RTE_UART2_TX_ID == 0)
#define RTE_UART2_TX_PIN_EN             0
#elif  (RTE_UART2_TX_ID == 1)
#define RTE_UART2_TX_PORT               0
#define RTE_UART2_TX_BIT                10
#define RTE_UART2_TX_FUNC               1
#elif  (RTE_UART2_TX_ID == 2)
#define RTE_UART2_TX_PORT               2
#define RTE_UART2_TX_BIT                8
#define RTE_UART2_TX_FUNC               2
#elif  (RTE_UART2_TX_ID == 3)
#define RTE_UART2_TX_PORT               4
#define RTE_UART2_TX_BIT                22
#define RTE_UART2_TX_FUNC               2
#else
#error "Invalid UART2_TX Pin Configuration!"
#endif
#ifndef RTE_UART2_TX_PIN_EN
#define RTE_UART2_TX_PIN_EN             1
#endif

//   <o> U2_RXD Pin <0=>Not used <1=>P0_11 <2=>P2_9 <3=>P4_23
//   <i> UART2 Serial Input pin
#define RTE_UART2_RX_ID                 0
#if    (RTE_UART2_RX_ID == 0)
#define RTE_UART2_RX_PIN_EN             0
#elif  (RTE_UART2_RX_ID == 1)
#define RTE_UART2_RX_PORT               0
#define RTE_UART2_RX_BIT                11
#define RTE_UART2_RX_FUNC               1
#elif  (RTE_UART2_RX_ID == 2)
#define RTE_UART2_RX_PORT               2
#define RTE_UART2_RX_BIT                9
#define RTE_UART2_RX_FUNC               2
#elif  (RTE_UART2_RX_ID == 3)
#define RTE_UART2_RX_PORT               4
#define RTE_UART2_RX_BIT                23
#define RTE_UART2_RX_FUNC               2
#else
#error "Invalid UART2_RX Pin Configuration!"
#endif
#ifndef RTE_UART2_RX_PIN_EN
#define RTE_UART2_RX_PIN_EN             1
#endif

//   <h> DMA
//     <e> Tx
//       <o1> Channel     <0=>0 <1=>1 <2=>2 <3=>3 <4=>4 <5=>5 <6=>6 <7=>7
//     </e>
#define   RTE_UART2_DMA_TX_EN           1
#define   RTE_UART2_DMA_TX_CH           0
//     <e> Rx
//       <o1> Channel    <0=>0 <1=>1 <2=>2 <3=>3 <4=>4 <5=>5 <6=>6 <7=>7
//     </e>
#define   RTE_UART2_DMA_RX_EN           1
#define   RTE_UART2_DMA_RX_CH           1
//   </h> DMA

//     </e>

// <e> UART3 (Universal asynchronous receiver transmitter)
#define RTE_UART3                       0

//   <o> U3_TXD Pin <0=>Not used <1=>P0_0 <2=>P0_2 <3=>P0_25 <4=>P4_28
//   <i> UART3 Serial Output pin
#define RTE_UART3_TX_ID                 0
#if    (RTE_UART3_TX_ID == 0)
#define RTE_UART3_TX_PIN_EN             0
#elif  (RTE_UART3_TX_ID == 1)
#define RTE_UART3_TX_PORT               0
#define RTE_UART3_TX_BIT                0
#define RTE_UART3_TX_FUNC               2
#elif  (RTE_UART3_TX_ID == 2)
#define RTE_UART3_TX_PORT               0
#define RTE_UART3_TX_BIT                2
#define RTE_UART3_TX_FUNC               2
#elif  (RTE_UART3_TX_ID == 3)
#define RTE_UART3_TX_PORT               0
#define RTE_UART3_TX_BIT                25
#define RTE_UART3_TX_FUNC               3
#define RTE_UART3_TX_IO_WA              1
#elif  (RTE_UART3_TX_ID == 4)
#define RTE_UART3_TX_PORT               4
#define RTE_UART3_TX_BIT                28
#define RTE_UART3_TX_FUNC               2
#else
#error "Invalid UART3_TX Pin Configuration!"
#endif
#ifndef RTE_UART3_TX_PIN_EN
#define RTE_UART3_TX_PIN_EN             1
#endif

//   <o> U3_RXD Pin <0=>Not used <1=>P0_1 <2=>P0_3 <3=>P0_26 <4=>P4_29
//   <i> UART3 Serial Input pin
#define RTE_UART3_RX_ID                 0
#if    (RTE_UART3_RX_ID == 0)
#define RTE_UART3_RX_PIN_EN             0
#elif  (RTE_UART3_RX_ID == 1)
#define RTE_UART3_RX_PORT               0
#define RTE_UART3_RX_BIT                1
#define RTE_UART3_RX_FUNC               2
#elif  (RTE_UART3_RX_ID == 2)
#define RTE_UART3_RX_PORT               0
#define RTE_UART3_RX_BIT                3
#define RTE_UART3_RX_FUNC               2
#elif  (RTE_UART3_RX_ID == 3)
#define RTE_UART3_RX_PORT               0
#define RTE_UART3_RX_BIT                26
#define RTE_UART3_RX_FUNC               3
#define RTE_UART3_RX_IO_WA              1
#elif  (RTE_UART3_RX_ID == 4)
#define RTE_UART3_RX_PORT               4
#define RTE_UART3_RX_BIT                29
#define RTE_UART3_RX_FUNC               2
#else
#error "Invalid UART3_RX Pin Configuration!"
#endif
#ifndef RTE_UART3_RX_PIN_EN
#define RTE_UART3_RX_PIN_EN             1
#endif

//   <h> DMA
//     <e> Tx
//       <o1> Channel     <0=>0 <1=>1 <2=>2 <3=>3 <4=>4 <5=>5 <6=>6 <7=>7
//     </e>
#define   RTE_UART3_DMA_TX_EN           1
#define   RTE_UART3_DMA_TX_CH           0
//     <e> Rx
//       <o1> Channel    <0=>0 <1=>1 <2=>2 <3=>3 <4=>4 <5=>5 <6=>6 <7=>7
//     </e>
#define   RTE_UART3_DMA_RX_EN           1
#define   RTE_UART3_DMA_RX_CH           1
//   </h> DMA

// </e>

// <e> UART4 (Universal asynchronous receiver transmitter)
#define RTE_UART4                       0

//   <o> U4_TXD Pin <0=>Not used <1=>P0_22 <2=>P1_29 <3=>P5_4
//   <i> UART4 Serial Output pin
#define RTE_UART4_TX_ID                 0
#if    (RTE_UART4_TX_ID == 0)
#define RTE_UART4_TX_PIN_EN             0
#elif  (RTE_UART4_TX_ID == 1)
#define RTE_UART4_TX_PORT               0
#define RTE_UART4_TX_BIT                22
#define RTE_UART4_TX_FUNC               3
#elif  (RTE_UART4_TX_ID == 2)
#define RTE_UART4_TX_PORT               1
#define RTE_UART4_TX_BIT                29
#define RTE_UART4_TX_FUNC               5
#elif  (RTE_UART4_TX_ID == 3)
#define RTE_UART4_TX_PORT               5
#define RTE_UART4_TX_BIT                4
#define RTE_UART4_TX_FUNC               4
#else
#error "Invalid UART4_TX Pin Configuration!"
#endif
#ifndef RTE_UART4_TX_PIN_EN
#define RTE_UART4_TX_PIN_EN             1
#endif

//   <o> U4_RXD Pin <0=>Not used <1=>P2_9 <2=>P5_3
//   <i> UART4 Serial Input pin
#define RTE_UART4_RX_ID                 0
#if    (RTE_UART4_RX_ID == 0)
#define RTE_UART4_RX_PIN_EN             0
#elif  (RTE_UART4_RX_ID == 1)
#define RTE_UART4_RX_PORT               2
#define RTE_UART4_RX_BIT                9
#define RTE_UART4_RX_FUNC               3
#elif  (RTE_UART4_RX_ID == 2)
#define RTE_UART4_RX_PORT               5
#define RTE_UART4_RX_BIT                3
#define RTE_UART4_RX_FUNC               4
#else
#error "Invalid UART4_RX Pin Configuration!"
#endif
#ifndef RTE_UART4_RX_PIN_EN
#define RTE_UART4_RX_PIN_EN             1
#endif

//   <o> U4_OE Pin <0=>Not used <1=>P0_21
//   <i> UART4 Output Enable pin
#define RTE_UART4_OE_ID                 0
#if    (RTE_UART4_OE_ID == 0)
#define RTE_UART4_OE_PIN_EN             0
#elif  (RTE_UART4_OE_ID == 1)
#define RTE_UART4_OE_PORT               0
#define RTE_UART4_OE_BIT                21
#define RTE_UART4_OE_FUNC               3
#else
#error "Invalid UART4_OE Pin Configuration!"
#endif
#ifndef   RTE_UART4_OE_PIN_EN
#define RTE_UART4_OE_PIN_EN             1
#endif

//   <o> U4_SCLK Pin <0=>Not used <1=>P0_21
//   <i> UART4 Serial Clock pin
#define RTE_UART4_SCLK_ID               0
#if    (RTE_UART4_SCLK_ID == 0)
#define RTE_UART4_SCLK_PIN_EN           0
#elif  (RTE_UART4_SCLK_ID == 1)
#define RTE_UART4_SCLK_PORT             0
#define RTE_UART4_SCLK_BIT              21
#define RTE_UART4_SCLK_FUNC             5
#else
#error "Invalid UART4_SCLK Pin Configuration!"
#endif
#ifndef   RTE_UART4_SCLK_PIN_EN
#define RTE_UART4_SCLK_PIN_EN           1
#endif

//   <h> DMA
//     <e> Tx
//       <o1> Channel     <0=>0 <1=>1 <2=>2 <3=>3 <4=>4 <5=>5 <6=>6 <7=>7
//     </e>
#define   RTE_UART4_DMA_TX_EN           1
#define   RTE_UART4_DMA_TX_CH           0
//     <e> Rx
//       <o1> Channel    <0=>0 <1=>1 <2=>2 <3=>3 <4=>4 <5=>5 <6=>6 <7=>7
//     </e>
#define   RTE_UART4_DMA_RX_EN           1
#define   RTE_UART4_DMA_RX_CH           1
//   </h> DMA

// </e>


// <e> CAN1 Controller [Driver_CAN1]
// <i> Configuration settings for Driver_CAN1 in component ::Drivers:CAN
#define   RTE_CAN_CAN1                  0

//   <h> Pin Configuration
//     <o> CAN1_RD <0=>Not used <1=>P0_0 <2=>P0_21
//     <i> CAN1 receiver input.
#define   RTE_CAN1_RD_ID                0
#if      (RTE_CAN1_RD_ID == 0)
  #define RTE_CAN1_RD_PIN_EN            0
#elif    (RTE_CAN1_RD_ID == 1)
  #define RTE_CAN1_RD_PORT              0
  #define RTE_CAN1_RD_BIT               0
  #define RTE_CAN1_RD_FUNC              1
#elif    (RTE_CAN1_RD_ID == 2)
  #define RTE_CAN1_RD_PORT              0
  #define RTE_CAN1_RD_BIT               21
  #define RTE_CAN1_RD_FUNC              4
#else
  #error "Invalid RTE_CAN1_RD Pin Configuration!"
#endif
#ifndef   RTE_CAN1_RD_PIN_EN
  #define RTE_CAN1_RD_PIN_EN            1
#endif
//     <o> CAN1_TD <0=>Not used <1=>P0_1 <2=>P0_22
//     <i> CAN1 transmitter output.
#define   RTE_CAN1_TD_ID                0
#if      (RTE_CAN1_TD_ID == 0)
  #define RTE_CAN1_TD_PIN_EN            0
#elif    (RTE_CAN1_TD_ID == 1)
  #define RTE_CAN1_TD_PORT              0
  #define RTE_CAN1_TD_BIT               1
  #define RTE_CAN1_TD_FUNC              1
#elif    (RTE_CAN1_TD_ID == 2)
  #define RTE_CAN1_TD_PORT              0
  #define RTE_CAN1_TD_BIT               22
  #define RTE_CAN1_TD_FUNC              4
#else
  #error "Invalid RTE_CAN1_TD Pin Configuration!"
#endif
#ifndef   RTE_CAN1_TD_PIN_EN
  #define RTE_CAN1_TD_PIN_EN            1
#endif
//   </h> Pin Configuration
// </e> CAN1 Controller [Driver_CAN1]

// <e> CAN2 Controller [Driver_CAN2]
// <i> Configuration settings for Driver_CAN2 in component ::Drivers:CAN
#define   RTE_CAN_CAN2                  0

//   <h> Pin Configuration
//     <o> CAN2_RD <0=>Not used <1=>P0_4 <2=>P2_7
//     <i> CAN2 receiver input.
#define   RTE_CAN2_RD_ID                0
#if      (RTE_CAN2_RD_ID == 0)
  #define RTE_CAN2_RD_PIN_EN            0
#elif    (RTE_CAN2_RD_ID == 1)
  #define RTE_CAN2_RD_PORT              0
  #define RTE_CAN2_RD_BIT               4
  #define RTE_CAN2_RD_FUNC              2
#elif    (RTE_CAN2_RD_ID == 2)
  #define RTE_CAN2_RD_PORT              2
  #define RTE_CAN2_RD_BIT               7
  #define RTE_CAN2_RD_FUNC              1
#else
  #error "Invalid RTE_CAN2_RD Pin Configuration!"
#endif
#ifndef   RTE_CAN2_RD_PIN_EN
  #define RTE_CAN2_RD_PIN_EN            1
#endif
//     <o> CAN2_TD <0=>Not used <1=>P0_5 <2=>P2_8
//     <i> CAN2 transmitter output.
#define   RTE_CAN2_TD_ID                0
#if      (RTE_CAN2_TD_ID == 0)
  #define RTE_CAN2_TD_PIN_EN            0
#elif    (RTE_CAN2_TD_ID == 1)
  #define RTE_CAN2_TD_PORT              0
  #define RTE_CAN2_TD_BIT               5
  #define RTE_CAN2_TD_FUNC              2
#elif    (RTE_CAN2_TD_ID == 2)
  #define RTE_CAN2_TD_PORT              2
  #define RTE_CAN2_TD_BIT               8
  #define RTE_CAN2_TD_FUNC              1
#else
  #error "Invalid RTE_CAN2_TD Pin Configuration!"
#endif
#ifndef   RTE_CAN2_TD_PIN_EN
  #define RTE_CAN2_TD_PIN_EN            1
#endif
//   </h> Pin Configuration
// </e> CAN2 Controller [Driver_CAN2]


// <e> SSP0 (Synchronous Serial Port 0) [Driver_SPI0]
// <i> Configuration settings for Driver_SPI0 in component ::Drivers:SPI
#define RTE_SSP0                        0

//   <h> Pin Configuration
//     <o> SSP0_SSEL <0=>Not used <1=>P0_16 <2=>P1_21 <3=>P1_28 <4=>P2_23
//     <i> Slave Select for SSP0
#define   RTE_SSP0_SSEL_PIN_SEL         1
#if      (RTE_SSP0_SSEL_PIN_SEL == 0)
#define   RTE_SSP0_SSEL_PIN_EN          0
#elif    (RTE_SSP0_SSEL_PIN_SEL == 1)
  #define RTE_SSP0_SSEL_PORT            0
  #define RTE_SSP0_SSEL_BIT             16
  #define RTE_SSP0_SSEL_FUNC            2
#elif    (RTE_SSP0_SSEL_PIN_SEL == 2)
  #define RTE_SSP0_SSEL_PORT            1
  #define RTE_SSP0_SSEL_BIT             21
  #define RTE_SSP0_SSEL_FUNC            3
#elif    (RTE_SSP0_SSEL_PIN_SEL == 3)
  #define RTE_SSP0_SSEL_PORT            1
  #define RTE_SSP0_SSEL_BIT             28
  #define RTE_SSP0_SSEL_FUNC            5
#elif    (RTE_SSP0_SSEL_PIN_SEL == 4)
  #define RTE_SSP0_SSEL_PORT            2
  #define RTE_SSP0_SSEL_BIT             23
  #define RTE_SSP0_SSEL_FUNC            2
#else
  #error "Invalid SSP0 SSP0_SSEL Pin Configuration!"
#endif
#ifndef   RTE_SSP0_SSEL_PIN_EN
#define   RTE_SSP0_SSEL_PIN_EN          1
#endif

//     <o> SSP0_SCK <0=>P0_15 <1=>P1_20 <2=>P2_22
//     <i> Serial clock for SSP0
#define   RTE_SSP0_SCK_PIN_SEL          0
#if      (RTE_SSP0_SCK_PIN_SEL == 0)
  #define RTE_SSP0_SCK_PORT             0
  #define RTE_SSP0_SCK_BIT              15
  #define RTE_SSP0_SCK_FUNC             2
#elif    (RTE_SSP0_SCK_PIN_SEL == 1)
  #define RTE_SSP0_SCK_PORT             1
  #define RTE_SSP0_SCK_BIT              20
  #define RTE_SSP0_SCK_FUNC             5
#elif    (RTE_SSP0_SCK_PIN_SEL == 2)
  #define RTE_SSP0_SCK_PORT             2
  #define RTE_SSP0_SCK_BIT              22
  #define RTE_SSP0_SCK_FUNC             2
#else
  #error "Invalid SSP0 SSP0_SCK Pin Configuration!"
#endif

//     <o> SSP0_MISO <0=>Not used <1=>P0_17 <2=>P1_23 <3=>P2_26
//     <i> Master In Slave Out for SSP0
#define   RTE_SSP0_MISO_PIN_SEL         0
#if      (RTE_SSP0_MISO_PIN_SEL == 0)
  #define RTE_SSP0_MISO_PIN_EN          0
#elif    (RTE_SSP0_MISO_PIN_SEL == 1)
  #define RTE_SSP0_MISO_PORT            0
  #define RTE_SSP0_MISO_BIT             17
  #define RTE_SSP0_MISO_FUNC            2
#elif    (RTE_SSP0_MISO_PIN_SEL == 2)
  #define RTE_SSP0_MISO_PORT            1
  #define RTE_SSP0_MISO_BIT             23
  #define RTE_SSP0_MISO_FUNC            5
#elif    (RTE_SSP0_MISO_PIN_SEL == 3)
  #define RTE_SSP0_MISO_PORT            2
  #define RTE_SSP0_MISO_BIT             26
  #define RTE_SSP0_MISO_FUNC            2
#else
  #error "Invalid SSP0 SSP0_MISO Pin Configuration!"
#endif
#ifndef   RTE_SSP0_MISO_PIN_EN
#define   RTE_SSP0_MISO_PIN_EN          1
#endif

//     <o> SSP0_MOSI <0=>Not used <1=>P0_18 <2=>P1_24 <3=>P2_27
//     <i> Master Out Slave In for SSP0
#define   RTE_SSP0_MOSI_PIN_SEL         0
#if      (RTE_SSP0_MOSI_PIN_SEL == 0)
  #define RTE_SSP0_MOSI_PIN_EN          0
#elif    (RTE_SSP0_MOSI_PIN_SEL == 1)
  #define RTE_SSP0_MOSI_PORT            0
  #define RTE_SSP0_MOSI_BIT             18
  #define RTE_SSP0_MOSI_FUNC            2
#elif    (RTE_SSP0_MOSI_PIN_SEL == 2)
  #define RTE_SSP0_MOSI_PORT            1
  #define RTE_SSP0_MOSI_BIT             24
  #define RTE_SSP0_MOSI_FUNC            5
#elif    (RTE_SSP0_MOSI_PIN_SEL == 3)
  #define RTE_SSP0_MOSI_PORT            2
  #define RTE_SSP0_MOSI_BIT             27
  #define RTE_SSP0_MOSI_FUNC            2
#else
  #error "Invalid SSP0 SSP0_MOSI Pin Configuration!"
#endif
#ifndef   RTE_SSP0_MOSI_PIN_EN
#define   RTE_SSP0_MOSI_PIN_EN          1
#endif

//   </h>
//   <h> DMA
//     <e> Tx
//       <o1> Channel     <0=>0 <1=>1 <2=>2 <3=>3 <4=>4 <5=>5 <6=>6 <7=>7
// </e>
#define   RTE_SSP0_DMA_TX_EN            0
#define   RTE_SSP0_DMA_TX_CH            0
//     <e> Rx
//       <o1> Channel     <0=>0 <1=>1 <2=>2 <3=>3 <4=>4 <5=>5 <6=>6 <7=>7
//     </e>
#define   RTE_SSP0_DMA_RX_EN            0
#define   RTE_SSP0_DMA_RX_CH            1
//   </h> DMA
// </e>

// <e> SSP1 (Synchronous Serial Port 1) [Driver_SPI1]
// <i> Configuration settings for Driver_SPI1 in component ::Drivers:SPI
#define RTE_SSP1                        0

//   <h> Pin Configuration
//     <o> SSP1_SSEL <0=>Not used <1=>P0_6 <2=>P0_14 <3=>P1_26 <4=>P4_21
//     <i> Slave Select for SSP1
#define   RTE_SSP1_SSEL_PIN_SEL         1
#if      (RTE_SSP1_SSEL_PIN_SEL == 0)
  #define RTE_SSP1_SSEL_PIN_EN          0
#elif    (RTE_SSP1_SSEL_PIN_SEL == 1)
  #define RTE_SSP1_SSEL_PORT            0
  #define RTE_SSP1_SSEL_BIT             6
  #define RTE_SSP1_SSEL_FUNC            2
#elif    (RTE_SSP1_SSEL_PIN_SEL == 2)
  #define RTE_SSP1_SSEL_PORT            0
  #define RTE_SSP1_SSEL_BIT             14
  #define RTE_SSP1_SSEL_FUNC            2
#elif    (RTE_SSP1_SSEL_PIN_SEL == 3)
  #define RTE_SSP1_SSEL_PORT            1
  #define RTE_SSP1_SSEL_BIT             26
  #define RTE_SSP1_SSEL_FUNC            5
#elif    (RTE_SSP1_SSEL_PIN_SEL == 4)  
  #define RTE_SSP1_SSEL_PORT            4
  #define RTE_SSP1_SSEL_BIT             21
  #define RTE_SSP1_SSEL_FUNC            3
#else
  #error "Invalid SSP1 SSP1_SSEL Pin Configuration!"
#endif
#ifndef   RTE_SSP1_SSEL_PIN_EN
#define   RTE_SSP1_SSEL_PIN_EN          1
#endif

//     <o> SSP1_SCK <0=>P0_7 <1=>P1_19 <2=>P1_31 <3=>P4_20
//     <i> Serial clock for SSP1
#define   RTE_SSP1_SCK_PIN_SEL          0
#if      (RTE_SSP1_SCK_PIN_SEL == 0)
  #define RTE_SSP1_SCK_PORT             0
  #define RTE_SSP1_SCK_BIT              7
  #define RTE_SSP1_SCK_FUNC             2
  #define RTE_SSP1_SCK_IO_WA            1
#elif    (RTE_SSP1_SCK_PIN_SEL == 1)
  #define RTE_SSP1_SCK_PORT             1
  #define RTE_SSP1_SCK_BIT              19
  #define RTE_SSP1_SCK_FUNC             5
#elif    (RTE_SSP1_SCK_PIN_SEL == 2)
  #define RTE_SSP1_SCK_PORT             1
  #define RTE_SSP1_SCK_BIT              31
  #define RTE_SSP1_SCK_FUNC             2
  #define RTE_SSP1_SCK_IO_WA            1
#elif    (RTE_SSP1_SCK_PIN_SEL == 3)
  #define RTE_SSP1_SCK_PORT             4
  #define RTE_SSP1_SCK_BIT              20
  #define RTE_SSP1_SCK_FUNC             3
#else
  #error "Invalid SSP1 SSP1_SCK Pin Configuration!"
#endif

#ifndef   RTE_SSP1_SCK_IO_WA
#define   RTE_SSP1_SCK_IO_WA            0
#endif

//     <o> SSP1_MISO <0=>Not used <1=>P0_8 <2=>P0_12 <3=>P1_18 <4=>P4_22
//     <i> Master In Slave Out for SSP1
#define   RTE_SSP1_MISO_PIN_SEL         0
#if      (RTE_SSP1_MISO_PIN_SEL == 0)
  #define RTE_SSP1_MISO_PIN_EN          0
#elif    (RTE_SSP1_MISO_PIN_SEL == 1)
  #define RTE_SSP1_MISO_PORT            0
  #define RTE_SSP1_MISO_BIT             8
  #define RTE_SSP1_MISO_FUNC            2
  #define RTE_SSP1_MISO_IO_WA           1
#elif    (RTE_SSP1_MISO_PIN_SEL == 2)
  #define RTE_SSP1_MISO_PORT            0
  #define RTE_SSP1_MISO_BIT             12
  #define RTE_SSP1_MISO_FUNC            2
  #define RTE_SSP1_MISO_IO_WA           1
#elif    (RTE_SSP1_MISO_PIN_SEL == 3)
  #define RTE_SSP1_MISO_PORT            1
  #define RTE_SSP1_MISO_BIT             18
  #define RTE_SSP1_MISO_FUNC            5
#elif    (RTE_SSP1_MISO_PIN_SEL == 4)
  #define RTE_SSP1_MISO_PORT            4
  #define RTE_SSP1_MISO_BIT             22
  #define RTE_SSP1_MISO_FUNC            3
#else
  #error "Invalid SSP1 SSP1_MISO Pin Configuration!"
#endif
#ifndef   RTE_SSP1_MISO_PIN_EN
#define   RTE_SSP1_MISO_PIN_EN          1
#endif
#ifndef   RTE_SSP1_MISO_IO_WA
#define   RTE_SSP1_MISO_IO_WA           0
#endif

//     <o> SSP1_MOSI <0=>Not used <1=>P0_9 <2=>P0_13 <3=>P1_22 <4=>P4_23
//     <i> Master Out Slave In for SSP1
#define   RTE_SSP1_MOSI_PIN_SEL         0
#if      (RTE_SSP1_MOSI_PIN_SEL == 0)
  #define RTE_SSP1_MOSI_PIN_EN          0
#elif    (RTE_SSP1_MOSI_PIN_SEL == 1)
  #define RTE_SSP1_MOSI_PORT            0
  #define RTE_SSP1_MOSI_BIT             9
  #define RTE_SSP1_MOSI_FUNC            2
  #define RTE_SSP1_MOSI_IO_WA           1
#elif    (RTE_SSP1_MOSI_PIN_SEL == 2)
  #define RTE_SSP1_MOSI_PORT            0
  #define RTE_SSP1_MOSI_BIT             13
  #define RTE_SSP1_MOSI_FUNC            2
  #define RTE_SSP1_MOSI_IO_WA           1
#elif    (RTE_SSP1_MOSI_PIN_SEL == 3)
  #define RTE_SSP1_MOSI_PORT            1
  #define RTE_SSP1_MOSI_BIT             22
  #define RTE_SSP1_MOSI_FUNC            5
#elif    (RTE_SSP1_MOSI_PIN_SEL == 4)
  #define RTE_SSP1_MOSI_PORT            4
  #define RTE_SSP1_MOSI_BIT             23
  #define RTE_SSP1_MOSI_FUNC            3
#else
  #error "Invalid SSP1 SSP1_MOSI Pin Configuration!"
#endif
#ifndef   RTE_SSP1_MOSI_PIN_EN
#define   RTE_SSP1_MOSI_PIN_EN          1
#endif
#ifndef   RTE_SSP1_MOSI_IO_WA
#define   RTE_SSP1_MOSI_IO_WA           0
#endif

//   </h>
//   <h> DMA
//     <e> Tx
//       <o1> Channel     <0=>0 <1=>1 <2=>2 <3=>3 <4=>4 <5=>5 <6=>6 <7=>7
//     </e>
#define   RTE_SSP1_DMA_TX_EN            0
#define   RTE_SSP1_DMA_TX_CH            2
//     <e> Rx
//       <o1> Channel     <0=>0 <1=>1 <2=>2 <3=>3 <4=>4 <5=>5 <6=>6 <7=>7
//     </e>
#define   RTE_SSP1_DMA_RX_EN            0
#define   RTE_SSP1_DMA_RX_CH            3
//   </h> DMA
// </e>

// <e> SSP2 (Synchronous Serial Port 1) [Driver_SPI2]
// <i> Configuration settings for Driver_SPI2 in component ::Drivers:SPI
#define RTE_SSP2                        0

//   <h> Pin Configuration
//     <o> SSP2_SSEL <0=>Not used <1=>P1_8 <2=>P5_3
//     <i> Slave Select for SSP2
#define   RTE_SSP2_SSEL_PIN_SEL         1
#if      (RTE_SSP2_SSEL_PIN_SEL == 0)
  #define RTE_SSP2_SSEL_PIN_EN          0
#elif    (RTE_SSP2_SSEL_PIN_SEL == 1)
  #define RTE_SSP2_SSEL_PORT            1
  #define RTE_SSP2_SSEL_BIT             8
  #define RTE_SSP2_SSEL_FUNC            4
#elif    (RTE_SSP2_SSEL_PIN_SEL == 2)
  #define RTE_SSP2_SSEL_PORT            5
  #define RTE_SSP2_SSEL_BIT             3
  #define RTE_SSP2_SSEL_FUNC            2
#else
  #error "Invalid SSP2 SSP2_SSEL Pin Configuration!"
#endif
#ifndef   RTE_SSP2_SSEL_PIN_EN
#define   RTE_SSP2_SSEL_PIN_EN          1
#endif

//     <o> SSP2_SCK <0=>P1_0 <1=>P5_2
//     <i> Serial clock for SSP2
#define   RTE_SSP2_SCK_PIN_SEL          0
#if      (RTE_SSP2_SCK_PIN_SEL == 0)
  #define RTE_SSP2_SCK_PORT             1
  #define RTE_SSP2_SCK_BIT              0
  #define RTE_SSP2_SCK_FUNC             4
#elif    (RTE_SSP2_SCK_PIN_SEL == 1)
  #define RTE_SSP2_SCK_PORT             5
  #define RTE_SSP2_SCK_BIT              2
  #define RTE_SSP2_SCK_FUNC             2
#else
  #error "Invalid SSP2 SSP2_SCK Pin Configuration!"
#endif

//     <o> SSP2_MISO <0=>Not used <1=>P1_4 <2=>P5_1
//     <i> Master In Slave Out for SSP2
#define   RTE_SSP2_MISO_PIN_SEL         0
#if      (RTE_SSP2_MISO_PIN_SEL == 0)
  #define RTE_SSP2_MISO_PIN_EN          0
#elif    (RTE_SSP2_MISO_PIN_SEL == 1)
  #define RTE_SSP2_MISO_PORT            1
  #define RTE_SSP2_MISO_BIT             4
  #define RTE_SSP2_MISO_FUNC            4
#elif    (RTE_SSP2_MISO_PIN_SEL == 2)
  #define RTE_SSP2_MISO_PORT            5
  #define RTE_SSP2_MISO_BIT             1
  #define RTE_SSP2_MISO_FUNC            2
#else
  #error "Invalid SSP2 SSP2_MISO Pin Configuration!"
#endif
#ifndef   RTE_SSP2_MISO_PIN_EN
#define   RTE_SSP2_MISO_PIN_EN          1
#endif

//     <o> SSP2_MOSI <0=>Not used <1=>P1_1 <2=>P5_0
//     <i> Master Out Slave In for SSP2
#define   RTE_SSP2_MOSI_PIN_SEL         0
#if      (RTE_SSP2_MOSI_PIN_SEL == 0)
  #define RTE_SSP2_MOSI_PIN_EN          0
#elif    (RTE_SSP2_MOSI_PIN_SEL == 1)
  #define RTE_SSP2_MOSI_PORT            1
  #define RTE_SSP2_MOSI_BIT             1
  #define RTE_SSP2_MOSI_FUNC            4
#elif    (RTE_SSP2_MOSI_PIN_SEL == 2)
  #define RTE_SSP2_MOSI_PORT            5
  #define RTE_SSP2_MOSI_BIT             0
  #define RTE_SSP2_MOSI_FUNC            2
#else
  #error "Invalid SSP2 SSP2_MOSI Pin Configuration!"
#endif
#ifndef   RTE_SSP2_MOSI_PIN_EN
#define   RTE_SSP2_MOSI_PIN_EN          1
#endif

//   </h>
//   <h> DMA
//     <e> Tx
//       <o1> Channel     <0=>0 <1=>1 <2=>2 <3=>3 <4=>4 <5=>5 <6=>6 <7=>7
//     </e>
#define   RTE_SSP2_DMA_TX_EN            0
#define   RTE_SSP2_DMA_TX_CH            4
//     <e> Rx
//       <o1> Channel     <0=>0 <1=>1 <2=>2 <3=>3 <4=>4 <5=>5 <6=>6 <7=>7
//     </e>
#define   RTE_SSP2_DMA_RX_EN            0
#define   RTE_SSP2_DMA_RX_CH            5
//   </h> DMA
// </e>

// <e> I2S0 (Integrated Interchip Sound 0) [Driver_SAI0]
// <i> Configuration settings for Driver_SAI0 in component ::Drivers:SAI
#define   RTE_I2S0                      0

//   <h> Pin Configuration
//     <o> I2S0_RX_SCK <0=>Not used <1=>P0_4 <2=>P0_23
//     <i> Receive clock for I2S0
#define   RTE_I2S0_RX_SCK_PIN_SEL       1
#if      (RTE_I2S0_RX_SCK_PIN_SEL == 0)
#define   RTE_I2S0_RX_SCK_PIN_EN        0
#elif    (RTE_I2S0_RX_SCK_PIN_SEL == 1)
  #define RTE_I2S0_RX_SCK_PORT          0
  #define RTE_I2S0_RX_SCK_BIT           4
  #define RTE_I2S0_RX_SCK_FUNC          1
#elif    (RTE_I2S0_RX_SCK_PIN_SEL == 2)
  #define RTE_I2S0_RX_SCK_PORT          0
  #define RTE_I2S0_RX_SCK_BIT           23
  #define RTE_I2S0_RX_SCK_FUNC          2
  #define RTE_I2S0_RX_SCK_IO_WA         1
#else
  #error "Invalid I2S0 I2S0_RX_SCK Pin Configuration!"
#endif
#ifndef   RTE_I2S0_RX_SCK_PIN_EN
#define   RTE_I2S0_RX_SCK_PIN_EN        1
#endif
#ifndef   RTE_I2S0_RX_SCK_IO_WA
#define   RTE_I2S0_RX_SCK_IO_WA         0
#endif
//     <o> I2S0_RX_WS <0=>Not used <1=>P0_5 <2=>P0_24
//     <i> Receive word select for I2S0
#define   RTE_I2S0_RX_WS_PIN_SEL        1
#if      (RTE_I2S0_RX_WS_PIN_SEL == 0)
#define   RTE_I2S0_RX_WS_PIN_EN         0
#elif    (RTE_I2S0_RX_WS_PIN_SEL == 1)
  #define RTE_I2S0_RX_WS_PORT           0
  #define RTE_I2S0_RX_WS_BIT            5
  #define RTE_I2S0_RX_WS_FUNC           1
#elif    (RTE_I2S0_RX_WS_PIN_SEL == 2)
  #define RTE_I2S0_RX_WS_PORT           0
  #define RTE_I2S0_RX_WS_BIT            24
  #define RTE_I2S0_RX_WS_FUNC           2
  #define RTE_I2S0_RX_WS_IO_WA          1
#else
  #error "Invalid I2S0 I2S0_RX_WS Pin Configuration!"
#endif
#ifndef   RTE_I2S0_RX_WS_PIN_EN
#define   RTE_I2S0_RX_WS_PIN_EN         1
#endif
#ifndef   RTE_I2S0_RX_WS_IO_WA
#define   RTE_I2S0_RX_WS_IO_WA          0
#endif
//     <o> I2S0_RX_SDA <0=>Not used <1=>P0_6 <2=>P0_25
//     <i> Receive master clock for I2S0
#define   RTE_I2S0_RX_SDA_PIN_SEL       1
#if      (RTE_I2S0_RX_SDA_PIN_SEL == 0)
#define   RTE_I2S0_RX_SDA_PIN_EN        0
#elif    (RTE_I2S0_RX_SDA_PIN_SEL == 1)
  #define RTE_I2S0_RX_SDA_PORT          0
  #define RTE_I2S0_RX_SDA_BIT           6
  #define RTE_I2S0_RX_SDA_FUNC          1
#elif    (RTE_I2S0_RX_SDA_PIN_SEL == 2)
  #define RTE_I2S0_RX_SDA_PORT          0
  #define RTE_I2S0_RX_SDA_BIT           25
  #define RTE_I2S0_RX_SDA_FUNC          2
  #define RTE_I2S0_RX_SDA_IO_WA         1
#else
  #error "Invalid I2S0 I2S0_RX_SDA Pin Configuration!"
#endif
#ifndef   RTE_I2S0_RX_SDA_PIN_EN
#define   RTE_I2S0_RX_SDA_PIN_EN        1
#endif
#ifndef   RTE_I2S0_RX_SDA_IO_WA
#define   RTE_I2S0_RX_SDA_IO_WA         0
#endif
//     <o> I2S0_RX_MCLK <0=>Not used <1=>P1_17
//     <i> Receive master clock for I2S0
#define   RTE_I2S0_RX_MCLK_PIN_SEL      0
#if      (RTE_I2S0_RX_MCLK_PIN_SEL == 0)
#define   RTE_I2S0_RX_MCLK_PIN_EN       0
#elif    (RTE_I2S0_RX_MCLK_PIN_SEL == 1)
  #define RTE_I2S0_RX_MCLK_PORT         1
  #define RTE_I2S0_RX_MCLK_BIT          17
  #define RTE_I2S0_RX_MCLK_FUNC         2
  #define RTE_I2S0_RX_MCLK_IO_WA        1
#else
  #error "Invalid I2S0 I2S0_RX_MCLK Pin Configuration!"
#endif
#ifndef   RTE_I2S0_RX_MCLK_PIN_EN
#define   RTE_I2S0_RX_MCLK_PIN_EN       1
#endif
#ifndef   RTE_I2S0_RX_MCLK_IO_WA
#define   RTE_I2S0_RX_MCLK_IO_WA        0
#endif
//     <o> I2S0_TX_SCK <0=>Not used <1=>P0_7 <2=>P2_11
//     <i> Transmit clock for I2S0
#define   RTE_I2S0_TX_SCK_PIN_SEL       1
#if      (RTE_I2S0_TX_SCK_PIN_SEL == 0)
#define   RTE_I2S0_TX_SCK_PIN_EN        0
#elif    (RTE_I2S0_TX_SCK_PIN_SEL == 1)
  #define RTE_I2S0_TX_SCK_PORT          0
  #define RTE_I2S0_TX_SCK_BIT           7
  #define RTE_I2S0_TX_SCK_FUNC          1
  #define RTE_I2S0_TX_SCK_IO_WA         1
#elif    (RTE_I2S0_TX_SCK_PIN_SEL == 2)
  #define RTE_I2S0_TX_SCK_PORT          2
  #define RTE_I2S0_TX_SCK_BIT           11
  #define RTE_I2S0_TX_SCK_FUNC          3
#else
  #error "Invalid I2S0 I2S0_TX_SCK Pin Configuration!"
#endif
#ifndef   RTE_I2S0_TX_SCK_PIN_EN
#define   RTE_I2S0_TX_SCK_PIN_EN        1
#endif
#ifndef   RTE_I2S0_TX_SCK_IO_WA
#define   RTE_I2S0_TX_SCK_IO_WA         0
#endif
//     <o> I2S0_TX_WS <0=>Not used <1=>P0_8 <2=>P2_12
//     <i> Transmit word select for I2S0
#define   RTE_I2S0_TX_WS_PIN_SEL        1
#if      (RTE_I2S0_TX_WS_PIN_SEL == 0)
#define   RTE_I2S0_TX_WS_PIN_EN         0
#elif    (RTE_I2S0_TX_WS_PIN_SEL == 1)
  #define RTE_I2S0_TX_WS_PORT           0
  #define RTE_I2S0_TX_WS_BIT            8
  #define RTE_I2S0_TX_WS_FUNC           1
  #define RTE_I2S0_TX_WS_IO_WA          1
#elif    (RTE_I2S0_TX_WS_PIN_SEL == 2)
  #define RTE_I2S0_TX_WS_PORT           2
  #define RTE_I2S0_TX_WS_BIT            12
  #define RTE_I2S0_TX_WS_FUNC           3
#else
  #error "Invalid I2S0 I2S0_TX_WS Pin Configuration!"
#endif
#ifndef   RTE_I2S0_TX_WS_PIN_EN
#define   RTE_I2S0_TX_WS_PIN_EN         1
#endif
#ifndef   RTE_I2S0_TX_WS_IO_WA
#define   RTE_I2S0_TX_WS_IO_WA          0
#endif
//     <o> I2S0_TX_SDA <0=>Not used <1=>P0_9 <2=>P2_13
//     <i> Transmit data for I2S0
#define   RTE_I2S0_TX_SDA_PIN_SEL       1
#if      (RTE_I2S0_TX_SDA_PIN_SEL == 0)
#define   RTE_I2S0_TX_SDA_PIN_EN        0
#elif    (RTE_I2S0_TX_SDA_PIN_SEL == 1)
  #define RTE_I2S0_TX_SDA_PORT          0
  #define RTE_I2S0_TX_SDA_BIT           9
  #define RTE_I2S0_TX_SDA_FUNC          1
  #define RTE_I2S0_TX_SDA_IO_WA         1
#elif    (RTE_I2S0_TX_SDA_PIN_SEL == 2)
  #define RTE_I2S0_TX_SDA_PORT          2
  #define RTE_I2S0_TX_SDA_BIT           13
  #define RTE_I2S0_TX_SDA_FUNC          3
#else
  #error "Invalid I2S0 I2S0_TX_SDA Pin Configuration!"
#endif
#ifndef   RTE_I2S0_TX_SDA_PIN_EN
#define   RTE_I2S0_TX_SDA_PIN_EN        1
#endif
#ifndef   RTE_I2S0_TX_SDA_IO_WA
#define   RTE_I2S0_TX_SDA_IO_WA         0
#endif
//     <o> I2S0_TX_MCLK <0=>Not used <1=>P1_16
//     <i> Transmit master clock for I2S0
#define   RTE_I2S0_TX_MCLK_PIN_SEL      1
#if      (RTE_I2S0_TX_MCLK_PIN_SEL == 0)
#define   RTE_I2S0_TX_MCLK_PIN_EN       0
#elif    (RTE_I2S0_TX_MCLK_PIN_SEL == 1)
  #define RTE_I2S0_TX_MCLK_PORT         1
  #define RTE_I2S0_TX_MCLK_BIT          16
  #define RTE_I2S0_TX_MCLK_FUNC         2
  #define RTE_I2S0_TX_MCLK_IO_WA        1
#else
  #error "Invalid I2S0 I2S0_TX_MCLK Pin Configuration!"
#endif
#ifndef   RTE_I2S0_TX_MCLK_PIN_EN
#define   RTE_I2S0_TX_MCLK_PIN_EN       1
#endif
#ifndef   RTE_I2S0_TX_MCLK_IO_WA
#define   RTE_I2S0_TX_MCLK_IO_WA        0
#endif
//   </h> Pin Configuration

//   <h> DMA
//     <e> Tx
//       <o1> Channel     <0=>0 <1=>1 <2=>2 <3=>3 <4=>4 <5=>5 <6=>6 <7=>7
//       <o2> Peripheral  <0=>9 (DMAMUXPER9)
//     </e>
#define   RTE_I2S0_DMA_TX_EN            1
#define   RTE_I2S0_DMA_TX_CH            0
//     <e> Rx
//       <o1> Channel     <0=>0 <1=>1 <2=>2 <3=>3 <4=>4 <5=>5 <6=>6 <7=>7
//       <o2> Peripheral  <0=>10 (DMAMUXPER10)
//     </e>
#define   RTE_I2S0_DMA_RX_EN            1
#define   RTE_I2S0_DMA_RX_CH            1
//   </h> DMA
// </e> I2S0 (Integrated Interchip Sound 0) [Driver_SAI0]

#endif  /* __RTE_DEVICE_H */
