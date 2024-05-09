 /******************************************************************************
 *
 * Module: Dio
 *
 * File Name: Dio_Cfg.h
 *
 * Description: Pre-Compile Configuration Header file for TM4C123GH6PM Microcontroller - Dio Driver
 *
 * Author: Mohamed Tarek
 ******************************************************************************/

#ifndef PORT_CFG_H
#define PORT_CFG_H

/*
 * Module Version 1.0.0
 */
#define PORT_SW_MAJOR_VERSION           (1U)
#define PORT_SW_MINOR_VERSION           (0U)
#define PORT_SW_PATCH_VERSION           (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_AR_RELEASE_MAJOR_VERSION   (4U)
#define PORT_AR_RELEASE_MINOR_VERSION   (0U)
#define PORT_AR_RELEASE_PATCH_VERSION   (3U)

/*Numbers of configured pins from autoSAR tool*/
#define PORT_CONFIGURED_PINS (uint8)0x2B

/* Pre-compile option for Development Error Detect */
#define PORT_DEV_ERROR_DETECT                (STD_ON)
/* Pre-compile option for Version Info API */
#define PORT_VERSION_INFO_API                (STD_ON)

/* PORT Configured Port ID's  */
#define PortConf_PORTA_NUM           (PORT_PortType)0
#define PortConf_PORTB_NUM           (PORT_PortType)1
#define PortConf_PORTC_NUM           (PORT_PortType)2
#define PortConf_PORTD_NUM           (PORT_PortType)3
#define PortConf_PORTE_NUM           (PORT_PortType)4
#define PortConf_PORTF_NUM           (PORT_PortType)5

/* PORT Configured Pins ID's  */
#define PortConf_PIN1_NUM             (Port_PinType)0
#define PortConf_PIN2_NUM             (Port_PinType)1
#define PortConf_PIN3_NUM             (Port_PinType)2
#define PortConf_PIN4_NUM             (Port_PinType)3
#define PortConf_PIN5_NUM             (Port_PinType)4
#define PortConf_PIN6_NUM             (Port_PinType)5
#define PortConf_PIN7_NUM             (Port_PinType)6
#define PortConf_PIN8_NUM             (Port_PinType)7

#endif /* PORT_CFG_H */
