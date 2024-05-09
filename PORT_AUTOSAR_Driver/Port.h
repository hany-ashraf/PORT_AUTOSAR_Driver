 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.h
 *
 * Description: Header file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Mohamed Tarek
 ******************************************************************************/

#ifndef PORT_H
#define PORT_H

#include "Common_Macros.h"
#include "Std_Types.h"
#include "Port_Cfg.h"


/*Variable for terminate loop = (PORT_CONFIGURED_PINS + 1)*/
#define TERMINATE_LOOP      (PORT_CONFIGURED_PINS + (uint8)0x01)
/*Initiation value to start looping from zero*/
#define Zero_Starting            0
/*Special port of NMI */
#define SPECIAL_PORTD            3
#define SPECIAL_PORTF            5
/*Special pins of NMI*/
#define PIN_ZERO                 0
#define PIN_SEVEN                7
/*JTAG pins from 0 to 3*/
#define PIN_THREE_JTAG           3
/*JTAG PORT C*/
#define PORT_JTAG                2
/*Shifting four bits each time*/
#define SHIFT_FOUR_BITS          4

/* Id for the company in the AUTOSAR for example Mohamed Tarek's ID = 1000 :) */
#define PORT_VENDOR_ID    (1000U)
/*Port Module Id */
#define PORT_MODULE_ID    (120U)
/*Port Instance Id */
#define PORT_INSTANCE_ID  (0U)


/*******************************************************************************
 *                              Module Definitions                             *
 *******************************************************************************/

/* GPIO Registers base addresses */
#define GPIO_PORTA_BASE_ADDRESS           0x40004000
#define GPIO_PORTB_BASE_ADDRESS           0x40005000
#define GPIO_PORTC_BASE_ADDRESS           0x40006000
#define GPIO_PORTD_BASE_ADDRESS           0x40007000
#define GPIO_PORTE_BASE_ADDRESS           0x40024000
#define GPIO_PORTF_BASE_ADDRESS           0x40025000

/* GPIO Registers offset addresses */
#define PORT_DATA_REG_OFFSET              0x3FC
#define PORT_DIR_REG_OFFSET               0x400
#define PORT_ALT_FUNC_REG_OFFSET          0x420
#define PORT_PULL_UP_REG_OFFSET           0x510
#define PORT_PULL_DOWN_REG_OFFSET         0x514
#define PORT_DIGITAL_ENABLE_REG_OFFSET    0x51C
#define PORT_LOCK_REG_OFFSET              0x520
#define PORT_COMMIT_REG_OFFSET            0x524
#define PORT_ANALOG_MODE_SEL_REG_OFFSET   0x528
#define PORT_CTL_REG_OFFSET               0x52C

/*******************************************************************************
 *                              Module Data Types                              *
 *******************************************************************************/
/* Type definition for PORT_ChannelType used by the DIO APIs */
typedef uint8 Port_PinType;
/* Type definition for PORT_PortType used by the DIO APIs */
typedef uint8 PORT_PortType;

/******************************************************************************
 *                      API Service Id Macros                                 *
 ******************************************************************************/
/* Service ID for Port Version info Channel */
#define PORT_GET_VERSION_INFO_SID          (uint8)0x01

/* Service ID for POrt pin initialization Port */
#define PORT_INIT_SID                      (uint8)0x02

/* Service ID for Port set pin direction Port */
#define PORT_SET_PIN_DIRECTION             (uint8)0x03

/* Service ID for PORT set pin mode Group */
#define PORT_SET_PIN_MODE                  (uint8)0x04

/*******************************************************************************
 *                      DET Error Codes                                        *
 *******************************************************************************/
/*Invalid Port Pin ID requested*/
#define PORT_E_PARAM_PIN                  (uint8)0x0A
/*Port Pin not configured as changeable*/
#define PORT_E_DIRECTION_UNCHANGEABLE     (uint8)0x0B
/*API Port_SetPinMode service called when mode is unchangeable*/
#define PORT_E_MODE_UNCHANGEABLE          (uint8)0x0E
/*APIs called with a Null Pointer*/
#define PORT_E_PARAM_POINTER              (uint8)0x10
/*Configuration error*/
#define PORT_E_PARAM_CONFIG               (uint8)0x0C
/*******************************************************************************
 *                              Module Data Types                              *
 *******************************************************************************/
/* Description: Enum to hold PIN direction */
typedef enum
{
    PORT_PIN_IN,PORT_PIN_OUT
}Port_PinDirectionType;

/* Description: Enum to hold internal resistor type for PIN */
typedef enum
{
    OFF,PULL_UP,PULL_DOWN
}Port_InternalResistor;
/*Description: Enum to specify pin mode is analog or digital*/
typedef enum
{
    ANALOG,DIGITAL
}Port_PinModeType;
/*Description: Enum to specify pin mode is CPIO or ASSOCIATED_PERIPHERAL*/
typedef enum
{
    GPIO,ASSOCIATED_PERIPHERAL
}PinModeType;
/*Description: Enum to specify pin mode is mode_unchangeable or mode_changeable*/

typedef enum
{
    mode_unchangeable,mode_changeable
}PinModeChangeable;
/*Description: Enum to specify pin mode is CPIO or ASSOCIATED_PERIPHERAL*/
typedef enum
{
    direction_unchangeable,direction_changeable
}PinDirectionChangeable;
typedef enum
{
    LOW,HIGH
}INITIAL_LEVEL;
/*Description: Structure that contains configuration of each pin*/
typedef struct 
{
    PORT_PortType port_num;
    Port_PinType pin_num;
    PinModeType  GPIO_AssociatedPeripheral;         /*GPIO pin or associated peripheral*/
    Port_PinModeType mode;   /*Digital or Analog*/
    PinModeChangeable mode_ischangable;
    PinDirectionChangeable pin_direction_ischangable;
    Port_PinDirectionType direction; /*input or output*/
    INITIAL_LEVEL initial_value;     /*0 or 1*/
    Port_InternalResistor resistor;  /*pull_up  or  pull_down*/
}Port_ConfigChannel;

/*Array of configuration structure of type of Port_ConfigType*/
typedef struct Port_ConfigType
{
    Port_ConfigChannel array_Config[PORT_CONFIGURED_PINS];
} Port_ConfigType;


/*******************************************************************************
 *                      Function Prototypes                                    *
 *******************************************************************************/
/************************************************************************************
* Service Name: Port_Init
* Service ID : 0x00
* Sync/ASync: Synchronous
* Re_entrancy: Non-reentrant
* Parameters (in): ConfigPtr - Pointer to Array of post-build configuration data for pins
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Function to Setup the pin configuration:
*              - Setup the pin as Digital or Analog GPIO pin
*              - Setup the direction of the GPIO pin
*              - Provide initial value for o/p pin
*              - Setup the internal resistor for i/p pin
*              - Setup the pin mode GPIO or ASSOCIATED_PERIPHERAL
************************************************************************************/
void Port_Init(const Port_ConfigType * ConfigPtr );
/************************************************************************************
* Service Name: Port_Init
* Service ID : 0x00
* Sync/ASync: Synchronous
* Re_entrancy: Non-reentrant
* Parameters (in): ConfigPtr - Pointer to Array of post-build configuration data for pins
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Function to Setup the pin configuration:
*              - Setup the pin as Digital or Analog GPIO pin
*              - Setup the direction of the GPIO pin
*              - Provide initial value for o/p pin
*              - Setup the internal resistor for i/p pin
*              - Setup the pin mode GPIO or ASSOCIATED_PERIPHERAL
************************************************************************************/
void Port_SetupGpioPin(const Port_ConfigType *ConfigPtr );
/************************************************************************************
* Service Name: Port_SetPinDirection
* Service ID: 0x01
* Sync/ASync: Synchronous
* Re_entrancy: reentrant
* Parameters (in): Port Pin ID number and Port Pin Direction
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description:Sets the port pin direction if the changeable
************************************************************************************/
void Port_SetPinDirection(Port_PinType Pin,Port_PinDirectionType Direction);
/************************************************************************************
* Service Name: Port_RefreshPortDirection
* Service ID: 0x02
* Sync/ASync: Synchronous
* Re_entrancy: Non-reentrant
* Parameters (in): None
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Refresh these pins are unchangeable during runtime
************************************************************************************/
void Port_RefreshPortDirection(void);
/************************************************************************************
* Service Name: Port_GetVersionInfo
* Service ID: 0x03
* Sync/ASync: Synchronous
* Re_entrancy: Non-reentrant
* Parameters (in): None
* Parameters (inout): None
* Parameters (out): versioninfo Pointer to where to store the version information of this module.
* Return value: None
* Description: Returns the version information of this module.
************************************************************************************/
void Port_GetVersionInfo(Std_VersionInfoType* versioninfo);
/************************************************************************************
* Service Name: Port_SetPinMode
* Service ID: 0x04
* Sync/ASync: Synchronous
* Re_entrancy: reentrant
* Parameters (in): Port Pin ID number and New Port Pin mode to be set on port pin.
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Sets the port pin mode
************************************************************************************/
void Port_SetPinMode(Port_PinType Pin,Port_PinModeType Mode);

/*******************************************************************************
 *                       External Variables                                    *
 *******************************************************************************/

/* Extern PB structures to be used by Dio and other modules */
extern const Port_ConfigType Port_Configuration;



#endif /* PORT_H */
