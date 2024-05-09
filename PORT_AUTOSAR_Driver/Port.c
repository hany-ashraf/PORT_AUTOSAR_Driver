 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.c
 *
 * Description: Source file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Mohamed Tarek
 ******************************************************************************/

#include "Port.h"
#include "tm4c123gh6pm_registers.h"

#if (PORT_DEV_ERROR_DETECT == STD_ON)
#include "Det.h"
/* AUTOSAR Version checking between Det and PORT Modules */
#if ((DET_AR_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 || (DET_AR_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 || (DET_AR_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Det.h does not match the expected version"
#endif
#endif

STATIC const Port_ConfigChannel * Port_PortChannels = NULL_PTR;

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
void Port_Init(const Port_ConfigType * ConfigPtr )
{
    volatile uint32 * PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */
    uint8 Index_of_configStr = Zero_Starting;
    Port_PortChannels = ConfigPtr->array_Config;

#if (PORT_DEV_ERROR_DETECT == STD_ON)
    /* check if the input configuration pointer is not a NULL_PTR */
    if (NULL_PTR == ConfigPtr)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_INIT_SID,
             PORT_E_PARAM_CONFIG);
    }
    else
#endif
/*Loop of all pin configuration structure*/
    for(Index_of_configStr=Zero_Starting; Index_of_configStr<=PORT_CONFIGURED_PINS; Index_of_configStr++){
        /* Point to the correct PORT register according to the Port Id stored in the Port_Num member */
        switch(ConfigPtr->array_Config[Index_of_configStr].port_num)
        {
            case 0: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS;   /*PortA base address*/
                    break;
            case 1: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS;   /*PortB base address*/
                    break;
            case 2: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS;   /*PortC base address*/
                    break;
            case 3: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS;   /*PortD base address*/
                    break;
            case 4: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS;   /*PortE base address*/
                    break;
            case 5: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS;   /*PortF base address*/
                    break;
        }
        /* PD7 or PF0 */
        if( ((ConfigPtr->array_Config[Index_of_configStr].port_num == SPECIAL_PORTD) && (ConfigPtr->array_Config[Index_of_configStr].pin_num == PIN_SEVEN)) || ((ConfigPtr->array_Config[Index_of_configStr].port_num == SPECIAL_PORTF) && (ConfigPtr->array_Config[Index_of_configStr].pin_num == PIN_ZERO)) )
            {
                /* Unlock the GPIOCR register */
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_LOCK_REG_OFFSET) = 0x4C4F434B;
                /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_COMMIT_REG_OFFSET) , ConfigPtr->array_Config[Index_of_configStr].pin_num);
            }
        /* PC0 to PC3 */
        else if((ConfigPtr->array_Config[Index_of_configStr].port_num == PORT_JTAG) && (ConfigPtr->array_Config[Index_of_configStr].pin_num <= PIN_THREE_JTAG))
            {
                /* Do Nothing ...  this is the JTAG pins */
            }
        else
            {
                /* Do Nothing ... No need to unlock the commit register for this pin */
            }
       if(STD_LOW == (ConfigPtr->array_Config[Index_of_configStr].GPIO_AssociatedPeripheral))
       {
           CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , ConfigPtr->array_Config[Index_of_configStr].pin_num);
           *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (ConfigPtr->array_Config[Index_of_configStr].pin_num * SHIFT_FOUR_BITS));
       }
       else if(STD_HIGH == (ConfigPtr->array_Config[Index_of_configStr].GPIO_AssociatedPeripheral))
       {
           SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , ConfigPtr->array_Config[Index_of_configStr].pin_num);
       }
       if(STD_LOW == (ConfigPtr->array_Config[Index_of_configStr].mode))
       {
           SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , ConfigPtr->array_Config[Index_of_configStr].pin_num);
           CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , ConfigPtr->array_Config[Index_of_configStr].pin_num);
       }
       else if(STD_HIGH == (ConfigPtr->array_Config[Index_of_configStr].mode))
       {
           CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , ConfigPtr->array_Config[Index_of_configStr].pin_num);
           SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , ConfigPtr->array_Config[Index_of_configStr].pin_num);
       }

       if(STD_ON == (ConfigPtr->array_Config[Index_of_configStr].direction))
       {
           SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET), ConfigPtr->array_Config[Index_of_configStr].pin_num);
           if(STD_HIGH == ConfigPtr->array_Config[Index_of_configStr].initial_value)
           {
               SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET), ConfigPtr->array_Config[Index_of_configStr].pin_num);
           }
           else
           {
               CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET), ConfigPtr->array_Config[Index_of_configStr].pin_num);
           }
       }
       else if(STD_OFF == (ConfigPtr->array_Config[Index_of_configStr].direction))
       {
           CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET), ConfigPtr->array_Config[Index_of_configStr].pin_num);
           if(PULL_UP == (ConfigPtr->array_Config[Index_of_configStr].resistor))
           {
               SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET), ConfigPtr->array_Config[Index_of_configStr].pin_num);
           }
           else if(PULL_DOWN == (ConfigPtr->array_Config[Index_of_configStr].resistor))
           {
               SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET), ConfigPtr->array_Config[Index_of_configStr].pin_num);
           }
           else
           {
               CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET), ConfigPtr->array_Config[Index_of_configStr].pin_num);
               CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET), ConfigPtr->array_Config[Index_of_configStr].pin_num);
           }
       }

    }
}
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
void Port_SetPinDirection(Port_PinType Pin,Port_PinDirectionType Direction)
{
    boolean error = FALSE;
    volatile uint32 * PortGpio_Ptr = NULL_PTR;
    uint8 Index_of_configStr = Zero_Starting;
    uint8 pin_index = Zero_Starting;
/*Report error if the pin out of range*/
#if (PORT_DEV_ERROR_DETECT == STD_ON)
    if (Pin > PORT_CONFIGURED_PINS)
    {

        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
                PORT_SET_PIN_DIRECTION, PORT_E_PARAM_PIN);
        error = TRUE;
    }
    else
    {
        /* No Action Required */
    }
#endif
    /* In-case there are no errors */
    if(FALSE == error)
    {
        /*Loop of PORT_CONFIGURED_PINS to catch the index of the pin passed*/
        for(Index_of_configStr = Zero_Starting; Index_of_configStr<=PORT_CONFIGURED_PINS; Index_of_configStr++)
        {
                if(Pin ==  Port_PortChannels[Index_of_configStr].pin_num)
                {
                    pin_index = Index_of_configStr;
                    Index_of_configStr = TERMINATE_LOOP;
                }

        }

#if (PORT_DEV_ERROR_DETECT == STD_ON)
        if(STD_OFF == (Port_PortChannels[pin_index].pin_direction_ischangable))
        {
            Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
                            PORT_SET_PIN_MODE, PORT_E_DIRECTION_UNCHANGEABLE);
            error = TRUE;
        }
        else
        {
            /* No Action Required */
        }
#endif
    }
        if(FALSE == error)
        {
            /* Point to the correct PORT register according to the Port Id stored in the Port_Num member */
                switch(Port_PortChannels[pin_index].port_num)
                {
                    case 0: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS;   /*PortA base address*/
                            break;
                    case 1: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS;   /*PortB base address*/
                            break;
                    case 2: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS;   /*PortC base address*/
                            break;
                    case 3: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS;   /*PortD base address*/
                            break;
                    case 4: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS;   /*PortE base address*/
                            break;
                    case 5: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS;   /*PortF base address*/
                            break;
                }
                if(STD_ON == Direction)
                {
                       SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET), Pin);
                       if(STD_HIGH == Port_PortChannels[pin_index].initial_value)
                       {
                              SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET), Pin);
                       }
                       else
                       {
                              CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET), Pin);
                       }
                 }
                 else if(STD_OFF == Direction)
                 {
                       CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET), Pin);
                       if(PULL_UP == (Port_PortChannels[pin_index].resistor))
                       {
                              SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET), Pin);
                       }
                       else if(PULL_DOWN == (Port_PortChannels[pin_index].resistor))
                       {
                              SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET), Pin);
                       }
                       else
                       {
                              CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET), Pin);
                              CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET), Pin);
                       }
                 }

        }

}
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
void Port_RefreshPortDirection(void)
{
    volatile uint32 * PortGpio_Ptr = NULL_PTR;
    uint8 loop_controller=Zero_Starting;
    /*loop of all pin to make a refreshment*/
    while(loop_controller<=PORT_CONFIGURED_PINS)
    {
        /*If the pin is not changeable it go through to refresh pins*/
        if(STD_OFF == (Port_PortChannels[loop_controller].pin_direction_ischangable))
        {
            /* Point to the correct PORT register according to the Port Id stored in the Port_Num member */
                switch(Port_PortChannels[loop_controller].port_num)
                {
                    case 0: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS;   /*PortA base address*/
                            break;
                    case 1: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS;   /*PortB base address*/
                            break;
                    case 2: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS;   /*PortC base address*/
                            break;
                    case 3: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS;   /*PortD base address*/
                            break;
                    case 4: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS;   /*PortE base address*/
                            break;
                    case 5: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS;   /*PortF base address*/
                            break;
                }
                if(STD_ON == Port_PortChannels[loop_controller].direction)
                {
                       SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET), Port_PortChannels[loop_controller].pin_num);
                       if(STD_HIGH == Port_PortChannels[loop_controller].initial_value)
                       {
                              SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET), Port_PortChannels[loop_controller].pin_num);
                       }
                       else
                       {
                              CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET), Port_PortChannels[loop_controller].pin_num);
                       }
                }
                else if(STD_OFF == Port_PortChannels[loop_controller].direction)
                {
                       CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET), Port_PortChannels[loop_controller].pin_num);
                       if(PULL_UP == (Port_PortChannels[loop_controller].resistor))
                       {
                              SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET), Port_PortChannels[loop_controller].pin_num);
                       }
                       else if(PULL_DOWN == (Port_PortChannels[loop_controller].resistor))
                       {
                              SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET), Port_PortChannels[loop_controller].pin_num);
                       }
                       else
                       {
                              CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET), Port_PortChannels[loop_controller].pin_num);
                              CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET), Port_PortChannels[loop_controller].pin_num);
                       }
                }

          }

        /*increment to the next index*/
        loop_controller++;
    }

}
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
#if (PORT_VERSION_INFO_API == STD_ON)
void Port_GetVersionInfo(Std_VersionInfoType* versioninfo)
{
#if (PORT_DEV_ERROR_DETECT == STD_ON)
    /* Check if input pointer is not Null pointer */
    if(NULL_PTR == versioninfo)
    {
        /* Report to DET  */
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
                PORT_GET_VERSION_INFO_SID, PORT_E_PARAM_POINTER);
    }
    else
#endif /* (DIO_DEV_ERROR_DETECT == STD_ON) */
    {
        /* Copy the vendor Id */
        versioninfo->vendorID = (uint16)PORT_VENDOR_ID;
        /* Copy the module Id */
        versioninfo->moduleID = (uint16)PORT_MODULE_ID;
        /* Copy Software Major Version */
        versioninfo->sw_major_version = (uint8)PORT_SW_MAJOR_VERSION;
        /* Copy Software Minor Version */
        versioninfo->sw_minor_version = (uint8)PORT_SW_MINOR_VERSION;
        /* Copy Software Patch Version */
        versioninfo->sw_patch_version = (uint8)PORT_SW_PATCH_VERSION;
    }
}
#endif
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
void Port_SetPinMode(Port_PinType Pin,Port_PinModeType Mode)
{
    boolean error = FALSE;
    volatile uint32 * PortGpio_Ptr = NULL_PTR;
    uint8 Index_of_configStr = Zero_Starting;
    uint8 pin_index = Zero_Starting;
#if (PORT_DEV_ERROR_DETECT == STD_ON)
    if (Pin > PORT_CONFIGURED_PINS)
    {

        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
                        PORT_SET_PIN_MODE, PORT_E_PARAM_PIN);
        error = TRUE;
    }
    else
    {
        /* No Action Required */
    }
#endif
    /* In-case there are no errors */
    if(FALSE == error)
    {
        /*Loop of PORT_CONFIGURED_PINS to catch the index of the pin passed*/
        for(Index_of_configStr = Zero_Starting; Index_of_configStr<=PORT_CONFIGURED_PINS; Index_of_configStr++)
        {
                if(Pin ==  Port_PortChannels[Index_of_configStr].pin_num)
                {
                    pin_index = Index_of_configStr;
                    Index_of_configStr = TERMINATE_LOOP;
                }

        }

#if (PORT_DEV_ERROR_DETECT == STD_ON)
        if(STD_ON == (Port_PortChannels[pin_index].mode_ischangable))
        {
            Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
                   PORT_SET_PIN_MODE, PORT_E_MODE_UNCHANGEABLE);
            error = TRUE;
        }
        else
        {
            /* No Action Required */
        }
#endif
    }
        if(FALSE == error)
        {
            if(STD_ON == (Port_PortChannels[pin_index].mode_ischangable))
            {
                switch(Port_PortChannels[pin_index].port_num)
                {
                    case 0: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS;   /*PortA base address*/
                            break;
                    case 1: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS;   /*PortB base address*/
                            break;
                    case 2: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS;   /*PortC base address*/
                            break;
                    case 3: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS;   /*PortD base address*/
                            break;
                    case 4: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS;   /*PortE base address*/
                            break;
                    case 5: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS;   /*PortF base address*/
                            break;
                }
                if(STD_OFF == Mode)
                {
                        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET), Pin);
                        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET), Pin);
                }
                else if(STD_ON == Mode)
                {
                        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET), Pin);
                        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET), Pin);
                }
                else
                {
                            /* No Action Required */
                }
            }
        }
}



