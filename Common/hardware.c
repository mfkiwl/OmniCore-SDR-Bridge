/********************************** (C) COPYRIGHT  *******************************
* File Name          : ch32h417_crc.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/03/01
* Description        : This file provides all the CRC firmware functions.
*********************************************************************************
* Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "hardware.h"
#include "ch32h417_usbss_device.h"
#include "ch32h417_usbhs_device.h"
#include "hd3ss3220.h"
#include "usbss_uart1_bridge.h"
#include "usb_ep3_debug.h"
#include "usb_desc.h"
/*********************************************************************
 * @fn      USART1_Init
 *
 * @brief   Initializes the USART1 peripheral.
 *
 * @param   baudrate - USART communication baud rate.
 *
 * @return  none
 */
void USART1_Init(uint32_t baudrate)
{
    UART1_Bridge_Init(baudrate);
} 

void LED_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure={0};

    RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOA, ENABLE);

    /* PA4 LED is active high; keep it off until 3 A is confirmed. */
    GPIO_ResetBits(GPIOA, GPIO_Pin_4);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Very_High;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

 /*********************************************************************
 * @fn      Hardware
 *
 * @brief   Resets the CRC Data register (DR).
 *
 * @param   none
 *
 * @return  none
 */
void Hardware( void )
{ 

    /* Disable SWD */
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_AFIO, ENABLE);
    Delay_Ms(3000);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE); //注释掉用于调试

    /* Enable the IO function of PC6 pins */
    RCC_HB1PeriphClockCmd(RCC_HB1Periph_SWPMI, ENABLE);
    SWPMI->OR |= SWPMI_SWP_TBYP;

    USART1_Init(UART1_BRIDGE_BAUD);

    /* 先初始化 USB */
    printf("USB Init\r\n");
    USB_SerialNumber_Init();
    USB_Timer_Init();
    USBSS_Device_Init(ENABLE);

    /* Either controller may complete enumeration; keep servicing UART/logs. */
    printf("Wait USB enumeration...\r\n");
    while(USBSS_DevEnumStatus == 0 && USBHS_DevEnumStatus == 0)
    {
        UART1_Bridge_Task();
        USB_Debug_Task();
        Delay_Ms(1);
    }

    /* 枚举完成后，再初始化并启动 UHSIF */
    printf("USB%u ready, UHSIF Init\r\n", USBSS_DevEnumStatus ? 3U : 2U);
    UHSIF_Init();

    LED_Init();
    
    #include "hd3ss3220.h"

    HD3SS3220_Result ret;
    HD3SS3220_Status typec_status;

    ret = HD3SS3220_Init();

    if (ret == HD3SS3220_OK)
    {
        ret = HD3SS3220_PrintInfo();  /* 读取并打印芯片信息和状态 */

        /* Startup snapshot: 0x08[5:4] = 11b means the source offers 3 A.
         * Detected current is valid here only while attached as UFP/Sink. */
        ret = HD3SS3220_ReadStatus(&typec_status);
        if (ret == HD3SS3220_OK &&
            typec_status.attached == HD3SS3220_ATTACHED_UFP &&
            typec_status.current_detected == 3u)
        {
            GPIO_SetBits(GPIOA, GPIO_Pin_4);
        }
    }


	while (1)
    {
        UART_Proc();
    }
		
}
 /*********************************************************************
 * @fn      UART_Proc
 *
 * @brief   Serial port-related code.
 *
 * @param   none
 *
 * @return  none
 */
void UART_Proc( void )
{
    UART1_Bridge_Task();
    USB_Debug_Task();
}