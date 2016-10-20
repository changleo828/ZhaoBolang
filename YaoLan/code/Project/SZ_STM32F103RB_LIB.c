/********************   (C) COPYRIGHT 2013 www.armjishu.com   ********************
 * 文件名  ：SZ_STM32F103RB_LIB.c
 * 描述    ：提供STM32F103RB神舟I号开发板的库函数
 * 实验平台：STM32神舟开发板
 * 作者    ：www.armjishu.com 
**********************************************************************************/
  
/* Includes ------------------------------------------------------------------*/
#include "SZ_STM32F103RB_LIB.h"

__IO uint32_t TimingDelay;

USART_TypeDef* COM_USART[COMn] = {SZ_STM32_COM1}; 
GPIO_TypeDef* COM_TX_PORT[COMn] = {SZ_STM32_COM1_TX_GPIO_PORT};
GPIO_TypeDef* COM_RX_PORT[COMn] = {SZ_STM32_COM1_RX_GPIO_PORT};
const uint32_t COM_USART_CLK[COMn] = {SZ_STM32_COM1_CLK};
const uint32_t COM_TX_PORT_CLK[COMn] = {SZ_STM32_COM1_TX_GPIO_CLK};
const uint32_t COM_RX_PORT_CLK[COMn] = {SZ_STM32_COM1_RX_GPIO_CLK};
const uint16_t COM_TX_PIN[COMn] = {SZ_STM32_COM1_TX_PIN};
const uint16_t COM_RX_PIN[COMn] = {SZ_STM32_COM1_RX_PIN};

/**-------------------------------------------------------
  * @函数名 delay
  * @功能   简单的delay延时函数.
  * @参数   延迟周期数 0--0xFFFFFFFF
  * @返回值 无
***------------------------------------------------------*/
void delay(__IO uint32_t nCount)
{
  for (; nCount != 0; nCount--);
}

/**-------------------------------------------------------
  * @函数名 __SZ_STM32_COMInit
  * @功能   对STM32的USART初始化底层函数
  * @参数1  COM1  对应STM32的USART1 对应开发板上串口1
  * @参数2  指向一个成员已赋值USART_InitTypeDef结构体的指针
  * @返回值 无
***------------------------------------------------------*/
void __SZ_STM32_COMInit(COM_TypeDef COM, USART_InitTypeDef* USART_InitStruct)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable GPIO clock */
    /* 使能STM32的USART对应GPIO的Clock时钟 */
    RCC_APB2PeriphClockCmd(COM_TX_PORT_CLK[COM] | COM_RX_PORT_CLK[COM] | RCC_APB2Periph_AFIO, ENABLE);

    /* 使能STM32的USART1的Clock时钟 */
    RCC_APB2PeriphClockCmd(COM_USART_CLK[COM], ENABLE); 

    /* Configure USART Tx as alternate function push-pull */
    /* 初始化STM32的USART的TX管脚，配置为复用功能推挽输出 */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = COM_TX_PIN[COM];
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(COM_TX_PORT[COM], &GPIO_InitStructure);

    /* Configure USART Rx as input floating */
    /* 初始化STM32的USART的RX管脚，配置为复用功能输入 */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = COM_RX_PIN[COM];
    GPIO_Init(COM_RX_PORT[COM], &GPIO_InitStructure);

    /* USART configuration */
    /* 根据传入的参数初始化STM32的USART配置 */
    USART_Init(COM_USART[COM], USART_InitStruct);

    /* Enable USART */
    /* 使能STM32的USART功能模块 */
    USART_Cmd(COM_USART[COM], ENABLE);
}

/**-------------------------------------------------------
  * @函数名 __SZ_STM32_COMInit
  * @功能   面向用户的STM32的USART初始化函数
  * @参数1  COM1  对应STM32的USART1 对应开发板上串口1
  * @参数2  BaudRate 串口的波特率，例如"115200"
  * @返回值 无
***------------------------------------------------------*/
void SZ_STM32_COMInit(COM_TypeDef COM, uint32_t BaudRate)
{
  
    USART_InitTypeDef USART_InitStructure;

    USART_InitStructure.USART_BaudRate = BaudRate;              //串口的波特率，例如115200 最高达4.5Mbits/s
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; //数据字长度(8位或9位)
    USART_InitStructure.USART_StopBits = USART_StopBits_1;      //可配置的停止位-支持1或2个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;         //无奇偶校验  
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //无硬件流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //双工模式，使能发送和接收
  
    __SZ_STM32_COMInit(COM, &USART_InitStructure);  // 调用STM32的USART初始化底层函数

    //printf("\n\r ############ WWW.ARMJISHU.COM! ############ \n");
 
}
/*
    加入以下代码,支持printf函数,不需要选择use MicroLIB	  
*/
#ifndef MicroLIB
#pragma import(__use_no_semihosting)   //没有实现fgetc时需要声明该参数          
/* 标准库需要的支持函数 使用printf()调试打印不需要实现该函数 */               
struct __FILE 
{ 
	int handle; 
    /* Whatever you require here. If the only file you are using is */    
    /* standard output using printf() for debugging, no file handling */    
    /* is required. */
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
_sys_exit(int x) 
{ 
	x = x; 
} 
/* 重定义fputc函数 如果使用MicroLIB只需要重定义fputc函数即可 */  
int fputc(int ch, FILE *f)
{
    /* Place your implementation of fputc here */
    /* Loop until the end of transmission */
    while (USART_GetFlagStatus(SZ_STM32_COM1, USART_FLAG_TC) == RESET)
    {}

    /* e.g. write a character to the USART */
    USART_SendData(SZ_STM32_COM1, (uint8_t) ch);

    return ch;
}
/*
可以直接使用putchar
不需要再定义 int putchar(int ch)，因为stdio.h中有如下定义
 #define putchar(c) putc(c, stdout)
*/

int ferror(FILE *f) {  
    /* Your implementation of ferror */  
    return EOF;  
} 
#endif 

#ifdef  USE_FULL_ASSERT
// 需要在工程设置Option(快捷键ALT+F7)C++属性页的define栏输入"USE_FULL_ASSERT"
/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
     
    printf("Wrong parameters value: file %s on line %d\r\n", file, line);

    /* Infinite loop */
    while (1)
    {
    }
}
#endif

