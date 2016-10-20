/******************** (C) COPYRIGHT 2013 www.armjishu.com  ********************
 * �ļ���  ��main.c
 * ʵ��ƽ̨��STM32���ۿ�����
 * ��׼��  ��STM32F10x_StdPeriph_Driver V3.5.0
 * ����    ��www.armjishu.com 
**********************************************************************************/
#include "SZ_STM32F103RB_LIB.h"
#include "LCD_ZK.h"
#include "nRF24L01_API.h"
#include "sys.h"
#include "string.h"

uchar rece_buf[32];
volatile uchar Key_pass_flag = 0;


void RTCAlarm_NVIC_Configuration(void);
void RTCAlarm_Configuration(void);
void SYSCLKConfig_STOP(void);

/*******************************************************************************
* Function Name  : InterruptConfig
* Description    : Configures the used IRQ Channels and sets their priority.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void InterruptConfig(void)
{   
  /* Set the Vector Table base address at 0x08000000 */
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x00000);			 /*�����ж�������ʼ��ַ*/
}
void LED_config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  /* Enable GPIOB */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); /*ʹ��LED��ʹ�õ�GPIOʱ��*/

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void Led_Turn_on_all(void)
{
	/* Turn On All LEDs */
	GPIO_ResetBits(GPIOB,GPIO_Pin_8);/*�������е�LEDָʾ��*/
}

void Led_Turn_off_all(void)
{
	/* Turn Off All LEDs */
	GPIO_SetBits(GPIOB,GPIO_Pin_8);
}
/**-------------------------------------------------------
  * @������ RTC_STANDBY_Configuration
  * @����   RTC��STANDBYģʽ��ʼ��
  * @����   ��
  * @����ֵ ��
***------------------------------------------------------*/
void RTC_STANDBY_Configuration(void)
{
    /* Enable PWR and BKP clocks */
    /* PWRʱ�ӣ���Դ���ƣ���BKPʱ�ӣ�RTC�󱸼Ĵ�����ʹ�� */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);

    /* Allow access to BKP Domain */
    /*ʹ��RTC�ͺ󱸼Ĵ������� */
    PWR_BackupAccessCmd(ENABLE);

    /* Enable WKUP pin */
    PWR_WakeUpPinCmd(ENABLE);


    /* Check if the StandBy flag is set */
    if(PWR_GetFlagStatus(PWR_FLAG_SB) != RESET)
    {/* System resumed from STANDBY mode */

        /* Turn on LED2 */
//        SZ_STM32_LEDOn(LED2);

//        printf("\r\n resumed from StandBy mode..��STANDBY����ģʽ�˳�");

        /* Clear StandBy flag */
        PWR_ClearFlag(PWR_FLAG_SB);

        /* Wait for RTC APB registers synchronisation */
        RTC_WaitForSynchro();
        /* No need to configure the RTC as the RTC configuration(clock source, enable,
        prescaler,...) is kept after wake-up from STANDBY */
    }
    else
    {/* StandBy flag is not set */
//        printf("\r\n resumed from Reset..������ģʽ��λ");
        /* RTC clock source configuration ----------------------------------------*/

        /* Reset Backup Domain */
        /* ������BKP��ȫ���Ĵ�������Ϊȱʡֵ */
        //BKP_DeInit();

        /* Enable LSE */
        /* ʹ��LSE���ⲿ32.768KHz���پ���*/
        RCC_LSEConfig(RCC_LSE_ON);

        /* Wait till LSE is ready */
        /* �ȴ��ⲿ�������ȶ���� */
        while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
        {
        }

        /* Select LSE as RTC Clock Source */
        /*ʹ���ⲿ32.768KHz������ΪRTCʱ�� */
        RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

        /* Enable RTC Clock */
        /* ʹ�� RTC ��ʱ�ӹ��� */
        RCC_RTCCLKCmd(ENABLE);

        /* RTC configuration -----------------------------------------------------*/
        /* Wait for RTC APB registers synchronisation */
        /*�ȴ�RTC�Ĵ���ͬ�� */
        RTC_WaitForSynchro();


        /* Set RTC prescaler: set RTC period to 1sec */
        /* 32.768KHz����Ԥ��Ƶֵ��32767,����Ծ���Ҫ��ܸ߿����޸Ĵ˷�Ƶֵ��У׼���� */
        RTC_SetPrescaler(32767); /* RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1) */

        /* Wait until last write operation on RTC registers has finished */
        /* �ȴ���һ�ζ�RTC�Ĵ�����д������� */
        RTC_WaitForLastTask();

    }
}


#define NRF_IRQ   PBin(5)  //IRQ������������,��������

uchar rece_buf[32];

void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

#ifdef  VECT_TAB_RAM  
    /* Set the Vector Table base location at 0x20000000 */ 
    NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0); 
#else  /* VECT_TAB_FLASH  */
    /* Set the Vector Table base location at 0x08000000 */ 
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);   
#endif

    /* Configure one bit for preemption priority */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

    /* Enable the EXTI0 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
void SPI_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_Configuration();
    /* ��������I��LED��ʹ�õ�GPIO�ܽ�ģʽ*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); /*ʹ��LED��ʹ�õ�GPIOʱ��*/

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_15;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);  /*GPIO�ڳ�ʼ��*/

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14|GPIO_Pin_5; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);  /*GPIO�ڳ�ʼ��*/
}


void My_KEYInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable the BUTTON Clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);

    /* Configure Button pin as input floating */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* Connect Button EXTI Line to Button GPIO Pin */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource13);

    /* Configure Button EXTI line */
    EXTI_InitStructure.EXTI_Line = EXTI_Line13;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* Enable and set Button EXTI Interrupt to the lowest priority */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure); 
}

uchar send_buf[32]={0x4,'W','X','Y','Z',0};

void work(void)
{
    long wait_rf_rx_count=0;

	LED_config();
    My_KEYInit();
	SPI_GPIO_Init();
	Led_Turn_on_all();
	NRF24L01_Wakeup();
	NRF24L01_RT_Init();	//160uS
	lcd_wakeup();
	//display_string_5x7(0,0,"   ");
    for(wait_rf_rx_count = 0;wait_rf_rx_count < 0x8000;wait_rf_rx_count++){
        if(NRF_IRQ==0)
        {	
            wait_rf_rx_count = 0;
            if(NRF24L01_RxPacket(rece_buf)==0)
            {
                display_string_5x7(0,0,&rece_buf[1]);
                if(rece_buf[1] == '1'){

                }
                
                switch(rece_buf[1]){
                case '1':
                    display_128x64(bmpflag1);
                    break;
                case '2':
                    display_128x64(bmpflag2);
                    break;
                case '3':
                    display_128x64(bmpflag3);
                    break;
                case '4':
                    display_128x64(bmpflag4);
                    break;
                case '5':
                    display_128x64(bmpflag5);
                    break;

                default:
                    display_128x64(bmp1);
                    break;
                }
                SEND_BUF(rece_buf);
            }
        }
        if(Key_pass_flag){
            Key_pass_flag = 0;
            display_string_5x7(0,0,&send_buf[1]);
            SEND_BUF(send_buf);
            //wait_rf_rx_count = 0;
        }        
    }

}


int main(void)
{
    //PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);
    SYSCLKConfig_STOP();
    //400uS
    LED_config();
    Led_Turn_off_all();
    SPI_GPIO_Init();//12uS
    initial_lcd();//214uS
    SZ_STM32_COMInit(COM1, 115200);	 /* ����1��ʼ�� *///30uS
    clear_screen();    //clear all dots //7.8mS
    while(NRF24L01_Check()); // �ȴ���⵽NRF24L01������Ż�����ִ��
    NRF24L01_RT_Init();	//160uS
    My_KEYInit();
    display_128x64(bmp1);//7.4mS
    /* RTC���ӳ�ʼ�� */
    RTCAlarm_Configuration();
    /* ����RTC�����ж��������� */
    RTCAlarm_NVIC_Configuration();
    InterruptConfig();/*�����ж�������ʼ��ַ*/

    /* Main loop */
    while (1)
    {
        /* RTC�жϴ������Ͱ���1�жϴ�����λ��"stm32f10x_it.c"�ļ��� */

        /* Wait till RTC Second event occurs */
        RTC_ClearFlag(RTC_FLAG_SEC);
        //while(RTC_GetFlagStatus(RTC_FLAG_SEC) == RESET);

        /* Alarm in 5 second */
        RTC_SetAlarm(RTC_GetCounter()+ 1);
        /* Wait until last write operation on RTC registers has finished */
        RTC_WaitForLastTask();

        {
            Led_Turn_off_all();
            lcd_sleep();
            NRF24L01_Sleep();
            USART_DeInit(USART1);
            //RCC_PLLCmd(DISABLE);
            //RCC_DeInit();
            //RCC_APB1PeriphClockCmd(RCC_APB1Periph_ALL, DISABLE);
            //RCC_APB2PeriphClockCmd(RCC_APB2Periph_ALL, DISABLE);
        }

        /* Request to enter STOP mode with regulator in low power mode*/
        PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);

        /* Configures system clock after wake-up from STOP: enable HSE, PLL and select 
            PLL as system clock source (HSE and PLL are disabled in STOP mode) */
        //SYSCLKConfig_STOP();
        /* �˴���������û��ĳ��� */
        work();
    }
}

/**-------------------------------------------------------
  * @������ RTCAlarm_NVIC_Configuration
  * @����   ����RTC�����ж�������������
  * @����   ��
  * @����ֵ ��
***------------------------------------------------------*/
void RTCAlarm_NVIC_Configuration(void)
{
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Configure EXTI Line17(RTC Alarm) to generate 
                      an interrupt on rising edge */
    EXTI_ClearITPendingBit(EXTI_Line17);
    EXTI_InitStructure.EXTI_Line = EXTI_Line17;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* 2 bits for Preemption Priority and 2 bits for Sub Priority */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    /* Enable the RTC Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = RTCAlarm_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/**-------------------------------------------------------
  * @������ RTCAlarm_Configuration
  * @����   RTC���ӳ�ʼ��
  * @����   ��
  * @����ֵ ��
***------------------------------------------------------*/
void RTCAlarm_Configuration(void)
{
    /* Enable PWR and BKP clocks */
    /* PWRʱ�ӣ���Դ���ƣ���BKPʱ�ӣ�RTC�󱸼Ĵ�����ʹ�� */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);

    /* Allow access to BKP Domain */
    /*ʹ��RTC�ͺ󱸼Ĵ������� */
    PWR_BackupAccessCmd(ENABLE);

    /* Reset Backup Domain */
    /* ������BKP��ȫ���Ĵ�������Ϊȱʡֵ */
    //BKP_DeInit();

    /* Enable LSE */
    /* ʹ��LSE���ⲿ32.768KHz���پ���*/
    RCC_LSEConfig(RCC_LSE_ON);

    /* Wait till LSE is ready */
    /* �ȴ��ⲿ�������ȶ���� */
    while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
    {}

    /* Select LSE as RTC Clock Source */
    /*ʹ���ⲿ32.768KHz������ΪRTCʱ�� */
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

    /* Enable RTC Clock */
    /* ʹ�� RTC ��ʱ�ӹ��� */
    RCC_RTCCLKCmd(ENABLE);

    /* Wait for RTC registers synchronization */
    /*�ȴ�RTC�Ĵ���ͬ�� */
    RTC_WaitForSynchro();

    /* Wait until last write operation on RTC registers has finished */
    /* �ȴ���һ�ζ�RTC�Ĵ�����д������� */
    RTC_WaitForLastTask();

    /* Enable the RTC Alarm interrupt */
    /* ʹ��RTC�������ж� */
    RTC_ITConfig(RTC_IT_ALR, ENABLE);

    /* Wait until last write operation on RTC registers has finished */
    /* �ȴ���һ�ζ�RTC�Ĵ�����д������� */
    RTC_WaitForLastTask();

    /* Set RTC prescaler: set RTC period to 1sec */
    /* 32.768KHz����Ԥ��Ƶֵ��32767,����Ծ���Ҫ��ܸ߿����޸Ĵ˷�Ƶֵ��У׼���� */
    RTC_SetPrescaler(32767); /* RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1) */

    /* Wait until last write operation on RTC registers has finished */
    /* �ȴ���һ�ζ�RTC�Ĵ�����д������� */
    RTC_WaitForLastTask();
}

/**
  * @brief  Configures system clock after wake-up from STOP: enable HSE, PLL
  *         and select PLL as system clock source.
  * @param  None
  * @retval None
  */
void SYSCLKConfig_STOP(void)
{
    ErrorStatus HSEStartUpStatus;

    /* Enable HSE */
    RCC_HSEConfig(RCC_HSE_ON);

    /* Wait till HSE is ready */
    HSEStartUpStatus = RCC_WaitForHSEStartUp();

    if(HSEStartUpStatus == SUCCESS)
    {

    #ifdef STM32F10X_CL
        /* Enable PLL2 */ 
        RCC_PLL2Cmd(ENABLE);

        /* Wait till PLL2 is ready */
        while(RCC_GetFlagStatus(RCC_FLAG_PLL2RDY) == RESET)
        {
        }

    #endif

        /* Enable PLL */ 
        RCC_PLLCmd(ENABLE);

        /* Wait till PLL is ready */
        while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
        {
        }

        /* Select PLL as system clock source */
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

        /* Wait till PLL is used as system clock source */
        while(RCC_GetSYSCLKSource() != 0x08)
        {
        }
    }
}
