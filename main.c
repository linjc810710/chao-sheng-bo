/**
  ******************************************************************************
  * @file    Project/main.c 
  * @author  MCD Application Team
  * @version V2.1.0
  * @date    18-November-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 


/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include "stdio.h"

/* Private defines -----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
#ifdef _RAISONANCE_
#define PUTCHAR_PROTOTYPE int putchar (char c)
#define GETCHAR_PROTOTYPE int getchar (void)
#elif defined (_COSMIC_)
#define PUTCHAR_PROTOTYPE char putchar (char c)
#define GETCHAR_PROTOTYPE char getchar (void)
#else /* _IAR_ */
#define PUTCHAR_PROTOTYPE int putchar (int c)
#define GETCHAR_PROTOTYPE int getchar (void)
#endif /* _RAISONANCE_ */

#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(u8* file, u32 line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/* Private functions ---------------------------------------------------------*/
#define LED_GPIO_PORT     (GPIOB)
#define LED_CTRL_PORT     (GPIOD)
#define HSR_GPIO_PORT     (GPIOC)
#define HSR_TRIG_PINS     (GPIO_PIN_7)
#define HSR_GET_PINS      (GPIO_PIN_5)
#define HSR_POWER_PINS    (GPIO_PIN_6)
#define HSR_GET_SIGNAL()   GPIO_ReadInputPin(HSR_GPIO_PORT,HSR_GET_PINS)   //超声波获取信号,非零值代表1，零值代表0
#define LED_DISPLAY_TIME   4000

#define LED_CS0_ON()         GPIO_WriteLow(LED_CTRL_PORT,GPIO_PIN_7)
#define LED_CS0_OFF()        GPIO_WriteHigh(LED_CTRL_PORT,GPIO_PIN_7)
#define LED_CS1_ON()         GPIO_WriteLow(LED_CTRL_PORT,GPIO_PIN_2)
#define LED_CS1_OFF()        GPIO_WriteHigh(LED_CTRL_PORT,GPIO_PIN_2)
#define LED_CS2_ON()         GPIO_WriteLow(LED_CTRL_PORT,GPIO_PIN_3)
#define LED_CS2_OFF()        GPIO_WriteHigh(LED_CTRL_PORT,GPIO_PIN_3)
#define LED_CS3_ON()         GPIO_WriteLow(LED_CTRL_PORT,GPIO_PIN_4)
#define LED_CS3_OFF()        GPIO_WriteHigh(LED_CTRL_PORT,GPIO_PIN_4)
#define LED_DATE(X)          GPIO_Write(LED_GPIO_PORT,X)
#define HSR_POWER_ON()       GPIO_WriteLow (HSR_GPIO_PORT,HSR_POWER_PINS);
#define HSR_POWER_OFF()      GPIO_WriteHigh (HSR_GPIO_PORT,HSR_POWER_PINS);

const uint8_t Led_Table[10] = {0x03,0x9f,0x25,0x0D,0x99,0x49,0x41,0x1f,0x01,0x09}; // 对应0~9；


uint8_t ghsr_time_over = 0;     // 用于指示时间溢出
uint8_t gled_flag = 0;         //用于超声波的数码显示刷频标志
uint32_t value = 0;
uint8_t  dis_buff[4] = {0};      //存放距离的数组，单位是厘米，XXX.X
 
// @brief Retargets the C library printf function to the UART.
// @param c Character to send
// @retval char Character sent

 PUTCHAR_PROTOTYPE
 {
    /* Write a character to the UART1 */
    UART1_SendData8(c);
    /* Loop until the end of transmission */
    while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET); 
    return (c);
 }

 GETCHAR_PROTOTYPE
 {
 #ifdef _COSMIC_
    char c = 0;
 #else
    int c = 0;
#endif
    /* Loop until the Read data register flag is SET */
    while (UART1_GetFlagStatus(UART1_FLAG_RXNE) == RESET);
    c = UART1_ReceiveData8();
    return (c);
 }

void Time2Isr(void)
{
  ghsr_time_over = 1;  
}
 
void Time4Isr(void)
{
   static uint8_t count = 0;
   count++;
   if(count > 5)          // 20ms的刷新
   {
     gled_flag = 1;
   }
}

//延时函数
void Delay(uint16_t nCount)
{
  // Decrement nCount value 
  while (nCount != 0)
  {   
    nCount--;
  }
}

//超声波初始化
void  HSR_Init(void)
{
     GPIO_Init(HSR_GPIO_PORT,HSR_TRIG_PINS, GPIO_MODE_OUT_PP_LOW_FAST);
     GPIO_Init(HSR_GPIO_PORT,HSR_GET_PINS, GPIO_MODE_IN_PU_NO_IT); 
     GPIO_Init(HSR_GPIO_PORT,HSR_POWER_PINS, GPIO_MODE_OUT_PP_LOW_FAST);
}

//20US超声波触发信号
 void HSR_Trig(void)
 {
     GPIO_WriteHigh(HSR_GPIO_PORT,HSR_TRIG_PINS);
     Delay(60);
     GPIO_WriteLow(HSR_GPIO_PORT,HSR_TRIG_PINS);
 }
 
// Time2用于超声波计时的时间函数初始化
void HSR_Time2_Init(void)
{
   CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER2,ENABLE);
   TIM2_TimeBaseInit(TIM2_PRESCALER_16,0xffff);
   TIM2_PrescalerConfig(TIM2_PRESCALER_16, TIM2_PSCRELOADMODE_IMMEDIATE);
  /* Clear TIM2 update flag */
   TIM2_ClearFlag(TIM2_FLAG_UPDATE);
  /* Enable update interrupt */
   TIM2_ITConfig(TIM2_IT_UPDATE, ENABLE);
  /* enable interrupts */
  // enableInterrupts();  
}

// LED数码管刷新定时器4函数初始化
void LED_Time4_Init(void)
{
   CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER4,ENABLE);
   TIM4_TimeBaseInit(TIM4_PRESCALER_128,249);     // 接近2MS
  /* Clear TIM4 update flag */
   TIM4_ClearFlag(TIM4_FLAG_UPDATE);
  /* Enable update interrupt */
   TIM4_ITConfig(TIM4_IT_UPDATE, ENABLE);   
}
void LED_Init(void)
{
  GPIO_Init(LED_GPIO_PORT,GPIO_PIN_7,GPIO_MODE_OUT_PP_HIGH_FAST);
  GPIO_Init(LED_GPIO_PORT,GPIO_PIN_6,GPIO_MODE_OUT_PP_HIGH_FAST);
  GPIO_Init(LED_GPIO_PORT,GPIO_PIN_5,GPIO_MODE_OUT_PP_HIGH_FAST);
  GPIO_Init(LED_GPIO_PORT,GPIO_PIN_4,GPIO_MODE_OUT_PP_HIGH_FAST);
  GPIO_Init(LED_GPIO_PORT,GPIO_PIN_3,GPIO_MODE_OUT_PP_HIGH_FAST);
  GPIO_Init(LED_GPIO_PORT,GPIO_PIN_2,GPIO_MODE_OUT_PP_HIGH_FAST);
  GPIO_Init(LED_GPIO_PORT,GPIO_PIN_1,GPIO_MODE_OUT_PP_HIGH_FAST);
  GPIO_Init(LED_GPIO_PORT,GPIO_PIN_0,GPIO_MODE_OUT_PP_HIGH_FAST);
  GPIO_Init(LED_CTRL_PORT,GPIO_PIN_7,GPIO_MODE_OUT_PP_HIGH_FAST);
  GPIO_Init(LED_CTRL_PORT,GPIO_PIN_2,GPIO_MODE_OUT_PP_HIGH_FAST);
  GPIO_Init(LED_CTRL_PORT,GPIO_PIN_3,GPIO_MODE_OUT_PP_HIGH_FAST);
  GPIO_Init(LED_CTRL_PORT,GPIO_PIN_4,GPIO_MODE_OUT_PP_HIGH_FAST); 
}
//-------------------------
//----Init the board 
//----ljc/2015/06/20
//--------------------------
void FunctionInit(void)
{
    CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);            // 使用内部16M晶振   
#if 0    //串口初始化
    UART1_DeInit();  
    UART1_Init((uint32_t)115200, UART1_WORDLENGTH_8D, UART1_STOPBITS_1, UART1_PARITY_NO,
                UART1_SYNCMODE_CLOCK_DISABLE, UART1_MODE_TXRX_ENABLE);
#endif
     HSR_Init();
     LED_Init();
     HSR_Time2_Init();
     LED_Time4_Init();
     enableInterrupts();
}

//-------计算声波距离---------
void Cal_Distance(void)
{
  uint8_t tmp;
  uint8_t first_none_zero = 0;        // 第一位非零标志
  if(value < 114)
  {
    value = 114;
  }
  value = value*174/1000;           // 将距离值放大10倍.单位cm
  tmp = value/1000;
  if(tmp == 0)
  {
    dis_buff[0] = 0xff; 
  }
  else
  {
    dis_buff[0] = Led_Table[tmp];
    first_none_zero = 1;
  }
  tmp = value/100%10;
  if(tmp == 0)
  {
     if(first_none_zero == 1)
     {
       dis_buff[1] =  Led_Table[tmp];
     }
     else
     {
       dis_buff[1] = 0xff;
     }    
  }
  else
  {
     dis_buff[1] = Led_Table[tmp];
     first_none_zero = 1;
  }
  tmp = (value % 100)/10;
  dis_buff[2] = Led_Table[tmp]& 0xfe;     // 强制增加小数点
  tmp = (value % 100)%10;
  dis_buff[3] = Led_Table[tmp]; 
}

//------------led数码管显示------
void Led_Display(void)
{
  LED_CS0_ON();
  LED_DATE(dis_buff[0]);
  Delay(LED_DISPLAY_TIME);
  LED_CS0_OFF();  
  LED_CS1_ON();
  LED_DATE(dis_buff[1]);
  Delay(LED_DISPLAY_TIME);  
  LED_CS1_OFF();
  LED_CS2_ON();
  LED_DATE(dis_buff[2]);
  Delay(LED_DISPLAY_TIME);
  LED_CS2_OFF();
  LED_CS3_ON();
  LED_DATE(dis_buff[3]);
  Delay(LED_DISPLAY_TIME); 
  LED_CS3_OFF();
}

//----------入口主函数-----------
void main(void)
{
 // LED指示引脚初始化
//  GPIO_Init(LED_GPIO_PORT, (GPIO_Pin_TypeDef)LED_GPIO_PINS, GPIO_MODE_OUT_PP_LOW_FAST);
//  GPIO_WriteReverse(LED_GPIO_PORT, (GPIO_Pin_TypeDef)LED_GPIO_PINS);
  uint8_t display_count = 0;
  FunctionInit();
  while (1)
  {
  FIRST:
    TIM2_SetCounter(0);
    HSR_Trig();
    TIM4_Cmd(ENABLE);
    while(HSR_GET_SIGNAL()== 0)
    {
      if(gled_flag == 1)
      {
        gled_flag = 0;
        display_count++;
      }
      if(display_count> 15)
      {
        display_count = 0;
        TIM4_Cmd(DISABLE);
        HSR_POWER_OFF();
        Delay(4000);
        HSR_POWER_ON();
        Delay(4000); 
        goto FIRST;        
      }
    }
    TIM2_Cmd(ENABLE);
    while(HSR_GET_SIGNAL()!= 0)
    {
      if(ghsr_time_over == 1)
      {
         TIM2_Cmd(DISABLE);
         ghsr_time_over = 0;
         HSR_POWER_OFF();
         Delay(4000);
         HSR_POWER_ON();
         Delay(4000);
         goto FIRST;
      }      
    }
    TIM2_Cmd(DISABLE);
    value = TIM2_GetCounter();
    Cal_Distance();
    TIM4_SetCounter(0);
    TIM4_Cmd(ENABLE);  
    while(display_count++ < 50)
    {
      while(gled_flag == 0);
      Led_Display();
      gled_flag = 0;     
    }
    display_count = 0;
    TIM4_Cmd(DISABLE);
   }
}
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
