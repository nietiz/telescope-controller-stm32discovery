/**
  ******************************************************************************
  * @file    system_time.c 
  * @author  Tiziano Niero
  * @version V1.0.0
  * @date    22-January-2014
  * @brief   manages systick exceptions handler some utility routines.
  ******************************************************************************
  * 
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
//#include "stm32f4xx_it.h"
#include "stm32f4_discovery.h"
#include "system_time.h"
//#include "telescope_control.h"
#include "keypad.h"


/* Private function prototypes -----------------------------------------------*/
static void TimingDelay_Decrement(void);

__IO uint32_t TimingDelay;






/* Private functions ---------------------------------------------------------*/


/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  ScanKeypad();
  //STM_EVAL_LEDToggle(LED6);
  TimingDelay_Decrement();
}


/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
static void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}



/* Public functions ---------------------------------------------------------*/

void InitSystemTime()
{
  RCC_ClocksTypeDef RCC_Clocks;  
  /* SysTick end of count event each 10ms */
  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000);
}


/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in 10 ms.
  * @retval None
  */
void Delay(__IO uint32_t nTime)
{
  TimingDelay = nTime;

  while(TimingDelay != 0);
}

