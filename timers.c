/**
  ******************************************************************************
  * @file    TIM_TimeBase/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    19-September-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
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
#include "timers.h"
#include "stm32f4_discovery.h"
#include "telescope_control.h"

/* Extern variables -----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/


#define TIMER_CLOCK             1000000 // Hz - timer clock
#define PULSE_WIDTH             10      // counts - number of counts for the output high side => 10 counts @ 500KHz == 20µs
#define RA_CLOCK_COUNTS         500     // counts - number of counts for the RA clock => 500 counts @ 500KHz == 1ms
//#define DISABLED_FREQUENCY      100     // Hz - timer output frequency when disabled
#define DISABLED_STEP_COUNT     10000   // number of counts for a disabled timer running at 100Hz := TIMER_CLOCK / 100(Hz)

/* Private macro -------------------------------------------------------------*/
#define ROUND(p)                (uint32_t)((p)+0.5)

#define GET_RA_STATUS           (GPIO_ReadInputDataBit(TELESCOPE_GPIO_PORT, RA_STEP_PIN))
#define RA_SET                  (GPIO_SetBits(TELESCOPE_GPIO_PORT, RA_STEP_PIN))
#define RA_RESET                (GPIO_ResetBits(TELESCOPE_GPIO_PORT, RA_STEP_PIN))
#define GET_DE_STATUS           (GPIO_ReadInputDataBit(TELESCOPE_GPIO_PORT, DE_STEP_PIN))
#define DE_SET                  (GPIO_SetBits(TELESCOPE_GPIO_PORT, DE_STEP_PIN))
#define DE_RESET                (GPIO_ResetBits(TELESCOPE_GPIO_PORT, DE_STEP_PIN))

/* Private function prototypes -----------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

// 480*200*16 / 86164 = 17.826; 500000 / 17.826 = 28048
//static __IO uint32_t TIM2_CCR1_Val = 5000;// 28048; // velocità siderale a 16µstep/step
static __IO uint32_t TIM2_CCR_Val_RaClock = DISABLED_STEP_COUNT;
static __IO uint32_t TIM2_CCR_Val_Ra = DISABLED_STEP_COUNT;
static __IO uint32_t TIM2_CCR_Val_De = DISABLED_STEP_COUNT;
static __IO uint32_t TIM2_CCR4_Val = TIMER_CLOCK / 100;
static __IO uint32_t guideStepRa = DISABLED_STEP_COUNT;
static __IO uint32_t guideStepDe = DISABLED_STEP_COUNT;
static __IO uint8_t enableRaClock = 0;
static __IO uint8_t enableRa = 0;
static __IO uint8_t enableDe = 0;

static uint32_t captureTim2 = 0;
/*
static __IO uint32_t guideSpeedRa;
static __IO uint32_t guideSpeedDe;
*/
#if 0
static uint16_t capture = 0;
static __IO uint32_t TIM3_CCR1_Val = 5000;
static __IO uint32_t TIM3_CCR2_Val = 5000;
static __IO uint32_t TIM3_CCR3_Val = 2500;
static __IO uint32_t TIM3_CCR4_Val = 25000;
#endif





/**
  * @brief  Configure the timer TIM2. TIM2 has 32bit resolution counters.
  * @param  None
  * @retval None
   -----------------------------------------------------------------------
    TIM2 Configuration: Output Compare Timing Mode:
    
    In this example TIM2 input clock (TIM2CLK) is set to 2 * APB1 clock (PCLK1), 
    since APB1 prescaler is different from 1.   
      TIM2CLK = 2 * PCLK1  
      PCLK1 = HCLK / 4 
      => TIM2CLK = HCLK / 2 = SystemCoreClock /2
  Note: 
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
     Each time the core clock (HCLK) changes, user had to call SystemCoreClockUpdate()
     function to update SystemCoreClock variable value. Otherwise, any configuration
     based on this variable will be incorrect.    
  -----------------------------------------------------------------------
*/
void InitTimers(void)
{
  // ------- Configure the TIM2 IRQ Handler ----------
  // enable the clock
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  // enable the global Interrupt 
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  // -------- Configure the Time base ---------
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_TimeBaseStructure.TIM_Period = 4294967295;        // 2^32-1
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  // -------- Configure the Prescaler ---------
  // SystemCoreClock = 168000000 Hz, TIM2 Clock = 500000 Hz => PrescalerValue = 167
  uint16_t PrescalerValue = 0;
  PrescalerValue = (uint16_t) ((SystemCoreClock / 2) / TIMER_CLOCK) - 1;
  TIM_PrescalerConfig(TIM2, PrescalerValue, TIM_PSCReloadMode_Immediate);
  
  // -------- Configure the Output Compare Mode (Timing) ---------
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  // Channel1
  TIM_OCInitStructure.TIM_Pulse = RA_CLOCK_COUNTS;
  TIM_OC1Init(TIM2, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Disable);
  // Channel2
  TIM_OCInitStructure.TIM_Pulse = TIM2_CCR_Val_Ra;
  TIM_OC2Init(TIM2, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Disable);
  // Channel3
  TIM_OCInitStructure.TIM_Pulse = TIM2_CCR_Val_De;
  TIM_OC3Init(TIM2, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Disable);
  // Channel4
  TIM_OCInitStructure.TIM_Pulse = TIM2_CCR4_Val;
  TIM_OC4Init(TIM2, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Disable);
  // -------- Finalize --------
  // enable interrupts
  TIM_ITConfig(TIM2, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4 , ENABLE);

  // enable counter
  TIM_Cmd(TIM2, ENABLE);  
}

void SetStepFrequencyRaClock(double frequency)
{
  assert_param(frequency >= 0);
  TIM2_CCR_Val_RaClock = ROUND(TIMER_CLOCK / frequency);   
  if(frequency > 0)
    enableRaClock = 1;
  else
    enableRaClock = 0;
}

void SetGoToFrequencyRa(long frequency)
{
    TIM2_CCR_Val_Ra = TIMER_CLOCK / frequency;   
    enableRa = 1;
}

void SetGoToFrequencyDe(long frequency)
{
    TIM2_CCR_Val_De = TIMER_CLOCK / frequency;
    enableDe = 1;
}



void SetGuideFrequencyRa(double frequency)
{
  if(frequency > 0)
  {
    TIM2_CCR_Val_Ra = ROUND(TIMER_CLOCK / frequency);   
    enableRa = 1;
  }
  else
  {
    TIM2_CCR_Val_Ra = DISABLED_STEP_COUNT;   
    enableRa = 0;
  }
}

/*
void StartGuideRa()
{
  if(guideStepRa > 0)
  {
    TIM2_CCR_Val_Ra = guideStepRa;   
    enableRa = 1;
  }
  else
  {    
    TIM2_CCR_Val_Ra = DISABLED_STEP_COUNT;   
    enableRa = 0;
  }
}
*/

void SetGuideFrequencyDe(double frequency)
{
  if(frequency > 0)
  {
    TIM2_CCR_Val_De = ROUND(TIMER_CLOCK / frequency);
    enableDe = 1;
  }
  else
  {
    TIM2_CCR_Val_De = DISABLED_STEP_COUNT;
    enableDe = 0;
  }
}
/*
void SetGuideFrequencies(double freqRa, double freqDe)
{
  if(freqRa > 0)
  {
    TIM2_CCR_Val_Ra = ROUND(TIMER_CLOCK / freqRa);   
    enableRa = 1;
  }
  else
  {
    TIM2_CCR_Val_Ra = DISABLED_STEP_COUNT;   
    enableRa = 0;
  }
  if(freqDe > 0)
  {
    TIM2_CCR_Val_De = ROUND(TIMER_CLOCK / freqDe);
    enableDe = 1;
  }
  else
  {
    TIM2_CCR_Val_De = DISABLED_STEP_COUNT;
    enableDe = 0;
  }
}
*/

/******************************************************************************/
/*            STM32F4xx Peripherals Interrupt Handlers                        */
/******************************************************************************/
/**
  * @brief  This function handles TIM3 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM2_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET)
  {
    // ------------- RA CLOCK -------------
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
//    STM_EVAL_LEDToggle(LED4);
    captureTim2 = TIM_GetCapture1(TIM2);
    TIM_SetCompare1(TIM2, captureTim2 + TIM2_CCR_Val_RaClock);
    if(enableRaClock)
      UpdateClockRa();
  }
  else if (TIM_GetITStatus(TIM2, TIM_IT_CC2) != RESET)
  {
    // --------- RA MOTOR STEPPING ---------
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
    captureTim2 = TIM_GetCapture2(TIM2);
   
    if(enableRa)
    {
      if(GET_RA_STATUS == 0)
      {
        RA_SET;
        PulseRa();
        TIM_SetCompare2(TIM2, captureTim2 + PULSE_WIDTH);
        STM_EVAL_LEDToggle(LED3);   // orange
      }
      else
      {
        RA_RESET;
        STM_EVAL_LEDOff(LED3);   // orange
        TIM_SetCompare2(TIM2, captureTim2 + TIM2_CCR_Val_Ra - PULSE_WIDTH);
      }
    }
    else
    {
      RA_RESET;
      STM_EVAL_LEDOff(LED3);   // orange
      TIM_SetCompare2(TIM2, captureTim2 + TIM2_CCR_Val_Ra);
    }
  }
  else if (TIM_GetITStatus(TIM2, TIM_IT_CC3) != RESET)
  {
    // --------- DE MOTOR STEPPING ---------
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC3);
    captureTim2 = TIM_GetCapture3(TIM2);

    if(enableDe)
    {
      STM_EVAL_LEDToggle(LED5); // red
      if(GPIO_ReadInputDataBit(LED5_GPIO_PORT, LED5_PIN))
      {
        DE_SET;
        PulseDe();
        TIM_SetCompare3(TIM2, captureTim2 + PULSE_WIDTH);
      }
      else
      {
        DE_RESET;
        TIM_SetCompare3(TIM2, captureTim2 + TIM2_CCR_Val_De - PULSE_WIDTH);
      }
    }
    else
    {
      DE_RESET;
      STM_EVAL_LEDOff(LED5); // red
      TIM_SetCompare3(TIM2, captureTim2 + TIM2_CCR_Val_De);
    }
  }
  else
  {
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC4);
//    STM_EVAL_LEDToggle(LED6);
    UpdateAccelerationSpeed(10);
    CheckSpiralSearchPosition();
    captureTim2 = TIM_GetCapture4(TIM2);
    TIM_SetCompare4(TIM2, captureTim2 + TIM2_CCR4_Val);
  }

}

#if 0
/**
  * @brief  This function handles TIM3 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM3_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
//    STM_EVAL_LEDToggle(LED4);
    TC_UpdateClockRa();
    capture = TIM_GetCapture1(TIM3);
    TIM_SetCompare1(TIM3, capture + TIM3_CCR1_Val);
  }
  else if (TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET)
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
    STM_EVAL_LEDToggle(LED3);
    TC_PulseRa();
    capture = TIM_GetCapture2(TIM3);
    TIM_SetCompare2(TIM3, capture + TIM3_CCR2_Val);
  }
  else if (TIM_GetITStatus(TIM3, TIM_IT_CC3) != RESET)
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);
    STM_EVAL_LEDToggle(LED5);
    TC_PulseDe();
    capture = TIM_GetCapture3(TIM3);
    TIM_SetCompare3(TIM3, capture + TIM3_CCR3_Val);
  }
  else
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);
    STM_EVAL_LEDToggle(LED6);
    capture = TIM_GetCapture4(TIM3);
    TIM_SetCompare4(TIM3, capture + TIM3_CCR4_Val);
  }
}


/**
  * @brief  Configure the TIM IRQ Handler.
  * @param  None
  * @retval None
  */
void TIM3_Config(void)
{
  uint16_t PrescalerValue = 0;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  /* Enable the TIM3 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* -----------------------------------------------------------------------
    TIM3 Configuration: Output Compare Timing Mode:
    
    In this example TIM3 input clock (TIM3CLK) is set to 2 * APB1 clock (PCLK1), 
    since APB1 prescaler is different from 1.   
      TIM3CLK = 2 * PCLK1  
      PCLK1 = HCLK / 4 
      => TIM3CLK = HCLK / 2 = SystemCoreClock /2
          
    To get TIM3 counter clock at 500 KHz, the prescaler is computed as follows:
       Prescaler = (TIM3CLK / TIM3 counter clock) - 1
       Prescaler = ((SystemCoreClock /2) / 500 KHz) - 1
                                              
    CC1 update rate = TIM3 counter clock / CCR1_Val = 9.154 Hz
    ==> Toggling frequency = 4.57 Hz
    
    CC2 update rate = TIM3 counter clock / CCR2_Val = 18.31 Hz
    ==> Toggling frequency = 9.15 Hz
    
    CC3 update rate = TIM3 counter clock / CCR3_Val = 36.62 Hz
    ==> Toggling frequency = 18.31 Hz
    
    CC4 update rate = TIM3 counter clock / CCR4_Val = 73.25 Hz
    ==> Toggling frequency = 36.62 Hz

    Note: 
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
     Each time the core clock (HCLK) changes, user had to call SystemCoreClockUpdate()
     function to update SystemCoreClock variable value. Otherwise, any configuration
     based on this variable will be incorrect.    
  ----------------------------------------------------------------------- */  

  /* Compute the prescaler value */
//  PrescalerValue = (uint16_t) ((SystemCoreClock / 2) / 500000) - 1;
  PrescalerValue = (uint16_t) ((SystemCoreClock / 2) / 500000) - 1;

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 65535;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  /* Prescaler configuration */
  TIM_PrescalerConfig(TIM3, PrescalerValue, TIM_PSCReloadMode_Immediate);

  /* Output Compare Timing Mode configuration: Channel1 */
//  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Toggle;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
//  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;

  TIM_OC1Init(TIM3, &TIM_OCInitStructure);

  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Disable);
  

  /* Output Compare Timing Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR2_Val;

  TIM_OC2Init(TIM3, &TIM_OCInitStructure);

  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Disable);

  /* Output Compare Timing Mode configuration: Channel3 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR3_Val;

  TIM_OC3Init(TIM3, &TIM_OCInitStructure);

  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Disable);

  /* Output Compare Timing Mode configuration: Channel4 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR4_Val;

  TIM_OC4Init(TIM3, &TIM_OCInitStructure);

  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Disable);

  
  /* TIM Interrupts enable */
  //TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4, ENABLE);
    TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);

  /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);

  
}

#endif



/* Private functions ---------------------------------------------------------*/
