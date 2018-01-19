/**
  ******************************************************************************
  * @file    TIM_TimeBase/stm32f4xx_it.h 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    19-September-2011
  * @brief   This file contains the headers of the interrupt handlers.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIMERS_H
#define __TIMERS_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

//void TIM3_Config(void);
void InitTimers(void);

//void TIM3_IRQHandler(void);
void TIM2_IRQHandler(void);

void SetStepFrequencyRaClock(double frequency);
void SetGoToFrequencyRa(long frequency);
void SetGoToFrequencyDe(long frequency);
//void SetGuideFrequencies(double freqRa, double freqDe);
void SetGuideFrequencyRa(double frequency);
void SetGuideFrequencyDe(double frequency);

//void SetEnableRa(uint8_t value);
//void SetEnableDe(uint8_t value);


#endif /* __TIMERS_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
