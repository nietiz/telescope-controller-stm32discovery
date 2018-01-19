/**
  ******************************************************************************
  * @file    system_time.h
  * @author  Tiziano Niero
  * @version V1.1.0
  * @date    22-January-2014
  * @brief   Header for telescope_control.c file.
  ******************************************************************************
  * @attention
  *
  *   
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SYSTEM_TIME_H
#define __SYSTEM_TIME_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4_discovery.h"
/* Exported typef ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void InitSystemTime();
void Delay(__IO uint32_t nTime);



#endif /* __SYSTEM_TIME_H */


