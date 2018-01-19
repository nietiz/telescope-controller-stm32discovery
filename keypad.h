/**
  ******************************************************************************
  * @file    keypad.h
  * @author  Tiziano Niero
  * @version V1.1.0
  * @date    February-2014
  * @brief   Header for keypad.c file.
   ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __KEYPAD_H
#define __KEYPAD_H

/* Includes ------------------------------------------------------------------*/
/* Define ------------------------------------------------------------*/
/*
#define TELESCOPE_GPIO_CLK      RCC_AHB1Periph_GPIOD  
#define TELESCOPE_GPIO_PORT     GPIOD  

#define RA_DIR_PIN              GPIO_Pin_0
#define RA_STEP_PIN             GPIO_Pin_2
#define DE_DIR_PIN              GPIO_Pin_1
#define DE_STEP_PIN             GPIO_Pin_3
*/

/* Exported typef ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void InitKeypad();
void ScanKeypad();

#endif /* __KEYPAD_H */