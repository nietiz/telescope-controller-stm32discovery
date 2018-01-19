/**
  ******************************************************************************
  * @file    telescope_control.h
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
#ifndef __TELESCOPE_CONTROL_H
#define __TELESCOPE_CONTROL_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4_discovery.h"
/* Define ------------------------------------------------------------*/

#define TELESCOPE_GPIO_CLK      RCC_AHB1Periph_GPIOD  
#define TELESCOPE_GPIO_PORT     GPIOD  

#define RA_DIR_PIN              GPIO_Pin_0
#define RA_STEP_PIN             GPIO_Pin_2
#define DE_DIR_PIN              GPIO_Pin_1
#define DE_STEP_PIN             GPIO_Pin_3

#define HALL_SENSOR_PORT        GPIOA

#define HALL_RA_PIN             GPIO_Pin_15
#define HALL_DE_PIN             GPIO_Pin_14


/* Exported typef ------------------------------------------------------------*/
typedef enum 
{
  GUIDE_SPEEDTYPE_NONE = 0,
  GUIDE_SPEEDTYPE_SIDEREAL = 1,
  GUIDE_SPEEDTYPE_SUN = 2,
  GUIDE_SPEEDTYPE_MOON = 3
} GUIDE_SPEEDTYPE;



/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void InitTelescopeControl();
void SetReference(int referenceArcsecRa, int referenceArcsecDe);
void StartGoTo(int targetArcsecRa, int targetArcsecDe);
void AbortGoTo();
void UpdateAccelerationSpeed(int intervalMs);
void MoveRaOn(int direction);
void MoveRaOff();
void MoveDeOn(int direction);
void MoveDeOff();
double GetFrequencyRa();
double GetFrequencyDe();

void SetGuideSpeed(int index);
void SetCorrectionSpeed(int index);
void UpdateClockRa();
void PulseRa();
void PulseDe();
long GetCurrentArcsecDe();
long GetCurrentArcsecRa();
void TransmitStatus(uint8_t what);
void TransmitPosition();
void StartSpiralSearch(int armArcsec, int nSteps, uint8_t returnToStart);
void CheckSpiralSearchPosition();
void AbortSpiralSearch();

uint8_t RunningMotorRa();
uint8_t RunningMotorDe();
#endif /* __TELESCOPE_CONTROL_H */