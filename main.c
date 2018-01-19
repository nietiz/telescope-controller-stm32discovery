/**
  ******************************************************************************
  * @file    TIM_TimeBase/main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    19-September-2011
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
#include "stm32f4_discovery.h"
#include "system_time.h"
#include "telescope_control.h"
#include "timers.h"
#include "keypad.h"
#include "remote_control.h"
#include "commands.h"
#include "utils.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void ProcessCommandQueue();

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */


int main(void)
{

/*!< At this stage the microcontroller clock setting is already configured, 
     this is done through SystemInit() function which is called from startup
     file (startup_stm32f4xx.s) before to branch to application main.
     To reconfigure the default setting of SystemInit() function, refer to
     system_stm32f4xx.c file
*/
 
  // Initialize Leds mounted on STM32F4-Discovery board
  STM_EVAL_LEDInit(LED4);       // green
  STM_EVAL_LEDInit(LED3);       // orange
  STM_EVAL_LEDInit(LED5);       // red
  STM_EVAL_LEDInit(LED6);       // blue

  InitSystemTime();
  InitTelescopeControl();
  InitTimers();
  InitKeypad();
  InitRemoteControl();
  
  while (1)
  {
    ProcessCommandQueue();
  }
}


void ProcessCommandQueue()
{
  int value1;
  int value2;
  if(GetCommandCount() > 0)
  {
    uint8_t* cmd = PopCommand();
    
    switch(cmd[0])
    {
    case 'S':   // query status
      TransmitStatus(cmd[1]);
      break;
    case 's':   // start/stop spiral search
      if(cmd[1] == 'b')
      {
        // Start SpiralSearch
        // command format: sbxxxxyyyz\n, where x is the spiral arm in arcsec and y the number of steps;
        // z=='1' => return to start point after spiral completion; z=='0' stay at final point
        value1 = ParseInt(cmd, 2, 4);
        value2 = ParseInt(cmd, 6, 3);
        StartSpiralSearch(value1, value2, cmd[9] - '0');
      }
      else if(cmd[1] == 'e')
      {
        // Abort SpiralSearch: the telescope stays at the point where Spiral Search is stopped
        // command format: se\n
        AbortSpiralSearch();
      }
      break;
    case 'R':   // set reference
      // reference: format R+aaaaaaa±ddddddd\n, where 
      // a is RA in arcsec (with zero padding)
      // d is DE in arcsec (with zero padding)
      value1 = ParseInt(cmd, 1, 8);
      value2 = ParseInt(cmd, 9, 8);
      SetReference(value1, value2);
      break;
    case 'G':   // goto
      // goto: format G+aaaaaaa±ddddddd\n, where 
      // a is RA in arcsec (with zero padding)
      // d is DE in arcsec (with zero padding)
      value1 = ParseInt(cmd, 1, 8);
      value2 = ParseInt(cmd, 9, 8);
      StartGoTo(value1, value2);
      break;
    case 'A':   // abort goto
      AbortGoTo();
      break;
    case 'P':   // query position
      TransmitPosition();
      break;
    case 'T':
      if(cmd[1] >= '0' && cmd[1] <= '3')
        SetGuideSpeed(cmd[1] - '0');
      break;
    case 'C':   // set correction speed: index from 0 to 5
      if(cmd[1] >= '0' && cmd[1] <= '5')
        SetCorrectionSpeed(cmd[1] - '0');
      break;
    case 'M':   // start manual move
      switch(cmd[1])
      {
      case 'W':
        MoveRaOn(1);
        break;
      case 'E':
        MoveRaOn(0);
        break;
      case 'N':
        MoveDeOn(1);
        if(cmd[2] == 'W')
          MoveRaOn(1);
        if(cmd[2] == 'E')
          MoveRaOn(0);        
        break;
      case 'S':
        MoveDeOn(0);        
        if(cmd[2] == 'W')
          MoveRaOn(1);
        if(cmd[2] == 'E')
          MoveRaOn(0);        
        break;
      }
      break;
    case 'm':   // end manual move RA
      MoveRaOff();
      break;
    case 'n':   // end manual move DE
      MoveDeOff();
      break;
    case 'o':   // end manual move both
      MoveRaOff();
      MoveDeOff();
      break;
    }
  }
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  while (1)
  {}
}
#endif

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
