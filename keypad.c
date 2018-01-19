/**
  ******************************************************************************
  * @file    keypad.c 
  * @author  Tiziano Niero
  * @version V1.0.0
  * @date    february-2014
  * @brief   Keypad management.
  *          This file provides template the exti interrupt service routine used
  *          by the keypad buttons.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "keypad.h"
#include "stm32f4xx_it.h"
#include "stm32f4_discovery.h"
#include "commands.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MAX_DEBOUNCE_CHECKS     2


#define KEYPAD_GPIO_CLK         RCC_AHB1Periph_GPIOE  
#define KEYPAD_PORT             GPIOE

#define KEYPAD_PIN_W            GPIO_Pin_11
#define KEYPAD_PIN_E            GPIO_Pin_13
#define KEYPAD_PIN_N            GPIO_Pin_12
#define KEYPAD_PIN_S            GPIO_Pin_14
#define KEYPAD_PIN_INCR         GPIO_Pin_10
#define KEYPAD_PIN_DECR         GPIO_Pin_15
#define KEYPAD_MASK             (KEYPAD_PIN_W|KEYPAD_PIN_E|KEYPAD_PIN_N|KEYPAD_PIN_S|KEYPAD_PIN_INCR|KEYPAD_PIN_DECR)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint16_t stateQueue[MAX_DEBOUNCE_CHECKS];         // Array that maintains bounce status
static int stateIndex = 0;                          // Pointer into State

static uint8_t oldKeyW = 1;
static uint8_t oldKeyE = 1;
static uint8_t oldKeyN = 1;
static uint8_t oldKeyS = 1;

static uint8_t commandMoveW[3] = "MW\n";
static uint8_t commandMoveE[3] = "ME\n";
static uint8_t commandMoveN[3] = "MN\n";
static uint8_t commandMoveS[3] = "MS\n";
static uint8_t commandMoveEndRa[3] = "m\n\0";
static uint8_t commandMoveEndDe[3] = "n\n\0";

/* Private function prototypes -----------------------------------------------*/
void InitGPIO(void);
uint16_t debounce_switch(uint16_t bounced_state);

// --------------------------------------------------------------
//
// PUBLIC METHODS
//
// --------------------------------------------------------------

void InitKeypad()
{
  InitGPIO();
}


void ScanKeypad()
{
  uint16_t keys = GPIO_ReadInputData(KEYPAD_PORT);
  keys &= KEYPAD_MASK;
  
  keys = debounce_switch(keys);
  if((keys & KEYPAD_PIN_W) == 0 && oldKeyW == 1)
  {
    oldKeyW = 0;
    PushCommand(commandMoveW, 3);
  }
  else if((keys & KEYPAD_PIN_W) > 0 && oldKeyW == 0)
  {
    oldKeyW = 1;
    PushCommand(commandMoveEndRa, 3);
  }
  if((keys & KEYPAD_PIN_E) == 0 && oldKeyE == 1)
  {
    oldKeyE = 0;
    PushCommand(commandMoveE, 3);
  }
  else if((keys & KEYPAD_PIN_E) > 0 && oldKeyE == 0)
  {
    oldKeyE = 1;
    PushCommand(commandMoveEndRa, 3);
  }
  if((keys & KEYPAD_PIN_N) == 0 && oldKeyN == 1)
  {
    oldKeyN = 0;
    PushCommand(commandMoveN, 3);
  }
  else if((keys & KEYPAD_PIN_N) > 0 && oldKeyN == 0)
  {
    oldKeyN = 1;
    PushCommand(commandMoveEndDe, 3);
  }
  if((keys & KEYPAD_PIN_S) == 0 && oldKeyS == 1)
  {
    oldKeyS = 0;
    PushCommand(commandMoveS, 3);
  }
  else if((keys & KEYPAD_PIN_S) > 0 && oldKeyS == 0)
  {
    oldKeyS = 1;
    PushCommand(commandMoveEndDe, 3);
  }
 /*
    if((keys & KEYPAD_PIN_W) == 0 && oldKeyW == 1)
  {
    oldKeyW = 0;
    PushCommand(commandMoveW, 3);
  }
  else if((keys & KEYPAD_PIN_W) > 0 && oldKeyW == 0)
  {
    oldKeyW = 1;
  }
  if((keys & KEYPAD_PIN_E) == 0 && oldKeyE == 1)
  {
    oldKeyE = 0;
    PushCommand(commandMoveE, 3);
  }
  else if((keys & KEYPAD_PIN_E) > 0 && oldKeyE == 0)
  {
    oldKeyE = 1;
  }
  if((keys & KEYPAD_PIN_N) == 0 && oldKeyN == 1)
  {
    oldKeyN = 0;
    PushCommand(commandMoveN, 3);
  }
  else if((keys & KEYPAD_PIN_N) > 0 && oldKeyN == 0)
  {
    oldKeyN = 1;
  }
  if((keys & KEYPAD_PIN_S) == 0 && oldKeyS == 1)
  {
    oldKeyS = 0;
    PushCommand(commandMoveS, 3);
  }
  else if((keys & KEYPAD_PIN_S) > 0 && oldKeyS == 0)
  {
    oldKeyS = 1;
  }
  if((oldKeyW & oldKeyE & oldKeyN & oldKeyS) == 1)
  {
  }
 
*/
}
// --------------------------------------------------------------
//
// PRIVATE FUNCTIONS
//
// --------------------------------------------------------------


/**
  * @brief  Configures EXTI Line0 (connected to PA0 pin) in interrupt mode
  * @param  None
  * @retval None
  */
void InitGPIO(void)
{
  
  GPIO_InitTypeDef   GPIO_InitStructure;

  // Enable GPIOA clock 
  RCC_AHB1PeriphClockCmd(KEYPAD_GPIO_CLK, ENABLE);
  // Enable SYSCFG clock 
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  
  // Configure PA0 pin as input floating 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Pin = KEYPAD_MASK;
  GPIO_Init(KEYPAD_PORT, &GPIO_InitStructure);

  /*
  // Connect EXTI Line0 to PA0 pin 
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);

  // Configure EXTI Line0 
  EXTI_InitTypeDef   EXTI_InitStructure;
  EXTI_InitStructure.EXTI_Line = EXTI_Line0;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;// EXTI_Trigger_Rising;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  // Enable and set EXTI Line0 Interrupt to the lowest priority 
  NVIC_InitTypeDef   NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  */
}


// input debounce routine
uint16_t debounce_switch(uint16_t bouncedState)
{
    uint16_t debouncedState = 0xFFFF;                   // Debounced state of the switches
    
    stateQueue[stateIndex++] = bouncedState;
    if(stateIndex >= MAX_DEBOUNCE_CHECKS)
        stateIndex = 0;
    for(int i = 0; i < MAX_DEBOUNCE_CHECKS; i++)
        debouncedState &= stateQueue[i];
    return debouncedState;
}

// --------------------------------------------------------------
//
// INTERRUPT HANDLERS
//
// --------------------------------------------------------------


/**
  * @brief  This function handles External line 0 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI0_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line0) != RESET)
  {
    uint8_t bitstatus = GPIO_ReadInputDataBit(EXTI_PortSourceGPIOA, EXTI_PinSource0);
    if(bitstatus)
      STM_EVAL_LEDOn(LED4);
    else
      STM_EVAL_LEDOff(LED4);
        
    /* Toggle LED4 */
    STM_EVAL_LEDToggle(LED4);
    /* Clear the EXTI line 0 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line0);
  }
}
