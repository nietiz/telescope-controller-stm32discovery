/**
  ******************************************************************************
  * @file    telescope_control.c 
  * @author  Tiziano Niero
  * @version V1.0.0
  * @date    22-january-2014
  * @brief   telescope control routines
  ******************************************************************************
  * @attention
  * 1 sidereal day = 86164.0916 secs
  * 
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "telescope_control.h"
#include "timers.h"
#include "remote_control.h"
#include "commands.h"
#include "utils.h"
    
#include <stdlib.h>

/* public variables ----------------------------------------------------------*/



/* Private define ------------------------------------------------------------*/
#define DEBUG_CONTROL           1

#define SOLAR_DAY_SECS          86400.0
#define SIDERAL_DAY_SECS        86164.0916
#define MOON_DAY_SECS           89520.0         // first approximation
#define FULL_CIRCLE_ARCSECS     1296000         // 360*60*60 = angolo giro in arcsecs
#define HALF_PI_ARCSECS         324000          // 90*60*60 = 90° in arcsecs        
#define MAX_CORRECTION_SPEEDS   6
#define MAX_GUIDE_SPEEDS        4               // defined by GUIDE_SPEEDTYPE enum
#define FORWARD                 1
#define REVERSE                 0
#define END_GOTO_CORRECTIONSPEED_INDEX  4

/* Private typedef -----------------------------------------------------------*/


typedef enum 
{
  MOTOR_STATUS_GUIDE,
  MOTOR_STATUS_CORRECTION,
  MOTOR_STATUS_GOTO
} MOTOR_STATUS;
  

typedef struct 
{
  long primaryReductionRatio;
  double secondaryReductionRatio;
  long totalMotorSteps;
  long deadSteps;
} AxisSetup;


typedef struct
{
  long maxSteps;             // full circle steps
  double stepToArcsec;          // conversion factor steps -> arcsecs
  double guideStepFrequency[MAX_GUIDE_SPEEDS];
  long correctionStepFrequency[MAX_CORRECTION_SPEEDS];
  long gotoStepFrequency;
  long accelerationTime;        // acceleration/deceleration time (ms)
  long maxAccelerationSteps;    // max number of steps for acceleration + deceleration (both linear ramp, constant accel.)
  uint8_t directionUp;
  uint8_t directionDown;
} AxisProperties;

typedef struct
{
  long stepPosition;
  long stepFrequency;         // current step frequency
  uint8_t direction;
  MOTOR_STATUS motorStatus;
  long accelerationSteps;
  long stepPositionTarget;
} AxisStatus;



/* Private macro -------------------------------------------------------------*/
#define ROUND(p)                ((p)> 0 ? (long)((p)+0.5) : (long)((p)-0.5))
#define SET_RA_DIRECTION(p)     GPIO_WriteBit(TELESCOPE_GPIO_PORT, RA_DIR_PIN, ((BitAction)p))
#define SET_DE_DIRECTION(p)     GPIO_WriteBit(TELESCOPE_GPIO_PORT, DE_DIR_PIN, ((BitAction)p))

/* Private variables ---------------------------------------------------------*/
static uint8_t buffer[MAX_COMMAND_LENGTH];

static AxisProperties axisPropertiesRa;
static AxisProperties axisPropertiesDe;
static AxisStatus axisStatusRa;
static AxisStatus axisStatusDe;
static AxisSetup axisSetupRa;
static AxisSetup axisSetupDe;


static int currentCorrectionSpeedIndex = 3; // x 16
static int currentGuideSpeedIndex;

static uint8_t spiralSearchOn = 0;
static int currentSpiralSearchStep;
static int spiralSearchStartingRa;
static int spiralSearchStartingDe;
static int spiralSearchArmArcsec;
static int spiralSearchSteps;
static uint8_t spiralSearchReturnToStartingPoint;
static uint8_t spiralSearchStepComplete;




/* Private function prototypes -----------------------------------------------*/
void LoadSettings();
void InitGpioPins();
void UpdatePositionRa();
void UpdatePositionDe();
void TerminateRaGoTo();
void TerminateDeGoTo();
void StepForwardSpiralSearch();


// --------------------------------------------------------------
//
// PUBLIC METHODS
//
// --------------------------------------------------------------

void LoadSettings()
{
  axisSetupRa.deadSteps = 0;
  axisSetupRa.primaryReductionRatio = 480;
  axisSetupRa.secondaryReductionRatio = 4.0;
  axisSetupRa.totalMotorSteps = 200 * 8;

  axisSetupDe.deadSteps = 0;
  axisSetupDe.primaryReductionRatio = 480;
  axisSetupDe.secondaryReductionRatio = 4.0;
  axisSetupDe.totalMotorSteps = 200 * 8;
}

/**
  * @brief  initialize telescope control
  * @param  None
  * @retval None
  */
void InitTelescopeControl()
{
  InitGpioPins();
  LoadSettings();
    
  axisPropertiesRa.maxSteps = ROUND(axisSetupRa.primaryReductionRatio * axisSetupRa.secondaryReductionRatio * axisSetupRa.totalMotorSteps);
  axisPropertiesRa.guideStepFrequency[GUIDE_SPEEDTYPE_NONE] = 0;
  axisPropertiesRa.guideStepFrequency[GUIDE_SPEEDTYPE_SIDEREAL] = (double)axisPropertiesRa.maxSteps / SIDERAL_DAY_SECS;
  axisPropertiesRa.guideStepFrequency[GUIDE_SPEEDTYPE_SUN] = (double)axisPropertiesRa.maxSteps / SOLAR_DAY_SECS;
  axisPropertiesRa.guideStepFrequency[GUIDE_SPEEDTYPE_MOON] = (double)axisPropertiesRa.maxSteps / MOON_DAY_SECS;
  axisPropertiesRa.correctionStepFrequency[0] = ROUND(axisPropertiesRa.guideStepFrequency[GUIDE_SPEEDTYPE_SIDEREAL] * 0.75);
  axisPropertiesRa.correctionStepFrequency[1] = ROUND(axisPropertiesRa.guideStepFrequency[GUIDE_SPEEDTYPE_SIDEREAL] * 1.5);
  axisPropertiesRa.correctionStepFrequency[2] = ROUND(axisPropertiesRa.guideStepFrequency[GUIDE_SPEEDTYPE_SIDEREAL] * 4);
  axisPropertiesRa.correctionStepFrequency[3] = ROUND(axisPropertiesRa.guideStepFrequency[GUIDE_SPEEDTYPE_SIDEREAL] * 16);
  axisPropertiesRa.correctionStepFrequency[4] = ROUND(axisPropertiesRa.guideStepFrequency[GUIDE_SPEEDTYPE_SIDEREAL] * 32);
  axisPropertiesRa.correctionStepFrequency[5] = ROUND(axisPropertiesRa.guideStepFrequency[GUIDE_SPEEDTYPE_SIDEREAL] * 64);
  axisPropertiesRa.stepToArcsec = (double)FULL_CIRCLE_ARCSECS / (double)axisPropertiesRa.maxSteps;
  axisPropertiesRa.gotoStepFrequency = axisSetupDe.totalMotorSteps * 4;//8
  axisPropertiesRa.accelerationTime = 1000;
  axisPropertiesRa.maxAccelerationSteps = ROUND(axisPropertiesRa.accelerationTime * axisPropertiesRa.gotoStepFrequency / 1000);
  axisPropertiesRa.directionUp = FORWARD;
  axisPropertiesRa.directionDown = REVERSE;
 
  axisPropertiesDe.maxSteps = ROUND(axisSetupDe.primaryReductionRatio * axisSetupDe.secondaryReductionRatio * axisSetupDe.totalMotorSteps / 4);
  axisPropertiesDe.guideStepFrequency[GUIDE_SPEEDTYPE_NONE] = 0;
  axisPropertiesDe.guideStepFrequency[GUIDE_SPEEDTYPE_SIDEREAL] = 0;
  axisPropertiesDe.guideStepFrequency[GUIDE_SPEEDTYPE_SUN] = 0;
  axisPropertiesDe.guideStepFrequency[GUIDE_SPEEDTYPE_MOON] = 0;
  axisPropertiesDe.correctionStepFrequency[0] = ROUND((double)axisPropertiesRa.maxSteps / SIDERAL_DAY_SECS * 0.75);
  axisPropertiesDe.correctionStepFrequency[1] = ROUND((double)axisPropertiesRa.maxSteps / SIDERAL_DAY_SECS * 1.5);
  axisPropertiesDe.correctionStepFrequency[2] = ROUND(axisPropertiesDe.correctionStepFrequency[1] * 4);
  axisPropertiesDe.correctionStepFrequency[3] = ROUND(axisPropertiesDe.correctionStepFrequency[1] * 16);
  axisPropertiesDe.correctionStepFrequency[4] = ROUND(axisPropertiesDe.correctionStepFrequency[1] * 32);
  axisPropertiesDe.correctionStepFrequency[5] = ROUND(axisPropertiesDe.correctionStepFrequency[1] * 64);
  axisPropertiesDe.stepToArcsec = (double)HALF_PI_ARCSECS / (double)axisPropertiesDe.maxSteps;
  axisPropertiesDe.gotoStepFrequency = axisSetupDe.totalMotorSteps * 4;//8
  axisPropertiesDe.accelerationTime = 1000;
  axisPropertiesDe.maxAccelerationSteps = ROUND(axisPropertiesDe.accelerationTime * axisPropertiesDe.gotoStepFrequency / 1000);
  axisPropertiesDe.directionUp = FORWARD;
  axisPropertiesDe.directionDown = REVERSE;
 
  axisStatusRa.stepPosition = 0;
  axisStatusRa.stepPositionTarget = 0;
  axisStatusDe.stepPosition = 0;
  axisStatusDe.stepPositionTarget = 0;
  
  axisStatusRa.motorStatus = MOTOR_STATUS_GUIDE;
  axisStatusDe.motorStatus = MOTOR_STATUS_GUIDE;
  
  spiralSearchOn = 0;
  
  SetGuideSpeed(GUIDE_SPEEDTYPE_NONE);
  SetStepFrequencyRaClock(axisPropertiesRa.guideStepFrequency[GUIDE_SPEEDTYPE_SIDEREAL]);
}

void SetReference(int referenceArcsecRa, int referenceArcsecDe)
{
  if(spiralSearchOn != 0)
    return;
  if(axisStatusRa.motorStatus != MOTOR_STATUS_GUIDE || axisStatusDe.motorStatus != MOTOR_STATUS_GUIDE)
    return;
  axisStatusRa.stepPosition = ROUND((double)referenceArcsecRa / axisPropertiesRa.stepToArcsec);
  if(axisStatusRa.stepPosition > axisPropertiesRa.maxSteps)
    axisStatusRa.stepPosition -= axisPropertiesRa.maxSteps;
  axisStatusDe.stepPosition = ROUND((double)referenceArcsecDe / axisPropertiesDe.stepToArcsec);
}

void StartGoTo(int targetArcsecRa, int targetArcsecDe)
{
  if(spiralSearchOn != 0)
    return;
  if(axisStatusRa.motorStatus != MOTOR_STATUS_GUIDE || axisStatusDe.motorStatus != MOTOR_STATUS_GUIDE)
    return;
  long delta;

  axisStatusRa.stepPositionTarget = ROUND((double)targetArcsecRa / axisPropertiesRa.stepToArcsec);
  if(axisStatusRa.stepPositionTarget > axisPropertiesRa.maxSteps)
    axisStatusRa.stepPositionTarget -= axisPropertiesRa.maxSteps;
  axisStatusDe.stepPositionTarget = ROUND((double)targetArcsecDe / axisPropertiesDe.stepToArcsec);
  // RA
  if(axisStatusRa.stepPosition != axisStatusRa.stepPositionTarget)
  {
    if(axisStatusRa.stepPositionTarget > axisStatusRa.stepPosition)
      axisStatusRa.direction = axisPropertiesRa.directionUp;
    else
      axisStatusRa.direction = axisPropertiesRa.directionDown;
    delta = abs(axisStatusRa.stepPositionTarget - axisStatusRa.stepPosition);
    if(delta > axisPropertiesRa.maxSteps / 2)
    {
      // invert direction
      axisStatusRa.direction = axisStatusRa.direction == axisPropertiesRa.directionUp ? axisPropertiesRa.directionDown : axisPropertiesRa.directionUp;
      delta = axisPropertiesRa.maxSteps - delta;        
    }

    SET_RA_DIRECTION(axisStatusRa.direction);
    axisStatusRa.motorStatus = MOTOR_STATUS_GOTO;   
    axisStatusRa.stepFrequency = 0;
    if(delta < axisPropertiesRa.maxAccelerationSteps / 2)
      axisStatusRa.accelerationSteps = delta / 2;
    else
      axisStatusRa.accelerationSteps = axisPropertiesRa.maxAccelerationSteps / 2;
  }
  // DEC
  if(axisStatusDe.stepPosition != axisStatusDe.stepPositionTarget)
  {
    delta = abs(axisStatusDe.stepPositionTarget - axisStatusDe.stepPosition);
    if(axisStatusDe.stepPositionTarget > axisStatusDe.stepPosition)
      axisStatusDe.direction = axisPropertiesDe.directionUp;
    else
      axisStatusDe.direction = axisPropertiesDe.directionDown;
    SET_DE_DIRECTION(axisStatusDe.direction);
    axisStatusDe.motorStatus = MOTOR_STATUS_GOTO;        
    axisStatusDe.stepFrequency = 0;
    if(delta < axisPropertiesDe.maxAccelerationSteps)
      axisStatusDe.accelerationSteps = delta / 2;
    else
      axisStatusDe.accelerationSteps = axisPropertiesDe.maxAccelerationSteps / 2;
  }
}

void AbortGoTo()
{
  long target;
  long delta;
  if(axisStatusRa.motorStatus == MOTOR_STATUS_GOTO)
  {
    delta = abs(axisStatusRa.stepPositionTarget - axisStatusRa.stepPosition);
    if(delta > axisStatusRa.accelerationSteps)
    {
      if(axisStatusRa.direction == axisPropertiesRa.directionUp)
        target = axisStatusRa.stepPosition + axisStatusRa.accelerationSteps;
      else
        target = axisStatusRa.stepPosition - axisStatusRa.accelerationSteps;
      axisStatusRa.stepPositionTarget = target;
    }
  }
  if(axisStatusDe.motorStatus == MOTOR_STATUS_GOTO)
  {
    delta = abs(axisStatusDe.stepPositionTarget - axisStatusDe.stepPosition);
    if(delta > axisStatusDe.accelerationSteps)
    {
      if(axisStatusDe.direction == axisPropertiesDe.directionUp)
        target = axisStatusDe.stepPosition + axisStatusDe.accelerationSteps * axisStatusRa.stepFrequency / axisPropertiesRa.gotoStepFrequency;
      else
        target = axisStatusDe.stepPosition - axisStatusDe.accelerationSteps * axisStatusRa.stepFrequency / axisPropertiesRa.gotoStepFrequency;
      axisStatusDe.stepPositionTarget = target;
    }
  }
}


// starts spiral search; possible only if in guiding state
void StartSpiralSearch(int armArcsec, int nSteps, uint8_t returnToStart)
{
  if(spiralSearchOn != 0)
    return;
  if(armArcsec < 0 || armArcsec > 9999)
    return;
  if(nSteps < 0 || nSteps > 999)
    return;
  if(axisStatusRa.motorStatus != MOTOR_STATUS_GUIDE || axisStatusDe.motorStatus != MOTOR_STATUS_GUIDE)
    return;
  spiralSearchArmArcsec = armArcsec;
  spiralSearchSteps = nSteps;
  spiralSearchReturnToStartingPoint = returnToStart;
  spiralSearchStartingRa = axisStatusRa.stepPosition;
  spiralSearchStartingDe = axisStatusDe.stepPosition;
  currentSpiralSearchStep = 0;  
  spiralSearchStepComplete = 0;
  spiralSearchOn = 1;
  StepForwardSpiralSearch();
}

void StepForwardSpiralSearch()
{
  if(spiralSearchOn == 0)
    return;
  if(currentSpiralSearchStep == spiralSearchSteps)
  {
    spiralSearchOn = 0;
    MoveRaOff();    
    MoveDeOff();    
    if(spiralSearchReturnToStartingPoint != 0)
      StartGoTo(spiralSearchStartingRa, spiralSearchStartingDe);
    return;
  }
  spiralSearchStepComplete = 0;
  int status = currentSpiralSearchStep & 0x3;
  int motorSteps;
  int armSteps;
  switch(status)
  {
  case 0:       // move N    
    MoveRaOff();    
    armSteps = ROUND((double)spiralSearchArmArcsec / axisPropertiesDe.stepToArcsec);
    motorSteps = armSteps * ((currentSpiralSearchStep / 2) + 1);
    axisStatusDe.stepPositionTarget = axisStatusDe.stepPosition + motorSteps;
    MoveDeOn(1);
    break;
  case 1:       // move W
    MoveDeOff();    
    armSteps = ROUND((double)spiralSearchArmArcsec / axisPropertiesRa.stepToArcsec);
    motorSteps = armSteps * ((currentSpiralSearchStep / 2) + 1);
    axisStatusRa.stepPositionTarget = axisStatusRa.stepPosition - motorSteps;
    if(axisStatusRa.stepPositionTarget < 0)
      axisStatusRa.stepPositionTarget += axisPropertiesRa.maxSteps;
    MoveRaOn(1);
    break;
  case 2:       // move S
    MoveRaOff();    
    armSteps = ROUND((double)spiralSearchArmArcsec / axisPropertiesDe.stepToArcsec);
    motorSteps = armSteps * ((currentSpiralSearchStep / 2) + 1);
    axisStatusDe.stepPositionTarget = axisStatusDe.stepPosition - motorSteps;
    MoveDeOn(0);
    break;
  case 3:       // move E        
    MoveDeOff();    
    armSteps = ROUND((double)spiralSearchArmArcsec / axisPropertiesRa.stepToArcsec);
    motorSteps = armSteps * ((currentSpiralSearchStep / 2) + 1);
    axisStatusDe.stepPositionTarget = axisStatusDe.stepPosition;
    axisStatusRa.stepPositionTarget = axisStatusRa.stepPosition + motorSteps;
    if(axisStatusRa.stepPositionTarget >= axisPropertiesRa.maxSteps)
      axisStatusRa.stepPositionTarget -= axisPropertiesRa.maxSteps;
    MoveRaOn(0);
    break;
  }
  currentSpiralSearchStep++;
  
}


void CheckSpiralSearchPosition()
{
  if(spiralSearchOn != 0 && spiralSearchStepComplete != 0)
  {
    if(axisStatusRa.motorStatus == MOTOR_STATUS_CORRECTION || axisStatusDe.motorStatus == MOTOR_STATUS_CORRECTION)
        StepForwardSpiralSearch();  
  }
}


void AbortSpiralSearch()
{
  if(spiralSearchOn == 0)
    return;
  spiralSearchReturnToStartingPoint = 0;
  spiralSearchStepComplete = 1;
  currentSpiralSearchStep = spiralSearchSteps; 
}




// parameter: intervalMs (in msec)
void UpdateAccelerationSpeed(int intervalMs)
{
  long delta;
  // RA
  if(axisStatusRa.motorStatus == MOTOR_STATUS_GOTO)
  {
    delta = abs(axisStatusRa.stepPositionTarget - axisStatusRa.stepPosition);
    if(delta > axisPropertiesRa.maxSteps / 2)
      delta = axisPropertiesRa.maxSteps - delta;        
    if(delta < axisStatusRa.accelerationSteps)
    {
      if(axisStatusRa.stepFrequency > axisPropertiesRa.correctionStepFrequency[END_GOTO_CORRECTIONSPEED_INDEX])
      {
        axisStatusRa.stepFrequency -= axisPropertiesRa.gotoStepFrequency * intervalMs / axisPropertiesRa.accelerationTime;
        if(axisStatusRa.stepFrequency < axisPropertiesRa.correctionStepFrequency[END_GOTO_CORRECTIONSPEED_INDEX])
          axisStatusRa.stepFrequency = axisPropertiesRa.correctionStepFrequency[END_GOTO_CORRECTIONSPEED_INDEX];
        SetGoToFrequencyRa(axisStatusRa.stepFrequency);
      }
    }
    else if(axisStatusRa.stepFrequency < axisPropertiesRa.gotoStepFrequency)
    {
      axisStatusRa.stepFrequency += axisPropertiesRa.gotoStepFrequency * intervalMs / axisPropertiesRa.accelerationTime;
      if(axisStatusRa.stepFrequency > axisPropertiesRa.gotoStepFrequency)
        axisStatusRa.stepFrequency = axisPropertiesRa.gotoStepFrequency;
      SetGoToFrequencyRa(axisStatusRa.stepFrequency);
    }
  }
  // DEC
  if(axisStatusDe.motorStatus == MOTOR_STATUS_GOTO)
  {
    delta = abs(axisStatusDe.stepPositionTarget - axisStatusDe.stepPosition);
    if(delta < axisStatusDe.accelerationSteps)
    {
      if(axisStatusDe.stepFrequency > axisPropertiesDe.correctionStepFrequency[END_GOTO_CORRECTIONSPEED_INDEX])
      {
        axisStatusDe.stepFrequency -= axisPropertiesDe.gotoStepFrequency * intervalMs / axisPropertiesDe.accelerationTime;
        if(axisStatusDe.stepFrequency < axisPropertiesDe.correctionStepFrequency[END_GOTO_CORRECTIONSPEED_INDEX])
          axisStatusDe.stepFrequency = axisPropertiesDe.correctionStepFrequency[END_GOTO_CORRECTIONSPEED_INDEX];
        SetGoToFrequencyDe(axisStatusDe.stepFrequency);
      }
    }
    else if(axisStatusDe.stepFrequency < axisPropertiesDe.gotoStepFrequency)
    {
      axisStatusDe.stepFrequency += axisPropertiesDe.gotoStepFrequency * intervalMs / axisPropertiesDe.accelerationTime;
      if(axisStatusDe.stepFrequency > axisPropertiesDe.gotoStepFrequency)
        axisStatusDe.stepFrequency = axisPropertiesDe.gotoStepFrequency;
      SetGoToFrequencyDe(axisStatusDe.stepFrequency);
    }
  }  
}

// Increment correction speed index
void IncrCorrectionSpeed()
{  
  if(currentCorrectionSpeedIndex < MAX_CORRECTION_SPEEDS - 1)
    currentCorrectionSpeedIndex++;
}

// Decrement correction speed index
void DecrCorrectionSpeed()
{
  if(currentCorrectionSpeedIndex > 0)
    currentCorrectionSpeedIndex--;
}

void MoveRaOn(int direction)
{
  if(axisStatusRa.motorStatus == MOTOR_STATUS_GUIDE)
  {
    axisStatusRa.motorStatus = MOTOR_STATUS_CORRECTION;
    if(direction == 1)
    {
      axisStatusRa.direction = axisPropertiesRa.directionDown;
      axisStatusRa.stepFrequency = ROUND(axisPropertiesRa.guideStepFrequency[currentGuideSpeedIndex]) + axisPropertiesRa.correctionStepFrequency[currentCorrectionSpeedIndex];
    }
    else
    {
      axisStatusRa.stepFrequency = ROUND(axisPropertiesRa.guideStepFrequency[currentGuideSpeedIndex]) - axisPropertiesRa.correctionStepFrequency[currentCorrectionSpeedIndex];
      if(axisStatusRa.stepFrequency < 0)
      {
        axisStatusRa.stepFrequency = -axisStatusRa.stepFrequency;
        axisStatusRa.direction = axisPropertiesRa.directionUp;   
      }
      else
      {
        axisStatusRa.direction = axisPropertiesRa.directionDown;   
      }
    }
    SET_RA_DIRECTION(axisStatusRa.direction);
    SetGoToFrequencyRa(axisStatusRa.stepFrequency);
  }
}

void MoveRaOff()
{
  if(axisStatusRa.motorStatus == MOTOR_STATUS_CORRECTION)
  {
    axisStatusRa.motorStatus = MOTOR_STATUS_GUIDE;
    axisStatusRa.direction = axisPropertiesRa.directionDown;
    SET_RA_DIRECTION(axisStatusRa.direction);
    SetGuideFrequencyRa(axisPropertiesRa.guideStepFrequency[currentGuideSpeedIndex]);
  }
}

void MoveDeOn(int direction)
{
  if(axisStatusDe.motorStatus == MOTOR_STATUS_GUIDE)
  {
    axisStatusDe.motorStatus = MOTOR_STATUS_CORRECTION;
    if(direction == 1)
      axisStatusDe.direction = axisPropertiesDe.directionUp;
    else
      axisStatusDe.direction = axisPropertiesDe.directionDown;
    axisStatusDe.stepFrequency = axisPropertiesDe.correctionStepFrequency[currentCorrectionSpeedIndex];
    SET_DE_DIRECTION(axisStatusDe.direction);
    SetGoToFrequencyDe(axisStatusDe.stepFrequency);
  }
}

void MoveDeOff()
{
  if(axisStatusDe.motorStatus == MOTOR_STATUS_CORRECTION)
  {
    axisStatusDe.motorStatus = MOTOR_STATUS_GUIDE;
    axisStatusDe.direction = axisPropertiesDe.directionUp;
    SET_DE_DIRECTION(axisStatusDe.direction);
    SetGuideFrequencyDe(axisPropertiesDe.guideStepFrequency[currentGuideSpeedIndex]);
  }
}

void SetGuideSpeed(int index)
{
  if(index >= 0 && index < MAX_GUIDE_SPEEDS)
  {
    currentGuideSpeedIndex = index;
    
    if(axisStatusRa.motorStatus == MOTOR_STATUS_GUIDE)
    {
      axisStatusRa.direction = axisPropertiesRa.directionDown;
      SET_RA_DIRECTION(axisStatusRa.direction);
      SetGuideFrequencyRa(axisPropertiesRa.guideStepFrequency[currentGuideSpeedIndex]);
    }

    if(axisStatusDe.motorStatus == MOTOR_STATUS_GUIDE)
    {
      axisStatusDe.direction = axisPropertiesDe.directionUp;
      SET_DE_DIRECTION(axisStatusDe.direction);
      SetGuideFrequencyDe(axisPropertiesDe.guideStepFrequency[currentGuideSpeedIndex]);
    }
  }
}

void SetCorrectionSpeed(int index)
{
  if(index >= 0 && index < MAX_CORRECTION_SPEEDS)
    currentCorrectionSpeedIndex = index;
}

/**
  * @brief  update the RA clock (every 100 msec)
  * @param  None
  * @retval None
  */
void UpdateClockRa()
{
  if(axisStatusRa.stepPosition < axisPropertiesRa.maxSteps - 1)
    axisStatusRa.stepPosition++;
  else
    axisStatusRa.stepPosition = 0;
  
  if(axisStatusRa.motorStatus == MOTOR_STATUS_GOTO)
  {
    if(axisStatusRa.stepPositionTarget == axisStatusRa.stepPosition)
      TerminateRaGoTo();
  }
  if(spiralSearchOn != 0 && axisStatusRa.motorStatus == MOTOR_STATUS_CORRECTION)
  {
    if(axisStatusRa.stepPositionTarget == axisStatusRa.stepPosition)
      spiralSearchStepComplete = 1;
  }

}

void PulseRa()
{
  UpdatePositionRa();
  
  if(axisStatusRa.motorStatus == MOTOR_STATUS_GOTO)
  {
    if(axisStatusRa.stepPositionTarget == axisStatusRa.stepPosition)
      TerminateRaGoTo();
  }
  if(spiralSearchOn != 0 && axisStatusRa.motorStatus == MOTOR_STATUS_CORRECTION)
  {
    if(axisStatusRa.stepPositionTarget == axisStatusRa.stepPosition)
      spiralSearchStepComplete = 1;
  }
}

void TerminateRaGoTo()
{
  axisStatusRa.motorStatus = MOTOR_STATUS_GUIDE;
  axisStatusRa.direction = axisPropertiesRa.directionDown;
  SET_RA_DIRECTION(axisStatusRa.direction);
  SetGuideFrequencyRa(axisPropertiesRa.guideStepFrequency[currentGuideSpeedIndex]);
}

void PulseDe()
{
  UpdatePositionDe();
  
  if(axisStatusDe.motorStatus == MOTOR_STATUS_GOTO)
  {
    if(axisStatusDe.stepPositionTarget == axisStatusDe.stepPosition)
      TerminateDeGoTo();
  }
  if(spiralSearchOn != 0 && axisStatusDe.motorStatus == MOTOR_STATUS_CORRECTION)
  {
    if(axisStatusDe.stepPositionTarget == axisStatusDe.stepPosition)
      spiralSearchStepComplete = 1;
  }
}

void TerminateDeGoTo()
{
  axisStatusDe.motorStatus = MOTOR_STATUS_GUIDE;
  axisStatusDe.direction = axisPropertiesDe.directionUp;
  SET_DE_DIRECTION(axisStatusDe.direction);
  SetGuideFrequencyDe(axisPropertiesDe.guideStepFrequency[currentGuideSpeedIndex]);  
}
long GetCurrentArcsecRa()
{
  long ra = ROUND((double)axisStatusRa.stepPosition * axisPropertiesRa.stepToArcsec);
  if(ra >= FULL_CIRCLE_ARCSECS)
    ra -= FULL_CIRCLE_ARCSECS;
  return ra;
}

long GetCurrentArcsecDe()
{
  return ROUND((double)axisStatusDe.stepPosition * axisPropertiesDe.stepToArcsec);
}


/* Private functions ---------------------------------------------------------*/

void InitGpioPins()
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  // Enable the TELESCOPE_GPIO_CLK Clock
  RCC_AHB1PeriphClockCmd(TELESCOPE_GPIO_CLK, ENABLE);

  GPIO_InitStructure.GPIO_Pin = 
    RA_DIR_PIN  | 
    RA_STEP_PIN | 
    DE_DIR_PIN  | 
    DE_STEP_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(TELESCOPE_GPIO_PORT, &GPIO_InitStructure);

  GPIO_SetBits(GPIOD, GPIO_Pin_0);
  GPIO_SetBits(GPIOD, GPIO_Pin_1);
  GPIO_SetBits(GPIOD, GPIO_Pin_2);
  GPIO_SetBits(GPIOD, GPIO_Pin_3);

 
}

void UpdatePositionRa()
{
  if(axisStatusRa.direction == axisPropertiesRa.directionDown)
  {
    if(axisStatusRa.stepPosition > 0)
      axisStatusRa.stepPosition--;
    else
      axisStatusRa.stepPosition = axisPropertiesRa.maxSteps - 1;
  }
  else
  {
    if(axisStatusRa.stepPosition < axisPropertiesRa.maxSteps - 1)
      axisStatusRa.stepPosition++;
    else
      axisStatusRa.stepPosition = 0;
  }
}

void UpdatePositionDe()
{
  if(axisStatusDe.direction == axisPropertiesDe.directionUp)
  {
    if(axisStatusDe.stepPosition < axisPropertiesDe.maxSteps)
      axisStatusDe.stepPosition++;
  }
  else
  {
    if(axisStatusDe.stepPosition > -axisPropertiesDe.maxSteps)
      axisStatusDe.stepPosition--;
  }
}

// transmit motor(s) status via USB to the client
void TransmitStatus(uint8_t what)
{
  int i;
  for(i=0;i<MAX_COMMAND_LENGTH;i++)
    buffer[i] = 0;
  i = 0;
  switch(what)
  {
  case 'M':     // motor status (guide, correction, goto)
    // motor(s) status:
    if(spiralSearchOn != 0)
    {
      buffer[i++] = 's';
      buffer[i++] = 'p';
      buffer[i++] = 'i';
      buffer[i++] = 'r';      
    }
    else if(axisStatusRa.motorStatus == MOTOR_STATUS_GOTO || axisStatusDe.motorStatus == MOTOR_STATUS_GOTO)
    {
      buffer[i++] = 'g';
      buffer[i++] = 'o';
      buffer[i++] = 't';
      buffer[i++] = 'o';
    }
    else if(axisStatusRa.motorStatus == MOTOR_STATUS_CORRECTION || axisStatusDe.motorStatus == MOTOR_STATUS_CORRECTION)
    {
      buffer[i++] = 'c';
      buffer[i++] = 'o';
      buffer[i++] = 'r';
      buffer[i++] = 'r';    
    }
    else
    {
      buffer[i++] = 'g';
      buffer[i++] = 'u';
      buffer[i++] = 'i';
      buffer[i++] = 'd';    
    }
    break;
  case 'T':     // tracking speed
    buffer[i++] = currentGuideSpeedIndex + '0';
    break;
  case 'C':
    buffer[i++] = currentCorrectionSpeedIndex + '0';
    break;
  }
  buffer[i++] = '\n';
  TransmitData(buffer, i);  
}

// transmit axes position in arcsec via USB to the client
void TransmitPosition()
{
  for(int i=0;i<MAX_COMMAND_LENGTH;i++)
    buffer[i] = 0;
  
  // position: format P+aaaaaaa±ddddddd\n, where 
  // a is RA in arcsec (with zero padding)
  // d is DE in arcsec (with zero padding)
  buffer[0] = 'P';
  IntToString(buffer, 1, 8, GetCurrentArcsecRa());
  IntToString(buffer, 9, 8, GetCurrentArcsecDe());
  buffer[17] = '\n';
  TransmitData(buffer, 18);  
}