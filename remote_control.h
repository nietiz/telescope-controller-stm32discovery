/**
  ******************************************************************************
  * @file    remote_control.h (ex usbd_cdc_vcp.h)
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    19-March-2012
  * @brief   Header for remote_control.c file.
  * modified by Tiziano niero - February 2014
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __REMOTE_CONTROL_H
#define __REMOTE_CONTROL_H

/* Includes ------------------------------------------------------------------*/
/* Exported typef ------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void InitRemoteControl();
void TransmitData(uint8_t* Buf, uint32_t Len);

#endif /* __REMOTE_CONTROL_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
