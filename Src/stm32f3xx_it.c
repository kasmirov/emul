/**
  ******************************************************************************
  * @file    stm32f3xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "stm32f3xx.h"
#include "stm32f3xx_it.h"

/* USER CODE BEGIN 0 */
/** USB device core handle. */
#include "usbd_def.h"
#include "usbd_cdc.h"
#include <math.h>

#define UID_GATE 0x00000000
#define UID_AN1 0x80180001
#define UID_AN2 0x80180002
#define UID_AN3 0x80180003
#define UID_AN4 0x80180004
#define UID_TAG1 0x20000001
#define UID_TAG2 0x20000002

uint64_t microtick = 0;

typedef struct
{
   int32_t x;
   int32_t y;
   int32_t z;   
}coord;

coord tag1 = {10, 20, 0};
coord tag2 = {5, 15, 1};

coord an1 = {0, 0, 3};
coord an2 = {0, 20, 3};
coord an3 = {25, 20, 3};
coord an4 = {25, 0, 3};


float GetDist(coord *tag, coord *anchor);
uint64_t GetTimeOfFlight(float dist);

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_FS;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Pre-fetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F3xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f3xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles USB low priority or CAN_RX0 interrupts.
*/
void USB_LP_CAN_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN USB_LP_CAN_RX0_IRQn 0 */

  /* USER CODE END USB_LP_CAN_RX0_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_FS);
  /* USER CODE BEGIN USB_LP_CAN_RX0_IRQn 1 */

  /* USER CODE END USB_LP_CAN_RX0_IRQn 1 */
}

/**
* @brief This function handles TIM1 update and TIM16 interrupts.
*/
void TIM1_UP_TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 0 */

  static uint8_t cnt_tag1 = 100;
  static uint8_t sel_anchor1 = 1;

  static uint64_t meas_time1 = 0;
  if (sel_anchor1 == 1) 
  {
    meas_time1 = microtick;
    cnt_tag1++;
    if (cnt_tag1 > 254) cnt_tag1 = 0;
  }

 
  
  /* USER CODE END TIM1_UP_TIM16_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 1 */
  
  USBD_CDC_HandleTypeDef* hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  char buf[64];
  uint32_t anchor_id;
  coord anchor_coord;
  uint32_t tag_id = (uint32_t)UID_TAG1;
    
  switch (sel_anchor1)
  {
  case 1:
    anchor_id = (uint32_t)UID_AN1;
    anchor_coord = an1;
    break;
  case 2:
    anchor_id = (uint32_t)UID_AN2;
    anchor_coord = an2;
    break;
  case 3:
    anchor_id = (uint32_t)UID_AN3;
    anchor_coord = an3;
    break;
  case 4:
    anchor_id = (uint32_t)UID_AN4;
    anchor_coord = an4;
    break;
  }
  sel_anchor1++;
  if (sel_anchor1 == 5)
  {
    sel_anchor1 = 1;
  }
  
  buf[0] = ':';
  buf[1] = 'D';
  buf[2] = 0x00;
  buf[3] = 0x00;
  buf[4] = 0x00;
  buf[5] = 0x00;
  memcpy(buf + 6, &anchor_id, 4);
  buf[10] = 0x00;
  memcpy(buf + 11, &tag_id, 4);
  
  uint64_t tof;
  volatile float dist = GetDist(&tag1, &anchor_coord);
  tof = meas_time1 + GetTimeOfFlight(dist);
  memcpy(buf + 15, &tof, 8);
  buf[23] = cnt_tag1;
  buf[24] = 0x00;
  buf[25] = 0x00;
  buf[26] = 0x00;
  buf[27] = 0x00;
  buf[28] = 0x00;
  buf[29] = 0x00;  
  buf[30] = 'C';
  buf[31] = 'R';  
  
  for (int i = 32; i < 63; i++) buf[i] = 0;
  
  if (hcdc->TxState == 0)
  {
      CDC_Transmit_FS(buf, 32);
  }
  
  
  /* USER CODE END TIM1_UP_TIM16_IRQn 1 */
}

/**
* @brief This function handles TIM2 global interrupt.
*/
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
  static uint8_t cnt_tag2 = 200;
  static uint8_t sel_anchor2 = 1;

  static uint64_t meas_time2 = 0;
  if (sel_anchor2 == 1) 
  {
    meas_time2 = microtick;
    cnt_tag2++;
    if (cnt_tag2 > 254) cnt_tag2 = 0;
  }
  
  
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
  
  USBD_CDC_HandleTypeDef* hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  char buf[64];
  uint32_t anchor_id;
  coord anchor_coord;
  uint32_t tag_id = (uint32_t)UID_TAG1;
    
  switch (sel_anchor2)
  {
  case 1:
    anchor_id = (uint32_t)UID_AN1;
    anchor_coord = an1;
    break;
  case 2:
    anchor_id = (uint32_t)UID_AN2;
    anchor_coord = an2;
    break;
  case 3:
    anchor_id = (uint32_t)UID_AN3;
    anchor_coord = an3;
    break;
  case 4:
    anchor_id = (uint32_t)UID_AN4;
    anchor_coord = an4;
    break;
  }
  sel_anchor2++;
  if (sel_anchor2 == 5)
  {
    sel_anchor2 = 1;
  }
  
  buf[0] = ':';
  buf[1] = 'D';
  buf[2] = 0x00;
  buf[3] = 0x00;
  buf[4] = 0x00;
  buf[5] = 0x00;
  memcpy(buf + 6, &anchor_id, 4);
  buf[10] = 0x00;
  memcpy(buf + 11, &tag_id, 4);
  
  uint64_t tof;
  volatile float dist = GetDist(&tag2, &anchor_coord);
  tof = meas_time2 + GetTimeOfFlight(dist);
  memcpy(buf + 15, &tof, 8);
  buf[23] = cnt_tag2;
  buf[24] = 0x00;
  buf[25] = 0x00;
  buf[26] = 0x00;
  buf[27] = 0x00;
  buf[28] = 0x00;
  buf[29] = 0x00;  
  buf[30] = 'C';
  buf[31] = 'R';  
  
  for (int i = 32; i < 63; i++) buf[i] = 0;
  
  if (hcdc->TxState == 0)
  {
      CDC_Transmit_FS(buf, 32);
  }
  /* USER CODE END TIM2_IRQn 1 */
}

/**
* @brief This function handles TIM3 global interrupt.
*/
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
  
  microtick++;
  if (microtick >= (UINT64_MAX -1)) microtick = 0;
  
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/* USER CODE BEGIN 1 */

// Возвращает дистанцию между точками
float GetDist(coord *tag, coord *anchor)
{
  return sqrt(powf((float)((tag->x) - (anchor->x)), 2.0f) + pow((float)((tag->y) - (anchor->y)), 2.0f) + pow((float)((tag->z) - (anchor->z)), 2.0f));
}

// Время прохождения пути в нс
uint64_t GetTimeOfFlight(float dist)
{
  return (uint64_t) ((uint64_t)(1000000000.0f * dist / 299704000.0f) & 0xFFFFFFFFFFFFFFFF); //299792458.0
}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
