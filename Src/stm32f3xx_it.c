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
#include "general_math.h"


extern uint32_t Position;
extern uint32_t TimeLimit_u;
extern uint32_t TimeLimit;
extern uint32_t Tick;
extern uint32_t psk_buff;
extern uint32_t psk;
extern uint32_t freq;
extern uint8_t  n;
extern uint8_t 	n_stop;
			 uint8_t  n_s;
extern uint8_t	dem;
extern uint8_t	Mode;
extern uint8_t	n_IC;
			 uint8_t	sec = 0;
			 uint32_t	min = 0;
extern uint32_t DiagTime;
extern uint8_t	TransMode;
extern uint8_t	f_change;
extern _Bool		StartSend;
extern _Bool		StopDiag;
extern uint32_t	tmpData;
extern uint8_t	tmpComand;
extern uint8_t	diag;
extern uint8_t 	SendForm;
extern uint16_t alarm_pause_cnt;
extern float 		BR;

extern StatusSystem_t STATUS;
extern SettingParametrs_t SETUP;
			 
extern uint32_t psk;
extern uint8_t  SoftStart;
extern uint32_t BufferADC_1;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;
extern DMA_HandleTypeDef hdma_adc3;
extern DMA_HandleTypeDef hdma_adc4;
extern COMP_HandleTypeDef hcomp2;
extern COMP_HandleTypeDef hcomp4;
extern DAC_HandleTypeDef hdac;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim16;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern UART_HandleTypeDef huart1;

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
* @brief This function handles EXTI line2 and Touch Sense controller.
*/
void EXTI2_TSC_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_TSC_IRQn 0 */

  /* USER CODE END EXTI2_TSC_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  /* USER CODE BEGIN EXTI2_TSC_IRQn 1 */

  /* USER CODE END EXTI2_TSC_IRQn 1 */
}

/**
* @brief This function handles DMA1 channel1 global interrupt.
*/
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */
		//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_SET);
		//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_RESET);
  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
* @brief This function handles DMA1 channel4 global interrupt.
*/
void DMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */

  /* USER CODE END DMA1_Channel4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */

  /* USER CODE END DMA1_Channel4_IRQn 1 */
}

/**
* @brief This function handles DMA1 channel5 global interrupt.
*/
void DMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */

  /* USER CODE END DMA1_Channel5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */

  /* USER CODE END DMA1_Channel5_IRQn 1 */
}

/**
* @brief This function handles TIM1 update and TIM16 interrupts.
*/
void TIM1_UP_TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 0 */
	sec++;
	if(SETUP.alarm_msg)
	{
		alarm_pause_cnt++;
		if(alarm_pause_cnt > (ALARM_PAUSE))
		{
			DiagnSendAlarm();
			alarm_pause_cnt = 0;
		}
		
	}
	else
		alarm_pause_cnt = 0;
	
	if(sec == 60)
	{
		min++;
		sec = 0;
	}
	if((StartSend == 1)&&(StopDiag == 1))
	{
		StartSend = 0;
		switch(SendForm)
		{
			case 0:
				SendData(tmpData);
			break;
			case 1:
				SendComand(tmpComand);
			break;
			case 2:
				SendComandnData(tmpData, tmpComand);
			break;
			case 3:
				SendAlarm(tmpData);
			break;
			case 4:
				TestTag(tmpData);
			break;
		}
	}
	if(sec > 1)
	{
		StopDiag = 1;
//		CommandReply(U2CT_TRANS_OK, 'i', STATUS.trans_ok);			// правильная отправка, а ещё привильнее добавить пинг-понг
		CommandReply(U2CT_TRANS_OK, 'i', 0);
	}
#ifndef GEN_MODE
	if(min == DiagTime)
		Diag();
#endif
  /* USER CODE END TIM1_UP_TIM16_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  HAL_TIM_IRQHandler(&htim16);
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM16_IRQn 1 */
}

/**
* @brief This function handles TIM1 capture compare interrupt.
*/
void TIM1_CC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_CC_IRQn 0 */
//	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_SET);
	switch(TransMode)
		{
		case DIAG:
			if(TimeLimit == TimeLimit_u*Tick)
				{
					if(n==10)
						n = 2;
//					HAL_TIM_Base_Stop_IT(&htim7);
					CorrectTIM();
				}
			if(f_change == 0)
				{
					TimeLimit_u = TimeLimit_u/2;
					TimingCalc();																								// расчет длитльностей импульсов
					SetTiming();
					f_change = 1;
				}
			else
				TIM1 -> BDTR &= TIM_BREAK_ENABLE;
		break;
		case SENDDATA: //if(TransMode == SENDDATA)
			if(TimeLimit == TimeLimit_u*Tick)
				{
					//if(n==8)
						n = 2;
//					HAL_TIM_Base_Stop_IT(&htim7);
					CorrectTIM();
				}
		break;
		case ALARM: //if(TransMode == SENDDATA)
			if(TimeLimit == TimeLimit_u*Tick)
				{
					//if(n==8)
						n = 2;
//					HAL_TIM_Base_Stop_IT(&htim7);
					CorrectTIM();
				}
		break;
	}
	//GetVoltage();
		
		
  /* USER CODE END TIM1_CC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_CC_IRQn 1 */
	if(SoftStart == ON)
		{
			if(TimeLimit < TimeLimit_u*Tick)
				{
					if(!n_s)
					//{
						TimingCalc();
						SetTiming();
					//}
					n_s = ~n_s;
				}
		//}
		
	if(n_stop >= 1)
	{
		if(n_s)
		{
			HAL_TIM_Base_Stop_IT(&htim7);
			TimingCalc();
			SetTiming();
			StopPWM();
		}
		n_s = ~n_s;
		//n_stop++;
	}}
//		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET);
  /* USER CODE END TIM1_CC_IRQn 1 */
}

/**
* @brief This function handles TIM2 global interrupt.
*/
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
	if(Mode == AUX)
		ExternPWM();
		
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
* @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
*/
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
* @brief This function handles EXTI line[15:10] interrupts.
*/
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
* @brief This function handles Timer 6 interrupt and DAC underrun interrupts.
*/
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
	switch(TransMode)
	{
		case SENDDATA:
			ProcData();
			break;
		case DEMAG:
			ProcDemagnetization();
			break;
		case DIAG:
			ProcDiag();
			break;	
		case ALARM:
			ProcAlarm();
			break;
		case TESTTAG:
			ProcTestTag();
			break;
	}	
  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  HAL_DAC_IRQHandler(&hdac);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
* @brief This function handles TIM7 global interrupt.
*/
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */
	
	GetVoltage();
//	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_SET);
  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */
//	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_RESET);
  /* USER CODE END TIM7_IRQn 1 */
}

/**
* @brief This function handles DMA2 channel1 global interrupt.
*/
void DMA2_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Channel1_IRQn 0 */

  /* USER CODE END DMA2_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc2);
  /* USER CODE BEGIN DMA2_Channel1_IRQn 1 */

  /* USER CODE END DMA2_Channel1_IRQn 1 */
}

/**
* @brief This function handles DMA2 channel2 global interrupt.
*/
void DMA2_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Channel2_IRQn 0 */

  /* USER CODE END DMA2_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc4);
  /* USER CODE BEGIN DMA2_Channel2_IRQn 1 */

  /* USER CODE END DMA2_Channel2_IRQn 1 */
}

/**
* @brief This function handles DMA2 channel5 global interrupt.
*/
void DMA2_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Channel5_IRQn 0 */

  /* USER CODE END DMA2_Channel5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc3);
  /* USER CODE BEGIN DMA2_Channel5_IRQn 1 */

  /* USER CODE END DMA2_Channel5_IRQn 1 */
}

/**
* @brief This function handles COMP1, COMP2 and COMP3 interrupts through EXTI lines 21, 22 and 29.
*/
void COMP1_2_3_IRQHandler(void)
{
  /* USER CODE BEGIN COMP1_2_3_IRQn 0 */

  /* USER CODE END COMP1_2_3_IRQn 0 */
  HAL_COMP_IRQHandler(&hcomp2);
  /* USER CODE BEGIN COMP1_2_3_IRQn 1 */

  /* USER CODE END COMP1_2_3_IRQn 1 */
}

/**
* @brief This function handles COMP4, COMP5 and COMP6 interrupts through EXTI lines 30, 31 and 32.
*/
void COMP4_5_6_IRQHandler(void)
{
  /* USER CODE BEGIN COMP4_5_6_IRQn 0 */

  /* USER CODE END COMP4_5_6_IRQn 0 */
  HAL_COMP_IRQHandler(&hcomp4);
  /* USER CODE BEGIN COMP4_5_6_IRQn 1 */

  /* USER CODE END COMP4_5_6_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
