/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#include "main.h"
#include "stm32f3xx_hal.h"
#include "adc.h"
#include "comp.h"
#include "crc.h"
#include "dac.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "general_math.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

uint32_t psk = 0;								// рабочая длительность импульса
uint32_t psk_buff = 0;					// временная длительность импульса
uint32_t Period;								// длительность периода
uint32_t Pulse_1;								// длительность верхнего ключа диагональ В
uint32_t Pulse_2;								// длительность нижнего ключа диагональ В
uint32_t Pulse_3;								// длительность верхнего ключа диагональ А
uint32_t Pulse_4;								// длительность нижнего ключа диагональ А
uint32_t DeathTime_u = 50;			// длительность паузы между диагоналями в  мкс
uint32_t DeathTime;							// рабочая длительность паузы в тиках таймера
int32_t  Overlap_u = -50;					// длительность перекрытия нижних ключей в мкс
uint32_t Overlap;								// рабочая длительность паузы в тиках таймера
uint32_t TimeLimit_u = 650;			// максимальная длительность импульса верхних ключей
uint32_t TimeLimit;							// рабочая длительность импульса верхних ключей
uint32_t Tick = 8;							// количество тиков таймера в 1 мкс
uint32_t SoftTick = 0;
uint8_t	 State = pwmSTART;			// состояние передачи 0 - запуск, 1 - передача, 2 - останов
uint16_t DataLenght = 8;				// длина сообщения
uint8_t  Position = 0;					// позиция в сообщении
uint8_t  Data[MAXDAT];		// данные 
uint8_t  mData[] = {85, 162, 34, 93, 205, 47, 13, 5};
uint32_t Deviation = 8;
uint32_t f1 = 719;							// передача "1"
uint32_t f2 = 769;							// передача "0"
float BR = 1;						 				// скорость передачи
#ifndef GEN_MODE
uint8_t  Mode = MAIN;						// режим работы генератора 1 - основной, 0 - внешний, 2 - генератор
#else
uint8_t  Mode = GEN;						// режим работы генератора 1 - основной, 0 - внешний, 2 - генератор
#endif
uint32_t BufferADC_1[4];				// буфер АЦП1 каналы 1 - 4
uint32_t BufferADC_2[4];				// буфер АЦП2 каналы 6 - 9
uint32_t BufferADC_3[2];				// буфер АЦП3 каналы 11 - 12

#ifdef GEN_MODE
		#ifdef F719
			uint16_t fgen = 719;
		#endif
		#ifdef F1000
			uint16_t fgen = 1000;
		#endif
		#ifdef F1200
			uint16_t fgen = 1200;
		#endif

#else
uint16_t fgen = 1000;
#endif
extern float mean1;
extern float mean2;
extern float mean3;
extern float mean4;
extern uint32_t tmpData;


extern float Us4;
extern float C4;

uint8_t  SoftStart = ON;

float ftmp1;
float ftmp2;
float ftmp3;
float ftmp4;

uint32_t tmp;

uint32_t aCNT;
uint8_t  bCNT;

extern uint32_t ADC1buff;
extern uint32_t ADC2buff;
extern uint32_t ADC3buff;
extern uint32_t ADC4buff;

extern uint8_t CodeMode;
extern uint8_t RepeatNum;


extern StatusSystem_t STATUS;
extern SettingParametrs_t SETUP;
extern UART2_Queue_Data UART2RecvData;
extern UART2Recv_t UART2_RecvType;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC3_Init();
  MX_ADC4_Init();
  MX_DAC_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_ADC2_Init();
  MX_COMP4_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_COMP2_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();
  MX_CRC_Init();

  /* USER CODE BEGIN 2 */
	
	
	HAL_TIM_OC_Init(&htim15);
	HAL_TIM_OC_Start(&htim15,TIM_CHANNEL_1);
	HAL_TIM_OC_Init(&htim16);
	HAL_TIM_OC_Start_IT(&htim16,TIM_CHANNEL_1);
	HAL_GPIO_WritePin(GPIOC,GPIO_DEF_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_TRANS_Pin,GPIO_PIN_SET);
	
	HAL_Delay(500);
	
	HAL_GPIO_WritePin(GPIOC,GPIO_DEF_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_TRANS_Pin,GPIO_PIN_RESET);
	
	
	
	
	/*HAL_DAC_Start(&hdac,DAC_CHANNEL_1);
  HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,1240);
	
	HAL_DAC_Start(&hdac,DAC_CHANNEL_2);
  HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,1240);
	
	HAL_COMP_Init(&hcomp2);
	HAL_COMP_Init(&hcomp4);
	HAL_COMP_Start_IT(&hcomp2);
	HAL_COMP_Start_IT(&hcomp4);
*/
	
	HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);
	tmp = HAL_ADCEx_Calibration_GetValue(&hadc1,ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_SetValue(&hadc1,ADC_SINGLE_ENDED,tmp);
	HAL_Delay(1);
	HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED);
	tmp = HAL_ADCEx_Calibration_GetValue(&hadc2,ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_SetValue(&hadc2,ADC_SINGLE_ENDED,tmp);
	HAL_Delay(1);
	HAL_ADCEx_Calibration_Start(&hadc3,ADC_SINGLE_ENDED);
	tmp = HAL_ADCEx_Calibration_GetValue(&hadc3,ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_SetValue(&hadc3,ADC_SINGLE_ENDED,tmp);
	HAL_Delay(1);
	HAL_ADCEx_Calibration_Start(&hadc4,ADC_SINGLE_ENDED);
	tmp = HAL_ADCEx_Calibration_GetValue(&hadc4,ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_SetValue(&hadc4,ADC_SINGLE_ENDED,tmp);
	HAL_Delay(1);
	
	
	
	for(uint16_t j = 0; j < 1000; j++)
	{
		HAL_ADC_Start(&hadc1);			// запуск ацп
		HAL_ADC_Start(&hadc2);
		HAL_ADC_Start(&hadc3);
		HAL_ADC_Start(&hadc4);
		
		HAL_ADC_PollForConversion(&hadc1,1);
		HAL_ADC_PollForConversion(&hadc2,1);
		HAL_ADC_PollForConversion(&hadc3,1);
		HAL_ADC_PollForConversion(&hadc4,1);

		mean1 += HAL_ADC_GetValue(&hadc1);
		mean2 += HAL_ADC_GetValue(&hadc2);
		mean3 += HAL_ADC_GetValue(&hadc3);
		mean4 += HAL_ADC_GetValue(&hadc4);
		
		HAL_ADC_Stop(&hadc1);
		HAL_ADC_Stop(&hadc2);
		HAL_ADC_Stop(&hadc3);
		HAL_ADC_Stop(&hadc4);
		
	}
	
	mean1 /= 1000;
	mean2 /= 1000;
	mean3 /= 1000;
	mean4 /= 1000;
	
	mean1 *= Vsupp/Effbit;
	mean2 *= Vsupp/Effbit;
	mean3 *= Vsupp/Effbit;
	mean4 *= Vsupp/Effbit;
	
	//if(*(__IO uint32_t*)(ADR_START) != 0xFFFFFFFF)
		//LoadSetting();
	
	//Us4 = CalculateSupply(0, 0, 1, 4);
	//SETUP.cap = 1 ;
	//C4 = 8.55e-6f;
	
	
	
	UART2_RecvType = UART2_RECV_CMD;
	HAL_UART_Receive_IT(&huart1,(uint8_t*)&UART2RecvData.cmd, sizeof(UART2RecvData.cmd));
	
	
	SETUP.softsrart = ON;
	
	
	STATUS.trans_ok = 1;
	STATUS.drvenable = 1;
	CommandReply(U2CT_DRVENABLE, 'i', STATUS.drvenable);
	
	
	#ifdef DEBUG
//		f1 = 984;
//		f2 = 966;
//		BR = 2;
//		CodeMode = 0;
		SETUP.softsrart = 0;
		SoftStart = 0;
		RepeatNum = 11;
		f1 = 769;
		f2 = 719;
		BR = 3;
		CodeMode = 2;
		tmpData = 55;
		Parsing(55,0);
	#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		
		if(TIM2->CNT > 0xF0000)
			StopPWM();
	
		
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
