/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
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
#include "stm32f0xx_hal.h"
#include "stdlib.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
enum state_machine{
	START_ADC,
	DATA_ADC_AQUIRED,
	DRIVE_MOTOR
};

enum{
	ZERO = 0x0000003F,
	ONE = 0x00000006,
	TWO = 0x0000005B,
	THREE = 0x0000004F,
	FOUR = 0x00000066,
	FIVE = 0x0000006D,
	SIX = 0x0000007D,
	SEVEN = 0x00000007,
	EIGHT = 0x0000007F,
	NINE = 0x00000067
} digit;

uint32_t digit_array[10] = {ZERO,ONE,TWO,THREE,FOUR,FIVE,SIX,SEVEN,EIGHT,NINE};

volatile uint8_t flag_start_ADC = FALSE, flag_ADC_interrupt_completed = FALSE;
uint32_t adc_voltage_sum = 0, adc_raw_data_sum = 0;
uint8_t adc_sample = 0;
uint16_t S2_profil_pulse = 0, S1_volet_pulse = 0, angle = 0;
float adc_voltage = 0, adc_raw_data = 0, voltage_ratio = 0;
int16_t S2_profil_pulse_difference = 0, S1_volet_pulse_difference;
uint8_t digit_1 = 0, digit_2 = 0, switch_digits_bit = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM1_Init(void);
static void USER_Start_Servo(void);
static void USER_Display_Angle(uint16_t mot_pulse);
static void USER_activate_fist_digit(void);
static void USER_activate_second_digit(void);

enum state_machine USER_state_machine = START_ADC;

                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

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
  MX_ADC_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
// 
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	HAL_ADC_Start_IT(&hadc);
  while (1)
  {
		switch(USER_state_machine)
		{
			case START_ADC:
				if(adc_sample == ADC_SAMPLES)
				{
					// Change tje state of the STATE-MACHINE
					USER_state_machine = DATA_ADC_AQUIRED;
					break;
				}
				else
				{
					if(flag_start_ADC)
					{
						// This condition will be enetered only after the previous adc value is read.
						// The flag is made TRUE by the SysTick intrrupt handler in stm32f0xx_it.c
						HAL_ADC_Start_IT(&hadc);
						flag_start_ADC = FALSE;
					}
				}
				break;
				
			case DATA_ADC_AQUIRED:
				adc_raw_data = (float)(adc_raw_data_sum)/ADC_SAMPLES;
				adc_voltage = adc_raw_data * 3290/ADC_REZOLUTION_12_BIT;
				voltage_ratio = adc_raw_data/ADC_REZOLUTION_12_BIT;
				
			// Clear the variables
				adc_sample = 0;
				adc_raw_data_sum = 0;
				adc_voltage_sum = 0;
			// Change tje state of the STATE-MACHINE
				USER_state_machine = DRIVE_MOTOR;
				break;
			
			case DRIVE_MOTOR:
				USER_Start_Servo();
			// Change tje state of the STATE-MACHINE
				USER_state_machine = START_ADC;
				break;

		}
  }

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(__HAL_ADC_GET_FLAG(hadc,ADC_FLAG_EOC))
	{
		adc_raw_data = HAL_ADC_GetValue(hadc);
		adc_raw_data_sum += adc_raw_data;
//		adc_voltage_sum += 3290.0/ADC_REZOLUTION_12_BIT * adc_raw_data;
		adc_sample++;
		flag_ADC_interrupt_completed = TRUE;
	}
}

/**
  * @brief  Drive the motor to the corresponding ratio (read value/ max value) read by the adc.
  * @note   The motor that is active and driven by the executiuon of this function depends on the selected mod (VOLET or PROFIL)
  *
  * @param  This function does not accept arguments

  * @retval None
  */
void USER_Start_Servo()
{
	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7) == S2_profil)
	{
		//  Mode PROFIL selected by the USER
		if (S2_profil_pulse !=0)	
		{
			// To modify the previous value of the OCP timer that will drive the motor to a new position,
			// the USER must firstly come close to the previous value of the rotary switch (read by the ADC)
			S2_profil_pulse_difference = S2_profil_pulse - ((uint16_t)(voltage_ratio*MIN_MOTOR_PULSE)+MIN_MOTOR_PULSE);
			
			// We aleatory select a 10% error of the previous pulse value to take control of the motor
			// TIM1 CNT register counts every 1us. Therefore 5% of the 
			if(abs(S2_profil_pulse_difference) < 100)
			{
				// Here the USER took control of the motor and updates its position
				S2_profil_pulse = (uint16_t)(voltage_ratio*MIN_MOTOR_PULSE)+ MIN_MOTOR_PULSE;

			}
		}
		else	// Enters one time (only for the first time when the USER selects this mode)
		{
			S2_profil_pulse = (uint16_t)(voltage_ratio*MIN_MOTOR_PULSE) + MIN_MOTOR_PULSE;
		}

		// We make sure that the MINIMUM and MAXIMUM allowed values aren't violated 
		if(S2_profil_pulse < MIN_MOTOR_PULSE)	S2_profil_pulse = MIN_MOTOR_PULSE;
		if(S2_profil_pulse > MAX_MOTOR_PULSE)	S2_profil_pulse = MAX_MOTOR_PULSE;
		// Update the corresponding register of the TIMER to produce the desired PWM value
		//TIM1->CCR2 = S2_profil_pulse;
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, S2_profil_pulse);
		
		USER_Display_Angle(S2_profil_pulse);
	}
	
	else
	{
		// Mode VOLET selected by the USER
		// The folowing explanations are the same as above...
		if (S1_volet_pulse !=0)	
		{
			S1_volet_pulse_difference = S1_volet_pulse - ((uint16_t)(voltage_ratio*MIN_MOTOR_PULSE) + MIN_MOTOR_PULSE);
			if(abs(S1_volet_pulse_difference) < 100)
			{
				S1_volet_pulse = (uint16_t)(voltage_ratio*MIN_MOTOR_PULSE) + MIN_MOTOR_PULSE;
			}
		}
		else
		{
			S1_volet_pulse = (uint16_t)(voltage_ratio*MIN_MOTOR_PULSE)+MIN_MOTOR_PULSE;

		}
		// We make sure that the MINIMUM and MAXIMUM allowed values aren't violated 
		if(S1_volet_pulse < MIN_MOTOR_PULSE)	S2_profil_pulse = MIN_MOTOR_PULSE;
		if(S1_volet_pulse > MAX_MOTOR_PULSE)	S2_profil_pulse = MAX_MOTOR_PULSE;
		// Update the corresponding register of the TIMER to produce the desired PWM value
		//TIM1->CCR3 = S1_volet_pulse;
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3, S1_volet_pulse);
		
		USER_Display_Angle(S1_volet_pulse);
	}
}

static void USER_Display_Angle(uint16_t mot_pulse)
{
	// We first determine if the angle is positive or negative
	// 0 degrees corresponds to a pulse of 1500us
	if( mot_pulse> MEDIUM_MOTOR_PULSE + 10)
	{
		//Positive angle
		angle = (float)(mot_pulse-MEDIUM_MOTOR_PULSE)/(MEDIUM_MOTOR_PULSE-MIN_MOTOR_PULSE) * ANGLE_MAX_VAL;
	}
	else
	{
		if( mot_pulse < MEDIUM_MOTOR_PULSE - 10)
		{
			//Negative angle
			angle = (float)(MEDIUM_MOTOR_PULSE-mot_pulse)/(MEDIUM_MOTOR_PULSE-MIN_MOTOR_PULSE) * ANGLE_MAX_VAL;
		}
		else
		{
			angle = 0;
		}
	}
	switch_digits_bit ^=0x01;
	if(switch_digits_bit)
	{
		digit_1 = angle%10;
		/* Display the first digit */
		// First we clear the output register
		GPIOA->ODR &= CLEAR_MASK_REGISTER;
		// Now we set the the output register according to the digit1 value
		USER_activate_fist_digit();
		GPIOA->ODR |= digit_array[digit_1];
	}
	else
	{	
		digit_2 = angle/10;
		/* Display the second digit */
		// First we clear the output register
		GPIOA->ODR &= CLEAR_MASK_REGISTER;
		// Now we set the the output register according to the digit2 value
		USER_activate_second_digit();
		GPIOA->ODR |= digit_array[digit_2];
	}
}

static void USER_activate_fist_digit(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_14, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_SET);
}

static void USER_activate_second_digit(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_14, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_RESET);	
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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

/* ADC init function */
static void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
//  sConfig.Channel = ADC_CHANNEL_VREFINT;
//  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
//  {
//    _Error_Handler(__FILE__, __LINE__);
//  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 47;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = MEDIUM_MOTOR_PULSE;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through 
        * the Code Generation settings)
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, a_PA0_Pin|b_PA1_Pin|c_PA2_Pin|d_PA3_Pin 
                          |e_PA5_Pin|f_PA6_Pin|g_PA7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : a_PA0_Pin b_PA1_Pin c_PA2_Pin d_PA3_Pin 
                           e_PA5_Pin f_PA6_Pin g_PA7_Pin */
  GPIO_InitStruct.Pin = a_PA0_Pin|b_PA1_Pin|c_PA2_Pin|d_PA3_Pin 
                          |e_PA5_Pin|f_PA6_Pin|g_PA7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	/*Configure GPIO pin : GPIO_Pin_13, GPIO_Pin_14 */
	// Select digit pins
	GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SLIDE_Pin */
	GPIO_InitStruct.Pin = SLIDE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//  GPIO_InitStruct.Pin = SLIDE_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(SLIDE_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
//  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 1, 0);
//  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
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
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
