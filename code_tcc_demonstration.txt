/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#define ARM_MATH_CM4
#include <stdint.h>
#include <stdio.h>
#include "arm_const_structs.h"
#include "arm_math.h"

/* This program it's used for demonstration purpose*/

/* 
Calculation: length of delay (D)
	BUFFER_LENGHT = D*Fs 
	D = 100ms
	Fs = 96kHz
	BUFFER_LENGTH = 9600 samples
*/
#define BUFFER_LENGTH  9600// Tamanho do buffer para o delay
/* 
Calculation: length of flanger (D)
	BUFFER_LENGHT = D*Fs 
	D = 15ms
	Fs = 96kHz
	BUFFER_LENGTH = 1440 samples
*/


#define BUFFER_CHORUS  1440						 // Delay chorus


float32_t ALPHA =  0.2f;					// Wet mix, used in delay effect
float32_t MEAN_DELAY = 0.01f;				 // mean delay in seconds(used in LFO - Flanger)
float32_t MODULATION_MAG = 0.002f;		 // Wet mix, used in flanger effect


/* Private variables ---------------------------------------------------------*/
I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_i2s2_ext_rx;
DMA_HandleTypeDef hdma_spi2_tx;


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S2_Init(void);

/* Definition of x[n] right and left, y[n] left and right
		Using only left channel for this application
		x[n] --> xLeft
		y[n] --> yLeft
*/
float32_t xLeft, xRight, yLeft, yRight;

/*
	Using two buffers, one for delay "buffer"
	and the other for flanger "buffer_c"

*/
float32_t buffer[BUFFER_LENGTH]; 
float32_t buffer_c[BUFFER_CHORUS];

float gain = 0.7f; /* set gain value for echoed sample */

uint16_t oldest = 0; /* index for  delay effect buffer value */
/*
	Contains which switch is on
*/
int chave=0;

/*
	Transmit and receive buffers, for use in I2S protocol
	Length is 8, because i'm using 24bit ADC/DAC and the I2S
	Only use 16 bits, so we need use two 16bit words
	So, to transmit left and right channel we need 4 16bit buffer
	| left[31:15] | left[15:0] | right[31:15] | right[15:0]
	
*/

uint16_t rxBuf[8];
uint16_t txBuf[8];
// flanger LFO frequency
float32_t fdesi=0.5f;
// tremolo LFO frequency
float32_t fdesit=5.0f;
// sampling rate
float32_t fs=96000.0f;
float32_t theta;		// Argument to sin function, used in LFO for delay modulated effects (Flanger)
float32_t thetat;		// Argument to sin function, used in LFO for tremolo 
float32_t phaset;		// Argument to sin function, used in LFO for tremolo 	
float32_t phase; 	// Argument to sin function, used in LFO for tremolo 
float32_t delay_in_seconds;				// Variable that contains the value of delay in seconds for LFO, used for flanger
uint16_t delay_in_samples;				// Same variable, but now in samples

// Pointers for flanger effect
uint16_t read_ptr;								// Read or tail pointer, to be used in circular buffer
uint16_t write_ptr = 0; 					// Write or head pointer, to be used in circular buffers


// Do linear interpolation, used in flanger effect
double doLinearInterpolation(double y1, double y2, double fractional_X)
{
// --- check invalid condition
if (fractional_X >= 1.0) return y2;
// --- use weighted sum method of interpolating
return fractional_X*y2 + (1.0 - fractional_X)*y1;
}


/**
  * @brief  Delay effect routine, using circular buffer
  * @param  float32_t inSample the current variable received by ADC
  * @note   This routine does the difference equation y[n] = x[n]+g*x[n-D]
  * @retval yLeft, reprents y of difference equation
  */


float Delay(float32_t inSample){
	
	// uint32_t lSample = (uint32_t) (rxBuf[0]<<16)|rxBuf[1];
	 float32_t delayed = buffer[oldest];
	 yLeft=inSample+delayed;
	 buffer[oldest] = inSample;
	
   if (oldest++ >= BUFFER_LENGTH) {
        oldest = 0;
     }
	 return yLeft;
}


/**
  * @brief  Flanger effect routine, using circular buffer, and a sine wave LFO
  * @param  float32_t inSample the current variable received by ADC
  * @note   This routine does the difference equation y[n] = x[n]+ALPHA*x[n-(D+frac)]
  * @retval yLeft, reprents y of difference equation
  */
float flanger(float32_t inSample){
	

	//float32_t delayed = buffer_c[write_ptr];
	//float32_t delayed = buffer[write_ptr]+inSample;
	buffer_c[write_ptr] = inSample;
	// Update the pointer in the ring buffer (head or write)
	write_ptr = (write_ptr + 1)%BUFFER_CHORUS;
	
	// Calculate the argument for sin LFO
	theta = (float32_t)6.28318531f*fdesi/fs;
	phase += (float32_t)theta;
	if (phase >= 6.28318531f) {
		phase -= 6.28318531f;
	}
	// Calculate delay in seconds
	delay_in_seconds = MEAN_DELAY+MODULATION_MAG*arm_sin_f32(phase);
	// Calculate delay in samples
	delay_in_samples = (delay_in_seconds)*96000.0f;
	int M = floor(delay_in_samples);
	//float frac = delay_in_samples - M;
	// Update read pointer (tail or read)
	//write_ptr = (write_ptr + 1)%BUFFER_CHORUS;
	read_ptr = (write_ptr + BUFFER_CHORUS-delay_in_samples) % (BUFFER_CHORUS);
	
	// Do difference equation without interpolation
	yLeft = inSample+buffer_c[read_ptr];
	return yLeft;
}


/**
  * @brief  Tremolo effect routine, using circular buffer, and a sine wave LFO
  * @param  float32_t inSample the current variable received by ADC
  * @note   This routine does the difference equation y[n] = tremolo*x[n]
  * @retval yLeft, reprents y of difference equation
  */

float tremelo_mine(float32_t inSample){
	//y[n] = x[n]*(1-aLFO);
	thetat = (float32_t)6.28318531f*fdesit/fs;
	phaset += (float32_t)thetat;
		if (phaset >= 6.28318531f) {
		phaset -= 6.28318531f;
	}
	// Calculate tremolo factor, ranges from 1 to 0
	float trem_factor = 1.0f - (tremolo_deptht*(arm_sin_f32(phaset)));
	// Use a sin

	yLeft = trem_factor*inSample;
	return yLeft;
}

/**
  * @brief  Distortion effect routine
  * @param  float32_t inSample the current variable received by ADC
  * @note   This routine does a hard clip in the signal
  * @retval yLeft, reprents y of difference equation
  */



	float32_t y;
	//Performs symmetrical distortion based on Udo Zolzer's book -> Schetzen Formula
	
	if (fabs(x) < dist){
		y = 2 * x;
	} 
	
	if (fabs(x)>= dist){
		if (x > 0) {
			y = (3-((2-3*x)*(2-3*x)))/3;
		}
		else{
			y = -(3-((2-3*x)*(2-3*x)))/3;
		}
	}
	
	if(fabs(x)>2*dist){
		if(x > 0){
			y = x;
		}
		else{
			y = -x;
		}
	}
	return y;
}


	
	// Based on patent by Araya and Suyama

	yLeft = inSample*(1 - (inSample*inSample)/3);

	return yLeft;
}


float32_t Do_Distortionclip (float32_t insample) {


	float32_t threshold_higher = 500000.0f;

	if(fabs(insample)>threshold_higher){
			if ((insample) > 0){
				yLeft = threshold_higher;
			}
			else{
				yLeft = -threshold_higher;
			}
	}
	else {
		yLeft = insample;
	}
	return yLeft;
}

  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_I2S2_Init();
  /* USER CODE BEGIN 2 */
HAL_I2SEx_TransmitReceive_DMA (&hi2s2, txBuf, rxBuf, 4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}
void HAL_I2SEx_TxRxHalfCpltCallback(I2S_HandleTypeDef *hi2s){

	//restore signed 24 bit sample from 16-bit buffers
	int lSample = (((int)rxBuf[0]<<16)|rxBuf[1])>>8;
	int rSample = (((int)rxBuf[2]<<16)|rxBuf[3])>>8;
	//sum to mono
	//yLeft = (lSample+rSample)/2;
	xRight= (lSample+rSample)/2;
		switch(chave){
			case 0:
			// Should be nothing, pass-thgrouh 
				yLeft=xRight;
	  	//yLeft=xLeft;
				break;
			case 1:
				yLeft=flanger(xRight);
	//yLeft = xRight;
				break;
			case 2:
				yLeft=Delay(xRight);
				break;
			case 3:
				//yLeft=Do_Distortion(xRight);
			//yLeft=symmetrical1(xRight, 150000.0f);
			 yLeft = Do_Distortionclip(xRight);
				//	yLeft=symmetrical1(xRight);
				break;
			case 4:
				yLeft=tremelo(xRight);
			break;		
	}
		
	//restore to buffer
	txBuf[0] = ((int)yLeft>>8)&0xFFFF;
	txBuf[1] = (int)yLeft&0xFFFF;
	txBuf[2] = ((int)yLeft>>8)&0xFFFF;
	txBuf[3] = (int)yLeft&0xFFFF;
}

void HAL_I2SEx_TxRxCpltCallback(I2S_HandleTypeDef *hi2s){
	//unsigned int newest;  // only used for infinite echo
	//restore signed 24 bit sample from 16-bit buffers
	int lSample = (((int)rxBuf[4]<<16)|rxBuf[5])>>8;
	int rSample = (((int)rxBuf[6]<<16)|rxBuf[7])>>8;

//	yLeft = (lSample+rSample)/2;
	xRight= (lSample+rSample)/2;
		switch(chave){
			case 0:
			// Should be nothing, pass-thgrouh 
				yLeft=xRight;
	  	//yLeft=xLeft;
				break;
			case 1:
				yLeft=flanger(xRight);
	//yLeft = xRight;
				break;
			case 2:
				yLeft=Delay(xRight);
				break;
			case 3:
				//yLeft=Do_Distortion(xRight);
					//yLeft=symmetrical1(xRight, 50000.0f);
//					yLeft=symmetrical1(xRight, 150000.0f);
						yLeft = Do_Distortionclip(xRight);
				break;
			case 4:
				yLeft=tremelo(xRight);
			break;		
	}

	//restore to buffer
	txBuf[4] = ((int)yLeft>>8)&0xFFFF;
	txBuf[5] = (int)yLeft&0xFFFF;
	txBuf[6] = ((int)yLeft>>8)&0xFFFF;
	txBuf[7] = (int)yLeft&0xFFFF;
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 172;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_24B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
//  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
 // HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : sdelay_Pin */
  GPIO_InitStruct.Pin = sdelay_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(sdelay_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : stremolo_Pin */
  GPIO_InitStruct.Pin = stremolo_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(stremolo_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : schorus_Pin */
  GPIO_InitStruct.Pin = schorus_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(schorus_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : sdist_Pin */
  GPIO_InitStruct.Pin = sdist_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(sdist_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin){
	if(GPIO_Pin == sdelay_Pin){
		chave=1;
	//l_buf_out=Do_Delay(xLeft);
	//r_buf_out=Do_Delay(xRight);
	}
	else if (GPIO_Pin == sdist_Pin){
		chave=2;
	}
	else if (GPIO_Pin == schorus_Pin){
		chave=3;
	}
	else if (GPIO_Pin == schorus_Pin){
		chave=4;
	}
	//else if (GPIO_Pin == sreset_Pin){
	 //chave=0;
	//}
	else{
		chave=0;
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
