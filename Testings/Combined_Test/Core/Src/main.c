/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fonts.h"
#include "ssd1306.h"
#include <string.h>  // Include for memset and memcpy

ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1;

/* Function Prototypes */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
void encrypt(uint32_t v[2], const uint32_t k[4]);

/* Variables */
const uint32_t key[4] = { 1, 2, 3, 4 };
uint8_t TxData[8];          // Buffer to hold encrypted data (8 bytes)
uint32_t data_buffer[2];    // Buffer for encryption (2 x 32-bit words)
uint32_t counter = 0;       // Counter to include in the data
uint16_t adcValue = 0;
uint8_t yPos = 0;
uint8_t prevYPos = 0;

int main(void)
{
    /* Initialization code */
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_ADC1_Init();
    MX_USART1_UART_Init();

    SSD1306_Init();

    /* Initial ADC read to initialize prevYPos */
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    adcValue = HAL_ADC_GetValue(&hadc1);
    yPos = SSD1306_HEIGHT - 1 - ((adcValue * (SSD1306_HEIGHT - 1)) / 4095);
    prevYPos = yPos;

    while (1)
    {
        /* Read ADC value */
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
        adcValue = HAL_ADC_GetValue(&hadc1);

        /* Map ADC value to yPos (0 to SSD1306_HEIGHT - 1) */
        yPos = SSD1306_HEIGHT - 1 - ((adcValue * (SSD1306_HEIGHT - 1)) / 4095);

        /* Shift display buffer to the left */
        SSD1306_ShiftBufferLeft();

        /* Draw vertical line between prevYPos and yPos in rightmost column */
        SSD1306_DrawVerticalLineInRightmostColumn(prevYPos, yPos, SSD1306_COLOR_WHITE);

        /* Update the display */
        SSD1306_UpdateScreen();

        /* Update prevYPos */
        prevYPos = yPos;

        /* Prepare data for encryption */
        data_buffer[0] = (uint32_t)adcValue;  // ADC value
        data_buffer[1] = counter++;           // Increment counter

        /* Encrypt the data */
        encrypt(data_buffer, key);

        /* Copy encrypted data to TxData buffer */
        memcpy(TxData, data_buffer, sizeof(data_buffer));

        /* Send encrypted data over UART */
        HAL_UART_Transmit(&huart1, TxData, sizeof(TxData), 1000);

        /* Delay */
        HAL_Delay(50);
    }
}

/* Encryption function */
void encrypt(uint32_t v[2], const uint32_t k[4]) {
    uint32_t v0=v[0], v1=v[1], sum=0, i;
    uint32_t delta=0x9E3779B9;
    for (i=0; i<32; i++) {
        sum += delta;
        v0 += ((v1<<4) + k[0]) ^ (v1 + sum) ^ ((v1>>5) + k[1]);
        v1 += ((v0<<4) + k[2]) ^ (v0 + sum) ^ ((v0>>5) + k[3]);
    }
    v[0]=v0; v[1]=v1;
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
