#include "main.h"
#include <stdio.h>
#include <string.h>
#include "fonts.h"
#include "ssd1306.h"

// Peripheral Handles
UART_HandleTypeDef huart1;
I2C_HandleTypeDef hi2c1;

// Function Prototypes
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);

void encrypt(uint32_t v[2], const uint32_t k[4]);
void decrypt(uint32_t v[2], const uint32_t k[4]);
void encryptMessage(uint8_t* input, size_t len);
void decryptMessage(uint8_t* input, size_t len);

// Global Variables
uint8_t RxData[8];
uint8_t RxData_Encrypted[8];  // New buffer for encrypted data
uint8_t yPos = 0;
char msg[50];
volatile uint8_t dataReceived = 0;

// Encryption key - must be the same on both sender and receiver
const uint32_t key[4] = {1, 2, 3, 4};

// Buffer for encryption/decryption operations (2 uint32_t for 8 bytes)
uint32_t data_buffer[2];

// Decryption Function
void decrypt(uint32_t v[2], const uint32_t k[4]) {
    uint32_t v0 = v[0], v1 = v[1], sum = 0xC6EF3720, i;
    uint32_t delta = 0x9E3779B9;
    uint32_t k0 = k[0], k1 = k[1], k2 = k[2], k3 = k[3];
    for (i = 0; i < 32; i++) {
        v1 -= ((v0 << 4) + k2) ^ (v0 + sum) ^ ((v0 >> 5) + k3);
        v0 -= ((v1 << 4) + k0) ^ (v1 + sum) ^ ((v1 >> 5) + k1);
        sum -= delta;
    }
    v[0] = v0;
    v[1] = v1;
}

// Decrypt Message Function
void decryptMessage(uint8_t* input, size_t len) {
    // Ensure len is a multiple of 8
    if (len % 8 != 0) {
        // Handle padding if necessary
        // For simplicity, ignore extra bytes
        len -= len % 8;
    }

    // Copy input to buffer as uint32_t
    memcpy(data_buffer, input, len);

    // Decrypt in blocks of 8 bytes (2 uint32_t)
    for (int i = 0; i < (len / 4); i += 2) {
        decrypt(&data_buffer[i], key);
    }

    // Copy back to input buffer
    memcpy(input, data_buffer, len);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart == &huart1 && Size > 0) {
        // Store encrypted data first
        memcpy(RxData_Encrypted, RxData, sizeof(RxData));

        // Decrypt the received data
        decryptMessage(RxData, sizeof(RxData));

        // Extract yPos from decrypted data
        yPos = RxData[0];
        dataReceived = 1;

        // Re-enable reception before processing
        HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxData, sizeof(RxData));

        // Format and transmit debug message
        sprintf(msg, "yPos: %u\r\n", yPos);

        // Switch to transmit mode for debug output
        HAL_HalfDuplex_EnableTransmitter(&huart1);
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        HAL_HalfDuplex_EnableReceiver(&huart1);
    }
}


int main(void)
{

    // Initialize the Hardware Abstraction Layer
    HAL_Init();

    // Configure the system clock
    SystemClock_Config();

    // Initialize all configured peripherals
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART1_UART_Init();

    // Initialize UART Reception in Half-Duplex Mode
    HAL_HalfDuplex_EnableReceiver(&huart1);
    HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxData, sizeof(RxData));



  SSD1306_Init();
  SSD1306_GotoXY (0,0);
  SSD1306_Puts ("MICROPETA", &Font_11x18, 1);
  HAL_Delay(5000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while(1) {
          if (dataReceived) {
              // Process received data if needed
              dataReceived = 0;

              // Optional: Add a small delay to prevent overwhelming the UART
              HAL_Delay(10);
          }
      }
  /* USER CODE END 3 */
}

static void MX_USART1_UART_Init(void) {
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX; // Enable both TX and RX for Half-Duplex
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_HalfDuplex_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
}




static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

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

void SystemClock_Config(void){
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    // Configure the main internal regulator output voltage
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    // Initialize RCC Oscillators
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 84;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK){
        Error_Handler();
    }

    // Initialize CPU, AHB and APB Clocks
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2)!= HAL_OK){
        Error_Handler();
    }
}

void Error_Handler(void){
    // Toggle an LED or indicate error in another way
    while (1){
        // Example: Toggle LED on PA5
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        HAL_Delay(500);
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line){}
#endif
