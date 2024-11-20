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

// Encryption and Decryption Function Prototypes
void encrypt(uint32_t v[2], const uint32_t k[4]);
void decrypt(uint32_t v[2], const uint32_t k[4]);
void encryptMessage(uint8_t* input, size_t len);
void decryptMessage(uint8_t* input, size_t len);

// Global Variables
uint8_t RxData[8];
uint8_t RxData_Encrypted[8];  // New buffer for encrypted data
uint8_t yPos = 0;
uint8_t prevYPos = 0;
char msg[50];
volatile uint8_t dataReceived = 0;
volatile uint8_t displayEncrypted = 0; // 0 for decrypted, 1 for encrypted


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
        // Store the encrypted data
        memcpy(RxData_Encrypted, RxData, sizeof(RxData));

        // Decrypt the received data for normal display
        decryptMessage(RxData, sizeof(RxData));

        // Extract yPos from decrypted data
        yPos = RxData[0];
        dataReceived = 1;

        // Re-enable reception before processing
        HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxData, sizeof(RxData));

        // Update OLED Display based on display mode
        if (displayEncrypted) {
            displayDecimalRxData(RxData_Encrypted, sizeof(RxData_Encrypted)); // Display raw encrypted data as ASCII
        } else {
            // Display decrypted yPos on the OLED
            SSD1306_ShiftBufferLeft();
            SSD1306_DrawVerticalLineInRightmostColumn(prevYPos, yPos, SSD1306_COLOR_WHITE);
            SSD1306_UpdateScreen();
            prevYPos = yPos;
        }
    }
}


void displayDecimalRxData(uint8_t *data, size_t len) {
    char buffer[4 * 8 + 3]; // Each byte up to 3 digits + ", " + brackets and null terminator

    buffer[0] = '['; // Start with an opening bracket
    size_t pos = 1;

    for (size_t i = 0; i < len; i++) {
        // Format each byte as a decimal number and add to buffer
        pos += snprintf(&buffer[pos], 5, "%u", data[i]);

        // Add comma and space if it's not the last element
        if (i < len - 1) {
            buffer[pos++] = ',';
            buffer[pos++] = ' ';
        }
    }

    buffer[pos++] = ']'; // Closing bracket
    buffer[pos] = '\0';   // Null-terminate the string

    // Clear the OLED and display the formatted string
    SSD1306_Clear();
    SSD1306_GotoXY(0, 0); // Start at top-left corner
    SSD1306_Puts(buffer, &Font_7x10, SSD1306_COLOR_WHITE); // Choose font as appropriate
    SSD1306_UpdateScreen();
}


int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART1_UART_Init();

    HAL_HalfDuplex_EnableReceiver(&huart1);
    HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxData, sizeof(RxData));

    SSD1306_Init();
    SSD1306_Clear();
    SSD1306_UpdateScreen();

    while (1) {
        if (dataReceived) {
            dataReceived = 0; // Reset the flag

            if (displayEncrypted) {
                displayDecimalRxData(RxData_Encrypted, sizeof(RxData_Encrypted)); // Show raw RxData
            } else {
                SSD1306_ShiftBufferLeft();
                SSD1306_DrawVerticalLineInRightmostColumn(prevYPos, yPos, SSD1306_COLOR_WHITE);
                SSD1306_UpdateScreen();
                prevYPos = yPos;
            }

            HAL_Delay(50); // Optional delay to debounce button presses
        }
    }

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
  // Configure PC13 as Input with External Interrupt
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  // Enable interrupt in NVIC
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);


/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_13) {
        displayEncrypted = !displayEncrypted; // Toggle display mode
        dataReceived = 1;                     // Trigger display update
    }
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
