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
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);

// Encryption and Decryption Function Prototypes
void encrypt(uint32_t v[2], const uint32_t k[4]);
void decrypt(uint32_t v[2], const uint32_t k[4]);
void encryptMessage(uint8_t* input, size_t len);
void decryptMessage(uint8_t* input, size_t len);

// Global Variables
uint8_t RxData[8];
uint8_t RxData_Encrypted[8];
uint8_t yPos = 0;
uint8_t prevYPos = 0;
char msg[50];
volatile uint8_t dataReceived = 0;
volatile uint8_t i2cBusy = 0;
volatile uint8_t oledUpdateNeeded = 0;
#define OLED_UPDATE_INTERVAL 10  // Minimum time between OLED updates in ms

uint32_t lastOledUpdate = 0;

// Encryption key - must be the same on both sender and receiver
const uint32_t key[4] = {1, 2, 3, 4};

// Buffer for encryption/decryption operations (2 uint32_t for 8 bytes)
uint32_t data_buffer[2];

void UpdateOLEDSafe(void) {
    uint32_t currentTime = HAL_GetTick();

    // Check if enough time has passed since last update
    if (currentTime - lastOledUpdate < OLED_UPDATE_INTERVAL) {
        return;
    }

    // Set busy flag before starting I2C operation
    i2cBusy = 1;

    // Temporarily disable UART interrupts
    HAL_NVIC_DisableIRQ(USART1_IRQn);

    // Perform OLED updates
    SSD1306_ShiftBufferLeft();
    SSD1306_DrawVerticalLineInRightmostColumn(prevYPos, yPos, SSD1306_COLOR_WHITE);
    SSD1306_UpdateScreen();

    // Re-enable UART interrupts
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    // Clear busy flag
    i2cBusy = 0;
    lastOledUpdate = currentTime;
    prevYPos = yPos;
}

// Transmission Complete Callback (Not used in Receiver)
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    // Not required for Receiver unless transmitting data
}

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

    // Process each 8-byte block
    for (size_t i = 0; i < len; i += 8) {
        uint32_t v[2];
        memcpy(&v, &input[i], sizeof(v));

        decrypt(v, key);

        memcpy(&input[i], v, sizeof(v));
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart == &huart1 && Size > 0) {
        if (Size >= sizeof(RxData)) {
            Size = sizeof(RxData);
        }

        // Only process data if I2C is not busy
        if (!i2cBusy) {
            // Store encrypted data first
            memcpy(RxData_Encrypted, RxData, Size);

            // Set flag for main loop to process
            dataReceived = 1;
        }

        // Re-enable reception
        HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxData, sizeof(RxData));
    }
}

int main(void) {
    // Initialize the Hardware Abstraction Layer
    HAL_Init();

    // Configure the system clock
    SystemClock_Config();

    // Initialize all configured peripherals
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART1_UART_Init();

    // Initialize the OLED display
    if (SSD1306_Init() != 0) { // Modify SSD1306_Init to accept I2C handle if necessary
        sprintf(msg, "OLED Init Failed\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        Error_Handler();
    } else {
        sprintf(msg, "OLED Init Success\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }

    // Clear the display initially
    SSD1306_Clear();
    SSD1306_UpdateScreen();

    // Initialize UART Reception in Half-Duplex Mode
    HAL_HalfDuplex_EnableReceiver(&huart1);
    HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxData, sizeof(RxData));

    while(1) {
        if (dataReceived && !i2cBusy) {
            // Process received data only when I2C is not busy
            decryptMessage(RxData, sizeof(RxData));
            yPos = RxData[0];
            oledUpdateNeeded = 1;
            dataReceived = 0;
        }

        if (oledUpdateNeeded && !i2cBusy) {
            UpdateOLEDSafe();
            oledUpdateNeeded = 0;
        }

        // Small delay to prevent tight polling
        HAL_Delay(1);
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

static void MX_GPIO_Init(void){
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Enable GPIO Ports Clock
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE(); // Enable GPIOB for I2C1

    // Configure PC13 as Input with External Interrupt
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // Configure I2C1 SCL and SDA (Assuming PB6=SCL, PB7=SDA)
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1; // AF4 for I2C1 on many STM32s
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Set EXTI Line 15_10 Interrupt Priority and Enable it
    HAL_NVIC_SetPriority(EXTI15_10_IRQn,0,0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

static void MX_I2C1_Init(void) {
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 400000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if(HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }

    // Enable analog and digital filtering
    HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE);
    HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 4);  // Add digital filtering
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
    // Send UART error message
    sprintf(msg, "Error_Handler Invoked\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
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