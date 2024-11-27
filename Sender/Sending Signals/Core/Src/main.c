#include "main.h"
#include "fonts.h"
#include "ssd1306.h"
#include <string.h>

// Peripheral Handles
ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1;

// Function Prototypes
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);

// Encryption and Decryption Function Prototypes
void encrypt(uint32_t v[2], const uint32_t k[4]);
void decrypt(uint32_t v[2], const uint32_t k[4]);
void encryptMessage(uint8_t* input, size_t len);
void decryptMessage(uint8_t* input, size_t len);

// Global Variables
// TxData and RxData are 8 bytes to match the encryption block size
uint8_t TxData[8] = {0};
uint8_t yPos = 0;
uint8_t prevYPos = 0;
volatile uint8_t transmissionComplete = 1;
volatile uint8_t displayVoltage = 0; // 0 for graph view, 1 for voltage view
float prevVoltage = -1.0f;           // Store previously displayed voltage


// Encryption key - must be the same on both sender and receiver
const uint32_t key[4] = {1, 2, 3, 4};

// Buffer for encryption/decryption operations (2 uint32_t for 8 bytes)
uint32_t data_buffer[2];

// Transmission Complete Callback
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart1) {
        transmissionComplete = 1;
    }
}

// Encryption Function
void encrypt(uint32_t v[2], const uint32_t k[4]) {
    uint32_t v0 = v[0], v1 = v[1], sum = 0, i;
    uint32_t delta = 0x9E3779B9;
    uint32_t k0 = k[0], k1 = k[1], k2 = k[2], k3 = k[3];
    for (i = 0; i < 32; i++) {
        sum += delta;
        v0 += ((v1 << 4) + k0) ^ (v1 + sum) ^ ((v1 >> 5) + k1);
        v1 += ((v0 << 4) + k2) ^ (v0 + sum) ^ ((v0 >> 5) + k3);
    }
    v[0] = v0;
    v[1] = v1;
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

// Encrypt Message Function
void encryptMessage(uint8_t* input, size_t len) {
    // Ensure len is a multiple of 8
    if (len % 8 != 0) {
        // Handle padding if necessary
        memset(input + len, 0, 8 - (len % 8));
        len += 8 - (len % 8);
    }

    // Clear the buffer first
    memset(data_buffer, 0, sizeof(data_buffer));

    // Copy input data to buffer as uint32_t
    memcpy(data_buffer, input, len);

    // Encrypt in blocks of 8 bytes (2 uint32_t)
    for (int i = 0; i < (len / 4); i += 2) {
        encrypt(&data_buffer[i], key);
    }

    // Copy back to input buffer
    memcpy(input, data_buffer, len);
}

// Decrypt Message Function (Not used in Sender)
void decryptMessage(uint8_t* input, size_t len) {
    // Sender does not decrypt, so this can remain empty or be removed
}

int main(void) {
    // Initialize the Hardware Abstraction Layer
    HAL_Init();

    // Configure the system clock
    SystemClock_Config();

    // Initialize all configured peripherals
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_ADC1_Init();
    MX_USART1_UART_Init();

    // Initialize the SSD1306 OLED display
    SSD1306_Init();

    // Perform an initial ADC conversion to set yPos and prevYPos
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) {
        uint16_t adcValue = HAL_ADC_GetValue(&hadc1);
        yPos = SSD1306_HEIGHT - 1 - ((adcValue * (SSD1306_HEIGHT - 1)) / 4095);
        prevYPos = yPos;
    }

    while (1) {
        if (transmissionComplete) {
            HAL_ADC_Start(&hadc1);

            if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) {
                uint16_t adcValue = HAL_ADC_GetValue(&hadc1);
                float lux = adcValue; // Use ADC value as 'lux'
                float VREF = 3.3f;    // Reference voltage
                float ADC_RESOLUTION = 4095.0f; // 12-bit ADC resolution
                float voltage = (lux * VREF) / ADC_RESOLUTION;
                float epsilon = 0.01f; // Threshold for voltage change
                yPos = SSD1306_HEIGHT - 1 - ((adcValue * (SSD1306_HEIGHT - 1)) / 4095);

                if (displayVoltage) {
                    // Display Voltage and yPos
                    char voltageStr[16];
                    char yPosStr[16];

                    // Format the voltage and yPos values
                    snprintf(voltageStr, sizeof(voltageStr), "Voltage: %.2fV", voltage);
                    snprintf(yPosStr, sizeof(yPosStr), "yPos: %u", yPos);

                    // Update OLED Display
                    SSD1306_Clear(); // Clear screen only when necessary
                    SSD1306_GotoXY(0, 0);
                    SSD1306_Puts(voltageStr, &Font_7x10, SSD1306_COLOR_WHITE);
                    SSD1306_GotoXY(0, 12); // Move to the next line
                    SSD1306_Puts(yPosStr, &Font_7x10, SSD1306_COLOR_WHITE);
                    SSD1306_UpdateScreen();
                } else {
                    SSD1306_ShiftBufferLeft();
                    SSD1306_DrawVerticalLineInRightmostColumn(prevYPos, yPos, SSD1306_COLOR_WHITE);
                    SSD1306_UpdateScreen();
                    prevYPos = yPos;
                }

                // Transmit Data (Always Transmit Regardless of Display Mode)
                memset(TxData, 0, sizeof(TxData));
                TxData[0] = yPos;

                // Encrypt the TxData before transmission
                encryptMessage(TxData, sizeof(TxData));
                HAL_HalfDuplex_EnableTransmitter(&huart1);
                transmissionComplete = 0;
                HAL_UART_Transmit_IT(&huart1, TxData, sizeof(TxData));
            }
        }

        HAL_Delay(1); // Minimal delay to prevent watchdog reset
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
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // Configure PC13 as Input with External Interrupt
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // Set EXTI Line 15_10 Interrupt Priority and Enable it
    HAL_NVIC_SetPriority(EXTI15_10_IRQn,0,0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_13) {
        displayVoltage = !displayVoltage; // Toggle display mode
    }
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

static void MX_I2C1_Init(void){
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 400000; // 400 kHz Fast Mode
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if(HAL_I2C_Init(&hi2c1)!= HAL_OK){
        Error_Handler();
    }
}

static void MX_ADC1_Init(void){
    ADC_ChannelConfTypeDef sConfig = {0};
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
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
    if (HAL_ADC_Init(&hadc1)!= HAL_OK){
        Error_Handler();
    }
    sConfig.Channel = ADC_CHANNEL_0;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
    if(HAL_ADC_ConfigChannel(&hadc1, &sConfig)!= HAL_OK){
        Error_Handler();
    }
}

void Error_Handler(void){
    __disable_irq();
    while (1){}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line){}
#endif
