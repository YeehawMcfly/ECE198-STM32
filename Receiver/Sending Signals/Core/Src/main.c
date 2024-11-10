#include "main.h"
#include <stdio.h>
#include <string.h>

// Peripheral Handles
UART_HandleTypeDef huart1;

// Function Prototypes
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);

// Encryption and Decryption Function Prototypes
void encrypt(uint32_t v[2], const uint32_t k[4]);
void decrypt(uint32_t v[2], const uint32_t k[4]);
void encryptMessage(uint8_t* input, size_t len);
void decryptMessage(uint8_t* input, size_t len);

// Global Variables
uint8_t RxData[8];
uint8_t yPos = 0;
char msg[50];
volatile uint8_t dataReceived = 0;

// Encryption key - must be the same on both sender and receiver
const uint32_t key[4] = {1, 2, 3, 4};

// Buffer for encryption/decryption operations (2 uint32_t for 8 bytes)
uint32_t data_buffer[2];

/**
 * @brief Converts binary data to a hexadecimal string.
 *
 * @param bin Pointer to the binary data.
 * @param len Length of the binary data.
 * @param hex_out Pointer to the output buffer (must be at least len*2 + 1 bytes).
 */
void bin_to_hex(uint8_t* bin, size_t len, char* hex_out) {
    const char hex_chars[] = "0123456789ABCDEF";
    for (size_t i = 0; i < len; i++) {
        hex_out[i * 2] = hex_chars[(bin[i] >> 4) & 0x0F];
        hex_out[i * 2 + 1] = hex_chars[bin[i] & 0x0F];
    }
    hex_out[len * 2] = '\0'; // Null-terminate the string
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
        // Step 1: Convert Encrypted RxData to Hexadecimal String
        char encrypted_hex[17]; // 8 bytes * 2 chars + 1 null terminator
        bin_to_hex(RxData, 8, encrypted_hex); // Assuming TxData is 8 bytes

        // Step 2: Format and Transmit Encrypted Data
        sprintf(msg, "Encrypted Data: %s\r\n", encrypted_hex);

        // Switch to transmit mode to send the encrypted data
        HAL_HalfDuplex_EnableTransmitter(&huart1);
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        HAL_HalfDuplex_EnableReceiver(&huart1);

        // Step 3: Decrypt the received data
        decryptMessage(RxData, sizeof(RxData));

        // Extract yPos from decrypted data
        yPos = RxData[0];
        dataReceived = 1;

        // Re-enable reception for the next incoming data
        HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxData, sizeof(RxData));

        // Step 4: Format and transmit the decrypted yPos value
        sprintf(msg, "yPos: %u\r\n", yPos);

        // Switch to transmit mode for debug output
        HAL_HalfDuplex_EnableTransmitter(&huart1);
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        HAL_HalfDuplex_EnableReceiver(&huart1);
    }
}

int main(void) {
    // Initialize the Hardware Abstraction Layer
    HAL_Init();

    // Configure the system clock
    SystemClock_Config();

    // Initialize all configured peripherals
    MX_GPIO_Init();
    MX_USART1_UART_Init();

    // Initialize UART Reception in Half-Duplex Mode
    HAL_HalfDuplex_EnableReceiver(&huart1);
    HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxData, sizeof(RxData));

    while(1) {
        if (dataReceived) {
            // Process received data if needed
            dataReceived = 0;

            // Optional: Add a small delay to prevent overwhelming the UART
            HAL_Delay(10);
        }

        // Optional: Implement low-power modes or other tasks
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

    // Configure PC13 as Input with External Interrupt
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // Set EXTI Line 15_10 Interrupt Priority and Enable it
    HAL_NVIC_SetPriority(EXTI15_10_IRQn,0,0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
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
    __disable_irq();
    while (1){}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line){}
#endif
