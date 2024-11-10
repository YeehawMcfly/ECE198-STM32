#include "main.h"
#include <stdio.h>
#include <string.h>

UART_HandleTypeDef huart1;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
// void decrypt(uint32_t v[2], const uint32_t k[4]);
// void decryptMessage(uint8_t* input, size_t len);

uint8_t RxData[8];
const uint32_t key[4] = {1, 2, 3, 4};
uint32_t data_buffer[2];
char msg[50];

/*
void decrypt(uint32_t v[2], const uint32_t k[4]){
    uint32_t v0 = v[0], v1 = v[1], sum = 0xC6EF3720, i;
    uint32_t delta = 0x9E3779B9;
    for(i=0;i<32;i++){
        v1 -= ((v0<<4)+k[2]) ^ (v0 + sum) ^ ((v0>>5)+k[3]);
        v0 -= ((v1<<4)+k[0]) ^ (v1 + sum) ^ ((v1>>5)+k[1]);
        sum -= delta;
    }
    v[0]=v0; v[1]=v1;
}

void decryptMessage(uint8_t* input, size_t len){
    memcpy(data_buffer, input, len);
    int blocks = (len +7)/8;
    for(int i=0;i < blocks*2;i+=2){
        decrypt(&data_buffer[i], key);
    }
    memcpy(input, data_buffer, len);
}
*/

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
    if(Size >0){
        // decryptMessage(RxData, Size);
        // Removed decryption

        memcpy(data_buffer, RxData, sizeof(RxData));

        uint16_t adcValue = (uint16_t)(data_buffer[0] & 0xFFFF);
        uint32_t counter = data_buffer[1];
        float voltage = (adcValue * 3.3f)/4095.0f;
        sprintf(msg, "ADC Value: %u, Voltage: %.2f V, Counter: %lu\r\n", adcValue, voltage, counter);
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxData, sizeof(RxData));
    }
}

int main(void){
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART1_UART_Init();
    HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxData, sizeof(RxData));
    while(1){}
}

static void MX_USART1_UART_Init(void){
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if(HAL_UART_Init(&huart1)!= HAL_OK){
        Error_Handler();
    }
}

static void MX_GPIO_Init(void){
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(EXTI15_10_IRQn,0,0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void SystemClock_Config(void){
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 4;
    RCC_OscInitStruct.PLL.PLLN = 84;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct)!= HAL_OK){
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
    while(1){}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line){}
#endif
