# Essential Code Snippets (Cheat Sheet)

## 1. Minimal `main.c` Structure
If you need to write `main.c` from scratch or verify it.

```c
#include "main.h"

// Handles
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart2;

// Prototypes
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);

int main(void)
{
  // 1. System Init
  HAL_Init();
  SystemClock_Config();
  
  // 2. Peripheral Init
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();

  // 3. START PERIPHERALS (Crucial Step!)
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_UART_Receive_IT(&huart2, &rxData, 1);
  
  while (1)
  {
      // 4. Main Logic
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
      HAL_Delay(500);
  }
}
```

## 2. State Machine (Common Pattern)
Useful for handling complex logic (e.g., "Waiting for Button", "Measuring", "Displaying").

```c
typedef enum {
    STATE_IDLE,
    STATE_MEASURE,
    STATE_ALARM
} SystemState;

SystemState currentState = STATE_IDLE;

while(1) {
    switch(currentState) {
        case STATE_IDLE:
            if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 0) { // Button Pressed
                currentState = STATE_MEASURE;
            }
            break;
            
        case STATE_MEASURE:
            val = HAL_ADC_GetValue(&hadc1);
            if (val > 3000) {
                currentState = STATE_ALARM;
            } else {
                currentState = STATE_IDLE;
            }
            break;
            
        case STATE_ALARM:
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // Blink LED
            HAL_Delay(100);
            // Exit alarm if button pressed again?
            break;
    }
}
```

## 3. UART Command Parser
How to handle string commands like "LED ON" or "SET 100".

```c
uint8_t rxBuffer[20];
uint8_t rxIndex = 0;
uint8_t rxData;

// In Callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        if (rxData != '\n' && rxIndex < 19) {
            rxBuffer[rxIndex++] = rxData; // Store char
        } else {
            rxBuffer[rxIndex] = '\0'; // Null-terminate string
            rxIndex = 0; // Reset for next message
            
            // Process Command
            if (strcmp((char*)rxBuffer, "LED ON") == 0) {
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
            } 
            else if (strncmp((char*)rxBuffer, "SET ", 4) == 0) {
                int val = atoi((char*)rxBuffer + 4); // Parse number
                // Use val...
            }
        }
        HAL_UART_Receive_IT(&huart2, &rxData, 1); // Re-arm
    }
}
```

## 4. Map Function
Convert one range to another (ADC 0-4095 -> PWM 0-100).
```c
long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
```

## 5. Non-Blocking Delay
Avoid `HAL_Delay()` inside `while(1)` if you are doing multiple things.
```c
uint32_t last_time = 0;
while(1) {
    if (HAL_GetTick() - last_time >= 1000) { // 1000ms
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        last_time = HAL_GetTick();
    }
}
```