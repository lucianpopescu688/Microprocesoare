# HAL GPIO & Interrupts (STM32F446RE)

## 1. Initialization (What `HAL_Init` does)
When you create a project, `main()` calls `HAL_Init()` and `MX_GPIO_Init()`.
*   **`HAL_Init()`:**
    1.  Configures the Flash prefetch.
    2.  Sets the `SysTick` timer to generate an interrupt every 1ms (used for `HAL_Delay`).
    3.  Initializes low-level hardware.

*   **`MX_GPIO_Init()` Breakdown:**
    ```c
    // 1. Enable Clocks
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    // 2. Create Handle Struct
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // 3. Configure Pin Parameters
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Push-Pull
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    
    // 4. Apply Settings
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    ```

## 2. GPIO Functions
Commonly used functions in `stm32f4xx_hal_gpio.c`.

### Write Pin
```c
// Set PA5 High
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

// Set PA5 Low
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
```

### Toggle Pin
```c
// Toggle PA5 state (ON -> OFF, OFF -> ON)
HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
```

### Read Pin (Polling)
```c
// Read PC13 (User Button, Active Low)
if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) {
    // Button Pressed
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // LED ON
} else {
    // Button Released
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // LED OFF
}
```

## 3. External Interrupts (EXTI)
Used to respond to button presses immediately, without constantly checking `if(button == pressed)` in the loop.

### Analogy: Polling vs. Interrupts
*   **Polling (Standard Loop):** You are cooking pasta. You stand by the pot and stare at it, waiting for it to boil. You cannot do anything else (read a book, clean up) because you are busy checking the pot 100 times a second.
*   **Interrupt (EXTI):** You set a timer (or use a whistling kettle) and go sit in the living room to read. When the kettle whistles (**Interrupt Request**), you put down the book, go to the kitchen, handle the pasta (**ISR**), and then go back to exactly where you left off in your book.

### The Interrupt Lifecycle
1.  **Event:** User presses Button (Pin goes Low).
2.  **Hardware:** The EXTI Controller notices the change.
3.  **NVIC:** The Nested Vectored Interrupt Controller checks: "Is this interrupt enabled? Is it higher priority than what I'm doing now?"
4.  **Pause:** The CPU pauses the `main()` loop/function.
5.  **Vector Table:** CPU looks up the address of `EXTI15_10_IRQHandler`.
6.  **ISR (Interrupt Service Routine):** Code inside `stm32f4xx_it.c` runs.
7.  **Callback:** HAL calls `HAL_GPIO_EXTI_Callback()` in your `main.c`.
8.  **Resume:** CPU returns to `main()` exactly where it left off.

### How Mapping Works
*   **EXTI0** connects to **Pin 0** of **ANY** Port (PA0, PB0, PC0...).
*   **EXTI15_10** connects to Pins 10 through 15.
*   **Limitation:** You cannot use PA0 and PB0 as interrupts simultaneously.

### Setup (in `MX_GPIO_Init`)
1.  Enable GPIO Clock.
2.  Configure Pin as `GPIO_MODE_IT_FALLING` (trigger when pressed) or `GPIO_MODE_IT_RISING` (trigger when released).
3.  **NVIC (Nested Vectored Interrupt Controller):**
    *   Enable the IRQ Channel (`HAL_NVIC_EnableIRQ`).
    *   Set Priority (`HAL_NVIC_SetPriority`).

```c
// Auto-generated example for PC13
GPIO_InitStruct.Pin = GPIO_PIN_13;
GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
GPIO_InitStruct.Pull = GPIO_NOPULL;
HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

// Enable Interrupt Vector
// Priority 0 = Highest, 15 = Lowest
HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
```

### Handling the Interrupt (The Callback)
When the button is pressed, the hardware jumps to `EXTI15_10_IRQHandler` (in `stm32f4xx_it.c`), which calls `HAL_GPIO_EXTI_IRQHandler`, which finally calls your callback.

**Do NOT** modify `stm32f4xx_it.c`. Define the callback in `main.c`:

```c
/* USER CODE BEGIN 4 */
// This function is called automatically when ANY GPIO interrupt occurs
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // Check WHICH pin triggered the interrupt
    if (GPIO_Pin == GPIO_PIN_13) {
        // Toggle LED
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); 
    }
}
/* USER CODE END 4 */
```

**Golden Rules for Interrupts:**
1.  **Keep it Short:** Don't do complex calculations or `printf` inside.
2.  **No Blocking:** NEVER use `HAL_Delay()` inside an interrupt. It relies on the SysTick interrupt, which might be blocked by the current interrupt (priority deadlock).
3.  **Volatile:** Variables shared between the interrupt and `main()` must be declared `volatile` (e.g., `volatile uint8_t flag = 0;`).

## 4. Software Debouncing
Mechanical buttons "bounce" when pressed, creating multiple electrical spikes (triggers).
A simple software fix inside the callback:

```c
volatile uint32_t last_interrupt_time = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t current_time = HAL_GetTick(); // Get current time in ms
    
    if (GPIO_Pin == GPIO_PIN_13) {
        // Only accept input if > 200ms has passed since last press
        if ((current_time - last_interrupt_time) > 200) {
             HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
             last_interrupt_time = current_time;
        }
    }
}
```