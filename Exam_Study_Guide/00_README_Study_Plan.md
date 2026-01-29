# Microprocessors Programming Exam Study Guide (STM32F446RE)

This folder contains a comprehensive set of study notes based on your course material (Register-level and HAL programming).

## Recommended Study Order

### 1. The Fundamentals
Start with **[01_Register_Level_Basics.md](01_Register_Level_Basics.md)**.
*   **Why?** The exam often checks if you understand *how* the microcontroller works under the hood (Memory Map, Bitwise operations, RCC).
*   **Key Concept:** Enabling Clocks (`RCC->AHB1ENR`) is the most common missing step in exam questions!

### 2. Modern Programming (HAL)
Move to **[02_HAL_GPIO_and_Interrupts.md](02_HAL_GPIO_and_Interrupts.md)**.
*   **Why?** This is the standard way to program STM32. You need to know how to read inputs, drive outputs, and handle buttons via Interrupts (Callbacks).
*   **Key Concept:** The difference between `HAL_GPIO_ReadPin` (Polling) and `HAL_GPIO_EXTI_Callback` (Interrupts).

### 3. Timing & Analog
Study **[03_Timers_PWM_and_DAC.md](03_Timers_PWM_and_DAC.md)** and **[04_ADC_and_UART.md](04_ADC_and_UART.md)**.
*   **Why?** These are the core peripherals.
    *   **PWM:** Used for fading LEDs or controlling motors.
    *   **ADC:** Used for reading sensors.
    *   **UART:** Used for debugging.
*   **Key Concept:** Remember to `Start` peripherals (e.g., `HAL_TIM_PWM_Start`) before using them!

### 4. Advanced Communication
Review **[05_Communication_I2C_SPI.md](05_Communication_I2C_SPI.md)**.
*   **Why?** Your workspace includes MPU6050 (I2C) and ST7735 LCD (SPI). You might be asked to initialize them or send data.

### 5. Cheat Sheet
Keep **[06_Code_Snippets.md](06_Code_Snippets.md)** open during your practice.
*   It contains copy-paste ready blocks for `main.c`, mapping functions, and non-blocking delays.

---

## Quick Tips for the Exam
1.  **Clock Configuration:** Always call `SystemClock_Config()` in main.
2.  **Enable Peripherals:** In Register level, set the bit in RCC. In HAL, ensure `HAL_..._Start()` is called.
3.  **Volatile:** Use `volatile` for variables shared between Interrupts and Main loop.
4.  **Debouncing:** Buttons usually need software debouncing (check timestamps).
