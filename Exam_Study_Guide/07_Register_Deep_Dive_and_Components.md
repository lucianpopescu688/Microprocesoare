# Register Deep Dive & Component Drivers

This guide covers the **Low-Level** details often asked in exams but hidden by HAL. Use this to answer questions like "Write the initialization code for Timer 2 using registers" or "How do you configure the MPU6050?".

---

## 1. Timer Registers (The "Under the Hood" of HAL_TIM_Base_Start)

When you use `HAL_TIM_Base_Start_IT`, it actually sets specific bits in the hardware registers.

### Key Registers (TIMx)
Assume `TIM2` for these examples.

1.  **CR1 (Control Register 1):** The "ON/OFF" Switch.
    *   **Bit 0 (CEN):** Counter Enable. Set to `1` to start the timer.
    *   **Bit 4 (DIR):** Direction. `0` = Up-counter (default), `1` = Down-counter.
    *   **Code:** `TIM2->CR1 |= TIM_CR1_CEN;` (Start Timer)

2.  **DIER (DMA/Interrupt Enable Register):** The "Notification" Settings.
    *   **Bit 0 (UIE):** Update Interrupt Enable. Set to `1` to trigger an interrupt when the counter overflows.
    *   **Code:** `TIM2->DIER |= TIM_DIER_UIE;` (Enable Interrupt)

3.  **SR (Status Register):** The "Flag" Checker.
    *   **Bit 0 (UIF):** Update Interrupt Flag. Hardware sets this to `1` when overflow happens. **You must clear this manually in the ISR.**
    *   **Code:** `if (TIM2->SR & TIM_SR_UIF) { TIM2->SR &= ~TIM_SR_UIF; }`

### Complete Init Example (No HAL)
```c
// 1. Enable Clock for TIM2 (APB1 Bus)
RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

// 2. Configure Timing (1 Hz Interrupt)
// Clock = 84 MHz. Target = 10,000 ticks/sec.
TIM2->PSC = 8399;  // Prescaler
TIM2->ARR = 9999;  // Auto-Reload

// 3. Enable Interrupt source in Timer
TIM2->DIER |= TIM_DIER_UIE;

// 4. Enable Interrupt in NVIC (The Processor's Ear)
NVIC_EnableIRQ(TIM2_IRQn); 

// 5. Start the Timer
TIM2->CR1 |= TIM_CR1_CEN;
```

---

## 2. Interrupt Registers (NVIC & EXTI)

How does the processor know `PA0` was pressed?

### EXTI Registers (External Interrupt Controller)
*   **IMR (Interrupt Mask Register):** Unmask (Enable) the line.
    *   `EXTI->IMR |= (1 << 0);` (Enable EXTI Line 0)
*   **RTSR (Rising Trigger Selection):** Trigger on release?
    *   `EXTI->RTSR |= (1 << 0);`
*   **FTSR (Falling Trigger Selection):** Trigger on press?
    *   `EXTI->FTSR |= (1 << 0);`
*   **PR (Pending Register):** The "Flag". Cleared by writing `1` to it (counter-intuitive!).
    *   `EXTI->PR |= (1 << 0);` (Clear flag for Line 0)

### NVIC (Nested Vectored Interrupt Controller)
This belongs to the Cortex-M4 core, not the STM32 peripherals.
*   **ISER[x] (Interrupt Set-Enable):**
    *   `NVIC->ISER[0] |= (1 << 6);` (Enable EXTI0_IRQn, which is position 6)
    *   *Easier way:* `NVIC_EnableIRQ(EXTI0_IRQn);`

---

## 3. MPU6050 (Accelerometer/Gyro) - I2C

**Address:** `0xD0` (Write) / `0xD1` (Read).  (Base `0x68 << 1`)

### Vital Registers
1.  **PWR_MGMT_1 (0x6B):** Power Management.
    *   **Default:** The sensor starts in **Sleep Mode**.
    *   **Action:** Write `0x00` to wake it up.
    *   *Exam Tip:* If you read 0 values, you probably forgot to write to this register.

2.  **SMPLRT_DIV (0x19):** Sample Rate Divider.
    *   `Sample Rate = Gyro Rate / (1 + SMPLRT_DIV)`

3.  **ACCEL_CONFIG (0x1C):** Sensitivity.
    *   `0x00`: ±2g (Most sensitive)
    *   `0x18`: ±16g (Least sensitive)

4.  **ACCEL_XOUT_H (0x3B):** Data Start.
    *   The data is stored in 6 sequential bytes: `X_H`, `X_L`, `Y_H`, `Y_L`, `Z_H`, `Z_L`.

### Initialization Code (HAL)
```c
#define MPU_ADDR 0xD0

void MPU_Init(void) {
    uint8_t data;
    
    // 1. Wake up (PWR_MGMT_1 = 0)
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, 0x6B, 1, &data, 1, 100);
    
    // 2. Set Data Rate (SMPLRT_DIV = 7) -> 1kHz
    data = 0x07;
    HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, 0x19, 1, &data, 1, 100);
    
    // 3. Set Range (ACCEL_CONFIG = ±2g)
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, 0x1C, 1, &data, 1, 100);
}
```

---

## 4. ST7735 LCD - SPI Initialization Sequence

The LCD needs a "Magic Sequence" of commands to start working. These are sent with `DC` Pin **Low** (Command) or **High** (Data).

### Key Commands (From Datasheet/Driver)
1.  **SWRESET (0x01):** Software Reset. Wait 150ms after this.
2.  **SLPOUT (0x11):** Sleep Out. Turn on internal circuits. Wait 255ms.
3.  **COLMOD (0x3A):** Interface Pixel Format.
    *   Argument `0x05`: 16-bit/pixel (RGB 565).
4.  **MADCTL (0x36):** Memory Data Access Control (Rotation).
    *   Controls RGB vs BGR order and Mirroring (X/Y exchange).
5.  **DISPON (0x29):** Display On. The final step to show the image.

### Code Snippet (Concept)
```c
void LCD_Init() {
    // 1. Reset
    Select();
    WriteCmd(0x01); // SWRESET
    HAL_Delay(150);
    
    // 2. Wake Up
    WriteCmd(0x11); // SLPOUT
    HAL_Delay(255);
    
    // 3. Color Mode 16-bit
    WriteCmd(0x3A);
    WriteData(0x05);
    
    // 4. Turn On
    WriteCmd(0x29); // DISPON
    Unselect();
}
```

### Drawing a Pixel (Address Window)
To draw a pixel, you define a "Window" of size 1x1 on the screen memory, then write the color.
1.  **CASET (0x2A):** Column Address Set (Where is X?).
2.  **RASET (0x2B):** Row Address Set (Where is Y?).
3.  **RAMWR (0x2C):** RAM Write (Here is the color).
