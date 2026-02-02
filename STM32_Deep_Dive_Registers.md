# STM32 Register Deep Dive & PDF Analysis

This guide details the internal registers of the STM32F446RE, referencing the specific PDF documents you provided. This is crucial for "fill in the blanks" questions where you might be asked for specific addresses, bit definitions, or register names.

---

## 1. The Documents: What is where?

*   **`Datasheet_stm32f446re.pdf`**: This is the "Physical Map". It tells you **WHERE** things are (Memory Addresses) and **WHAT** the pins do (Pinout).
    *   *Key Section:* **Memory Map** (Figure 15, Table 12).
*   **`pm0214-stm32...programming-manual...pdf`**: This is the "Brain Manual". It covers the **Cortex-M4 Core** peripherals which are standard across *all* Cortex-M4 chips (not just STM32).
    *   *Key Sections:* **NVIC** (Nested Vectored Interrupt Controller) and **SysTick** (System Timer).

---

## 2. The Core Peripherals (Source: PM0214 Programming Manual)

These registers live inside the Cortex-M4 core. They control *how* code runs (interrupts, timing).

### A. NVIC (Nested Vectored Interrupt Controller)
*   **Location:** Starts at `0xE000 E100` (Internal Private Peripheral Bus).
*   **Function:** Manages all interrupts (EXTI, UART, TIM, etc.).
*   **Key Registers:**
    *   **`NVIC_ISERx` (Interrupt Set-Enable):** Writing `1` enables an interrupt.
        *   *Example:* To enable EXTI15_10, you set the bit corresponding to position 40.
    *   **`NVIC_ICERx` (Interrupt Clear-Enable):** Writing `1` disables an interrupt.
    *   **`NVIC_IPRx` (Interrupt Priority):** Sets the priority level (0-15). Lower number = Higher priority.

### B. SysTick (System Timer)
*   **Location:** `0xE000 E010` (See PM0214, Section 4.5).
*   **Function:** A simple countdown timer used to generate `HAL_Delay()` and the RTOS "heartbeat".
*   **Key Registers:**
    *   **`STK_CTRL` (Control & Status):**
        *   **Bit 0 (ENABLE):** Starts the timer.
        *   **Bit 1 (TICKINT):** Enables the interrupt when it hits zero (used for HAL_IncTick).
        *   **Bit 2 (CLKSOURCE):** 1 = Processor Clock (180 MHz), 0 = AHB/8.
    *   **`STK_LOAD` (Reload Value):** The value to count down from (e.g., 180,000 for 1ms at 180MHz).
    *   **`STK_VAL` (Current Value):** Reads the current count. Writing *any* value clears it.

---

## 3. The STM32 Peripherals (Source: Datasheet Memory Map + Projects)

These registers control the specific features of the chip.

### A. Memory Map (Addresses)
*Crucial for Fill-in-the-Blanks:* You often need to know which **Bus** a peripheral is on to enable its clock!

| Peripheral | Bus | Base Address (Hex) | Notes (from Datasheet Table 12) |
| :--- | :--- | :--- | :--- |
| **GPIOA** | AHB1 | `0x4002 0000` | High-speed GPIO |
| **GPIOC** | AHB1 | `0x4002 0800` | User Button is usually here (PC13) |
| **RCC** | AHB1 | `0x4002 3800` | Reset & Clock Control |
| **ADC1** | APB2 | `0x4001 2000` | Analog to Digital |
| **USART2** | APB1 | `0x4000 4400` | **Note:** APB1 is slower (45MHz max) |
| **TIM2** | APB1 | `0x4000 0000` | General Purpose Timer |

### B. RCC (Reset and Clock Control)
**Project Reference:** `01LA/Src/main.c`
Before you do *anything*, you must enable the clock here.
*   **`RCC->AHB1ENR`:** Enable GPIOs.
    *   Bit 0: `GPIOAEN`
    *   Bit 2: `GPIOCEN`
*   **`RCC->APB1ENR`:** Enable USART2, I2C1, TIM2/3/4.
*   **`RCC->APB2ENR`:** Enable ADC1, SPI1, SYSCFG (needed for EXTI).

### C. GPIO (General Purpose I/O)
**Project Reference:** `01LA` (Blinking LED)
Each Port (A, B, C...) has a set of registers at offsets from its Base Address.

1.  **`MODER` (Mode Register):** Offset `0x00`. 2 bits per pin.
    *   `00`: Input (Reset State)
    *   `01`: Output (LEDs)
    *   `10`: Alternate Function (UART, SPI)
    *   `11`: Analog (ADC)
    *   *Exam Tip:* To set PA5 to Output: `GPIOA->MODER |= (1 << 10);` (Bit 10 is the '1' for Pin 5).

2.  **`ODR` (Output Data Register):** Offset `0x14`.
    *   Write `1` to turn High, `0` to turn Low.
    *   *Code:* `GPIOA->ODR |= (1 << 5);` (Set PA5 High).

3.  **`IDR` (Input Data Register):** Offset `0x10`.
    *   Read-only.
    *   *Code:* `if (GPIOC->IDR & (1 << 13))` (Check Button PC13).

4.  **`AFR` (Alternate Function):** `AFR[0]` (Low) and `AFR[1]` (High).
    *   Used when `MODER` is `10`. Connects a pin to a peripheral (like USART2_TX to PA2).
    *   *Datasheet Check:* Look at the "Alternate Function Mapping" table in the Datasheet to find which AF number corresponds to USART2 (usually AF7).

### D. EXTI (External Interrupts)
**Project Reference:** `03C`
Connects GPIO pins to the NVIC.

1.  **`SYSCFG->EXTICR`:** Selects the Port (A, B, C...) for the EXTI line.
    *   *Code:* `SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC;` (PC13 to EXTI13).
2.  **`EXTI->IMR` (Interrupt Mask):** `1` = Interrupt Unmasked (Enabled).
3.  **`EXTI->RTSR` / `FTSR`:** Rising/Falling Trigger Selection.
4.  **`EXTI->PR` (Pending Register):**
    *   **CRITICAL:** When an interrupt happens, a `1` is set here.
    *   You **MUST** clear it manually in the ISR by writing a `1` to it.
    *   *Code:* `EXTI->PR |= (1 << 13);`

---

## 4. Exam Cheat Sheet (Fill in the Blanks)

*   **SysTick Control Register:** `STK_CTRL`
*   **GPIO Mode for ADC:** `Analog` (`11` in binary)
*   **Register to Clear Interrupt Flag:** `EXTI->PR`
*   **Bus for GPIO:** `AHB1`
*   **Bus for USART2:** `APB1`
*   **Base Address of GPIOA:** `0x4002 0000`
*   **Function to enable Interrupt in NVIC:** `NVIC_EnableIRQ(...)`
