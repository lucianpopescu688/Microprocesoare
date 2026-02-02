# STM32 Advanced Deep Dive: Clocks, Startup & Timers
**Based on Project Analysis of `06LA`, `01LA` and others.**

This guide covers the advanced "under the hood" topics that are favorite targets for complex exam questions.

---

## 1. The Clock Tree (Calculated from `06LA`)
**File:** `06LA/Core/Src/main.c` -> `SystemClock_Config`

Understanding these numbers is key to calculating Timer Frequencies and Baud Rates.

### Step 1: The Input
*   **Source:** `HSI` (High Speed Internal) oscillator.
*   **Frequency:** **16 MHz** (Standard for STM32F4).

### Step 2: The PLL (Phase Locked Loop) Calculation
The code uses `PLLM = 16`, `PLLN = 336`, `PLLP = 4`.
The formula is: $$V_{CO} = HSI \times \frac{PLLN}{PLLM}$$ 
$$SYSCLK = \frac{V_{CO}}{PLLP}$$ 

1.  **Input Division:** $16\text{ MHz} / 16 = 1\text{ MHz}$.
2.  **Multiplication:** $1\text{ MHz} \times 336 = 336\text{ MHz}$.
3.  **Output Division:** $336\text{ MHz} / 4 = \mathbf{84\text{ MHz}}$.
4.  **Result:** **SYSCLK = 84 MHz**.

### Step 3: Bus Clocks (HCLK, PCLK1, PCLK2)
*   **AHB Prescaler (`HPRE`):** Div 1. $\to$ **HCLK = 84 MHz**.
*   **APB1 Prescaler (`PPRE1`):** Div 2. $\to$ **PCLK1 = 42 MHz**.
    *   *Note:* APB1 is the "Slow" bus (max 45 MHz).
*   **APB2 Prescaler (`PPRE2`):** Div 1. $\to$ **PCLK2 = 84 MHz**.
    *   *Note:* APB2 is the "Fast" bus (max 90 MHz).

### Step 4: The Tricky Timer Clock Rule
If the APB Prescaler is **not 1**, the Timer clock is **x2** the bus clock.
*   **TIM3 (on APB1):** APB1 is Div 2. So Timer Clock = $42\text{ MHz} \times 2 = \mathbf{84\text{ MHz}}$.

---

## 2. Timer Frequency Calculation (The "Prescaler" Question)
**Code:** `htim3.Init.Prescaler = 83;`

We established the Timer Clock is **84 MHz**.
The counter updates at a frequency:
$$F_{update} = \frac{F_{timer}}{Prescaler + 1}$$ 

$$F_{update} = \frac{84,000,000}{83 + 1} = \frac{84,000,000}{84} = \mathbf{1\text{ MHz}}$$ 

*   **Conclusion:** The timer counts up every **1 µs**.
*   If `Period (ARR)` is set to 999, the interrupt triggers every:
    $$(999 + 1) \times 1\text{ µs} = 1000\text{ µs} = \mathbf{1\text{ ms}}.$$ 

---

## 3. System Startup (The `Reset_Handler`)
**File:** `Startup/startup_stm32f446retx.s`

This file is written in **Assembly**. It is the *first* thing that runs.

1.  **Vector Table:** The CPU looks here first.
    *   Address `0x0000 0000`: **Initial Stack Pointer** (`_estack`).
    *   Address `0x0000 0004`: **Reset Handler** Address.

2.  **Reset Handler Sequence (Memorize this order):**
    1.  **Set Stack Pointer (`sp`):** `ldr r0, =_estack`, `mov sp, r0`.
    2.  **Call `SystemInit`:** Initializes the FPU and minimal Clock/Flash settings.
    3.  **Copy Data:** Copies initialized variables (`.data`) from Flash (ROM) to RAM.
    4.  **Zero BSS:** Clears uninitialized variables (`.bss`) in RAM to zero.
    5.  **Call `main`:** Finally jumps to your C code.

---

## 4. Communication Frames (Fill in the Blank Concepts)

### UART (Asynchronous)
*   **Lines:** TX, RX (Ground required).
*   **Idle State:** High (1).
*   **Start Bit:** 1 bit (Low/0).
*   **Data Bits:** Usually 8.
*   **Stop Bit:** 1 or 2 bits (High/1).
*   **Parity:** None, Even, or Odd.

### I2C (Synchronous)
*   **Lines:** SDA (Data), SCL (Clock).
*   **Start Condition:** SDA goes **Low** while SCL is **High**.
*   **Stop Condition:** SDA goes **High** while SCL is **High**.
*   **Ack (ACK):** Receiver pulls SDA **Low** on the 9th clock pulse.
*   **Nack (NACK):** SDA stays **High**.

### SPI (Synchronous)
*   **Lines:** MOSI, MISO, SCK, CS (Chip Select).
*   **Modes (CPOL/CPHA):**
    *   **CPOL (Polarity):** Idle Low (0) or High (1).
    *   **CPHA (Phase):** Sample on 1st edge or 2nd edge.
    *   *Exam Tip:* ST7735 (LCD) usually uses Mode 0 (CPOL=0, CPHA=0) or Mode 3.

---

## 5. Bitwise Logic Cheat Sheet
*   **Set bit 5:** `REG |= (1 << 5);`
*   **Clear bit 5:** `REG &= ~(1 << 5);`
*   **Toggle bit 5:** `REG ^= (1 << 5);`
*   **Check bit 5:** `if (REG & (1 << 5))`
*   **Set bits 4 and 5 (Binary 11):** `REG |= (3 << 4);`
