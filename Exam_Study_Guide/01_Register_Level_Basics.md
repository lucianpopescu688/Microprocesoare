# Register-Level Programming (STM32F446RE)

## 0. The Mental Model (How to think about Registers)
Imagine the microcontroller as a massive control panel with thousands of tiny switches.
*   **A "Register"** is just a row of 32 switches (bits).
*   **The "Address"** is the unique label for that row (e.g., `0x40020000`).
*   **Writing Code** (`GPIOA->MODER |= ...`) is effectively reaching out with a robotic arm to flip specific switches ON or OFF.

**Why do we need to Enable Clocks?**
Think of the microcontroller like a large office building. To save electricity (battery), all the lights in every room (peripheral) are turned **OFF** by default.
*   The **RCC (Reset and Clock Control)** is the main breaker box.
*   Before you can do work in the "GPIO A Room", you must flip the breaker (`RCC->AHB1ENR`) to send power to that room. If you don't, writing to the registers does nothing—the room is dark!

## 1. Memory Map & Base Addresses
Accessing peripherals directly requires knowing their base addresses. All peripherals are mapped to specific memory locations.

*   **AHB1 Bus (High Speed):** GPIOs (A-H), RCC, DMA.
*   **APB1 Bus (Low Speed - 42/45MHz):** TIM2-5, USART2, I2C1-3, SPI2-3, DAC.
*   **APB2 Bus (High Speed - 84/90MHz):** TIM1, TIM8, ADC1-3, SPI1, USART1.

**Key Base Addresses:**
```c
#define PERIPH_BASE       0x40000000UL
#define AHB1PERIPH_BASE   (PERIPH_BASE + 0x00020000UL)
#define APB1PERIPH_BASE   (PERIPH_BASE + 0x00000000UL)

// GPIOs
#define GPIOA_BASE        (AHB1PERIPH_BASE + 0x00000000UL)
#define GPIOB_BASE        (AHB1PERIPH_BASE + 0x00000400UL)
#define GPIOC_BASE        (AHB1PERIPH_BASE + 0x00000800UL)

// RCC (Reset & Clock Control)
#define RCC_BASE          (AHB1PERIPH_BASE + 0x00003800UL)
```

## 2. RCC (Reset and Clock Control)
**Crucial Rule:** The microcontroller powers down all peripherals by default to save energy. **You must enable the clock for a peripheral before using it.**

*   **Register:** `RCC->AHB1ENR` (for GPIOs).
*   **Register:** `RCC->APB1ENR` (for UART2, I2C1, TIM2).

**Bits for AHB1ENR:**
*   Bit 0: **GPIOAEN**
*   Bit 1: **GPIOBEN**
*   Bit 2: **GPIOCEN**

**Example: Enable GPIOA and GPIOC Clocks**
```c
// Method 1: Pointer Casting (Raw Address)
#define RCC_AHB1ENR (*((volatile uint32_t *)(0x40023830)))
RCC_AHB1ENR |= (1 << 0); // Enable GPIOA
RCC_AHB1ENR |= (1 << 2); // Enable GPIOC

// Method 2: Struct Access (Standard CMSIS)
RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Defined in stm32f446xx.h
RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
```

## 3. GPIO Configuration Registers
Each GPIO port has several 32-bit registers. `x` is the pin number (0-15).
**General Setup Step:** Clear the bits first (`&= ~`) then set the new value (`|=`).

### MODER (Mode Register)
Determines the pin direction. 2 bits per pin.
*   `00`: **Input** (Reset State) - Use for Buttons, Sensors.
*   `01`: **Output** - Use for LEDs, Relays.
*   `10`: **Alternate Function** - Use for UART, SPI, I2C, PWM.
*   `11`: **Analog** - Use for ADC.

**Example: Set PA5 (LED) as Output**
```c
// 1. Clear bits 10 and 11 (2 bits for Pin 5)
GPIOA->MODER &= ~(3U << (5 * 2)); 
// 2. Set bit 10 to 1 (01 binary = Output)
GPIOA->MODER |=  (1U << (5 * 2)); 
```

### OTYPER (Output Type)
1 bit per pin. Only matters if MODER is Output or AF.
*   `0`: **Push-Pull** (Default) - Actively drives high (3.3V) and low (0V). Good for LEDs.
*   `1`: **Open-Drain** - Actively drives low (0V) but "floats" for high. Requires a Pull-Up resistor. **Essential for I2C (SDA/SCL).**

### OSPEEDR (Output Speed)
Controls the rise/fall time of the signal. Higher speed = more noise/power.
*   `00`: Low (2 MHz)
*   `01`: Medium (25 MHz)
*   `10`: Fast (50 MHz)
*   `11`: High (100 MHz) - Use for SPI/fast communications.

### PUPDR (Pull-up/Pull-down)
Defines state when no signal is driven (floating).
*   `00`: **No Pull** (Floating) - Voltage is undefined if not connected.
*   `01`: **Pull-Up** - Weakly connected to 3.3V. Default for Buttons connected to GND (Active Low).
*   `10`: **Pull-Down** - Weakly connected to GND. Default for Buttons connected to 3.3V (Active High).
*   `11`: Reserved

**Example: Set PC13 (Button) as Input with Pull-up**
```c
// Clear Mode bits for Pin 13 (Input = 00)
GPIOC->MODER &= ~(3U << (13 * 2)); 

// Clear PUPDR bits for Pin 13
GPIOC->PUPDR &= ~(3U << (13 * 2));
// Set Pull-Up (01) - because button connects to GND when pressed
GPIOC->PUPDR |= (1U << (13 * 2));
```

### AFR (Alternate Function Register)
**Crucial for UART/SPI/PWM.**
Mapping a generic GPIO pin to a specific internal peripheral.
*   **AFRL (Low):** Pins 0-7.
*   **AFRH (High):** Pins 8-15.
*   4 bits per pin. You need to look up the "AF Mapping" in the datasheet (e.g., AF5 for SPI1, AF7 for USART2).

**Example: Set PA2 as USART2_TX (AF7)**
```c
// 1. Set Mode to AF (10)
GPIOA->MODER |= (2U << (2 * 2));

// 2. Configure AFR Low register for Pin 2
GPIOA->AFR[0] &= ~(0xF << (2 * 4)); // Clear 4 bits
GPIOA->AFR[0] |=  (0x7 << (2 * 4)); // Set AF7 (0111)
```

## 4. GPIO Data Registers

### ODR (Output Data Register)
Write to this to set pin state.
```c
GPIOA->ODR |= (1 << 5);  // Set PA5 High (LED ON)
GPIOA->ODR &= ~(1 << 5); // Set PA5 Low (LED OFF)
```

### IDR (Input Data Register)
Read from this to check pin state. **Read-Only.**
```c
if (GPIOC->IDR & (1 << 13)) {
    // Pin is HIGH (1)
} else {
    // Pin is LOW (0)
}
```

### BSRR (Bit Set/Reset Register)
Atomic setting/resetting. Safer than `ODR |= ...` because it prevents Read-Modify-Write errors in interrupts.
*   **Lower 16 bits:** Set pin (1 = High).
*   **Upper 16 bits:** Reset pin (1 = Low).

```c
GPIOA->BSRR = (1 << 5);      // Set PA5 High
GPIOA->BSRR = (1 << (5+16)); // Reset PA5 Low
```

## 5. Bitwise Operations Cheat Sheet
*   **Set Bit k:** `REG |= (1 << k);`
*   **Clear Bit k:** `REG &= ~(1 << k);`
*   **Toggle Bit k:** `REG ^= (1 << k);`
*   **Check Bit k:** `if (REG & (1 << k))`

## 6. Visualizing Bitwise Operations
Students often get confused by `|=` and `&= ~`. Here is what happens visually.

### Setting a Bit (OR `|`)
**Goal:** Force the 5th bit to `1` without touching others.
```text
  0010 1000  (Current Register Value)
| 0010 0000  (Mask: 1 << 5)
-----------
  0010 1000  (Result: 5th bit is SET, others unchanged)
```

### Clearing a Bit (AND NOT `& ~`)
**Goal:** Force the 5th bit to `0` without touching others.
1.  **Create Mask:** `(1 << 5)` -> `0010 0000`
2.  **Invert Mask (`~`):** `1101 1111` (Hole punch at bit 5)
3.  **AND (`&`):**
```text
  0011 1000  (Current Register Value)
& 1101 1111  (Inverted Mask)
-----------
  0001 1000  (Result: 5th bit forced to 0, others kept)
```