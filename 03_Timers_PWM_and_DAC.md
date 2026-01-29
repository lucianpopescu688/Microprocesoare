# Timers, PWM & DAC

## 0. Theory: The Bucket Analogy (Timers)
Imagine a water bucket filling up with drops of water.
*   **System Clock:** The faucet dripping water.
*   **Prescaler (PSC):** A funnel that discards some drops. A high Prescaler means only every 8000th drop enters the bucket. This **slows down** the filling speed.
*   **Counter (CNT):** The current water level in the bucket.
*   **Period (ARR):** The size of the bucket. When it's full (Overflow), it dumps the water (Update Event/Interrupt) and starts over from 0.

## 1. Timers (General Purpose)
Used for measuring time, generating delays without blocking CPU, or triggering events periodically.

### Key Concepts
*   **CNT (Counter):** The value counting up (0, 1, 2...).
*   **PSC (Prescaler):** Divides the system clock to slow down the counter.
*   **ARR (Auto-Reload Register):** The target value. When CNT == ARR, the timer resets to 0 and an "Update Event" (Interrupt) occurs.

### Calculation Formula (Very Important for Exam)
`Update Frequency = Timer Clock / ((PSC + 1) * (ARR + 1))`

**Example: Create a 1 Hz (1 second) interrupt.**
*   **Timer Clock:** 84 MHz (Default APB1 timer clock for 84MHz sysclk).
*   **Step 1:** Choose a large PSC to make the math easy. Let's make the counter tick at 10 kHz (10,000 ticks/sec).
    *   `10,000 = 84,000,000 / (PSC + 1)`
    *   `PSC + 1 = 8400`
    *   **PSC = 8399**
*   **Step 2:** How many ticks for 1 second? 10,000.
    *   **ARR = 9999** (Counts 0 to 9999 is 10,000 steps).

### Code Implementation
```c
// Start Timer in Interrupt Mode
HAL_TIM_Base_Start_IT(&htim6);

// Handle the Interrupt in main.c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        // Runs every 1 second
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    }
}
```

## 2. PWM (Pulse Width Modulation)
Controls power to LEDs (dimming) or Servos (position) by switching a pin High/Low very fast.

### Visualizing PWM
A square wave signal.
*   **Period (ARR):** The total width of one cycle.
*   **Pulse (CCR):** The width of the "High" part.

```text
      <------- Period (ARR) ------->
      
      +-------+                    +-------+
      |       |                    |       |
3.3V  |  ON   |        OFF         |  ON   |
      | (CCR) |                    |       |
0V    +       +--------------------+       +----
      ^       ^                    ^
      0      CNT=CCR              CNT=ARR (Reset)
```
*   **Duty Cycle 50%:** ON for half the time (CCR = ARR / 2).
*   **Duty Cycle 10%:** ON for a short blip (CCR = ARR / 10).

### Configuration
*   **Frequency:** Determined by PSC and ARR (same formula as above).
*   **Duty Cycle:** Percentage of time the signal is HIGH.
    *   `Duty % = (CCR / (ARR + 1)) * 100`
    *   **CCR (Capture Compare Register):** The value where the pin flips from High to Low.

**Example: Servo Motor (50 Hz)**
*   Target Freq: 50 Hz (20ms period).
*   Clock: 84 MHz.
*   `PSC = 83` => Timer Clock = 1 MHz (1us per tick).
*   `ARR = 19999` => 20,000 ticks = 20ms = 50 Hz.
*   **Servo Control:**
    *   1ms Pulse (-90 deg) = CCR 1000
    *   1.5ms Pulse (0 deg) = CCR 1500
    *   2ms Pulse (+90 deg) = CCR 2000

### Start & Control Code
**Crucial:** PWM does not start automatically.
```c
// 1. Start PWM on Channel 1
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

// 2. Change Duty Cycle (Brightness/Position)
__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 500); // Set CCR1 to 500
```

## 3. DAC (Digital to Analog Converter)
Outputs a true analog voltage (0V to 3.3V).
STM32F446RE has 2 DAC channels (PA4, PA5).

### Specs
*   **Resolution:** 12-bit (Values 0 to 4095).
    *   0 = 0V
    *   4095 = 3.3V
    *   2048 = ~1.65V

### Initialization & Usage
```c
// Enable DAC Channel 1
HAL_DAC_Start(&hdac, DAC_CHANNEL_1);

// Output specific voltage
uint32_t val = 2048; // 1.65V
HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, val);
```

### Generating Waveforms (Sine Wave)
You cannot just loop `HAL_DAC_SetValue` in `while(1)` efficiently. Use a **Timer Interrupt** to update the DAC at a precise sampling rate.

```c
// Global Array
uint16_t sin_wave[20] = {2048, 2690, 3251, ... }; // 20 points
uint8_t idx = 0;

// Timer Interrupt (e.g., 20 kHz freq for 1 kHz sine wave)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, sin_wave[idx]);
        idx++;
        if (idx >= 20) idx = 0;
    }
}
```