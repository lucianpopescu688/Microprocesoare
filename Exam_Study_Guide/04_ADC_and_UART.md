# ADC & UART

## 1. ADC (Analog to Digital Converter)
Reads voltage (sensors, potentiometers). The microcontroller cannot see "2.5 Volts". It only sees numbers (integers).

### Visualizing Resolution (12-bit)
Imagine a staircase with **4096 steps** (2^12).
*   **Bottom Step (0):** 0.0 Volts
*   **Top Step (4095):** 3.3 Volts
*   **Step Size:** 3.3V / 4095 ≈ 0.8 millivolts.

If you measure **1.65V**, you land exactly on step **2048**.

### Sampling Time
The ADC needs time to charge its internal capacitor to match the pin voltage.
*   **Higher Impedance Sources** (e.g., Temperature Sensor) need **Longer Sampling Time** (e.g., 480 cycles).
*   **Low Impedance** (Potentiometer) can use **Shorter Time** (e.g., 3 cycles).
*   *Configured in CubeMX: `Rank -> Sampling Time`*.

### Polling Mode (Single Read)
Best for reading a value once in a while (e.g., every 100ms).
```c
// 1. Start Conversion
HAL_ADC_Start(&hadc1);

// 2. Wait for it to finish (Blocking!)
if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
    // 3. Read Result
    uint32_t rawValue = HAL_ADC_GetValue(&hadc1);
}
```

### Multi-Channel / DMA Mode
If you need to read multiple pins (e.g., Channel 0 and Channel 1) efficiently.
1.  Enable **Scan Conversion Mode**.
2.  Enable **DMA Continuous Requests**.
3.  Set **Number of Conversions** to 2.

```c
uint32_t adcBuffer[2]; // Stores [Ch0, Ch1]

// Start ADC with DMA (Non-blocking)
// &hadc1: ADC Handle
// adcBuffer: Array to store results
// 2: Number of data points to transfer
HAL_ADC_Start_DMA(&hadc1, adcBuffer, 2);

// Access values anytime in main loop
uint32_t sensor1 = adcBuffer[0];
uint32_t sensor2 = adcBuffer[1];
```

## 2. UART (Universal Asynchronous Receiver-Transmitter)
Serial communication. Used for "printf" debugging or talking to Bluetooth/GPS modules.

### The Data Frame (Visualized)
How a single letter 'A' (ASCII 65 = `01000001`) travels down the wire.
Idle line is High (3.3V).

```text
      Idle    Start   D0  D1  D2  D3  D4  D5  D6  D7   Stop   Idle
Line: ----+   +------------------------------------+   +---------
          |   |                                    |   |
          +---+   1   0   0   0   0   0   1   0    +---+
```
*   **Start Bit:** Pulls line LOW to wake up the receiver.
*   **Data:** 8 bits of actual information.
*   **Stop Bit:** Pulls line HIGH to ensure the line is ready for the next byte.

### Configuration
*   **Baud Rate:** Speed (e.g., 9600, 115200). Must match the receiver (Tera Term).
*   **Word Length:** 8 Bits.
*   **Parity:** None.
*   **Stop Bits:** 1.

### Transmitting Data (Printf style)
```c
char msg[50];
float temperature = 24.5;

// Format the string
sprintf(msg, "Temp: %.2f C\r\n", temperature);

// Send (Blocking mode - waits until sent)
HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
```

### Receiving Data (Interrupt Mode - Best Practice)
Polling `HAL_UART_Receive` freezes your code. Use Interrupts.

**Step 1: Start Listening**
Call this ONCE in `main()` before `while(1)`.
```c
uint8_t rxByte;
HAL_UART_Receive_IT(&huart2, &rxByte, 1);
```

**Step 2: The Callback**
This runs automatically when a byte arrives.
```c
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        // Example: Toggle LED if '1' is received
        if (rxByte == '1') {
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        }
        
        // CRITICAL: Re-arm the interrupt for the next byte!
        HAL_UART_Receive_IT(&huart2, &rxByte, 1);
    }
}
```

### Debugging with UART
If you see garbage characters:
1.  Check Baud Rate mismatch (Code vs Tera Term).
2.  Check Clock settings (if system clock is wrong, baud rate generation is wrong).
3.  Ground loops (ensure GND is connected if using external USB-UART).

```