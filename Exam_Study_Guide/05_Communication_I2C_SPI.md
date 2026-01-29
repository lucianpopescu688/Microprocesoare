# Communication Protocols (I2C & SPI)

## Comparison: The Dinner Table Analogy
*   **I2C (The Conference Call):**
    *   **2 Wires (SDA, SCL).**
    *   Many people (Slaves) on the same phone line.
    *   The Boss (Master) has to say "Hey Bob" (Address) before giving instructions.
    *   **Pros:** Uses few pins. **Cons:** Slower, complex addressing.
*   **SPI (The Direct Line):**
    *   **4 Wires (MOSI, MISO, SCK, CS).**
    *   The Boss has a separate dedicated phone line (CS pin) for *each* employee.
    *   When the Boss picks up Phone #1 (CS Low), Employee #1 listens immediately. No addressing needed.
    *   **Pros:** Very fast (good for Screens/SD Cards). **Cons:** Uses many pins (1 CS per device).

## 1. I2C (Inter-Integrated Circuit)
Two-wire protocol, good for sensors (MPU6050, BH1750).
*   **SDA (Serial Data):** Bidirectional data.
*   **SCL (Serial Clock):** Clock signal (controlled by Master).
*   **Address:** Every device has a 7-bit address. (e.g., MPU6050 = 0xD0 or 0x68<<1).

### Key Concepts
*   **Start/Stop:** Master initiates and ends communication.
*   **ACK/NACK:** Receiver sends ACK (low) after every byte to say "I got it".
*   **Open Drain:** Requires Pull-up resistors (4.7k) on SDA and SCL lines.

### MPU6050 Example (Write)
Writing to a configuration register.
```c
#define MPU_ADDR (0x68 << 1) // 0xD0
#define PWR_MGMT_1 0x6B

// Wake up MPU6050 (Write 0 to Power Management Register)
uint8_t data = 0x00;
HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
```

### MPU6050 Example (Read)
Reading sensor data (Accelerometer).
```c
#define ACCEL_XOUT_H 0x3B
uint8_t buffer[6]; // X_H, X_L, Y_H, Y_L, Z_H, Z_L

// Read 6 bytes starting from 0x3B
HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, buffer, 6, 100);

// Combine Bytes (MSB << 8 | LSB)
int16_t accel_x = (int16_t)((buffer[0] << 8) | buffer[1]);
int16_t accel_y = (int16_t)((buffer[2] << 8) | buffer[3]);
```

---

## 2. SPI (Serial Peripheral Interface)
Four-wire, full-duplex, high speed. Used for LCDs (ST7735), SD Cards.
*   **MOSI:** Master Out Slave In.
*   **MISO:** Master In Slave Out.
*   **SCK:** Serial Clock.
*   **CS/SS:** Chip Select (Active Low).

### Modes (CPOL/CPHA)
*   **CPOL (Clock Polarity):** Is the clock Idle Low (`_|_|_`) or Idle High (`-|-|-'`)?
*   **CPHA (Clock Phase):** Do we read data on the *Leading* edge (first change) or *Trailing* edge (second change)?

```text
      Mode 0 (Standard for ST7735)
      
CS    High (Idle)
      ----------+                  +-----------
                | (Select)         | (Deselect)
                +------------------+

SCK   Low (Idle)
      +---+   +---+   +---+   +---+
      |   |   |   |   |   |   |   |
      +---+   +---+   +---+   +---+
      ^       ^
    Sample   Setup
```

### Controlling the Display (ST7735)
Unlike I2C, SPI devices often use extra "Data/Command" (DC) and "Reset" (RST) pins.

**The Protocol:**
1.  **Select:** Pull CS Low.
2.  **Mode:** Set DC Low for Command, High for Data.
3.  **Transmit:** Send bits via SPI.
4.  **Deselect:** Pull CS High.

**Code Example (Command vs Data):**
```c
// Send Command (e.g., "Sleep Out")
void ST7735_WriteCommand(uint8_t cmd) {
    HAL_GPIO_WritePin(DC_PORT, DC_PIN, GPIO_PIN_RESET); // Command Mode
    HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET); // Select
    HAL_SPI_Transmit(&hspi1, &cmd, 1, 10);
    HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);   // Deselect
}

// Send Data (e.g., Pixel Color)
void ST7735_WriteData(uint8_t data) {
    HAL_GPIO_WritePin(DC_PORT, DC_PIN, GPIO_PIN_SET);   // Data Mode
    HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET); // Select
    HAL_SPI_Transmit(&hspi1, &data, 1, 10);
    HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);   // Deselect
}
```

### Debugging Tips
*   **White Screen:** Usually Reset or Init sequence failed. Check wiring.
*   **Garbage Pixels:** SPI Baud rate too high? Wire loose?
*   **No Motion (I2C):** Forgot Pull-up resistors? Address wrong?