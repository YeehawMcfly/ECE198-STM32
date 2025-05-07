# STM32 ECG Signal Transmission Prototype

This repository contains firmware for STM32 microcontrollers to demonstrate secure transmission of simulated ECG (heart signal) data, developed as a class project for ECE 198: Project Studio. The system uses a potentiometer to simulate heart signals, encrypts them with the Tiny Encryption Algorithm (TEA), transmits them via UART, decrypts them, and displays the results on an SSD1306 OLED display. It includes multiple implementations (`Receiver`, `Sender`, `Old-Receiver`, `Old-Sender`) and test projects (`Testings`) for STM32F401RE and STM32F446RE microcontrollers, designed as a proof-of-concept for healthcare data security.

## Project Overview
- **Purpose**: Prototype a secure ECG signal transmission system for educational purposes, simulating heart monitoring with encryption to ensure data privacy.
- **Hardware**: STM32F446RE (transmitter), STM32F401RE (receiver), potentiometer (signal simulator), SSD1306 OLED display, UART wiring.
- **Key Features**:
  - Simulates ECG signals (0–3.3V) using a potentiometer.
  - Encrypts and decrypts signals with TEA (32 cycles, 128-bit key).
  - Transmits encrypted data via UART (up to 10.5 Mbit/s).
  - Displays decrypted signals on an OLED via I2C (100–400 kHz).
  - Demonstrates compliance with health data privacy concepts (e.g., PHIPA-inspired).

## Repository Contents
The repository includes firmware projects for STM32 microcontrollers, organized into directories:

### Receiver
- **Path**: `Receiver/Receiving Data`
- **Purpose**: Receives, decrypts, and displays simulated ECG signals.
- **Key Files**:
  - `Core/Src/main.c`: Handles UART reception, TEA decryption, and OLED display output. Initializes peripherals (UART, I2C, GPIO) and processes encrypted data into readable signals.
  - `Core/Src/ssd1306.c`: Implements functions for SSD1306 OLED control (e.g., initialization, text/graphical rendering).
  - `Core/Src/fonts.c`: Provides font data for OLED text display.
  - `Core/Src/stm32f4xx_hal_msp.c`: Configures HAL Module Support Package (MSP) for peripheral initialization (e.g., UART, I2C).
  - `Core/Src/stm32f4xx_it.c`: Defines interrupt service routines for UART and I2C events.
  - `Core/Src/syscalls.c`: Implements system calls (e.g., for debugging output).
  - `Core/Src/sysmem.c`: Manages memory allocation for the system.
  - `Core/Src/system_stm32f4xx.c`: Initializes system clock and peripherals.
  - `Core/Inc/main.h`: Defines constants and macros for the application.
  - `Core/Inc/ssd1306.h`: Declares SSD1306 OLED functions.
  - `Core/Inc/stm32f4xx_hal_conf.h`: Configures HAL settings (e.g., peripheral enables).
  - `Core/Inc/stm32f4xx_it.h`: Declares interrupt handlers.
  - `STM32F401RETX_FLASH.ld`, `STM32F401RETX_RAM.ld`: Linker scripts for STM32F401RETX memory layout (flash and RAM).
- **Additional Project**: `Receiver/Sending Signals` contains a sender implementation (see `Sender` below).

### Sender
- **Path**: `Sender/Sending Signals`
- **Purpose**: Simulates ECG signals, encrypts, and transmits them.
- **Key Files**:
  - `Core/Src/main.c`: Reads analog signals via 12-bit ADC, encrypts with TEA, transmits via UART, and displays status on OLED. Configures ADC for potentiometer input and UART for half-duplex communication.
  - Other files: Similar to `Receiver`, including HAL, interrupt, OLED, and system initialization code.

### Old-Receiver
- **Path**: `Old-Receiver/Receiving Data`
- **Purpose**: Early prototype of the receiver with basic functionality.
- **Key Files**:
  - `Core/Src/main.c`: Initializes peripherals, performs TEA decryption, and supports ADC conversion for initial testing.

### Old-Sender
- **Path**: `Old-Sender/Sending Signals`
- **Purpose**: Early prototype of the sender with basic signal processing.
- **Key Files**:
  - `Core/Src/main.c`: Handles ADC conversion, TEA encryption, and OLED display output for early experiments.

### Testings
- **Path**: `Testings`
- **Purpose**: Modular tests for system components and combined functionality.
- **Subdirectories**:
  - `Combined_Test/Core/Src/main.c`: Tests end-to-end functionality (signal sampling, encryption, transmission, decryption, display).
  - `Input_Test/Core/Src/main.c`: Tests potentiometer input and ADC accuracy.
  - `OLED_Test_F401/Core/Src/main.c`: Tests SSD1306 OLED on STM32F401.
  - `OLED_Test/Core/Src/main.c`: General OLED display tests.
  - `Testing Signals/Core/Src/main.c`: Tests signal encryption and UART transmission.

**Note**: The repository contains source code and linker scripts only. Build artifacts (e.g., `Debug` folders) and ECG data are not included.

## Prerequisites
- **Hardware**:
  - STM32F446RE microcontroller (transmitter).
  - STM32F401RE microcontroller (receiver).
  - 10kΩ potentiometer (for ECG signal simulation).
  - SSD1306 OLED display (I2C interface).
  - Wiring and connectors (UART, I2C, power).
  - USB power source (5V, e.g., power bank or laptop).
- **Software**:
  - STM32CubeIDE (or equivalent IDE for STM32 development).
  - STM32CubeMX (optional, for project configuration).
  - STMicroelectronics STM32CubeF4 HAL drivers.
- **Development Environment**:
  - C compiler (e.g., GCC for ARM).
  - USB-to-serial debugger (e.g., ST-Link).

## Setup
1. **Clone the Repository**:
   ```bash
   git clone https://github.com/your-username/STM32-ECG-Prototype.git
   cd STM32-ECG-Prototype
   ```

2. **Hardware Setup**:
   - **Transmitter (STM32F446RE)**:
     - Connect potentiometer to PA0 (ADC Channel 0).
     - Connect UART TX (PA9) to receiver’s RX.
     - Connect OLED display via I2C (SCL: PB8, SDA: PB9).
   - **Receiver (STM32F401RE)**:
     - Connect UART RX (PA9) to transmitter’s TX.
     - Connect OLED display via I2C (SCL: PB8, SDA: PB9).
   - **Power**: Use USB (5V) for both boards or a 5V DC adapter.
   - Add 4.7kΩ pull-up resistors for I2C lines if needed.

3. **Software Setup**:
   - Install [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html).
   - Import projects (e.g., `Receiver/Receiving Data`) into STM32CubeIDE.
   - Install [STM32CubeF4 HAL drivers](https://www.st.com/en/embedded-software/stm32cubef4.html).
   - Configure build settings to use the provided linker scripts (`STM32F401RETX_FLASH.ld` or `STM32F401RETX_RAM.ld`).

## Usage
### Building and Flashing
1. Open a project in STM32CubeIDE (e.g., `Receiver/Receiving Data`).
2. Build the project (`Project > Build Project`).
3. Flash the firmware to the respective microcontroller using ST-Link (`Run > Debug`).
4. Repeat for `Sender` and `Testings` projects as needed.

### Running the System
1. **Simulate ECG Signals**:
   - Adjust the potentiometer on the STM32F446RE to generate voltages (0–3.3V), mimicking ECG signals.
2. **Transmit Data**:
   - Press a push button (PC13) on the STM32F446RE to trigger ADC sampling, TEA encryption, and UART transmission.
3. **Receive and Display**:
   - The STM32F401RE receives encrypted data, decrypts it with TEA, and displays the recovered signal on the OLED.
4. **Monitor Output**:
   - Check OLED displays for real-time encrypted (sender) and decrypted (receiver) signal values.

### Testing
The `Testings` directory includes projects to validate components:
- **Combined_Test**: Verifies full system functionality (sampling, encryption, transmission, decryption, display).
- **Input_Test**: Tests ADC accuracy with potentiometer input.
- **OLED_Test_F401**, **OLED_Test**: Validates OLED display output.
- **Testing Signals**: Tests encryption and UART transmission.

Run these tests to ensure components work before running the full system.

## Technical Details
### System Design
- **Signal Source**: Potentiometer simulates ECG signals (0–3.3V, 12-bit ADC).
- **Encryption**: TEA algorithm (32 cycles, 128-bit key) secures data.
- **Transmission**: UART (half-duplex, up to 10.5 Mbit/s, ≥1m distance).
- **Display**: SSD1306 OLED via I2C (100–400 kHz).
- **Microcontrollers**: STM32F446RE (transmitter), STM32F401RE (receiver).

### Code Summary
- **Main Logic** (`main.c` files):
  - **Sender**: Samples analog input via ADC, encrypts with TEA, sends via UART, and shows status on OLED.
  - **Receiver**: Receives UART data, decrypts with TEA, and displays signals on OLED.
  - **Old-Receiver**, **Old-Sender**: Early prototypes with basic ADC, encryption, and display functions.
  - **Testings**: Tests for ADC, encryption, UART, OLED, and combined functionality.
- **OLED Support** (`ssd1306.c`, `fonts.c`): Initializes OLED and renders text/graphics.
- **HAL and System** (`stm32f4xx_*` files): Configures clocks, peripherals, interrupts, and memory.
- **Linker Scripts** (`*.ld`): Defines memory layout for STM32F401RETX.

### Power and Energy
- **Power Consumption**: Max 0.64W (4V, 160mA), within USB 2.0’s 2.5W limit.
- **Energy Storage**: Minimal (4.0 × 10⁻¹¹ J in capacitors), below 500 mJ limit.

### Testing Plan
- **Voltage Sampling**: Verifies ADC accuracy (±5% tolerance).
- **Encryption Integrity**: Ensures TEA encryption matches expected outputs.
- **Data Transmission**: Confirms UART sends correct encrypted data.
- **Decryption Accuracy**: Validates decrypted values match originals (±5%).
- **Display**: Checks OLED displays correct encrypted/decrypted values.

## Notes
- **Prototype Nature**: This is a class project using a potentiometer to simulate ECG signals. Real-world applications would use ECG sensors (e.g., SparkFun AD8232).
- **No ECG Data**: The system generates simulated signals; no dataset is included.
- **Environment**: Test in stable conditions (indoor, 20–25°C) to avoid noise.
- **Privacy**: TEA encryption demonstrates health data security concepts, inspired by PHIPA.
- **Debugging**: Use STM32CubeIDE’s debugger to monitor UART/I2C signals.

## Contributing
Contributions are welcome, but please ensure they align with ECE 198 class project guidelines. To contribute:
1. Fork the repository.
2. Create a feature branch (`git checkout -b feature/YourFeature`).
3. Commit changes (`git commit -m "Add YourFeature"`).
4. Push to the branch (`git push origin feature/YourFeature`).
5. Open a pull request and coordinate with the project team.

## Acknowledgments
- Developed for ECE 198: Project Studio by Vishnu Ajit and Eugene Zhao.
- Uses [STM32CubeF4 HAL](https://www.st.com/en/embedded-software/stm32cubef4.html) and SSD1306 drivers.
- Inspired by ECG monitoring and embedded systems tutorials.