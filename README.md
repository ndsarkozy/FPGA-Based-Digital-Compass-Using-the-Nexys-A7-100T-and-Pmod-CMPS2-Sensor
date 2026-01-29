# FPGA-Based Digital Compass

A real-time digital compass implemented on the Nexys A7-100T FPGA board using the Pmod CMPS2 magnetometer sensor. The system calculates and displays heading angles relative to magnetic north on the onboard seven-segment display.

![Running Board](docs/images/board_north.jpg)

## Overview

This project demonstrates the integration of digital logic design principles to create a hardware-based embedded navigation system. The FPGA acquires magnetic field data from the sensor, processes it through custom logic to compute the real-time heading angle, and displays the result on the seven-segment display.

### Key Features

- **Real-time heading calculation** with 22.5° resolution (16 compass directions)
- **I²C communication** with the Pmod CMPS2 magnetometer
- **SPI communication** with the onboard ADXL362 accelerometer
- **Seven-segment display** output showing heading in degrees (0-359°)
- **Debug LED indicators** for system status monitoring
- **Configurable calibration offset** for true North alignment
- **Rate-limited updates** (~640ms) for stable display output

## Hardware Requirements

| Component | Description |
|-----------|-------------|
| **Nexys A7-100T** | Digilent FPGA development board (Artix-7 XC7A100T) |
| **Pmod CMPS2** | 3-axis magnetometer module (LSM303DLHC) |
| **USB Cable** | For programming and power |

### Pin Connections

The Pmod CMPS2 connects to the **JA header** on the Nexys A7 board:

| Signal | JA Pin | FPGA Pin |
|--------|--------|----------|
| SCL    | JA1    | C17      |
| SDA    | JA2    | D18      |
| VCC    | JA6    | 3.3V     |
| GND    | JA5    | GND      |

## System Architecture

```
┌─────────────────┐     ┌─────────────────┐
│  Accelerometer  │     │       I²C       │
│     (SPI)       │     │    Interface    │
└────────┬────────┘     └────────┬────────┘
         │                       │
         │              ┌────────▼────────┐
         │              │  Magnetometer   │
         │              │    Driver       │
         │              └────────┬────────┘
         │                       │
         └───────────┬───────────┘
                     │
            ┌────────▼────────┐
            │      Tilt       │
            │  Compensation   │
            └────────┬────────┘
                     │
            ┌────────▼────────┐
            │    Heading      │
            │  Calculation    │
            └────────┬────────┘
                     │
         ┌───────────┴───────────┐
         │                       │
┌────────▼────────┐     ┌────────▼────────┐
│  Binary to BCD  │     │  BCD to 7-Seg   │
└────────┬────────┘     └─────────────────┘
         │
┌────────▼────────┐
│   Top Module    │
│  (Integration)  │
└─────────────────┘
```

## Module Descriptions

### Core Modules

| Module | Description |
|--------|-------------|
| `top.v` | Top-level integration connecting all subsystems |
| `heading_calculation.v` | Computes 16-direction heading using quadrant-aware atan2 approximation |
| `tilt_compensation.v` | Sensor driver wrapper providing magnetometer data interface |
| `magnetometer_driver.v` | High-level I²C controller for Pmod CMPS2 |
| `i2c_master.v` | Custom I²C master with START/STOP/RESTART conditions |
| `SPI_master.v` | SPI interface for onboard accelerometer |

### Display Modules

| Module | Description |
|--------|-------------|
| `binary_to_bcd.v` | Converts 10-bit binary (0-511) to 3-digit BCD |
| `bcd_to_7seg.v` | Maps BCD digits to 7-segment display patterns |

### Testbenches

Each module includes a comprehensive testbench for verification:

- `i2c_master_tb.v` - Behavioral I²C slave model with ACK/NACK testing
- `SPI_master_tb.v` - Simulated accelerometer responses
- `magnetometer_driver_tb.v` - Full I²C communication verification
- `heading_calculation_tb.v` - All 16 compass direction tests
- `tilt_compensation_tb.v` - Data conversion and pass-through verification
- `top_tb.v` - Full system integration testing

## Building and Programming

### Prerequisites

- Xilinx Vivado 2020.1 or later
- Nexys A7-100T board drivers

### Steps

1. **Clone the repository**
   ```bash
   git clone https://github.com/yourusername/fpga-digital-compass.git
   cd fpga-digital-compass
   ```

2. **Open Vivado and create a new project**
   - Select Nexys A7-100T as the target board
   - Add all source files from the `src/` directory
   - Add the constraints file from `constraints/`

3. **Run Synthesis and Implementation**
   ```
   Run Synthesis → Run Implementation → Generate Bitstream
   ```

4. **Program the FPGA**
   - Connect the Nexys A7 via USB
   - Open Hardware Manager
   - Program the device with the generated bitstream

5. **Connect the Pmod CMPS2**
   - Attach the magnetometer to the JA header
   - Power cycle the board if necessary

## Usage

Once programmed:

1. The seven-segment display shows the current heading in degrees (0-359°)
2. Debug LEDs indicate system status:
   - **LED[0]**: Heading valid
   - **LED[1]**: Reset status
   - **LED[2]**: I²C busy
   - **LED[3]**: I²C error
   - **LED[4]**: Magnetometer data present

3. Rotate the board horizontally to see the heading change
4. Press the reset button (BTNC) to reinitialize the system

### Cardinal Directions

| Direction | Degrees |
|-----------|---------|
| North     | 0°      |
| East      | 90°     |
| South     | 180°    |
| West      | 270°    |

## Project Structure

```
fpga-digital-compass/
├── src/
│   ├── top.v
│   ├── heading_calculation.v
│   ├── tilt_compensation.v
│   ├── magnetometer_driver.v
│   ├── i2c_master.v
│   ├── SPI_master.v
│   ├── binary_to_bcd.v
│   └── bcd_to_7seg.v
├── testbench/
│   ├── top_tb.v
│   ├── heading_calculation_tb.v
│   ├── tilt_compensation_tb.v
│   ├── magnetometer_driver_tb.v
│   ├── i2c_master_tb.v
│   └── SPI_master_tb.v
├── constraints/
│   └── nexys_a7.xdc
├── docs/
│   └── images/
└── README.md
```

## Simulation

To run the testbenches in Vivado:

1. Set the desired testbench as the top module for simulation
2. Run behavioral simulation
3. View waveforms and console output for pass/fail results

All testbenches are self-checking and report test statistics upon completion.

## Design Decisions

- **Tilt compensation disabled**: For improved stability, the tilt compensation module passes through raw magnetometer data without applying pitch/roll corrections. The accelerometer interface is retained for potential future enhancements.

- **Rate-limited updates**: Heading updates occur every 64 samples (~640ms) to prevent display flickering and provide stable readings.

- **Calibration offset**: A configurable offset parameter allows alignment of the computed heading with true North.

## Future Improvements

- [ ] Enable full tilt compensation using accelerometer data
- [ ] Implement hard/soft iron calibration routines
- [ ] Add LCD display support for cardinal direction names
- [ ] Integrate GPS module for true north correction
- [ ] Implement data logging via UART

## Authors

- **Christian Vanegas** - [Christian.Vanegas02@student.csulb.edu](mailto:Christian.Vanegas02@student.csulb.edu)
- **Nathan Sarkozy** - [Nathan.Sarkozy@student.csulb.edu](mailto:Nathan.Sarkozy@student.csulb.edu)
- **Kaiya Hayashida** - [Kaiya.Hayashida@student.csulb.edu](mailto:Kaiya.Hayashida@student.csulb.edu)

California State University, Long Beach  
College of Engineering  
CECS 201/301

## References

1. [ADXL362 Datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/adxl362.pdf) - Analog Devices
2. [Pmod CMPS2 Reference Manual](https://digilent.com/reference/pmod/pmodcmps2/reference-manual) - Digilent Inc.
3. [Nexys A7 Reference Manual](https://digilent.com/reference/programmable-logic/nexys-a7/reference-manual) - Digilent Inc.

## License

This project was developed as part of coursework at California State University, Long Beach.
