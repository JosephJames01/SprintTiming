# Sprint Timing System with Wireless Clock Sync

This project is a high-precision, Arduino-based sprint timing system. It uses two Arduino modules (start and finish), wireless synchronization via RFM69HCW radios, a laser break detector, and an LCD display to accurately measure and display sprint times. Clock synchronization is achieved using a PI controller, inspired by IEEE 1588 PTP.

---

## Features

- **Accurate Sprint Timing:** Laser break detection for precise start/finish measurement.
- **Wireless Synchronization:** Two RFM69HCW modules keep both ends in sync.
- **PI Clock Control:** Precise clock alignment using a PI controller.
- **LCD Display:** 16x2 LCD (4-bit mode) shows timing, sync error, and system status.
- **Audio Feedback:** Adafruit Class D amplifier and 8Ω speaker provide start/set sounds.
- **Debounced Button Input:** Hardware debounce with capacitors for reliable manual input.
- **Portable:** Battery-powered and housed in 3D-printed or standard electrical enclosures.

---

## Hardware List

| Component                 | Quantity | Purpose                                      |
|---------------------------|----------|----------------------------------------------|
| Arduino Uno/Nano          | 2        | Main controllers (start & finish modules)    |
| RFM69HCW 868MHz           | 2        | Wireless communication                       |
| 16x2 LCD (4-bit mode)     | 1        | Display timing and sync info                 |
| TEPT4400 Phototransistor  | 1        | Laser break detection                        |
| Laser Module              | 1        | Beam for break detection                     |
| Adafruit Class D Amp      | 1        | Drives the speaker                           |
| 8Ω Speaker                | 1        | Audio output                                 |
| Tactile Button            | 1        | Manual race start                            |
| Capacitors                | 1 or 2   | Button debounce                              |
| 3D-printed/Electrical Box | 2        | Enclosures                                   |
| Battery Pack (7-12V)      | 3        | Portable power                               |
| Wiring + miscellaneous    | as requ  | Connection                                   |
| camera stands and         |          | position                                     |
| connectors for stands     |          |

---

## How It Works

- **Start Module:**  
  - Waits for a button press, plays a "set" sound, waits a random interval, then plays a "start" sound.
  - Sends a wireless event packet with a precise timestamp.
  - Periodically sends sync packets to keep the remote clock aligned.

- **Finish Module:**  
  - Receives sync packets and applies PI control to align its clock to the master.
  - Receives event packets and records the event timestamp.
  - Monitors the laser beam; when broken, calculates and displays the reaction/sprint time.
  - Shows sync error and timing on the LCD.

---

## Wiring Overview

- **RFM69HCW:** Connects to SPI pins and dedicated CS/INT/RST pins on Arduino.
- **LCD:** Wired in 4-bit mode to digital pins.
- **Laser Module:** Powered from Arduino, aimed at TEPT4400.
- **TEPT4400:** Connected to an analog input, with a pull-down resistor.
- **Button:** Connected to a digital input with 0.1μF capacitor for debounce.
- **Amp/Speaker:** Amp shutdown and audio pins controlled by Arduino.

---

## Software Setup

1. **Clone this repository:**
