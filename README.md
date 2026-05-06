# STM32 + Raspberry Pi Audio Event Recognition

## Project Overview

This project aims to build a simple real-time audio event recognition system using an STM32 board and a Raspberry Pi.

The STM32 is responsible for audio acquisition and feature extraction, while the Raspberry Pi receives the extracted features and performs audio event classification.

## System Workflow

```text
STM32 Microphone
      ↓
DMA + PCM Buffer
      ↓
CMSIS-DSP Feature Extraction
      ↓
WiFi Communication
      ↓
Raspberry Pi Event Recognition
```

## Main Components

- **STM32**
  - Captures audio using the onboard digital microphone
  - Uses DMA and `PCM_Buffer` for continuous audio sampling
  - Extracts audio features using CMSIS-DSP

- **Raspberry Pi**
  - Receives feature vectors from STM32 through WiFi
  - Classifies simple audio events such as:
    - Clap
    - Knock
    - Background noise

## Expected Result

The final system is expected to demonstrate a prototype of distributed audio event recognition, where STM32 handles audio sensing and preprocessing, and Raspberry Pi performs event classification.

## References

- STM32CubeIDE & CMSIS-RTOS documentation  
  https://www.st.com/resource/en/user_manual/um1722-developing-applications-on-stm32cube-with-rtos-stmicroelectronics.pdf

- CMSIS-DSP software library documentation  
  https://arm-software.github.io/CMSIS-DSP/main/
