# ShieldTuner - Digital Tuner

Welcome to the ShieldTuner project! This repository contains the design and implementation details of our digital tuning device. The ShieldTuner is a shield-type device designed to work with the STM32L452RE development board. It captures acoustic signals using a microphone, processes the sampled signal to analyze its frequency spectrum, and determines its fundamental frequency. The frequency is then compared to a reference frequency, and LEDs indicate whether the detected frequency is below, above, or within the threshold of the reference frequency. Additionally, a button allows users to change the reference frequency.

## Project Structure

### Firmware
The firmware for the ShieldTuner is developed using STM32CubeIDE. It includes all the C code required to sample the acoustic signals, perform the FFT analysis, and control the LEDs and button. You can find the firmware details in the `ELO301_FFT` directory.

### Hardware
The hardware section includes the PCB design files created with Fusion 360 from Autodesk. It contains the PCB project, component list, and component libraries. All hardware-related information can be found in the `PCB_DESIGN` directory.

## Features
- **Acoustic Signal Capture:** Utilizes a high-quality microphone to capture audio signals.
- **Frequency Analysis:** Processes the sampled signals to extract the fundamental frequency.
- **Visual Indication:** Two LEDs indicate if the frequency is too low, too high, or within the acceptable range.
- **Reference Frequency Adjustment:** A button allows users to adjust the reference frequency for tuning.

Thank you for visiting our project. We hope you find ShieldTuner useful and innovative! Feel free to explore the repository and contribute.