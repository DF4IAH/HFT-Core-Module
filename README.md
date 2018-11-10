# HFT-Core-Module

## V1.1 PCB after placement was done:
![Screenshot of V1.1](https://raw.githubusercontent.com/DF4IAH/HFT-Core-Module/master/Docs/09_Results/Pictures/HFT-Core-Module_1V1_PCB_PlacementDone.png)

## These are the included devices for the V1.1:
* __CPU: ST Microdevices STM32L496ZGT6P__ (1,024kB Flash / 320kB RAM)
* __Flash and EEPROM__ (more data to be kept outside of the MCU)
* __GSM / GPS / BT: SIM868__ (SMS / GPRS / ...)
* __VHF/UHF FSK and LoRa: SX1262__ (VHF TX/RX)
* __VHF/UHF All but Lora: AX5243__ (VHF TX/RX)
* __Accel/Gyro/Mag: BNO085__ (3Axis-Acceleration, 3Axis-Gyro, 3Axis-Magneto)
* __Baro/Hygro/Temp/AirQuality: BME680__ (0.12Pa equiv. 1.7cm)
* __VCTCXO: CFPT-141_20MHz__ (being synchronized to the GPS 1PPS signal by pulling the VC)
* __Clock PLL__ (distributing any clocks that are needed)
* __Audio ADC and DAC__ (connected to the I2S facilities)
* __LCD 16x2 display__ (draws less than 1mA from the battery)
* __E-Ink-Display w/ 200x200px__ (more fun with even less energy)
* __Archer Headers__ (expanded by up to two daughter boards)
* __SMA sockets: GSM, GNSS, BlueTooth, SX1262, AX5243__ (each port its own antenna when needed)
* __PMICs for 12V, USB, Solar-Cells, 3.3V, 1.2V__ (high efficiency on board)
* __Electronic Switches: Subcircuits to be powered on/off by request__ (ultra-low-power to full-media)
* __LiPo-Connector: 1-cell 3.7V__ (high power request by the SIM868 device to be satisfied)
