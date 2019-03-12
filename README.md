# HFT-Core-Module

## V1.1 PCB after placement was done:
![Screenshot of V1.1](https://raw.githubusercontent.com/DF4IAH/HFT-Core-Module/master/Docs/09_Results/Pictures/HFT-Core-Module_1V1_PCB_PlacementDone.png)

## These are the included devices for the V1.1:
* __CPU: ST Microelectronics STM32L496ZGT6P__ (80MHz ARM Cortex-M4 w/ 1MB FLASH / 320kB RAM)
* __ext. Flash and EEPROM__ (8MB more data to be kept outside of the MCU / 8kB configuration data)
* __GSM / GNSS / BT: SIM868__ (SMS / GPRS / GPS, Glonass ...)
* __VHF/UHF FSK and LoRa: SX1262__ (VHF TX/RX)
* __VHF/UHF All but Lora: AX5243__ (VHF TX/RX)
* __Accel/Gyro/Mag: BNO085__ (3Axis-Acceleration, 3Axis-Gyro, 3Axis-Magneto - motion tracking engine)
* __Baro/Hygro/Temp/AirQuality: BME680__ (0.12Pa equiv. 1.7cm)
* __VCTCXO: CFPT-141_20MHz__ (being synchronized to the GPS 1PPS signal by pulling the VC)
* __Clock PLL__ (distributing any clocks that are needed)
* __Audio ADC and DAC__ (connected to the I2S facilities)
* __LCD 16x2 display__ (draws less than 1mA from the battery)
* __E-Ink-Display w/ 200x200px__ (more fun with even less energy)
* __Archer Headers__ (expanded by up to two daughter boards)
* __SMA sockets: GSM, GNSS, BlueTooth, SX1262, AX5243__ (each device has its own antenna socket)
* __PMICs for 12V, USB, Solar-Cells, 3.3V, 1.2V__ (high efficiency on board)
* __Electronic Switches: Subcircuits to be powered on/off by request__ (ultra-low-power to full-media)
* __LiPo-Connector: 1-cell 3.7V__ (high power request by the SIM868 device to be satisfied)
* __... and some more fun...__ (what is missing?  ;-)
