# STWIN_drivers
This repo contain all sensors documentation and drivers for STWIN board, based on STM32.
* Implemented sensors
    * STTS 751 (Temperature)
    * LPS22HH (Pressure)
    * HTS221 (Humidity and Temperature)
* TODO
    * SD card saving
    * ISM330DHCX (6-Axis IMU)
    * IIS2MDC (3D Magnetometer)

## Licence
* Source code for sensors drivers is provided by [STMicroelectronics](https://github.com/STMicroelectronics/STMems_Standard_C_drivers) under [BSD 3-Clause License](https://github.com/STMicroelectronics/STMems_Standard_C_drivers/blob/master/LICENSE).

* Init functions, global header file, [`example_main`](main.c) and advice for using STWIN sensors are provided by me, under [BSD 3-Clause License](LICENSE).

## Example main
An example main file is provided. It used `init_sensor` functions, and print sensors values on UART.
### UART
Displaying datas on UART 2
* 115 200 baud
* 8 bits data
* No parity
* 1 stop bit

### Timer
Displaying datas each 1 seconds with Timer 16 interrupt
* Proc frequency = 120 MHz
* Prescaler 4999
* Counter period : 23999

## Contact
For any bug report or feature request, please create an issue.
