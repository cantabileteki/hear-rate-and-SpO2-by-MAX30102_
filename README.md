# hear-rate-and-SpO2-by-MAX30102

Communicate PSOC 6 MCU and MAX30102 PPG sensor using I2C protocol, calculate heart 
rate and blood oxygen saturation when pressing finger on the sensor, then pass these 
physiological data to the serial port of PSOC6 via UART, and also send them to an Android 
device via BLE functionality.


MAX30102.c works as the mbedded controller driver for the MAX30102, and MAX30102.h works as the
header file for MAX30102 driver. 
algorithm.c calculates the heart rate/SpO2 level, and
algorithm.h is its header file. 
main_cm4.c works as the main application for this program.

the algorithm mostly referenced https://github.com/aromring/MAX30102_by_RF/blob/master/algorithm_by_RF.cpp
