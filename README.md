# Mpu9250---Digital-Motion-Processor-Landscape-Orientation-
  This project is covering a CC1350 microcontroller system with a MPU-9250 motion tracking device. This project is mostly based on SparkFun's MPU9250 DMP library (https://github.com/sparkfun/SparkFun_MPU-9250-DMP_Arduino_Library).
 
 Chosen serial comm. protocol for the project is I2C. I2C communication is handled with the help of source files in the above-mentioned library and CC1350 HAL (Hardware Abstraction Layer) functions. This code was designed to run on an inhouse-designed board, so be cautious about the I2C slave address and I2C pins, should you use a different model of MPU sensors. Main purpose is to activate the Digital Motion Processor that is included with this model, and get specific outputs. 
 
 This project runs TI-RTOS and has 2 different tasks, one being an example included in the SimpleLink library. And the other task is based on the above-mentioned SparkFun library. CC1350 communicates with MPU9250 using I2C protocol, and gets a Landscape Orientation output and prints it using UART serial communication protocol or using "printf" function that could be only used when debugging. Most important files in this project are:
 -SensorTask (source and header files)
 -SensorMPU9250 (source and header files)
 -SensorMpu9250_DMP (source and header files)
 -SensorI2C (source and header files)
 -inv_mpu (source and header files)
 -inv_mpu_dmp_motion_driver (source and header files)
