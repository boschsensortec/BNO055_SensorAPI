## Table of Contents
- [Introduction](#intro)
- [Version](#ver)
- [Integration details](#integration)
- [Driver files information](#fileinfo)
- [Supported sensor interface](#interface)
- [Copyright](#copy)

### Introduction<a name=intro></a>
- This package contains the Bosch Sensortec MEMS BNO055 sensor driver (sensor API)
- The sensor driver package includes bno055.h, bno055.c and bno055_support.c files

### Version<a name=ver></a>
- Version of bno055 sensor driver is:
    * bno055.c 		- V2.0.3
    * bno055.h 		- V2.0.3
    * bno055_support.c 	- V1.0.4

### Integration details<a name=integration></a>
- Integrate bno055.h and bno055.c file in to your project.
- The bno055_support.c file contains only examples for API use cases, so it is not required to integrate into project.

### Driver files information<a name=fileinfo></a>
- bno055.h
    * This header file has the register address definition, constant definitions, data type definition and supported sensor driver calls declarations.
- bno055.c
    * This file contains the implementation for the sensor driver APIs.
- bno055_support.c
    * This file shall be used as an user guidance, here you can find samples of
            * Initialize the sensor with I2C communication
                    - Add your code to the I2C bus read and bus write functions.
                            - Return value can be chosen by yourself
                        - API just passes that value to your application code
                    - Add your code to the delay function
                    - Change I2C address accordingly in bno055.h
        * Power mode configuration of the sensor
        * Get and set functions usage
        * Reading the sensor read out data

### Supported sensor interface<a name=interface></a>
- This BNO055 sensor driver supports I2C and SPI interfaces

### Copyright<a name=copy></a>
- Copyright (C) 2015 - 2016 Bosch Sensortec GmbH

