## EEL7415/7515 - IoT LoRa

This branch contains the firmware for EEL7415/7515 courses. The LoRaWAN stack version is [1.0.3](https://lora-alliance.org/sites/default/files/2018-07/lorawan1.0.3.pdf), which can be obtained in the [LoRa-Mac node](https://github.com/Lora-net/LoRaMac-node/) project. The buid system is composed by CMake and the GNU ARM-Toolchain.

#### Folders structure

The folders structure used in the project is:

    .
    ├── cmake                   # CMAKE generation files.
        ├── stm32l0.cmake       # File that contains the compilation flags.
    ├── Docs                    # Boards schematics and usefull commands.
    ├── src
        ├── apps                # Main application files for different boards.
        ├── boards              # Boards peripherals and HAL drivers.
        ├── mac                 # LoRaWAN stack.
        ├── peripherals         # External and internal sensors drivers.
        ├── radio               # Semtech radios (SX127x) drivers.
        ├── system              # Peripherals abstraction functions, used in main application.
    └── ...

Each `src/` subfolder contains a `CMakeLists.txt` file, where it is specified what C files should be compiled and auxiliary directories paths are included. When adding a new file, it is necessary to update the correspondet CMake file.

#### Prerequisites

It is highly recommended to use Linux Ubuntu version 18.xx or 19.xx to compile the project.

* **CMake >= 3.6**

```sh
$ sudo apt-get install cmake
```

* **GNU ARM-Toolchain (tested with versions 6.3.1 and 7.3.1)**

```sh
$ sudo apt-get install gcc-arm-none-eabi
```

* **Make**

```sh
$ sudo apt-get install make
```

#### Compiling the code

Before compiling the code, it is necessary to generate the Makefile using the **cmake** tool. The sequence of commands below can be used with the project:

```sh
$ cd iot-eel/
$ mkdir build
$ cd build
$ cmake -DCMAKE_TOOLCHAIN_FILE="cmake/toolchain-arm-none-eabi.cmake" -DAPPLICATION="LoRaMac" -DSUB_PROJECT="classA" -DACTIVE_REGION="LORAMAC_REGION_AU915" -DBOARD="B-L072Z-LRWAN1" -DREGION_AU915="ON" ..
```
There is the possibility to choose the application, target board and other options using some parameters when executing the CMake through command line.

Parameters available:

| Parameter          | Possible values |
| -------------      | :-------------|
| `APPLICATION`      | **LoRaMac** |
| `SUB_PROJECT`      | **classA**, **classC** |
| `BOARD`            | **B-L072Z-LRWAN1** |
| `ACTIVE_REGION`    | **LORAMAC_REGION_AU915**, **LORAMAC_REGION_US915**, **LORAMAC_REGION_EU868** |
| `REGION_XXXXX="ON"`| **REGION_AU915="ON"**, **REGION_US915="ON"**, **REGION_EU868="ON"** |

To compile:

```sh
$ make
```

The binary file will be located in ```src/apps/LoRaMac/LoRaMac-classA.bin```. To clean the build files, use: ```make clean```. To remove all files created by CMake, just delete the ```build/``` folder and start over.

#### Flashing the binary

To upload the firmware to the board, just copy the binary file to the USB device created when the B-L072Z-LRWAN1 is connected to the computer.

Using command line, this can the perform, inside the `build/` folder, as:

```sh
$ cp src/apps/LoRaMac/LoRaMac-classA.bin /media/$your-user-name$/DIS_L072Z/
```

#### Visualizing the messages through Serial

The UART interface of the board is used to debug with prints. To access this interface, install the `Cutecom` program:

```sh
$ sudo apt-get install cutecom
```

To open the software:

```sh
$ sudo cutecom
```
The baudrate should be configured as 115200.

#### Updating the code

The main files the project contains that can be updated to change the code behavior, implement new functionalities or features are:

* src/apps/LoRaMac/classA/B-L072Z-LRWAN1/**main.c**: project main application. This file contains some `#define` that can be used to change some configuration of the device. Not only this, but also has the functions of LoRaWAN parameters configuration, payload creation, stack callbacks and a FSM to handle the project routines.

* src/apps/LoRaMac/classA/B-L072Z-LRWAN1/**Commissioning.h**: file where the device keys, address and activation mode (ABP or OTAA) can be configured.

* src/boards/B-L072Z-LRWAN1/**board.c**: hardware interfaces and peripherals initializtion.

* src/apps/LoRaMac/classA/B-L072Z-LRWAN1/**tools.c**: hardware interfaces and peripherals initializtion (HTS221 and LPS22HB).
