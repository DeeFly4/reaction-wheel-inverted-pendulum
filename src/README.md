# Requirements
Both projects are Arduino code using PlatformIO. It can not be run through the standard Arduino IDE. Instead you will need to use Microsoft's Visual Studio Code with the PlatformIO extension.

## PlatformIO
1. Open VSCode Extension Manager
2. Search for offical PlatformIO IDE extension
3. Install PlatformIO IDE

For further info on PlatformIO, go to their website https://platformio.org/

# Running the code
Open either PlatformIO project, balance or imu, in a separate window. This will build the project locally to make sure all dependencies are installed according to the project's platformio.ini file. The standard Arduino code with setup() and loop() functions will be located in the files called main.cpp. In these files you will find further instructions to make sure the wiring is correct. Make sure to also follow the instructions in the serial monitor to start the program correctly.

The project must then be uploaded to the Arduino. This can be done by pressing the upload button at the bottom tool bar in VSCode.