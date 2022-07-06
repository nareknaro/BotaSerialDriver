# BotaSerialDriver

This repository is a minimum driver to read sensor data from Bota Systems serial sensors. The driver is consist of a header file and source file. The driver is implemented as abstract class in C++ as it requires acess to hardware specifically a serial port and C++ doesnt offer an abstracted serial port acess. For this reason the user will have to derive its own class from the driver and implement the serialReadBytes, serialAvailable functions.
In the example folder can be found such implementation for different platforms (Arduino and Linux)

## Usage

### Linux

Compile
```
g++ examples/read_sensor_linux.cpp BotaForceTorqueSensorComm.cpp -o serial_driver
```
Run
```
./serial_driver
```
