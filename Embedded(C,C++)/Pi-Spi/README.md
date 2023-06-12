## SPI Test

This folder contains code to test the SPI communication interface.


### SPI : SPI0 Pinout
| Function | Pin |
| :---: | :---: |
|MOSI | 19 |
|MISO| 21|
|CLK | 23|
|CE0 | 24|
|CE1 | 26|
| GND | 6|

### SPI test application 1

#### Usage
Compilation
```
$ gcc -o spidev_test spidev_test.c
```

Execution:
```
$ ./spidev_test
```

This will return back the output that looks like:
```
Device: /dev/spidev0.0
spi mode: 4
bits per word: 8
max speed: 5000000 Hz (5000 KHz)

00 00 01 09 00 00
00 00 00 00 00 00
00 00 00 00 00 00
00 00 00 00 00 00
00 00 00 00 00 00
00 00 00 00 00 00
00 00
```

#### Functionality
This test code will send out a read command (first byte = `0x01`) to the slave device to obtain data from register address starting at `0x09` (second byte of the data packet). The slave device responds by sending data starting at the address spcified.


### SPI test application 2

#### Usage
Compilation
```
$ g++ -o spi_interface spi_interface.cpp
```

Execution:
```
$ ./spi_interface
```

This will return back the output that looks like:
```
FF FF FF FF FF FF 00 00
00 00 00 00 
```
Note that the value will change from `0xFF` to some other number if the sensors are connected and working. The value of `0xFF` indicates that there has been a timeout, either because the sensor is not connected or because it is not working.

#### Functionality
This test code will send out a read command (first byte = `0x01`) to the slave device to obtain data from register address starting at `0x8B`. The slave device responds by sending data starting at the address specified.

The test code will also write to 10 registers starting at address `0x05` with multiples of 5 starting at 0. 
