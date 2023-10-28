# hart_fsk

## Introduction

This project allows to communicate with HART field devices over serial port - 100% python based. HART stands for Highway Addressable Remote Transducer and is a widely 
used protocol for communication with field devices in process automation. This project focuses on physical layer FSK and talks to a HART modem connected to a 
serial port (ttyUSB0, com1) using pyserial.  
Encoding and decoding of data into HART frames supported for RX/TX. The layout of HART commands for a specific device is read from a ddl file using python module eddly.

## License

This work is distributed under the MIT license. See [LICENSE](LICENSE) for details.

## Features

- connect to a device by sending cmd 0 and verifying received data against device description from ddl (optional: talk to device directly using long address)
- object oriented design: a device class object holds all information for this device (assigned serial port, device description, connection status, device revisions, ...)
- high-level functions (e.g. command(), read_primary_variable()) are easy to use and self-document the source code
- low-level-function command_raw() can send/receive arbitrary bytearray data
- received variables can be accessed as class elements: print(response.loop_current)
- fast: RX function returns immediately after extected number of bytes arrived; no waiting for timeouts
- error handling if return_code != 0, can be disabled for testing if rc != is expected

## Caution

This project is proof-of-concept. Use it on your own risk!

## Contribution

Welcome. Thanks to all contributors.

