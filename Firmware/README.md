# B201 Location demo software

## General behavior

* Position, fix status, UTC time  and data valid are read from the NMEA message $xxRMC
* If fix status is ok and data are valid the position and time are presented in the standardized GATT Service ”Location and Navigation”
* If a remote device subscribes to GATT notifications these will be sent for every $xxRMC message NINA-B1 receives from the GNSS module
* When switching off the device with the ON/OFF button NINA-B1 will switch off the power to the GNSS module before entering sleep mode
* The device will automatically switch off if inactive for more than 3 min
* LED1 will indicate device running (0.2/1.8s) as well as any active BLE connection (0.4/4.0s)

## Program the device

Precompiled firmware is provided and can be used with B201 hardware. The .hex file is available in the ```/hex``` folder.   
**Please note!** This firmware requires SoftDevice s132_nrf52_3.0.0 to be programmed to the device.

## Compile the software

* Clone this repository   
```> git clone https://github.com/u-blox/blueprint-B201-NINA-B1-EVA-M8```

* Download the source code for nRF5 SDK version 12.2.0 from [Nordic](https://www.nordicsemi.com/eng/nordic/Products/nRF5-SDK/nRF5-SDK-v12-zip/54291)

* Copy ```custom_board.h``` to the ```/components/boards``` folder of the SDK

* Update the SDK_ROOT variable in the Makefile according to your installation of the nRF5 SDK   
```For example: SDK_ROOT := C:/nRF5_SDK_12.2.0```

* Go to the project directory (where the Makefile is located) and execute the ```make``` command

* The generated software (ninab1.hex) is located in the ```/_build``` folder