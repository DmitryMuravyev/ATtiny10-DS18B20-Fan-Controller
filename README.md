<h1>Features</h1>

- Uses DS18B20 digital temperature sensor
- Automatically detects the presence of PWM control pin (4th fan wire)
- Operates in PWM or On/Off mode (in case if DC-DC converter or power MOSFET used)
- 25 kHz PWM
- Rotation control with an attempt to restart in case of rotor stop

The controller regulates the fan speed using PWM, and the full PWM range (0 - 100%) fits into the temperature range of 25°C - 40°C. In the case of connecting a fan without PWM control or without a speed sensor (2- 3-wire fans), the control is carried out for a temperature threshold of 30°C: if sensor's temperature below this limit, then a logical 0 is set at the 6th pin of the chip, and if higher, then 1. This pin can be used for connecting to a DC-DC converter EN pin or to build another fan powering circuit (for example, using a MOSFET).

See the detailed review of the project on my YT-channel (English subtitles): https://youtu.be/6zM7jxHXjjU

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/6zM7jxHXjjU/maxresdefault.jpg)](https://www.youtube.com/watch?v=6zM7jxHXjjU)

<h1>Details</h1>
This code is written for the ATtiny10 microcontroller and as a starting point, you can refer to my project, designed for use in various household and professional equipment (e.g. LAB PSUs) and requiring two voltages for its power supply (5V and any other voltage in the 4-18V range required for your fan): https://easyeda.com/sneer2sneer/ATtiny10-Fan-Controller
You can use a minimum of elements (pullup resistors for the lines of the fan speed sensor and temperature sensor and a decoupling capacitor for power supply), but in my case the module schematic looked like this:
<br /><br />

![Schematic_ATtiny10 Fan Controller](https://github.com/DmitryMuravyev/ATtiny10-DS18B20-Fan-Controller/assets/152902525/3fb18249-e0b6-4f27-b2cb-f037db2c0447)

Description of the microcontroller pins:

Pin | Function
----|-----------------------
1   | Fan PWM Control signal / TPIDATA
2   | GND
3   | DS18B20 Sensor data line (DQ, 1-Wire) / TPICLK
4   | Fan Rotation Detector signal
5   | VCC
6   | On/Off Control signal / #RESET

For MCU Flash ROM programming you will need a USBasp programmer (or any other that supports the TPI - Tiny Programming Interface), which is connected according to this schematic:

![USBasp connection](https://github.com/DmitryMuravyev/ATtiny10-DS18B20-Fan-Controller/assets/152902525/acedddab-b8e7-42fe-8021-ab99e9c8b6b6)


The programming itself is performed from the Arduino IDE, with the ATtiny10 core installed in it (see the links below).

${\color{red}Please \space note \space (!)}$ that by default, the 6th pin of the microcontroller is used for the RESET signal (active level is low). To use this pin as a GPIO pin, you must rewrite the microcontroller's fuses (see the commands below). After you do this, you will no longer be able to flash the chip with a regular programmer. To program such a chip, you'll need 12V to be applied to the RESET pin.

${\color{red}Also \space note \space (!)}$ that the code is written for the MCU clock frequency of 10 MHz. This means that in the Arduino IDE, in the Tools->Clock menu, you should set the value to 8MHz, and then preferably calibrate the value of the OSCCAL register in the code for your specific exemplar of the chip. This can be done, for example, by measuring the duration of the 1-wire reset pulse using an oscilloscope. Its duration should be as close as possible to 480 microseconds.

<h1>Commands</h1>

Fuse programming (0xFE - set RSTDISBL bit):

    avrdude -C ..\etc\avrdude.conf -c usbasp -P usb -b 115200 -p attiny10 -v -U fuse:w:0xFE:m 


Fuse reset:

    avrdude -C ..\etc\avrdude.conf -c usbasp -P usb -b 115200 -p attiny10 -v -x section_config -e 


 
<h1>Links</h1>

Project - https://easyeda.com/sneer2sneer/ATtiny10-Fan-Controller

Datasheet ATtiny4-5-9-10 - http://ww1.microchip.com/downloads/en/DeviceDoc/atmel-8127-avr-8-bit-microcontroller-attiny4-attiny5-attiny9-attiny10_datasheet.pdf

Tips and Tricks to Optimize Code for 8-bit AVR - https://ww1.microchip.com/downloads/en/AppNotes/doc8453.pdf

Datasheet DS18B20 - https://datasheets.maximintegrated.com/en/ds/DS18B20.pdf

1-Wire protocol (Book of iButton standards) - https://pdfserv.maximintegrated.com/en/an/AN937.pdf

USBasp firmware - https://www.fischl.de/usbasp/

USBasp firmware update guide - https://www.electronics-lab.com/project/usbasp-firmware-update-guide/

ATtiny10Core for Arduino IDE - https://github.com/technoblogy/attiny10core

Programming the ATtiny10 - http://www.technoblogy.com/show?1YQY

ATtiny10 Thermometer - http://www.technoblogy.com/show?2G8A
