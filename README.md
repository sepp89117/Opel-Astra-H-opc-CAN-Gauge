# Opel-Astra-H-opc-CAN-Gauge
Display motor data, display maximum values, read out and delete error codes, analog display

<b>Check the NEW VERSION https://github.com/sepp89117/Opel_Astra_H_opc_CAN-GaugeV2</b>

![](https://raw.githubusercontent.com/sepp89117/Opel-Astra-H-opc-CAN-Gauge/master/screenshot.png)<br>
Author: Sebastian Balzer (GER)<br>
Version: 1.0 (19.01.2020) <br>
Hardware: Teensy 4.0, Waveshare CAN Board SN65HVD230, ILI9341 320x240 TFT with XPT2046 Touch, LM2596 DC/DC Converter - set 5.0 Volt out <br>
Car: Opel Astra H opc <br>
Interface: HSCAN 500 kBaud (Pin 6 and 14 on OBD connector) <br>

TODO
- Replace all String with char*
- Acoustic alarm when limit values are exceeded
- Create an opcButton class with is-touched function
- Switch off/on ESP by pressing the opc logo in the top right corner
- Test read out several trouble codes

As a hobby programmer, I developed this tool using CAN sniffing. It gives me a lot of joy in my vehicle.
A video of the project on YouTube: https://www.youtube.com/watch?v=h3EL3UFl1sk

You are welcome to improve the code
