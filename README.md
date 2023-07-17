# Wilfred, a robot design example
In this repository we provide the Circuitpython code and the Fusion 360 files needed to build and use a two DC motors robot. The robot support Wifi connection and is controlled
using MQTT protocol. (Note that some parts of the code depend of the used microcontroller and electronic modules).
The Fusion 360 files of the robot CADs and the designed PCB are available [HERE](/Robot%20design%20example/Fusion%20files).

## Main components:
- Microcontroller: [Arduino Nano RP2040 Connect](https://store.arduino.cc/products/arduino-nano-rp2040-connect)
  - <img src="/Robot%20design%20example/microcontroller.png" width="100">
- Motor driver: [DRV8833 Dual Motor Driver Carrier from Pololu](https://www.pololu.com/product/2130)
  - <img src="/Robot%20design%20example/motor_driver.png" width="100">
- IMU: [Adafruit 9-DOF Absolute Orientation IMU Fusion Breakout - BNO055](https://www.adafruit.com/product/4646)
  - <img src="/Robot%20design%20example/IMU.jpg" width="100">
- Switch: [RS PRO PCB (SPDT)](https://es.rs-online.com/web/p/interruptores-deslizantes/7347299?redirect-relevancy-data=7365617263685F636173636164655F6F726465723D31267365617263685F696E746572666163655F6E616D653D4931384E525353746F636B4E756D626572267365617263685F6D617463685F6D6F64653D6D61746368616C6C267365617263685F7061747465726E5F6D6174636865643D5E2828282872737C5253295B205D3F293F285C647B337D5B5C2D5C735D3F5C647B332C347D5B705061415D3F29297C283235285C647B387D7C5C647B317D5C2D5C647B377D29292924267365617263685F747970653D52535F53544F434B5F4E554D424552267365617263685F77696C645F63617264696E675F6D6F64653D4E4F4E45267365617263685F6B6579776F72643D3733342D37323939267365617263685F6B6579776F72645F6170703D3733343732393926)
- Batteries: 4 x [Ni-MH AA Cell](https://tienda.bricogeek.com/baterias-lipo/315-bateria-aa-recargable-nimh-2500mah.html?search_query=AA+NiMH&results=2)
- Motors: 2 x [50:1 Micro Metal Gearmotor HP 6V from Pololu](https://www.pololu.com/product/998)
- Chassis: [Zumo Chassis Kit from Pololu](https://www.pololu.com/product/1418)

## Finished robot
<img src="/Robot%20design%20example/diagram1.png" width="700">

## Circuit diagram
<img src="/Robot%20design%20example/diagram2.png" width="700">

## Circuit Schematic
<img src="/Robot%20design%20example/sch_wilfred.png" width="700">

## Designed PCB
<img src="/Robot%20design%20example/PCB_CAD.png" width="700">
  
