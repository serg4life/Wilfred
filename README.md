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
- Switch: [RS PRO PCB (SPDT)](https://es.rs-online.com/web/p/interruptores-deslizantes/7347296)
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
The Gerber files of this PCB design are availabe [HERE](/Robot%20design%20example/GERBER)
<img src="/Robot%20design%20example/PCB_CAD.png" width="700">
  
