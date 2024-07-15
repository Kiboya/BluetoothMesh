# Bluetooth Mesh LED Control

This project demonstrates the use of Bluetooth Mesh technology to control dimmable LEDs using Nordic Semiconductor's nRF52840 DK boards. It includes two main functionalities: a brightness controller and multiple dimmable LEDs. The code is based on the Bluetooth Mesh Developer Study Guide provided by the Bluetooth SIG.

## Components

### Brightness Controller

The brightness controller is implemented as a Bluetooth mesh Generic OnOff Client Model. It allows controlling the state (on/off) of the dimmable LEDs wirelessly.

### Dimmable LEDs

The dimmable LEDs act as Bluetooth mesh Generic OnOff Server Models. They respond to commands from the controller to adjust brightness levels.
