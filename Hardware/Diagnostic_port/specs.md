# USB - C Connector pinout

| Pin ID | Name | Pin ID | Name | Adapted function |
| ------ | ---- | ------ | ---- | ---------------- |
| A1 | GND | B1 | GND | GND |
| A2 | TX1+ | B2 | TX2+ | CAN1+ |
| A3 | TX1- | B3 | TX2- | CAN1- |
| A4 | VBUS | B4 | VBUS | Strictly +5V |
| A5 | CC1 | B5 | CC2 | SWCLK |
| A6 | D+ | B6 | D+ | USB_D+ |
| A7 | D- | A7 | D- | USB_D- |
| A8 | SBU1 | B8 | SBU2 | SWDIO |
| A9 | VBUS | B9 | VBUS | Strictly +5V |
| A10 | RX2- | B10 | RX1- | CAN2- |
| A11 | RX2+ | B11 | RX1+ | CAN2+ |
| A12 | GND | B12 | GND | GND |

NRST does not need to be connected - the SWD protocol supports rebooting over SWDIO