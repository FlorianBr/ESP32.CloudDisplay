# ESP32 Cloud-Display

Ziel: Per ESP32 angesteuertes LCD-Display plus Matrix-Display mit Cloud-Anbindung (AWS)
## Feature Liste

| Modul             | Implementiert?            | Anmerkung |
|---                |---                        |---        |
| Basissystem       | :heavy_check_mark:        |           |
| WiFi              | :heavy_check_mark:        |           |
| HTTPS-Server      | :heavy_check_mark:        |           |
| Anbindung MQTT    | :heavy_check_mark:        |           |
| NTP               | :heavy_check_mark:        |           |
| Ansteuerung GLCD  | :heavy_check_mark:        |           |
| Ansteuerung Matrix|                           |           |

TODOs:
- Bilder-Upload per Web
- GIF oder JPG Decoder [Link](https://github.com/richgel999/picojpeg)


## Hardware

* CPU: Espressif ESP32-DevKitC V4 WROVER [Link](https://www.aliexpress.com/item/1005001966992155.html)
* LCD: 2,8-Zoll SPI-LCD [Link](https://www.amazon.de/dp/B07MXH92RL)
* Matrix: 4x RGB LED Matrix Display with HUB75 Interface  [Hardware](https://de.aliexpress.com/item/32733178058.html)


### Pinbelegung

[HW-Infos](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/hw-reference/esp32/get-started-devkitc.html)
#### J1

| Nr.  |  ESP32-Pin    |  Usage |
|------|:-------------:|-------:|
| 1    | 3V3           |        |
| 2    | EN            |        |
| 3    | GPIO36        |        |
| 4    | GPIO39        |        |
| 5    | GPIO34        |        |
| 6    | GPIO35        |        |
| 7    | GPIO32        | LCD_BL   |
| 8    | GPIO33        | SPI-CLK  |
| 9    | GPIO25        | SPI-RST  |
| 10   | GPIO26        | SPI-CS   |
| 11   | GPIO27        | SPI-DC   |
| 12   | GPIO14        | SPI-MOSI |
| 13   | GPIO12        | SPI-MISO |
| 14   | GND           |        |
| 15   | GPIO13        |        |
| 16   | GPIO9         |        |
| 17   | GPIO19        |        |
| 18   | GPIO11        |        |
| 19   | 5V            |        |

#### J3

| Nr.  |  ESP32-Pin    |  Usage |
|------|:-------------:|-------:|
| 1    | GND           |        |
| 2    | GPIO33        |        |
| 3    | GPIO22        |        |
| 4    | GPIO1         |        |
| 5    | GPIO3         |        |
| 6    | GPIO21        |        |
| 7    | GND           |        |
| 8    | GPIO19        |        |
| 9    | GPIO18        |        |
| 10   | GPIO5         |        |
| 11   | GPIO17        | Reserved |
| 12   | GPIO16        | Reserved |
| 13   | GPIO4         |        |
| 14   | GPIO0         |        |
| 15   | GPIO2         |        |
| 16   | GPIO15        |        |
| 17   | GPIO8         |        |
| 18   | GPIO7         |        |
| 19   | GPIO6         |        |

## Software

[PxMatrix-Library](https://github.com/2dom/PxMatrix)
