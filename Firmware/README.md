# E256 eTextile-matrix-sensor / ESP8266

### Transforming textiles into an intuitive way to interact with computers. This project is part of an electronic textiles research on HCI gesture interaction that was started in 2005.

## Requirements
- E256 brekout board
- ESP8266 (D1 MINI)
- Arduino IDE additional hardware
  - ESP8266: https://github.com/esp8266/Arduino#installing-with-boards-manager
- Arduino IDE additional library
  - OSC: https://github.com/CNMAT/OSC (Installed with Arduino library manager)

### Setting up Arduino IDE
To be able to use Arduino IDE with ESP8266 MCU add this web link to to "File > Preferences > Additional Boards Manager > URLs:
    http://arduino.esp8266.com/stable/package_esp8266com_index.json
Open Boards Manager from Tools > Board > Boards Manager, and type ESP8266
Working with ESP8266 V4.0.0

Linux requirements: add UDEV rules to /etc/udev/init.d/

- Board:       	 ESP8266
- CPU frequency  160 MHz
- Uplode speed   230400
- Flash size 	 3M (4M SPIFFS)

### Setting up Arduino-mk (Optional)
    sudo apt-get install arduino-mk
    git clone https://github.com/sudar/Arduino-Makefile.git

## Program Synopsis
- E256 brekout board communicate with ESP8266 via SPI (Hardware).
- The Arduio sketch implemant rows and columns scaning algorithm.
  - COLS = Two 8_Bits shift registers connected directly to the matrix columns.
  - ROWS = One 8_Bits shift register connected to two analog multiplexers that sens the matrix rows.
- The 16x16 Analog sensors values are interpolated into 64x64 with a bilinear algorithm.
- The blob tracking algorithm (connected component labeling) is applyed onto the interpolated matrix.
- Each blob are tracked with persistent ID (this is done with linked list implementation).
- The blobs coordinates, size and presure are transmited via OSC communication protocol.

## OSC data paket
### on_touch_pressed
    [0-255]	UID		   // Percistant blob ID
    [1]		alive	   //
    [0-64]	centroid.X // X blob centroid coordinate
    [0-64]	centroid.Y // Y blob centroid coordinate
    [0-255]	box.W 	   // Bounding box Width
    [0-255]	box.H 	   // Bounding box Height
    [0-255]	box.D 	   // Bounding box depth

### on_touch_release
    [0-255]	UID		   // Percistant blob ID
    [0]		alive	   // 
    [0-64]	centroid.X // X blob centroid coordinate
    [0-64]	centroid.Y // Y blob centroid coordinate
    [0-255]	box.W 	   // Bounding box Width
    [0-255]	box.H 	   // Bounding box Height
    [0-255]	box.D 	   // Bounding box depth

## E256 & Teensy pins
Control pins to send values to the 8-BITs shift registers used on the E-256 PCB

| ESP8266 PINS         | E256 PINS | E256 components                                      |
| -------------------- | --------- | ---------------------------------------------------- |
| AD0                  | AN1       | OUTPUT of an analog multiplexer 8:1                  |
| GND                  | GND       | Ground                                               |
| D5  (SPI:SCK)        | SCK       | 74HC595 clock pin (SH_CP)                            |
| AD0                  | AN0       | OUTPUT of an analog multiplexer 8:1                  |
| 3V3                  | VCC       | VCC                                                  |
| D7  (SPI:MOSI)       | DS        | Data input of the first 8-BIT shift register 74HC595 |
| D8  (SPI:SS)         | RCK       | Latch pin of the first 74HC595 (ST_CP)               |
| GND                  | GND       | Ground                                               |

## Copyright
Except as otherwise noted, all files in the resistiveMatrix project folder

    Copyright (c) 2005-2017 Maurin Donneaud and others.

For information on usage and redistribution, and for a DISCLAIMER OF ALL
WARRANTIES, see LICENSE.txt included in the resistiveMatrix project folder.

## Acknowledgements
Thanks to Vincent Roudaut, Hannah Perner Willson, Cedric Honnet, Antoine Meisso, Paul Strohmeier

## TODO
- Optimise interpolation method
  - Retrieval method from Microchip TB3064 white paper (p12)
  - microchip.com/stellent/groups/techpub_sg/documents/devicedoc/en550192.pdf
- Add TUIO 1.0 Protocol Specification: https://www.tuio.org/?tuio10

## E256 PCB Hardware routing 

## Shift register_0 is pluged to MUX_A & MUX_B
    Q0 -> SO_A  // Pin Q0 connected to Analog MUX_A pin S0
    Q1 -> S1_A  // Pin Q1 connected to Analog MUX_A pin S1
    Q2 -> S2_A  // Pin Q2 connected to Analog MUX_A pin S2
    Q3 -> EN_A  // Pin Q3 connected to Analog MUX_A pin Enable (active LOW)
    Q4 -> SO_B  // Pin Q4 connected to Analog MUX_B pin S0
    Q5 -> S1_B  // Pin Q5 connected to Analog MUX_B pin S1
    Q6 -> S2_B  // Pin Q6 connected to Analog MUX_B pin S2
    Q7 -> EN_B  // Pin Q7 connected to Analog MUX_A pin Enable (active LOW)

### Analog MUX_A outputs are direcly pluged to the columns
    Y0 -> ROW_6 // Pin Y0 connected to ROW 6
    Y1 -> ROW_5 // Pin Y1 connected to ROW 5
    Y2 -> ROW_4 // Pin Y2 connected to ROW 4
    Y3 -> ROW_7 // Pin Y3 connected to ROW 7
    Y4 -> ROW_3 // Pin Y4 connected to ROW 3
    Y5 -> ROW_0 // Pin Y5 connected to ROW 0
    Y6 -> ROW_2 // Pin Y6 connected to ROW 2
    Y7 -> ROW_1 // Pin Y7 connected to ROW 1

### Analog MUX_B outputs are direcly pluged to the columns
    Y0 -> ROW_14 // Pin Y0 connected to ROW 14
    Y1 -> ROW_13 // Pin Y1 connected to ROW 13
    Y2 -> ROW_12 // Pin Y2 connected to ROW 12
    Y3 -> ROW_15 // Pin Y3 connected to ROW 15
    Y4 -> ROW_11 // Pin Y4 connected to ROW 11
    Y5 -> ROW_8  // Pin Y5 connected to ROW 8
    Y6 -> ROW_10 // Pin Y6 connected to ROW 10
    Y7 -> ROW_9  // Pin Y7 connected to ROW 9

## Shift register_1 outputs are direcly pluged to the columns
    Q0 -> COL_7  // Pin Q0 connected to column 7
    Q1 -> COL_6  // Pin Q1 connected to column 6
    Q2 -> COL_5  // Pin Q2 connected to column 5
    Q3 -> COL_4  // Pin Q3 connected to column 4
    Q4 -> COL_3  // Pin Q4 connected to column 3
    Q5 -> COL_2  // Pin Q5 connected to column 2
    Q6 -> COL_1  // Pin Q6 connected to column 1
    Q7 -> COL_0  // Pin Q7 connected to column 0

## Shift register_2 outputs are direcly pluged to the columns
    Q0 -> COL_15  // Pin Q0 connected to column 15
    Q1 -> COL_14  // Pin Q1 connected to column 14
    Q2 -> COL_13  // Pin Q2 connected to column 13
    Q3 -> COL_12  // Pin Q3 connected to column 12
    Q4 -> COL_11  // Pin Q4 connected to column 11
    Q5 -> COL_10  // Pin Q5 connected to column 10
    Q6 -> COL_9   // Pin Q6 connected to column 9
    Q7 -> COL_8   // Pin Q7 connected to column 8

## Bytes to scan the matrix

### Byte_A
    COL_8 ->  Q7 : 10000000 -> HEX 0x80
    COL_9 ->  Q6 : 01000000 -> HEX 0x40
    COL_10 -> Q5 : 00100000 -> HEX 0x20
    COL_11 -> Q4 : 00010000 -> HEX 0x10
    COL_12 -> Q3 : 00001000 -> HEX 0x8
    COL_13 -> Q2 : 00000100 -> HEX 0x4
    COL_14 -> Q1 : 00000010 -> HEX 0x2
    COL_15 -> Q0 : 00000001 -> HEX 0x1

    COL_0 -> Q7 : 10000000 -> HEX 0x80
    COL_1 -> Q6 : 01000000 -> HEX 0x40
    COL_2 -> Q5 : 00100000 -> HEX 0x20
    COL_3 -> Q4 : 00010000 -> HEX 0x10
    COL_4 -> Q3 : 00001000 -> HEX 0x8
    COL_5 -> Q2 : 00000100 -> HEX 0x4
    COL_6 -> Q1 : 00000010 -> HEX 0x2
    COL_7 -> Q0 : 00000001 -> HEX 0x1

### Byte_B
    ROW_0 -> Y5 : 0101 1000 -> HEX 0x58
    ROW_1 -> Y7 : 0111 1000 -> HEX 0x78
    ROW_2 -> Y6 : 0110 1000 -> HEX 0x68
    ROW_3 -> Y4 : 0100 1000 -> HEX 0x48
    ROW_4 -> Y2 : 0010 1000 -> HEX 0x28
    ROW_5 -> Y1 : 0001 1000 -> HEX 0x18
    ROW_6 -> Y0 : 0000 1000 -> HEX 0x8
    ROW_7 -> Y3 : 0011 1000 -> HEX 0x38

    ROW_8 -> Y5  : 1000 0101 -> HEX 0x85
    ROW_9 -> Y7  : 1000 0111 -> HEX 0x87
    ROW_10 -> Y6 : 1000 0110 -> HEX 0x86
    ROW_11 -> Y4 : 1000 0100 -> HEX 0x84
    ROW_12 -> Y2 : 1000 0010 -> HEX 0x82
    ROW_13 -> Y1 : 1000 0001 -> HEX 0x81
    ROW_14 -> Y0 : 1000 0000 -> HEX 0x80
    ROW_15 -> Y3 : 1000 0011 -> HEX 0x83

