/*
  This file is part of the eTextile-matrix-sensor project - http://matrix.eTextile.org
  Copyright (c) 2014-2019 Maurin Donneaud <maurin@etextile.org>
  This work is licensed under Creative Commons Attribution-ShareAlike 4.0 International license, see the LICENSE file for details.
*/

#ifndef __CONFIG_H__
#define __CONFIG_H__

/*__E256_ESP8266_BOARD_CONTROL__*/
#define STA_SSID          "lele"
#define STA_PSK           "michtopatato"
//#define REMOTE_IP         "192,168,0,255"
#define REMOTE_UDP_PORT    3001
#define LOCAL_UDP_PORT     7771
//#define E256_X0          -1            // TODO: select X-axis origine [-1:1]
//#define E256_Y0          -1            // TODO: select Y-axis origine [-1:1]

//#define DEBUG_ADC
//#define DEBUG_INTERP
//#define DEBUG_BLOBS_OSC

//#define DEBUG_BITMAP
//#define DEBUG_CCL
//#define DEBUG_LIST
//#define DEBUG_BLOBS_ID
//#define DEBUG_BLOBS_CENTER

/*__DO_NOT_CHANGE__*/
#define E256_SS_PIN        D8             // E2B56:RCK  - ESP8266 D1_MINI Hardware SPI:SS   -> D8 
//#define E256_SCK_PIN       D5             // E2B56:SCK  - ESP8266 D1_MINI Hardware SPI:SCLK -> D5 // OK
//#define E256_MOSI_PIN      D7             // E2B56:DS   - ESP8266 D1_MINI Hardware SPI:MOSI -> D7 // SER
//#define E256_MISO_PIN      D6             // E2B56:(NA) - ESP8266 D1_MINI Hardware SPI:MISO -> D6

// If you want to plug the E256 breakout board to an MCU with sigle ADC
// you will need to connect the E256 AN0 & AN1 to the same ADC INPUT of your MCU
#define ADC_PIN            A0             // This is the Anolog INPUT PIN of the ESP8266 D1 mini

#define LED_BUILTIN        2              // D1 MINI LED PIN
//#define BUTTON_PIN       x              // FIXME - NO BUTTON_PIN yet
#define BAUD_RATE          115200
#define COLS               16
#define ROWS               16
#define DUAL_ROWS          (ROWS / 2)
#define SCALE_X            4
#define SCALE_Y            4
#define ROW_FRAME          (COLS * ROWS)
#define NEW_COLS           (COLS * SCALE_X)
#define NEW_ROWS           (ROWS * SCALE_Y)
#define NEW_FRAME          (NEW_COLS * NEW_ROWS)
#define MAX_NODES          40             // Set the maximum nodes number
#define MIN_BLOB_PIX       4              // Set the minimum blob pixels
#define MAX_BLOB_PIX       1024           // Set the maximum blob pixels
#define BLOB_PACKET_SIZE   7              // Blob data packet (bytes)

#endif /*__CONFIG_H__*/
