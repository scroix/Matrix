/*
  This file is part of the eTextile-matrix-sensor project - http://matrix.eTextile.org
  Copyright (c) 2014-2018 Maurin Donneaud <maurin@etextile.org>
  This work is licensed under Creative Commons Attribution-ShareAlike 4.0 International license, see the LICENSE file for details.
*/

#ifndef __CONFIG_H__
#define __CONFIG_H__

/*__E256_ESP8266_BOARD CONTROL__*/
//#define E256_X0 -1                      // TODO: select X-axis origine [-1:1]
//#define E256_Y0 -1                      // TODO: select Y-axis origine [-1:1]

#define LED_BUILTIN        D2             //
//#define BUTTON_PIN       32             // FIXME - NO BUTTON_PIN on the E256 PCB
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

#define E256_SS_PIN        D8             // SPI:SS    E2B56:RCK  // D8 - ESP8266 D1_MINI Hardware SPI
#define E256_SCK_PIN       D5             // SPI:SCK   E2B56:SCK  // D5 - ESP8266 D1_MINI Hardware SPI
#define E256_MOSI_PIN      D7             // SPI:MOSI  E2B56:DS   // D7 - ESP8266 D1_MINI Hardware SPI

// If you want to plug the E256 breakout board to an MCU with sigle ADC you will need to add a strap on the E256 PCB
// Connect the multiplexerA OUTPUT PIN (AN0) to the multiplexerB OUTPUT PIN (AN1)
// Use the multiplexerA output (AN0) as main Analog OUTPUT PIN
#define ADC_PIN            A0             // This is the Anolog INPUT PIN of the ESP8266 D1 mini

//#define DEBUG_ADC
//#define DEBUG_INTERP
//#define DEBUG_BLOBS_OSC
//#define DEBUG_BITMAP
//#define DEBUG_CCL
//#define DEBUG_BLOB_ID
//#define DEBUG_CENTER
//#define DEBUG_LIST

#endif /*__CONFIG_H__*/
