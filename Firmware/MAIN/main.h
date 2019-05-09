/*
  This file is part of the eTextile-matrix-sensor project - http://matrix.eTextile.org
  Copyright (c) 2014-2018 Maurin Donneaud <maurin@etextile.org>
  This work is licensed under Creative Commons Attribution-ShareAlike 4.0 International license, see the LICENSE file for details.
*/

#ifndef __MAIN_H__
#define __MAIN_H__

#include <SPI.h>

#include <ESP8266WiFi.h>                //
#include <WiFiUdp.h>                    //

#include <OSCBoards.h>                  // https://github.com/CNMAT/OSC
#include <OSCMessage.h>                 // https://github.com/CNMAT/OSC
#include <OSCBundle.h>                  // https://github.com/CNMAT/OSC

#include "config.h"
#include "interp.h"
#include "llist.h"
#include "blob.h"

SPISettings settings(16000000, MSBFIRST, SPI_MODE0);

WiFiUDP Udp;                            // UDP instance to send and receive packets over UDP

// SINGLE Analog INPUT (ESP8266)
// Array to store all parameters used to configure the two 8:1 analog INPUT multiplexeurs (REF:
// If the chipset have SINGLE Analog INPUT you have to Perform a single Rows scanning
// Eatch byte |ENA|A|B|C|ENA|A|B|C|
byte setSingleRows[ROWS] = {
  0x58, 0x78, 0x68, 0x48, 0x28, 0x18, 0x8, 0x38,
  0x85, 0x87, 0x86, 0x84, 0x82, 0x81, 0x80, 0x83
};

char      serialConf[4] = {0};              // Array to store boot serial config
uint8_t   minVals[ROW_FRAME] = {0};         // Array to store smallest values

uint8_t   frameValues[ROW_FRAME] = {0};     // Array to store ofseted input values
image_t   rawFrame;                         // Instance of struct image_t

float     coef_A[SCALE_X * SCALE_Y] = {0};
float     coef_B[SCALE_X * SCALE_Y] = {0};
float     coef_C[SCALE_X * SCALE_Y] = {0};
float     coef_D[SCALE_X * SCALE_Y] = {0};

interp_t  interp;                           // Instance of struct interp_t

uint8_t   bilinIntOutput[NEW_FRAME] = {0};  // Bilinear interpolation output buffer
image_t   interpolatedFrame;                // Instance of struct image_t

char      bitmap[NEW_FRAME] = {0};          // 64 x 64
blob_t    blobArray[MAX_NODES] = {0};       // 40 nodes

llist_t   freeBlobs;
llist_t   blobs;
llist_t   outputBlobs;

uint8_t blobPacket[BLOB_PACKET_SIZE] = {0};

inline void matrix_scan(void);

void matrix_calibration(OSCMessage &msg);
void matrix_threshold(OSCMessage &msg);
void matrix_raw_data(OSCMessage &msg);
void matrix_blobs(OSCMessage &msg);

void bootBlink(const uint8_t pin, uint8_t flash);

#endif /*__MAIN_H__*/
