/*
  This file is part of the eTextile-matrix-sensor project - http://matrix.eTextile.org
  Copyright (c) 2014-2019 Maurin Donneaud <maurin@etextile.org>
  This work is licensed under Creative Commons Attribution-ShareAlike 4.0 International license, see the LICENSE file for details.
*/

#ifndef __MAIN_H__
#define __MAIN_H__

#include <SPI.h>
#include <ESP8266WiFi.h>                //
#include <WiFiUdp.h>                    //
#include <OSCMessage.h>                 // https://github.com/CNMAT/OSC
#include <OSCBundle.h>                  // https://github.com/CNMAT/OSC

#include "config.h"
#include "interp.h"
#include "llist.h"
#include "blob.h"

WiFiUDP Udp;                            // UDP instance to send and receive packets over UDP
OSCErrorCode error;

// ESP8266 SINGLE Analog INPUT collecting the E256 TWO Analog OUTPUTS
// Array to store all parameters used to configure the two 8:1 analog INPUT multiplexeurs (REF: 74HC4051BQ)
// If the chipset have SINGLE Analog INPUT you have to Perform a single Rows scanning
// Eatch byte |ENA|A|B|C|ENA|A|B|C|
const byte setRow[ROWS] = {
  0x85, 0x87, 0x86, 0x84, 0x82, 0x81, 0x80, 0x83, 0x58, 0x78, 0x68, 0x48, 0x28, 0x18, 0x8, 0x38
};

uint16_t  minVals[ROW_FRAME] = {0};         // Array to store smallest values
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

uint8_t   E256_threshold = 30;                // Default threshold used to adjust toutch sensitivity (10 is low 40 is high)

uint8_t blobPacket[BLOB_PACKET_SIZE] = {0};

inline void matrix_scan(void);

void matrix_calibration(OSCMessage &msg);
void matrix_threshold(OSCMessage &msg);
void matrix_raw_data(OSCMessage &msg);
void matrix_blobs();

void bootBlink(uint8_t flash);

#endif /*__MAIN_H__*/
