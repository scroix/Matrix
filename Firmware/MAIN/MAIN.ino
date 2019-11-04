/*
  ** E256 Firmware v1.1 ** (DO NOT WORK!)
  This file is part of the eTextile-matrix-sensor project - http://matrix.eTextile.org
  Copyright (c) 2014-2019 Maurin Donneaud <maurin@etextile.org>
  This work is licensed under Creative Commons Attribution-ShareAlike 4.0 International license, see the LICENSE file for details.
*/

// FPS with CPU speed to 120 MHz (Overclock)
// Optimize Fastest with LTO (Link Time Optimizations)
//    .. FPS : ADC_INPUT
//    .. FPS : ADC_INPUT / BLOB_TRACKING
//    .. FPS : ADC_INPUT / BILINEAR_INTERPOLATION
//    .. FPS : ADC_INPUT / BILINEAR_INTERPOLATION / BLOB_TRACKING

#include <OSCBoards.h>              // https://github.com/CNMAT/OSC
#include <OSCMessage.h>             // https://github.com/CNMAT/OSC
#include <OSCBundle.h>              // https://github.com/CNMAT/OSC
#include <SLIPEncodedUSBSerial.h>   // https://github.com/CNMAT/OSC

#include "config.h"
#include "interp.h"
#include "blob.h"

// Digital pins array
// See the attached home made PCB (Eagle file) to understand the Digital and Analog pin mapping
const int rowPins[RAW_ROWS] = {
  27, 26, 25, 24, 12, 11, 10, 9, 8, 7, 6, 5, 33, 2, 1, 0
};

// Analog pins array
const int columnPins[RAW_COLS] = {
  A17, A18, A19, A0, A20, A1, A2, A3, A4, A5, A6, A7, A11, A8, A10, A9
};

uint8_t minValsArray[RAW_FRAME] = {0};            // 1D Array to store smallest values
uint8_t frameArray[RAW_FRAME] = {0};              // 1D Array to store ofseted analog input values

uint8_t bitmapArray[NEW_FRAME] = {0};             // 1D Array to store binary values (16*16) array containing (64*64) values
uint8_t interpFrameArray[NEW_FRAME] = {0};        // 1D Array to store bilinear interpolated values

xylr_t lifoArray[LIFO_MAX_NODES] = {0};           // 1D Array to store lifo nodes
blob_t blobArray[MAX_NODES] = {0};                // 1D Array to store blobs

uint8_t blobPacket[BLOB_PACKET_SIZE] = {0};       // 1D Array to store blobs in OSC format

image_t  inputFrame;         // Input frame values
interp_t interp;             //
image_t  interpolatedFrame;  //
image_t  bitmap;             // Used by Scanline Flood Fill algorithm / SFF
lifo_t   lifo_stack;         // Lifo free nodes stack
lifo_t   lifo;               // Lifo stack
llist_t  blobs_stack;        // Blobs free nodes linked list
llist_t  blobs;              // Intermediate blobs linked list
llist_t  outputBlobs;        // Output blobs linked list

SLIPEncodedUSBSerial SLIPSerial(thisBoardsSerialUSB);

uint8_t threshold = 25;
uint8_t calibration_cycles = 20;
boolean calibrate = true;
boolean scanning = true;

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);   // FIXME - BUILTIN_LED is used for SPI hardware
  digitalWrite(LED_BUILTIN, LOW); // FIXME - BUILTIN_LED is used for SPI hardware

#ifdef DEBUG_ADC
#ifdef DEBUG_INTERP
#ifdef DEBUG_BLOBS_OSC
#ifdef DEBUG_SFF_BITMAP
#ifdef DEBUG_FPS
  Serial.begin(BAUD_RATE);       // Arduino serial library ** 230400 **
  while (!Serial.dtr());         // Wait for user to start the serial monitor
#else
  SLIPSerial.begin(BAUD_RATE);   // Arduino serial library ** 230400 ** extended with SLIP encoding
#endif /*__DEBUG_ADC__*/
#endif /*__DEBUG_INTERP__*/
#endif /*__DEBUG_BLOBS_OSC__*/
#endif /*__DEBUG_SFF_BITMAP__*/
#endif /*__DEBUG_FPS__*/

  //pinMode(BUTTON_PIN, INPUT_PULLUP);          // Set button pin as input and activate the input pullup resistor // FIXME - NO BUTTON_PIN ON the E256!
  //attachInterrupt(BUTTON_PIN, calib, RISING); // Attach interrrupt on button PIN // FIXME - NO BUTTON_PIN ON the E256

  //ADC_SETUP();

  for (int i = 0; i < RAW_ROWS; i++) {
    pinMode(rowPins[i], OUTPUT);
  }

  INTERP_SETUP(
    &inputFrame,          // image_t*
    &frameArray[0],       // uint8_t*
    &interpolatedFrame,   // image_t*
    &interpFrameArray[0], // uint8_t*
    &interp               // interp_t*
  );

  BLOB_SETUP(
    &inputFrame,          // image_t*
    &bitmap,              // image_t*
    &bitmapArray[0],      // uint8_t*
    &lifo,                // lifo_t*
    &lifo_stack,          // lifo_t*
    &lifoArray[0],        // xylr_t*
    &blobs,               // list_t*
    &blobs_stack,         // list_t*
    &blobArray[0],        // blob_t*
    &outputBlobs          // list_t*
  );
}

//////////////////// LOOP
void loop() {

#ifdef E256_RUN

  OSCMessage OSCmsg;

  int size;
  while (!SLIPSerial.endofPacket()) {
    if ((size = SLIPSerial.available()) > 0) {
      while (size--)
        OSCmsg.fill(SLIPSerial.read());
    }
  }
  if (!OSCmsg.hasError()) {
    OSCmsg.dispatch("/c", matrix_calibration_set);
    OSCmsg.dispatch("/t", matrix_threshold_set);
    OSCmsg.dispatch("/r", matrix_raw_data_get);
    OSCmsg.dispatch("/i", matrix_interp_data_get);
    OSCmsg.dispatch("/x", matrix_interp_data_bin_get);
    OSCmsg.dispatch("/b", matrix_blobs_get);
  }
#endif /*__E256_RUN__*/

#ifdef DEBUG_ADC
  matrix_scan(&frameArray[0]);

  for (uint8_t col = 0; col < RAW_COLS; col++) {
    for (uint8_t row = 0; row < RAW_ROWS; row++) {
      uint8_t index = col * RAW_COLS + row;          // Compute 1D array index
      Serial.printf("\t%d", frameArray[index]);
    }
    Serial.println();
  }
  Serial.println();
  delay(50);
#endif /*__DEBUG_ADC__*/

#ifdef DEBUG_INTERP
  if (calibrate) matrix_calibrate(&minValsArray[0]);
  matrix_scan(&frameArray[0]);
  matrix_interp(&interpolatedFrame, &inputFrame, &interp);
  for (uint8_t col = 0; col < NEW_COLS; col++) {
    for (uint8_t row = 0; row < NEW_ROWS; row++) {
      uint16_t index = col * NEW_COLS + row;          // Compute 1D array index
      Serial.printf("-%d", interpFrameArray[index]);
    }
    Serial.println();
  }
  Serial.println();
  delay(50);
#endif /*__DEBUG_INTERP__*/

#ifdef DEBUG_BITMAP
  if (calibrate) matrix_calibrate(&minValsArray[0]);
  matrix_scan(&frameArray[0]);
  matrix_interp(&interpolatedFrame, &inputFrame, &interp);
  find_blobs(
    threshold,          // uint8_t
    &interpolatedFrame, // image_t (uint8_t array[NEW_FRAME] - 64*64 1D array)
    &bitmap,            // image_t (uint8_t array[NEW_FRAME] - 64*64 1D array)
    &lifo_stack,        // lifo_t
    &lifo,              // lifo_t
    &blobs_stack,       // list_t
    &blobs,             // list_t
    &outputBlobs        // list_t
  );

  for (uint8_t posY = 0; posY < NEW_COLS; posY++) {
    uint8_t* bmp_row = COMPUTE_BINARY_IMAGE_ROW_PTR (&bitmap, posY);
    for (uint8_t posX = 0; posX < NEW_ROWS; posX++) {
      Serial.print(IMAGE_GET_BINARY_PIXEL_FAST(bmp_row, posX));
    }
    Serial.println();
  }
  Serial.println();
  delay(50);
#endif /*__DEBUG_BITMAP__*/

#ifdef DEBUG_BLOBS_OSC
  if (calibrate) matrix_calibrate(&minValsArray[0]);
  matrix_scan(&frameArray[0]);
  matrix_interp(&interpolatedFrame, &inputFrame, &interp);
  find_blobs(
    threshold,          // uint8_t
    &interpolatedFrame, // image_t (uint8_t array[NEW_FRAME] - 64*64 1D array)
    &bitmap,            // image_t (uint8_t array[NEW_FRAME] - 64*64 1D array)
    &lifo_stack,        // lifo_t
    &lifo,              // lifo_t
    &blobs_stack,       // list_t
    &blobs,             // list_t
    &outputBlobs        // list_t
  );

  for (blob_t* blob = ITERATOR_START_FROM_HEAD(&outputBlobs); blob != NULL; blob = ITERATOR_NEXT(blob)) {
    Serial.print (blob->UID);        // uint8_t unique session ID
    Serial.print(" ");
    Serial.print (blob->alive);      // uint8_t
    Serial.print(" ");
    Serial.print (blob->centroid.X); // uint8_t
    Serial.print(" ");
    Serial.print (blob->centroid.Y); // uint8_t
    Serial.print(" ");
    Serial.print (blob->box.W);      // uint8_t
    Serial.print(" ");
    Serial.print (blob->box.H);      // uint8_t
    Serial.print(" ");
    Serial.print (blob->box.D);      // uint8_t
    Serial.println();
  }
#endif /*__DEBUG_BLOBS_OSC__*/
}

//////////////////////////////////////////////////// FONCTIONS

void ADC_SETUP(void) {
  // TODO: set all ADC to 8 bits
}

// Columns are digital OUTPUT PINS - We supply them one by one sequentially
// Rows are analog INPUT PINS - We sens them two by two
void matrix_scan(uint8_t* outputFrame) {

  for (int row = 0; row < RAW_ROWS; row++) {
    // Set row pin as output + 3.3V
    digitalWrite(rowPins[row], HIGH);
    for (int column = 0; column < RAW_COLS; column++) {

      int adcVal = analogRead(columnPins[column]); // Read the sensor value -> TODO: set all ADC to 8 bits

      int sensorIndex = row * RAW_ROWS + column; // Calculate the index of the unidimensional array

      adcVal = map(adcVal, minValsArray[sensorIndex], 1023, 0, 255);
      adcVal = constrain(adcVal, 0, 255);

      int sensorVal = adcVal - minValsArray[sensorIndex];
      sensorVal > 0 ? outputFrame[sensorIndex] = (uint8_t)sensorVal : outputFrame[sensorIndex] = 0;
    }
    // pinMode(rowPins[row], INPUT); // Set row pin in high-impedance state
    digitalWrite(rowPins[row], LOW); // Set row pin to GND
  }
}

void matrix_calibration_set(OSCMessage & msg) {
  calibration_cycles = msg.getInt(0) & 0xFF;   // Get the first uint8_t in an int32_t
  calibrate = true;
}

// Columns are digital OUTPUT PINS - We supply them one by one sequentially
// Rows are analog INPUT PINS - We sens them two by two
void matrix_calibrate(uint8_t* outputFrame) {

  for (uint8_t i = 0; i < calibration_cycles; i++) {
    for (int row = 0; row < RAW_ROWS; row++) {
      // Set row pin as output + 3.3V
      digitalWrite(rowPins[row], HIGH);
      for (int column = 0; column < RAW_COLS; column++) {
        int adcVal = analogRead(columnPins[column]); // Read the sensor value -> TODO: set all ADC to 8 bits
        int sensorIndex = row * RAW_ROWS + column; // Calculate the index of the unidimensional array
        if (adcVal > outputFrame[sensorIndex]) outputFrame[sensorIndex] = adcVal;
      }
      // pinMode(rowPins[row], INPUT); // Set row pin in high-impedance state
      digitalWrite(rowPins[row], LOW); // Set row pin to GND
    }
  }
  calibrate = false;
}

// Set the threshold
void matrix_threshold_set(OSCMessage & msg) {
  threshold = msg.getInt(0) & 0xFF; // Get the first uint8_t of the int32_t
}

// Send raw frame values in SLIP-OSC formmat
void matrix_raw_data_get(OSCMessage & msg) {

  if (calibrate) matrix_calibrate(&minValsArray[0]);
  matrix_scan(&frameArray[0]);
  OSCMessage m("/r");
  m.add(frameArray, RAW_FRAME);
  SLIPSerial.beginPacket();
  m.send(SLIPSerial);        // Send the bytes to the SLIP stream
  SLIPSerial.endPacket();    // Mark the end of the OSC Packet
}

// Send interpolated frame values in SLIP-OSC formmat
void matrix_interp_data_get(OSCMessage & msg) {

  if (calibrate) matrix_calibrate(&minValsArray[0]);
  matrix_scan(&frameArray[0]);
  matrix_interp(&interpolatedFrame, &inputFrame, &interp);
  OSCMessage m("/i");
  m.add(interpFrameArray, NEW_FRAME);
  SLIPSerial.beginPacket();
  m.send(SLIPSerial);        // Send the bytes to the SLIP stream
  SLIPSerial.endPacket();    // Mark the end of the OSC Packet
}

// Send interpolated frame values in SLIP-OSC formmat
void matrix_interp_data_bin_get(OSCMessage & msg) {

  if (calibrate) matrix_calibrate(&minValsArray[0]);
  matrix_scan(&frameArray[0]);
  matrix_interp(&interpolatedFrame, &inputFrame, &interp);
  find_blobs(
    threshold,          // uint8_t
    &interpolatedFrame, // image_t (uint8_t array[NEW_FRAME] - 64*64 1D array)
    &bitmap,            // image_t (uint8_t array[NEW_FRAME] - 64*64 1D array)
    &lifo_stack,        // lifo_t
    &lifo,              // lifo_t
    &blobs_stack,       // list_t
    &blobs,             // list_t
    &outputBlobs        // list_t
  );
  OSCMessage m("/x");
  m.add(&bitmapArray[0], RAW_FRAME);
  SLIPSerial.beginPacket();
  m.send(SLIPSerial);        // Send the bytes to the SLIP stream
  SLIPSerial.endPacket();    // Mark the end of the OSC Packet
}

void matrix_blobs_get(OSCMessage & msg) {

  if (calibrate) matrix_calibrate(&minValsArray[0]);
  matrix_scan(&frameArray[0]);
  matrix_interp(&interpolatedFrame, &inputFrame, &interp);
  find_blobs(
    threshold,          // uint8_t
    &interpolatedFrame, // image_t (uint8_t array[NEW_FRAME] - 64*64 1D array)
    &bitmap,            // image_t (uint8_t array[NEW_FRAME] - 64*64 1D array)
    &lifo_stack,        // lifo_t
    &lifo,              // lifo_t
    &blobs_stack,       // list_t
    &blobs,             // list_t
    &outputBlobs        // list_t
  );

  OSCBundle OSCbundle;

  // Send all blobs in OCS bundle
  for (blob_t* blob = ITERATOR_START_FROM_HEAD(&outputBlobs); blob != NULL; blob = ITERATOR_NEXT(blob)) {
    blobPacket[0] = blob->UID;        // uint8_t unique session ID
    blobPacket[1] = blob->alive;      // uint8_t
    blobPacket[2] = blob->centroid.X; // uint8_t
    blobPacket[3] = blob->centroid.Y; // uint8_t
    blobPacket[4] = blob->box.W;      // uint8_t
    blobPacket[5] = blob->box.H;      // uint8_t
    blobPacket[6] = blob->box.D;      // uint8_t

    OSCMessage msg("/b");
    msg.add(blobPacket, BLOB_PACKET_SIZE);
    OSCbundle.add(msg);
  }
  SLIPSerial.beginPacket();     //
  OSCbundle.send(SLIPSerial);   // Send the bytes to the SLIP stream
  SLIPSerial.endPacket();       // Mark the end of the OSC Packet
}
