/*
  E256 Firmware v1.0 (ESP8266)
  This file is part of the eTextile-matrix-sensor project - http://matrix.eTextile.org
  Copyright (c) 2014-2019 Maurin Donneaud <maurin@etextile.org>
  This work is licensed under Creative Commons Attribution-ShareAlike 4.0 International license, see the LICENSE file for details.
*/

#include "main.h"

IPAddress REMOTE_IP(192, 168, 0, 110);

//////////////////////////////////////////////////// SETUP
void setup() {

  //Serial.begin(BAUD_RATE);

  pinMode(LED_BUILTIN, OUTPUT);                 // Set LED_BUILTIN as OUTPUT
  digitalWrite(LED_BUILTIN, LOW);               // Set LED_BUILTIN to LOW

  //pinMode(BUTTON_PIN, INPUT_PULLUP);          // Set button pin as input and activate the input pullup resistor // FIXME - NO BUTTON_PIN yet
  //attachInterrupt(BUTTON_PIN, calib, RISING); // Attach interrrupt on button PIN // FIXME - NO BUTTON_PIN yet

  pinMode(E256_SS_PIN, OUTPUT);                  //
  SPI.begin();                                   // Start the SPI module

  // Set the ESP8266 to WiFi-client MODE
  WiFi.mode(WIFI_STA);
  WiFi.begin(STA_SSID, STA_PSK);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(200);
  }
  /*
    Serial.println();
    Serial.println("Connected!");
    Serial.print("ESP_E256 IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("ESP_E256 LOCAL_UDP_PORT: ");
    Serial.println(LOCAL_UDP_PORT);
    Serial.print("ESP_E256 REMOTE_UDP_PORT: ");
    Serial.println(REMOTE_UDP_PORT);
  */
  Udp.begin(LOCAL_UDP_PORT);

  pinMode(ADC_PIN, INPUT);               // ESP8266 PIN A0

  // Raw frame init
  rawFrame.numCols = COLS;
  rawFrame.numRows = ROWS;
  rawFrame.pData = frameValues; // 16 x 16 // uint8_t frameValues[ROW_FRAME];

  // Interpolate config init
  interp.scale_X = SCALE_X;
  interp.scale_Y = SCALE_Y;
  interp.outputStride_Y = SCALE_X * SCALE_Y * COLS;
  interp.pCoefA = coef_A;
  interp.pCoefB = coef_B;
  interp.pCoefC = coef_C;
  interp.pCoefD = coef_D;

  // Interpolated frame init
  interpolatedFrame.numCols = NEW_COLS;
  interpolatedFrame.numRows = NEW_ROWS;
  interpolatedFrame.pData = bilinIntOutput; // 64 x 64

  // Blobs list init
  llist_raz(&freeBlobs);
  llist_init(&freeBlobs, blobArray, MAX_NODES); // Add 40 nodes in the freeBlobs linked list
  llist_raz(&blobs);
  llist_raz(&outputBlobs);

  bilinear_interp_init(&interp);
}

//////////////////////////////////////////////////// LOOP
void loop() {

  /*
    OSCMessage OSCmsg;
    int size = Udp.parsePacket();

    if ( size > 0 ) {
    while ( size-- ) {
      OSCmsg.fill(Udp.read());
    }
    if (!OSCmsg.hasError()) {
      OSCmsg.dispatch("/c", matrix_calibration);
      OSCmsg.dispatch("/t", matrix_threshold);
      OSCmsg.dispatch("/r", matrix_raw_data);
    } else {
      error = OSCmsg.getError();
      //bootBlink(error);
    }
    }
  */

  matrix_scan();

#ifndef DEBUG_ADC || DEBUG_INTERP
  OSCBundle OSCbundle;

  bilinear_interp(&interpolatedFrame, &rawFrame, &interp);

  find_blobs(
    &interpolatedFrame,    // image_t uint8_t [NEW_FRAME] - 1D array
    bitmap,                // char array [NEW_FRAME] - 1D array // NOT &bitmap !?
    NEW_ROWS,              // const int
    NEW_COLS,              // const int
    E256_threshold,        // uint8_t
    MIN_BLOB_PIX,          // const int
    MAX_BLOB_PIX,          // const int
    &freeBlobs,            // list_t
    &blobs,                // list_t
    &outputBlobs           // list_t
  );
#endif

#ifndef DEBUG_BLOBS_OSC
  // Send all blobs an OCS bundle
  for (blob_t* blob = iterator_start_from_head(&outputBlobs); blob != NULL; blob = iterator_next(blob)) {
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
  Udp.beginPacket(REMOTE_IP, REMOTE_UDP_PORT);
  OSCbundle.send(Udp);               // Send the bytes to the SLIP stream
  Udp.endPacket();                   // Mark the end of the OSC Packet
#else

  for (blob_t* blob = iterator_start_from_head(&outputBlobs); blob != NULL; blob = iterator_next(blob)) {
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
#endif

#ifdef DEBUG_ADC
  for (uint8_t col = 0; col < COLS; col++) {
    for (uint8_t row = 0; row < ROWS; row++) {
      uint8_t index = col * COLS + row;          // Compute 1D array index
      Serial.print("\t");
      Serial.print(frameValues[index]);
    }
    Serial.println();
  }
  Serial.println();
#endif /*__DEBUG_ADC__*/

#ifdef DEBUG_INTERP
  for (uint8_t col = 0; col < NEW_COLS; col++) {
    for (uint8_t row = 0; row < NEW_ROWS; row++) {
      uint16_t index = col * NEW_COLS + row;          // Compute 1D array index
      Serial.print(" ");
      Serial.print(bilinIntOutput[index]);
    }
    Serial.println();
  }
  Serial.println();
#endif /*__DEBUG_INTERP__*/

}

//////////////////////////////////////////////////// FONCTIONS

void matrix_scan(void) {

  // Columns are TWO shift registers (digital OUTPUT PINS). We do supply (VCC) ONE column at a time
  // Rows are TWO 8:1 analog multiplexers (analog INPUT PINS). We do sens ONE row at a time
  uint32_t column = 0x10000;                           // Supply the first column (MSB:10000000 LSB:00000000)
  for (uint8_t col = 0; col < COLS; col++) {
    column = column >> 1;
    for (uint8_t row = 0; row < ROWS; row++) {
      digitalWrite(E256_SS_PIN, LOW);                  // Set latchPin LOW (IO-D8,10k pull-down, SS GPIO15)
      SPI.transfer(column & 0xFF);                     // Shift out the LSB byte to set up the OUTPUT shift register
      SPI.transfer(column >> 8);                       // Shift out the MSB byte to set up the OUTPUT shift register
      SPI.transfer(setRow[row]);                       // Shift out one byte to setup the two 8:1 analog multiplexers
      digitalWrite(E256_SS_PIN, HIGH);                 // Set the latchPin HIGH
      //delayMicroseconds(1);                           // TODO: See switching time of the 74HC4051BQ multiplexeur

      uint16_t rowVal = analogRead(ADC_PIN);           // https://arduino-esp8266.readthedocs.io/en/2.5.0/reference.html#analog-input
      uint8_t index = col * COLS + row;                // Compute 1D array index

      if (rowVal > E256_threshold) {
        uint8_t val = (rowVal >> 2) & 0xFF;
        frameValues[index] = val;
      }
      else {
        frameValues[index] = 0;
      }
    }
  }
}

void matrix_calibration(OSCMessage & msg) {

  uint8_t calibration_cycles = (uint8_t)msg.getInt(0) & 0xFF; // Get the first uint8_t in an int32_t
  for (uint8_t i = 0; i < calibration_cycles; i++) {
    // Columns are digital OUTPUT PINS that have to be supply one by one
    // Rows are analog INPUT PINS that have to be sens one by one
    uint32_t column = 0x10000;                     // Supply the first column (MSB:10000000 LSB:00000000)
    for (uint8_t col = 0; col < COLS; col++) {
      column = column >> 1;
      for (uint8_t row = 0; row < ROWS; row++) {
        digitalWrite(E256_SS_PIN, LOW);            // Set latchPin LOW (IO-D8, 10k pull-down, SS GPIO15)
        SPI.transfer(column & 0xFF);               // Shift out the LSB byte to set up the OUTPUT shift register
        SPI.transfer(column >> 8);                 // Shift out the MSB byte to set up the OUTPUT shift register
        SPI.transfer(setRow[row]);                 // Shift out one byte to setup the two 8:1 analog multiplexers (
        digitalWrite(E256_SS_PIN, HIGH);           // Set the latchPin HIGH

        uint16_t rowVal = analogRead(ADC_PIN);
        uint8_t index = col * COLS + row;          // Compute 1D array index
        if (rowVal > minVals[index]) {
          minVals[index] = rowVal;
        }
      }
    }
  }
}

// Set the threshold with incoming OSC message
void matrix_threshold(OSCMessage & msg) {
  // Teensy is Little-endian!
  // The sequence addresses/sends/stores the least significant byte first (lowest address)
  // and the most significant byte last (highest address).
  E256_threshold = msg.getInt(0) & 0xFF; // Get the first int8_t of the int32_t
  Serial.print("__THRESHOLD: ");
  Serial.println(E256_threshold);
}

/// Send raw frame values in OSC formmat
void matrix_raw_data(OSCMessage & msg) {
  OSCBundle OSCbundle;

  //uint8_t ... = msg.getInt(0) & 0xFF;   // Get the first int8_t of the int32_t

  Udp.beginPacket(REMOTE_IP, REMOTE_UDP_PORT);
  OSCbundle.send(Udp);            // Send the bytes to the SLIP stream
  Udp.endPacket();                // Mark the end of the OSC Packet
}

void bootBlink(uint8_t flash) {
  for (uint8_t i = 0; i < flash; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
}
