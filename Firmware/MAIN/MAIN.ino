/*
  ** E256 Firmware v1.0 (ESP8266)**
  This file is part of the eTextile-matrix-sensor project - http://matrix.eTextile.org
  Copyright (c) 2014-2018 Maurin Donneaud <maurin@etextile.org>
  This work is licensed under Creative Commons Attribution-ShareAlike 4.0 International license, see the LICENSE file for details.
*/

#include "main.h"

const char WIFI_SSID[] = "Chevrette";             // Connect to WiFi SSID
const char WIFI_PASS[] = "ch0c0latchienjaune";    // WiFi SSID setup
const IPAddress REMOTE_IP(10, 72, 1, 255);        // Brodcast IP
const unsigned int REMOTE_UDP_PORT = 9999;        // Remote port to send OSC mesages
const unsigned int LOCAL_UDP_PORT = 7777;         // Local port to listen for OSC packets (actually not used for sending)

uint8_t E256_threshold = 30; // Default threshold used to adjust toutch sensitivity (10 is low 40 is high)

OSCErrorCode error;

//////////////////////////////////////////////////// SETUP

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);        // FIXME - BUILTIN_LED is used for SPI hardware
  digitalWrite(LED_BUILTIN, LOW);      // FIXME - BUILTIN_LED is used for SPI hardware

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED);
  bootBlink(LED_BUILTIN, 12);
  Udp.begin(LOCAL_UDP_PORT);

  //pinMode(BUTTON_PIN, INPUT_PULLUP);          // Set button pin as input and activate the input pullup resistor // FIXME - NO BUTTON_PIN ON the E256!
  //attachInterrupt(BUTTON_PIN, calib, RISING); // Attach interrrupt on button PIN // FIXME - NO BUTTON_PIN ON the E256

  //SPI.setSCK(E256_SS_PIN);             // D8 - Hardware SPI no need to specify it!
  //SPI.setSCK(E256_SCK_PIN);            // D5 - Hardware SPI no need to specify it!
  //SPI.setMOSI(E256_MOSI_PIN);          // D7 - Hardware SPI no need to specify it!

  pinMode(E256_SS_PIN, OUTPUT);
  pinMode(E256_SCK_PIN, OUTPUT);
  pinMode(E256_MOSI_PIN, OUTPUT);
  digitalWrite(E256_SS_PIN, LOW);        // Set latchPin LOW (only for Teensy)
  digitalWrite(E256_SS_PIN, HIGH);       // Set latchPin HIGH
  SPI.begin();                           // Start the SPI module
  SPI.beginTransaction(settings);        // (16000000, MSBFIRST, SPI_MODE0);
  pinMode(ADC_PIN, INPUT);               // Teensy PIN A9

  //analogReadResolution(8); // Set the ADC converteur resolution to 8 bit // FIXME!

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
  bootBlink(LED_BUILTIN, 9);
}

//////////////////////////////////////////////////// LOOP
void loop() {

  OSCMessage OSCmsg;
  int size = Udp.parsePacket();

  if ( size > 0 ) {
    while ( size-- ) {
      OSCmsg.fill(Udp.read());
    }
    if (!OSCmsg.hasError()) {
      OSCmsg.dispatch("/c", matrix_calibration);
      OSCmsg.dispatch("/t", matrix_threshold);
      OSCmsg.dispatch("/r", matrix_raw_data); // TODO
      OSCmsg.dispatch("/b", matrix_blobs);
    } else {
      error = OSCmsg.getError();
      bootBlink(LED_BUILTIN, error);
    }
  }
}

//////////////////////////////////////////////////// FONCTIONS

void matrix_scan(void) {

  // Columns are TWO shift registers (digital OUTPUT PINS). We do supply ONE column at a time
  // Rows are TWO 8:1 analog multiplexers (analog INPUT PINS). We can sens ONE or TWO rows at a time

  uint16_t setCols = 0x8000;                        // Powering the first column (MSB:10000000 LSB:00000000)

  for (uint8_t col = 0; col < COLS; col++) {        // 0 to 15                                              // Sens ONE rows at a time
    for (uint8_t row = 0; row < ROWS; row++) {      // 0 to 15
      digitalWrite(E256_SS_PIN, LOW);               // Set latchPin LOW
      SPI.transfer(setCols & 0xff);                 // Shift out the LSB byte to set up the OUTPUT shift registers
      SPI.transfer(setCols >> 8);                   // Shift out the MSB byte to set up the OUTPUT shift registers
      SPI.transfer(setSingleRows[row]);             // Shift out one byte that setup the two 8:1 analog multiplexers
      digitalWrite(E256_SS_PIN, HIGH);              // Set latchPin LOW
      delayMicroseconds(5);                         // TODO: See switching time of the 74HC4051BQ multiplexeur
      uint8_t rowIndex = row * COLS + col;          // Compute rowIndex
      int val = analogRead(ADC_PIN) - minVals[rowIndex];
      val >= 0 ? frameValues[rowIndex] = (uint8_t)val : frameValues[rowIndex] = 0;
    }
    setCols = setCols >> 1;
  }
#ifdef DEBUG_ADC
  for (uint16_t i = 0; i < NEW_FRAME; i++) {
    if ((i % NEW_COLS) == (NEW_COLS - 1)) Serial.println();
    Serial.printf(F("\t%d"), frameValues[i]);
    delay(1);
  }
  Serial.println();
  delay(500);
#endif /*__DEBUG_ADC__*/
}

void matrix_calibration(OSCMessage & msg) {

  uint8_t calibration_cycles = msg.getInt(0) & 0xFF; // Get the first uint8_t in an int32_t

  for (uint8_t i = 0; i < calibration_cycles; i++) {
    // Columns are digital OUTPUT PINS that have to be supply one by one
    // Rows are analog INPUT PINS that have to be sens one by one
    uint16_t setCols = 0x8000;

    for (uint8_t col = 0; col < COLS; col++) {
      for (uint8_t row = 0; row < ROWS; row++) {
        digitalWrite(E256_SS_PIN, LOW);        // Set latchPin LOW
        SPI.transfer(setCols & 0xff);          // Shift out the LSB byte to set up the OUTPUT shift register
        SPI.transfer(setCols >> 8);            // Shift out the MSB byte to set up the OUTPUT shift register
        SPI.transfer(setSingleRows[row]);      // Shift out one byte that setup the two 8:1 analog multiplexers
        digitalWrite(E256_SS_PIN, HIGH);       // Set latchPin LOW
        delayMicroseconds(10);                 // TODO: see switching time of the 74HC4051BQ multiplexeur
        uint8_t rowIndex = row * COLS + col;   // Compute rowIndex
        uint8_t ADC_val = analogRead(ADC_PIN);
        //if (ADC_val > minVals[rowIndex]) minVals[rowIndex] = (uint8_t)ADC_val; // FIXME
        if (ADC_val > minVals[rowIndex]) minVals[rowIndex] = ADC_val >> 2;
      }
      setCols = setCols >> 1;
    }
  }
}

// Set the threshold with incoming OSC message
void matrix_threshold(OSCMessage & msg) {
  // Teensy is Little-endian!
  // The sequence addresses/sends/stores the least significant byte first (lowest address)
  // and the most significant byte last (highest address).
  E256_threshold = msg.getInt(0) & 0xFF; // Get the first int8_t of the int32_t
}

/// Send raw frame values in OSC formmat
void matrix_raw_data(OSCMessage & msg) {
  OSCBundle OSCbundle;
  //uint8_t ... = msg.getInt(0) & 0xFF;   // Get the first int8_t of the int32_t

  Udp.beginPacket(REMOTE_IP, REMOTE_UDP_PORT);
  OSCbundle.send(Udp);            // Send the bytes to the SLIP stream
  Udp.endPacket();                // Mark the end of the OSC Packet
}

/// Send all blobs values in OSC formmat
void matrix_blobs(OSCMessage & msg) {

  OSCBundle OSCbundle;
  //... = msg.getInt(0) & 0xFF; // Get the first int8_t in an int32_t

  matrix_scan();

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

  // Hear is an method to minimise the size of the OCS packet.
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
  OSCbundle.send(Udp);            // Send the bytes to the SLIP stream
  Udp.endPacket();                // Mark the end of the OSC Packet
  //OSCbundle.empty();            // empty the OSCMessage ready to use for new messages
}


void bootBlink(const uint8_t pin, uint8_t flash) {
  for (uint8_t i = 0; i < flash; i++) {
    digitalWrite(pin, HIGH);
    delay(40);
    digitalWrite(pin, LOW);
    delay(40);
  }
}
