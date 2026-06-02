// ---------------------------------------------------------
// WiFi Helper Functions
// ---------------------------------------------------------

// Build and transmit a framed packet: [0xAA][0x55][6 payload bytes][1-byte checksum].
// The checksum is the sum of all 6 payload bytes, truncated to uint8.
void sendFramedPayload(const uint8_t p[6]) {
  uint8_t sum = 0;
  for (int i = 0; i < 6; ++i) sum += p[i];

  // Write frame: 0xAA 0x55 [6 bytes] [sum]
  WiFi_serial.write(0xAA);
  WiFi_serial.write(0x55);
  WiFi_serial.write(p, 6);
  WiFi_serial.write(sum);
}


// Pack three int16 values big-endian into a 6-byte payload and send.
// The 200 µs delay prevents back-to-back frames from colliding at the receiver.
void sendResponse(int16_t r1, int16_t r2, int16_t r3) {
  uint8_t out[6];
  out[0] = (uint8_t)((r1 >> 8) & 0xFF);
  out[1] = (uint8_t)( r1       & 0xFF);
  out[2] = (uint8_t)((r2 >> 8) & 0xFF);
  out[3] = (uint8_t)( r2       & 0xFF);
  out[4] = (uint8_t)((r3 >> 8) & 0xFF);
  out[5] = (uint8_t)( r3       & 0xFF);

  delayMicroseconds(200);
  sendFramedPayload(out);
}
