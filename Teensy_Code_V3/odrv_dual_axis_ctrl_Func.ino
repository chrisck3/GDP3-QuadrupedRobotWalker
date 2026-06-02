// Helper functions for dual-axis control 
void setAxisState(Stream& serial, int axis, int state) {
  serial.print("w axis");
  serial.print(axis);
  serial.print(".requested_state ");
  serial.println(state);
}

void setAxisPosition(Stream& serial, int axis, float position) {
  serial.print("w axis");
  serial.print(axis);
  serial.print(".controller.input_pos ");
  serial.println(position);
}

float getAxisPosition(Stream& serial, int axis) {
  serial.print("r axis");
  serial.print(axis);
  serial.println(".encoder.pos_estimate");
  return serial.parseFloat();
}
int readInt(Stream& s) {
  // ODrive ASCII returns something like: "3\n"
  while (!s.available()) { /* small wait */ }
  return s.parseInt();
}

float readFloat(Stream& s) {
  while (!s.available()) { /* small wait */ }
  return s.parseFloat();
}

int getAxisState(Stream& serial, int axis) {
  serial.print("r axis"); serial.print(axis); serial.println(".current_state");
  return serial.parseInt();
}

int getAxisError(Stream& serial, int axis) {
  serial.print("r axis"); serial.print(axis); serial.println(".error");
  return serial.parseInt();
}
