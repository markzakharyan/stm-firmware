#pragma once

#include <Arduino.h>
#include <Peripherals/PeripheralCommsController.h>

class ADCBoard {
 private:
  int sync_pin;
  int data_ready_pin;
  int reset_pin;
  PeripheralCommsController &commsController;

  float map2(float x, long in_min, long in_max, float out_min,
             float out_max)  // float
  {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

  int twoByteToInt(byte DB1,
                   byte DB2)  // This gives a 16 bit integer (between +/- 2^16)
  {
    return ((int)((DB1 << 8) | DB2));
  }

  void intToTwoByte(int s, byte *DB1, byte *DB2) {
    *DB1 = ((byte)((s >> 8) & 0xFF));
    *DB2 = ((byte)(s & 0xFF));
  }

  void waitDRDY() {
    int count = 0;
    while (digitalRead(data_ready_pin) == HIGH && count < 2000) {
      count = count + 1;
      delay(1);
    }
  }

 public:
  ADCBoard(PeripheralCommsController &commsController, int data_ready_pin,
             int reset_pin, int sync_pin)
      : data_ready_pin(data_ready_pin),
        reset_pin(reset_pin),
        sync_pin(sync_pin),
        commsController(commsController) {}

  void setup() {
    pinMode(reset_pin, OUTPUT);
    pinMode(data_ready_pin, INPUT);
    pinMode(sync_pin, OUTPUT);
    digitalWrite(sync_pin, HIGH);

    // Resets ADC on startup.
    digitalWrite(reset_pin, HIGH);
    digitalWrite(reset_pin, LOW);
    delay(5);
    digitalWrite(reset_pin, HIGH);
  }

  void initialize() {}

  float readVoltage(int channel_index) {
    int statusbyte = 0;
    byte o2;
    byte o3;
    int ovr;
    digitalWrite(sync_pin, LOW);
    commsController.sendByte(
        0x38 + channel_index);  // Indicates comm register to access
                                      // mode register with channel
    digitalWrite(sync_pin, HIGH);
    digitalWrite(sync_pin, LOW);
    commsController.sendByte(0x48);  // Indicates mode register to start
                                     // single convertion in dump mode
    digitalWrite(sync_pin, HIGH);
    waitDRDY();  // Waits until convertion finishes
    digitalWrite(sync_pin, LOW);
    commsController.sendByte(
        0x48 + channel_index);  // Indcates comm register to read data
                                      // channel data register
    digitalWrite(sync_pin, HIGH);
    digitalWrite(sync_pin, LOW);
    statusbyte = commsController.receiveByte();  // Reads Channel 'ch' status
    digitalWrite(sync_pin, HIGH);
    digitalWrite(sync_pin, LOW);
    o2 = commsController.receiveByte();  // Reads first byte
    digitalWrite(sync_pin, HIGH);
    digitalWrite(sync_pin, LOW);
    o3 = commsController.receiveByte();  // Reads second byte
    digitalWrite(sync_pin, HIGH);

    ovr = statusbyte & 1;
    switch (ovr) {
      case 0:
        int decimal;
        decimal = twoByteToInt(o2, o3);
        float voltage;
        voltage = map2(decimal, 0, 65536, -10.0, 10.0);
        return voltage;
        break;

      case 1:
        return 0.0;
        break;
    }
  }
};