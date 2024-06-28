#include <Arduino.h>
#include <SPI.h>
#include "Config.h"

class DACBoard {
 private:
  static const int NUM_CHANNELS = 4;
  int cs_pins[NUM_CHANNELS];
  int ldac_pin;
  float offset_error[NUM_CHANNELS];
  float gain_error[NUM_CHANNELS];
  float voltage[NUM_CHANNELS];
  float voltage_upper_bound[4];
  float voltage_lower_bound[4];
  float DAC_FULL_SCALE = DEFAULT_DAC_FULL_SCALE;

 public:
  DACBoard(const int *cs_pins, int ldac_pin) {
    for (int i = 0; i < NUM_CHANNELS; i++) {
      this->cs_pins[i] = cs_pins[i];
      offset_error[i] = 0.0;
      gain_error[i] = 1.0;
      voltage[i] = 0.0;
    }

    for (int i = 0; i < NUM_CHANNELS; i++) {
      pinMode(cs_pins[i], OUTPUT);
      digitalWrite(cs_pins[i], HIGH);
    }
    pinMode(ldac_pin, OUTPUT);
    digitalWrite(ldac_pin,
                 HIGH);  // Load DAC pin for DAC. Make it LOW if not in use.

    for (int i = 0; i < NUM_CHANNELS; i++) {
      voltage_upper_bound[i] = DAC_FULL_SCALE * gain_error[i] + offset_error[i];
      voltage_lower_bound[i] =
          -DAC_FULL_SCALE * gain_error[i] + offset_error[i];
    }
  }

  int getNumChannels() { return NUM_CHANNELS; }

  void initialize() { normalMode(); }

  void normalMode() {
    for (int i = 0; i < NUM_CHANNELS; i++) {
      setVoltage(i, 0);
      SPI.beginTransaction(DAC_SPI_SETTINGS);
      digitalWrite(cs_pins[i], LOW);
      SPI.transfer(32);  // Write to control register
      SPI.transfer(0);   // Reserved byte
      SPI.transfer(2);   // Unclamp DAC from GND
      digitalWrite(cs_pins[i], HIGH);
      SPI.endTransaction();
    }
  }

  float setVoltage(int channel, float voltage) {
    if (channel < 0 || channel >= NUM_CHANNELS) {
      return -1;  // Invalid channel
    }

    byte b1;
    byte b2;
    byte b3;

    voltageToDecimal(voltage / gain_error[channel] - offset_error[channel], &b1,
                     &b2, &b3);
    SPI.beginTransaction(DAC_SPI_SETTINGS);
    digitalWrite(dac_cs_pins[channel], LOW);
    SPI.transfer(b1);  // send command byte to DAC
    SPI.transfer(b2);  // MS data bits, DAC2
    SPI.transfer(b3);  // LS 8 data bits, DAC2
    digitalWrite(dac_cs_pins[channel], HIGH);
    SPI.endTransaction();

    digitalWrite(ldac, LOW);

    digitalWrite(ldac, HIGH);

    float v = gain_error[channel] *
           (threeByteToVoltage(b1, b2, b3) + offset_error[channel]);

    this->voltage[channel] = v;

    return v;
  }

  float setVoltageBuffer(int channel, float voltage) {
    byte b1;
    byte b2;
    byte b3;

    voltageToDecimal(voltage / gain_error[channel] - offset_error[channel], &b1,
                     &b2, &b3);

    SPI.beginTransaction(DAC_SPI_SETTINGS);
    digitalWrite(channel, LOW);
    SPI.transfer(b1);  // send command byte to DAC
    SPI.transfer(b2);  // MS data bits, DAC2
    SPI.transfer(b3);  // LS 8 data bits, DAC2
    digitalWrite(channel, HIGH);
    SPI.endTransaction();

    return threeByteToVoltage(b1, b2, b3);
  }

  float getVoltage(int channel) {
    if (channel < 0 || channel >= NUM_CHANNELS) {
      return -1;  // Invalid channel
    }
    return voltage[channel];
  }

  void setCalibration(int channel, float offset, float gain) {
    if (channel < 0 || channel >= NUM_CHANNELS) {
      return;  // Invalid channel
    }
    this->offset_error[channel] = offset;
    this->gain_error[channel] = gain;
  }

  void setFullScale(float full_scale) {
    DAC_FULL_SCALE = full_scale;
    for (int i = 0; i < NUM_CHANNELS; i++) {
      voltage_upper_bound[i] = DAC_FULL_SCALE * gain_error[i] + offset_error[i];
      voltage_lower_bound[i] =
          -DAC_FULL_SCALE * gain_error[i] + offset_error[i];
    }
  }


  float getLowerBound(int channel) {
    if (channel < 0 || channel >= NUM_CHANNELS) {
      return 0.0;  // Invalid channel
    }
    return voltage_lower_bound[channel];
  }

  float getUpperBound(int channel) {
    if (channel < 0 || channel >= NUM_CHANNELS) {
      return 0.0;  // Invalid channel
    }
    return voltage_upper_bound[channel];
  }

  float getOffsetError(int channel) { return offset_error[channel]; }
  float getGainError(int channel) { return gain_error[channel]; }

  float sendCode(int channel, int decimal) {
    byte b1;
    byte b2;
    byte b3;

    intToThreeBytes(decimal, &b1, &b2, &b3);
    SPI.beginTransaction(DAC_SPI_SETTINGS);
    digitalWrite(dac_cs_pins[channel], LOW);
    SPI.transfer(b1);  // send command byte to DAC
    SPI.transfer(b2);  // MS data bits, DAC2
    SPI.transfer(b3);  // LS 8 data bits, DAC2
    digitalWrite(dac_cs_pins[channel], HIGH);
    SPI.endTransaction();

    digitalWrite(ldac, LOW);

    digitalWrite(ldac, HIGH);

    float v = gain_error[channel] *
           (threeByteToVoltage(b1, b2, b3) + offset_error[channel]);

    voltage[channel] = v;

    return v;
  }

 private:
  int voltageToDecimal(float voltage, byte *DB1, byte *DB2, byte *DB3) {
    int decimal;
    if (voltage >= 0) {
      decimal = voltage * 524287 / DAC_FULL_SCALE;
    } else {
      decimal = voltage * 524288 / DAC_FULL_SCALE + 1048576;
    }
    intToThreeBytes(decimal, DB1, DB2, DB3);
  }

  void intToThreeBytes(int decimal, byte *DB1, byte *DB2, byte *DB3) {
    *DB1 = (byte)((decimal >> 16) | 16);
    *DB2 = (byte)((decimal >> 8) & 255);
    *DB3 = (byte)(decimal & 255);
  }

  int threeByteToInt(
      byte DB1, byte DB2,
      byte DB3)  // This gives a 16 bit integer (between +/- 2^16)
  {
    return ((int)(((((DB1 & 15) << 8) | DB2) << 8) | DB3));
  }

  float threeByteToVoltage(byte DB1, byte DB2, byte DB3) {
    int decimal;
    float voltage;

    decimal = threeByteToInt(DB1, DB2, DB3);

    if (decimal <= 524287) {
      voltage = decimal * DAC_FULL_SCALE / 524287;
    } else {
      voltage = -(1048576 - decimal) * DAC_FULL_SCALE / 524288;
    }
    return voltage;
  }
};