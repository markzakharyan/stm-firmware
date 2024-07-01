#pragma once

#include <Arduino.h>

#include "SPI.h"

class PeripheralCommsController {
 private:
  SPISettings spiSettings;
  bool spiInitialized;

 public:
  PeripheralCommsController(SPISettings spi_s)
      : spiSettings(spi_s), spiInitialized(false) {}

  void setup() {
    if (!spiInitialized) {
      SPI.begin();
      spiInitialized = true;
    }
  }

  void sendByte(byte b) {
    SPI.beginTransaction(spiSettings);
    SPI.transfer(b);
    SPI.endTransaction();
  }

  void sendBytes(const byte* data, size_t len) {
    SPI.beginTransaction(spiSettings);
    for (size_t i = 0; i < len; ++i) {
      SPI.transfer(data[i]);
    }
    SPI.endTransaction();
  }

  template <typename... byte>
  void sendBytes(byte... args) {
    SPI.beginTransaction(spiSettings);
    // (SPI.transfer(static_cast<byte>(args)), ...);
    for (auto b : {args...}) {
      SPI.transfer(b);
    }
    SPI.endTransaction();
  }

  byte receiveByte() {
    SPI.beginTransaction(spiSettings);
    byte b = SPI.transfer(0);
    SPI.endTransaction();
    return b;
  }

  void receiveBytes(byte* data, size_t len) {
    SPI.beginTransaction(spiSettings);
    for (size_t i = 0; i < len; ++i) {
      data[i] = SPI.transfer(0);
    }
    SPI.endTransaction();
  }
};