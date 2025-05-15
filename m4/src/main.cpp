#include <Arduino.h>

#include "Config.h"
#include "FunctionRegistry/FunctionRegistry.h"
#include "FunctionRegistry/FunctionRegistryHelpers.h"
#include "Peripherals/ADC/ADCController.h"
#include "Peripherals/DAC/DACController.h"
#include "Peripherals/PeripheralCommsController.h"
#include <vector>
#include "UserIOHandler.h"

#include "Utils/shared_memory.h"



void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  if (!initSharedMemory()) {
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
  UserIOHandler::setup();

  PeripheralCommsController::setup();
  

  for (int i : dac_cs_pins) {
    DACController::addChannel(i);
  }

  for (int i=0; i<NUM_ADC_BOARDS; i++) {
    ADCController::addBoard(adc_cs_pins[i], drdy[i], reset[i]);
  }

  DACController::setup();
  ADCController::setup();
  
  // wait for calibration data to be loaded
  while (!isBootComplete());

  CalibrationData calibrationData;
  m4ReceiveCalibrationData(calibrationData);

  for (int i=0; i<NUM_DAC_CHANNELS; i++) {
    DACController::setCalibration(i, calibrationData.offset[i], calibrationData.gain[i]);
  }

}



void loop() {
  UserIOHandler::handleUserIO();
}