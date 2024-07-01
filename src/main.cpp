#include <Arduino.h>

#include <vector>

#include "Config.h"
#include "FunctionRegistry.h"
#include "FunctionRegistryMacros.h"
#include "Peripherals/DAC/DACController.h"
#include "Peripherals/PeripheralCommsController.h"
#include "UserIOHandler.h"
#include "Peripherals/ADC/ADCController.h"

FunctionRegistry registry;
UserIOHandler userIOHandler(registry);
PeripheralCommsController dacCommsController(DAC_SPI_SETTINGS);
PeripheralCommsController adcCommsController(ADC_SPI_SETTINGS);
DACController dacController(registry, dacCommsController);
ADCController adcController(registry, adcCommsController);

void setup() {
  userIOHandler.initialize();
  dacController.setup();
  adcController.setup();
  

  for (int i : dac_cs_pins) {
    dacController.addChannel(i, ldac);
  }


  for (unsigned int i = 0; i < sizeof(drdy)/sizeof(drdy[0]); i++) {
      adcController.addBoard(adc_sync_pins[i], drdy[i], reset[i]);
  }
}

void loop() { userIOHandler.handleUserIO(); }