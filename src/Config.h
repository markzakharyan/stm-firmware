// Config.h
#ifndef CONFIG_H
#define CONFIG_H


#define SERIAL_BAUD_RATE 115200

///////////////////////////////////////////////////////////////
//  Arduino pin initialization - used to appropriately map   //
//  physical GPIO to software data names                     //
///////////////////////////////////////////////////////////////

const int adc_sync_pins[2] = {48, 42}; // SYNC for both 16-bit ADCs
const int dac_cs_pins[4] = {24, 26, 38, 36}; // CS for 4x 20-bit DAC channels
const int ldac = 22; // LDAC pin shared across all AD5791 -- used to synchronize DAC voltage output
const int reset[2] = {46, 44}; // reset pins on ADC
const int drdy[2] = {50, 40}; // data_ready pin for both ADCs -- used as input to indicate ADC conversion has completed
const int led = 7; // indicator LED
const int data = 6; // data indicator LED
const int err = 11;


const static SPISettings DAC_SPI_SETTINGS(35000000, MSBFIRST, SPI_MODE1);

const float DEFAULT_DAC_FULL_SCALE = 10.0;


#endif // CONFIG_H