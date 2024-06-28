// Ardunio *DUE*code for controlling EVAL-AD7734 ADC and EVAL-AD5791 DAC
// Andrea Young
// Carlos Kometter
// James Ehrets
// Liam A. Cohen

// 3/23/2018
// Updated: 09/04/2019 by Liam A. Cohen for DAC + 2xDiff ADC box base line code
// Updated: 08/09/2020 by Liam A. Cohen -- updated SPI initialization
// Updated: 07/01/2020 by Davis S. Thuillier -- added functions to send DAC code
// directly and set offset/gain error in Flash storage

#include <math.h>

#include <vector>

#include "SPI.h"  // necessary library for SPI communication
// #include <DueFlashStorage.h>
#include <Arduino.h>
#include <Config.h>

#include "DAC_Controller.h"

// DueFlashStorage dueFlashStorage;

///////////////////////////////////////////////////////////////
//  Initialization procedures: serial input functions,       //
//  initial variables, SPI parameters, and calibration const.//
///////////////////////////////////////////////////////////////

// Define commands recognized via serial input
const int Noperations = 26;
String operations[Noperations] = {"NOP",
                                  "INITIALIZE",
                                  "SET",
                                  "SET_DAC_CODE",
                                  "GET_DAC",
                                  "GET_ADC",
                                  "RAMP1",
                                  "RAMP2",
                                  "BUFFER_RAMP",
                                  "BUFFER_RAMP_DIS",
                                  "RESET",
                                  "TALK",
                                  "CONVERT_TIME",
                                  "*IDN?",
                                  "*RDY?",
                                  "GET_DUNIT",
                                  "SET_DUNIT",
                                  "ADC_ZERO_SC_CAL",
                                  "ADC_CH_ZERO_SC_CAL",
                                  "ADC_CH_FULL_SC_CAL",
                                  "DAC_CH_CAL",
                                  "FULL_SCALE",
                                  "INQUIRY_OSG",
                                  "SET_OSG",
                                  "CHECKSUM",
                                  "SERIAL_NUMBER"};

// initial variables
int initialized = 0;  // address of where initialized variable is stored
int first_run_indicator = 1024;
int delayUnit = 0;  // 0=microseconds 1=miliseconds
int adc_select = 0;

// Calibration constants

// define SPI settings for DACs and ADCs
SPISettings dacSettings(35000000, MSBFIRST, SPI_MODE1);
SPISettings adcSettings(8000000, MSBFIRST, SPI_MODE3);

DACController dac_controller;

std::vector<String> query_serial() {
  char received;
  String inByte = "";
  std::vector<String> comm;
  while (received != '\r')  // Wait for carriage return
  {
    if (Serial.available()) {
      received = Serial.read();
      if (received == '\n' || received == ' ') {
      } else if (received == ',' || received == '\r') {
        comm.push_back(inByte);  // Adds string to vector of command arguments
        inByte = "";             // Resets to a null string for the next word
      } else {
        inByte += received;  // Adds newest char to end of string
      }
    }
  }
  return comm;
}

/* Extension of namespace std that provides implementations of
   __throw_bad_alloc() and __throw_length_error() since STL is not fully
   supported on Arduino DUE */
// namespace std {
//   void __throw_bad_alloc() {}
//   void __throw_length_error( char const*e ) {}
// };

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);

  pinMode(reset[0], OUTPUT);
  pinMode(reset[1], OUTPUT);
  pinMode(drdy[0], INPUT);  // Data ready pin for the ADC1.
  pinMode(drdy[1], INPUT);  // Data ready pin for the ADC2.
  pinMode(led, OUTPUT);     // Used for blinking indicator LED
  digitalWrite(led, HIGH);
  pinMode(data, OUTPUT);
  pinMode(adc_sync_pins[0], OUTPUT);
  pinMode(adc_sync_pins[1], OUTPUT);
  digitalWrite(adc_sync_pins[0], HIGH);
  digitalWrite(adc_sync_pins[1], HIGH);

  dac_controller.addBoard(dac_cs_pins, ldac);

  SPI.begin();

  digitalWrite(reset[0], HIGH);
  digitalWrite(data, LOW);
  digitalWrite(reset[0], LOW);
  digitalWrite(data, HIGH);
  delay(5);
  digitalWrite(reset[0], HIGH);
  digitalWrite(data, LOW);  // Resets ADC1 on startup.
  digitalWrite(reset[1], HIGH);
  digitalWrite(data, LOW);
  digitalWrite(reset[1], LOW);
  digitalWrite(data, HIGH);
  delay(5);
  digitalWrite(reset[1], HIGH);
  digitalWrite(data, LOW);  // Resets ADC2 on startup.

  // for (int i = 0; i < 8; i ++)
  // {
  //   union
  //   {
  //     float f;
  //     unsigned char b[4];
  //   } num;
  //   for (int j = 0; j < 4; j++){
  //     num.b[j] = dueFlashStorage.read(4*(i+1)+256+j); // Read offset/gain
  //     value from flash storage
  //   }
  //   if (i < 4)
  //   {
  //     if (isnan(num.f)) { //Set offset to 0 if there is no offset stored in
  //     flash
  //       OS[i] = 0.0;
  //       num.f = 0.0;
  //       dueFlashStorage.write(4*(i+1)+256, num.b, 4);
  //     }
  //     else
  //     {
  //       OS[i] = num.f;
  //     }
  //   }
  //   else
  //   {
  //     if (isnan(num.f)) { //Set gain to 1 if there is no gain stored in flash
  //       GE[i] = 1.0;
  //       num.f = 1.0;
  //       dueFlashStorage.write(4*(i+1)+256, num.b, 4);
  //     }
  //     else
  //     {
  //       GE[i-4] = num.f;
  //     }
  //   }
  //   //Serial.println(num.f,6);
  // }

  // if (dueFlashStorage.read(initialized) != 1)
  // {
  //   normalMode();
  //   dueFlashStorage.write(initialized, 1);
  // }
}

void blinker(int s) {
  digitalWrite(data, HIGH);
  delay(s);
  digitalWrite(data, LOW);
  delay(s);
}

void sos() {
  blinker(50);
  blinker(50);
  blinker(50);
  blinker(500);
  blinker(500);
  blinker(500);
  blinker(50);
  blinker(50);
  blinker(50);
}

void error() {
  digitalWrite(err, HIGH);
  delay(3000);
  digitalWrite(err, LOW);
  delay(500);
}

bool isADCChannel(int channel) {
  if (channel < 8 && channel > -1) {
    return true;
  } else {
    return false;
  }
}

bool isDACChannel(int channel) {
  if (channel < 4 && channel > -1) {
    return true;
  } else {
    return false;
  }
}

int indexOfOperation(String operation) {
  for (int index = 0; index < Noperations; index++) {
    if (operations[index] == operation) {
      return index;
    }
  }
  return 0;
}

void waitDRDY() {
  int count = 0;
  while (digitalRead(drdy[0]) == HIGH && digitalRead(drdy[1]) == HIGH &&
         count < 2000) {
    count = count + 1;
    delay(1);
  }
}

void resetADC()  // Resets the ADC, and sets the range to default +-10 V
{
  for (int i = 0; i <= 1; i = i + 1) {
    SPI.beginTransaction(adcSettings);
    digitalWrite(data, HIGH);
    digitalWrite(reset[i], HIGH);
    digitalWrite(reset[i], LOW);
    digitalWrite(reset[i], HIGH);
    digitalWrite(adc_sync_pins[i], LOW);
    SPI.transfer(0x28);
    digitalWrite(adc_sync_pins[i], HIGH);
    digitalWrite(adc_sync_pins[i], LOW);
    SPI.transfer(0);
    digitalWrite(adc_sync_pins[i], HIGH);
    digitalWrite(adc_sync_pins[i], LOW);
    SPI.transfer(0x2A);
    digitalWrite(adc_sync_pins[i], HIGH);
    digitalWrite(adc_sync_pins[i], LOW);
    SPI.transfer(0);
    digitalWrite(adc_sync_pins[i], HIGH);
    SPI.endTransaction();
  }
}

void talkADC(std::vector<String> DB) {
  for (int i = 0; i <= 1; i = i + 1) {
    int comm;
    SPI.beginTransaction(adcSettings);
    digitalWrite(adc_sync_pins[i], LOW);
    comm = SPI.transfer(DB[1].toInt());
    digitalWrite(adc_sync_pins[i], HIGH);
    SPI.endTransaction();

    Serial.println(comm);
    Serial.flush();
  }
}

void writeADCConversionTime(std::vector<String> DB) {
  int adcChannel = DB[1].toInt();
  byte cr;

  byte fw = ((byte)(((DB[2].toFloat() * 6.144 - 249) / 128) + 0.5)) | 128;

  if (adcChannel >= 4) {
    adc_select = 1;
  } else {
    adc_select = 0;
  }
  SPI.beginTransaction(adcSettings);
  digitalWrite(adc_sync_pins[adc_select], LOW);
  SPI.transfer(0x30 + adcChannel);
  digitalWrite(adc_sync_pins[adc_select], HIGH);
  digitalWrite(adc_sync_pins[adc_select], LOW);
  SPI.transfer(fw);
  digitalWrite(adc_sync_pins[adc_select], HIGH);
  delayMicroseconds(100);
  digitalWrite(adc_sync_pins[adc_select], LOW);
  SPI.transfer(0x70 + adcChannel);
  digitalWrite(adc_sync_pins[adc_select], HIGH);
  digitalWrite(adc_sync_pins[adc_select], LOW);
  cr = SPI.transfer(0);  // Read back the CT register
  digitalWrite(adc_sync_pins[adc_select], HIGH);
  SPI.endTransaction();

  int convtime = ((int)(((cr & 127) * 128 + 249) / 6.144) + 0.5);
  Serial.println(convtime);
}

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

float twoByteToVoltage(byte DB1, byte DB2) {
  int decimal;
  float voltage;

  decimal = twoByteToInt(DB1, DB2);

  if (decimal <= 32767) {
    voltage = decimal * 10.0 / 32767;
  } else {
    voltage = -(65536 - decimal) * 10.0 / 32768;
  }
  return voltage;
}

void voltageToTwoByte(float voltage, byte *DB1, byte *DB2) {
  int decimal;
  if (voltage > 10 || voltage < -10) {
    *DB1 = 128;
    *DB2 = 0;
    error();
  } else if (voltage >= 0) {
    decimal = voltage * 32767 / 10;
  } else {
    decimal = voltage * 32768 / 10 + 65536;
  }
  intToTwoByte(decimal, DB1, DB2);
}

float getSingleReading(int adcchan) {
  Serial.flush();
  int statusbyte = 0;
  byte o2;
  byte o3;
  int ovr;
  int indchan;
  if (adcchan <= 7) {
    if (adcchan >= 4) {
      adc_select = 1;
      indchan = adcchan - 4;
    } else {
      adc_select = 0;
      indchan = adcchan;
    }
    SPI.beginTransaction(adcSettings);
    digitalWrite(adc_sync_pins[adc_select], LOW);
    SPI.transfer(0x38 + indchan);  // Indicates comm register to access mode
                                   // register with channel
    digitalWrite(adc_sync_pins[adc_select], HIGH);
    digitalWrite(adc_sync_pins[adc_select], LOW);
    SPI.transfer(0x48);  // Indicates mode register to start single convertion
                         // in dump mode
    digitalWrite(adc_sync_pins[adc_select], HIGH);
    waitDRDY();  // Waits until convertion finishes
    digitalWrite(adc_sync_pins[adc_select], LOW);
    SPI.transfer(
        0x48 +
        indchan);  // Indcates comm register to read data channel data register
    digitalWrite(adc_sync_pins[adc_select], HIGH);
    digitalWrite(adc_sync_pins[adc_select], LOW);
    statusbyte = SPI.transfer(0);  // Reads Channel 'ch' status
    digitalWrite(adc_sync_pins[adc_select], HIGH);
    digitalWrite(adc_sync_pins[adc_select], LOW);
    o2 = SPI.transfer(0);  // Reads first byte
    digitalWrite(adc_sync_pins[adc_select], HIGH);
    digitalWrite(adc_sync_pins[adc_select], LOW);
    o3 = SPI.transfer(0);  // Reads second byte
    digitalWrite(adc_sync_pins[adc_select], HIGH);
    SPI.endTransaction();

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
}

float readADC(byte DB) {
  int adcChannel = DB;
  if (adcChannel < 0 || adcChannel > 7) {
    Serial.println("Invalid ADC channel. (Valid Channels: 0,1,2,3,4,5,6,7)");
  } else {
    return getSingleReading(adcChannel);
  }
}

int threeByteToInt(byte DB1, byte DB2,
                   byte DB3)  // This gives a 16 bit integer (between +/- 2^16)
{
  return ((int)(((((DB1 & 15) << 8) | DB2) << 8) | DB3));
}

void intToThreeBytes(int decimal, byte *DB1, byte *DB2, byte *DB3) {
  *DB1 = (byte)((decimal >> 16) | 16);
  *DB2 = (byte)((decimal >> 8) & 255);
  *DB3 = (byte)(decimal & 255);
}



void readingRampAvg(int adcchan, byte b1, byte b2, byte *o1, byte *o2,
                    int count, int nReadings) {
  Serial.flush();
  int statusbyte = 0;
  int ovr;
  byte db1;
  byte db2;
  float sum = 0;
  float avg;
  bool toSend = true;
  int indchan;
  if (adcchan <= 7) {
    if (adcchan >= 4) {
      adc_select = 1;
      indchan = adcchan - 4;
    } else {
      adc_select = 0;
      indchan = adcchan;
    }
    for (int i = 1; i <= nReadings; i++) {
      SPI.beginTransaction(adcSettings);
      digitalWrite(adc_sync_pins[adc_select], LOW);
      SPI.transfer(0x38 + indchan);  // Indicates comm register to access mode
                                     // register with channel
      digitalWrite(adc_sync_pins[adc_select], HIGH);
      digitalWrite(adc_sync_pins[adc_select], LOW);
      SPI.transfer(0x48);  // Indicates mode register to start single convertion
                           // in dump mode
      digitalWrite(adc_sync_pins[adc_select], HIGH);
      if (count > 0 && toSend) {
        Serial.write(
            b1);  // Sends previous reading while it is waiting for new reading
        Serial.write(b2);
        toSend = false;
      }
      waitDRDY();  // Waits until convertion finishes
      digitalWrite(adc_sync_pins[adc_select], LOW);
      SPI.transfer(0x48 + indchan);  // Indcates comm register to read data
                                     // channel data register
      digitalWrite(adc_sync_pins[adc_select], HIGH);
      digitalWrite(adc_sync_pins[adc_select], LOW);
      statusbyte = SPI.transfer(0);  // Reads Channel 'ch' status
      digitalWrite(adc_sync_pins[adc_select], HIGH);
      digitalWrite(adc_sync_pins[adc_select], LOW);
      db1 = SPI.transfer(0);  // Reads first byte
      digitalWrite(adc_sync_pins[adc_select], HIGH);
      digitalWrite(adc_sync_pins[adc_select], LOW);
      db2 = SPI.transfer(0);  // Reads second byte
      digitalWrite(adc_sync_pins[adc_select], HIGH);
      SPI.endTransaction();
      ovr = statusbyte & 1;
      if (ovr) {
        break;
      }
      int decimal = twoByteToInt(db1, db2);
      float voltage = map2(decimal, 0, 65536, -10.0, 10.0);
      sum += voltage;
    }
    if (ovr) {
      *o1 = 128;
      *o2 = 0;
    } else {
      avg = sum / nReadings;
      int decimal = map2(avg, -10.0, 10.0, 0, 65536);
      intToTwoByte(decimal, &db1, &db2);
      *o1 = db1;
      *o2 = db2;
    }
  }
}

void rampRead(byte DB, byte b1, byte b2, byte *o1, byte *o2, int count,
              int nReadings) {
  int adcChannel = DB;
  switch (adcChannel) {
    case 0:
      readingRampAvg(0, b1, b2, o1, o2, count, nReadings);
      break;

    case 1:
      readingRampAvg(1, b1, b2, o1, o2, count, nReadings);
      break;

    case 2:
      readingRampAvg(2, b1, b2, o1, o2, count, nReadings);
      break;

    case 3:
      readingRampAvg(3, b1, b2, o1, o2, count, nReadings);
      break;

    case 4:
      readingRampAvg(4, b1, b2, o1, o2, count, nReadings);
      break;

    case 5:
      readingRampAvg(5, b1, b2, o1, o2, count, nReadings);
      break;

    case 6:
      readingRampAvg(6, b1, b2, o1, o2, count, nReadings);
      break;

    case 7:
      readingRampAvg(7, b1, b2, o1, o2, count, nReadings);
      break;
    default:
      break;
  }
}


// void bufferRamp(std::vector<String> DB)
// {
//   String channelsDAC = DB[1];
//   int NchannelsDAC = channelsDAC.length();
//   String channelsADC = DB[2];
//   int NchannelsADC = channelsADC.length();
//   std::vector<float> vi;
//   std::vector<float> vf;

//   for (int i = 3; i < NchannelsDAC + 3; i++)
//   {
//     vi.push_back(DB[i].toFloat());
//     vf.push_back(DB[i + NchannelsDAC].toFloat());
//   }
//   int nSteps = (DB[NchannelsDAC * 2 + 3].toInt());
//   byte b1;
//   byte b2;
//   int count = 0;
//   int overrange = 0;
//   for (int i = 0; i < NchannelsDAC; i++)
//   {
//     if (vi[i] < LB[i] || vi[i] > UB[i] || vf[i] < LB[i] || vf[i] > UB[i]){
//       overrange = 1;
//     }
//   }
//   if (overrange == 1){
//     Serial.print("VOLTAGE_OVERRANGE");
//     Serial.print(LB[DB[1].toInt()],6);
//     Serial.println(UB[DB[1].toInt()],6);
//   }else{
//     for (int j = 0; j < nSteps; j++)
//     {
//       for (int i = 0; i < NchannelsDAC; i++)
//       {
//         float v;
//         v = vi[i] + (vf[i] - vi[i]) * j / (nSteps - 1);
//         v = writeDAC_buffer(channelsDAC[i] - '0', v);
//         DAC_Voltage[channelsDAC[i] - '0'] = v;
//       }
//       digitalWrite(ldac, LOW);
//       digitalWrite(ldac, HIGH);
//       if (delayUnit)
//       {
//         delay(DB[NchannelsDAC * 2 + 4].toInt());
//       }
//       else
//       {
//         delayMicroseconds(DB[NchannelsDAC * 2 + 4].toInt());
//       }
//       for (int i = 0; i < NchannelsADC; i++)
//       {
//         rampRead(channelsADC[i] - '0', b1, b2, &b1, &b2, count,
//         DB[NchannelsDAC * 2 + 5].toInt()); count += 1;
//       }
//       if (Serial.available())
//       {
//         std::vector<String> comm;
//         comm = query_serial();
//         if (comm[0] == "STOP")
//         {
//           break;
//         }
//       }
//     }
//     Serial.write(b1);
//     Serial.write(b2);
//     Serial.println("BUFFER_RAMP_FINISHED");
//   }

// }

// int bufferRampDis(std::vector<String> DB)
// {
//   String channelsDAC = DB[1];
//   int NchannelsDAC = channelsDAC.length();
//   String channelsADC = DB[2];
//   int NchannelsADC = channelsADC.length();
//   int nAdcSteps = DB[NchannelsDAC * 2 + 6].toInt();
//   int nSteps = (DB[NchannelsDAC * 2 + 3].toInt());

//   if (nAdcSteps > nSteps)
//   {
//     Serial.println("nAdcSteps must be smaller or equal to nSteps");
//     return 0;
//   }

//   std::vector<float> vi;
//   std::vector<float> vf;
//   for (int i = 3; i < NchannelsDAC + 3; i++)
//   {
//     vi.push_back(DB[i].toFloat());
//     vf.push_back(DB[i + NchannelsDAC].toFloat());
//   }
//   byte b1;
//   byte b2;
//   int count = 0;
//   int adcCount = 0;
//   int overrange = 0;
//   for (int i = 0; i < NchannelsDAC; i++)
//   {
//     if (vi[i] < LB[i] || vi[i] > UB[i] || vf[i] < LB[i] || vf[i] > UB[i]){
//       overrange = 1;
//     }
//   }
//   if (overrange == 1){
//     Serial.print("VOLTAGE_OVERRANGE");
//     Serial.print(LB[DB[1].toInt()],6);
//     Serial.println(UB[DB[1].toInt()],6);
//   }else{
//     for (int j = 0; j < nSteps; j++)
//     {
//       for (int i = 0; i < NchannelsDAC; i++)
//       {
//         float v;
//         v = vi[i] + (vf[i] - vi[i]) * j / (nSteps - 1);
//         v = writeDAC_buffer(channelsDAC[i] - '0', v);
//         DAC_Voltage[channelsDAC[i] - '0'] = v;
//       }
//       digitalWrite(ldac, LOW);
//       digitalWrite(ldac, HIGH);
//       if (delayUnit)
//       {
//         delay(DB[NchannelsDAC * 2 + 4].toInt());
//       }
//       else
//       {
//         delayMicroseconds(DB[NchannelsDAC * 2 + 4].toInt());
//       }
//       if (j == round(adcCount*float(nSteps-1)/float(nAdcSteps-1)))
//       {
//         for (int i = 0; i < NchannelsADC; i++)
//         {
//           rampRead(channelsADC[i] - '0', b1, b2, &b1, &b2, count,
//           DB[NchannelsDAC * 2 + 5].toInt()); count += 1;
//         }
//       adcCount += 1;
//       }
//       if (Serial.available())
//       {
//         std::vector<String> comm;
//         comm = query_serial();
//         if (comm[0] == "STOP")
//         {
//           break;
//         }
//       }
//     }
//     Serial.write(b1);
//     Serial.write(b2);
//     Serial.println("BUFFER_RAMP_FINISHED");
//     return 0;
//   }
// }

// void autoRamp1(std::vector<String> DB)
// {
//   float v1 = DB[2].toFloat();
//   float v2 = DB[3].toFloat();
//   float v;
//   int nSteps = DB[4].toInt();
//   int dacChannel = DB[1].toInt();
//   if (v1 < LB[dacChannel] || v1 > UB[dacChannel] || v2 < LB[dacChannel] || v2
//   > UB[dacChannel]){
//     Serial.println("VOLTAGE_OVERRANGE");
//   }else{
//     for (int j = 0; j < nSteps; j++)
//     {
//       int timer = micros();
//       v = writeDAC(dacChannel, v1 + (v2 - v1)*j / (nSteps - 1));
//       DAC_Voltage[dacChannel] = v;
//       while (micros() <= timer + DB[5].toInt());
//     }
//   }
// }

// void autoRamp2(std::vector<String> DB)
// {
//   float vi1 = DB[3].toFloat();
//   float vi2 = DB[4].toFloat();
//   float vf1 = DB[5].toFloat();
//   float vf2 = DB[6].toFloat();
//   float v1;
//   float v2;
//   int nSteps = DB[7].toInt();
//   byte b1;
//   byte b2;
//   int dacChannel1 = DB[1].toInt();
//   int dacChannel2 = DB[2].toInt();
//   if (vi1 < LB[dacChannel1] || vi1 > UB[dacChannel1] || vi2 < LB[dacChannel2]
//   || vi2 > UB[dacChannel2] || vf1 < LB[dacChannel1] || vf1 > UB[dacChannel1]
//   || vf2 < LB[dacChannel2] || vf2 > UB[dacChannel2]){
//     Serial.println("VOLTAGE_OVERRANGE");
//   }else{
//     for (int j = 0; j < nSteps; j++)
//     {
//       int timer = micros();
//       v1 = writeDAC(dacChannel1, vi1 + (vf1 - vi1)*j / (nSteps - 1));
//       v2 = writeDAC(dacChannel2, vi2 + (vf2 - vi2)*j / (nSteps - 1));
//       DAC_Voltage[dacChannel1] = v1;
//       DAC_Voltage[dacChannel2] = v2;
//       while (micros() <= timer + DB[8].toInt());
//     }
//   }

// }

// void readDAC(int DACChannel) // Does not work; always reads 0 from register
// {
//   byte o1;
//   byte o2;
//   byte o3;
//   float voltage;

//   SPI.beginTransaction(dacSettings);
//   digitalWrite(dac_cs_pins[DACChannel], LOW);
//   SPI.transfer(144); // send command byte to DAC
//   SPI.transfer(0); // MS data bits, DAC2
//   SPI.transfer(0); //LS 8 data bits, DAC2
//   digitalWrite(dac_cs_pins[DACChannel], HIGH);
//   delayMicroseconds(2);
//   digitalWrite(dac_cs_pins[DACChannel], LOW);
//   o1 = SPI.transfer(0x00);
//   o2 = SPI.transfer(0x00);
//   o3 = SPI.transfer(0x00);
//   digitalWrite(dac_cs_pins[DACChannel], HIGH);
//   SPI.endTransaction();

//   voltage = threeByteToVoltage(o1, o2, o3);
//   voltage = (voltage + OS[0]) * GE[0];
//   Serial.println(voltage, 5);
// }

void ID() { Serial.println("DAC-ADC_AD7734-AD5791"); }

void RDY() { Serial.println("READY"); }

void setUnit(int unit) {
  if (unit == 0) {
    delayUnit = 0;
    Serial.println("Delay unit set to microseconds");
  } else if (unit == 1) {
    delayUnit = 1;
    Serial.println("Delay unit set to miliseconds");
  } else {
    Serial.println("Unit should be 0 (microseconds) or 1 (miliseconds)");
  }
}

void adc_checksum(std::vector<String> DB) {
  byte b1;
  byte b2;
  int i = DB[1].toInt();

  SPI.beginTransaction(adcSettings);
  digitalWrite(adc_sync_pins[i], LOW);
  SPI.transfer(0x00);  // Indicates comm register to access mode register
  digitalWrite(adc_sync_pins[i], HIGH);
  digitalWrite(adc_sync_pins[i], LOW);
  SPI.transfer(0xFF);  // Indicates comm register to access mode register
  digitalWrite(adc_sync_pins[i], HIGH);
  digitalWrite(adc_sync_pins[i], LOW);
  SPI.transfer(0xFF);  // Indicates comm register to access mode register
  digitalWrite(adc_sync_pins[i], HIGH);
  digitalWrite(adc_sync_pins[i], LOW);
  SPI.transfer(0xFF);  // Indicates comm register to access mode register
  digitalWrite(adc_sync_pins[i], HIGH);
  digitalWrite(adc_sync_pins[i], LOW);
  SPI.transfer(0xFF);  // Indicates comm register to access mode register
  digitalWrite(adc_sync_pins[i], HIGH);

  digitalWrite(adc_sync_pins[i], LOW);
  SPI.transfer(0x05);  // Indicates comm register to access mode register
  digitalWrite(adc_sync_pins[i], HIGH);
  digitalWrite(adc_sync_pins[i], LOW);
  SPI.transfer(0x00);  // Write to checksum register
  digitalWrite(adc_sync_pins[i], HIGH);
  digitalWrite(adc_sync_pins[i], LOW);
  SPI.transfer(0x00);  // Write to checksum register
  digitalWrite(adc_sync_pins[i], HIGH);

  digitalWrite(adc_sync_pins[i], LOW);
  SPI.transfer(0x45);  // Indicates comm register to access mode register
  digitalWrite(adc_sync_pins[i], HIGH);
  digitalWrite(adc_sync_pins[i], LOW);
  b1 = SPI.transfer(0x00);  // Read checksum register
  digitalWrite(adc_sync_pins[i], HIGH);
  digitalWrite(adc_sync_pins[i], LOW);
  b2 = SPI.transfer(0x00);  // Read to checksum register
  digitalWrite(adc_sync_pins[i], HIGH);
  SPI.endTransaction();

  Serial.println((b1 << 8) | b2);
  Serial.flush();
}

void adc_zero_scale_cal()

{
  for (int i = 0; i <= 1; i++) {
    SPI.beginTransaction(adcSettings);
    digitalWrite(adc_sync_pins[i], LOW);
    SPI.transfer(0x38);  // Indicates comm register to access mode register
    digitalWrite(adc_sync_pins[i], HIGH);
    digitalWrite(adc_sync_pins[i], LOW);
    SPI.transfer(0x80);
    digitalWrite(adc_sync_pins[i], HIGH);
    SPI.endTransaction();
    waitDRDY();
  }
}

void adc_ch_zero_scale_cal() {
  for (int i = 0; i <= 1; i++) {
    SPI.beginTransaction(adcSettings);
    digitalWrite(adc_sync_pins[i], LOW);
    SPI.transfer(0x38);  // Indicates comm register to access mode register
    digitalWrite(adc_sync_pins[i], HIGH);
    digitalWrite(adc_sync_pins[i], LOW);
    SPI.transfer(0xC0);
    digitalWrite(adc_sync_pins[i], HIGH);
    waitDRDY();

    digitalWrite(adc_sync_pins[i], LOW);
    SPI.transfer(0x38 + 1);  // Indicates comm register to access mode register
    digitalWrite(adc_sync_pins[i], HIGH);
    digitalWrite(adc_sync_pins[i], LOW);
    SPI.transfer(0xC0);
    digitalWrite(adc_sync_pins[i], HIGH);
    waitDRDY();

    digitalWrite(adc_sync_pins[i], LOW);
    SPI.transfer(0x38 + 2);  // Indicates comm register to access mode register
    digitalWrite(adc_sync_pins[i], HIGH);
    digitalWrite(adc_sync_pins[i], LOW);
    SPI.transfer(0xC0);
    digitalWrite(adc_sync_pins[i], HIGH);
    waitDRDY();

    digitalWrite(adc_sync_pins[i], LOW);
    SPI.transfer(0x38 + 3);  // Indicates comm register to access mode register
    digitalWrite(adc_sync_pins[i], HIGH);
    digitalWrite(adc_sync_pins[i], LOW);
    SPI.transfer(0xC0);
    digitalWrite(adc_sync_pins[i], HIGH);
    SPI.endTransaction();
    waitDRDY();
  }
}

void adc_ch_full_scale_cal() {
  for (int i = 0; i <= 1; i = i + 1) {
    SPI.beginTransaction(adcSettings);
    digitalWrite(adc_sync_pins[i], LOW);
    SPI.transfer(0x38);  // Indicates comm register to access mode register
    digitalWrite(adc_sync_pins[i], HIGH);
    digitalWrite(adc_sync_pins[i], LOW);
    SPI.transfer(0xE0);
    digitalWrite(adc_sync_pins[i], HIGH);
    waitDRDY();

    digitalWrite(adc_sync_pins[i], LOW);
    SPI.transfer(0x38 + 1);  // Indicates comm register to access mode register
    digitalWrite(adc_sync_pins[i], HIGH);
    digitalWrite(adc_sync_pins[i], LOW);
    SPI.transfer(0xE0);
    digitalWrite(adc_sync_pins[i], HIGH);
    waitDRDY();

    digitalWrite(adc_sync_pins[i], LOW);
    SPI.transfer(0x38 + 2);  // Indicates comm register to access mode register
    digitalWrite(adc_sync_pins[i], HIGH);
    digitalWrite(adc_sync_pins[i], LOW);
    SPI.transfer(0xE0);
    digitalWrite(adc_sync_pins[i], HIGH);
    waitDRDY();

    digitalWrite(adc_sync_pins[i], LOW);
    SPI.transfer(0x38 + 3);  // Indicates comm register to access mode register
    digitalWrite(adc_sync_pins[i], HIGH);
    digitalWrite(adc_sync_pins[i], LOW);
    SPI.transfer(0xE0);
    digitalWrite(adc_sync_pins[i], HIGH);
    SPI.endTransaction();
    waitDRDY();
  }
}

// void dac_ch_cal()
// {
//   for (int i = 0; i <= 3; i++)
//   {
//     OS[i] = 0; // offset error
//     GE[i] = 1; // gain error
//   }
//   //set dacs to zero volts
//   for (int i = 0; i <= 3; i++)
//   {
//     dacDataSend(dac_cs_pins[i], 0);
//   }
//   delay(1);
//   //reads the offset of each channel
//   for (int i = 0; i <= 3; i++)
//   {
//     OS[i] = readADC(i);
//   }

//   //set dacs to 9 volts
//   float ifs = 9;
//   for (int i = 0; i <= 3; i++)
//   {
//     dacDataSend(dac_cs_pins[i], ifs);
//   }
//   delay(1);
//   //reads each channel and calculates the gain error
//   for (int i = 0; i <= 3; i++)
//   {
//     float nfs;
//     nfs = readADC(i);
//     GE[i] = (nfs - OS[i]) / ifs;
//   }
// }

// void changeosg(std::vector<String> DB)
// {
//   for (int i = 1; i < DB.size(); i++)
//   {
//     if (i < 5) {
//       union {
//         float f;
//         unsigned char b[4];
//       } offset;

//       OS[i-1] = DB[i].toFloat();
//       offset.f = OS[i-1];
//       // dueFlashStorage.write(4*i+256, offset.b, 4);
//     }
//     else {
//       union {
//         float f;
//         unsigned char b[4];
//       } gainError;

//       GE[i-5] = DB[i].toFloat();
//       gainError.f = GE[i-5];
//       // dueFlashStorage.write(4*i+256, gainError.b, 4);
//     }
//   }
// }

void debug() {
  delay(3000);
  delay(3000);
}

void router(std::vector<String> DB) {
  float v;
  int code;
  int channel;

  int operation = indexOfOperation(DB[0]);
  switch (operation) {
    case 0:
      Serial.println("NOP");
      break;

    case 1:
      digitalWrite(data, HIGH);
      dac_controller.initialize();
      Serial.println("INITIALIZATION COMPLETE");
      digitalWrite(data, LOW);
      break;

    case 2: {
      channel = DB[1].toInt();
      float lowerBound = dac_controller.getLowerBound(0, channel);
      float upperBound = dac_controller.getUpperBound(0, channel);
      if (!isDACChannel(channel)) {
        Serial.println("INVALID DAC CHANNEL.");
      } else if (DB[2].toFloat() <= lowerBound ||
                 DB[2].toFloat() >= upperBound) {
        Serial.print("VOLTAGE_OVERRANGE: ");
        Serial.print(lowerBound, 6);
        Serial.print(" TO ");
        Serial.println(upperBound, 6);
        break;
      } else {
        v = dac_controller.setVoltage(0, DB[1].toInt(), DB[2].toFloat());
        Serial.print("DAC ");
        Serial.print(DB[1]);
        Serial.print(" UPDATED TO ");
        Serial.print(v, 6);
        Serial.println(" V");
        break;
      }
    }
    case 3:
      code = DB[2].toInt();
      channel = DB[1].toInt();
      if (!isDACChannel(channel)) {
        Serial.println("INVALID DAC CHANNEL.");
      } else if (code < 0 || code > 1048576) {
        Serial.print("CODE OVERRANGE (0-1048576)");
      } else {
        dac_controller.sendCode(0, channel, code);
        Serial.print("DAC ");
        Serial.print(channel);
        Serial.print(" CODE UPDATED TO ");
        Serial.print(code);
        Serial.println(".");
      }
      break;

    case 4:
      channel = DB[1].toInt();
      if (!isDACChannel(channel)) {
        Serial.println("INVALID DAC CHANNEL.");
      } else {
        Serial.println(dac_controller.getVoltage(0, channel), 6);
      }
      break;

    case 5:  // Read ADC
      digitalWrite(data, HIGH);
      v = readADC(DB[1].toInt());
      Serial.println(v, 4);
      digitalWrite(data, LOW);
      break;

    case 6:
      Serial.println("UNIMPLEMENTED");
      // digitalWrite(data, HIGH);
      // autoRamp1(DB);
      // Serial.println("RAMP_FINISHED");
      // digitalWrite(data, LOW);
      break;

    case 7:
      Serial.println("UNIMPLEMENTED");
      // digitalWrite(data, HIGH);
      // autoRamp2(DB);
      // Serial.println("RAMP_FINISHED");
      // digitalWrite(data, LOW);
      break;

    case 8:  // Autoramp
      Serial.println("UNIMPLEMENTED");
      // digitalWrite(data, HIGH);
      // bufferRamp(DB);
      // digitalWrite(data, LOW);
      break;

    case 9:
      Serial.println("UNIMPLEMENTED");
      // digitalWrite(data, HIGH);
      // bufferRampDis(DB);
      // digitalWrite(data, LOW);
      break;

    case 10:
      resetADC();
      break;

    case 11:
      talkADC(DB);
      break;

    case 12:  // Write conversion time registers
      writeADCConversionTime(DB);
      break;

    case 13:  // ID
      ID();
      break;

    case 14:
      RDY();
      break;

    case 15:
      Serial.println(delayUnit);
      break;

    case 16:
      setUnit(DB[1].toInt());  // 0 = microseconds 1 = miliseconds
      break;

    case 17:
      adc_zero_scale_cal();
      Serial.println("CALIBRATION_FINISHED");
      break;

    case 18:
      adc_ch_zero_scale_cal();
      Serial.println("CALIBRATION_FINISHED");
      break;

    case 19:
      adc_ch_full_scale_cal();
      Serial.println("CALIBRATION_FINISHED");
      break;

    case 20:
      // dac_ch_cal();
      // Serial.println("CALIBRATION_FINISHED");
      Serial.println("UNIMPLEMENTED");

    case 21:
      dac_controller.setFullScale(0, DB[1].toInt());
      Serial.println("FULL_SCALE_UPDATED");
      break;

    case 22:
      for (DACBoard *board : dac_controller.getAllBoards()) {
        for (int i = 0; i < board->getNumChannels(); i++) {
          Serial.println(board->getOffsetError(i), 6);
        }
      }

      for (DACBoard *board : dac_controller.getAllBoards()) {
        for (int i = 0; i < board->getNumChannels(); i++) {
          Serial.println(board->getGainError(i), 6);
        }
      }
      break;

    case 23:
      Serial.println("UNIMPLEMENTED");
      // changeosg(DB);
      // for (DACBoard* board : dac_controller.getAllBoards()) {
      // for (int i = 0; i < board->getNumChannels(); i++) {
      // Serial.println(board->getOffsetError(i),6);
      // }
      // }

      // for (DACBoard* board : dac_controller.getAllBoards()) {
      // for (int i = 0; i < board->getNumChannels(); i++) {
      // Serial.println(board->getGainError(i),6);
      // }
      // }
      break;

    case 24:
      adc_checksum(DB);
      break;

    case 25:
      Serial.println("DA20_16_08");
      break;

    default:
      break;
  }
}

void loop() {
  Serial.flush();
  std::vector<String> comm;

  if (Serial.available()) {
    comm = query_serial();
    router(comm);
  }
}