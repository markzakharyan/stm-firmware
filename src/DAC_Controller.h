#include <vector>

#include "DACBoard.h"

class DACController {
 private:
  std::vector<DACBoard*> dac_boards;

 public:
  float DAC_FULL_SCALE = DEFAULT_DAC_FULL_SCALE;
  DACController() {}

  ~DACController() {
    for (auto dac : dac_boards) {
      delete dac;
    }
  }

  void addBoard(const int* cs_pins, int ldac_pin) {
    DACBoard* new_board = new DACBoard(cs_pins, ldac_pin);
    dac_boards.push_back(new_board);
  }

  void initialize() {
    for (auto dac : dac_boards) {
      dac->initialize();
    }
  }

  float setVoltage(int board, int channel, float voltage) {
    if (board < 0 || board >= dac_boards.size()) {
      return -1;  // Invalid board
    }
    return dac_boards[board]->setVoltage(channel, voltage);
  }

  float setVoltageBuffer(int board, int channel, float voltage) {
    if (board < 0 || board >= dac_boards.size()) {
      return -1;  // Invalid board
    }
    return dac_boards[board]->setVoltageBuffer(channel, voltage);
  }

  float getVoltage(int board, int channel) {
    if (board < 0 || board >= dac_boards.size()) {
      return -1;  // Invalid board
    }
    return dac_boards[board]->getVoltage(channel);
  }

  void setCalibration(int board, int channel, float offset, float gain) {
    if (board < 0 || board >= dac_boards.size()) {
      return;  // Invalid board
    }
    dac_boards[board]->setCalibration(channel, offset, gain);
  }

  void setFullScale(int board, float full_scale) {
    if (board < 0 || board >= dac_boards.size()) {
      return;  // Invalid board
    }
    dac_boards[board]->setFullScale(full_scale);
  }

  float getLowerBound(int board, int channel) {
    if (board < 0 || board >= dac_boards.size()) {
      return 0.0;  // Invalid board
    }
    return dac_boards[board]->getLowerBound(channel);
  }

  float getUpperBound(int board, int channel) {
    if (board < 0 || board >= dac_boards.size()) {
      return 0.0;  // Invalid board
    }
    return dac_boards[board]->getUpperBound(channel);
  }

  DACBoard* getBoard(int board) {
    if (board < 0 || board >= dac_boards.size()) {
      return nullptr;  // Invalid board
    }
    return dac_boards[board];
  }

  std::vector<DACBoard*> getAllBoards() { return dac_boards; }

  void sendCode(int board, int channel, int code) {
    if (board < 0 || board >= dac_boards.size()) {
      return;  // Invalid board
    }
    dac_boards[board]->sendCode(channel, code);
  }
};