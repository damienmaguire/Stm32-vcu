// Host stub: no CAN hardware; the host check logs what is sent.
#pragma once
#include <stdint.h>
class CanHardware {
public:
  virtual ~CanHardware() {}
  void Send(uint32_t canId, uint8_t data[8], uint8_t len,
            bool forceExt = false) {
    Send(canId, (uint32_t *)data, len, forceExt);
  }
  virtual void Send(uint32_t canId, uint32_t data[2], uint8_t len,
                    bool forceExt = false) = 0;
  virtual bool RegisterUserMessage(uint32_t canId, uint32_t mask = 0) = 0;
};
