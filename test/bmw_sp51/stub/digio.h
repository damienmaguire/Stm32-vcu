// Host stub of libopeninv digio.h for the BMW_SP51 host check
// (test/bmw_sp51). Only the two contactor outputs the module reads.
#pragma once
class DigIo {
public:
  bool Get() { return on; }
  void Set() { on = true; }
  void Clear() { on = false; }
  static DigIo prec_out;
  static DigIo dcsw_out;

private:
  bool on = false;
};
