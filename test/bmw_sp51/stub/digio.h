// Host stub of libopeninv digio.h: the two contactor outputs the SP51
// class follows, as plain flags set by the test.
#pragma once
class DigIo {
public:
  bool Get() { return on; }
  void Set() { on = true; }
  void Clear() { on = false; }
  bool on = false;
  static DigIo prec_out;
  static DigIo dcsw_out;
};
