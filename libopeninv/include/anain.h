#ifndef ANAIO_H_INCLUDED
#define ANAIO_H_INCLUDED

#include <stdint.h>
#include "anain_prj.h"


class AnaIn
{
public:
   AnaIn(int chan): firstValue(&values[chan]) {}

   #define ANA_IN_ENTRY(name, port, pin) static AnaIn name;
   ANA_IN_LIST
   #undef ANA_IN_ENTRY

   #define ANA_IN_ENTRY(name, port, pin) +1
   static const int ANA_IN_COUNT = ANA_IN_LIST;
   #undef ANA_IN_ENTRY

   struct AnaInfo
   {
      uint32_t port;
      uint16_t pin;
   };

   static void Start();
   void Configure(uint32_t port, uint8_t pin);
   uint16_t Get();
   uint16_t GetIndex() { return firstValue - values; }

private:
   static uint16_t values[];
   static uint8_t channel_array[];

   static uint8_t AdcChFromPort(uint32_t command_port, int command_bit);
   static int median3(int a, int b, int c);

   uint16_t* const firstValue;
};

//Configure all AnaIn objects from the given list
#define ANA_IN_ENTRY(name, port, pin) AnaIn::name.Configure(port, pin);
#define ANA_IN_CONFIGURE(l) l

#endif // ANAIO_H_INCLUDED
