#ifndef DIGIO_H_INCLUDED
#define DIGIO_H_INCLUDED

#include <libopencm3/stm32/gpio.h>
#include "digio_prj.h"

namespace PinMode {
   enum PinMode
   {
       INPUT_PD,
       INPUT_PU,
       INPUT_FLT,
       INPUT_AIN,
       OUTPUT,
       LAST
   };
}

class DigIo
{
public:
   #define DIG_IO_ENTRY(name, port, pin, mode) static DigIo name;
   DIG_IO_LIST
   #undef DIG_IO_ENTRY

   /** Map GPIO pin object to hardware pin.
    * @param[in] port port to use for this pin
    * @param[in] pin port-pin to use for this pin
    * @param[in] mode pinmode to use
    */
   void Configure(uint32_t port, uint16_t pin, PinMode::PinMode pinMode);

   /**
   * Get pin value
   *
   * @param[in] io pin index
   * @return pin value
   */
   bool Get() { return gpio_get(_port, _pin) > 0; }

   /**
   * Set pin high
   *
   * @param[in] io pin index
   */
   void Set() { gpio_set(_port, _pin); }

   /**
   * Set pin low
   *
   * @param[in] io pin index
   */
   void Clear() { gpio_clear(_port, _pin); }

   /**
   * Toggle pin
   *
   * @param[in] io pin index
   */
   void Toggle() { gpio_toggle(_port, _pin); }

private:
   uint32_t _port;
   uint16_t _pin;
};
//Configure all digio objects from the given list
#define DIG_IO_ENTRY(name, port, pin, mode) DigIo::name.Configure(port, pin, mode);
#define DIG_IO_CONFIGURE(l) l

#endif // DIGIO_H_INCLUDED
