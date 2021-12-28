#ifndef digpot_h
#define digpot_h

#include <stdint.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/gpio.h>
#include <digio.h>

//Class for transmitting an value to the DIGITAL SPI POT
//On the VCU V1 there are 2 POTS on SPI3
class digpot
{ 
    public:
        
        digpot(uint32_t spiport, DigIo cspin );

    void SetPotStep(uint8_t step);

    uint8_t GetCurrentStep();

    private:
        //saves the SPI port used
        uint32_t _spiport;

        //internal step value
        uint8_t _mystep;
   
        //contains the CSPIN which needs tobe toggled
        DigIo _cspin;
};

#endif /* digpot_h */