#include "digipot.h"


void DigiPot::SetPot1Step() {
    DigIo::pot1_cs.Clear();
    spi_xfer(SPI3, Param::GetInt(Param::DigiPot1Step));
    DigIo::pot1_cs.Set();
}

void DigiPot::SetPot2Step() {
    DigIo::pot2_cs.Clear();
    spi_xfer(SPI3, Param::GetInt(Param::DigiPot2Step));
    DigIo::pot2_cs.Set();
}