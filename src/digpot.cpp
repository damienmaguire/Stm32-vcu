#include <digpot.h>

//Class constructor
digpot::digpot(uint32_t spiport, DigIo cspin){
    this->_spiport  = spiport;
    this->_mystep   = 0;
    this->_cspin    = cspin;
   
    cspin.Set();
 }

 void digpot::SetPotStep(uint8_t step){

      //now the ic starts clocking in the data...
      this->_cspin.Clear();

      //delay a tiny bit so the device can start listening  
       __asm__("nop");      
      
      spi_write(this->_spiport, step);  
      
      /* Wait for last transfer finished. */
      while (!(SPI_SR(this->_spiport) & SPI_SR_TXE));

      //Delay tiny bit for device to fetch data... 
      for (int i=0;i<100;i++){
          __asm__("nop");      
      }
    
      //we raise the cs now the value becomes active  
      this->_cspin.Set();

      //update internal placeholder  
      this->_mystep=step;

 }

//Returns the last transmitted step value to the SPI DIG POT
uint8_t digpot::GetCurrentStep(){
    return this->_mystep;
}