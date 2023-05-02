
#ifndef amperaCharger_h
#define amperaCharger_h

/*  TODO: Add can info for control of Chevy Volt , Opel/Vauxhaul Ampera charger here.

*/

#include <stdint.h>
#include "my_fp.h"
#include "params.h"
#include "chargerhw.h"

class amperaCharger: public Chargerhw
{

public:
void DecodeCAN(int id, uint32_t data[2]);
void Task100Ms();
bool ControlCharge(bool RunCh, bool ACReq);
void SetCanInterface(CanHardware* c);


private:

};

#endif /* amperaCharger_h */


