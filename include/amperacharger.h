
#ifndef amperacharger_h
#define amperacharger_h

/*  TODO: Add can info for control of Chevy Volt , Opel/Vauxhaul Ampera charger here.

*/

#include <stdint.h>
#include "my_fp.h"
#include "chargerhw.h"

class amperaCharger: public Chargerhw
{
public:
  bool ControlCharge(bool RunCh, bool ACreq);
};

#endif /* amperacharger_h */



