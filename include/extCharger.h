
#ifndef extCharger_h
#define extCharger_h

/*  This library supports the various opensource tesla charger controllers e.g. PCS , Gen2/3 etc.

*/

#include <stdint.h>
#include "digio.h"
#include "iomatrix.h"
#include "chargerhw.h"

class extCharger: public Chargerhw
{

public:
extCharger();
bool ControlCharge(bool RunCh);


private:

};

#endif /* extCharger_h */

