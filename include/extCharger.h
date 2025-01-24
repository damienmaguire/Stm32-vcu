
#ifndef EXTCHARGER_H
#define EXTCHARGER_H

/*  This library supports the various opensource tesla charger controllers e.g. PCS , Gen2/3 etc.

*/

#include <stdint.h>
#include "digio.h"
#include "iomatrix.h"
#include "chargerhw.h"

class extCharger: public Chargerhw
{
  public:
    bool ControlCharge(bool RunCh, bool ACReq);
};

#endif /* EXTCHARGER_H */
