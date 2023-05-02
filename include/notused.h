
#ifndef notused_h
#define notused_h

/*  Dummy library for whrn no charge interface is used.

*/

#include <stdint.h>
#include "chargerint.h"

class notused: public Chargerint
{

public:
bool ACRequest(bool RunCh){return RunCh;};

private:

};

#endif /* notused_h */
