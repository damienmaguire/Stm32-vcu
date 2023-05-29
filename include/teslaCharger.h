
#ifndef teslaCharger_h
#define teslsCharger_h

/*  This library supports the various opensource tesla charger controllers e.g. PCS , Gen2/3 etc.

*/

#include <stdint.h>
#include "my_fp.h"
#include "params.h"
#include "chargerhw.h"
#include "my_math.h"

class teslaCharger: public Chargerhw
{

public:
void DecodeCAN(int id, uint32_t data[2]);
void Task100Ms();
bool ControlCharge(bool RunCh, bool ACReq);
void SetCanInterface(CanHardware* c);
/*
static void handle108(uint32_t data[2]);
static bool HVreq;
static void Send100msMessages(bool ChRun, CanHardware* can);
*/
private:

};

#endif /* teslaCharger_h */
