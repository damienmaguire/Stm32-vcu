
#ifndef ElconCharger_h
#define ElconCharger_h

/*  This library supports the various Elcon charger protocol based chargers

*/

#include <stdint.h>
#include "my_fp.h"
#include "params.h"
#include "chargerhw.h"
#include "my_math.h"

class ElconCharger: public Chargerhw
{

public:
void DecodeCAN(int id, const uint8_t bytes[8]) override;
void Task200Ms();
bool ControlCharge(bool RunCh, bool ACReq);
void SetCanInterface(CanHardware* c);
void handle18FF50E5(const uint8_t bytes[8]);
/*
static void handle108(uint32_t data[2]);
static bool HVreq;
static void Send100msMessages(bool ChRun, CanHardware* can);
*/
private:

};

#endif /* ElconCharger_h */
