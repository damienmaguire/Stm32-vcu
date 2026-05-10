/* Minimal stub to satisfy the linker for `make Test`.
 *
 * iX4_Lever::SetCanInterface() calls CanHardware::RegisterUserMessage(),
 * which lives in libopeninv/src/canhardware.cpp. That implementation pulls
 * in libopencm3 RTC / timer code that has no place in a host-side test
 * binary. This stub satisfies the linker without dragging in firmware deps.
 * SetCanInterface is never called from the unit tests themselves.
 */

#include "canhardware.h"

bool CanHardware::RegisterUserMessage(uint32_t /*canId*/, uint32_t /*mask*/) { return false; }
