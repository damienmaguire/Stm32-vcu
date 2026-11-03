// Host stub of libopeninv errormessage.h for the BMW_SP51 host check
// (test/bmw_sp51). Counts the errors posted.
#pragma once
enum ERROR_MESSAGE_NUM { ERR_BMSCOMM, ERROR_MESSAGE_LAST };
namespace ErrorMessage {
extern int posted[ERROR_MESSAGE_LAST];
inline void Post(ERROR_MESSAGE_NUM msg) { posted[msg]++; }
} // namespace ErrorMessage
