
#include "Can_VAG.h"

void Can_VAG::SendVAG100msMessage()
{
   static int seqCtr = 0;
   static uint8_t ctr = 0;

   const uint8_t seq1[] = { 0x0f, 0x28, 0x7f, 0x28 };
   const uint8_t seq2[] = { 0x1e, 0x10, 0x00, 0x10 };
   const uint8_t seq3[] = { 0x70, 0x56, 0xf0, 0x56 };
   const uint8_t seq4[] = { 0x0c, 0x48, 0xa7, 0x48 };
   const uint8_t seq5[] = { 0x46, 0x90, 0x28, 0x90 };

   uint8_t canData[8] = { (uint8_t)(0x80 | ctr), 0, 0, seq1[seqCtr], seq2[seqCtr], seq3[seqCtr], seq4[seqCtr], seq5[seqCtr] };

   Can::GetInterface(1)->Send(0x580, (uint32_t*)canData,8); //Send on CAN2
   seqCtr = (seqCtr + 1) & 0x3;
   ctr = (ctr + 1) & 0xF;
}
