/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2011 Johannes Huebner <dev@johanneshuebner.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "my_string.h"
#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/dma.h>
#include "terminal.h"
#include "hwdefs.h"
#include <stdarg.h>

static const TERM_CMD *CmdLookup(char *buf);
static void term_send(uint32_t usart, const char *str);
static void ResetDMA();

extern const TERM_CMD TermCmds[];
static char inBuf[TERM_BUFSIZE];
static char outBuf[2][TERM_BUFSIZE]; //double buffering

void term_Init()
{
   ResetDMA();
}

/** Run the terminal */
void term_Run()
{
   char args[TERM_BUFSIZE];
   const TERM_CMD *pCurCmd = NULL;
   int lastIdx = 0;

   while (1)
   {
      int numRcvd = dma_get_number_of_data(DMA1, TERM_USART_DMARX);
      int currentIdx = TERM_BUFSIZE - numRcvd;

      if (0 == numRcvd)
         ResetDMA();

      while (lastIdx < currentIdx) //echo
         usart_send_blocking(TERM_USART, inBuf[lastIdx++]);

      if (currentIdx > 0)
      {
         if (inBuf[currentIdx - 1] == '\n' || inBuf[currentIdx - 1] == '\r')
         {
            inBuf[currentIdx] = 0;
            lastIdx = 0;
            char *space = (char*)my_strchr(inBuf, ' ');

            if (0 == *space) //No args after command, look for end of line
            {
               space = (char*)my_strchr(inBuf, '\n');
               args[0] = 0;
            }
            else //There are arguments, copy everything behind the space
            {
               my_strcpy(args, space + 1);
            }

            if (0 == *space) //No \n found? try \r
               space = (char*)my_strchr(inBuf, '\r');

            *space = 0;
            pCurCmd = CmdLookup(inBuf);
            ResetDMA();

            if (NULL != pCurCmd)
            {
               usart_wait_send_ready(TERM_USART);
               pCurCmd->CmdFunc(args);
            }
            else if (currentIdx > 1)
            {
               term_send(TERM_USART, "Unknown command sequence\r\n");
            }
         }
         else if (inBuf[0] == '!' && NULL != pCurCmd)
         {
            ResetDMA();
            lastIdx = 0;
            pCurCmd->CmdFunc(args);
         }
      }
   } /* while(1) */
} /* term_Run */

/*
 * Revision 1 hardware can only use synchronous sending as the DMA channel is
 * occupied by the encoder timer (TIM3, channel 3).
 * All other hardware can use DMA for seamless sending of data. We use double
 * buffering, so while one buffer is sent by DMA we can prepare the other
 * buffer to go next.
*/
int putchar(int c)
{
   static uint32_t curIdx = 0, curBuf = 0, first = 1;

   if (hwRev == HW_REV1)
   {
      usart_send_blocking(TERM_USART, c);
   }
   else if (c == '\n' || curIdx == (TERM_BUFSIZE - 1))
   {
      outBuf[curBuf][curIdx] = c;

      while (!dma_get_interrupt_flag(DMA1, TERM_USART_DMATX, DMA_TCIF) && !first);

      dma_disable_channel(DMA1, TERM_USART_DMATX);
      dma_set_number_of_data(DMA1, TERM_USART_DMATX, curIdx + 1);
      dma_set_memory_address(DMA1, TERM_USART_DMATX, (uint32_t)outBuf[curBuf]);
      dma_clear_interrupt_flags(DMA1, TERM_USART_DMATX, DMA_TCIF);
      dma_enable_channel(DMA1, TERM_USART_DMATX);

      curBuf = !curBuf; //switch buffers
      first = 0; //only needed once so we don't get stuck in the while loop above
      curIdx = 0;
   }
   else
   {
      outBuf[curBuf][curIdx] = c;
      curIdx++;
   }
   return 0;
}

static void ResetDMA()
{
   dma_disable_channel(DMA1, TERM_USART_DMARX);
   dma_set_memory_address(DMA1, TERM_USART_DMARX, (uint32_t)inBuf);
   dma_set_number_of_data(DMA1, TERM_USART_DMARX, TERM_BUFSIZE);
   dma_enable_channel(DMA1, TERM_USART_DMARX);
}

static const TERM_CMD *CmdLookup(char *buf)
{
   const TERM_CMD *pCmd = TermCmds;

   for (; NULL != pCmd->cmd; pCmd++)
   {
      if (0 == my_strcmp(buf, pCmd->cmd))
      {
         break;
      }
   }
   if (NULL == pCmd->cmd)
   {
      pCmd = NULL;
   }
   return pCmd;
}

static void term_send(uint32_t usart, const char *str)
{
   for (;*str > 0; str++)
       usart_send_blocking(usart, *str);
}


