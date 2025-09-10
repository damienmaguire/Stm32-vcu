/*
 * This file is part of the ZombieVeter project.
 *
 * Copyright (C) 2020 Johannes Huebner <dev@johanneshuebner.com>
 *               2021-2022 Damien Maguire <info@evbmw.com>
 * Yes I'm really writing software now........run.....run away.......
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

#ifndef ERRORMESSAGE_PRJ_H_INCLUDED
#define ERRORMESSAGE_PRJ_H_INCLUDED

#define ERROR_BUF_SIZE 4

#define ERROR_MESSAGE_LIST                                                     \
  ERROR_MESSAGE_ENTRY(BMSCOMM, ERROR_STOP)                                     \
  ERROR_MESSAGE_ENTRY(OVERVOLTAGE, ERROR_STOP)                                 \
  ERROR_MESSAGE_ENTRY(PRECHARGE, ERROR_STOP)                                   \
  ERROR_MESSAGE_ENTRY(THROTTLE1, ERROR_DISPLAY)                                \
  ERROR_MESSAGE_ENTRY(THROTTLE2, ERROR_DISPLAY)                                \
  ERROR_MESSAGE_ENTRY(THROTTLE12, ERROR_DISPLAY)                               \
  ERROR_MESSAGE_ENTRY(THROTTLE12DIFF, ERROR_DISPLAY)                           \
  ERROR_MESSAGE_ENTRY(THROTTLEMODE, ERROR_DISPLAY)                             \
  ERROR_MESSAGE_ENTRY(CANTIMEOUT, ERROR_DISPLAY)                               \
  ERROR_MESSAGE_ENTRY(TMPHSMAX, ERROR_DERATE)                                  \
  ERROR_MESSAGE_ENTRY(TMPMMAX, ERROR_DERATE)                                   \
  ERROR_MESSAGE_ENTRY(HVILERR, ERROR_DISPLAY)

#endif // ERRORMESSAGE_PRJ_H_INCLUDED
