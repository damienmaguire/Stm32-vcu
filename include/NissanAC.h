
/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2026 Damien Maguire
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
#ifndef LEAFCOMPRESSOR_H
#define LEAFCOMPRESSOR_H

#include <compressor.h>
#include "linbus.h"
#include "my_string.h"

class LeafCompressor : public Compressor
{
   public:
      /** Default constructor */
      LeafCompressor();
      void SetLinInterface(LinBus* l);
      void Task10Ms();
      void Task200Ms();

   private:
      bool isAwake=false;
      LinBus* lin;
};

#endif // LEAFCOMPRESSOR_H
