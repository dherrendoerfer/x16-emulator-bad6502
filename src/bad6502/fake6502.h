/*  bad6502 A Raspberry Pi-based backend to a 65C02 CPU
    Copyright (C) 2025  D.Herrendoerfer

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>. 
*/

#ifndef _BADFAKE6502_H_
#define _BADFAKE6502_H_

#include <stdint.h>
#include "../endian.h"

struct regs
{
    LOW_HIGH_UNION(c, a, b);
    LOW_HIGH_UNION(x, xl, xh);
    LOW_HIGH_UNION(y, yl, yh);

    uint16_t dp;
    uint16_t sp;

    uint8_t db;
    uint16_t pc;
    uint8_t k;

    uint8_t status;
    uint8_t e;

    uint8_t is65c816;
};

extern void reset6502(uint8_t c816);
extern void step6502();
//extern void exec6502(uint32_t tickcount);
extern void irq6502(uint8_t state);
extern void nmi6502();
extern uint32_t clockticks6502;
extern uint8_t waiting;
//extern bool warn_rockwell;

#endif