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

// TIMING for TICKS to STEPS
static const uint32_t ticktable_65c02[256] = {
/*        |  0  |  1  |  2  |  3  |  4  |  5  |  6  |  7  |  8  |  9  |  A  |  B  |  C  |  D  |  E  |  F  |     */
/* 0 */       7,    6,    2,    1,    5,    3,    5,    5,    3,    2,    2,    1,    6,    4,    6,    5, /* 0 */
/* 1 */       2,    5,    5,    1,    5,    4,    6,    5,    2,    4,    2,    1,    6,    4,    7,    5, /* 1 */
/* 2 */       6,    6,    2,    1,    3,    3,    5,    5,    4,    2,    2,    1,    4,    4,    6,    5, /* 2 */
/* 3 */       2,    5,    5,    1,    4,    4,    6,    5,    2,    4,    2,    1,    4,    4,    7,    5, /* 3 */
/* 4 */       6,    6,    2,    1,    3,    3,    5,    5,    3,    2,    2,    1,    3,    4,    6,    5, /* 4 */
/* 5 */       2,    5,    5,    1,    4,    4,    6,    5,    2,    4,    3,    1,    8,    4,    7,    5, /* 5 */
/* 6 */       6,    6,    2,    1,    3,    3,    5,    5,    4,    2,    2,    1,    5,    4,    6,    5, /* 6 */
/* 7 */       2,    5,    5,    1,    4,    4,    6,    5,    2,    4,    4,    1,    6,    4,    7,    5, /* 7 */
/* 8 */       3,    6,    2,    1,    3,    3,    3,    5,    2,    2,    2,    1,    4,    4,    4,    5, /* 8 */
/* 9 */       2,    6,    5,    1,    4,    4,    4,    5,    2,    5,    2,    1,    4,    5,    5,    5, /* 9 */
/* A */       2,    6,    2,    1,    3,    3,    3,    5,    2,    2,    2,    1,    4,    4,    4,    5, /* A */
/* B */       2,    5,    5,    1,    4,    4,    4,    5,    2,    4,    2,    1,    4,    4,    4,    5, /* B */
/* C */       2,    6,    2,    1,    3,    3,    5,    5,    2,    2,    2,    3,    4,    4,    6,    5, /* C */
/* D */       2,    5,    5,    1,    4,    4,    6,    5,    2,    4,    3,    1,    4,    4,    7,    5, /* D */
/* E */       2,    6,    2,    1,    3,    3,    5,    5,    2,    2,    2,    1,    4,    4,    6,    5, /* E */
/* F */       2,    5,    5,    1,    4,    4,    6,    5,    2,    4,    4,    1,    4,    4,    7,    5  /* F */
};

#endif