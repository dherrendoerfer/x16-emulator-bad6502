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

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#include "fake6502.h"

//#define DEBUG 1
//#define CPUDEBUG 1

uint16_t opcode_addr, oldpc, reladdr, value;
struct regs regs;
uint8_t waiting = 1;

// PI defs
#define GPIO_BASE                (mmio_peri_base + 0x200000) /* GPIO controller */

// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x) or SET_GPIO_ALT(x,y)
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))

#define GPIO_SET *(gpio+7)  // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR *(gpio+10) // clears bits which are 1 ignores bits which are 0

#define GET_GPIO(g) (*(gpio+13)&(1<<g)) // 0 if LOW, (1<<g) if HIGH

#define GPIO_PULL *(gpio+37) // Pull up/pull down
#define GPIO_PULLCLK0 *(gpio+38) // Pull up/pull down clock

#define GET_ADDR (*(gpio+13)>>8)&0xFFFF
#define GET_DATA (*(gpio+13))&0xFF
#define GET_RW (*(gpio+13))&(1<<24)
#define GET_ALL_GPIO (*(gpio+13))

// delay helpers
#define ndelay20 __asm__ volatile ("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop");
#define ndelay10 __asm__ volatile ("nop\nnop\nnop\nnop\nnop");

// Defined for speed
#define _6502_gpio_data_r 0x0
#define _6502_gpio_data_w 0x249249

// Defined memory io elsewehere (hopefully)
extern uint8_t read6502(uint16_t address, uint8_t bank);
extern void write6502(uint16_t address, uint8_t bank, uint8_t value);

// MMIO base (detected)
uint32_t mmio_peri_base;
uint8_t pi_caps;

int  mem_fd;
void *gpio_map;

// I/O access
volatile unsigned *gpio;
volatile uint32_t *gpio_i;
volatile uint32_t *gpio_o_set;
volatile uint32_t *gpio_o_clear;
uint8_t proc_init_done = 0;

uint32_t clockticks6502; //increases on transition from low to high
static uint8_t irqstate = 0;

// HELPERS 
//
// Detect Pi model to correctly map GPIO
//
void detectPi()
{
  int num;
  int id_fd = 0;
  char id_str[256];
  bzero((void*)id_str,256);

  if ((id_fd=open("/sys/firmware/devicetree/base/model",O_RDONLY)) < 0) {
    printf("can't open /sys/firmware/devicetree/base/model\n");
    exit(1);
  }

  if ((num=read(id_fd, id_str, sizeof(id_str))) < 0) {
    printf("can't read /sys/firmware/devicetree/base/model\n");
    exit(1);
  }

  close(id_fd);

  if (strncmp(id_str, "Raspberry Pi Zero 2", 19) == 0) { 
      mmio_peri_base = 0x3F000000;
      pi_caps = 2;
  } else if (strncmp(id_str, "Raspberry Pi 4 Model B", 22) == 0) { 
      mmio_peri_base = 0xFE000000; 
      pi_caps = 4;
  } else if (strncmp(id_str, "Raspberry Pi 3 Model B Plus", 27) == 0) { 
      mmio_peri_base = 0x3F000000; 
      pi_caps = 3;
  } else { 
      //default
      mmio_peri_base = 0x3F000000;
      pi_caps = 3;
      printf("Pi model not known please update cpu.c with the output of\n  cat /sys/firmware/devicetree/base/model");
      printf(". Defaulting to Pi3");
  } 

    //printf("id:%s\n", id_str );
}
//
// Set up a memory regions to access GPIO
//
void setup_io()
{
   /* open /dev/mem */
   if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
      printf("can't open /dev/mem \n");
      exit(-1);
   }

   /* mmap GPIO */
   gpio_map = mmap(
      NULL, 
      4096,
      PROT_READ|PROT_WRITE,
      MAP_SHARED,
      mem_fd,
      GPIO_BASE
   );
   /* close fd */
   close(mem_fd); 

   if (gpio_map == MAP_FAILED) {
      printf("mmap error %d\n", (int)gpio_map);//errno also set!
      exit(-1);
   }

   /* store map pointer */
   gpio = (volatile unsigned *)gpio_map;
   gpio_i        = (volatile void *)gpio+0x34;
   gpio_o_set    = (volatile void *)gpio+0x1C;
   gpio_o_clear  = (volatile void *)gpio+0x28;
}

void init6502()
{
  int g;
#ifdef DEBUG
  printf("Init65C05\n");
#endif

  // Detects Pi model and set some constants
  detectPi();

  // Set up gpio pointer for direct register access
  setup_io();

  // Set GPIO pins 0-7 to output
  for (g=0; g<=7; g++) {
    INP_GPIO(g); // must use INP_GPIO before we can use OUT_GPIO
    OUT_GPIO(g);
  }

  // Set GPIO pins 8-23 to input
  for (g=8; g<=23; g++) {
    INP_GPIO(g);
  }

  // Set GPIO pins 24 to input (!RW)
  INP_GPIO(24);

  // Set GPIO pins 25 to output (CLOCK)
  INP_GPIO(25);
  OUT_GPIO(25);

  // Set GPIO pins 26 to output (!RESET)
  INP_GPIO(26);
  OUT_GPIO(26);
  GPIO_SET = 1<<26; //release !RESET

  // Set GPIO pins 27 to output (!IRQ)
  INP_GPIO(27);
  OUT_GPIO(27);
  GPIO_SET = 1<<27; //release !IRQ
 
  proc_init_done = 1; 
}

//
// one_clock: do one clock cycle (ignore everything)
//
void one_clock()
{
  *gpio_o_clear = 1<<25;
  ndelay20;
  ndelay20;
  *gpio_o_set = 1<<25;
  ndelay20;
  ndelay20;
}

void reset6502(uint8_t c816)
{
  if (c816) {
    printf("816 not supported\n");
    exit(1);
  }

  if (!proc_init_done)
    init6502();

#ifdef DEBUG
  printf("Reset6502");
#endif

  // Perform 6502 reset
  GPIO_CLR = 1<<26; //set !RESET
  one_clock();  
  one_clock();  
  one_clock();  
  one_clock();  
  GPIO_SET = 1<<26; //clear !RESET
  clockticks6502 = 0;

    regs.c = 0;
    regs.x = 0;
    regs.y = 0;
    regs.dp = 0;
    regs.sp = 0x1FD;
    regs.e = 1;
    regs.k = 0;
    regs.db = 0;
        regs.status = 0;
        regs.is65c816 = 0;
        //ticktable = ticktable_c02;
        //optable = optable_c02;
        //addrtable = addrtable_c02;
    waiting = 0;
}

void irq6502(uint8_t state)
{
#ifdef DEBUG
  printf("IRQ65C05\n");
#endif

  if (state) {
    // Perform 6502 irq
    GPIO_CLR = 1<<27; //set !IRQ
  }
  else {
    GPIO_SET = 1<<27; //release !IRQ
  }
  irqstate = state;
}

void nmi6502()
{
#ifdef DEBUG
  printf("NMI6502\n");
#endif

  // Perform 6502 reset
  GPIO_CLR = 1<<26; //set !RESET
  one_clock();  
  one_clock();  
  one_clock();  
  one_clock();  
  GPIO_SET = 1<<26; //clear !RESET
  clockticks6502 = 0;
}

static uint32_t bus_rw = 0;
static uint32_t bus_addr = 0;
static uint32_t data = 0;
static uint32_t tmp;
static uint32_t vtmp;

void tick6502()
{  
#ifdef DEBUG
  printf("Step6502\n");
#endif

  //TODO: The delays need to be optimized (scoped)

  // Clock cycle start (clock is low)
  *gpio = _6502_gpio_data_r;
  tmp = *gpio_i;
  
  // 2nd part of clock cycle (clock goes high)
  *gpio_o_set = 1<<25;
  // Flush the cache (make sure the write really happened)
//  __asm__ volatile ("dsb ishst" : : : "memory");
  vtmp = *gpio_o_set;

  // decode addr and !RW from the bus
  bus_rw = tmp & 1<<24;
  bus_addr = (tmp >> 8) & 0xFFFF;

  // Set DATA to INP or OUT based on RW
  if (bus_rw) {
    //set DATA to OUT
    *gpio = _6502_gpio_data_w;

    data=read6502(bus_addr,0);

    //write to 6502
    *gpio_o_clear=(uint32_t) 0xff;
    *gpio_o_set=data;
    vtmp = *gpio_o_set;

    ndelay20;
  }
  else {
    ndelay10;
    //read from 6502
    data = *gpio_i & 0xFF;
    write6502(bus_addr, 0, data);
  }

  // Finish the clock cycle
  // Clock cycle start (clock goes low)
  clockticks6502++;
  *gpio_o_clear = 1<<25;
  // Flush the cache (make sure the write really happened)
  //__asm__ volatile ("dsb ishst" : : : "memory");
  vtmp = *gpio_o_clear;

  #ifdef CPUDEBUG
    printf("ADDR: %04X, DATA: %02X, /RW: %01X, IRQ: %X\n",bus_addr, data, bus_rw>>24, irqstate);
  #endif

  ndelay20;
}

void step6502()
{
  uint8_t ticks=0;
  tick6502();
  ndelay20;
  if (bus_addr > 0xFFF9)
    return;

  ticks=ticktable_65c02[data];
  while (--ticks) {
    tick6502();
    ndelay20;
    if (bus_addr > 0xFFF9)
      return;
  }
}

void exec6502(uint32_t tickcount)
{
  uint32_t g;
  for (g=0; g<tickcount; g++)
    step6502();  
}

//a few general functions used by various other functions

uint16_t add_wrap_at_page_boundary(uint16_t value, uint8_t add) {
    if (regs.e) {
        return (value & 0xFF00) | ((uint16_t) (((uint8_t) (value & 0x00FF)) + add) & 0x00FF);
    } else {
        return value + add;
    }
}

uint16_t subtract_wrap_at_page_boundary(uint16_t value, uint8_t subtract) {
    if (regs.e) {
        return (value & 0xFF00) | ((uint16_t) (((uint8_t) (value & 0x00FF)) - subtract) & 0x00FF);
    } else {
        return value - subtract;
    }
}

void increment_wrap_at_page_boundary(uint16_t *value) {
    if (regs.e) {
        *value = (*value & 0xFF00) | ((uint16_t) (((uint8_t) (*value & 0x00FF)) + 1) & 0x00FF);
    } else {
        (*value)++;
    }
}

void decrement_wrap_at_page_boundary(uint16_t *value) {
    if (regs.e) {
        *value = (*value & 0xFF00) | ((uint16_t) (((uint8_t) (*value & 0x00FF)) - 1) & 0x00FF);
    } else {
        (*value)--;
    }
}

uint16_t direct_page_add(uint16_t offset) {
    if (regs.e && (regs.dp & 0x00FF) == 0) {
        return (regs.dp & 0xFF00) | ((uint16_t) ((uint8_t) (regs.dp & 0x00FF)) + (offset & 0xFF));
    } else {
        return regs.dp + offset;
    }
}

#define incsp() increment_wrap_at_page_boundary(&regs.sp)
#define decsp() decrement_wrap_at_page_boundary(&regs.sp)