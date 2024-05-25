/*  Uzebox Emulation of a Commodore 64
    based on Arduino sketch: Copyright (C) 2021  Doctor Volt
	https://github.com/michalin/Arduino-C64-Emulator

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

#include <ctype.h>
#include <stdbool.h>
#include <avr/io.h>
#include <stdlib.h>
#include <avr/pgmspace.h>
#include <uzebox.h>
#include "data/hack-font.inc"
#include "data/roms.inc"
#include "data/scancodes.inc"

extern volatile unsigned int joypad1_status_lo,joypad2_status_lo;
extern volatile unsigned int joypad1_status_hi,joypad2_status_hi;
#define Wait200ns() asm volatile("lpm\n\tlpm\n\t");
#define Wait100ns() asm volatile("lpm\n\t");


#define NO_SPI_RAM 1

#ifndef NO_SPI_RAM
	#include <spiram.h>

	uint16_t spiram_cursor = 0;
	uint16_t spiram_state = 0;

	uint8_t SpiRamCursorInit(){
		spiram_cursor = 0xFFFC;//reset vector
		spiram_state = 0;
		if(!SpiRamInit())
			return 0;
		SpiRamSeqReadStart(0, 0xFFFC);
		return 1;
	}

	uint8_t SpiRamCursorRead(uint16_t addr){
		if(spiram_state){//in a sequential write?
			SpiRamSeqWriteEnd();
			SpiRamSeqReadStart(0, addr);
			spiram_state = 0;
			spiram_cursor = addr;
			return SpiRamSeqReadU8();
		}
		if(spiram_cursor != addr){//current sequential read position doesn't match?
			SpiRamSeqReadEnd();
			SpiRamSeqReadStart(0, addr);
			spiram_cursor = addr;
			return SpiRamSeqReadU8();
		}
		return SpiRamSeqReadU8();
	}

	void SpiRamCursorWrite(uint16_t addr, uint8_t val){
		if(!spiram_state){//in a sequential read?
			SpiRamSeqReadEnd();
			SpiRamSeqWriteStart(0, addr);
			spiram_state = 1;
			spiram_cursor = addr;
			SpiRamSeqWriteU8(val);
			return;
		}
		if(spiram_cursor != addr){//current sequential write position doesn't match?
			SpiRamSeqWriteEnd();
			SpiRamSeqWriteStart(0, addr);
			spiram_cursor = addr;
			SpiRamSeqWriteU8(val);
			return;
		}
	}
#endif

void dbgb(char x,char y,unsigned char byte){
	unsigned char nibble;

	//hi nibble	
	nibble=(byte>>4);
	if(nibble<=9){
		SetFont(x,y,nibble+48);
	}else{
		SetFont(x,y,nibble-9);
	}

	//lo nibble	
	nibble=(byte&0xf);
	if(nibble<=9){		
		SetFont(x+1,y,nibble+48);
	}else{
		SetFont(x+1,y,nibble-9);
	}

}















#define FLAG_CARRY 0x01
#define FLAG_ZERO 0x02
#define FLAG_INTERRUPT 0x04
#define FLAG_DECIMAL 0x08
#define FLAG_BREAK 0x10
#define FLAG_CONSTANT 0x20
#define FLAG_OVERFLOW 0x40
#define FLAG_SIGN 0x80
#define BASE_STACK 0x100
#define saveaccum(n) a = (uint8_t)((n)&0x00FF)

// flag modifier macros
#define setcarry() cpustatus |= FLAG_CARRY
#define clearcarry() cpustatus &= (~FLAG_CARRY)
#define setzero() cpustatus |= FLAG_ZERO
#define clearzero() cpustatus &= (~FLAG_ZERO)
#define setinterrupt() cpustatus |= FLAG_INTERRUPT
#define clearinterrupt() cpustatus &= (~FLAG_INTERRUPT)
#define setdecimal() cpustatus |= FLAG_DECIMAL
#define cleardecimal() cpustatus &= (~FLAG_DECIMAL)
#define setoverflow() cpustatus |= FLAG_OVERFLOW
#define clearoverflow() cpustatus &= (~FLAG_OVERFLOW)
#define setsign() cpustatus |= FLAG_SIGN
#define clearsign() cpustatus &= (~FLAG_SIGN)

// flag calculation macros
#define zerocalc(n)      \
	{                    \
		if((n)&0x00FF)  \
			clearzero(); \
		else             \
			setzero();   \
	}

#define signcalc(n)      \
	{                    \
		if((n)&0x0080)  \
			setsign();   \
		else             \
			clearsign(); \
	}

#define carrycalc(n)      \
	{                     \
		if((n)&0xFF00)   \
			setcarry();   \
		else              \
			clearcarry(); \
	}

#define overflowcalc(n, m, o)                             \
	{                                                     \
		if(((n) ^ (uint16_t)(m)) & ((n) ^ (o)) & 0x0080) \
			setoverflow();                                \
		else                                              \
			clearoverflow();                              \
	}

// 6502 CPU registers
uint16_t pc;
uint8_t sp, a, x, y, cpustatus;

// helper variables
// uint32_t instructions = 0; //keep track of total instructions executed
// int32_t clockticks6502 = 0, clockgoal6502 = 0;
uint16_t oldpc, ea, reladdr, value, result;
uint8_t opcode, oldcpustatus, useaccum;

#define VIDEOADDR 0x400
#define VIDEOLEN 1024
#define RAMSIZE 101//101 //More memory will make it unstable
#define EESIZE 0//1024

#define LOW_MEM_TOP 0x333 // Page 0: 0x000-0xFF, stack: 0x100-0x1FF, Kernal & Basic working area: 0x200-0x3ff
#define RAMADDR 0x800
#define EEADDR RAMADDR + RAMSIZE

uint8_t sysram[LOW_MEM_TOP + 1];
//uint8_t videomem[VIDEOLEN];
uint8_t basicram[RAMSIZE];

uint8_t m_read(uint16_t address){
	//HACKITY HACK HACK
	sysram[0x38]=0x08;sysram[0x37] = RAMSIZE;//HACK set end of BASIC RAM, how did this work on an Uno with unmodified kernal??

	if(address <= LOW_MEM_TOP)
		return (sysram[address]);
	if((address >= VIDEOADDR) && (address < VIDEOADDR + VIDEOLEN)) // 0x400
		return vram[address - VIDEOADDR];
	if(address >= RAMADDR && address < RAMADDR + RAMSIZE)
		return basicram[address - RAMADDR];
//	if((address >= EEADDR && (address < EEADDR + EESIZE)))
//		return 0;//EEPROM.read(address - EEADDR);
	if(address >= 0xA000 && address < 0xBFFF) // Basic
		return pgm_read_byte(&basicROM[(address - 0xA000)]);//return pgm_read_byte_near(basic + (address - 0xA000));
	if(address >= 0xE000 && address < 0xFFFF) // Kernal
		return pgm_read_byte(&kernalROM[(address - 0xE000)]);//return pgm_read_byte_near(kernal + (address - 0xE000));
#ifndef NO_SPI_RAM
	return SpiRamCursorRead(address);
#else
	return 0;
#endif
}

void m_write(uint16_t address, uint8_t value){
//vram[address%1000]=value;
	if(address < LOW_MEM_TOP)
		sysram[address] = value;
	else if((address >= VIDEOADDR) && (address < VIDEOADDR + VIDEOLEN))
		vram[address - VIDEOADDR] = value;
	else if(address >= RAMADDR && address < RAMADDR + RAMSIZE)
		basicram[address - RAMADDR] = value;
#ifndef NO_SPI_RAM
	else if(address < 0xA000)
		SpiRamCursorWrite(address, value);
#endif
//	else if((address >= EEADDR && (address < EEADDR + EESIZE)))
//		EEPROM.write(address - EEADDR, value);
}
// a few general functions used by various other functions
void push16(uint16_t pushval){
	m_write(BASE_STACK + sp, (pushval >> 8) & 0xFF);
	m_write(BASE_STACK + ((sp - 1) & 0xFF), pushval & 0xFF);
	sp -= 2;
}

void push8(uint8_t pushval){
	m_write(BASE_STACK + sp--, pushval);
}

uint16_t pull16(){
	uint16_t temp16;
	temp16 = m_read(BASE_STACK + ((sp + 1) & 0xFF)) | ((uint16_t)m_read(BASE_STACK + ((sp + 2) & 0xFF)) << 8);
	sp += 2;
	return (temp16);
}

uint8_t pull8(){
	return (m_read(BASE_STACK + ++sp));
}

void resetCPU(){
	pc = (uint16_t)m_read(0xFFFC) | ((uint16_t)m_read(0xFFFD) << 8);
	a = 0;
	x = 0;
	y = 0;
	sp = 0xFF;
	cpustatus |= FLAG_CONSTANT;
}

// addressing mode functions, calculates effective addresses
void imp(){ // implied
}

void acc(){ // accumulator
	useaccum = 1;
}

void imm(){ // immediate
	ea = pc++;
}

void zp(){ // zero-page
	ea = (uint16_t)m_read((uint16_t)pc++);
}

void zpx(){																// zero-page,X
	ea = ((uint16_t)m_read((uint16_t)pc++) + (uint16_t)x) & 0xFF; // zero-page wraparound
}

void zpy(){																// zero-page,Y
	ea = ((uint16_t)m_read((uint16_t)pc++) + (uint16_t)y) & 0xFF; // zero-page wraparound
}

void rel(){ // relative for branch ops (8-bit immediate value, sign-extended)
	reladdr = (uint16_t)m_read(pc++);
	if(reladdr & 0x80)
		reladdr |= 0xFF00;
}

void abso(){ // absolute
	ea = (uint16_t)m_read(pc) | ((uint16_t)m_read(pc + 1) << 8);
	pc += 2;
}

void absx(){ // absolute,X
	//uint16_t startpage;
	ea = ((uint16_t)m_read(pc) | ((uint16_t)m_read(pc + 1) << 8));
	//startpage = ea & 0xFF00;
	ea += (uint16_t)x;
	pc += 2;
}

void absy(){ // absolute,Y
	//uint16_t startpage;
	ea = ((uint16_t)m_read(pc) | ((uint16_t)m_read(pc + 1) << 8));
	//startpage = ea & 0xFF00;
	ea += (uint16_t)y;
	pc += 2;
}

void ind(){ // indirect
	uint16_t eahelp, eahelp2;
	eahelp = (uint16_t)m_read(pc) | (uint16_t)((uint16_t)m_read(pc + 1) << 8);
	eahelp2 = (eahelp & 0xFF00) | ((eahelp + 1) & 0x00FF); // replicate 6502 page-boundary wraparound bug
	ea = (uint16_t)m_read(eahelp) | ((uint16_t)m_read(eahelp2) << 8);
	pc += 2;
}

void indx(){ // (indirect,X)
	uint16_t eahelp;
	eahelp = (uint16_t)(((uint16_t)m_read(pc++) + (uint16_t)x) & 0xFF); // zero-page wraparound for table pointer
	ea = (uint16_t)m_read(eahelp & 0x00FF) | ((uint16_t)m_read((eahelp + 1) & 0x00FF) << 8);
}

void indy(){ // (indirect),Y
	uint16_t eahelp, eahelp2;//, startpage;
	eahelp = (uint16_t)m_read(pc++);
	eahelp2 = (eahelp & 0xFF00) | ((eahelp + 1) & 0x00FF); // zero-page wraparound
	ea = (uint16_t)m_read(eahelp) | ((uint16_t)m_read(eahelp2) << 8);
	//startpage = ea & 0xFF00;
	ea += (uint16_t)y;
}

uint16_t getvalue(){
	if(useaccum)
		return ((uint16_t)a);
	else
		return ((uint16_t)m_read(ea));
}

uint16_t getvalue16(){
	return ((uint16_t)m_read(ea) | ((uint16_t)m_read(ea + 1) << 8));
}

void putvalue(uint16_t saveval){
	if(useaccum)
		a = (uint8_t)(saveval & 0x00FF);
	else
		m_write(ea, (saveval & 0x00FF));
}

// instruction handler functions
void adc(){
	value = getvalue();
	result = (uint16_t)a + value + (uint16_t)(cpustatus & FLAG_CARRY);
	carrycalc(result);
	zerocalc(result);
	overflowcalc(result, a, value);
	signcalc(result);

#ifndef NES_CPU
	if(cpustatus & FLAG_DECIMAL){
		clearcarry();

		if((a & 0x0F) > 0x09){
			a += 0x06;
		}
		if((a & 0xF0) > 0x90){
			a += 0x60;
			setcarry();
		}
		// clockticks6502++;
	}
#endif
	saveaccum(result);
}

void op_and(){
	value = getvalue();
	result = (uint16_t)a & value;

	zerocalc(result);
	signcalc(result);

	saveaccum(result);
}

void asl(){
	value = getvalue();
	result = value << 1;

	carrycalc(result);
	zerocalc(result);
	signcalc(result);

	putvalue(result);
}

void bcc(){
	if((cpustatus & FLAG_CARRY) == 0){
		oldpc = pc;
		pc += reladdr;
		// if((oldpc & 0xFF00) != (pc & 0xFF00)) clockticks6502 += 2; //check if jump crossed a page boundary
		//     else clockticks6502++;
	}
}

void bcs(){
	if((cpustatus & FLAG_CARRY) == FLAG_CARRY){
		oldpc = pc;
		pc += reladdr;
		// if((oldpc & 0xFF00) != (pc & 0xFF00)) clockticks6502 += 2; //check if jump crossed a page boundary
		//     else clockticks6502++;
	}
}

void beq(){
	if((cpustatus & FLAG_ZERO) == FLAG_ZERO){
		oldpc = pc;
		pc += reladdr;
		// if((oldpc & 0xFF00) != (pc & 0xFF00)) clockticks6502 += 2; //check if jump crossed a page boundary
		//     else clockticks6502++;
	}
}

void op_bit(){
	value = getvalue();
	result = (uint16_t)a & value;
	zerocalc(result);
	cpustatus = (cpustatus & 0x3F) | (uint8_t)(value & 0xC0);
}

void bmi(){
	if((cpustatus & FLAG_SIGN) == FLAG_SIGN){
		oldpc = pc;
		pc += reladdr;
		// if((oldpc & 0xFF00) != (pc & 0xFF00)) clockticks6502 += 2; //check if jump crossed a page boundary
		//     else clockticks6502++;
	}
}

void bne(){
	if((cpustatus & FLAG_ZERO) == 0){
		oldpc = pc;
		pc += reladdr;
		// if((oldpc & 0xFF00) != (pc & 0xFF00)) clockticks6502 += 2; //check if jump crossed a page boundary
		//     else clockticks6502++;
	}
}

void bpl(){
	if((cpustatus & FLAG_SIGN) == 0){
		oldpc = pc;
		pc += reladdr;
		// if((oldpc & 0xFF00) != (pc & 0xFF00)) clockticks6502 += 2; //check if jump crossed a page boundary
		//     else clockticks6502++;
	}
}

void brk(){
	pc++;
	push16(pc);					   // push next instruction address onto stack
	push8(cpustatus | FLAG_BREAK); // push CPU cpustatus to stack
	setinterrupt();				   // set interrupt flag
	pc = (uint16_t)m_read(0xFFFE) | ((uint16_t)m_read(0xFFFF) << 8);
}

void bvc(){
	if((cpustatus & FLAG_OVERFLOW) == 0){
		oldpc = pc;
		pc += reladdr;
		// if((oldpc & 0xFF00) != (pc & 0xFF00)) clockticks6502 += 2; //check if jump crossed a page boundary
		//     else clockticks6502++;
	}
}

void bvs(){
	if((cpustatus & FLAG_OVERFLOW) == FLAG_OVERFLOW){
		oldpc = pc;
		pc += reladdr;
		// if((oldpc & 0xFF00) != (pc & 0xFF00)) clockticks6502 += 2; //check if jump crossed a page boundary
		//     else clockticks6502++;
	}
}

void clc(){
	clearcarry();
}

void cld(){
	cleardecimal();
}

void clv(){
	clearoverflow();
}

void cmp(){
	value = getvalue();
	result = (uint16_t)a - value;

	if(a >= (uint8_t)(value & 0x00FF))
		setcarry();
	else
		clearcarry();
	if(a == (uint8_t)(value & 0x00FF))
		setzero();
	else
		clearzero();
	signcalc(result);
}

void cpx(){
	value = getvalue();
	result = (uint16_t)x - value;

	if(x >= (uint8_t)(value & 0x00FF))
		setcarry();
	else
		clearcarry();
	if(x == (uint8_t)(value & 0x00FF))
		setzero();
	else
		clearzero();
	signcalc(result);
}

void cpy(){
	value = getvalue();
	result = (uint16_t)y - value;

	if(y >= (uint8_t)(value & 0x00FF))
		setcarry();
	else
		clearcarry();
	if(y == (uint8_t)(value & 0x00FF))
		setzero();
	else
		clearzero();
	signcalc(result);
}

void dec(){
	value = getvalue();
	result = value - 1;
	zerocalc(result);
	signcalc(result);
	putvalue(result);
}

void dex(){
	x--;
	zerocalc(x);
	signcalc(x);
}

void dey(){
	y--;
	zerocalc(y);
	signcalc(y);
}

void eor(){
	value = getvalue();
	result = (uint16_t)a ^ value;
	zerocalc(result);
	signcalc(result);
	saveaccum(result);
}

void inc(){
	value = getvalue();
	result = value + 1;
	zerocalc(result);
	signcalc(result);
	putvalue(result);
}

void inx(){
	x++;
	zerocalc(x);
	signcalc(x);
}

void iny(){
	y++;
	zerocalc(y);
	signcalc(y);
}

void jmp(){
	pc = ea;
}

void jsr(){
	push16(pc - 1);
	pc = ea;
}

void lda(){
	value = getvalue();
	a = (uint8_t)(value & 0x00FF);
	zerocalc(a);
	signcalc(a);
}

void ldx(){
	value = getvalue();
	x = (uint8_t)(value & 0x00FF);
	zerocalc(x);
	signcalc(x);
}

void ldy(){
	value = getvalue();
	y = (uint8_t)(value & 0x00FF);
	zerocalc(y);
	signcalc(y);
}

void lsr(){
	value = getvalue();
	result = value >> 1;

	if(value & 1)
		setcarry();
	else
		clearcarry();
	zerocalc(result);
	signcalc(result);
	putvalue(result);
}

void nop(){
}

void ora(){
	value = getvalue();
	result = (uint16_t)a | value;
	zerocalc(result);
	signcalc(result);
	saveaccum(result);
}

void pha(){
	push8(a);
}

void php(){
	push8(cpustatus | FLAG_BREAK);
}

void pla(){
	a = pull8();
	zerocalc(a);
	signcalc(a);
}

void plp(){
	cpustatus = pull8() | FLAG_CONSTANT;
}

void rol(){
	value = getvalue();
	result = (value << 1) | (cpustatus & FLAG_CARRY);
	carrycalc(result);
	zerocalc(result);
	signcalc(result);
	putvalue(result);
}

void ror(){
	value = getvalue();
	result = (value >> 1) | ((cpustatus & FLAG_CARRY) << 7);

	if(value & 1)
		setcarry();
	else
		clearcarry();
	zerocalc(result);
	signcalc(result);
	putvalue(result);
}

void rti(){
	cpustatus = pull8();
	value = pull16();
	pc = value;
}

void rts(){
	value = pull16();
	pc = value + 1;
}

void sbc(){
	value = getvalue() ^ 0x00FF;
	result = (uint16_t)a + value + (uint16_t)(cpustatus & FLAG_CARRY);
	carrycalc(result);
	zerocalc(result);
	overflowcalc(result, a, value);
	signcalc(result);

#ifndef NES_CPU
	if(cpustatus & FLAG_DECIMAL){
		clearcarry();

		a -= 0x66;
		if((a & 0x0F) > 0x09){
			a += 0x06;
		}
		if((a & 0xF0) > 0x90){
			a += 0x60;
			setcarry();
		}

		// clockticks6502++;
	}
#endif
	saveaccum(result);
}

void sec(){
	setcarry();
}

void sed(){
	setdecimal();
}

void sta(){
	putvalue(a);
}

void stx(){
	putvalue(x);
}

void sty(){
	putvalue(y);
}

void tax(){
	x = a;
	zerocalc(x);
	signcalc(x);
}

void tay(){
	y = a;
	zerocalc(y);
	signcalc(y);
}

void tsx(){
	x = sp;
	zerocalc(x);
	signcalc(x);
}

void txa(){
	a = x;
	zerocalc(a);
	signcalc(a);
}

void txs(){
	sp = x;
}

void tya(){
	a = y;
	zerocalc(a);
	signcalc(a);
}

#ifdef UNDOCUMENTED
void lax(){lda();ldx();}
void sax(){sta();stx();putvalue(a & x);}
void dcp(){dec();cmp();}
void isb(){inc();sbc();}
void slo(){asl();ora();}
void rla(){rol();op_and();}
void sre(){lsr();eor();}
void rra(){ror();adc();}
#else
#define lax nop
#define sax nop
#define dcp nop
#define isb nop
#define slo nop
#define rla nop
#define sre nop
#define rra nop
#endif

void nmirq(){
	// push16(pc);
	// push8(cpustatus);
	cpustatus |= FLAG_INTERRUPT;
	pc = (uint16_t)m_read(0xFFFA) | ((uint16_t)m_read(0xFFFB) << 8);
}

void intrq(){ // Interrupt Request
	// push16(pc);
	// push8(cpustatus);
	cpustatus |= FLAG_INTERRUPT;
	pc = (uint16_t)m_read(0xFFFE) | ((uint16_t)m_read(0xFFFF) << 8);
}

#ifdef USE_TIMING
const uint8_t ticktable[256] PROGMEM = {
	/*        |  0  |  1  |  2  |  3  |  4  |  5  |  6  |  7  |  8  |  9  |  A  |  B  |  C  |  D  |  E  |  F  |     */
	/* 0 */ 7, 6, 2, 8, 3, 3, 5, 5, 3, 2, 2, 2, 4, 4, 6, 6, /* 0 */
	/* 1 */ 2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7, /* 1 */
	/* 2 */ 6, 6, 2, 8, 3, 3, 5, 5, 4, 2, 2, 2, 4, 4, 6, 6, /* 2 */
	/* 3 */ 2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7, /* 3 */
	/* 4 */ 6, 6, 2, 8, 3, 3, 5, 5, 3, 2, 2, 2, 3, 4, 6, 6, /* 4 */
	/* 5 */ 2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7, /* 5 */
	/* 6 */ 6, 6, 2, 8, 3, 3, 5, 5, 4, 2, 2, 2, 5, 4, 6, 6, /* 6 */
	/* 7 */ 2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7, /* 7 */
	/* 8 */ 2, 6, 2, 6, 3, 3, 3, 3, 2, 2, 2, 2, 4, 4, 4, 4, /* 8 */
	/* 9 */ 2, 6, 2, 6, 4, 4, 4, 4, 2, 5, 2, 5, 5, 5, 5, 5, /* 9 */
	/* A */ 2, 6, 2, 6, 3, 3, 3, 3, 2, 2, 2, 2, 4, 4, 4, 4, /* A */
	/* B */ 2, 5, 2, 5, 4, 4, 4, 4, 2, 4, 2, 4, 4, 4, 4, 4, /* B */
	/* C */ 2, 6, 2, 8, 3, 3, 5, 5, 2, 2, 2, 2, 4, 4, 6, 6, /* C */
	/* D */ 2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7, /* D */
	/* E */ 2, 6, 2, 8, 3, 3, 5, 5, 2, 2, 2, 2, 4, 4, 6, 6, /* E */
	/* F */ 2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7	/* F */
};
#endif



uint16_t getpc(){
	return (pc);
}

uint8_t getop(){
	return (opcode);
}






///////////////////////////////////////////////////////////////////////////////////////////////////


u8 kb_readByte(u8 command){
	static u8 state=0;
	u8 data=0;
	unsigned char i;

	if(state==0){//ready to transmit condition 
		JOYPAD_OUT_PORT&=~(_BV(JOYPAD_CLOCK_PIN));
		JOYPAD_OUT_PORT|=_BV(JOYPAD_LATCH_PIN);
		Wait200ns();Wait200ns();Wait200ns();Wait200ns();Wait200ns();
		JOYPAD_OUT_PORT&=~(_BV(JOYPAD_LATCH_PIN));
		JOYPAD_OUT_PORT|=_BV(JOYPAD_CLOCK_PIN);
		Wait200ns();Wait200ns();Wait200ns();Wait200ns();Wait200ns();
		state = (command == KB_SEND_END ? 0:1);
	}

	for(i=0;i<8;i++){//read data
		data<<=1;

		if(command&0x80)
			JOYPAD_OUT_PORT |= _BV(JOYPAD_LATCH_PIN);		
		else
			JOYPAD_OUT_PORT &= ~(_BV(JOYPAD_LATCH_PIN));

		JOYPAD_OUT_PORT &= ~(_BV(JOYPAD_CLOCK_PIN));//pulse clock pin		
		command<<=1;
		Wait100ns();
		if((JOYPAD_IN_PORT&(1<<JOYPAD_DATA2_PIN))!=0)
			data |= 1;
		JOYPAD_OUT_PORT|=_BV(JOYPAD_CLOCK_PIN);		
	}
	JOYPAD_OUT_PORT&=~(_BV(JOYPAD_LATCH_PIN));	

//	if(!data)
	//	return;
	return data;
}


//Disable ROM routine for keyboard scanning @ 0xEA87 to save cycles?
//0x28A-D = keyboard repeat items
//0x291 = commodore shift key
//reclaim free bytes at 0x334-0x3FF
//vram is 1000 bytes followed by 16 unused bytes??
//0xD020=BG,0xD021=FG
//0xD800=COLOR RAM

void on_keypressed(uint8_t code){ // Called when key pressed or released

	if(code){
		if(code == 3){ // Stop (Esc) key
			push16(pc);
			setcarry();	 // Break
			setzero();	 // Simulate CTRL-C
			pc = 0xA832; // Jump to BASIC Stop routine
			return;
		}
	sysram[0x277] = code;
	sysram[0xC6] = 1;//num buffered keys
	}

}


uint8_t kb_read(){
  uint8_t code = 0, scnval = 0;
  static uint8_t modifier;

  uint8_t kbbyte = kb_readByte(KB_SEND_END);
  switch (kbbyte)
  {
  case CRSR: //Cursor control
    scnval = kb_readByte(KB_SEND_END);
    modifier = CRSR;
    if (scnval == BREAK)
    {
      kb_readByte(KB_SEND_END);
      modifier = IDLE;
      scnval = 0;
    }
    break;
  case BREAK:
    kbbyte = kb_readByte(KB_SEND_END);
    if (modifier && modifier == kbbyte)
      modifier = IDLE;
    break;
  case SHIFT: //Shift key
  case ALT_L: //-> Commodore
  case CTRL:
    modifier = kbbyte;
    break;
  case ERR_TIMEOUT:
    //scnval = ERR_TIMEOUT;
  break;
  default:
    scnval = kbbyte;
    break;
  }
  if (!scnval || scnval > 127)
    return 0;
#ifdef KBTEST
  Debug.print(DBG_INFO, "modifier: %x, value: %d", modifier, scnval);
#endif

  switch (modifier)
  {
  case CRSR:
    switch (scnval)
    {
    case 0x74: //CRSR RIGHT
      code = 29;
      break;
    case 0x72: //CRSR DN
      code = 17;
      break;
    case 0x6B: //CRSR LEFT
      code = 157;
      break;
    case 0x75: //CRSR UP
      code = 145;
      break;
    case 0x70: //INST
      code = 148;
      break;
    case 0x6C:   //POS1
      code = 19; //HOME
      break;
    case 0x71: //DEL
      code = 20;
      break;
    }
    break;
  case IDLE:
    code = pgm_read_byte(scancodes + scnval);
    break;
  case SHIFT:
    code = pgm_read_byte(scancodes_l2 + scnval);
    break;
  default:
    code = 0;
  }
#ifdef KBTEST
  Debug.print(DBG_INFO, "modifier: %x, petscii: %d", modifier, code);
#endif
  return code;
}


int main(){
	SetTileTable(hack_font);
	ClearVram();
//	SetBorderColor(0xBFU);
#ifndef NO_SPI_RAM
	SpiRamCursorInit();
#endif
	resetCPU();

	while(1){

#ifdef USE_TIMING
	clockgoal6502 += tickcount;

	while (clockgoal6502 > 0){
#else
	// while (tickcount--) {
#endif
		opcode = m_read(pc++);
		// Debug.print(DBG_INFO, "PC: %04X, Opcode: %02X", pc, opcode);
		cpustatus |= FLAG_CONSTANT;
		useaccum = 0;

		switch (opcode){
		case 0x0: imp();brk();break;
		case 0x1: indx();ora();break;
		case 0x5: zp();ora();break;
		case 0x6: zp();asl();break;
		case 0x8: imp();php();break;
		case 0x9: imm();ora();break;
		case 0xA: acc();asl();break;
		case 0xD: abso();ora();break;
		case 0xE: abso();asl();break;
		case 0x10: rel();bpl();break;
		case 0x11: indy();ora();break;
		case 0x15: zpx();ora();break;
		case 0x16: zpx();asl();break;
		case 0x18: imp();clc();break;
		case 0x19: absy();ora();break;
		case 0x1D: absx();ora();break;
		case 0x1E: absx();asl();break;
		case 0x20: abso();jsr();break;
		case 0x21: indx();op_and();break;
		case 0x24: zp();op_bit();break;
		case 0x25: zp();op_and();break;
		case 0x26: zp();rol();break;
		case 0x28: imp();plp();break;
		case 0x29: imm();op_and();break;
		case 0x2A: acc();rol();break;
		case 0x2C: abso();op_bit();break;
		case 0x2D: abso();op_and();break;
		case 0x2E: abso();rol();break;
		case 0x30: rel();bmi();break;
		case 0x31: indy();op_and();break;
		case 0x35: zpx();op_and();break;
		case 0x36: zpx();rol();break;
		case 0x38: imp();sec();break;
		case 0x39: absy();op_and();break;
		case 0x3D: absx();op_and();break;
		case 0x3E: absx();rol();break;
		case 0x40: imp();rti();break;
		case 0x41: indx();eor();break;
		case 0x45: zp();eor();break;
		case 0x46: zp();lsr();break;
		case 0x48: imp();pha();break;
		case 0x49: imm();eor();break;
		case 0x4A: acc();lsr();break;
		case 0x4C: abso();jmp();break;
		case 0x4D: abso();eor();break;
		case 0x4E: abso();lsr();break;
		case 0x50: rel();bvc();break;
		case 0x51: indy();eor();break;
		case 0x55: zpx();eor();break;
		case 0x56: zpx();lsr();break;
		case 0x58: imp();clearinterrupt();/* cli();*/break;
		case 0x59: absy();eor();break;
		case 0x5D: absx();eor();break;
		case 0x5E: absx();lsr();break;
		case 0x60: imp();rts();break;
		case 0x61: indx();adc();break;
		case 0x65: zp();adc();break;
		case 0x66: zp();ror();break;
		case 0x68: imp();pla();break;
		case 0x69: imm();adc();break;
		case 0x6A: acc();ror();break;
		case 0x6C: ind();jmp();break;
		case 0x6D: abso();adc();break;
		case 0x6E: abso();ror();break;
		case 0x70: rel();bvs();break;
		case 0x71: indy();adc();break;
		case 0x75: zpx();adc();break;
		case 0x76: zpx();ror();break;
		case 0x78: imp();setinterrupt();break;
		case 0x79: absy();adc();break;
		case 0x7D: absx();adc();break;
		case 0x7E: absx();ror();break;
		case 0x81: indx();sta();break;
		case 0x84: zp();sty();break;
		case 0x85: zp();sta();break;
		case 0x86: zp();stx();break;
		case 0x88: imp();dey();break;
		case 0x8A: imp();txa();break;
		case 0x8C: abso();sty();break;
		case 0x8D: abso();sta();break;
		case 0x8E: abso();stx();break;
		case 0x90: rel();bcc();break;
		case 0x91: indy();sta();break;
		case 0x94: zpx();sty();break;
		case 0x95: zpx();sta();break;
		case 0x96: zpy();stx();break;
		case 0x98: imp();tya();break;
		case 0x99: absy();sta();break;
		case 0x9A: imp();txs();break;
		case 0x9D: absx();sta();break;
		case 0xA0: imm();ldy();break;
		case 0xA1: indx();lda();break;
		case 0xA2: imm();ldx();break;
		case 0xA4: zp();ldy();break;
		case 0xA5: zp();lda();break;
		case 0xA6: zp();ldx();break;
		case 0xA8: imp();tay();break;
		case 0xA9: imm();lda();break;
		case 0xAA: imp();tax();break;
		case 0xAC: abso();ldy();break;
		case 0xAD: abso();lda();break;
		case 0xAE: abso();ldx();break;
		case 0xB0: rel();bcs();break;
		case 0xB1: indy();lda();break;
		case 0xB4: zpx();ldy();break;
		case 0xB5: zpx();lda();break;
		case 0xB6: zpy();ldx();break;
		case 0xB8: imp();clv();break;
		case 0xB9: absy();lda();break;
		case 0xBA: imp();tsx();break;
		case 0xBC: absx();ldy();break;
		case 0xBD: absx();lda();break;
		case 0xBE: absy();ldx();break;
		case 0xC0: imm();cpy();break;
		case 0xC1: indx();cmp();break;
		case 0xC4: zp();cpy();break;
		case 0xC5: zp();cmp();break;
		case 0xC6: zp();dec();break;
		case 0xC8: imp();iny();break;
		case 0xC9: imm();cmp();break;
		case 0xCA: imp();dex();break;
		case 0xCC: abso();cpy();break;
		case 0xCD: abso();cmp();break;
		case 0xCE: abso();dec();break;
		case 0xD0: rel();bne();break;
		case 0xD1: indy();cmp();break;
		case 0xD5: zpx();cmp();break;
		case 0xD6: zpx();dec();break;
		case 0xD8: imp();cld();break;
		case 0xD9: absy();cmp();break;
		case 0xDD: absx();cmp();break;
		case 0xDE: absx();dec();break;
		case 0xE0: imm();cpx();break;
		case 0xE1: indx();sbc();break;
		case 0xE4: zp();cpx();break;
		case 0xE5: zp();sbc();break;
		case 0xE6: zp();inc();break;
		case 0xE8: imp();inx();break;
		case 0xE9: imm();sbc();break;
		case 0xEB: imm();sbc();break;
		case 0xEC: abso();cpx();break;
		case 0xED: abso();sbc();break;
		case 0xEE: abso();inc();break;
		case 0xF0: rel();beq();break;
		case 0xF1: indy();sbc();break;
		case 0xF5: zpx();sbc();break;
		case 0xF6: zpx();inc();break;
		case 0xF8: imp();sed();break;
		case 0xF9: absy();sbc();break;
		case 0xFD: absx();sbc();break;
		case 0xFE: absx();inc();break;
		}
#ifdef USE_TIMING
	//	clockgoal6502 -= (int32_t)pgm_read_byte_near(ticktable + opcode);
#endif
		if(GetVsyncFlag()){
			ClearVsyncFlag();
			on_keypressed(kb_read(KB_SEND_END));
			//uint8_t t = kb_readByte(KB_SEND_END);
			//dbgb(10,10,t);
			//if(t){while(1);}
			if(!sysram[0xCC]){//cursor enabled
				if(!sysram[0xCD]--){
					uint8_t phase = sysram[0xCF];//0 if reversd, 1 if not
					uint8_t cuc = sysram[0xCE];//character under cursor
					uint16_t pos = ((sysram[0xD2] << 8) + sysram[0xD1] - VIDEOADDR) + sysram[0xD3];
					phase ^= 1;
					if(phase)
						cuc = vram[pos];
				//	vram[pos] = cuc | (phase << 7);//invert it
					sysram[0xCD] = 20;//blink phase
				}
			}

//			dbgb(10,4,sysram[0x2A]);

//			dbgb(10,2,sysram[0x2C]);dbgb(12,2,sysram[0x2B]);//start of BASIC(0x2A must be 0)
//	sysram[0x38]=0x08;sysram[0x37] = 0x2A;		dbgb(10,3,sysram[0x38]);dbgb(12,3,sysram[0x37]);//end of BASIC
			//dbgb(10,3,sysram[0x);
		}
	}
}


