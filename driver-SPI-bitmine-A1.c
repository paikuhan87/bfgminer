/*
 * cgminer SPI driver for Bitmine.ch A1 devices
 *
 * Copyright 2013 Zefir Kurtisi <zefir.kurtisi@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version.  See COPYING for more details.
 */

#include "config.h"

#include <stdlib.h>
#include <assert.h>
#include <fcntl.h>
#include <limits.h>
#include <unistd.h>
#include <stdbool.h>
#ifdef HAVE_CURSES
#include <curses.h>
#endif

#include <linux/spi/spidev.h>

#include "deviceapi.h"
#include "logging.h"
#include "miner.h"

#include "lowl-spi.h"

#define USE_BITMINE_A1
#define USE_ST_MCU
#define USE_GPIO_CS

#ifdef USE_ST_MCU

//features
#define ST_MCU_EN //using ST MCU to control ASIC
#define A1_PLL_CLOCK_EN //config A1's clip PLL clock

#define API_STATE_EN //support show extra ASIC information
#define A1_SPI_SPEED_EN //support change Master SPI speed
#define A1_DIFFICULTY_EN
#define HW_RESET_EN  // hardware reset to STMCU
#define TEST_PIN_EN  // using test pin to test somethings

#define A1_CHK_CMD_TM //check command timeout

#define A1_TEST_MODE_EN 


#define A1_TEMP_EN //support read temperature from ASIC
#ifdef A1_TEMP_EN 
#define A1_CUTOFF_TEMP_CHIP  // don't send job when higher than cutoff temperature
#define A1_SHUTDOWN_ASIC_BD_1  // shutdown ASIC when sigle chip higher than cutoff temperature > 15
#define A1_SHUTDOWN_ASIC_BD_2  // shutdown ASIC when more chips higher than cutoff temperature > 10
#endif

#define A1_DELAY_RESET 6000 // 6 sec
#define A1_MAX_SPEED 200

#ifdef HW_RESET_EN
#define ASIC_RESET_PIN_GPIO  3
#endif

#endif //USE_ST_MCU

#define MAX_POLL_NUM   20 // 2


//#define TEST_DELAY
#define A1_ALL_DELAY 1 // 2 sec
#define MAX_ASIC_BOARD   6

#define MAX_JOB  4
#define MAX_CORE_NUMS 32
#define MAX_ASIC_NUMS 8 // Max ASIC chip in a chain



#define TEST_PIN_GPIO  23



#ifdef ST_MCU_EN
#define ST_MCU_NULL   0
#define ST_MCU_PRE_HEADER 1
static uint8_t stMcu=ST_MCU_NULL;
static int32_t stTempCut=-1;
static int testMode=-1;
static bool testFirstTime=true;

#endif
#define PRE_HEADER  0xb5 // send Pre-header first


#ifdef USE_GPIO_CS
int opt_gcs = 9;  // -1 not gpio cs
#endif

#ifdef USE_ST_MCU  //Extra Option
int opt_stmcu = 0;  // -1 No ST MCU controller
int opt_diff = 5;  // 1:Difficults
int opt_spiSpeed = -1;  // -1 Default
int opt_A1Pll = 850;  // -1 Default
bool opt_hwReset = false;  // -1 No hardware reset, 1:Hardware reset
int opt_test = -1;  // -1 test mode, 1:run test mode
int opt_tempCut = 66;  // -1 no cut off temperature, [no]:cutoff temperature
#endif


BFG_REGISTER_DRIVER(bitmineA1_drv)

#ifdef USE_GPIO_CS

#define ASIC_SPI_CS0  14
#define ASIC_SPI_CS1  15
#define ASIC_SPI_CS2  18
#define ASIC_SPI_EN   25

static pthread_mutex_t gpio_lock;

typedef enum {
	G_CS_0,
	G_CS_1,
	G_CS_2,
	G_CS_3,
	G_CS_4,
	G_CS_5,
	G_CS_6,
	G_CS_7,
	G_CS_MAX,
} G_CS_t;

static void gpio_CS_Setoutput(void)
{
	bfg_gpio_setpin_output(ASIC_SPI_CS0);
	bfg_gpio_setpin_output(ASIC_SPI_CS1);
	bfg_gpio_setpin_output(ASIC_SPI_CS2);
}

static
int gpio_CS_Init(void)
{
	RUNONCE(1);
	
	mutex_init(&gpio_lock);
	
	bfg_gpio_setpin_output(ASIC_SPI_EN);
	bfg_gpio_set_low(1<<ASIC_SPI_EN);
	
	bfg_gpio_setpin_output(ASIC_SPI_CS0);
	bfg_gpio_set_low(1<<ASIC_SPI_CS0);
	
	bfg_gpio_setpin_output(ASIC_SPI_CS1);
	bfg_gpio_set_low(1<<ASIC_SPI_CS1);
	
	bfg_gpio_setpin_output(ASIC_SPI_CS2);
	bfg_gpio_set_low(1<<ASIC_SPI_CS2);
	
	return 0;
}

#define GPIO_CS_MASK    (1<<ASIC_SPI_CS0|1<<ASIC_SPI_CS1|1<<ASIC_SPI_CS2)
#define GPIO_CS_CLR     GPIO_CS_MASK

#define SET_CS0_BIT0    GPIO_CS_CLR
#define SET_CS0_BIT1    (0<<ASIC_SPI_CS0|1<<ASIC_SPI_CS1|0<<ASIC_SPI_CS2)
#define GPIO_CS0        SET_CS0_BIT1

#define SET_CS1_BIT0    GPIO_CS_CLR
#define SET_CS1_BIT1    (1<<ASIC_SPI_CS0|0<<ASIC_SPI_CS1|0<<ASIC_SPI_CS2)
#define GPIO_CS1        SET_CS1_BIT1

#define SET_CS2_BIT0    GPIO_CS_CLR
#define SET_CS2_BIT1    (0<<ASIC_SPI_CS0|0<<ASIC_SPI_CS1|0<<ASIC_SPI_CS2)
#define GPIO_CS2        SET_CS2_BIT1

#define SET_CS3_BIT0    GPIO_CS_CLR
#define SET_CS3_BIT1    (1<<ASIC_SPI_CS0|1<<ASIC_SPI_CS1|0<<ASIC_SPI_CS2)
#define GPIO_CS3        SET_CS3_BIT1

#define SET_CS4_BIT0    GPIO_CS_CLR
#define SET_CS4_BIT1    (0<<ASIC_SPI_CS0|1<<ASIC_SPI_CS1|1<<ASIC_SPI_CS2)
#define GPIO_CS4        SET_CS4_BIT1

#define SET_CS5_BIT0    GPIO_CS_CLR
#define SET_CS5_BIT1    (0<<ASIC_SPI_CS0|0<<ASIC_SPI_CS1|1<<ASIC_SPI_CS2)
#define GPIO_CS5        SET_CS5_BIT1


#define SET_CS6_BIT0    GPIO_CS_CLR
#define SET_CS6_BIT1    (1<<ASIC_SPI_CS0|0<<ASIC_SPI_CS1|1<<ASIC_SPI_CS2)
#define GPIO_CS6        SET_CS6_BIT1

#define SET_CS7_BIT0    GPIO_CS_CLR
#define SET_CS7_BIT1    (1<<ASIC_SPI_CS0|1<<ASIC_SPI_CS1|1<<ASIC_SPI_CS2)
#define GPIO_CS7        SET_CS7_BIT1

struct cs_config {
	uint32_t clr;
	uint32_t set;
	uint32_t read;  // key hold time in sec
};
struct cs_config CSs[G_CS_MAX] = {
	{.clr=SET_CS0_BIT0,.set=SET_CS0_BIT1,.read=GPIO_CS0},
	{.clr=SET_CS1_BIT0,.set=SET_CS1_BIT1,.read=GPIO_CS1},
	{.clr=SET_CS2_BIT0,.set=SET_CS2_BIT1,.read=GPIO_CS2},
	{.clr=SET_CS3_BIT0,.set=SET_CS3_BIT1,.read=GPIO_CS3},
	{.clr=SET_CS4_BIT0,.set=SET_CS4_BIT1,.read=GPIO_CS4},
	{.clr=SET_CS5_BIT0,.set=SET_CS5_BIT1,.read=GPIO_CS5},
	{.clr=SET_CS6_BIT0,.set=SET_CS6_BIT1,.read=GPIO_CS6},
	{.clr=SET_CS7_BIT0,.set=SET_CS7_BIT1,.read=GPIO_CS7},
};

static void gpioChipSelectOn(G_CS_t cs)
{
	static G_CS_t preCs = G_CS_MAX;
	if (opt_gcs < 0)
		return;
	
	uint8_t ret;
	struct cs_config *csCfg;
	
	if (cs >= G_CS_MAX)
		return;  // out of range
	
	// make stable in chip select IC
	if (preCs == cs)
		return;  // not need change
	
	gpio_CS_Setoutput();
	
	csCfg = &CSs[cs];
	
	do {
		bfg_gpio_set_low(csCfg->clr);
		bfg_gpio_set_low(csCfg->clr);
		
		bfg_gpio_set_high(csCfg->set);
		bfg_gpio_set_high(csCfg->set);
		
		bfg_gpio_get();
		ret = ((bfg_gpio_get() & GPIO_CS_MASK) != csCfg->read);
		if (ret)
			applog(LOG_ERR, "Cannot set CS Pin(%d):0x%X",
			       cs, bfg_gpio_get() & GPIO_CS_MASK);
	} while(ret);
	
	// make stable in chip select IC
	if (preCs != cs)
		preCs = cs;
	
	bfg_gpio_setpin_output(ASIC_SPI_EN);
	bfg_gpio_set_low(1<<ASIC_SPI_EN);
}
#endif

/********** work queue */
struct work_ent {
	struct work *work;
	struct work_ent *next;
	struct work_ent *prev;
};

struct work_queue {
	int num_elems;
	struct work_ent *worklist;
};


static bool wq_enqueue(struct work_queue *wq, struct work *work)
{
	if (work == NULL)
		return false;
	struct work_ent *we = malloc(sizeof(*we));
	if (we == NULL)
		return false;
	we->work = work;
	DL_APPEND(wq->worklist, we);
	wq->num_elems++;
	return true;
}

static struct work *wq_dequeue(struct work_queue *wq)
{
	if (wq == NULL)
		return NULL;
	if (wq->num_elems == 0)
		return NULL;
	struct work_ent *we;
	we = wq->worklist;
	struct work *work = we->work;

	DL_DELETE(wq->worklist, we);
	free(we);
	wq->num_elems--;
	return work;
}

/********** chip and chain context structures */
#ifdef A1_TEST_MODE_EN
#define ASIC_BOARD_OK 			0x00000000
//#define NO_ASIC_BOARD 			0x00000010
//#define ASIC_BOARD_TESTING		0x00000001
#define ERROR_CHIP	 			0x00000001
#define ERROR_CORE 				0x00000002
#define ERROR_TEMP_SENSOR	 	0x00000004
#define ERROR_BOARD			 	0x00000008
#define ERROR_OVERHEAD			 	0x00000010


//#define ERROR_OVERHEAT		 	0x00000008

#endif

#ifdef A1_TEMP_EN	
struct Alarm {
	struct timeval tv_start; //alarm start time
	int overheat; //overheat alarm
	int timeout; //sec
	int cutoffTemp; //cutoff temperature 	
};
#endif	

struct A1_chip {
	int num_cores;
	int last_queued_id;
	struct work *work[MAX_JOB];
	/* stats */
	int hw_errors;
	int stales;
	int nonces_found;
	int nonce_ranges_done;
	uint8_t reg[6]; //latest update from ASIC
#ifdef A1_TEMP_EN	
	struct Alarm alarm[2]; // two temp alarm mode
	int temp; //temperature
#endif	

};
#if 0
struct timeval {
	__kernel_time_t		tv_sec;		/* seconds */
	__kernel_suseconds_t	tv_usec;	/* microseconds */
};
#endif
struct spi_config {
	int bus;
	int cs_line;
	uint8_t mode;
	uint32_t speed;
	uint8_t bits;
	uint16_t delay;
	uint8_t gpio_cs;
};

static const struct spi_config default_spi_config = {
	.mode           = SPI_MODE_0,
	.speed          = 4000000,
	.bits           = 8,
};

struct spi_ctx {
	struct spi_port *port;
	struct spi_config config;
};

struct A1_chain {
	struct cgpu_info *cgpu;
	int num_chips;
	int num_cores;
	uint8_t spi_tx[128];
	uint8_t spi_rx[128];
#ifdef A1_PLL_CLOCK_EN 	
	uint16_t regPll;
#endif

	struct spi_ctx *spi_ctx;
	struct A1_chip *chips;
	pthread_mutex_t lock;

	struct work_queue active_wq;
#ifdef A1_CHK_CMD_TM
	struct timeval tvStart;
	struct timeval tvEnd;
	struct timeval tvDiff;
#endif
//	int reinit;
	int restart;
#ifdef A1_TEMP_EN	
	int cutTemp; //cutoff temperature 
	struct Alarm alarm;
	int shutdown;
#endif
#ifdef A1_TEST_MODE_EN
	uint32_t status;
#endif
};

enum A1_command {
	A1_BIST_START		= 0x01,
	A1_BIST_FIX		= 0x03,
	A1_RESET		= 0x04,
	A1_WRITE_JOB		= 0x07,
	A1_READ_RESULT		= 0x08,
	A1_WRITE_REG		= 0x09,
	A1_READ_REG		= 0x0a,
	A1_READ_REG_RESP	= 0x1a,
};

void A1_ConfigA1PLLClock(int optPll);
void A1_SetA1PLLClock(struct A1_chain *a1,int pllClkIdx);
static bool cmd_WRITE_REG(struct A1_chain *a1, uint8_t chip, uint8_t *reg);
static void A1_reinit_device(struct cgpu_info *cgpu);
void A1_reinit(void);	

/********** temporary helper for hexdumping SPI traffic */
#define hexdump a1hexdump
#define DEBUG_HEXDUMP 1
static void hexdump(char *prefix, uint8_t *buff, int len)
{
#if DEBUG_HEXDUMP
	static char line[2048];
	char *pos = line;
	int i;
	if (len < 1)
		return;

	pos += sprintf(pos, "%s: %d bytes:", prefix, len);
	for (i = 0; i < len; i++) {
		if (i > 0 && (i % 32) == 0)
			pos += sprintf(pos, "\n\t");
		pos += sprintf(pos, "%.2X ", buff[i]);
	}
	applog(LOG_DEBUG, "%s", line);
#endif
}

#ifdef A1_PLL_CLOCK_EN

#define A1_PLL_POSTDIV_MASK		0b11
#define A1_PLL_PREDIV_MASK		0b11111
#define A1_PLL_FBDIV_H_MASK		0b111111111

// Register masks for A1
//Reg32 Upper
#define A1_PLL_POSTDIV							(46-32)	// (2) pll post divider
#define A1_PLL_PREDIV							(41-32)	// (5) pll pre divider
#define A1_PLL_FBDIV_H							(32-32)	// (9) pll fb divider
//Reg32 Lower
#define A1_PLL_PD								25	// (1) PLL Reset
#define A1_PLL_LOCK								24	// (1) PLL Lock status (1: PLL locked , 0: PLL not locked )
#define A1_INTERNAL_SPI_FREQUENCY_CONFIG		18 	// (1) 1:  Internal  SPI  frequency  is System  clock  frequency  divide 64 
													//      0:  Internal  SPI  frequency  is System  clock  frequency  divide 128

#define A1_BACKUP_QUEUE_FULL_FLAG				17	// (1) 1:Backup queue is full , 0: Backup queue empty 
#define A1_ACTIVE_QUEUE_BUSY_FLAG				16	// (1) 1:Current active queue busy, 0:  Current  active  queue  not busy 
#define A1_BACKUP_QUEUE_JOB_ID					12	// (4) Job  id  corresponding  with  the backup queue  
#define A1_ACTIVE_QUEUE_JOB_ID					8	// (4) Job  id  corresponding  with  the current active queue 
#define A1_GOOD_CORE_NO       					0 	// (8) Good cores inside the chip, 32 maximum

typedef enum
{
    A1_PLL_CLOCK_1000MHz  = 0,
    A1_PLL_CLOCK_950HMz,
	A1_PLL_CLOCK_900HMz,
	A1_PLL_CLOCK_850HMz,
	A1_PLL_CLOCK_800HMz,  //default
	A1_PLL_CLOCK_750HMz,
	A1_PLL_CLOCK_700HMz,
	A1_PLL_CLOCK_650HMz,
	A1_PLL_CLOCK_600HMz,
	A1_PLL_CLOCK_550HMz,
	A1_PLL_CLOCK_500HMz,
	A1_PLL_CLOCK_100HMz,
	A1_PLL_CLOCK_MAX,
} A1_PLL_CLOCK;

static uint8_t a1Pll=A1_PLL_CLOCK_800HMz;

struct PLL_Clock {
	uint32_t speedMHz;  // unit MHz
	uint32_t hastRate;  // divider 10
	uint16_t pll_reg;// pll_postdiv(2)|pll_prediv(5)|pll_fbdiv(9)
};
// PLL programming guide table with input reference clock frequency 12MHz

/*
1GHz  32.0GH/s  2!|b01  5!|b00011   9!|b011111010 
950MHz  30.4GH/s  2!|b10  5!|b00011  9!|b111011011 
900MHz  28.8GH/s  2!|b01  5!|b00001  9!|b001001011 
850MHz  27.2GH/s  2!|b10  5!|b00011  9!|b110101001 
800MHz(Default)  25.6GH/s  2!|b01  5!|b00011  9!|b011001000 
750MHz  24.0GH/s  2!|b10  5!|b00001  9!|b001111101 
700MHz  22.4GH/s  2!|b01  5!|b00011  9!|b010101111 
650MHz  20.8GH/s  2!|b10  5!|b00011  9!|b101000101 
600MHz  19.2GH/s  2!|b01  5!|b00001  9!|b000110010 
550MHz  17.6GH/s  2!|b10  5!|b00011  9!|b100010011 
500MHz  16GH/s  2!|b10  5!|b00011  9!|b011111010 
100MHz  3.2GH/s  2'b11  5'b00011  9!|b001100100

*/

#if 1
#define A1_PLL(postdiv,prediv,fbdiv) ((postdiv<<14)|(prediv<<9)|fbdiv)
const struct PLL_Clock PLL_Clk_12Mhz[A1_PLL_CLOCK_MAX]={
	{1000,320,A1_PLL(0b01,0b00011,0b011111010)},
	{ 950,304,A1_PLL(0b10,0b00011,0b111011011)},	
	{ 900,288,A1_PLL(0b01,0b00001,0b001001011)},
	{ 850,272,A1_PLL(0b10,0b00011,0b110101001)},
	{ 800,256,A1_PLL(0b01,0b00011,0b011001000)},  //default
	{ 750,240,A1_PLL(0b10,0b00001,0b001111101)},
	{ 700,224,A1_PLL(0b01,0b00011,0b010101111)},
	{ 650,208,A1_PLL(0b10,0b00011,0b101000101)}, 
	{ 600,192,A1_PLL(0b01,0b00001,0b000110010)},
	{ 550,176,A1_PLL(0b10,0b00011,0b100010011)},
	{ 500,160,A1_PLL(0b10,0b00011,0b011111010)},
	{ 100, 32,A1_PLL(0b11,0b00011,0b001100100)}
};

#else
struct PLL_Clock PLL_Clk_12Mhz[A1_PLL_CLOCK_MAX]={
{1000,320,0b01,0b00011,0b011111010},
{ 950,304,0b10,0b00011,0b111011011},	
{ 900,288,0b01,0b00001,0b001001011},
{ 850,272,0b10,0b00011,0b110101001},
{ 800,256,0b01,0b00011,0b011001000},  //default
{ 750,240,0b10,0b00001,0b001111101},
{ 700,224,0b01,0b00011,0b010101111},
{ 650,208,0b10,0b00011,0b101000101}, 
{ 600,192,0b01,0b00001,0b000110010},
{ 550,176,0b10,0b00011,0b100010011},
{ 500,160,0b10,0b00011,0b011111010},
{ 100, 32,0b11,0b00011,0b001100100}
};
#endif

// PLL programming guide table with input reference clock frequency 16MHz
/*
	1GHz  32.0GH/s	2!|b01	5!|b00100  9!|b011111010 
	950MHz	30.4GH/s  2!|b10  5!|b00100  9!|b111011011 
	900MHz	28.8GH/s  2!|b01  5!|b00100  9!|b011100001 
	850MHz	27.2GH/s  2!|b10  5!|b00100  9!|b110101001 
	800MHz	25.6GH/s  2!|b01  5!|b00100  9!|b011001000 
	750MHz	24.0GH/s  2!|b10  5!|b00100  9!|b101110111 
	700MHz	22.4GH/s  2!|b01  5!|b00100  9!|b010101111 
	650MHz	20.8GH/s  2!|b10  5!|b00100  9!|b101000101 
	600MHz	19.2GH/s  2!|b01  5!|b00100  9!|b010010110 
	550MHz	17.6GH/s  2!|b10  5!|b00100  9!|b100010011 
	500MHz	16GH/s	2!|b10	5!|b00100  9!|b011111010 
	100MHz	3.2GH/s  2'b11	5'b00100  9!|b001100100
*/
struct PLL_Clock PLL_Clk_16Mhz[A1_PLL_CLOCK_MAX]= {
	{1000,320,A1_PLL(0b01,0b00100,0b011111010)},
	{ 950,304,A1_PLL(0b10,0b00100,0b111011011)},
	{ 900,288,A1_PLL(0b01,0b00100,0b011100001)},
	{ 850,272,A1_PLL(0b10,0b00100,0b110101001)},
	{ 800,256,A1_PLL(0b01,0b00100,0b011001000)},
	{ 750,240,A1_PLL(0b10,0b00100,0b101110111)},
	{ 700,224,A1_PLL(0b01,0b00100,0b010101111)},
	{ 650,208,A1_PLL(0b10,0b00100,0b101000101)},
	{ 600,192,A1_PLL(0b01,0b00100,0b010010110)},
	{ 550,176,A1_PLL(0b10,0b00100,0b100010011)},
	{ 500,160,A1_PLL(0b10,0b00100,0b011111010)}, 
	{ 100, 32,A1_PLL(0b11,0b00100,0b001100100)},
};
	


uint64_t A1_SetPLLClock(uint64_t  reg64, uint8_t pll_postdiv,uint8_t pll_prediv,uint8_t pll_fbdiv)
{
	uint64_t  newReg64; 


	uint64_t  mask = (A1_PLL_POSTDIV_MASK<< A1_PLL_POSTDIV)| \
					 (A1_PLL_PREDIV_MASK<< A1_PLL_PREDIV)| \
					 (A1_PLL_FBDIV_H_MASK<< A1_PLL_FBDIV_H);

	uint64_t  value  = ((pll_postdiv&A1_PLL_POSTDIV_MASK)<< A1_PLL_POSTDIV)| \
					 ((pll_prediv&A1_PLL_PREDIV_MASK)<< A1_PLL_PREDIV)| \
					 ((pll_fbdiv&A1_PLL_FBDIV_H_MASK)<< A1_PLL_FBDIV_H);

	newReg64=(reg64 & ~mask) |value;

	return newReg64;

}

	
void A1_ConfigA1PLLClock(int optPll)
{
	int i;
	if(optPll>0)
	{
		a1Pll=A1_PLL_CLOCK_1000MHz; 
		if(optPll<=PLL_Clk_12Mhz[A1_PLL_CLOCK_100HMz].speedMHz)
		{
			a1Pll=A1_PLL_CLOCK_100HMz; //found
		}
		else
		{
			for(i=1;i<A1_PLL_CLOCK_MAX;i++)
		{
				if((optPll>=PLL_Clk_12Mhz[i].speedMHz)&&(optPll<PLL_Clk_12Mhz[i-1].speedMHz))
			{
				a1Pll=i; //found
					break;
			}
		}
	}
		// Max speed if no found
//		if(i>=A1_PLL_CLOCK_MAX)

	applog(LOG_NOTICE, "A1 = %d,%d",optPll,i);


	applog(LOG_NOTICE, "A1 PLL Clock = %dMHz, HastRate = %d.%1dGH/s",\
		PLL_Clk_12Mhz[a1Pll].speedMHz, \
		PLL_Clk_12Mhz[a1Pll].hastRate/10,PLL_Clk_12Mhz[a1Pll].hastRate%10); 

	}
	
}

void A1_SetA1PLLClock(struct A1_chain *a1,int pllClkIdx)
{

	uint8_t i;
	struct A1_chip *chip;
	
	if(pllClkIdx<0||pllClkIdx>A1_PLL_CLOCK_MAX) //out of range
		return;
#if 0 // not defined
	a1->regPll=bswap_16(PLL_Clk_12Mhz[pllClkIdx].pll_reg);
#else
	a1->regPll=((((PLL_Clk_12Mhz[pllClkIdx].pll_reg) >> 8) & 0x00FF) | (((PLL_Clk_12Mhz[pllClkIdx].pll_reg) << 8) & 0xFF00));
#endif	

	if(a1->chips == NULL)
		return;

	for(i=0;i<a1->num_chips;i++)
	{
		chip = &a1->chips[i];
		memcpy(chip->reg,(uint8_t*)&a1->regPll,2);
		cmd_WRITE_REG(a1, i, chip->reg);
	}
}
#endif //A1_PLL_CLOCK_EN



/********** upper layer SPI functions */

static
bool spi_transfer(struct spi_ctx * const spi_ctx, const void * const tx, void * const rx, const size_t len)
{
	struct spi_port * const port = spi_ctx->port;
#ifdef USE_GPIO_CS
	mutex_lock(&gpio_lock);
	gpioChipSelectOn(spi_ctx->config.gpio_cs);
#endif
	port->speed = spi_ctx->config.speed;
	port->delay = spi_ctx->config.delay;
	port->mode  = spi_ctx->config.mode;
	port->bits  = spi_ctx->config.bits;
	spi_clear_buf(port);
	if (tx)
		spi_emit_buf(port, tx, len);
	else
		spi_emit_nop(port, len);
	bool rv = spi_txrx(port);
	if (rv)
		memcpy(rx, port->spibuf_rx, len);
#ifdef USE_GPIO_CS
	mutex_unlock(&gpio_lock);
#endif
	return rv;
}

static
void spi_exit(struct spi_ctx * __maybe_unused const spi_ctx)
{
	
}

static bool spi_send_command(struct A1_chain *a1, uint8_t cmd, uint8_t addr,
			     uint8_t *buff, int len)
{
	int tx_len;

if(stMcu==ST_MCU_PRE_HEADER)
{

	memset(a1->spi_tx + 2, 0, len);
	a1->spi_tx[0] = PRE_HEADER;
	a1->spi_tx[1] = PRE_HEADER;

	a1->spi_tx[2] = cmd;
	a1->spi_tx[3] = addr;
	if (len > 0 && buff != NULL)
		memcpy(a1->spi_tx + 4, buff, len);
	tx_len = (4 + len + 1) & ~1;
}
else
{

	memset(a1->spi_tx + 2, 0, len);
	a1->spi_tx[0] = cmd;
	a1->spi_tx[1] = addr;
	if (len > 0 && buff != NULL)
		memcpy(a1->spi_tx + 2, buff, len);
	tx_len = (2 + len + 1) & ~1;

}

	applog(LOG_DEBUG, "Processing command 0x%02x%02x", cmd, addr);
	bool retval = spi_transfer(a1->spi_ctx, a1->spi_tx, a1->spi_rx, tx_len);
	hexdump("TX", a1->spi_tx, tx_len);
	hexdump("RX", a1->spi_rx, tx_len);

	if(retval==false)
		applog(LOG_ERR, "SPI Err:spi_send_command 0x%2x",cmd);

	return retval;
}

static bool spi_poll_result(struct A1_chain *a1, uint8_t cmd,
			    uint8_t chip_id, int len)
{
	int i;
	int pollLen;
	pollLen=MAX_POLL_NUM*a1->num_chips;
	if(pollLen<=0)
		pollLen=MAX_POLL_NUM;
		
	for(i = 0; i < pollLen; i++) {
		bool s = spi_transfer(a1->spi_ctx, NULL, a1->spi_rx, 2);
		hexdump("TX", a1->spi_tx, 2);
		hexdump("RX", a1->spi_rx, 2);
		if (!s)
		{
			applog(LOG_ERR, "SPI(cs%d) Err:spi_poll_result 0x%2x",a1->spi_ctx->config.gpio_cs,cmd);
			return false;
		}
		if (a1->spi_rx[0] == cmd && a1->spi_rx[1] == chip_id) {
			applog(LOG_DEBUG, "Cmd 0x%02x ACK'd", cmd);
			if (len == 0)
				return true;
			s = spi_transfer(a1->spi_ctx, NULL,
					 a1->spi_rx + 2, len);
			hexdump("RX", a1->spi_rx + 2, len);
			if (!s)
			{
				applog(LOG_ERR, "SPI(cs%d) Err:spi_poll_result 0x%2x",a1->spi_ctx->config.gpio_cs,cmd);
				return false;
			}
			hexdump("poll_result", a1->spi_rx, len + 2);
			return true;
		}
//		cgsleep_us(10);

	}
	applog(LOG_WARNING, "Failure(cs%d)(%d): missing ACK for cmd 0x%02x", a1->spi_ctx->config.gpio_cs,pollLen,cmd);
	return false;
}

static bool spi_poll_result_fast(struct A1_chain *a1, uint8_t cmd,
			    uint8_t chip_id, int len,uint32_t timeout)
{
	int i;
	int pollLen;
	bool firstTime=true;
	pollLen=MAX_POLL_NUM*a1->num_chips;
	if(pollLen<=0)
		pollLen=MAX_POLL_NUM;
		
	for(i = 0; i < pollLen; i++) {
		bool s = spi_transfer(a1->spi_ctx, NULL, a1->spi_rx, 2);
		hexdump("TX", a1->spi_tx, 2);
		hexdump("RX", a1->spi_rx, 2);
		if (!s)
		{
			applog(LOG_ERR, "SPI(cs%d) Err:spi_poll_result_fast 0x%2x",a1->spi_ctx->config.gpio_cs,cmd);
			return false;
		}

		if (a1->spi_rx[0] == 0xff&& a1->spi_rx[1] == 0xff)
			{
				applog(LOG_ERR, "SPI(cs%d) no device",a1->spi_ctx->config.gpio_cs);
				return false;
			}

		if (a1->spi_rx[0] == cmd && a1->spi_rx[1] == chip_id) {
			applog(LOG_DEBUG, "Cmd 0x%02x ACK'd", cmd);
			if (len == 0)
				return true;
			s = spi_transfer(a1->spi_ctx, NULL,
					 a1->spi_rx + 2, len);
			hexdump("RX", a1->spi_rx + 2, len);
			if (!s)
			{
				applog(LOG_ERR, "SPI(cs%d) Err2:spi_poll_result_fast 0x%2x",a1->spi_ctx->config.gpio_cs,cmd);
				return false;
			}
			hexdump("poll_result", a1->spi_rx, len + 2);
			return true;
		}
		if(firstTime)
		{
			cgsleep_ms(timeout);
			firstTime=false;
		}
//		cgsleep_ms(10);
//		cgsleep_us(500);
//	cgsleep_us(10);

	}
//	applog(LOG_WARNING, "Failure: missing ACK for cmd 0x%02x", cmd);
	applog(LOG_WARNING, "Failure(cs%d)(%d): missing ACK for cmd 0x%02x", a1->spi_ctx->config.gpio_cs,pollLen,cmd);

	return false;
}

/********** A1 SPI commands */
#ifdef ST_MCU_EN
static bool cmd_BIST_START(struct A1_chain *a1)
{
	bool ret;

#ifdef A1_CHK_CMD_TM
	cgtime(&a1->tvStart);
#endif

	ret=spi_send_command(a1, A1_BIST_START, 0x00, NULL, 0);
//	ret=spi_poll_result_fast(a1, A1_BIST_START, 0x00, 2,A1_DELAY_RESET);
	ret=spi_poll_result(a1, A1_BIST_START, 0x00, 2);
	if(ret==false) 	
	{
		cgtime(&a1->tvEnd);
		timersub(&a1->tvEnd, &a1->tvStart, &a1->tvDiff);
		applog(LOG_WARNING, "ACK(cs%d) timeout:cmd_BIST_START-%ld.%04lds",
		       a1->spi_ctx->config.gpio_cs,
		       (long)a1->tvDiff.tv_sec, (long)a1->tvDiff.tv_usec);
	}
	return ret;
}

static bool cmd_BIST_FIX_BCAST(struct A1_chain *a1)
{
	bool ret;

#ifdef A1_CHK_CMD_TM
	cgtime(&a1->tvStart);
#endif


	ret=spi_send_command(a1, A1_BIST_FIX, 0x00, NULL, 0);
//	ret=spi_poll_result_fast(a1, A1_BIST_FIX, 0x00, 0,A1_DELAY_RESET);
	ret=spi_poll_result(a1, A1_BIST_FIX, 0x00, 0);
	if(ret==false) 	
	{
		cgtime(&a1->tvEnd);
		timersub(&a1->tvEnd, &a1->tvStart, &a1->tvDiff);
		applog(LOG_WARNING, "ACK(%d) timeout:cmd_BIST_FIX_BCAST-%ld.%04lds",
		       a1->spi_ctx->config.gpio_cs,
		       (long)a1->tvDiff.tv_sec, (long)a1->tvDiff.tv_usec);
	}

	return ret;
}

static bool cmd_RESET_BCAST(struct A1_chain *a1)
{
//	struct timeval tv_start, tv_end,tv_diff;

	bool ret;
#ifdef A1_CHK_CMD_TM
	cgtime(&a1->tvStart);
#endif

	ret=spi_send_command(a1, A1_RESET, 0x00, NULL, 0);
#ifdef ST_MCU_EN
	if(ret==false)
	{
		return ret;
	}	

#if 0	
	cgtime(&tv_start);

	cgsleep_ms(A1_DELAY_RESET); 
	cgtime(&tv_end);

	timersub(&tv_end, &tv_start, &tv_diff);

	applog(LOG_WARNING, "Reset time %d\n",tv_diff.tv_sec);
#endif	

#endif

	ret=spi_poll_result_fast(a1, A1_RESET, 0x00, 0,A1_DELAY_RESET);
//	ret=spi_poll_result(a1, A1_RESET, 0x00, 0);

	if(ret==false) 	
	{
		cgtime(&a1->tvEnd);
		timersub(&a1->tvEnd, &a1->tvStart, &a1->tvDiff);
		applog(LOG_WARNING, "ACK(cs%d) timeout:cmd_RESET_BCAST-%ld.%04lds",
		       a1->spi_ctx->config.gpio_cs,
		       (long)a1->tvDiff.tv_sec, (long)a1->tvDiff.tv_usec);
	}


//	if(ret==false) 	applog(LOG_WARNING, "ACK timeout:cmd_RESET_BCAST\n");
	return ret;
}

static bool cmd_WRITE_REG(struct A1_chain *a1, uint8_t chip, uint8_t *reg)
{
	bool ret;

#if 0 //patch A
	return	spi_send_command(a1, A1_WRITE_REG, chip, reg, 6)&&
		spi_poll_result(a1, A1_WRITE_REG, chip, 6);
#else
	ret=spi_send_command(a1, A1_WRITE_REG, chip, reg, 6);
#endif
	return ret;
}

static bool cmd_READ_REG(struct A1_chain *a1, uint8_t chip)
{
	bool ret;
#ifdef TEST_PIN_EN
	bfg_gpio_setpin_output(TEST_PIN_GPIO);
	bfg_gpio_set_high(1<<TEST_PIN_GPIO);
#endif

#ifdef A1_CHK_CMD_TM
	cgtime(&a1->tvStart);
#endif

	ret=spi_send_command(a1, A1_READ_REG, chip, NULL, 0);
#ifdef A1_TEMP_EN	
	ret=spi_poll_result(a1, A1_READ_REG_RESP, chip, 8);
#else
	ret=spi_poll_result(a1, A1_READ_REG_RESP, chip, 6);
#endif	

	if(ret==false) 	
	{
		cgtime(&a1->tvEnd);
		timersub(&a1->tvEnd, &a1->tvStart, &a1->tvDiff);
		applog(LOG_WARNING, "ACK(cs%d) timeout:cmd_READ_REG-%ld.%lds",
		       a1->spi_ctx->config.gpio_cs,
		       a1->tvDiff.tv_sec, (long)a1->tvDiff.tv_usec);
	}
#ifdef TEST_PIN_EN
	bfg_gpio_setpin_output(TEST_PIN_GPIO);
	bfg_gpio_set_low(1<<TEST_PIN_GPIO);
#endif


	return ret;
}

#else

static bool cmd_BIST_START(struct A1_chain *a1)
{
	return	spi_send_command(a1, A1_BIST_START, 0x00, NULL, 0) &&
		spi_poll_result(a1, A1_BIST_START, 0x00, 2);
}

static bool cmd_BIST_FIX_BCAST(struct A1_chain *a1)
{
	return	spi_send_command(a1, A1_BIST_FIX, 0x00, NULL, 0) &&
		spi_poll_result(a1, A1_BIST_FIX, 0x00, 0);
}

static bool cmd_RESET_BCAST(struct A1_chain *a1)
{
	return	spi_send_command(a1, A1_RESET, 0x00, NULL, 0) &&
		spi_poll_result(a1, A1_RESET, 0x00, 0);
}

static bool cmd_WRITE_REG(struct A1_chain *a1, uint8_t chip, uint8_t *reg)
{
	return	spi_send_command(a1, A1_WRITE_REG, chip, reg, 6);
}

static bool cmd_READ_REG(struct A1_chain *a1, uint8_t chip)
{
	return	spi_send_command(a1, A1_READ_REG, chip, NULL, 0) &&
		spi_poll_result(a1, A1_READ_REG_RESP, chip, 6);
}
#endif //GPIO_CS_EN


/********** A1 low level functions */
static void A1_hw_reset(void)
{

#ifdef HW_RESET_EN

		applog(LOG_NOTICE, "ST MCU hardware reset");

		bfg_gpio_setpin_output(ASIC_RESET_PIN_GPIO);
		bfg_gpio_set_high(1<<ASIC_RESET_PIN_GPIO);
	
		cgsleep_ms(200);
	
		bfg_gpio_setpin_output(ASIC_RESET_PIN_GPIO);
		bfg_gpio_set_low(1<<ASIC_RESET_PIN_GPIO);
	
		cgsleep_ms(10000);
		
#else
	/* TODO: issue cold reset */
//	usleep(100000);
#endif
}

static bool is_busy(struct A1_chain *a1, uint8_t chip)
{
	if (!cmd_READ_REG(a1, chip))
		return false;
	return (a1->spi_rx[5] & 0x01) == 0x01;
}


/********** job creation and result evaluation */
#ifdef A1_DIFFICULTY_EN

/*
ffff001d==1
ff7f001d==2
ff5f001d==3
ff3f001d==4
ff1f001d==8
ff0f001d==16
ff07001d==32
ff06001d==37
ff05001d==43
ff04001d==52
ff03001d==65
ff02001d==86
7f02001d=103
ff01001d==129
ffff001c==256
ff00001d==259
*/
static const uint32_t difficult_Tbl[16] = {
		0x1d00ffff,// 1
		0x1d007fff,// 2
		0x1d005fff,// 3
		0x1d003fff,// 4
		0x1d001fff,// 8

		0x1d000fff,// 16
		0x1d0007ff,// 32
		0x1d0006ff,// 37
		0x1d0005ff,// 43
		0x1d0004ff,// 52

		0x1d0003ff,// 65
		0x1d0002ff,// 86
		0x1d00027f,// 103
		0x1d0001ff,// 129
		0x1c00ffff,// 256

		0x1d0000ff,// 259
};

#endif
static uint8_t *create_job(uint8_t chip_id, uint8_t job_id,
			   void *midstate, void *wdata)
{
	static uint8_t job[58] = {
		/* command */
		0x00, 0x00,
		/* midstate */
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		/* wdata */
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00,
		/* start nonce */
		0x00, 0x00, 0x00, 0x00,
		/* difficulty 1 */
		0xff, 0xff, 0x00, 0x1d,
		/* end nonce */
		0xff, 0xff, 0xff, 0xff,
	};
	uint32_t *p1 = (uint32_t *) &job[34];
	uint32_t *p2 = (uint32_t *) wdata;
	uint32_t diffIdx,*diff=(uint32_t*)&job[50]; //difficulty pointer

	job[0] = (job_id << 4) | A1_WRITE_JOB;
	job[1] = chip_id;

	swab256(job + 2, midstate);
	p1 = (uint32_t *) &job[34];
	p2 = (uint32_t *) wdata;
	p1[0] = bswap_32(p2[0]);
	p1[1] = bswap_32(p2[1]);
	p1[2] = bswap_32(p2[2]);

#ifdef A1_DIFFICULTY_EN
	if(opt_diff>=1&&opt_diff<=15)
	{
		diffIdx=opt_diff-1;
		*diff=difficult_Tbl[diffIdx];
	//printf("%d: 0x%.8x\n",i,*diff);printf("%d: 0x%.2x%.2x%.2x%.2x\n",i,job[53],job[52],job[51],job[50]);}
	}
#endif
	return job;
}

/* set work for given chip, returns true if a nonce range was finished */
static bool set_work(struct A1_chain *a1, uint8_t chip_id, struct work *work)
{
	unsigned char *midstate = work->midstate;
	unsigned char *wdata = work->data + 64;

	if(a1->chips == NULL)
		return false;

	struct A1_chip *chip = &a1->chips[chip_id - 1];
	bool retval = false;

	chip->last_queued_id++;
	chip->last_queued_id &= 3;

	if (chip->work[chip->last_queued_id] != NULL) {
		work_completed(a1->cgpu, chip->work[chip->last_queued_id]);
		chip->work[chip->last_queued_id] = NULL;
		retval = true;
	}
	uint8_t *jobdata = create_job(chip_id, chip->last_queued_id + 1,
				      midstate, wdata);

if(stMcu==ST_MCU_PRE_HEADER)
{
	hexdump("JOB", jobdata, 58);
	a1->spi_tx[0]=PRE_HEADER;
	a1->spi_tx[1]=PRE_HEADER;
	memcpy(&a1->spi_tx[2], jobdata, 58);
	if (!spi_transfer(a1->spi_ctx, a1->spi_tx, a1->spi_rx, 60)) {
		/* give back work */
		work_completed(a1->cgpu, work);

		applog(LOG_ERR, "Failed to set work for chip(cs%d) %d.%d",
		       a1->spi_ctx->config.gpio_cs,chip_id, chip->last_queued_id + 1);
		// TODO: what else?
	} else {
		chip->work[chip->last_queued_id] = work;
	}
}
else
{
	hexdump("JOB", jobdata, 58);
	memcpy(a1->spi_tx, jobdata, 58);
	if (!spi_transfer(a1->spi_ctx, jobdata, a1->spi_rx, 58)) {

		/* give back work */
		work_completed(a1->cgpu, work);

		applog(LOG_ERR, "Failed to set work for chip(cs%d) %d.%d",
		       a1->spi_ctx->config.gpio_cs,chip_id, chip->last_queued_id + 1);
		// TODO: what else?
	} else {
		chip->work[chip->last_queued_id] = work;
	}

}
		
	return retval;
}

/* check for pending results in a chain, returns false if output queue empty */
static bool get_nonce(struct A1_chain *a1, uint8_t *nonce,
		      uint8_t *chip, uint8_t *job_id)
{
	int i;

	int pollLen=0;
	pollLen=MAX_POLL_NUM*a1->num_chips;
	if(pollLen<=0)
		pollLen=MAX_POLL_NUM;


#ifdef A1_CHK_CMD_TM
	cgtime(&a1->tvStart);
#endif
	
	if (!spi_send_command(a1, A1_READ_RESULT, 0x00, NULL, 0))
	{
		return false;
	}

	for(i = 0; i < pollLen; i++) {
		memset(a1->spi_tx, 0, 2);
		if (!spi_transfer(a1->spi_ctx, a1->spi_tx, a1->spi_rx, 2))
			{
				applog(LOG_ERR, "SPI Err(cs%d):get_nonce result",a1->spi_ctx->config.gpio_cs);
			return false;
			}
		hexdump("RX", a1->spi_rx, 2);
		if (a1->spi_rx[0] == A1_READ_RESULT && a1->spi_rx[1] == 0x00) {
			applog(LOG_DEBUG, "Output queue empty");
			return false;
		}
		if ((a1->spi_rx[0] & 0x0f) == A1_READ_RESULT &&
#if 1 //Patch C
		    a1->spi_rx[1] != 0) {
#else		    
		    a1->spi_rx[0] != 0) {
#endif		    
			*job_id = a1->spi_rx[0] >> 4;
			*chip = a1->spi_rx[1];

			if (!spi_transfer(a1->spi_ctx, NULL, nonce, 4))
			{
					applog(LOG_ERR, "SPI Err(cs%d):get_nonce",a1->spi_ctx->config.gpio_cs);
				return false;
			}


			applog(LOG_DEBUG, "Got nonce for chip %d / job_id %d",
			       *chip, *job_id);

			return true;
		}
//nouse		cgsleep_us(50);
	}

	cgtime(&a1->tvEnd);
	timersub(&a1->tvEnd, &a1->tvStart, &a1->tvDiff);
	applog(LOG_WARNING, "ACK(cs%d) timeout:get_nonce-%ld.%04lds",
	       a1->spi_ctx->config.gpio_cs,
	       (long)a1->tvDiff.tv_sec, (long)a1->tvDiff.tv_usec);
	return false;
}

/* reset input work queues in chip chain */
static bool abort_work(struct A1_chain *a1)
{
	/*
	 * for now, the proposed input queue reset does not seem to work
	 * TODO: implement reliable abort method
	 * NOTE: until then, we are completing queued work => stales
	 */

	return true;

	int i;
	applog(LOG_WARNING, "abort_work...");
	for (i = 0; i < a1->num_chips; i++) {
		int chip_id = i + 1;
		if (!cmd_READ_REG(a1, i + 1)) {
			applog(LOG_ERR, "Failed to read reg from chip %d",
			       chip_id);
			// TODO: what to do now?
			continue;
		}
		hexdump("A1 RX", a1->spi_rx, 8);
		uint8_t reg[160];
		memset(reg, 0, 160);
		memcpy(reg, a1->spi_rx + 2, 6);
		reg[3] &= 0xfc;
		reg[4] = 0;
		if (!cmd_WRITE_REG(a1, i + 1, reg)) {
			applog(LOG_ERR, "Failed to write reg of chip %d",
			       chip_id);
			// TODO: what to do now?
			continue;
		}
	}
	return cmd_RESET_BCAST(a1);
}

/********** driver interface */
void exit_A1_chain(struct A1_chain *a1)
{
	if (a1 == NULL)
		return;
	
	if(a1->chips)
	free(a1->chips);
	a1->chips = NULL;
	spi_exit(a1->spi_ctx);
	a1->spi_ctx = NULL;
	free(a1);
}
#ifdef A1_TEST_MODE_EN //assume ST_MCU_EN enable
struct A1_chain *init_Test_A1_chain(struct spi_ctx *ctx)
{
	struct A1_chain *a1 = malloc(sizeof(*a1));
	applog(LOG_DEBUG, "A1 init chain");
	if (a1 == NULL) {
		applog(LOG_ERR, "A1_chain allocation error");
		goto failure;
	}
	memset(a1, 0, sizeof(*a1));
	a1->spi_ctx = ctx;
	a1->num_chips = 0;
	a1->num_cores = 0;	
	a1->chips = NULL;
	mutex_init(&a1->lock);
	a1->active_wq.worklist = NULL;

	return a1;

failure:
	if(a1->chips)
	free(a1->chips);
	a1->chips = NULL;
	spi_exit(a1->spi_ctx);
	a1->spi_ctx = NULL;

	return a1;
}

#endif // A1_TEST_MODE_EN //assume ST_MCU_EN enable

struct A1_chain *init_A1_chain(struct spi_ctx *ctx)
{
	int i,num_chips =0;
	struct A1_chain *a1 = malloc(sizeof(*a1));
	applog(LOG_DEBUG, "A1 init chain");
	if (a1 == NULL) {
		applog(LOG_ERR, "A1_chain allocation error");
		goto failure;
	}
	memset(a1, 0, sizeof(*a1));
	a1->spi_ctx = ctx;
	a1->status=ASIC_BOARD_OK;
	a1->shutdown=0;

#ifdef ST_MCU_EN

	if (!cmd_RESET_BCAST(a1))
		goto failure;

	if (!cmd_BIST_START(a1))
		goto failure;
#else

	if (!cmd_RESET_BCAST(a1) || !cmd_BIST_START(a1))
		goto failure;
#endif
	a1->num_chips = a1->spi_rx[3];
	applog(LOG_WARNING, "spidev%d.%d(cs%d): Found %d A1 chips",
	       a1->spi_ctx->config.bus, a1->spi_ctx->config.cs_line,
	       a1->spi_ctx->config.gpio_cs,a1->num_chips);

	a1->num_cores=0;

	if (a1->num_chips == 0)
		goto failure;

	a1->chips = calloc(a1->num_chips, sizeof(struct A1_chip));
	if (a1->chips == NULL) {
		applog(LOG_ERR, "A1_chips allocation error");
		goto failure;
	}
	if (!cmd_BIST_FIX_BCAST(a1))
		goto failure;


	if(a1->num_chips<MAX_ASIC_NUMS)
		a1->status|=ERROR_CHIP;
	

	for (i = 0; i < a1->num_chips; i++) {
		int chip_id = i + 1;
		if (!cmd_READ_REG(a1, chip_id)) {
			applog(LOG_WARNING, "Failed to read register for "
			       "chip %d -> disabling", chip_id);
			a1->chips[i].num_cores = 0;
			continue;
		}
		num_chips++;
		a1->chips[i].num_cores = a1->spi_rx[7];
		if(a1->spi_rx[7]>MAX_CORE_NUMS)
			a1->chips[i].num_cores = MAX_CORE_NUMS;

		if(a1->chips[i].num_cores<MAX_CORE_NUMS)
			a1->status=ERROR_CORE;

		memcpy(a1->chips[i].reg, a1->spi_rx + 2, 6); //keep ASIC register value
#ifdef A1_TEMP_EN	
		a1->chips[i].temp= (a1->spi_rx[8]<<8)|a1->spi_rx[9];
#endif

#ifdef A1_SHUTDOWN_ASIC_BD_1
		a1->chips[i].alarm[0].overheat=0;
		a1->chips[i].alarm[0].timeout=5;
		a1->chips[i].alarm[0].cutoffTemp=stTempCut+15;

#endif
		
#ifdef A1_SHUTDOWN_ASIC_BD_2
		a1->chips[i].alarm[1].overheat=0;
		a1->chips[i].alarm[1].timeout=5;
		a1->chips[i].alarm[1].cutoffTemp=stTempCut+10;
#endif

		a1->num_cores += a1->chips[i].num_cores;
		applog(LOG_WARNING, "Found chip %d with %d active cores",
		       chip_id, a1->chips[i].num_cores);
	}
	applog(LOG_WARNING, "Found %d chips with total %d active cores",
	       a1->num_chips, a1->num_cores);

	if(a1->num_cores==0) // The A1 board  haven't any cores
		goto failure;		

	mutex_init(&a1->lock);
	a1->active_wq.worklist = NULL;

	return a1;

failure:

	a1->status=ERROR_BOARD;

	exit_A1_chain(a1);
	return NULL;
}

static bool A1_detect_one_chain(struct spi_config *cfg)
{
	struct cgpu_info *cgpu;
	struct spi_ctx _ctx = {
		.config = *cfg,
	}, *ctx = &_ctx;
	struct A1_chain *a1;
	
	ctx->port = sys_spi;

	if (ctx->port == NULL)
		return false;
#ifdef A1_TEST_MODE_EN
	if(testMode>0)
		a1 = init_Test_A1_chain(ctx);
	else
#endif
		a1 = init_A1_chain(ctx);

	if (a1 == NULL)
		return false;

	cgpu = malloc(sizeof(*cgpu));
	if (cgpu == NULL) {
		applog(LOG_ERR, "cgpu allocation error");
		exit_A1_chain(a1);
		return false;
	}
	memset(cgpu, 0, sizeof(*cgpu));
	cgpu->drv = &bitmineA1_drv;
	cgpu->name = "BitmineA1";
	cgpu->threads = 1;

	cgpu->device_data = a1;

#ifdef A1_TEST_MODE_EN
	if(testMode)
		cgpu->deven = DEV_DISABLED;
	else
#endif
	cgpu->deven = DEV_ENABLED;


	a1->cgpu = cgpu;
#ifdef USE_ST_MCU	
	a1->restart= 0;
#endif
	a1->cutTemp= stTempCut;


	add_cgpu(cgpu);
#ifdef A1_PLL_CLOCK_EN
	if(opt_A1Pll>0)
	A1_SetA1PLLClock(a1, a1Pll);
#endif
	return true;
}

#define MAX_SPI_BUS	1
//#define MAX_SPI_CS	2
#define MAX_SPI_CS	1

/* Probe SPI channel and register chip chain */
void A1_detect()
{
	int bus;
	int cs_line;
//	uint32_t speed=DEFAULT_SPI_SPEED; // 1 500 000;
	uint32_t speed=4000000; // 4Mhz
	uint8_t no_cgpu=0;

#if 0
	if(!CheckForAnotherInstance())
	{
        pabort("Another cgminer is running");
		return;
	}
#endif
	
	applog(LOG_DEBUG, "A1 detect");

	spi_init();
	gpio_CS_Init();
	
#ifdef A1_TEST_MODE_EN
	if(opt_test>0)
	{
		testMode=opt_test;
		applog(LOG_NOTICE, "Run Test Mode");

	}
#endif
	applog(LOG_NOTICE, "Run Reset=%d",opt_hwReset);

	if(opt_hwReset==true)
	A1_hw_reset();

#ifdef A1_SPI_SPEED_EN
	if(opt_spiSpeed>0)
		speed=opt_spiSpeed*100000; // x 1kHz
	applog(LOG_NOTICE, "SPI Speed %d kHz",speed/1000);

#endif

#ifdef ST_MCU_EN
	if(opt_stmcu<0)
	{
		stMcu=ST_MCU_NULL;
		applog(LOG_NOTICE, "ST MCU - Disable");		
	}
	else
	{
		stMcu=ST_MCU_PRE_HEADER;
		applog(LOG_NOTICE, "ST MCU - Enable (Pre-header)");		
	}
#endif

#ifdef A1_TEMP_EN	
	if(opt_tempCut>0)
	{
		stTempCut=opt_tempCut;
		applog(LOG_NOTICE, "Cutoff temperature %dC",stTempCut);		
	}
#endif

#ifdef A1_PLL_CLOCK_EN
		A1_ConfigA1PLLClock(opt_A1Pll);
#endif

	const int diff_table[16]={1,2,3,4,8,16,32,37,43,52,65,86,103,129,256,259};

	if(opt_diff>=1&&opt_diff<=16)
		applog(LOG_NOTICE, "ASIC Difficulty =  %d",diff_table[opt_diff-1]);		
	else
		applog(LOG_NOTICE, "ASIC Difficulty =  %d",1); //default

#ifdef USE_GPIO_CS 
	uint8_t gpio_cs,minGpioCS,maxGpioCS;
	struct spi_config cfg = default_spi_config;


#ifdef A1_TEST_MODE_EN
	if(testMode>0)
		opt_gcs=G_CS_MAX; //force auto CS if test mode
#endif
	if(opt_gcs<G_CS_MAX)
	{
		if(opt_gcs<0)
		{
			minGpioCS=G_CS_0;
			maxGpioCS=G_CS_0+1;
			applog(LOG_NOTICE, "GPIO CS is OFF");
		}
		else
		{
			minGpioCS=opt_gcs;
			maxGpioCS=opt_gcs+1;
			applog(LOG_NOTICE, "GPIO CS is ON at CS%d",opt_gcs);		
		}
	}
	else //auto >=G_CS_MAX
	{
		minGpioCS=G_CS_0;
		maxGpioCS=MAX_ASIC_BOARD;
		applog(LOG_NOTICE, "AUTO GPIO CS");
	}

	for (gpio_cs = minGpioCS; gpio_cs < maxGpioCS; gpio_cs++) {
		for (bus = 0; bus < MAX_SPI_BUS; bus++) {
			for (cs_line = 0; cs_line < MAX_SPI_CS; cs_line++) {
//				if(gpio_cs ==G_CS_6) //hardware issue skip CS 6
//					continue;
				cfg = default_spi_config;
				cfg.mode = SPI_MODE_1;
				cfg.speed = speed;
				cfg.bus = bus;
				cfg.cs_line = cs_line;
				cfg.gpio_cs = gpio_cs;
				if(A1_detect_one_chain(&cfg))
					no_cgpu++;
			}
		}
	}

#else
	for (bus = 0; bus < MAX_SPI_BUS; bus++) {
		for (cs_line = 0; cs_line < MAX_SPI_CS; cs_line++) {
			struct spi_config cfg = default_spi_config;
			cfg.mode = SPI_MODE_1;
			cfg.speed = speed;
			cfg.bus = bus;
			cfg.cs_line = cs_line;
			if(A1_detect_one_chain(&cfg))
				no_cgpu++;
		}
	}

#endif //USE_GPIO_CS 


	if (no_cgpu==0) {
		applog(LOG_ERR, "No any A1 board");
		quit(1, "\nNo any A1 board\n");
	}
	else 
	{
		struct cgpu_info *cgpu;
		struct A1_chain *a1;
		uint32_t num_cores=0,speed,eff;
		int i;
		for(i=0;i<no_cgpu;i++)
		{
			cgpu = get_devices(i);
			a1 = cgpu->device_data;
			num_cores+=a1->num_cores;
		}
		eff=((num_cores*100)/(no_cgpu*MAX_CORE_NUMS*MAX_ASIC_NUMS));
		speed=(no_cgpu*eff*A1_MAX_SPEED)/100;
		applog(LOG_NOTICE, "A1 boards=%d, active cores=%d, Efficient=%d%%, speed=%dG",no_cgpu,num_cores,eff,speed);
	}



}

/* return value is nonces processed since previous call */
void A1_DetectOverHeat(struct A1_chain *a1,int chip)
{
	int i,temp;
	struct Alarm *alarm;
	struct timeval tv_now; //alarm start time

	temp=a1->chips[chip].temp;
	for(i=0;i<2;i++)
	{
		alarm=&a1->chips[chip].alarm[i];
		if(temp>=alarm->cutoffTemp)
		{
			cgtime(&tv_now); //now
			if((tv_now.tv_sec-alarm->tv_start.tv_sec)>=alarm->timeout)
			{
				alarm->overheat=1;
			}
		}
		else
		{
			alarm->overheat=0;
			 cgtime(&alarm->tv_start); //reset timer

		}

	}
}

/* return value is nonces processed since previous call */
void A1_CheckOverHeatAlarm(struct A1_chain *a1)
{
	int i,alarmCnt=0;
#ifdef A1_SHUTDOWN_ASIC_BD_1
	for(i=0;i<a1->num_chips;i++)
	{
		if(a1->chips[i].alarm[0].overheat)
		{
			a1->status|=ERROR_OVERHEAD;
			a1->shutdown=1;
			break;
		}
	}
#endif
#ifdef A1_SHUTDOWN_ASIC_BD_2
	for(i=0;i<a1->num_chips;i++)
	{
		if(a1->chips[i].alarm[1].overheat)
		{
			if(alarmCnt)
			{
				a1->status|=ERROR_OVERHEAD;
				a1->shutdown=2;
				break;
			}
			alarmCnt++;
		}
	}
#endif	

	if(a1->shutdown)
		applog(LOG_ERR, "ASIC board cs=%d shutdown due to overheat",a1->spi_ctx->config.gpio_cs);


}

static int64_t A1_scanwork(struct thr_info *thr)
{
	int i;
	struct cgpu_info *cgpu = thr->cgpu;
	struct A1_chain *a1 = cgpu->device_data;
	int32_t nonce_ranges_processed = 0;

//	applog(LOG_ERR, "A1_scanwork cs=%d",a1->spi_ctx->config.gpio_cs);

	uint32_t nonce;
	uint8_t chip_id;
	uint8_t job_id;
	bool work_updated = false;
#if 0
	switch(KeyDetect())
	{
		case KEY_EVENT_RESET:
			A1_reinit();
			break;
	}
#endif

	mutex_lock(&a1->lock);
#ifdef USE_ST_MCU	
	if(a1->restart)
	{
		A1_reinit_device(cgpu);
		a1->restart=0;
		mutex_unlock(&a1->lock);
		return 0;
	}
#endif
	if(a1->chips == NULL||a1->shutdown) //not need post job as not chip avaiable
	{
		cgsleep_ms(100);/* in case of no progress, prevent busy looping */
		mutex_unlock(&a1->lock);
		return 0;
	}


	/* poll queued results */
	while (get_nonce(a1, (uint8_t*)&nonce, &chip_id, &job_id)) {
		nonce = bswap_32(nonce);
		work_updated = true;
#if 1
	if(chip_id == 0 ||chip_id > a1->num_chips)
		continue;
	if(job_id == 0 ||job_id>4 )
		continue;
#else
		assert(chip_id > 0 && chip_id <= a1->num_chips);
		assert(job_id > 0 && job_id <= 4);
#endif		

		struct A1_chip *chip = &a1->chips[chip_id - 1];
		struct work *work = chip->work[job_id - 1];
		if (work == NULL) {
			/* already been flushed => stale */
			applog(LOG_WARNING, "chip(cs%d) %d: stale nonce 0x%08x",
			       a1->spi_ctx->config.gpio_cs,chip_id, nonce);
			chip->stales++;
			continue;
		}
		if (!submit_nonce(thr, work, nonce)) {
			if(opt_realquiet)
				applog(LOG_INFO, "chip(cs%d) %d: invalid nonce 0x%08x", \
			       a1->spi_ctx->config.gpio_cs,chip_id, nonce);
			else				
				applog(LOG_ERR, "chip(cs%d) %d: invalid nonce 0x%08x", \
			       a1->spi_ctx->config.gpio_cs,chip_id, nonce);
			chip->hw_errors++;
			/* add a penalty of a full nonce range on HW errors */
			nonce_ranges_processed--;
			continue;
		}
		applog(LOG_INFO, "YEAH: chip(cs%d) %d: nonce 0x%08x",
		       a1->spi_ctx->config.gpio_cs,chip_id, nonce);
		chip->nonces_found++;
	}

	/* check for completed works */
	for (i = 0; i < a1->num_chips; i++) {
#if 1 //Patch D
		if(a1->chips[i].num_cores==0) //No any core can do work in this chip
			continue;
#endif

		if (!cmd_READ_REG(a1, i + 1)) {
			applog(LOG_ERR, "Failed to read reg from chip(cs%d) %d",a1->spi_ctx->config.gpio_cs,  i);
			// TODO: what to do now?
			continue;
		}
#if 0
		a1->chips[i].num_cores = a1->spi_rx[7];
		if(a1->spi_rx[7]>MAX_CORE_NUMS)
			a1->chips[i].num_cores = MAX_CORE_NUMS;
#endif		
		hexdump("A1 RX", a1->spi_rx, 8);
#ifdef A1_TEMP_EN	
		a1->chips[i].temp= (a1->spi_rx[8]<<8)|a1->spi_rx[9];
#endif
		if ((a1->spi_rx[5] & 0x02) != 0x02) {
			work_updated = true;
			struct work *work = wq_dequeue(&a1->active_wq);
			assert(work != NULL);

#ifdef A1_TEMP_EN	
		if(a1->cutTemp>0)
		{
		
			A1_DetectOverHeat(a1,i);
		
#ifdef A1_CUTOFF_TEMP_CHIP			
#if 1 // if current ASIC is higher then cutoff temperature, don't setwork
			if(a1->cutTemp<a1->chips[i].temp)
				continue;
#else // if any one ASIC is higher then cutoff temperature, don't setwork
			bool overheat=false;
			for (j = 0; j < a1->num_chips; j++) {
				if(a1->cutTemp<a1->chips[j].temp)
				{
					overheat=true;
					break;
				}
			}
			if(overheat)
				continue;
#endif
#endif
		}
#endif


			if (!set_work(a1, i + 1, work))
				continue;

			nonce_ranges_processed++;
			struct A1_chip *chip = &a1->chips[i];
			chip->nonce_ranges_done++;
#if 0 // not need			
			applog(LOG_INFO, "chip(cs%d) %d: job done => %d/%d/%d/%d",
			       a1->spi_ctx->config.gpio_cs,i + 1,
			       chip->nonce_ranges_done, chip->nonces_found,
			       chip->hw_errors, chip->stales);
#endif
		}
	}
	mutex_unlock(&a1->lock);

	if (nonce_ranges_processed < 0) {
		applog(LOG_INFO, "Negative nonce_processed");
		nonce_ranges_processed = 0;
	}

	if (nonce_ranges_processed != 0) {
		applog(LOG_INFO, "nonces processed %d",nonce_ranges_processed);
	}
	/* in case of no progress, prevent busy looping */
	if (!work_updated)
		cgsleep_ms(100);

	if(nonce_ranges_processed==-1) //prevent A1 board disable
		nonce_ranges_processed =0;

	if(a1->cutTemp>0)
		A1_CheckOverHeatAlarm(a1);

	return (int64_t)nonce_ranges_processed << 32;
}

/* queue one work per chip in chain */
static bool A1_queue_full(struct cgpu_info *cgpu)
{
	struct A1_chain *a1 = cgpu->device_data;
	int queue_full = false;

	mutex_lock(&a1->lock);
	applog(LOG_DEBUG, "A1 running queue_full: %d/%d",
	       a1->active_wq.num_elems, a1->num_chips);

	if (a1->active_wq.num_elems >= a1->num_chips) {
		applog(LOG_DEBUG, "active_wq full");
		queue_full = true;
	} else {
		wq_enqueue(&a1->active_wq, get_queued(cgpu));
	}
	mutex_unlock(&a1->lock);

	return queue_full;
}

static void A1_flush_work(struct cgpu_info *cgpu)
{
	struct A1_chain *a1 = cgpu->device_data;
	applog(LOG_DEBUG, "A1 running flushwork");

	int i;

	if(a1->chips == NULL)
		return;

	mutex_lock(&a1->lock);
	/* stop chips hashing current work */
	if (!abort_work(a1)) {
		applog(LOG_ERR, "failed to abort work in chip chain!");
	}
	/* flush the work chips were currently hashing */
	for (i = 0; i < a1->num_chips; i++) {
		int j;
		struct A1_chip *chip = &a1->chips[i];
		for (j = 0; j < 4; j++) {
			struct work *work = chip->work[j];
			if (work == NULL)
				continue;
			applog(LOG_DEBUG, "flushing chip %d, work %d: 0x%p",
			       i, j + 1, work);
			work_completed(cgpu, work);
			chip->work[j] = NULL;
		}
	}
	/* flush queued work */
	applog(LOG_DEBUG, "flushing queued work...");
	while (a1->active_wq.num_elems > 0) {
		struct work *work = wq_dequeue(&a1->active_wq);
		assert(work != NULL);
		applog(LOG_DEBUG, "flushing 0x%p", work);
		work_completed(cgpu, work);
	}
	mutex_unlock(&a1->lock);
}

#ifdef API_STATE_EN
static void A1_reinit_device(struct cgpu_info *cgpu)
{
	struct A1_chain *a1 = cgpu->device_data;

//	cgpu->status= LIFE_INIT;
	a1->restart=0;

//	if(testMode>0)
	a1->status=ASIC_BOARD_OK;
	a1->shutdown=0;

//	cgtime(&thr->last);

//	applog(LOG_INFO, "A1_reinit_device cs=%d", a1->spi_ctx->config.gpio_cs);

	int i,j,num_chips =0;

#ifdef ST_MCU_EN
	if (!cmd_RESET_BCAST(a1))
		goto reinit_failure;

	if (!cmd_BIST_START(a1))
		goto reinit_failure;
#else
		if (!cmd_RESET_BCAST(a1) || !cmd_BIST_START(a1))
			goto reinit_failure;
#endif

//		cgpu->status= LIFE_WELL;

		// Set zero cores before check num chips
		for (i = 0; i < a1->num_chips; i++)
		{
			a1->chips[i].num_cores = 0;
			a1->chips[i].alarm[0].overheat= 0;
			a1->chips[i].alarm[1].overheat= 0;				
		}

		a1->num_cores=0;

		a1->num_chips = a1->spi_rx[3];
		applog(LOG_WARNING, "spidev%d.%d(cs%d): Found %d A1 chips",
			   a1->spi_ctx->config.bus, a1->spi_ctx->config.cs_line,
			   a1->spi_ctx->config.gpio_cs,a1->num_chips);
	
		if (a1->num_chips == 0)
			goto reinit_failure;

		if(a1->chips)
		free(a1->chips);
		a1->chips = NULL;
		a1->chips = calloc(a1->num_chips, sizeof(struct A1_chip));
		if (a1->chips == NULL) {
			applog(LOG_ERR, "A1_chips allocation error");
			goto reinit_failure;
		}

		if (!cmd_BIST_FIX_BCAST(a1))
			goto reinit_failure;

	
	if(a1->num_chips<MAX_ASIC_NUMS)
		a1->status|=ERROR_CHIP;

		for (i = 0; i < a1->num_chips; i++) {
			int chip_id = i + 1;
			if (!cmd_READ_REG(a1, chip_id)) {
				applog(LOG_WARNING, "Failed to read register for "
					   "chip %d -> disabling", chip_id);
				a1->chips[i].num_cores = 0;
				continue;
			}
			num_chips++;
			a1->chips[i].num_cores = a1->spi_rx[7];
			if(a1->spi_rx[7]>MAX_CORE_NUMS)
				a1->chips[i].num_cores = MAX_CORE_NUMS;
#ifdef A1_TEMP_EN	
			a1->chips[i].temp= (a1->spi_rx[8]<<8)|a1->spi_rx[9];
#endif
			a1->num_cores += a1->chips[i].num_cores;
			applog(LOG_WARNING, "Found chip %d with %d active cores",
				   chip_id, a1->chips[i].num_cores);

#ifdef A1_SHUTDOWN_ASIC_BD_1
				a1->chips[i].alarm[0].overheat=0;
				a1->chips[i].alarm[0].timeout=5;
				a1->chips[i].alarm[0].cutoffTemp=stTempCut+15;
				cgtime(&a1->chips[i].alarm[0].tv_start); //reset timer
		
#endif
				
#ifdef A1_SHUTDOWN_ASIC_BD_2
				a1->chips[i].alarm[1].overheat=0;
				a1->chips[i].alarm[1].timeout=5;
				a1->chips[i].alarm[1].cutoffTemp=stTempCut+10;
				cgtime(&a1->chips[i].alarm[1].tv_start); //reset timer				
#endif


		for (j = 0; j < 4; j++) {
			a1->chips[i].work[j]=NULL;
			}


		}
		applog(LOG_WARNING, "Found %d chips with total %d active cores",
			   a1->num_chips, a1->num_cores);
	
		if(a1->num_cores==0) // The A1 board  haven't any cores
			goto reinit_failure;		


		cgpu->deven =DEV_ENABLED;
		return;
	
reinit_failure:
	// Set zero cores before check num chips
	for (i = 0; i < a1->num_chips; i++)
			a1->chips[i].num_cores = 0;

	if(a1->chips)
	free(a1->chips);
	a1->chips = NULL;
	cgpu->deven = DEV_DISABLED;
//	cgpu->deven = DEV_ENABLED;

	a1->status=ERROR_BOARD;
	return ;

}
void A1_reinit(void)
{
	int i;
	struct A1_chain *a1;
	struct cgpu_info *cgpu;

	applog(LOG_WARNING, "A1_reinit");

	testFirstTime=false;


	rd_lock(&devices_lock);
	for (i = 0; i < total_devices; ++i)
	{
		cgpu=devices[i];
		a1=cgpu->device_data;

		applog(LOG_WARNING, "%d,%d,%d",cgpu->deven,cgpu->status,a1->status);
		cgpu->deven = DEV_RECOVER;
//		a1->reinit=1;
		a1->restart=1;

	}
	rd_unlock(&devices_lock);
	

}
void A1_shutdown(void)
{
	int i;
	struct A1_chain *a1;
	struct cgpu_info *cgpu;

	applog(LOG_WARNING, "A1_shutdown");

	testFirstTime=true;

	rd_lock(&devices_lock);
	for (i = 0; i < total_devices; ++i)
	{
		cgpu=devices[i];
		a1=cgpu->device_data;
		a1->shutdown=1;
	}
	rd_unlock(&devices_lock);
	

}

static void A1_get_statline(char *buf, size_t bufsiz, struct cgpu_info *cgpu)
{
	struct A1_chain *a1 = cgpu->device_data;
	int i;
	char strT[100],*pStrT;
	float avgTemp=0,maxTemp=0;
	bool overheat=false;

	// Warning, access to these is not locked - but we don't really
	// care since hashing performance is way more important than
	// locking access to displaying API debug 'stats'
	// If locking becomes an issue for any of them, use copy_data=true also


	if (a1->chips == NULL) 
	{
//		tailsprintf(buf, bufsiz, " | T:--C");
		return;
	}

	if(cgpu->deven != DEV_ENABLED)
		return;


	avgTemp=a1->chips[0].temp;
	pStrT=strT;
	sprintf(pStrT,"%02d",a1->chips[0].temp);
	pStrT+=2;


	for(i=1;i<a1->num_chips;i++)
	{
		if(maxTemp<a1->chips[i].temp)
			maxTemp=a1->chips[i].temp;

		if(a1->cutTemp<a1->chips[i].temp)
			overheat=true;
			
		avgTemp+=a1->chips[i].temp;
		sprintf(pStrT,"-%02d",a1->chips[i].temp);		
		pStrT+=3;

	}
	avgTemp/=a1->num_chips;

	if(a1->shutdown)
		tailsprintf(buf, bufsiz, " | T:%2.0fC(%s)(shutdown=%d)", maxTemp,strT,a1->shutdown);
	else if(overheat)
		tailsprintf(buf, bufsiz, " | T:%2.0fC*Hot*(%s)", maxTemp,strT);
	else	
		tailsprintf(buf, bufsiz, " | T:%2.0fC     (%s)", maxTemp,strT);


}

static void A1_statline_before(char *buf, size_t bufsiz, struct cgpu_info *cgpu)
{
	struct A1_chain *a1 = cgpu->device_data;
	char strT[100],*pStrT;
	int i;
	
	if (a1->chips == NULL) 
		return;

	if(cgpu->deven != DEV_ENABLED)
		return;



#ifdef USE_GPIO_CS	
#if 1
	pStrT=strT;

	if(a1->num_chips<MAX_ASIC_NUMS)
		sprintf(pStrT,"cs:%d A:%d*-C:%03d ", a1->spi_ctx->config.gpio_cs, a1->num_chips, a1->num_cores);
	else if(a1->num_cores<MAX_CORE_NUMS*MAX_ASIC_NUMS)
		sprintf(pStrT,"cs:%d A:%d -C:%03d*", a1->spi_ctx->config.gpio_cs, a1->num_chips, a1->num_cores);
	else
		sprintf(pStrT,"cs:%d A:%d -C:%03d ", a1->spi_ctx->config.gpio_cs, a1->num_chips, a1->num_cores);

	pStrT+=15;
	if(a1->num_chips)
	{
		sprintf(pStrT," (%02d", a1->chips[0].num_cores);
		pStrT+=4;

		for(i=1;i<a1->num_chips;i++)
		{
				sprintf(pStrT,"-%02d", a1->chips[i].num_cores);
				pStrT+=3;
		}
		sprintf(pStrT,")");
		pStrT+=1;
	}
	sprintf(pStrT," | ");
	pStrT+=3;

	tailsprintf(buf, bufsiz, "%s", strT);

#else
		if(a1->num_chips<MAX_ASIC_NUMS)
			tailsprintf(buf, bufsiz, "cs:%d A:%dE-C:%03d | ", a1->spi_ctx->config.gpio_cs, a1->num_chips, a1->num_cores);
		else if(a1->num_cores<MAX_CORE_NUMS)
			tailsprintf(buf, bufsiz, "cs:%d A:%d-C:%03dE | ", a1->spi_ctx->config.gpio_cs, a1->num_chips, a1->num_cores);
		else
			tailsprintf(buf, bufsiz, "cs:%d A:%d-C:%03d | ", a1->spi_ctx->config.gpio_cs, a1->num_chips, a1->num_cores);
#endif		
//	}

#else
#ifdef A1_TEST_MODE_EN
	if(testMode>0)
	{
		if(a1->num_chips<MAX_ASIC_NUMS)
			tailsprintf(buf, bufsiz, "A:%dE-C:%03d | ", a1->num_chips, a1->num_cores);
		else if(a1->num_cores<MAX_CORE_NUMS)
			tailsprintf(buf, bufsiz, "A:%d-C:%03dE | ", a1->num_chips, a1->num_cores);
		else
			tailsprintf(buf, bufsiz, "A:%d-C:%03d | ", a1->num_chips, a1->num_cores);
	}
	else
#endif
	{
		tailsprintf(buf, bufsiz, "A:%d-C:%03d | ", a1->num_chips, a1->num_cores);
	}
#endif
}

#ifdef API_STATE_EN
static struct api_data *A1_api_stats(struct cgpu_info *cgpu)
{
	struct api_data *root = NULL;
	struct A1_chain *a1 = cgpu->device_data;
	int i;
	char strT[100],*pStrT;
	float avgTemp=0;

	if (a1->chips == NULL) 
	{
		root = api_add_string(root, "Temperature", strT, false);
		return root ;
	}

	// Warning, access to these is not locked - but we don't really
	// care since hashing performance is way more important than
	// locking access to displaying API debug 'stats'
	// If locking becomes an issue for any of them, use copy_data=true also
#ifdef USE_GPIO_CS	
	root = api_add_int(root, "CS", (int*)&(a1->spi_ctx->config.gpio_cs), false);
#endif
	root = api_add_int(root, "ASIC", &(a1->num_chips), false);
	root = api_add_int(root, "cores", &(a1->num_cores), false);
	pStrT=strT;
	sprintf(pStrT,"%02d",a1->chips[0].temp);
	pStrT+=2;

	for(i=1;i<a1->num_chips	;i++)
	{
		if(a1->chips[i].temp)
		{
			avgTemp+=a1->chips[i].temp;
			sprintf(pStrT,"-%02d",a1->chips[i].temp);
		}
		else
			sprintf(pStrT,"- F");
		
		pStrT+=3;
	}
	root = api_add_string(root, "Temperature", strT, false);

	avgTemp/=a1->num_chips;
//	root = api_add_temp(root, "Avg Temp: %d", &avgTemp, false);
	root = api_add_temp(root, "Temp(avg)", &avgTemp, false);
	
	return root;
}
#endif

#ifdef HAVE_CURSES

#define CURBUFSIZ 256
#define cg_mvwprintw(win, y, x, fmt, ...) do { \
	char tmp42[CURBUFSIZ]; \
	snprintf(tmp42, sizeof(tmp42), fmt, ##__VA_ARGS__); \
	mvwprintw(win, y, x, "%s", tmp42); \
} while (0)
#define cg_wprintw(win, fmt, ...) do { \
	char tmp42[CURBUFSIZ]; \
	snprintf(tmp42, sizeof(tmp42), fmt, ##__VA_ARGS__); \
	wprintw(win, "%s", tmp42); \
} while (0)


extern WINDOW *mainwin, *statuswin, *logwin;


void A1_curses_print_status(int y)
{

//	applog(LOG_WARNING, "A1_curses_print_status");
	struct A1_chain *a1;
	struct cgpu_info *cgpu;
	int i,testing=0,errHw=0,errBoard=0,errOverheat=0,errChip=0,errCore=0,errSenor=0,numChips=0;


	char strT[100],*pStrT;

	if(testFirstTime)
	{
		y+=2;
		cg_mvwprintw(statuswin, y, 1, "Press 'R' key for hardware testing");
		return;
	}


#if 1
	//Note: not allow write any data to  devices[i]
	rd_lock(&devices_lock);
	for (i = 0; i < total_devices; ++i)
	{
		cgpu=devices[i];
		a1=cgpu->device_data;

		if(cgpu->deven == DEV_RECOVER)
		{
			testing=1;
			break;
		}
		if(a1->status&(ERROR_BOARD|ERROR_CHIP|ERROR_CORE|ERROR_TEMP_SENSOR))
		{
			errHw=1;

			if(a1->status&ERROR_CHIP)
				errChip=1;
			if(a1->status&ERROR_CORE)
				errCore=1;
			if(a1->status&ERROR_TEMP_SENSOR)
				errSenor=1;
			if(a1->status&ERROR_BOARD)
				errBoard=1;
			if(a1->status&ERROR_OVERHEAD)
				errOverheat=1;
		}

		

		if(a1->status==ASIC_BOARD_OK)
		{
			numChips++;
		}

	}
	rd_unlock(&devices_lock);
#endif

	if(numChips<MAX_ASIC_BOARD)
	{
//		errHw=1;
	//	errBoard=1;
	}

	++y;
	mvwhline(statuswin, y, 0, '-', 80);

	if(total_devices)
	{
		if(testing)
			cg_mvwprintw(statuswin, ++y, 1, "Testing .....");
		else if(errHw)
		{
			pStrT=strT;
			sprintf(pStrT,"Hardware: ");
			pStrT+=strlen("Hardware: ");			
			if(errBoard)
			{
				sprintf(pStrT,"ASIC board error,");
				pStrT+=strlen("ASIC board error,");
			}
			if(errChip)
			{
				sprintf(pStrT,"Chip error,");
				pStrT+=strlen("Chip error,");
			}

			if(errCore)
			{
				sprintf(pStrT,"Core error,");
				pStrT+=strlen("Core error,");
			}

			if(errSenor)
			{
				sprintf(pStrT,"Senor error,");
				pStrT+=strlen("Senor error,");
			}

			if(errOverheat)
			{
				sprintf(pStrT,"Overheat");
				pStrT+=strlen("Overheat");
			}

			cg_mvwprintw(statuswin, ++y, 1, "%s",strT);
		}
		else
			cg_mvwprintw(statuswin, ++y, 1, "Hardware testing: Passed");	
	}
	else
		cg_mvwprintw(statuswin, ++y, 1, "Test finished: No any ASIC board");	

	wclrtoeol(statuswin);
	++y;
	mvwhline(statuswin, y, 0, '-', 80);
	//wattroff(statuswin, A_BOLD);
	wclrtoeol(statuswin);
}

#endif

void A1_stop_test(struct cgpu_info *cgpu)
{
// 	struct A1_chain *a1 = cgpu->device_data;
//    a1->status  = (a1->status & (~ASIC_BOARD_TESTING));
}
static bool A1_get_stats(struct cgpu_info *cgpu)
{

	struct A1_chain *a1 = cgpu->device_data;
	int i;

	// Warning, access to these is not locked - but we don't really
	// care since hashing performance is way more important than
	// locking access to displaying API debug 'stats'
	// If locking becomes an issue for any of them, use copy_data=true also

	if (a1->chips == NULL) 
	{
//		cgpu->deven=DEV_DISABLED;
		//cgpu->status= LIFE_NOSTART;
//		a1->status=NO_ASIC_BOARD;
		return true;
	}


	if(a1->num_chips<MAX_ASIC_NUMS)
	{
//		cgpu->status = LIFE_DEAD;
		a1->status|=ERROR_CHIP;
	}
	else
	{
		if(a1->num_cores<(MAX_ASIC_NUMS*MAX_CORE_NUMS))
		{
//			cgpu->status = LIFE_SICK;
			a1->status|=ERROR_CORE;
		}

		for(i=0;i<a1->num_chips;i++)
		{
			if(a1->chips[i].temp==0)
			{
//				cgpu->status = LIFE_SICK;
				a1->status|=ERROR_TEMP_SENSOR;				
			}
		}
	}

/* Check temperature, then stop if
	2) For two or more asic chip has temperature higher than the argu value for 5 seconds, it stop submitting job for the whole board forever.
	
	3) For only one chip has temperature 10 degree higher than the argu value for 5 seconds, it stop submitting job for the whole board forever.
	
	4) even only one chip suddenly 15 degree higher than the argu value, it stop immediately.
*/

#if 0
#ifdef A1_SHUTDOWN_ASIC_BD_1
		for(i=0;i<a1->num_chips;i++)
		{
			if(a1->chips[i].temp>=(a1->cutTemp+15))
			{
				a1->shutdown=1;
				break;
			}
	
		}
	
#endif

#ifdef A1_SHUTDOWN_ASIC_BD_2 
		int overheatCnt=0;
		for(i=0;i<a1->num_chips;i++)
		{
			if(a1->chips[i].temp>=a1->cutTemp+10)
			{
				if (overheatCnt) 
				{
					a1->shutdown=2;
					break;
				}
				overheatCnt++;
			}
		}
	
#endif
#endif
	return true;
}
#endif
struct device_drv bitmineA1_drv = {
	.dname = "BitmineA1",
	.name = "BA1",
	.drv_detect = A1_detect,

	.minerloop = hash_queued_work,
	.scanwork = A1_scanwork,
	.queue_full = A1_queue_full,
	.flush_work = A1_flush_work,
	.reinit_device = A1_reinit_device,
#ifdef USE_ST_MCU
#if 0
	.get_statline_before = A1_statline_before,	
	.get_statline= A1_get_statline,	
#endif
	.get_stats= A1_get_stats,
#ifdef API_STATE_EN
	.get_api_stats = A1_api_stats,
#endif	
#endif	
};
