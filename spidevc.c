/*
 * Copyright 2013 bitfury
 * Copyright 2013 Luke Dashjr
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "config.h"

#ifdef HAVE_LINUX_SPI_SPIDEV_H
#define HAVE_LINUX_SPI
#endif

#include "spidevc.h"

#include <stdbool.h>
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>

#ifdef HAVE_LINUX_SPI
#include <sys/mman.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <signal.h>
#include <sys/types.h>
#include <linux/spi/spidev.h>
#include <time.h>
#include <unistd.h>
#include <sys/stat.h>
#endif

#include "logging.h"
#include "util.h"

#ifdef HAVE_LINUX_SPI
bool sys_spi_txrx(struct spi_port *port);
static volatile unsigned *gpio;
#endif

struct spi_port *sys_spi;

void spi_init(void)
{
#ifdef HAVE_LINUX_SPI
	int fd;
	fd = open("/dev/mem",O_RDWR|O_SYNC);
	if (fd < 0)
	{
		perror("/dev/mem trouble");
		return;
	}
	gpio = mmap(0,4096,PROT_READ|PROT_WRITE,MAP_SHARED,fd,0x20200000);
	if (gpio == MAP_FAILED)
	{
		perror("gpio mmap trouble");
		return;
	}
	close(fd);
	
	sys_spi = malloc(sizeof(*sys_spi));
	*sys_spi = (struct spi_port){
		.txrx = sys_spi_txrx,
		.sclk = 11,
		.mosi = 10,
		.miso =  9,
		.speed = 4000000,
	};
#endif
}

#ifdef HAVE_LINUX_SPI

#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))

#define GPIO_SET *(gpio+7)  // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR *(gpio+10) // clears bits which are 1 ignores bits which are 0
#define GPIO_LEV *(gpio+13)

static inline
void gpio_assign(int bit, bool nv)
{
	unsigned mask = 1 << bit;
	if (nv)
		GPIO_SET = mask;
	else
		GPIO_CLR = mask;
}

static inline
bool gpio_get(int bit)
{
	return GPIO_LEV & (1 << bit);
}

// Bit-banging reset, to reset more chips in chain - toggle for longer period... Each 3 reset cycles reset first chip in chain
static
int spi_reset(int a)
{
	int i,j;
	int len = 8;
	INP_GPIO(10); OUT_GPIO(10);
	INP_GPIO(11); OUT_GPIO(11);
	GPIO_SET = 1 << 11; // Set SCK
	for (i = 0; i < 32; i++) { // On standard settings this unoptimized code produces 1 Mhz freq.
		GPIO_SET = 1 << 10;
		for (j = 0; j < len; j++) {
			a *= a;
		}
		GPIO_CLR = 1 << 10;
		for (j = 0; j < len; j++) {
			a *= a;
		}
	}
	GPIO_CLR = 1 << 10;
	GPIO_CLR = 1 << 11;
	INP_GPIO(10);
	SET_GPIO_ALT(10,0);
	INP_GPIO(11);
	SET_GPIO_ALT(11,0);
	INP_GPIO(9);
	SET_GPIO_ALT(9,0);

	return a;
}

#define BAILOUT(s)  do{  \
	perror(s);  \
	close(fd);  \
	return false;  \
}while(0)

bool sys_spi_txrx(struct spi_port *port)
{
	const void *wrbuf = spi_gettxbuf(port);
	void *rdbuf = spi_getrxbuf(port);
	size_t bufsz = spi_getbufsz(port);
	int fd;
	int mode, bits, speed, rv, i, j;
	struct spi_ioc_transfer tr[16];

	memset(&tr,0,sizeof(tr));
	mode = 0; bits = 8; speed = 4000000;
	if (port->speed)
		speed = port->speed;

	spi_reset(1234);
	fd = open("/dev/spidev0.0", O_RDWR);
	if (fd < 0) {
		perror("Unable to open SPI device");
		return false;
	}
	if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0)
		BAILOUT("Unable to set WR MODE");
	if (ioctl(fd, SPI_IOC_RD_MODE, &mode) < 0)
		BAILOUT("Unable to set RD MODE");
	if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0)
		BAILOUT("Unable to set WR_BITS_PER_WORD");
	if (ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits) < 0)
		BAILOUT("Unable to set RD_BITS_PER_WORD");
	if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0)
		BAILOUT("Unable to set WR_MAX_SPEED_HZ");
	if (ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed) < 0)
		BAILOUT("Unable to set RD_MAX_SPEED_HZ");

	rv = 0;
	while (bufsz >= 4096) {
                tr[rv].tx_buf = (uintptr_t) wrbuf;
                tr[rv].rx_buf = (uintptr_t) rdbuf;
                tr[rv].len = 4096;
                tr[rv].delay_usecs = 1;
                tr[rv].speed_hz = speed;
                tr[rv].bits_per_word = bits;
                bufsz -= 4096;
                wrbuf += 4096; rdbuf += 4096; rv ++;
        }
        if (bufsz > 0) {
                tr[rv].tx_buf = (uintptr_t) wrbuf;
                tr[rv].rx_buf = (uintptr_t) rdbuf;
                tr[rv].len = (unsigned)bufsz;
                tr[rv].delay_usecs = 1;
                tr[rv].speed_hz = speed;
                tr[rv].bits_per_word = bits;
                rv ++;
        }

        i = rv;
        for (j = 0; j < i; j++) {
                rv = (int)ioctl(fd, SPI_IOC_MESSAGE(1), (intptr_t)&tr[j]);
		if (rv < 0)
			BAILOUT("WTF!");
        }

	close(fd);
	spi_reset(4321);

	return true;
}

static
bool gpio_spi_txrx(struct spi_port * const port)
{
	const uint8_t *wrbuf = spi_gettxbuf(port);
	uint8_t *rdbuf = spi_getrxbuf(port);
	size_t bufsz = spi_getbufsz(port);
	int i;
	
	// reset
	gpio_assign(port->sclk, true);
	for (i = 0; i < 78*3; ++i)
	{
		gpio_assign(port->mosi, true);
		gpio_assign(port->mosi, false);
	}
	gpio_assign(port->sclk, false);
	
	for ( ; bufsz; --bufsz, ++rdbuf, ++wrbuf)
	{
		rdbuf[0] = 0;
		for (i = 0x80; i; i >>= 1)
		{
			gpio_assign(port->mosi, wrbuf[0] & i);
			gpio_assign(port->sclk, true);
			busyloop_baud(port->speed);
			if (gpio_get(port->miso))
				rdbuf[0] |= i;
			gpio_assign(port->sclk, false);
		}
	}
	gpio_assign(port->mosi, false);
	return true;
}

void spi_gpio_multi_txrx(struct spi_port **spis, const int spis_count)
{
	struct spi_port *port;
	int spi_num, i;
	size_t bufsz;
	unsigned pos = 0;
	unsigned all_sclk = 0, all_mosi = 0;
	unsigned active_sclk, do_set[8], do_clr[8], in_lev[8];
	const uint8_t *wrbuf;
	uint8_t *rdbuf;
	
	for (spi_num = 0; spi_num < spis_count; ++spi_num)
	{
		port = spis[spi_num];
		all_sclk |= 1 << port->sclk;
		all_mosi |= 1 << port->mosi;
	}
	
	// reset all at once
	GPIO_SET = all_sclk;
	for (i = 0; i < 78*3; ++i)
	{
		GPIO_SET = all_mosi;
		GPIO_CLR = all_mosi;
	}
	GPIO_CLR = all_sclk;
	
	while (true)
	{
		active_sclk = 0;
		memset(do_set, 0, sizeof(do_set));
		memset(do_clr, 0, sizeof(do_set));
		for (spi_num = 0; spi_num < spis_count; ++spi_num)
		{
			port = spis[spi_num];
			bufsz = spi_getbufsz(port);
			if (pos >= bufsz)
			{
				if (pos == bufsz)
					// In case the sclk is shared, ensure mosi is low
					do_clr[0] |= 1 << port->mosi;
				continue;
			}
			
			active_sclk |= 1 << port->sclk;
			for (i = 7; i >= 0; --i)
			{
				wrbuf = spi_gettxbuf(port);
				if (wrbuf[pos] & (1 << i))
					do_set[i] |= 1 << port->mosi;
				else
					do_clr[i] |= 1 << port->mosi;
			}
		}
		
		if (!active_sclk)
			break;
		
		do_set[0] |= active_sclk;
		for (i = 7; i >= 0; --i)
		{
			GPIO_SET = do_set[i];
			if (do_clr[i])
				GPIO_CLR = do_clr[i];
			// TODO: delay?
			in_lev[i] = GPIO_LEV;
			GPIO_CLR = active_sclk;
		}
		
		for (spi_num = 0; spi_num < spis_count; ++spi_num)
		{
			port = spis[spi_num];
			bufsz = spi_getbufsz(port);
			if (pos >= bufsz)
				continue;
			
			rdbuf = spi_getrxbuf(port);
			rdbuf[pos] = 0;
			for (i = 7; i >= 0; --i)
				if (in_lev[i] & (1 << port->miso))
					rdbuf[pos] |= (1 << i);
		}
	}
}

void spi_init_gpio(struct spi_port * const port)
{
	port->txrx = gpio_spi_txrx;
	INP_GPIO(port->sclk); OUT_GPIO(port->sclk);
	INP_GPIO(port->mosi); OUT_GPIO(port->mosi);
	INP_GPIO(port->miso);
	busyloop_calibrate();
}

#endif

static
void *spi_emit_buf_reverse(struct spi_port *port, const void *p, size_t sz)
{
	const unsigned char *str = p;
	void * const rv = &port->spibuf_rx[port->spibufsz];
	if (port->spibufsz + sz >= SPIMAXSZ)
		return NULL;
	for (size_t i = 0; i < sz; ++i)
	{
		// Reverse bit order in each byte!
		unsigned char p = str[i];
		p = ((p & 0xaa)>>1) | ((p & 0x55) << 1);
		p = ((p & 0xcc)>>2) | ((p & 0x33) << 2);
		p = ((p & 0xf0)>>4) | ((p & 0x0f) << 4);
		port->spibuf[port->spibufsz++] = p;
	}
	return rv;
}

void spi_emit_buf(struct spi_port * const port, const void * const str, const size_t sz)
{
	if (port->spibufsz + sz >= SPIMAXSZ)
		return;
	memcpy(&port->spibuf[port->spibufsz], str, sz);
	port->spibufsz += sz;
}

/* TODO: in production, emit just bit-sequences! Eliminate padding to byte! */
void spi_emit_break(struct spi_port *port)
{
	spi_emit_buf(port, "\x4", 1);
}

void spi_emit_fsync(struct spi_port *port)
{
	spi_emit_buf(port, "\x6", 1);
}

void spi_emit_fasync(struct spi_port *port, int n)
{
	int i;
	for (i = 0; i < n; i++) {
		spi_emit_buf(port, "\x5", 1);
	}
}

void spi_emit_nop(struct spi_port *port, int n) {
	int i;
	for (i = 0; i < n; ++i) {
		spi_emit_buf(port, "\x0", 1);
	}
}

void *spi_emit_data(struct spi_port *port, uint16_t addr, const void *buf, size_t len)
{
	unsigned char otmp[3];
	if (len < 4 || len > 128)
		return NULL;  /* This cannot be programmed in single frame! */
	len /= 4; /* Strip */
	otmp[0] = (len - 1) | 0xE0;
	otmp[1] = (addr >> 8)&0xFF; otmp[2] = addr & 0xFF;
	spi_emit_buf(port, otmp, 3);
	return spi_emit_buf_reverse(port, buf, len*4);
}

#ifdef USE_BFSB
void spi_bfsb_select_bank(int bank)
{
	static int last_bank = -2;
	if (bank == last_bank)
		return;
	const int banks[4]={18,23,24,25}; // GPIO connected to OE of level shifters
	int i;
	for(i=0;i<4;i++)
	{
		INP_GPIO(banks[i]);
		OUT_GPIO(banks[i]);
		if(i==bank)
		{
			GPIO_SET = 1 << banks[i]; // enable bank
		} 
		else
		{
			GPIO_CLR = 1 << banks[i];// disable bank
		}
	}
	last_bank = bank;
}
#endif
