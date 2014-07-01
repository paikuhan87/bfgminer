/*
 * This code based on the original Icarus code from and copyright by:
 * Copyright 2012 Luke Dashjr
 * Copyright 2012 Xiangfu <xiangfu@openmobilefree.com>
 * Copyright 2012 Andrew Smith
 *
 * The original code was modified by ZeusMiner.com for CGMiner 3.1.1 branch
 * and has been completely reworked for BFGminer by:
 * Copyright 2014 Darkwinde
 * Copyright 2014 jstefanop
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version.  See COPYING for more details.
 *
 * Documentation for the behaviour of the Zeus ASICs can be found at:
 * https://mega.co.nz/#F!eVM3xZIY!ogQPjmNfC-ahZuTkwyAWDw
 */

/**
 * INCLUDES
 **/
#include "config.h"
#include "miner.h"

/*#include <limits.h>
#include <pthread.h>
#include <stdio.h>
#include <sys/time.h>
#include <sys/types.h>
#include <dirent.h>
#include <unistd.h>*/

#ifndef WIN32
  #include <termios.h>
  #include <sys/stat.h>
  #include <fcntl.h>
  #ifndef O_CLOEXEC
    #define O_CLOEXEC 0
  #endif
#else
  #include <windows.h>
  #include <io.h>
#endif

#include <math.h>
#include "lowl-vcom.h"
#include "compat.h"
/**
 * External variables
 **/
extern bool opt_debug;
extern bool opt_check_golden;
extern int opt_chips_count;
extern int opt_chip_clk;
extern int opt_read_count;
/**
 * Internal variables
 **/
static const uint64_t diff_one = 0xFFFF000000000000ull;
uint8_t flush_buf[400];
#define ZEUS_CHIP_GEN 1
#define ZEUS_CHIP_GEN1_CORES 8
#define ZEUS_CHIP_CORES ZEUS_CHIP_GEN1_CORES
#define ZEUS_CHIPS_COUNT_MAX 1
// The serial I/O speed - Linux uses a define 'B115200' in bits/termios.h
#define ZEUS_IO_SPEED 115200
// The number of bytes in a nonce (always 4)
// This is NOT the read-size for the Zeus driver
// That is defined in ZEUS_INFO->read_size
#define ZEUS_NONCE_SIZE 4
// Default value for ZEUS_INFO->read_size
#define ZEUS_READ_SIZE 4
// Fraction of a second, USB timeout is measured in
// i.e. 10 means 1/10 of a second
#define ZEUS_TIME_FACTOR 10
// It's 10 per second, thus value = 10/TIME_FACTOR =
#define ZEUS_READ_FAULT_DECISECONDS 1

#define ZEUS_MIN_NONCE_DIFF 1./0x10000
/**
 * Error codes
 **/
#define ZEUS_GETS_ERROR -1
#define ZEUS_GETS_OK 0
#define ZEUS_GETS_RESTART 1
#define ZEUS_GETS_TIMEOUT 2
/**
 * Device information
 **/
static struct ZEUS_INFO **zeus_info;
struct ZEUS_INFO {
	// time to calculate the golden_ob
	struct timeval golden_tv;

	uint32_t read_count;
	int probe_read_count;

	int check_num;
	int baud;
	int cores_perchip;
	int chips_count_max;
	int chips_count;
	int chip_clk;
	uint32_t clk_header;
	int chips_bit_num;                 
  
	uint32_t core_hash;
	uint32_t chip_hash;
	uint32_t board_hash;
	
	// Bytes to read from Zeus for nonce
	int read_size;
	
	size_t ob_size;
	const char *golden_ob;
	const char *golden_ob2;
	const char *golden_nonce;
	// speed per core per sec
	uint64_t golden_speed_percore;     
	bool nonce_littleendian;
};

struct ZEUS_STATE {
	struct work *last1_work;
	struct work *last2_work;
	struct work *last3_work;
};


BFG_REGISTER_DRIVER(zeus_drv);

static const struct bfg_set_device_definition zeus_set_device_funcs[];

#define zeus_open2(devpath, baud, purge)  serial_open(devpath, baud, ZEUS_READ_FAULT_DECISECONDS, purge)
#define zeus_open(devpath, baud)  zeus_open2(devpath, baud, false)
#define zeus_close(fd) serial_close(fd)
