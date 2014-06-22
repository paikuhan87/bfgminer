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
#include "driver-zeus.h"



/**
 * Flush the UART on the Zeus controller board.
 **/
void zeus_flush_uart(int fd)
{
	#ifdef WIN32  
		const HANDLE fh = (HANDLE)_get_osfhandle(fd);
		PurgeComm(fh, PURGE_RXCLEAR);
	#else
		tcflush(fd, TCIFLUSH);
	#endif
}
	
/**
 * Calculate the log_2 of a value.
 **/
int zeus_log_2(int value) 
{
	int x=0;  
	while(value>1)
	{  
		value>>=1;  
		x++;  
	}
	return x;  
}  

uint32_t zeus_get_revindex(uint32_t value,int bit_num)
{
  uint32_t newvalue;
  int i;
  
#if ZEUS_CHIP_GEN==1
  value = (value&0x1ff80000)>>(29-bit_num);
#else
#error
#endif
  
  newvalue=0;
  
  for(i=0;i<bit_num;i++){
    newvalue = newvalue<<1;
    newvalue += value&0x01;
    value = value>>1;
  }
  return newvalue; 
}

static void rev(unsigned char *s, size_t l)
{
  size_t i, j;
  unsigned char t;
  
  for (i = 0, j = l - 1; i < j; i++, j--) {
    t = s[i];
    s[i] = s[j];
    s[j] = t;
  }
}

static int zeus_gets(unsigned char *buf, int fd, struct timeval *tv_finish, struct thr_info *thr, int read_count,uint32_t *elapsed_count)
{
  ssize_t ret = 0;
  int rc = 0;
  int read_amount = ZEUS_READ_SIZE;
  bool first = true;
  
  *elapsed_count = 0;
  // Read reply 1 byte at a time to get earliest tv_finish
  while (true) {
    ret = read(fd, buf, 1);
    
    if (ret < 0)
	{		
      return ZEUS_GETS_ERROR;
    }
    if (first)
	{
      cgtime(tv_finish);
    }
    if (ret >= read_amount)
	{
      return ZEUS_GETS_OK;
    }
    if (ret > 0) {
      buf += ret;
      read_amount -= ret;
      first = false;	
      continue;
    }
    
    rc++;
    *elapsed_count=rc;
    if (rc >= read_count) {
      if (opt_debug) {
	applog(LOG_DEBUG,
	       "Zeus Read: No data in %.2f seconds",
	       (float)rc/(float)ZEUS_TIME_FACTOR);
      }
      return ZEUS_GETS_TIMEOUT;
    }
    
    if (thr && thr->work_restart) {
      if (opt_debug) {
	applog(LOG_DEBUG,
	       "Zeus Read: Work restart at %.2f seconds",
	       (float)(rc)/(float)ZEUS_TIME_FACTOR);
      }
      return ZEUS_GETS_RESTART;
    }
  }
}

static int zeus_write(int fd, const void *buf, size_t bufLen)
{
	#if 0 //PMA: implement DEBUG command
		char hex[(bufLen * 2) + 1];
		bin2hex(hex, buf, bufLen);
		applog(LOG_ERR, "fd=%d: HEX: %s", fd, hex);
	#endif

	size_t ret;
	if (unlikely(fd == -1))
		return 1;
	
	ret = write(fd, buf, bufLen);
	if (unlikely(ret != bufLen))
		return 1;
		
	return 0;
}

static void zeus_shutdown(struct thr_info *thr)
{
	struct cgpu_info *zeus = thr->cgpu;
	const int fd = zeus->device_fd;
	if (fd == -1)
		return;
	zeus_close(fd);
	zeus->device_fd = -1;
	free(thr->cgpu_data);
}


int zeus_update_num(int chips_count)
{
  int i;
  for (i=1;i<1024;i=i*2){
    if (chips_count<=i){
      return i;
    }
  }
  return 1024;
}


// Number of bytes remaining after reading a nonce from Zeus
int zeus_excess_nonce_size(int fd, struct ZEUS_INFO *info)
{
	// How big a buffer?
	int excess_size = info->read_size - ZEUS_NONCE_SIZE;

	// Try to read one more to ensure the device doesn't return
	// more than we want for this driver
	excess_size++;

	unsigned char excess_bin[excess_size];
	// Read excess_size from Zeus
	struct timeval tv_now;
	timer_set_now(&tv_now);
//PMA: Is this required? Needs to be checked
	//zeus_gets(excess_bin, fd, &tv_now, NULL, 1, excess_size);
	int bytes_read = read(fd, excess_bin, excess_size);
	
	// Number of bytes that were still available
	return bytes_read;
}

//Calculates the target difficulty
static double target_diff(const unsigned char *target)
{
	uint64_t *data64, d64;
	char rtarget[32];
 
	swab256(rtarget, target);
	data64=(uint64_t *)(rtarget + 2);
	d64=be64toh(*data64);
	if(unlikely(!d64))
		d64=1;
 	
 	return diff_one/d64;
}

static bool zeus_detect_custom(const char *devpath, struct device_drv *dev, struct ZEUS_INFO *info)
{
	unsigned char nonce_bin[ZEUS_NONCE_SIZE];
	char nonce_hex[(sizeof(nonce_bin) * 2) + 1];	
	int baud = info->baud;

	int fd = zeus_open2(devpath, baud, true);
	if (unlikely(fd == -1))
	{
		applog(LOG_ERR, "Please restart BFGminer: Failed to open %s", devpath);
		return false;
	}

	char golden_ob[] = 
	"55aa0001"
	"00038000063b0b1b028f32535e900609c15dc49a42b1d8492a6dd4f8f15295c989a1decf584a6aa93be26066d3185f55ef635b5865a7a79b7fa74121a6bb819da416328a9bd2f8cef72794bf02000000";
		
	char golden_ob2[] = 
	"55aa00ff"
	"c00278894532091be6f16a5381ad33619dacb9e6a4a6e79956aac97b51112bfb93dc450b8fc765181a344b6244d42d78625f5c39463bbfdc10405ff711dc1222dd065b015ac9c2c66e28da7202000000";
		
	const char golden_nonce[] = "00038d26";

	int ob_size = strlen(golden_ob)/2;
	unsigned char ob_bin[ob_size];	
	
	char clk_header_str[10];
	uint32_t clk_reg_init;
	uint32_t clk_reg= (uint32_t)(info->chip_clk*2/3);
	if(clk_reg>(100))
		clk_reg_init = 110;
	else
		clk_reg_init = 92;
	
	uint32_t clk_header = (clk_reg_init<<24)+((0xff-clk_reg_init)<<16);
	sprintf(clk_header_str,"%08x",clk_header+0x01);
	memcpy(golden_ob2,clk_header_str,8);
  
	hex2bin(ob_bin, golden_ob2, ob_size);
	for (int i = 0; i < 3; i++)
	{
		zeus_flush_uart(fd);
		zeus_write(fd, ob_bin, ob_size);
		sleep(1);	
	}
	read(fd, flush_buf, 400);

	clk_header = (clk_reg<<24)+((0xff-clk_reg)<<16);
	sprintf(clk_header_str,"%08x",clk_header+0x01);
	memcpy(golden_ob2,clk_header_str,8);
  
	hex2bin(ob_bin, golden_ob2, ob_size);
	for (int i = 0; i < 2; i++)
	{
		zeus_flush_uart(fd);
		zeus_write(fd, ob_bin, ob_size);
		sleep(1);		
	}
 
	clk_header = (clk_reg<<24)+((0xff-clk_reg)<<16);
	sprintf(clk_header_str,"%08x",clk_header+1);
	memcpy(golden_ob,clk_header_str,8);
		
	int bytes_left = zeus_excess_nonce_size(fd, info);
	if (info->read_size - ZEUS_NONCE_SIZE != bytes_left) 
	{
		applog(LOG_ERR, "%s: Test failed at %s: expected %d bytes, got %d",
			   dev->dname, devpath, info->read_size, ZEUS_NONCE_SIZE + bytes_left);
		zeus_close(fd);
		return false;
	}

	if (serial_claim_v(devpath, dev))
	{
		zeus_close(fd);
		return false;
	}
	
	//PMA: Disabled until GETS is fixed by rework
	struct timeval golden_tv;
	opt_check_golden = false;
	if (opt_check_golden)
	{
		struct timeval tv_start, tv_finish;
		zeus_flush_uart(fd);
		hex2bin(ob_bin, golden_ob, ob_size);
		zeus_write(fd, ob_bin, ob_size);
	
		cgtime(&tv_start);
		memset(nonce_bin, 0, sizeof(nonce_bin));
	
		uint32_t elapsed_count;
		zeus_gets(nonce_bin, fd, &tv_finish, NULL, info->probe_read_count, &elapsed_count);
		timersub(&tv_finish, &tv_start, &golden_tv);
		//PMA: Re-Integration required
		//zeus_gets(nonce_bin, fd, &tv_finish, NULL, info->probe_read_count, &elapsed_count, ZEUS_NONCE_SIZE);
	
		bin2hex(nonce_hex, nonce_bin, sizeof(nonce_bin));
		if (strncmp(nonce_hex, golden_nonce, 8))
		{
			applog(LOG_ERR, "%s: Test failed at %s: get %s, should: %s",
				dev->dname, devpath, nonce_hex, golden_nonce);
			zeus_close(fd);
			return false;
		}
		info->golden_speed_percore = (uint64_t)(((double)0xd26)/((double)(golden_tv.tv_sec) + ((double)(golden_tv.tv_usec))/((double)1000000)));
		applog(LOG_ERR, "__PMA__: Speed: %d or %I64d ", info->golden_speed_percore, info->golden_speed_percore);
	}	
	
	info->clk_header = clk_header;
	info->golden_ob = golden_ob;
	info->golden_ob2 = golden_ob2;
	info->golden_nonce = golden_nonce;
	info->golden_tv = golden_tv;
	
	// Create a new Zeus
	struct cgpu_info *zeus;
	zeus = calloc(1, sizeof(struct cgpu_info));
	zeus->drv = dev;
	zeus->device_path = strdup(devpath);
	zeus->device_fd = -1;
	zeus->threads = 1;
	add_cgpu(zeus);	
	zeus->device_data = info;
	
	applog(LOG_INFO, "Zeus %i at %s (Init: baud=%d, readcount=%d, bitnum=%d)with the following parameters:", zeus->device_id, devpath, baud, info->read_count, info->chips_bit_num);
	applog(LOG_INFO, "[Speed] %iMHz core|chip|board: [%ikH/s], [%ikH/s], [%ikH/s]",
		info->chip_clk, info->core_hash/1000, info->chip_hash/1000, info->board_hash/1000);

	zeus_close(fd);
	return true;
}

static bool zeus_detect_one(const char *devpath)
{
	struct ZEUS_INFO *info = calloc(1, sizeof(struct ZEUS_INFO));
	if (unlikely(!info))
		quit(1, "Failed to malloc ZEUS_INFO");
	
	// Get values defined per --set-device, if not set use global variable
	//PMA: Reduce Debug information with clean up
	struct device_drv *drv = &zeus_drv;
	drv_set_defaults(drv, zeus_set_device_funcs, info, devpath, detectone_meta_info.serial, 1);
	if (!info->chip_clk)
		info->chip_clk = opt_chip_clk;
	if (!info->chips_count)
		info->chips_count = opt_chips_count;
	if (!info->read_count)
		info->read_count = opt_read_count;

	

	info->check_num = 0x1234;	
	info->baud = ZEUS_IO_SPEED;
	info->read_size = ZEUS_READ_SIZE;
	info->probe_read_count = 50;
	info->cores_perchip = ZEUS_CHIP_CORES;
	//max clock 382MHz, min clock 200MHz
	if(info->chip_clk > 382)
		info->chip_clk = 382;
	else if(info->chip_clk < 200)
		info->chip_clk = 200;
	
	if(info->chips_count > ZEUS_CHIPS_COUNT_MAX)
	{
		info->chips_count_max = zeus_update_num(info->chips_count);
	}
	info->chips_bit_num = zeus_log_2(info->chips_count_max);
	info->golden_speed_percore = (((info->chip_clk*2)/3)*1024)/8;

	info->core_hash = info->golden_speed_percore;
	info->chip_hash = info->golden_speed_percore*info->cores_perchip;
	info->board_hash = info->golden_speed_percore*info->cores_perchip*info->chips_count;
	
	uint32_t read_count = (uint32_t)((4294967296*10)/(info->cores_perchip*info->chips_count_max*info->golden_speed_percore*2));
	applog(LOG_INFO, "%s calculated ReadCount: %d", devpath, read_count);
	if(read_count < info->read_count)
		info->read_count = read_count;
	
	if (!zeus_detect_custom(devpath, drv, info))
	{
		free(info);
		return false;
	}
	
	return true;
}


static bool zeus_lowl_probe(const struct lowlevel_device_info * const info)
{
	return vcom_lowl_probe_wrapper(info, zeus_detect_one);
}

static bool zeus_prepare(struct thr_info *thr)
{
	struct cgpu_info *zeus = thr->cgpu; 
	struct ZEUS_INFO *info = zeus->device_data;	
	// PMA: Required first run? Think not. Has to be checked by rework
	//struct ZEUS_STATE *state;
	//thr->cgpu_data = state = calloc(1, sizeof(*state));
	//state->firstrun = true;
	
	int fd = zeus_open2(zeus->device_path, info->baud, true);
	if (unlikely(-1 == fd))
	{
		applog(LOG_ERR, "Failed to open Zeus on %s", zeus->device_path);
		return false;
	}

	zeus->device_fd = fd;
//PMA: Also required? Has to be checked by rework	
	zeus->min_nonce_diff = ZEUS_MIN_NONCE_DIFF;		
	zeus->status = LIFE_INIT2;
  
	applog(LOG_INFO, "Opened Zeus %i (fd=%i) on %s", zeus->device_id, zeus->device_fd, zeus->device_path);
	
	return true;
}




static int64_t zeus_scanhash(struct thr_info *thr, struct work *work, __maybe_unused int64_t max_nonce)
{
  struct cgpu_info *zeus;
  int fd;
  int ret;
  
  struct ZEUS_INFO *info;
  
  int numbytes = 84;				// KRAMBLE 84 byte protocol

  unsigned char ob_bin[84], nonce_bin[ZEUS_READ_SIZE];
  char *ob_hex;
  uint32_t nonce;
  int64_t hash_count;
  uint32_t mask;
  struct timeval tv_start, tv_finish, elapsed;
  int curr_hw_errors, i;
  bool was_hw_error;
  int64_t estimate_hashes;

  elapsed.tv_sec = elapsed.tv_usec = 0;
  
  zeus = thr->cgpu;
  if (zeus->device_fd == -1)
    if (!zeus_prepare(thr)) {
      applog(LOG_ERR, "%s%i: Connection error", zeus->drv->name, zeus->device_id);
      dev_error(zeus, REASON_DEV_COMMS_ERROR);
      
      // fail the device if the reopen attempt fails
      return -1;
    }

  fd = zeus->device_fd;
  info = zeus->device_data;

  uint32_t clock = info->clk_header;

uint32_t diff = floor(target_diff(work->target));
applog(LOG_DEBUG, "Work Diff: %d", diff);

if(diff < 1) diff = 1;
		
  uint32_t target_me = 0xffff/diff;
  uint32_t header = clock+target_me;

#if !defined (__BIG_ENDIAN__) && !defined(MIPSEB)
  header = header;
#else
  header = swab32(header);
#endif

  memcpy(ob_bin,(uint8_t *)&header,4);
  memcpy(&ob_bin[4], work->data, 80);	
  rev(ob_bin, 4);
  rev(ob_bin+4, 80);

  if (opt_debug & 0x01) {
    char nonce2[32];

    ob_hex = calloc(1, (8+1)*2);
    bin2hex(ob_hex, (void *)&ob_bin, 8);

    memset(nonce2, 0x00, sizeof(nonce2));
    bin2hex(nonce2, (void *)&work->nonce2, sizeof(work->nonce2));
    
    applog(LOG_ERR, "Zeus %d nounce2 = %s readcount = %d try sent: %s",
		zeus->device_id, nonce2, info->read_count, ob_hex);
    free(ob_hex);
  }

  //int discard = read(fd, flush_buf, 400);
  zeus_flush_uart(fd);
  ret = zeus_write(fd, ob_bin, 84); 
  if (ret) {
    zeus_shutdown(thr);
    applog(LOG_ERR, "%s%i: Comms error", zeus->drv->name, zeus->device_id);
    dev_error(zeus, REASON_DEV_COMMS_ERROR);
    return 0;	/* This should never happen */
  }

  cgtime(&tv_start);

  /* Zeus will return 4 bytes (ZEUS_READ_SIZE) nonces or nothing */
  memset(nonce_bin, 0, sizeof(nonce_bin));

  if (opt_debug&0) {
    applog(LOG_ERR, "diff is %d",diff);
  }

  uint32_t elapsed_count;
  uint32_t read_count = info->read_count;

  while(1){		
    ret = zeus_gets(nonce_bin, fd, &tv_finish, thr, read_count,&elapsed_count);
    if (ret == ZEUS_GETS_ERROR) {
      zeus_shutdown(thr);
      applog(LOG_ERR, "%s%i: Comms error", zeus->drv->name, zeus->device_id);
      dev_error(zeus, REASON_DEV_COMMS_ERROR);
      return 0;
    }

#ifndef WIN32
//openwrt
//    zeus_flush_uart(fd);
#endif

    work->blk.nonce = 0xffffffff;

    // aborted before becoming idle, get new work
    if (ret == ZEUS_GETS_TIMEOUT || ret == ZEUS_GETS_RESTART) {
      
      if (opt_debug&1) {
	applog(LOG_ERR, "1restart or 2timeout:%d ",ret);
      }

      timersub(&tv_finish, &tv_start, &elapsed);
      
      estimate_hashes = ((double)(elapsed.tv_sec) + ((double)(elapsed.tv_usec))/((double)1000000))
	* info->golden_speed_percore*info->chips_count*info->cores_perchip;

      if (unlikely(estimate_hashes > 0xffffffff))
	estimate_hashes = 0xffffffff;

      return estimate_hashes;
    }

    if(read_count>elapsed_count){
      read_count -= elapsed_count;
    }
    else {
      read_count=0;
    }

    memcpy((char *)&nonce, nonce_bin, sizeof(nonce_bin));
    //rev(nonce_bin,4);
#if !defined (__BIG_ENDIAN__) && !defined(MIPSEB)
    nonce = swab32(nonce);
#endif

    curr_hw_errors = zeus->hw_errors;
    
    submit_nonce(thr, work, nonce);
    
    was_hw_error = (curr_hw_errors < zeus->hw_errors);
 
    if (was_hw_error){			
      zeus_flush_uart(fd);
	  applog(LOG_ERR, "HW Error of calculating nonce: %08x ", nonce);
      if (opt_debug&&1) {
	applog(LOG_ERR, "ERR nonce:%08x ",nonce);
      }
    }
    else {			
      if (opt_debug&&0) {
#if ZEUS_CHIP_GEN==1
	uint32_t chip_index=zeus_get_revindex(nonce,info->chips_bit_num);
				uint32_t core_index=(nonce&0xe0000000)>>29;
#else
#error
#endif

	applog(LOG_ERR, "nonce:%08x,chip_index:%d ",nonce,chip_index);
      }			
    }

  }
}


// PMA: test for support of --set-device
static const char *zeus_set_chip_clk(struct cgpu_info * const device, const char * const option, const char * const setting, char * const replybuf, enum bfg_set_device_replytype * const success)
{
	struct ZEUS_INFO * const info = device->device_data;

	info->chip_clk = atoi(setting);

	return NULL;
}

static const char *zeus_set_chips_count(struct cgpu_info * const device, const char * const option, const char * const setting, char * const replybuf, enum bfg_set_device_replytype * const success)
{
	struct ZEUS_INFO * const info = device->device_data;

	info->chips_count = atoi(setting);

	return NULL;
}

static const char *zeus_set_read_count(struct cgpu_info * const device, const char * const option, const char * const setting, char * const replybuf, enum bfg_set_device_replytype * const success)
{
	struct ZEUS_INFO * const info = device->device_data;

	info->read_count = atoi(setting);

	return NULL;
}


static const struct bfg_set_device_definition zeus_set_device_funcs[] = {
	{ "clock", zeus_set_chip_clk, NULL },
	{ "chips", zeus_set_chips_count, NULL },
	{ "read", zeus_set_read_count, NULL },
	{ NULL },
};



static struct api_data *zeus_api_stats(struct cgpu_info *cgpu)
{
	struct api_data *root = NULL;
	struct ZEUS_INFO *info = zeus_info[cgpu->device_id];

	root = api_add_uint32(root, "golden_speed_chip", &(info->chip_hash), false);
	root = api_add_int(root, "chipclk", &(info->chip_clk), false);
	root = api_add_int(root, "chips_count", &(info->chips_count), false);
	root = api_add_int(root, "chips_count_max", &(info->chips_count_max), false);
	root = api_add_uint32(root, "readcount", &(info->read_count), false);

	return root;
}

struct device_drv zeus_drv = 
{
	.dname = "zeus",
	.name = "ZUS",
	.probe_priority = -115,
	.supported_algos = POW_SCRYPT,
	.lowl_probe = zeus_lowl_probe,
	.get_api_stats = zeus_api_stats,
	.thread_prepare = zeus_prepare,
	.scanhash = zeus_scanhash,
	.minerloop = minerloop_scanhash,
	.thread_disable = close_device_fd,
	.thread_shutdown = zeus_shutdown,
};


	
