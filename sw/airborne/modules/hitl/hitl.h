/*
 * hitl.h
 *
 *  Created on: Apr 23, 2015
 *      Author: fwmav
 */

#ifndef HITL_MODULE_H
#define HITL_MODULE_H

#include "std.h"
#include "subsystems/commands.h"

/*
 * Message headers
 */
#define HITL_SYNC 0xFA
#define HITL_HEADER_SIZE 1
#define HITL_BUFFER_SIZE 128

struct Hitl {
  uint8_t msg_buf[HITL_BUFFER_SIZE];
  uint32_t msg_cnt;
  uint16_t chk_err;
  uint16_t good_msg;
};

extern struct Hitl hitl;

void hitl_init(void);

void hitl_periodic(void);

void hitl_cksum(uint16_t size, uint8_t *buf, uint8_t *cksum0, uint8_t *cksum1 );

uint8_t hitl_verify_chk(volatile uint8_t *buf_add, uint16_t size);

#endif /* HITL_MODULE_H_ */
