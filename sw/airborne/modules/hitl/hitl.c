/*
 * hitl.c
 *
 *  Created on: Apr 23, 2015
 *      Author: fwmav
 */

#include "modules/hitl/hitl.h"
#include "mcu_periph/uart.h"
#include "mcu_periph/usb_serial.h"
#include <string.h>

struct Hitl hitl;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_hitl(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_HITL(trans, dev, AC_ID,
                     &hitl.msg_cnt,
                     &hitl.chk_err,
                     &hitl.good_msg);
}

#endif

void hitl_init(void)
{
  hitl.msg_cnt = 0;
  hitl.chk_err = 0;
  hitl.good_msg = 0;
  memset(hitl.msg_buf, 0, sizeof(hitl.msg_buf));
  hitl.msg_buf[0] = HITL_SYNC;

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, "HITL", send_hitl);
#endif
  }

void hitl_periodic(void)
{
  static uint8_t cksum0, cksum1;
  static uint16_t idx;

  // index after the header
  idx = HITL_HEADER_SIZE;

  // copy counter
  hitl.msg_buf[idx] = (uint8_t)(hitl.msg_cnt>>24);
  idx++;
  hitl.msg_buf[idx] = (uint8_t)(hitl.msg_cnt>>16);
  idx++;
  hitl.msg_buf[idx] = (uint8_t)(hitl.msg_cnt>>8);
  idx++;
  hitl.msg_buf[idx] = (uint8_t)(hitl.msg_cnt);
  idx++;

  // commands
  memcpy(&hitl.msg_buf[idx], &commands, sizeof(pprz_t)*COMMANDS_NB);
  idx += sizeof(pprz_t)*COMMANDS_NB;

  // calculate checksum & send
  hitl_cksum(idx, hitl.msg_buf, &cksum0, &cksum1);
  hitl.msg_buf[idx] = cksum0;
  idx++;
  hitl.msg_buf[idx] = cksum1;
  idx++;

  for (int i=0; i<idx; i++) {
    //uart_transmit(&HITL_PORT, hitl.msg_buf[i]);
    VCOM_putchar(hitl.msg_buf[i]);
  }

  // verify checksum
  idx--;
  if (hitl_verify_chk(&hitl.msg_buf[1], idx-2)) {
    hitl.good_msg++;
  } else {
    hitl.chk_err++;
  }

  hitl.msg_cnt++;
  }

/**
 * Calculate checksum
 *
 * @param size - data to be checksumed (it is effectively header+data)
 * @param buf - pointer to the buffer
 * @param cksum0 - first byte of cksum
 * @param cksum1 - second byte of cksum
 */
void hitl_cksum(uint16_t size, uint8_t *buf, uint8_t *cksum0, uint8_t *cksum1 ) {
  static uint8_t c0, c1;
  static uint16_t i;//, size;
  c0 = 0;
  c1 = 0;

  //Start at byte two so the header is not part of the checksum
  for ( i = 1; i < size; i++ ) {
    c0 += (uint8_t)buf[i];
      c1 += c0;
  }
  *cksum0 = c0;
  *cksum1 = c1;
}

/**
 * Verify checksum
 */
uint8_t hitl_verify_chk(volatile uint8_t *buf_add, uint16_t size) {
  static uint8_t c0, c1;
  static uint16_t i;//, size;
  c0 = 0;
  c1 = 0;
  //Start at byte two so the header is not part of the checksum
  for ( i = 0; i < size; i++ ) {
    c0 += (uint8_t)buf_add[i];
    c1 += c0;
  }

  return ( (uint16_t)(c0 << 8 | c1) == (uint16_t)(buf_add[size] << 8 | buf_add[size+1]));
}
