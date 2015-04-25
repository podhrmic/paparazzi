/*
 * MsgPacket.h
 *
 *  Created on: Apr 25, 2015
 *      Author: fwmav
 */

#ifndef MSGPACKET_H_
#define MSGPACKET_H_

#include "MsgConfig.h"

/*
 * Packet
 * contains variables for packet parsing
 */
class MsgPacket
{
public:
  bool  msg_available;
  uint32_t chksm_error;
  uint32_t hdr_error;
  uint8_t msg_buf[BUF_SIZE] = {0};
  uint16_t datalength;
  enum MsgStatus  status;
  uint8_t  msg_idx;

MsgPacket(){
    msg_available = false;
    chksm_error = 0;
    hdr_error = 0;
    datalength = HITL_DATALENGTH;
    status = MsgSync;
    msg_idx = 0;
  }
};


#endif /* MSGPACKET_H_ */
