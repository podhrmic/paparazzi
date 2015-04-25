/*
 * MsgConfig.h
 *
 *  Created on: Apr 25, 2015
 *      Author: fwmav
 */

#ifndef MSGCONFIG_H_
#define MSGCONFIG_H_

/*
 * Message status for parsing
 */
enum MsgStatus {
  MsgSync,
  MsgData,
  MsgCheck
};

/*
 * Message headers
 */
#define HITL_SYNC 0xFA
#define BUF_SIZE 64 // todo maybe smaller buffer
#define HITL_DATALENGTH 12 // for quadrotor
#define CHK_LEN 2

/*
 * Port settings
 */
#define AUTOPILOT_UPDATE_RATE 2 //in ms
#define AP_BAUD 921600
#define AP_DEV "/dev/ttyACM2"

/*
 * Default actuators number (8 is the max available on Lia)
 */
#define ACTUATORS_NB 4



#endif /* MSGCONFIG_H_ */
