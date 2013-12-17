/*
 * main_chibios_fw.h
 *
 *  Created on: Nov 28, 2013
 *      Author: aggieair
 */

#ifndef MAIN_CHIBIOS_FW_H_
#define MAIN_CHIBIOS_FW_H_

/** Fly by wire modes */
#define FBW_MODE_MANUAL   0
#define FBW_MODE_AUTO     1
#define FBW_MODE_FAILSAFE 2
#define FBW_MODE_OF_PPRZ(mode) ((mode) < THRESHOLD_MANUAL_PPRZ ? FBW_MODE_MANUAL : FBW_MODE_AUTO)

extern uint8_t fbw_mode;
extern bool_t failsafe_mode;

void init_fbw( void );
void init_ap( void );
void init_threads(void);

void sensors_task( void );
void navigation_task( void );
void monitor_task( void );
void reporting_task( void );
void attitude_loop( void );

void event_task_fbw( void);
static inline void telecommand_task( void );

#endif /* MAIN_CHIBIOS_FW_H_ */
