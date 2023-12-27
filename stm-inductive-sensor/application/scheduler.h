/*
 * scheduler.h
 *
 *  Created on: Mar 27, 2022
 *      Author: ftobler
 */

#ifndef SCHEDULER_H_
#define SCHEDULER_H_

#ifdef __cplusplus
extern "C" {
#endif


#include "stdint.h"


enum {
	STATE_STOPPED = 0, //task is not started/initialized
	STATE_RUNNING = 1,     //task is currently running
	STATE_WAIT_FLAG = 2,   //task is waiting on flags
	STATE_WAIT_TIME = 3,   //task is waiting time
	STATE_READY = 4,       //task is ready to run
};


typedef struct {
	uint32_t* stackPointer;
	uint32_t timeout;
	uint32_t eventMask;
	uint32_t eventFlags;
	uint8_t state;
} SchedulerTask_t;

typedef void (*SchedulerTaskFunction)();

/**
 * initialized RTOS
 */
void scheduler_init();

/**
 * Add a task to RTOS
 * the ID of the task is equal to it's priority.
 *     [0] is always the idle task.
 *     [1] has higher priority than idle, but lowest otherwise
 *     [2] has higher priority than [1] but less than [n]
 *     [n] has the highest priority
 * is is not allowed to produce gaps between the IDs, each task from 0 to n needs to be initialized.
 */
void scheduler_addTask(uint32_t id, SchedulerTaskFunction function, uint8_t* stackBuffer, uint32_t stackSize);

/**
 * start RTOS
 * No line gets executed after this function.
 * The current stack memory can be used for something else.
 */
void scheduler_join();

/**
 * Task sleep function. Call from within a task.
 * Never call it from idle task [0]
 */
void scheduler_task_sleep(uint32_t time);

/**
 * Wait for some event flags
 * If a flag the task is waiting on is set this function exits.
 * Never call it from idle task [0]
 *
 * Returns the set flags at wake time. The wait bits will be reset.
 */
uint32_t scheduler_event_wait(uint32_t eventWaitMask);


/**
 * Wait for some event flags
 * If a flag the task is waiting on is set this function exits.
 * Never call it from idle task [0]
 *
 * Set a timeout for this call to return even if event is not present
 *
 * Returns the set flags at wake time. The wait bits will be reset.
 */
uint32_t scheduler_event_wait_timeout(uint32_t eventWaitMask, uint32_t time);

/**
 * Set remote tasks event flags to wake them up from wait state
 */
void scheduler_event_set(uint32_t id, uint32_t eventSetMask);


void scheduler_systick_handler();
void scheduler_pendSV_handler();


#ifdef __cplusplus
}
#endif


#endif /* SCHEDULER_H_ */
