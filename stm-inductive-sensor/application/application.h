/*
 * application.h
 *
 *  Created on: Dec 27, 2023
 *      Author: ftobler
 */

#ifndef APPLICATION_H_
#define APPLICATION_H_

void application_init();
void appliation_loop();
void application_dma_complete_isr();
void application_timer_isr();

#endif /* APPLICATION_H_ */
