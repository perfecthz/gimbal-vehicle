/*
 * loop.c
 *
 *  Created on: Feb 13, 2022
 *      Author: bohang
 */

#include "Loop.h"

//extern __IO uint16_t Loop_1000msTime;
extern __IO uint16_t Loop_100msTime;
extern __IO uint16_t Loop_500msTime;
extern __IO uint16_t Loop_60msTime;
extern __IO uint16_t Loop_10msTime;
extern __IO uint16_t Loop_5msTime;

void HAL_SYSTICK_Callback()
{
	//if(Loop_1000msTime) Loop_1000msTime--;
	if(Loop_500msTime) Loop_500msTime--;
	if(Loop_10msTime) Loop_10msTime--;
	if(Loop_100msTime) Loop_100msTime--;
	if(Loop_60msTime) Loop_60msTime--;
	if(Loop_5msTime) Loop_5msTime--;

}
