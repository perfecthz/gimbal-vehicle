/*
 * Encoder.c
 *
 *  Created on: Feb 13, 2022
 *      Author: lzr93
 */

#include "Encoder.h"
#include "main.h"
__IO uint16_t TIM2_OverflowCount;//定时器2溢出次数计数值
__IO uint16_t TIM3_OverflowCount;//定时器3溢出次数计数值
__IO uint16_t TIM4_OverflowCount;//定时器4溢出次数计数值
__IO uint16_t TIM5_OverflowCount;//定时器5溢出次数计数值

void Encoder_Struct_Init(Encoder_Struct *Encoder, uint8_t TIMx)
{
	Encoder->CaptureCount = 0;	//编码器捕获计数值
	Encoder->OverflowCount = 0;//编码器溢出次数
	Encoder->Capture_D_Value = 0;//编码器前后2次捕获计数的差值
	Encoder->CNT_Last = 0;//缓存上一次的TIMx->CNT计数器值
	Encoder->TIMx = TIMx;//对应的定时器序号

	switch(TIMx)//清0对应定时器溢出次数计数值
	{
		case 2: TIM2_OverflowCount=0;break;
		case 3: TIM3_OverflowCount=0;break;
		case 4: TIM4_OverflowCount=0;break;
		case 5: TIM5_OverflowCount=0;break;
		default:break;
	}
}






void Encoder_UpdataValue(Encoder_Struct *Encoder, int8_t Signed)
{
  uint16_t Encoder_TIM;

	switch(Encoder->TIMx)//获取对应定时器的计数值与溢出次数计数值
	{

		case 2:
		{
			Encoder_TIM = __HAL_TIM_GET_COUNTER(&htim2);
			Encoder->OverflowCount = TIM2_OverflowCount;//定时器1溢出次数计数值
		}break;

		case 3:
		{
			Encoder_TIM = __HAL_TIM_GET_COUNTER(&htim3);//读取TIMx->CNT计数器值
			Encoder->OverflowCount = TIM3_OverflowCount;//定时器1溢出次数计数值
		}break;

		case 4:
		{
			Encoder_TIM = __HAL_TIM_GET_COUNTER(&htim4);//读取TIMx->CNT计数器值
			Encoder->OverflowCount = TIM4_OverflowCount;//定时器1溢出次数计数值
		}break;

		case 5:
		{
			Encoder_TIM = __HAL_TIM_GET_COUNTER(&htim5);//读取TIMx->CNT计数器值
			Encoder->OverflowCount = TIM5_OverflowCount;//定时器1溢出次数计数值
		}break;

		default: return;//退出函数
	}

	if(Encoder_TIM > Encoder->CNT_Last)
	{
		Encoder->Capture_D_Value = Encoder_TIM - Encoder->CNT_Last;//获取编码器前后2次捕获计数的差值
		if((uint16_t)Encoder->Capture_D_Value > 10000) Encoder->Capture_D_Value -= 65536;//判断是否溢出跳变
	}
	else
	{
		Encoder->Capture_D_Value = 0-(Encoder->CNT_Last - Encoder_TIM);//获取编码器前后2次捕获计数的差值
		if((uint16_t)Encoder->Capture_D_Value > 10000) Encoder->Capture_D_Value += 65536;//判断是否溢出跳变
	}

	Encoder->CNT_Last = Encoder_TIM;

	Encoder->Capture_D_Value = (Encoder->Capture_D_Value) * Signed;//获取编码器前后2次捕获计数的差值
	Encoder->CaptureCount = (Encoder->OverflowCount*65536 + Encoder_TIM) * Signed;//获取编码器捕获计数值
}




















void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){


	if(htim->Instance == TIM2)
	{
		if(__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
		{
			TIM2_OverflowCount--;       //向下计数溢出
		}
		else
		{
			TIM2_OverflowCount++;  		 //向上计数溢出
		}
	}

	else if(htim->Instance == TIM3)
	{
		if(__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
		{
			TIM3_OverflowCount--;       //向下计数溢出
		}
		else
		{
			TIM3_OverflowCount++;  		 //向上计数溢出
		}
	}

	else if(htim->Instance == TIM4)
	{
		if(__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
		{
			TIM4_OverflowCount--;       //向下计数溢出
		}
		else
		{
			TIM4_OverflowCount++;  		 //向上计数溢出
		}
	}
	else if(htim->Instance == TIM5)
	{
		if(__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
		{
			TIM5_OverflowCount--;       //向下计数溢出
		}
		else
		{
			TIM5_OverflowCount++;  		 //向上计数溢出
		}
	}
}
