//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"
#include "stm32f30x_gpio.h"
#include "stm32f30x_tim.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_usart.h"
#include "stm32f30x_misc.h"

#define USART_RX_PIN	GPIO_Pin_10
#define USART_TX_PIN	GPIO_Pin_9

#define USART_RX_PIN1	GPIO_Pin_3	//PORTA
#define USART_TX_PIN1	GPIO_Pin_2	//PORTA

#define I2C_SDA_PIN		GPIO_Pin_9	// PORTB
#define I2C_SCL_PIN		GPIO_Pin_8	// PORTB

#define TIM2_CH2_PIN	GPIO_Pin_1	// PORTA
#define TIM2_CH4_PIN	GPIO_Pin_3	// PORTA
#define TIM2_CH3_PIN	GPIO_Pin_2	// PORTA
#define TIM3_CH2_PIN	GPIO_Pin_4	// PORTA
#define TIM3_CH1_PIN	GPIO_Pin_6	// PORTC
#define TIM4_CH3_PIN	GPIO_Pin_14	// PORTD
#define TIM4_CH2_PIN	GPIO_Pin_13	// PORTD
#define TIM4_CH4_PIN	GPIO_Pin_15	// PORTD

#define MAX_STRLEN	4
uint8_t cnt = 0;
char received_string[MAX_STRLEN+1];
int data = 0;
// ---------------------------------------------------------------------------
//
// Standalone STM32F3 empty sample (trace via DEBUG).
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the DEBUG output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"



int
main(int argc, char* argv[])
{


	int i = 0;
	GPIO_InitTypeDef GPIO_InitStructure;
	/* Enable GPIO peripheral clock for ports B and C */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE);

	/* Set up Alternate function for pin 4 and 5 on port A to USART */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_7);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_7);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_7);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_7);

	/* Set up Alternate function for pin 9 and 8 on port B to i2c */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_4);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_4);

	/* Set up Alternate functions for TIM pins */
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_2);

	/* Configuration of pin 9 and 10 */
	GPIO_InitStructure.GPIO_Pin = USART_RX_PIN | USART_TX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configuration of pin 9 and 10 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* Configuration of pin 9 and 10 */
	GPIO_InitStructure.GPIO_Pin = USART_RX_PIN1 | USART_TX_PIN1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configuration of pin 9 and 10 */
	GPIO_InitStructure.GPIO_Pin = TIM4_CH2_PIN | TIM4_CH4_PIN | TIM4_CH3_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* Configuration of pin 9 and 10 */
	GPIO_InitStructure.GPIO_Pin = TIM3_CH1_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* Configuration of pin 9 and 10 */
	GPIO_InitStructure.GPIO_Pin = I2C_SDA_PIN | I2C_SCL_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	TIM_TimeBaseInitTypeDef TimerInitStructure;
	TIM_OCInitTypeDef OutputChannelInit;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	/* Time base configuration */
	// Prescaler and period are defined for 10ms
	TimerInitStructure.TIM_Prescaler = 72-1;
	TimerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TimerInitStructure.TIM_Period = 10000-1;
	TimerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TimerInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TimerInitStructure);
	/* TIM2 enable counter */
	TIM_Cmd(TIM1, ENABLE);

	TimerInitStructure.TIM_Prescaler = 71;
	TimerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TimerInitStructure.TIM_Period = 20000-1;
	TimerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TimerInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4, &TimerInitStructure);
	TIM_TimeBaseInit(TIM3, &TimerInitStructure);

	OutputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
	OutputChannelInit.TIM_Pulse = 1000;
	OutputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
	OutputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC2Init(TIM4, &OutputChannelInit);
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Disable);

	TIM_OC4Init(TIM4, &OutputChannelInit);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Disable);

	TIM_OC3Init(TIM4, &OutputChannelInit);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Disable);

	TIM_OC1Init(TIM3, &OutputChannelInit);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Disable);

	/* Enable timers */
	TIM_Cmd(TIM3, ENABLE);
	TIM_Cmd(TIM4, ENABLE);

	USART_InitTypeDef USART_InitStructure;

	/* USART clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	/* USART structure init*/
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);

	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);



	NVIC_InitTypeDef NVIC_InitStructure;
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn; // we want to configure the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;// this sets the priority group of the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; // this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // the USART1 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);

	USART_Cmd(USART1, ENABLE);
	USART_Cmd(USART2, ENABLE);

	int j = 9000000;
	while(j>0){
		TIM4->CCR2 = 1000;
		TIM4->CCR3 = 1000;
		TIM4->CCR4 = 1000;
		TIM3->CCR1 = 1000;
		j--;
	}
	while (1)
    {

		//char data = received_string[1];
		//while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
		//USART_SendData(USART2, );

		if(data==1050){
			GPIO_SetBits(GPIOC, GPIO_Pin_9);
		}
		else GPIO_ResetBits(GPIOC, GPIO_Pin_9);
		TIM4->CCR2 = (data-1000)*2+1200;
		TIM4->CCR3 = (data-1000)*2+1200;
		TIM4->CCR4 = (data-1000)*2+1200;
		TIM3->CCR1 = (data-1000)*2+1200;


    }
}

void USART1_IRQHandler(void){

	if( USART_GetITStatus(USART1, USART_IT_RXNE) ){
		//static uint8_t cnt = 0;
		char t = USART1->RDR;

		if( (t != '\n' && t!='\0') && (cnt < 7) ){
			received_string[cnt] = t;
			cnt++;
			}
		else
			{
			cnt = 0;
			}
			t = 0;
			received_string[4] = 0;
	}
	if(cnt==4){
		data = atoi(received_string);
	}

}


#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
