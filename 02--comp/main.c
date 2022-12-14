/**
 * Copyright (c) 2014 - 2021, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 * @defgroup nrf_lpcomp_example main.c
 * @{
 * @ingroup nrf_lpcomp_example
 * @brief LPCOMP example application main file.
 *
 * This is an example low power comparator application.
 * The example requires that LPCOMP A,B inputs are connected with QENC A,B outputs and
 * LPCOMP LED output is connected with LPCOMP LED input.
 *
 * Example uses software quadrature encoder simulator QENC.
 * Quadrature encoder simulator uses one channel of GPIOTE module.
 * The state of the encoder changes on the inactive edge of the sampling clock generated by LED output.
 *
 * In a infinite loop QENC produces variable number of positive and negative pulses
 * synchronously with bursts of clock impulses generated by LPCOMP at LED output.
 * The pulses are counted by LPCOMP operating in a REPORT mode.
 * Pulses counted by LPCOMP are compared with pulses generated by QENC.
 * The tests stops if there is a difference between number of pulses counted and generated.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_drv_comp.h"
#include "nrf_comp.h"
#include "nrf_error.h"
#include "app_error.h"
#include "boards.h"
#include "app_uart.h"
#include "bsp.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_gpiote.h"
#include "nrf_gpio.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_timer.h"

#define GPIO_OUTPUT_PIN_NUMBER 13

//static const nrf_drv_comp_config_t m_comp;


#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

		



//#define ENABLE_LOOPBACK_TEST  /**< if defined, then this example will be a loopback test, which means that TX should be connected to RX to get data loopback. */

#define MAX_TEST_DATA_BYTES     (15U)                /**< max number of test bytes to be used for tx and rx. */
#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */


#define PPI_EXAMPLE_TIMERS_PHASE_SHIFT_DELAY    (10)    // 1s = 10 * 100ms (Timer 0 interrupt)
#define PPI_EXAMPLE_TIMER0_INTERVAL             (1)   // Timer interval in milliseconds
#define PPI_EXAMPLE_TIMER1_INTERVAL             (2)   // Timer interval in milliseconds


static const nrf_drv_timer_t m_timer0 = NRF_DRV_TIMER_INSTANCE(0);
static const nrf_drv_timer_t m_timer1 = NRF_DRV_TIMER_INSTANCE(1);


static volatile uint32_t m_counter;

static void timer0_event_handler(nrf_timer_event_t event_type, void * p_context)
{
    ++m_counter;
	printf("timer 0 m_counter: %d\r\n", m_counter);
}
/* Timer event handler. Not used since Timer1 and Timer2 are used only for PPI. */
static void empty_timer_handler(nrf_timer_event_t event_type, void * p_context)
{
	++m_counter;
	printf("timer 1 m_counter: %d\r\n", m_counter);
}

static void timer0_init(void)
{
    // Check TIMER0 configuration for details.
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.frequency = NRF_TIMER_FREQ_31250Hz;
    ret_code_t err_code = nrf_drv_timer_init(&m_timer0, &timer_cfg, timer0_event_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_timer_extended_compare(&m_timer0,
                                   NRF_TIMER_CC_CHANNEL0,
                                   nrf_drv_timer_ms_to_ticks(&m_timer0,
                                                             PPI_EXAMPLE_TIMER0_INTERVAL),
                                   NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                   true);
	
		nrf_drv_timer_enable(&m_timer0);
}

static void timer1_init(void)
{
    // Check TIMER1 configuration for details.
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.frequency = NRF_TIMER_FREQ_31250Hz;
    ret_code_t err_code = nrf_drv_timer_init(&m_timer1, &timer_cfg, empty_timer_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_timer_extended_compare(&m_timer1,
                                   NRF_TIMER_CC_CHANNEL0,
                                   nrf_drv_timer_ms_to_ticks(&m_timer1,
                                                             PPI_EXAMPLE_TIMER1_INTERVAL),
                                   NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                   true);
	
	nrf_drv_timer_enable(&m_timer1);
}

void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}


#ifdef ENABLE_LOOPBACK_TEST
/* Use flow control in loopback test. */
#define UART_HWFC APP_UART_FLOW_CONTROL_ENABLED

/** @brief Function for setting the @ref ERROR_PIN high, and then enter an infinite loop.
 */
static void show_error(void)
{

    bsp_board_leds_on();
    while (true)
    {
        // Do nothing.
    }
}


/** @brief Function for testing UART loop back.
 *  @details Transmitts one character at a time to check if the data received from the loopback is same as the transmitted data.
 *  @note  @ref TX_PIN_NUMBER must be connected to @ref RX_PIN_NUMBER)
 */
static void uart_loopback_test()
{
    uint8_t * tx_data = (uint8_t *)("\r\nLOOPBACK_TEST\r\n");
    uint8_t   rx_data;

    // Start sending one byte and see if you get the same
    for (uint32_t i = 0; i < MAX_TEST_DATA_BYTES; i++)
    {
        uint32_t err_code;
        while (app_uart_put(tx_data[i]) != NRF_SUCCESS);

        nrf_delay_ms(10);
        err_code = app_uart_get(&rx_data);

        if ((rx_data != tx_data[i]) || (err_code != NRF_SUCCESS))
        {
            show_error();
        }
    }
    return;
}
#else
/* When UART is used for communication with the host do not use flow control.*/
#define UART_HWFC APP_UART_FLOW_CONTROL_DISABLED
#endif


//void COMP_LPCOMP_IRQHandler(void)
//{
//	if(NRF_COMP->EVENTS_DOWN!=0)
//	{
//		
//		nrf_gpio_pin_set(12);
//		NRF_COMP->EVENTS_DOWN=0;
//	}
//	if(NRF_COMP->EVENTS_UP!=0)
//	{
//		nrf_gpio_pin_clear(12);
//		NRF_COMP->EVENTS_UP=0;		
//	}
//}

static void comp_init(void)
{
	NRF_COMP->INTENSET=(COMP_INTENSET_UP_Msk|COMP_INTENSET_DOWN_Msk);
	NRF_COMP->PSEL=(COMP_PSEL_PSEL_AnalogInput0<<COMP_PSEL_PSEL_Pos);//0.02
	NRF_COMP->EXTREFSEL=(COMP_EXTREFSEL_EXTREFSEL_AnalogReference1<<COMP_EXTREFSEL_EXTREFSEL_Pos);//0.03
	NRF_COMP->MODE=(COMP_MODE_MAIN_Diff<<COMP_MODE_MAIN_Pos);
	NRF_COMP->MODE|=(COMP_MODE_SP_High<<COMP_MODE_SP_Pos);
	NRF_COMP->HYST=COMP_HYST_HYST_NoHyst<<COMP_HYST_HYST_Pos;
	//NRF_COMP->HYST=COMP_HYST_HYST_Hyst50mV<<COMP_HYST_HYST_Pos;

	
//	NVIC_SetPriority(COMP_LPCOMP_IRQn,3);
//	NVIC_EnableIRQ(LPCOMP_IRQn);
	NRF_COMP->REFSEL =7; 
	NRF_COMP->ENABLE=2;
	NRF_COMP->TASKS_START=1;
	
	
	
	while(NRF_COMP->EVENTS_READY==0);
	//NRF_COMP->EVENTS_READY=0;
}

void GPIOTE_PPI_init(void)
{
	
		NRF_GPIOTE->CONFIG[0]=3;
	
		uint32_t compare_evt_addr_down,compare_evt_addr_up;
    uint32_t gpiote_task_addr_up,gpiote_task_addr_down;	
    nrf_ppi_channel_t ppi_channel1;
		nrf_ppi_channel_t ppi_channel2;
    ret_code_t err_code;
	
	
	  nrf_drv_gpiote_out_config_t config = GPIOTE_CONFIG_OUT_TASK_TOGGLE(false);	
    err_code = nrf_drv_gpiote_out_init(GPIO_OUTPUT_PIN_NUMBER, &config);
    APP_ERROR_CHECK(err_code);

	  compare_evt_addr_down = nrf_drv_comp_event_address_get(NRF_COMP_EVENT_DOWN);	
    gpiote_task_addr_down = nrf_drv_gpiote_clr_task_addr_get(GPIO_OUTPUT_PIN_NUMBER);
		compare_evt_addr_up	=	nrf_drv_comp_event_address_get(NRF_COMP_EVENT_UP);
    gpiote_task_addr_up = nrf_drv_gpiote_set_task_addr_get(GPIO_OUTPUT_PIN_NUMBER);
	
		/*ppi_channel1*/
		err_code = nrf_drv_ppi_channel_alloc(&ppi_channel1);
    APP_ERROR_CHECK(err_code);
	
    err_code = nrf_drv_ppi_channel_assign(ppi_channel1, compare_evt_addr_down, gpiote_task_addr_up);
    APP_ERROR_CHECK(err_code);
		
    err_code = nrf_drv_ppi_channel_enable(ppi_channel1);
    APP_ERROR_CHECK(err_code);
		
		
		/*ppi_channel2*/
		err_code = nrf_drv_ppi_channel_alloc(&ppi_channel2);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_ppi_channel_assign(ppi_channel2, compare_evt_addr_up, gpiote_task_addr_down);
	  APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_ppi_channel_enable(ppi_channel2);
    APP_ERROR_CHECK(err_code);
		
    nrf_drv_gpiote_out_task_enable(GPIO_OUTPUT_PIN_NUMBER);
		
		
}

int main(void)
{	 
	 uint32_t err_code;
	
	  NRF_CLOCK->EVENTS_HFCLKSTARTED=0;
	  NRF_CLOCK->TASKS_HFCLKSTART=1;
	  while(NRF_CLOCK->EVENTS_HFCLKSTARTED==0)
		{
		}
		
    const app_uart_comm_params_t comm_params =
      {
          RX_PIN_NUMBER,
          TX_PIN_NUMBER,
          RTS_PIN_NUMBER,
          CTS_PIN_NUMBER,
          UART_HWFC,
          false,
#if defined (UART_PRESENT)
          NRF_UART_BAUDRATE_115200
#else
          NRF_UARTE_BAUDRATE_115200
#endif
      };

    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOWEST,
                         err_code);

    APP_ERROR_CHECK(err_code);

#ifndef ENABLE_LOOPBACK_TEST
    printf("\r\nCOMP example started.\r\n");
#endif	   
	
		err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);	
			
		err_code = nrf_drv_ppi_init();
		APP_ERROR_CHECK(err_code);
			
		comp_init();
		GPIOTE_PPI_init();
//		timer0_init();
//		timer1_init();
//		
//		nrf_gpio_cfg_output(19);
//		nrf_gpio_pin_set(19);	
			
//		nrf_gpio_cfg_output(2);
//		nrf_gpio_pin_clear(2);
//			
//		nrf_gpio_cfg_output(17);
//		nrf_gpio_pin_set(17);
		
    while (true)
    {				
			  
    }
}


/** @} */
