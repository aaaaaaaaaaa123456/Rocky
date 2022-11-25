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
 * @defgroup uart_example_main main.c
 * @{
 * @ingroup uart_example
 * @brief UART Example Application main file.
 *
 * This file contains the source code for a sample application using UART.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf.h"
#include "bsp.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_pwm.h"
//#include "nrf_drv_ppi.h"
#include "data.h"



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
int Count_Tip=0;	
int Count_Ring=0;	

extern nrf_pwm_values_wave_form_t const seq_values_Ring[];
extern nrf_pwm_values_wave_form_t const seq_values_Tip[];
extern	int number_Tip[4];
extern	int number_Ring[3];

nrf_pwm_values_wave_form_t /*const*/ seq_values_Tip0[1];
nrf_pwm_values_wave_form_t /*const*/ seq_values_Tip1[1];
nrf_pwm_values_wave_form_t /*const*/ seq_values_Ring0[1];
nrf_pwm_values_wave_form_t /*const*/ seq_values_Ring1[1];

static nrf_drv_pwm_t m_pwm0 = NRF_DRV_PWM_INSTANCE(0);
static nrf_drv_pwm_t m_pwm1 = NRF_DRV_PWM_INSTANCE(1);
static nrf_drv_pwm_t m_pwm2 = NRF_DRV_PWM_INSTANCE(2);


const nrf_drv_timer_t TIMER_LED = NRF_DRV_TIMER_INSTANCE(0);


#define USED_PWM(idx) (1UL << idx)
static uint8_t m_used = 0;

static void TipRing0(void);

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

static void ReleasePWM(void)
{
    if (m_used & USED_PWM(0))
    {
        nrf_drv_pwm_uninit(&m_pwm0);
    }
    if (m_used & USED_PWM(1))
    {
        nrf_drv_pwm_uninit(&m_pwm1);
    }
    if (m_used & USED_PWM(2))
    {
        nrf_drv_pwm_uninit(&m_pwm2);
    }
    m_used = 0;
}


static void TipHandler(nrfx_pwm_evt_type_t event_type)
{
	
	if (event_type == NRFX_PWM_EVT_END_SEQ0)
	{	
		Count_Tip = Count_Tip+2;
		seq_values_Tip0[0] = seq_values_Tip[Count_Tip]; 
		NRF_PWM0->SEQ[0].REFRESH = number_Tip[Count_Tip]-1;

		//printf("Count_Tip = %d\r\n",seq_values_Tip[Count_Tip].counter_top);

	}
	if (event_type == NRFX_PWM_EVT_END_SEQ1)
	{	
		seq_values_Tip1[0] = seq_values_Tip[Count_Tip+1];
		NRF_PWM0->SEQ[1].REFRESH = number_Tip[Count_Tip+1]-1;

		//printf("Tip_1\r\n");
	}

}

static void RingHandler(nrfx_pwm_evt_type_t event_type) 
{
	if (event_type == NRFX_PWM_EVT_END_SEQ0)
	{	
		Count_Ring = Count_Ring+2;
		seq_values_Ring0[0] = seq_values_Ring[Count_Ring]; 
		NRF_PWM1->SEQ[0].REFRESH = number_Ring[Count_Ring]-1;
		
		//printf("Count_Ring=%d\r\n",Count_Ring);
	}
	
	if (event_type == NRFX_PWM_EVT_END_SEQ1)
	{	
		seq_values_Ring1[0] = seq_values_Ring[Count_Ring]; 
		NRF_PWM1->SEQ[1].REFRESH = number_Ring[Count_Ring+1]-1;
		
		//printf("Ring_1\r\n");
	}

}


static void TipRing0(void)
{

	seq_values_Tip0[0] = seq_values_Tip[Count_Tip];
	seq_values_Tip1[0] = seq_values_Tip[Count_Tip+1];
	
	seq_values_Ring0[0] = seq_values_Ring[Count_Ring];  
	seq_values_Ring1[0] = seq_values_Ring[Count_Ring+1];  
	
	nrf_drv_pwm_config_t const configTip0 =
	{
			.output_pins =
			{
					BSP_LED_1 | NRF_DRV_PWM_PIN_INVERTED, // channel 0
					NRF_DRV_PWM_PIN_NOT_USED,             // channel 1
					NRF_DRV_PWM_PIN_NOT_USED,             // channel 2
					NRF_DRV_PWM_PIN_NOT_USED,             // channel 3
			},
			.irq_priority = APP_IRQ_PRIORITY_LOWEST,
			.base_clock   = NRF_PWM_CLK_16MHz,
			.count_mode   = NRF_PWM_MODE_UP,
			.top_value    = 0,												
			.load_mode    = NRF_PWM_LOAD_WAVE_FORM,
			.step_mode    = NRF_PWM_STEP_AUTO
	};
	nrf_drv_pwm_config_t const configRing0 =
	{
			.output_pins =
			{
					BSP_LED_0 | NRF_DRV_PWM_PIN_INVERTED, // channel 0
					NRF_DRV_PWM_PIN_NOT_USED,             // channel 1
					NRF_DRV_PWM_PIN_NOT_USED,             // channel 2
					NRF_DRV_PWM_PIN_NOT_USED,             // channel 3
			},
			.irq_priority = APP_IRQ_PRIORITY_LOWEST,
			.base_clock   = NRF_PWM_CLK_16MHz,
			.count_mode   = NRF_PWM_MODE_UP,
			.top_value    = 0,												
			.load_mode    = NRF_PWM_LOAD_WAVE_FORM,
			.step_mode    = NRF_PWM_STEP_AUTO
	};
	
	nrf_pwm_sequence_t const seq_Tip0 =
	{
			.values.p_wave_form = seq_values_Tip0,
			.length          = NRF_PWM_VALUES_LENGTH(seq_values_Tip0),
			.repeats         = number_Tip[Count_Tip]-1,
			.end_delay       = 0
	};

	 nrf_pwm_sequence_t const seq_Tip1 =
	{
			.values.p_wave_form = seq_values_Tip1,
			.length          = NRF_PWM_VALUES_LENGTH(seq_values_Tip1),
			.repeats         = number_Tip[Count_Tip+1]-1,
			.end_delay       = 0
	};
		
	nrf_pwm_sequence_t const seq_Ring0 =
	{
			.values.p_wave_form = seq_values_Ring0,
			.length          = NRF_PWM_VALUES_LENGTH(seq_values_Ring0),
			.repeats         = number_Ring[Count_Ring]-1,
			.end_delay       = 0
	};
	
	nrf_pwm_sequence_t const seq_Ring1 =
	{
			.values.p_wave_form = seq_values_Ring1,
			.length          = NRF_PWM_VALUES_LENGTH(seq_values_Ring1),
			.repeats         = number_Ring[Count_Ring+1]-1,
			.end_delay       = 0
	};
	
	ReleasePWM();

//    APP_ERROR_CHECK(nrf_drv_pwm_init(&m_pwm0, &configTip0, NULL));
//    APP_ERROR_CHECK(nrf_drv_pwm_init(&m_pwm1, &configRing0, NULL));
		
	APP_ERROR_CHECK(nrf_drv_pwm_init(&m_pwm0, &configTip0, TipHandler));
	APP_ERROR_CHECK(nrf_drv_pwm_init(&m_pwm1, &configRing0, RingHandler));

	
	m_used |= USED_PWM(0);
	m_used |= USED_PWM(1);

//    (void)nrf_drv_pwm_simple_playback(&m_pwm0, &seq_Tip0, 1, NRFX_PWM_FLAG_LOOP);
//    (void)nrf_drv_pwm_simple_playback(&m_pwm1, &seq_Ring0, 1, NRFX_PWM_FLAG_LOOP);

//		uint32_t start_pwm0_task_addr = nrf_drv_pwm_simple_playback(&m_pwm0, &seq_Tip0, 1, NRFX_PWM_FLAG_STOP| NRF_DRV_PWM_FLAG_START_VIA_TASK);               
//    uint32_t *start_pwm0_task = (uint32_t *) start_pwm0_task_addr;     
//    uint32_t start_pwm1_task_addr = nrf_drv_pwm_simple_playback(&m_pwm1, &seq_Ring0, 1, NRFX_PWM_FLAG_STOP| NRF_DRV_PWM_FLAG_START_VIA_TASK);               
//    uint32_t *start_pwm1_task = (uint32_t *) start_pwm1_task_addr;
//    *start_pwm0_task = 1;   
//    *start_pwm1_task = 1;
	
//	(void)nrf_drv_pwm_complex_playback(&m_pwm0, &seq_Tip0, &seq_Tip1, (sizeof(number_Tip)/sizeof(number_Tip[0])), NRFX_PWM_FLAG_STOP|NRF_DRV_PWM_FLAG_SIGNAL_END_SEQ0 |NRF_DRV_PWM_FLAG_SIGNAL_END_SEQ1);
//	(void)nrf_drv_pwm_complex_playback(&m_pwm1, &seq_Ring0, &seq_Ring1, (sizeof(number_Ring)/sizeof(number_Ring[0])), NRFX_PWM_FLAG_STOP|NRF_DRV_PWM_FLAG_SIGNAL_END_SEQ0 |NRF_DRV_PWM_FLAG_SIGNAL_END_SEQ1);
	

	uint32_t start_pwm0_task_addr = nrf_drv_pwm_complex_playback(&m_pwm0, &seq_Tip0, &seq_Tip1, (sizeof(number_Tip)/sizeof(number_Tip[0])), NRFX_PWM_FLAG_STOP | NRF_DRV_PWM_FLAG_START_VIA_TASK|NRF_DRV_PWM_FLAG_SIGNAL_END_SEQ0 | NRF_DRV_PWM_FLAG_SIGNAL_END_SEQ1);               
	uint32_t *start_pwm0_task = (uint32_t *) start_pwm0_task_addr;     
	uint32_t start_pwm1_task_addr = nrf_drv_pwm_complex_playback(&m_pwm1, &seq_Ring0, &seq_Ring1, (sizeof(number_Ring)/sizeof(number_Ring[0])), NRFX_PWM_FLAG_STOP | NRF_DRV_PWM_FLAG_START_VIA_TASK|NRF_DRV_PWM_FLAG_SIGNAL_END_SEQ0 | NRF_DRV_PWM_FLAG_SIGNAL_END_SEQ1);               
	uint32_t *start_pwm1_task = (uint32_t *) start_pwm1_task_addr;
	*start_pwm0_task = 1;   
	*start_pwm1_task = 1;	
					 
}




static void demoFSK(void)
{
	TipRing0();
}

static void demoBPSK(void)
{
    
    nrf_drv_pwm_config_t const config0 =
    {
        .output_pins =
        {
            BSP_LED_0 | NRF_DRV_PWM_PIN_INVERTED, // channel 0
            NRF_DRV_PWM_PIN_NOT_USED,             // channel 1
            NRF_DRV_PWM_PIN_NOT_USED,             // channel 2
            NRF_DRV_PWM_PIN_NOT_USED,             // channel 3
        },
        .irq_priority = APP_IRQ_PRIORITY_LOWEST,
        .base_clock   = NRF_PWM_CLK_16MHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = 42,
        .load_mode    = NRF_PWM_LOAD_COMMON,
        .step_mode    = NRF_PWM_STEP_AUTO
    };

    // This array cannot be allocated on stack (hence "static") and it must
    // be in RAM (hence no "const", though its content is not changed).

    static uint16_t /*const*/ seq_valuesBit[] =
    {
        /*  0   */
        0x8000,
             0,
        0x8000,
             0,
        0x8000,
             0,
        /*  1   */
             0,
        0x8000,
             0,
        0x8000,
             0,
        0x8000,
    };
    nrf_pwm_sequence_t const seqBit =
    {
        .values.p_common = seq_valuesBit,
        .length          = NRF_PWM_VALUES_LENGTH(seq_valuesBit),
        .repeats         = 0,
        .end_delay       = 0
    };


    APP_ERROR_CHECK(nrf_drv_pwm_init(&m_pwm0, &config0, NULL));

    m_used |= USED_PWM(0);
    //(void)nrf_drv_pwm_simple_playback(&m_pwm0, &seqBit, 100, NRF_DRV_PWM_FLAG_STOP);
		(void)nrf_drv_pwm_simple_playback(&m_pwm0, &seqBit, 100, NRF_DRV_PWM_FLAG_LOOP);
}
/**
 * @brief Function for main application entry.
 */
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
    printf("\r\nMPP example started.\r\n");
#endif	   
  	
		demoFSK();
		

    for (;;)
    {
        // Wait for an event.
        __WFE();

        // Clear the event register.
        __SEV();
        __WFE();
			
    }
 
}




