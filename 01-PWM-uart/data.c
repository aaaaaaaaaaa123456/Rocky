#include <stdio.h>
#include <string.h>
#include "nrf_drv_pwm.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "boards.h"
#include "bsp.h"
#include "nrf_delay.h"
#include "data.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "app_uart.h"

	nrf_pwm_values_wave_form_t const seq_values_Tip[] =
	{
		//32pcs 25K hz --- 1281us
		{ 0x8000+320,      0,      0,      640 },  	// 640----25K hz
		
		//61pcs 1M hz --- 61us
		{ 0x8000,      0,      0,      16 },
					
		//40pcs 40K hz --- 1014us
		{ 0x8000+200,      0,      0,      400 },  	// 400----40K hz
				
		//114pcs 10K hz --- 11400us
		{ 0x8000,      0,      0,      1600 },
		
	};

// nrf_pwm_values_wave_form_t const seq_values_Tip[] =
//    {
//			//29pcs 205K hz --- 142us
//      { 0x8000+40,      0,      0,      79 },  	// 39----205K hz


//			//1pcs 100K hz --- 10us
//      { 0x8000,      0,      0,      160 },
//			
//			//28pcs 196K hz --- 142us
//			{ 0x8000+40,      0,      0,      83 },		// 41----196K hz

//			
//			//extra 2pcs 196K hz --- 142us
//			{ 0x8000+40,      0,      0,      83 },		// 41----196K hz
//			{ 0x8000+40,      0,      0,      83 },		// 41----196K hz
//			
//    };
	
	
	nrf_pwm_values_wave_form_t const seq_values_Ring[] =
	{
		
		//25pcs 178K hz --- 142us
		{ 0x8000+45,      0,      0,      91 }, 	// 45----178K hz
		
		//1pcs 100K hz --- 10us
		{ 0x8000,      0,      0,      160 },		
		
		//24pcs 169K hZ --- 142us
		{ 0x8000+48,      0,      0,      96 },	// 48----169K hz		
		
	};
		
	int number_Tip[]={32,61,40,114};
	int number_Ring[]={25,1,24};
	

		
//	void copy_data_Tip(nrf_pwm_values_wave_form_t  seq_values[],int start,int end)
//	{ 
//		int k,Tip_count=0;
//		
//		for(k=start;k<end;k++)
//			{					
//				seq_values[Tip_count] = seq_values_Tip[k];
//				Tip_count++;
//			}

//	}
//	
//	void copy_data_Ring(nrf_pwm_values_wave_form_t  seq_values[],int start,int end)
//	{ 
//		int k,Ring_count=0;
//		
//		for(k=start;k<end;k++)
//			{					
//				seq_values[Ring_count] = seq_values_Ring[k];
//				Ring_count++;
//			}

//	}
