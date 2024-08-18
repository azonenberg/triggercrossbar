/***********************************************************************************************************************
*                                                                                                                      *
* trigger-crossbar                                                                                                     *
*                                                                                                                      *
* Copyright (c) 2023-2024 Andrew D. Zonenberg and contributors                                                         *
* All rights reserved.                                                                                                 *
*                                                                                                                      *
* Redistribution and use in source and binary forms, with or without modification, are permitted provided that the     *
* following conditions are met:                                                                                        *
*                                                                                                                      *
*    * Redistributions of source code must retain the above copyright notice, this list of conditions, and the         *
*      following disclaimer.                                                                                           *
*                                                                                                                      *
*    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the       *
*      following disclaimer in the documentation and/or other materials provided with the distribution.                *
*                                                                                                                      *
*    * Neither the name of the author nor the names of any contributors may be used to endorse or promote products     *
*      derived from this software without specific prior written permission.                                           *
*                                                                                                                      *
* THIS SOFTWARE IS PROVIDED BY THE AUTHORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED   *
* TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL *
* THE AUTHORS BE HELD LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES        *
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR       *
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE       *
* POSSIBILITY OF SUCH DAMAGE.                                                                                          *
*                                                                                                                      *
***********************************************************************************************************************/

/**
	@file
	@author	Andrew D. Zonenberg
	@brief	Boot-time hardware initialization
 */
#include <core/platform.h>
#include <supervisor/supervisor-common.h>
#include "hwinit.h"
#include <peripheral/Power.h>

void InitSPI();

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Common global hardware config used by both bootloader and application

//UART console
//USART2 is on APB1 (32MHz), so we need a divisor of 277.77, round to 278
UART<16, 256> g_uart(&USART2, 278);

//SPI bus to the main MCU
SPI<64, 64> g_spi(&SPI1, true, 2, false);

//I2C1 runs off our kernel clock (32 MHz)
//Prescale by 8 to get 4 MHz
//Divide by 10 after that to get 400 kHz
I2C g_i2c(&I2C1, 8, 10);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Low level init

void BSP_InitUART()
{
	//Initialize the UART for local console: 115.2 Kbps
	GPIOPin uart_tx(&GPIOB, 6, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_SLOW, 0);
	GPIOPin uart_rx(&GPIOB, 7, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_SLOW, 0);

	g_logTimer.Sleep(10);	//wait for UART pins to be high long enough to remove any glitches during powerup

	//Enable the UART interrupt
	NVIC_EnableIRQ(28);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Common features shared by both application and bootloader

void BSP_Init()
{
	//Bring up the IBC before powering up the rest of the system
	InitGPIOs();
	Super_Init();
	InitSPI();

	App_Init();
}

void InitSPI()
{
	g_log("Initializing management SPI bus\n");

	//Set up GPIOs for management bus
	static GPIOPin mgmt_sck(&GPIOA, 5, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_FAST, 0);
	static GPIOPin mgmt_mosi(&GPIOA, 7, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_FAST, 0);
	static GPIOPin mgmt_cs_n(&GPIOA, 4, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_FAST, 0);
	static GPIOPin mgmt_miso(&GPIOA, 6, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_FAST, 0);

	//Save the CS# pin
	g_spiCS = &mgmt_cs_n;

	//Set up IRQ7 for SPI CS# (PA4) change
	RCCHelper::EnableSyscfg();
	NVIC_EnableIRQ(7);
	EXTI::SetExtInterruptMux(4, EXTI::PORT_A);
	EXTI::EnableChannel(4);
	EXTI::EnableFallingEdgeTrigger(4);

	//Set up IRQ25 as SPI1 interrupt
	NVIC_EnableIRQ(25);
	g_spi.EnableRxInterrupt();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// GPIOs for all of the rail enables

void InitGPIOs()
{
	g_log("Initializing GPIOs\n");

	//all PGOOD have external resistors on this board, no need to add our own
	//... except R28 got knocked off the prototype during rework and I didn't feel like putting it back on
	g_1v2_pgood.SetPullMode(GPIOPin::PULL_UP);

	//turn off all LEDs
	g_faultLED = 0;
	g_sysokLED = 0;

	//Set up GPIOs for I2C bus
	static GPIOPin i2c_scl(&GPIOA, 9, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_SLOW, 1, true);
	static GPIOPin i2c_sda(&GPIOA, 10, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_SLOW, 1, true);
}
