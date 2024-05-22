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
	@brief	Boot-time hardware intialization
 */
#include "frontpanel.h"
#include "TCA6424A.h"
#include "Display.h"
#include "regids.h"

void InitDisplaySPI();

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Peripheral initialization

void App_Init()
{
	InitGPIOs();
	InitI2C();
	InitSensors();
	InitExpander();
	InitDisplaySPI();
	InitDisplay();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Other hardware init

void InitGPIOs()
{
	g_log("Initializing GPIOs\n");

	static GPIOPin dirin_led0(&GPIOD, 10, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
	static GPIOPin dirin_led1(&GPIOC, 9, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
	static GPIOPin dirin_led2(&GPIOA, 11, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
	static GPIOPin dirin_led3(&GPIOA, 12, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);

	static GPIOPin dirout_led0(&GPIOE, 7, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
	static GPIOPin dirout_led1(&GPIOE, 9, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
	static GPIOPin dirout_led2(&GPIOE, 11, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
	static GPIOPin dirout_led3(&GPIOE, 14, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);

	g_inmodeLED[0] = &dirin_led0;
	g_inmodeLED[1] = &dirin_led1;
	g_inmodeLED[2] = &dirin_led2;
	g_inmodeLED[3] = &dirin_led3;

	g_outmodeLED[0] = &dirout_led0;
	g_outmodeLED[1] = &dirout_led1;
	g_outmodeLED[2] = &dirout_led2;
	g_outmodeLED[3] = &dirout_led3;

	//Turn on all port direction LEDs until we're initialized
	for(int i=0; i<4; i++)
	{
		*g_inmodeLED[i] = 1;
		*g_outmodeLED[i] = 1;
	}
}

void InitI2C()
{
	g_log("Initializing I2C interface\n");

	static GPIOPin i2c_scl(&GPIOB, 13, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_SLOW, 4, true);
	static GPIOPin i2c_sda(&GPIOB, 11, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_SLOW, 4, true);
	//PB12 is used as a route-through for SDA, so just leave it in the default floating state
}

void InitExpander()
{
	g_log("Initializing IO expander\n");

	//Clear reset
	static GPIOPin tca_rst(&GPIOB, 1, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
	tca_rst = 1;
	g_logTimer.Sleep(20);

	//Initialize the expander
	static TCA6424A expander(&g_i2c, 0x44);
	g_expander = &expander;

	//Set all the IOs as output and turn them all on until the FPGA configures us
	for(int i=0; i<24; i++)
	{
		expander.SetDirection(i, false);
		expander.SetOutputValue(i, true);
	}
}

void InitDisplaySPI()
{
	g_log("Initializing display SPI\n");

	//Set up GPIOs for display bus
	static GPIOPin display_sck(&GPIOD, 1, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_FAST, 5);
	//PD0 used as route through, leave tristated
	static GPIOPin display_mosi(&GPIOD, 4, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_FAST, 5);
}

void InitDisplay()
{
	g_log("Initializing display\n");
	LogIndenter li(g_log);

	//Set up GPIOs
	static GPIOPin display_busy_n(&GPIOA, 4, GPIOPin::MODE_INPUT, GPIOPin::SLEW_FAST);
	static GPIOPin display_bs(&GPIOA, 7, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_FAST);
	static GPIOPin display_cs_n(&GPIOC, 14, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_FAST);
	static GPIOPin display_dc(&GPIOH, 0, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_FAST);
	static GPIOPin display_rst_n(&GPIOH, 1, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_FAST);

	//BS pin seems to be a strap that just needs to be held low
	display_bs = 0;

	//Set up the display itself
	static Display display(&g_displaySPI, &display_busy_n, &display_cs_n, &display_dc, &display_rst_n);
	g_display = &display;

	//Do not do an initial display refresh, the main MCU will do that when it's ready

	//Change the SPI baud rate once the display has read the ROM
	//div 16 = 2.5 MHz, should be fine
	g_displaySPI.SetBaudDiv(32);
}

void InitSensors()
{
	g_log("Initializing sensors\n");
	LogIndenter li(g_log);

	//Set temperature sensor to max resolution
	uint8_t cmd[3] = {0x01, 0x60, 0x00};
	if(!g_i2c.BlockingWrite(g_tempI2cAddress, cmd, sizeof(cmd)))
		g_log(Logger::ERROR, "Failed to initialize I2C temp sensor at 0x%02x\n", g_tempI2cAddress);

	//Print initial values from every sensor
	g_log("Board temperature: %uhk C\n", ReadThermalSensor(g_tempI2cAddress));
}
