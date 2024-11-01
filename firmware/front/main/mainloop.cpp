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

#include "frontpanel.h"

void RefreshDisplay(bool full);

const uint8_t g_tempI2cAddress = 0x90;

//I2C2 runs off our APB1 clock (40 MHz)
//Prescale by 4 to get 10 MHz
//Divide by 100 after that to get 100 kHz
I2C g_i2c(&I2C2, 4, 100);

TCA6424A* g_expander = nullptr;

//Divide 40 MHz base clock by 128 to get 312.5 kHz SPI
//We can run up to 10 MHz for writes but readback Fmax is ~2 MHz
DisplaySPIType g_displaySPI(&SPI2, false, 128);

Display* g_display = nullptr;

GPIOPin* g_inmodeLED[4] = {nullptr};
GPIOPin* g_outmodeLED[4] = {nullptr};

void BSP_MainLoopIteration()
{
	//Check for overflows on our timer
	const int logTimerMax = 60000;
	if(g_log.UpdateOffset(logTimerMax))
	{
		for(auto t : g_timerTasks)
			t->OnTimerShift(logTimerMax);
	}

	//Run all of our regular tasks
	for(auto t : g_tasks)
		t->Iteration();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sensor interfacing

/**
	@brief Read a temperature sensor at the given I2C address and return the temperature (in 8.8 fixed point format)
 */
uint16_t ReadThermalSensor(uint8_t addr)
{
	if(!g_i2c.BlockingWrite8(addr, 0x00))
		return 0xff;
	uint16_t reply;
	if(!g_i2c.BlockingRead16(addr, reply))
		return 0xff;

	return reply;
}

