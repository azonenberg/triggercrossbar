/***********************************************************************************************************************
*                                                                                                                      *
* LATENTPACKET v0.1                                                                                                    *
*                                                                                                                      *
* Copyright (c) 2023 Andrew D. Zonenberg and contributors                                                              *
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

#include "ibc.h"

//UART console
UART* g_uart = NULL;
Logger g_log;

Timer* g_logTimer;

void InitClocks();
void InitUART();
void InitLog();
void DetectHardware();
void InitI2C();
void InitSensors();
uint16_t ReadThermalSensor(uint8_t addr = 0x90);

GPIOPin* g_fault_led = nullptr;
GPIOPin* g_ok_led = nullptr;

I2C* g_i2c = nullptr;

int main()
{
	//Copy .data from flash to SRAM (for some reason the default newlib startup won't do this??)
	memcpy(&__data_start, &__data_romstart, &__data_end - &__data_start + 1);

	//Enable SYSCFG before changing any settings on it
	//RCCHelper::EnableSyscfg();

	//Hardware setup
	InitClocks();
	InitUART();
	InitLog();
	DetectHardware();
	InitI2C();
	InitSensors();

	//Initalize our GPIOs and make sure all rails are off
	GPIOPin ok_led(&GPIOA, 11, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
	GPIOPin fault_led(&GPIOA, 12, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);

	ok_led = 0;
	fault_led = 0;

	//Save pointers to all the rails for use in other functions
	g_ok_led = &ok_led;
	g_fault_led = &fault_led;

	//Wait 5 seconds in case something goes wrong during first power up
	g_log("5 second delay\n");
	g_logTimer->Sleep(50000);

	ok_led = 1;

	//Poll for problems
	while(1)
	{
		/*
		//Check for overflows on our log message timer
		g_log.UpdateOffset(60000);

		PollFPGA();

		MonitorRail(pgood_3v3, "3V3");
		MonitorRail(pgood_2v5, "2V5");
		MonitorRail(pgood_1v8, "1V8");
		MonitorRail(pgood_1v2, "1V2");
		MonitorRail(pgood_1v0, "1V0");
		*/
	}
	return 0;
}

void InitI2C()
{
	g_log("Initializing I2C interface\n");

	static GPIOPin i2c_scl(&GPIOB, 7, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_SLOW, 1, true);
	static GPIOPin i2c_sda(&GPIOB, 8, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_SLOW, 4, true);

	//I2C1 runs off our kernel clock (16 MHz)
	//Prescale by 4 to get 4 MHz
	//Divide by 40 after that to get 100 kHz
	static I2C i2c(&I2C1, 4, 40);
	g_i2c = &i2c;
}

void InitClocks()
{
	//No need to mess with flash wait states
	//At full voltage (Range 1) we can run up to 16 MHz with no wait states, which is more than adequate for our needs
	RCCHelper::InitializePLLFromHSI16(
		4,	//VCO at 16*4 = 64 MHz
		4,	//CPU frequency is 64/4 = 16 MHz (max 32)
		1,	//AHB at 16 MHz (max 32)
		1,	//APB2 at 16 MHz (max 32)
		1);	//APB1 at 16 MHz (max 32)
}

void InitUART()
{
	//Initialize the UART for local console: 115.2 Kbps using PB6 for USART2 transmit and PA15 for USART2 receive
	//TODO: nice interface for enabling UART interrupts
	GPIOPin uart_tx(&GPIOB, 6, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_SLOW, 0);
	GPIOPin uart_rx(&GPIOA, 15, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_SLOW, 0);

	//USART2 is on APB1 (16MHz), so we need a divisor of 138.88, round to 139
	static UART uart(&USART2, 139);
	g_uart = &uart;

	/*
	//Enable the UART RX interrupt
	//TODO: Make an RCC method for this
	volatile uint32_t* NVIC_ISER1 = (volatile uint32_t*)(0xe000e104);
	*NVIC_ISER1 = 0x100000;
	*/

	//Clear screen and move cursor to X0Y0
	uart.Printf("\x1b[2J\x1b[0;0H");
}

void InitLog()
{
	//APB1 is 32 MHz
	//Divide down to get 10 kHz ticks
	static Timer logtim(&TIMER2, Timer::FEATURE_GENERAL_PURPOSE_16BIT, 3200);
	g_logTimer = &logtim;

	g_log.Initialize(g_uart, &logtim);
	g_log("UART logging ready\n");
}

void DetectHardware()
{
	g_log("Identifying hardware\n");
	LogIndenter li(g_log);

	uint16_t rev = DBGMCU.IDCODE >> 16;
	uint16_t device = DBGMCU.IDCODE & 0xfff;

	//Category 2
	if(device == 0x425)
	{
		//Look up the stepping number
		const char* srev = NULL;
		switch(rev)
		{
			case 0x1000:
				srev = "A";
				break;

			case 0x2000:
				srev = "B";
				break;

			case 0x2008:
				srev = "Y";
				break;

			case 0x2018:
				srev = "X";
				break;

			default:
				srev = "(unknown)";
		}

		g_log("STM32L031/041 stepping %s\n", srev);
		g_log("8 kB total SRAM, 1 kB EEPROM, 20 byte backup SRAM\n");
		g_log("%d kB Flash\n", F_ID);

		uint8_t waferNum = (U_ID[0] >> 24) & 0xff;
		char waferLot[8] =
		{
			static_cast<char>((U_ID[0] >> 16) & 0xff),
			static_cast<char>((U_ID[0] >> 8) & 0xff),
			static_cast<char>((U_ID[0] >> 0) & 0xff),
			static_cast<char>((U_ID[1] >> 24) & 0xff),
			static_cast<char>((U_ID[1] >> 16) & 0xff),
			static_cast<char>((U_ID[1] >> 8) & 0xff),
			static_cast<char>((U_ID[1] >> 0) & 0xff),
			'\0'
		};
		g_log("Lot %s, wafer %d, unique ID 0x%08x\n", waferLot, waferNum, U_ID[2]);
	}
	else
		g_log(Logger::WARNING, "Unknown device (0x%06x)\n", device);
}

void InitSensors()
{
	g_log("Initializing sensors\n");
	LogIndenter li(g_log);

	//Set temperature sensor to max resolution
	uint8_t addr = 0x90;
	uint8_t cmd[3] = {0x01, 0x60, 0x00};
	if(!g_i2c->BlockingWrite(addr, cmd, sizeof(cmd)))
		g_log(Logger::ERROR, "Failed to initialize I2C temp sensor at 0x%02x\n", addr);

	//Print value
	auto temp = ReadThermalSensor(addr);
	g_log("Temperature: %d.%02d C\n", (temp >> 8), static_cast<int>(((temp & 0xff) / 256.0) * 100));
}

/**
	@brief Read a temperature sensor at the given I2C address and return the temperature (in 8.8 fixed point format)
 */
uint16_t ReadThermalSensor(uint8_t addr)
{
	if(!g_i2c->BlockingWrite8(addr, 0x00))
		return 0xff;
	uint16_t reply;
	if(!g_i2c->BlockingRead16(addr, reply))
		return 0xff;

	return reply;
}
