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

#include "supervisor.h"

//UART console
UART* g_uart = NULL;
Logger g_log;

Timer* g_logTimer;

void InitClocks();
void InitUART();
void InitLog();
void DetectHardware();

void StartRail(GPIOPin& en, GPIOPin& pgood, uint32_t timeout, const char* name);
//void StartSubRail(GPIOPin& en, GPIOPin& pgood, uint32_t timeout, const char* name);
void MonitorRail(GPIOPin& pgood, const char* name);
void PanicShutdown();

GPIOPin* g_en_12v0 = nullptr;
GPIOPin* g_en_1v0 = nullptr;
GPIOPin* g_en_gtx_1v0 = nullptr;
GPIOPin* g_en_1v2 = nullptr;
GPIOPin* g_en_1v8 = nullptr;
GPIOPin* g_en_gtx_1v8 = nullptr;
GPIOPin* g_en_3v3 = nullptr;
GPIOPin* g_en_3v0_n = nullptr;

GPIOPin* g_fail_led = nullptr;
GPIOPin* g_ok_led = nullptr;
/*

bool g_fpgaUp = false;
GPIOPin* g_fpga_done = nullptr;
GPIOPin* g_mcu_rst_n = nullptr;

void PollFPGA();
*/
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

	//Initalize our GPIOs and make sure all rails are off
	GPIOPin en_12v0(&GPIOB, 5, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
	GPIOPin en_1v0(&GPIOB, 9, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
	GPIOPin en_gtx_1v0(&GPIOB, 10, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
	GPIOPin en_1v2(&GPIOC, 13, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
	GPIOPin en_1v8(&GPIOB, 14, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
	GPIOPin en_gtx_1v8(&GPIOB, 2, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
	GPIOPin en_3v3(&GPIOB, 4, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
	GPIOPin en_3v0_n(&GPIOC, 13, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
	GPIOPin pgood_1v0(&GPIOB, 8, GPIOPin::MODE_INPUT, 0, false);
	GPIOPin pgood_gtx_1v0(&GPIOA, 2, GPIOPin::MODE_INPUT, 0, false);
	GPIOPin pgood_1v2(&GPIOC, 0, GPIOPin::MODE_INPUT, 0, false);
	GPIOPin pgood_1v8(&GPIOB, 13, GPIOPin::MODE_INPUT, 0, false);
	GPIOPin pgood_gtx_1v8(&GPIOB, 0, GPIOPin::MODE_INPUT, 0, false);
	GPIOPin pgood_3v3(&GPIOB, 3, GPIOPin::MODE_INPUT, 0, false);

	GPIOPin fail_led(&GPIOH, 1, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
	GPIOPin ok_led(&GPIOH, 0, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
	/*

	GPIOPin fpga_done(&GPIOB, 2, GPIOPin::MODE_INPUT, 0, false);
	GPIOPin mcu_rst_n(&GPIOB, 0, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
	GPIOPin fpga_rst_n(&GPIOA, 3, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW, 0, true);
	*/
	en_12v0 = 0;
	en_1v0 = 0;
	en_gtx_1v0 = 0;
	en_1v2 = 0;
	en_1v8 = 0;
	en_gtx_1v8 = 0;

	/*mcu_rst_n = 0;
	fpga_rst_n = 0;
	*/
	fail_led = 0;
	ok_led = 0;

	//Save pointers to all the rails for use in other functions
	g_en_12v0 = &en_12v0;
	g_en_1v0 = &en_1v0;
	g_en_gtx_1v0 = &en_gtx_1v0;
	g_en_1v2 = &en_1v2;
	g_en_1v8 = &en_1v8;
	g_en_gtx_1v8 = &en_gtx_1v8;
	g_en_3v3 = &en_3v3;
	g_en_3v0_n = &en_3v0_n;

	g_fail_led = &fail_led;
	g_ok_led = &ok_led;
	/*
	g_fpga_done = &fpga_done;
	g_mcu_rst_n = &mcu_rst_n;
	*/
	//Wait 5 seconds in case something goes wrong during first power up
	g_log("5 second delay\n");
	g_logTimer->Sleep(50000);

	//Before we can bring up anything else, the 12V rail has to come up
	//12V ramp rate is slew rate controlled to about 2 kV/sec, so should take 0.5 ms to come up
	//Give it 5 ms to be safe (plus extra delay from UART messages)
	//(we don't have any sensing on this rail so we have to just hope it came up)
	g_log("Turning on 12V0\n");
	en_12v0 = 1;
	g_logTimer->Sleep(50);

	//TODO: poll the 12V rail via our ADC and measure it to verify it's good

	//Now turn on the core power DC-DC's, giving them each 5ms to come up
	ok_led = 1;
	StartRail(en_1v0, pgood_1v0, 50, "1V0");
	//StartRail(en_gtx_1v0, pgood_1v0, 50, "GTX_1V0");

	//1V8 runs VCCAUX and should come up before VCCO
	//Turn on 1V8 and hope for the best
	g_log("Turning on 1V8, ignoring PGOOD\n");
	en_1v8 = 1;
	g_logTimer->Sleep(50);
	//StartRail(en_1v8, pgood_1v8, 50, "1V8");

	//Then GTX 1V8
	//StartRail(en_gtx_1v8, pgood_gtx_1v8, 50, "GTX_1V8");

	//Try to bring up 3V3 next
	StartRail(en_3v3, pgood_3v3, 50, "3V3");

	//TODO where should we sequence 1V2
	//StartRail(en_1v2, pgood_1v2, 50, "1V2");

	//Turn on 3V0_N last
	//g_log("Turning on 3V0_N (no PGOOD signal available)\n");
	//en_3v0_n = 1;

	/*

	//We really SHOULD start 1V8 then 3V3, but then U18 will have PVIN > AVIN and bad things will happen
	//to Vref / Vtt. This will be fixed in LATENTRED by running the LP2996 off 3V3_SB instead of 3V3.

	//DP83867 requires 2.5 analog up before 1.8 analog (no requirement for analog vs vccio rail)
	StartRail(en_2v5, pgood_2v5, 50, "2V5");

	//We then need to start 1V8 right away (within Tvcco2vccaux of starting 3V3) to avoid damage to the FPGA!
	//1.8V must be stable within 25ms of 2.5V ramping up
	StartRail(en_1v8, pgood_1v8, 50, "1V8");

	//1V8_IO and Vref/Vtt have no PGOOD feedback.
	//If it overcurrents, they will probably drag down the main 1V8 rail so check for faults over the next 10ms
	StartSubRail(en_1v8_io, pgood_1v8, 100, "1V8_IO");
	StartSubRail(en_vtt, pgood_1v8, 100, "Vtt + Vref");

	//All power rails came up if we get here
	g_log("Power is good, releasing FPGA reset\n");
	pwr_ok_led = 1;
	fpga_rst_n = 1;

	//Everything started up if we get here
	//TODO: wait for heartbeat from MCU too

	//Poll inputs and check to see if anything ever went out of spec
	//TODO: support warm reset and/or hard power cycle on request
	*/
	while(1)
	{
		//Check for overflows on our log message timer
		g_log.UpdateOffset(60000);

		//PollFPGA();

		MonitorRail(pgood_3v3, "3V3");
		//MonitorRail(pgood_2v5, "2V5");
		//MonitorRail(pgood_1v8, "1V8");
		//MonitorRail(pgood_gtx_1v8, "GTX_1V8");
		//MonitorRail(pgood_1v2, "1V2");
		MonitorRail(pgood_1v0, "1V0");
		//MonitorRail(pgood_gtx_1v0, "GTX_1V0");
	}

	return 0;
}

/*
void PollFPGA()
{
	bool done = g_fpga_done->Get();

	if(done && !g_fpgaUp)
	{
		g_all_ok_led->Set(true);
		g_log("FPGA is up, releasing MCU reset\n");
		g_mcu_rst_n->Set(true);
		g_fpgaUp = true;
	}

	else if(!done && g_fpgaUp)
	{
		g_all_ok_led->Set(false);
		g_log("FPGA is down, resetting MCU\n");
		g_mcu_rst_n->Set(false);
		g_fpgaUp = false;
	}
}*/

/**
	@brief Polls a power rail and look for signs of trouble
 */
void MonitorRail(GPIOPin& pgood, const char* name)
{
	if(!pgood)
	{
		PanicShutdown();
		g_log(Logger::ERROR, "PGOOD for rail %s went low, triggering panic shutdown\n", name);

		while(1)
		{}
	}
}

/**
	@brief Shuts down all rails in reverse order but without any added sequencing delays
 */
void PanicShutdown()
{
	g_en_3v0_n->Set(false);
	g_en_3v3->Set(false);
	g_en_1v2->Set(false);
	g_en_1v8->Set(false);
	g_en_gtx_1v8->Set(false);
	g_en_1v0->Set(false);
	g_en_gtx_1v0->Set(false);
	g_en_12v0->Set(false);

	g_fail_led->Set(1);
	g_ok_led->Set(0);
}

/**
	@brief Turns on a single power rail, checking for failure
 */
void StartRail(GPIOPin& en, GPIOPin& pgood, uint32_t timeout, const char* name)
{
	g_log("Turning on %s\n", name);

	en = 1;
	for(uint32_t i=0; i<timeout; i++)
	{
		if(pgood)
			return;
		g_logTimer->Sleep(1);
	}
	if(!pgood)
	{
		PanicShutdown();

		g_log(Logger::ERROR, "Rail %s failed to come up\n", name);

		while(1)
		{}
	}
}

/**
	@brief Turns on a sub-rail switched or regulated from a parent rail, but lacking its own PGOOD output
 */
 /*
void StartSubRail(GPIOPin& en, GPIOPin& pgood, uint32_t timeout, const char* name)
{
	//These rails have no PGOOD feedback.
	//If it overcurrents, it will probably drag down the parent rail so check for faults over a fixed time window
	g_log("Turning on %s\n", name);
	en = 1;
	for(uint32_t i=0; i<timeout; i++)
	{
		if(!pgood)
		{
			PanicShutdown();

			g_log(Logger::ERROR, "Rail %s failed to come up\n", name);

			while(1)
			{}
		}
		g_logTimer->Sleep(1);
	}
}
*/
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
	//Initialize the UART for local console: 115.2 Kbps using PB6 for USART2 transmit and PB7 for USART2 receive
	//TODO: nice interface for enabling UART interrupts
	GPIOPin uart_tx(&GPIOB, 6, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_SLOW, 0);
	GPIOPin uart_rx(&GPIOB, 7, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_SLOW, 0);

	//USART2 is on APB1 (16MHz), so we need a divisor of 138.88, round to 139
	static UART uart(&USART2, 139);
	g_uart = &uart;

	//Enable the UART RX interrupt
	//TODO: Make an RCC method for this
	//volatile uint32_t* NVIC_ISER1 = (volatile uint32_t*)(0xe000e104);
	// *NVIC_ISER1 = 0x100000;

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