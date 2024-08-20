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
#include <supervisor/SupervisorSPIServer.h>

//TODO: fix this path somehow?
#include "../../../../common-ibc/firmware/main/regids.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// System status indicator LEDs

GPIOPin g_faultLED(&GPIOH, 1, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
GPIOPin g_sysokLED(&GPIOH, 0, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Power rail descriptors

//12V ramp rate is slew rate controlled to about 2 kV/sec, so should take 0.5 ms to come up
//Give it 5 ms to be safe (plus extra delay from UART messages) and check with the ADC
GPIOPin g_12v0_en(&GPIOB, 5, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
RailDescriptor12V0 g_12v0("12V0", g_12v0_en, g_logTimer, 50);

GPIOPin g_1v0_en(&GPIOB, 9, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
GPIOPin g_1v0_pgood(&GPIOB, 8, GPIOPin::MODE_INPUT, GPIOPin::SLEW_SLOW);
RailDescriptorWithEnableAndPGood g_1v0("1V0", g_1v0_en, g_1v0_pgood, g_logTimer, 75);

GPIOPin g_gtx_1v0_en(&GPIOB, 10, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
GPIOPin g_gtx_1v0_pgood(&GPIOA, 2, GPIOPin::MODE_INPUT, GPIOPin::SLEW_SLOW);
RailDescriptorWithEnableAndPGood g_gtx_1v0("GTX_1V0", g_gtx_1v0_en, g_gtx_1v0_pgood, g_logTimer, 75);

//1V2 PGOOD also seems to be intermittent, ignore
GPIOPin g_1v2_en(&GPIOC, 13, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
GPIOPin g_1v2_pgood(&GPIOC, 0, GPIOPin::MODE_INPUT, GPIOPin::SLEW_SLOW);
RailDescriptorWithEnable g_1v2("1V2", g_1v2_en, g_logTimer, 75);

//1V8 PGOOD isn't working (suspected solder defect) so ignore it
GPIOPin g_1v8_en(&GPIOB, 14, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
//GPIOPin g_1v8_pgood(&GPIOB, 13, GPIOPin::MODE_INPUT, GPIOPin::SLEW_SLOW);
RailDescriptorWithEnable g_1v8("1V8", g_1v8_en, g_logTimer, 75);

GPIOPin g_gtx_1v8_en(&GPIOB, 2, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
GPIOPin g_gtx_1v8_pgood(&GPIOB, 0, GPIOPin::MODE_INPUT, GPIOPin::SLEW_SLOW);
RailDescriptorWithEnableAndPGood g_gtx_1v8("GTX_1V8", g_gtx_1v8_en, g_gtx_1v8_pgood, g_logTimer, 75);

GPIOPin g_3v3_en(&GPIOB, 4, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
GPIOPin g_3v3_pgood(&GPIOB, 3, GPIOPin::MODE_INPUT, GPIOPin::SLEW_SLOW);
RailDescriptorWithEnableAndPGood g_3v3("3V3", g_3v3_en, g_3v3_pgood, g_logTimer, 75);

//3V0_N is C14 but we aren't using that rail
//since the input comparator didn't work for reasons unknown. Probably safer to just leave it off.

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Power rail sequence

etl::vector g_powerSequence
{
	//12V has to come up first since it supplies everything else
	(RailDescriptor*)&g_12v0,					//need to cast at least one entry to base class
												//for proper template deduction

	//VCCINT - VCCAUX - VCCO for the FPGA
	&g_1v0,
	&g_gtx_1v0,
	&g_1v8,
	&g_gtx_1v8,

	//3V3 needs to come up after 1V8 to ensure VCCO-VCCAUX < 2.625V at all times
	&g_3v3,

	//1V2 rail for the PHY should come up after 3.3V rail (note 1 on page 62)
	//also GTX_1V2 (same rail) should turn on at or after GTX_1V0)
	&g_1v2
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Reset descriptors

//Active low edge triggered reset
GPIOPin g_fpgaResetN(&GPIOA, 1, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW, 0, true);
ActiveLowResetDescriptor g_fpgaResetDescriptor(g_fpgaResetN, "FPGA PROG");

//Active low level triggered delay-boot flag
//Use this as the "FPGA is done booting" indicator
GPIOPin g_fpgaInitN(&GPIOB, 12, GPIOPin::MODE_OUTPUT, 0, true);
GPIOPin g_fpgaDone(&GPIOB, 11, GPIOPin::MODE_INPUT, 0, false);
ActiveLowResetDescriptorWithActiveHighDone g_fpgaInitDescriptor(g_fpgaInitN, g_fpgaDone, "FPGA INIT");

//MCU reset comes at the end
GPIOPin g_mcuResetN(&GPIOA, 3, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
GPIOPin g_mcuReady(&GPIOA, 0, GPIOPin::MODE_INPUT, 0, false);
ActiveLowResetDescriptorWithActiveHighDone g_mcuResetDescriptor(g_mcuResetN, g_mcuReady, "MCU");

etl::vector g_resetSequence
{
	//First boot the FPGA
	(ResetDescriptor*)&g_fpgaResetDescriptor,	//need to cast at least one entry to base class
												//for proper template deduction
	&g_fpgaInitDescriptor,

	//then release the MCU
	&g_mcuResetDescriptor,
};
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// The top level supervisor controller

CrossbarPowerResetSupervisor g_super(g_powerSequence, g_resetSequence);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Peripheral initialization

void App_Init()
{
	//RCCHelper::Enable(&_RTC);

	//Format version string
	StringBuffer buf(g_version, sizeof(g_version));
	static const char* buildtime = __TIME__;
	buf.Printf("%s %c%c%c%c%c%c",
		__DATE__, buildtime[0], buildtime[1], buildtime[3], buildtime[4], buildtime[6], buildtime[7]);
	g_log("Firmware version %s\n", g_version);

	g_super.PowerOn();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Local sensors

uint16_t Get12VRailVoltage()
{
	//12V rail output is ADC_IN9
	//5.094x division so one LSB = 4.105 mV at the input nominally
	//Tweak with hard coded trim constants for now; TODO make thse go in flash
	return g_adc->ReadChannel(9) * 4079 / 1000;
}

void PrintIBCSensors()
{
	{
		g_log("IBC status\n");
		LogIndenter li(g_log);
		g_log("Temperature = %uhk C\n", g_ibcTemp);
		g_log("vin         = %2d.%03d V\n", g_vin48 / 1000, g_vin48 % 1000);
		g_log("vout        = %2d.%03d V\n", g_vout12 / 1000, g_vout12 % 1000);
		g_log("vsense      = %2d.%03d V\n", g_voutsense / 1000, g_voutsense % 1000);
		g_log("iin         = %2d.%03d A\n", g_iin / 1000, g_iin % 1000);
		g_log("iout        = %2d.%03d A\n", g_iout / 1000, g_iout % 1000);
	}

	auto v = Get12VRailVoltage();
	g_log("Local 12V0      = %2d.%03d V\n", v / 1000, v % 1000);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Main loop

void UpdateLEDs();
void UpdateResets();
void CheckButtons();
void CheckPowerFailure();

const int g_logTimerMax = 60000;

void BSP_MainLoopIteration()
{
	//Handle wraps of the timer in the logger
	g_log.UpdateOffset(g_logTimerMax);

	//Core supervisor state machine
	UpdateLEDs();
	g_super.Iteration();

	CheckButtons();

	//Management and system health
	static SupervisorSPIServer spiserver(g_spi);
	if(PollIBCSensors())
		g_super.PrintIfPending();
	spiserver.Poll();
}

/**
	@brief Run the button state machine
 */
void CheckButtons()
{
	static GPIOPin pwr_button(&GPIOA, 12, GPIOPin::MODE_INPUT, 0, false);
	static GPIOPin rst_button(&GPIOA, 15, GPIOPin::MODE_INPUT, 0, false);

	static bool buttonDown = false;

	if(pwr_button && !buttonDown)
	{
		g_log("Power button pressed\n");
		if(g_super.IsPowerOn())
			g_super.PowerOff();
		else
			g_super.PowerOn();

	}

	buttonDown = pwr_button;
}

/**
	@brief Check for input power loss
 */
void CheckPowerFailure()
{

}

/**
	@brief Update the system status indicator LEDs
 */
void UpdateLEDs()
{
	//OK LED: off = down, blinking = booting, solid = normal operation
	if(g_super.IsPowerOn())
	{
		if(g_super.IsResetsDone())
			g_sysokLED = true;

		else
		{
			//Blink OK LED at 2 Hz if reset state machine is running but we're waiting for everything to come up
			static uint32_t nextBlink = 0;
			const uint32_t blinkDelay = 2500;

			auto now = g_logTimer.GetCount();
			if(now >= nextBlink)
			{
				g_sysokLED = !g_sysokLED;
				nextBlink = now + blinkDelay;
				if(nextBlink > g_logTimerMax)
					nextBlink -= g_logTimerMax;
			}
		}
	}
	else
		g_sysokLED = false;
}
