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
#include "../ibc/i2cregs.h"
#include "superregs.h"
#include "IBCRegisterReader.h"
#include "TempSensorReader.h"
#include <util/StringBuffer.h>

//USART2 is on APB1 (32MHz), so we need a divisor of 277.77, round to 278
UART<32, 256> g_uart(&USART2, 278);
Logger g_log;

Timer* g_logTimer;

void InitPower();
void InitGPIOs();
void InitClocks();
void InitUART();
void InitLog();
void DetectHardware();
void InitADC();
void InitI2C();
void InitSensors();
void InitSPI();

void PrintIBCSensors();

void StartRail(GPIOPin& en, GPIOPin& pgood, uint32_t timeout, const char* name);
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

GPIOPin* g_pgood_1v0 = nullptr;
GPIOPin* g_pgood_1v2 = nullptr;
GPIOPin* g_pgood_gtx_1v0 = nullptr;
GPIOPin* g_pgood_gtx_1v8 = nullptr;
GPIOPin* g_pgood_3v3 = nullptr;

GPIOPin* g_fail_led = nullptr;
GPIOPin* g_ok_led = nullptr;

bool g_fpgaUp = false;
GPIOPin* g_fpga_done = nullptr;
GPIOPin* g_mcu_rst_n = nullptr;
GPIOPin* g_fpga_rst_n = nullptr;
GPIOPin* g_fpga_init_n = nullptr;

GPIOPin* g_softPower = nullptr;
GPIOPin* g_softReset = nullptr;

ADC* g_adc = nullptr;
I2C* g_i2c = nullptr;
uint16_t Get12VRailVoltage();
uint16_t ReadIBCRegister(uint8_t addr);
const uint8_t g_tempI2cAddress = 0x90;
const uint8_t g_ibcI2cAddress = 0x42;

void PollFPGA();

void PollPowerButtons();
bool PollPowerFailure();

enum
{
	STATE_OFF,			//nothing happening
	STATE_STARTING,		//startup but button may still be pressed
	STATE_ON,			//on, button released
	STATE_STOPPING,		//shutdown but button may still be pressed
} g_powerState = STATE_OFF;

//500 ms
const uint16_t g_blinkDelay = 5000;

//Timer for blinking LEDs
uint16_t g_nextBlink = 0;

void PowerOn();
void PowerOff();

//IBC version string
char g_ibcVersion[20] = {0};

//Our version string
char g_version[20] = {0};

SPI<64, 64> g_spi(&SPI1, true, 2, false);

GPIOPin* g_spiCS = nullptr;

//Latest IBC sensor readings
uint16_t g_vin48 = 0;
uint16_t g_vout12 = 0;
uint16_t g_voutsense = 0;
uint16_t g_ibcTemp = 0;
uint16_t g_iin = 0;
uint16_t g_iout = 0;
bool PollIBCSensors();

//Status output from the IBC
enum
{
	PRINT_IDLE,
	PRINT_REQUESTED,
	PRINT_REFRESHING
} g_ibcPrintState;

int main()
{
	//Copy .data from flash to SRAM (for some reason the default newlib startup won't do this??)
	memcpy(&__data_start, &__data_romstart, &__data_end - &__data_start + 1);

	//Enable SYSCFG before changing any settings on it
	RCCHelper::EnableSyscfg();

	//Hardware setup
	InitPower();
	InitClocks();
	InitUART();
	InitLog();
	DetectHardware();
	InitGPIOs();
	InitADC();
	InitI2C();
	InitSensors();
	InitSPI();

	//Format version string
	StringBuffer buf(g_version, sizeof(g_version));
	static const char* buildtime = __TIME__;
	buf.Printf("%s %c%c%c%c%c%c",
		__DATE__, buildtime[0], buildtime[1], buildtime[3], buildtime[4], buildtime[6], buildtime[7]);
	g_log("Firmware version %s\n", g_version);

	//Wait 5 seconds in case something goes wrong during first power up
	//g_log("5 second delay\n");
	//g_logTimer->Sleep(50000);
	g_log("Ready\n");

	//Run the normal power-on procedure
	//PowerOn();

	//Main event loop
	//TODO: support warm reset and/or hard power cycle on request
	uint8_t nbyte = 0;
	uint8_t cmd = 0;
	const uint16_t timerMax = 60000;
	while(1)
	{
		//Read and process SPI events
		if(g_spi.HasEvents())
		{
			auto event = g_spi.GetEvent();

			//Reset byte count on CS# rising or falling edge
			if(event.type == SPIEvent::TYPE_CS)
				nbyte = 0;

			//Process data byte
			else
			{
				auto data = event.data;

				//First byte is command
				if(nbyte == 0)
				{
					cmd = data;

					//Send reply data
					switch(cmd)
					{
						//Read our fimware version
						case SUPER_REG_VERSION:
							g_spi.NonblockingWriteFifo((uint8_t*)g_version, sizeof(g_version));
							break;

						//Read IBC firmware version
						case SUPER_REG_IBCVERSION:
							g_spi.NonblockingWriteFifo((uint8_t*)g_ibcVersion, sizeof(g_ibcVersion));
							break;

						//IBC sensors
						case SUPER_REG_IBCVIN:
							g_spi.NonblockingWriteFifo((uint8_t*)&g_vin48, sizeof(g_vin48));
							break;
						case SUPER_REG_IBCIIN:
							g_spi.NonblockingWriteFifo((uint8_t*)&g_iin, sizeof(g_iin));
							break;
						case SUPER_REG_IBCTEMP:
							g_spi.NonblockingWriteFifo((uint8_t*)&g_ibcTemp, sizeof(g_ibcTemp));
							break;
						case SUPER_REG_IBCVOUT:
							g_spi.NonblockingWriteFifo((uint8_t*)&g_vout12, sizeof(g_vout12));
							break;
						case SUPER_REG_IBCIOUT:
							g_spi.NonblockingWriteFifo((uint8_t*)&g_iout, sizeof(g_iout));
							break;
						case SUPER_REG_IBCVSENSE:
							g_spi.NonblockingWriteFifo((uint8_t*)&g_voutsense, sizeof(g_voutsense));
							break;
					}
				}

				nbyte ++;
			}
		}

		//Update sensors
		bool sensorsUpdated = PollIBCSensors();

		//Check for overflows on our log message timer
		g_log.UpdateOffset(timerMax);

		//Check for power button activity
		PollPowerButtons();

		//Blink power LED if rails are up but FPGA isn't
		if( ( (g_powerState == STATE_ON) || (g_powerState == STATE_STARTING) ) && !g_fpgaUp)
		{
			if(g_logTimer->GetCount() >= g_nextBlink)
			{
				*g_ok_led = !*g_ok_led;
				g_nextBlink = g_logTimer->GetCount() + g_blinkDelay;
				if(g_nextBlink > timerMax)
					g_nextBlink -= timerMax;
			}
		}

		//Normal operation
		if(g_powerState == STATE_ON)
		{
			//Check if 12V input power is lost
			if(sensorsUpdated)
			{
				if(PollPowerFailure())
				{
					//Continue to log 12V and 3.3V at 10ms intervals until we lose power,
					//so we can track how long the decay took
					g_log("Logging voltages until we lose power completely...\n");
					LogIndenter li(g_log);
					while(true)
					{
						auto vin = Get12VRailVoltage();
						auto v48 = ReadIBCRegister(IBC_REG_VIN);
						g_log("48V0:    %d.%03d V\n", v48/1000, v48 % 1000);
						g_log("12V0:    %d.%03d V\n", vin/1000, vin % 1000);
						g_log("3V3_SB:  %d mV\n", g_adc->GetSupplyVoltage());
						g_logTimer->Sleep(100);
					}
				}
			}

			//Check all rails (other than 1v8 which still has a solder defect on PGOOD that I don't feel like bodging)
			//to see if any of them went out of tolerance (indicating a short)
			//1V2 PGOOD also seems to have issues? skip check for the moment
			MonitorRail(*g_pgood_3v3, "3V3");
			//MonitorRail(pgood_1v8, "1V8");
			MonitorRail(*g_pgood_gtx_1v8, "GTX_1V8");
			//MonitorRail(*g_pgood_1v2, "1V2");
			MonitorRail(*g_pgood_1v0, "1V0");
			MonitorRail(*g_pgood_gtx_1v0, "GTX_1V0");

			//See if the FPGA went up or down
			PollFPGA();
		}
	}

	return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sensor interfacing

/**
	@brief Requests more sensor data from the IBC

	@return true if sensor values are updated
 */
bool PollIBCSensors()
{
	static IBCRegisterReader regreader;
	static TempSensorReader tempreader;

	static int state = 0;

	//Read the values
	switch(state)
	{
		case 0:
			if(tempreader.ReadTempNonblocking(g_ibcTemp))
				state ++;
			break;

		case 1:
			if(regreader.ReadRegisterNonblocking(IBC_REG_VIN, g_vin48))
				state ++;
			break;

		case 2:
			if(regreader.ReadRegisterNonblocking(IBC_REG_VOUT, g_vout12))
				state ++;
			break;

		case 3:
			if(regreader.ReadRegisterNonblocking(IBC_REG_VSENSE, g_voutsense))
				state ++;
			break;

		case 4:
			if(regreader.ReadRegisterNonblocking(IBC_REG_IIN, g_iin))
				state ++;
			break;

		case 5:
			if(regreader.ReadRegisterNonblocking(IBC_REG_IOUT, g_iout))
				state ++;
			break;

		//end of loop, wrap around
		default:

			//print sensor values if requested (power up or down)
			switch(g_ibcPrintState)
			{
				case PRINT_REQUESTED:
					g_ibcPrintState = PRINT_REFRESHING;
					break;

				case PRINT_REFRESHING:
					g_ibcPrintState = PRINT_IDLE;
					PrintIBCSensors();
					break;

				default:
					break;
			}

			state = 0;
			return true;
	}

	return false;
}

/**
	@brief Reads a 16-bit I2C register from the IBC
 */
uint16_t ReadIBCRegister(uint8_t addr)
{
	uint16_t ret = 0;
	g_i2c->BlockingWrite8(g_ibcI2cAddress, addr);
	g_i2c->BlockingRead16(g_ibcI2cAddress, ret);
	return ret;
}

uint16_t Get12VRailVoltage()
{
	//12V rail output is ADC_IN9
	//5.094x division so one LSB = 4.105 mV at the input nominally
	//Tweak with hard coded trim constants for now; TODO make thse go in flash
	return g_adc->ReadChannel(9) * 4079 / 1000;
}

void PrintIBCSensors()
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Soft power sequencing

/**
	@brief Check for power failure

	@return true if power failure detected
 */
bool PollPowerFailure()
{
	auto v12 = Get12VRailVoltage();
	if( (v12 < 11000) || (g_vin48 < 40000) )
	{
		g_log(Logger::ERROR, "Input power failure detected!\n");
		g_log("48V0:  %d.%03d V\n", g_vin48/1000, g_vin48 % 1000);
		g_log("12V0:  %d.%03d V\n", v12/1000, v12 % 1000);

		*g_fail_led = 0;
		PowerOff();
		g_powerState = STATE_OFF;

		g_log("Power failure process completed\n");
		v12 = Get12VRailVoltage();
		g_vin48 = ReadIBCRegister(IBC_REG_VIN);
		g_log("48V0:   %d.%03d V\n", g_vin48/1000, g_vin48 % 1000);
		g_log("12V0:   %d.%03d V\n", v12/1000, v12 % 1000);
		g_log("3V3_SB: %d mV\n", g_adc->GetSupplyVoltage());
		return true;
	}

	return false;
}

/**
	@brief Turn off all power rails and reset things cleanly
 */
void PowerOff()
{
	g_log("Beginning power-down sequence\n");
	LogIndenter li(g_log);

	*g_ok_led = 0;

	//Place FPGA and MCU in reset immediately
	//TODO: do we want to tip them off and give them some window of time to do a clean shutdown??
	*g_fpga_rst_n = 0;
	*g_fpga_init_n = 0;
	*g_mcu_rst_n = 0;

	//Shut rails off at 1ms intervals in reverse order of startup
	*g_en_3v0_n = 0;
	g_logTimer->Sleep(10);
	*g_en_3v3 = 0;
	g_logTimer->Sleep(10);
	*g_en_1v2 = 0;
	g_logTimer->Sleep(10);
	*g_en_1v8 = 0;
	g_logTimer->Sleep(10);
	*g_en_gtx_1v8 = 0;
	g_logTimer->Sleep(10);
	*g_en_1v0 = 0;
	g_logTimer->Sleep(10);
	*g_en_gtx_1v0 = 0;
	g_logTimer->Sleep(10);

	//Log 12V rail voltage before we turn off the load switch
	auto vin = Get12VRailVoltage();
	g_log("All regulators shut down\n");
	g_log("Measured 12V0 rail voltage: %d.%03d V\n", vin/1000, vin % 1000);
	g_log("Turning off 12V load switch\n");

	*g_en_12v0 = 0;
	g_powerState = STATE_STOPPING;
}

/**
	@brief Turn on all power rails and reset things cleanly
 */
void PowerOn()
{
	g_log("Beginning power-up sequence\n");
	LogIndenter li(g_log);

	//Hold everything in reset during powerup
	*g_fpga_rst_n = 0;
	*g_mcu_rst_n = 0;

	//Before we can bring up anything else, the 12V rail has to come up
	//12V ramp rate is slew rate controlled to about 2 kV/sec, so should take 0.5 ms to come up
	//Give it 5 ms to be safe (plus extra delay from UART messages)
	//(we don't have any sensing on this rail so we have to just hope it came up)
	g_log("Turning on 12V0\n");
	*g_en_12v0 = 1;
	g_logTimer->Sleep(50);

	//TODO: poll the 12V rail via our ADC and measure it to verify it's good
	auto vin = Get12VRailVoltage();
	g_log("Measured 12V0 rail voltage: %d.%03d V\n", vin/1000, vin % 1000);
	if(vin < 11000)
	{
		PanicShutdown();

		g_log(Logger::ERROR, "12V supply failed to come up\n");

		while(1)
		{}
	}

	//Now turn on the core power DC-DC's, giving them each 5ms to come up
	StartRail(*g_en_1v0, *g_pgood_1v0, 50, "1V0");
	StartRail(*g_en_gtx_1v0, *g_pgood_1v0, 50, "GTX_1V0");

	//1V8 runs VCCAUX and should come up before VCCO
	//Turn on 1V8 and hope for the best since PGOOD isn't working (probable short)
	g_log("Turning on 1V8, ignoring PGOOD\n");
	*g_en_1v8 = 1;
	g_logTimer->Sleep(50);
	//StartRail(en_1v8, pgood_1v8, 50, "1V8");

	//No recommended sequence for GTX_1V8, now is as good a time as any to turn it on
	StartRail(*g_en_gtx_1v8, *g_pgood_gtx_1v8, 50, "GTX_1V8");

	//3V3 needs to come up after 1V8 so that VCCO - VCCAUX (1V8) is always <2.625V
	StartRail(*g_en_3v3, *g_pgood_3v3, 100, "3V3");

	//1V2 should turn on at the same time or later than GTX_1V0
	StartRail(*g_en_1v2, *g_pgood_1v2, 100, "1V2");

	//Turn on 3V0_N last
	g_log("Turning on 3V0_N (no PGOOD signal available)\n");
	*g_en_3v0_n = 1;

	//Queue print of all IBC sensor values
	g_ibcPrintState = PRINT_REQUESTED;

	//S25FL128S datasheet calls for 300us minimum from Vcc high to reads
	//Give it 500 to be safe
	g_log("All power rails are up, waiting for flash Tpu\n");
	g_logTimer->Sleep(5000);

	//Start up the FPGA and have fun
	g_log("Releasing FPGA reset\n");
	*g_ok_led = 1;
	*g_fpga_rst_n = 1;
	*g_fpga_init_n = 1;

	//Everything started up if we get here
	//TODO: wait for heartbeat from MCU too?
	if(*g_softPower)
		g_powerState = STATE_STARTING;
	else
		g_powerState = STATE_ON;

	//Prepare to blink LED until FPGA comes up
	g_nextBlink = g_logTimer->GetCount() + g_blinkDelay;
}

/**
	@brief Check for soft power button presses
 */
void PollPowerButtons()
{
	//Power button state machine
	switch(g_powerState)
	{
		//Off to starting
		case STATE_OFF:
			if(*g_softPower)
			{
				g_log("Power button pressed\n");
				PowerOn();
			}
			break;

		//Starting up but button is possibly still pressed
		case STATE_STARTING:
			if(!*g_softPower)
			{
				g_log("Power button released\n");
				g_powerState = STATE_ON;
			}
			break;

		//Shutting down
		case STATE_ON:
			if(*g_softPower)
			{
				g_log("Power button pressed\n");
				PowerOff();
			}
			break;

		//Shutting down but button is possibly still pressed
		case STATE_STOPPING:
			if(!*g_softPower)
			{
				g_log("Power button released\n");
				g_powerState = STATE_OFF;
			}
			break;
	}

	//Reset button state machine
}

/**
	@brief Check for DONE pin state changes
 */
void PollFPGA()
{
	bool done = *g_fpga_done;

	if(done && !g_fpgaUp)
	{
		*g_ok_led = 1;
		g_log("FPGA is up, releasing MCU reset\n");
		*g_mcu_rst_n = 1;
		g_fpgaUp = true;
	}

	else if(!done && g_fpgaUp)
	{
		*g_ok_led = 0;
		g_log("FPGA is down, resetting MCU\n");
		*g_mcu_rst_n = 0;
		g_fpgaUp = false;
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Power on / off process

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
	*g_en_3v0_n = 0;
	*g_en_3v3 = 0;
	*g_en_1v2 = 0;
	*g_en_1v8 = 0;
	*g_en_gtx_1v8 = 0;
	*g_en_1v0 = 0;
	*g_en_gtx_1v0 = 0;
	*g_en_12v0 = 0;

	*g_fail_led = 1;
	*g_ok_led = 0;

	g_powerState = STATE_OFF;
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Peripheral initialization

void InitPower()
{
	RCCHelper::Enable(&PWR);
	Power::ConfigureLDO(RANGE_VOS1);
}

void InitClocks()
{
	//Configure the flash with wait states and prefetching before making any changes to the clock setup.
	//A bit of extra latency is fine, the CPU being faster than flash is not.
	Flash::SetConfiguration(32, RANGE_VOS1);

	//Set operating frequency
	RCCHelper::InitializePLLFromHSI16(
		4,	//VCO at 16*4 = 64 MHz
		2,	//CPU frequency is 64/2 = 32 MHz (max 32)
		1,	//AHB at 32 MHz (max 32)
		1,	//APB2 at 32 MHz (max 32)
		1);	//APB1 at 32 MHz (max 32)
}

void InitUART()
{
	//Initialize the UART for local console: 115.2 Kbps using PB6 for USART2 transmit and PB7 for USART2 receive
	//TODO: nice interface for enabling UART interrupts
	GPIOPin uart_tx(&GPIOB, 6, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_SLOW, 0);
	GPIOPin uart_rx(&GPIOB, 7, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_SLOW, 0);

	//Enable UART interrupts
	NVIC_EnableIRQ(28);
}

void InitLog()
{
	//APB1 is 32 MHz
	//Divide down to get 10 kHz ticks
	static Timer logtim(&TIMER2, Timer::FEATURE_GENERAL_PURPOSE_16BIT, 3200);
	g_logTimer = &logtim;

	//Wait 10ms to avoid resets during shutdown from destroying diagnostic output
	g_logTimer->Sleep(100);

	//Clear screen and move cursor to X0Y0
	g_uart.Printf("\x1b[2J\x1b[0;0H");

	//Start the logger
	g_log.Initialize(&g_uart, &logtim);
	g_log("UART logging ready\n");
	g_log("Firmware version %s\n", g_version);
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

void InitI2C()
{
	g_log("Initializing I2C interface\n");

	static GPIOPin i2c_scl(&GPIOA, 9, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_SLOW, 1, true);
	static GPIOPin i2c_sda(&GPIOA, 10, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_SLOW, 1, true);

	//I2C1 runs off our kernel clock (32 MHz)
	//Prescale by 8 to get 4 MHz
	//Divide by 10 after that to get 400 kHz
	static I2C i2c(&I2C1, 8, 10);
	g_i2c = &i2c;
}

void InitSensors()
{
	g_log("Initializing sensors\n");
	LogIndenter li(g_log);

	g_logTimer->Sleep(50);

	//Set temperature sensor to max resolution
	uint8_t cmd[3] = {0x01, 0x60, 0x00};
	if(!g_i2c->BlockingWrite(g_tempI2cAddress, cmd, sizeof(cmd)))
		g_log(Logger::ERROR, "Failed to initialize I2C temp sensor at 0x%02x\n", g_tempI2cAddress);

	//Print initial values from every sensor
	g_ibcPrintState = PRINT_REQUESTED;

	//Read IBC firmware version
	g_i2c->BlockingWrite8(g_ibcI2cAddress, IBC_REG_VERSION);
	g_i2c->BlockingRead(g_ibcI2cAddress, (uint8_t*)g_ibcVersion, sizeof(g_ibcVersion));
	g_log("IBC firmware version %s\n", g_ibcVersion);
}

void InitADC()
{
	g_log("Initializing ADC\n");
	LogIndenter li(g_log);

	//Enable ADC to run at PCLK/2 (8 MHz)
	static ADC adc(&ADC1, 2);

	//g_log("Zero calibration: %d\n", ADC1.CALFACT);
	//g_log("Temp cal 1: %d\n", TSENSE_CAL1);
	//g_log("Temp cal 2: %d\n", TSENSE_CAL2);
	//g_log("Vref cal: %d\n", VREFINT_CAL);
	//adc 18 is vsense (temp)
	//adc17 is vrefint (vcc)

	//Read the temperature
	//10us sampling time (80 ADC clocks) required for reading the temp sensor
	//79.5 is close enough
	adc.SetSampleTime(159);
	//auto temp = adc.GetTemperature();
	//g_log("MCU temperature: %uhk C\n", temp);
	g_log("3V3_SB:  %d mV\n", adc.GetSupplyVoltage());

	g_adc = &adc;
}

void InitGPIOs()
{
	g_log("Initializing GPIOs\n");

	//Initalize our GPIOs and make sure all rails are off
	static GPIOPin en_12v0(&GPIOB, 5, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
	static GPIOPin en_1v0(&GPIOB, 9, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
	static GPIOPin en_gtx_1v0(&GPIOB, 10, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
	static GPIOPin en_1v2(&GPIOC, 13, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
	static GPIOPin en_1v8(&GPIOB, 14, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
	static GPIOPin en_gtx_1v8(&GPIOB, 2, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
	static GPIOPin en_3v3(&GPIOB, 4, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
	static GPIOPin en_3v0_n(&GPIOC, 14, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);

	static GPIOPin pgood_1v0(&GPIOB, 8, GPIOPin::MODE_INPUT, 0, false);
	static GPIOPin pgood_gtx_1v0(&GPIOA, 2, GPIOPin::MODE_INPUT, 0, false);
	static GPIOPin pgood_1v2(&GPIOC, 0, GPIOPin::MODE_INPUT, 0, false);
	static GPIOPin pgood_1v8(&GPIOB, 13, GPIOPin::MODE_INPUT, 0, false);
	static GPIOPin pgood_gtx_1v8(&GPIOB, 0, GPIOPin::MODE_INPUT, 0, false);
	static GPIOPin pgood_3v3(&GPIOB, 3, GPIOPin::MODE_INPUT, 0, false);

	static GPIOPin fail_led(&GPIOH, 1, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
	static GPIOPin ok_led(&GPIOH, 0, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);

	static GPIOPin fpga_done(&GPIOB, 11, GPIOPin::MODE_INPUT, 0, false);
	static GPIOPin mcu_rst_n(&GPIOA, 3, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
	static GPIOPin mcu_ready(&GPIOA, 0, GPIOPin::MODE_INPUT, 0, false);
	static GPIOPin fpga_rst_n(&GPIOA, 1, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW, 0, true);
	static GPIOPin fpga_init(&GPIOB, 12, GPIOPin::MODE_OUTPUT, 0, true);

	static GPIOPin pwr_button(&GPIOA, 12, GPIOPin::MODE_INPUT, 0, false);
	static GPIOPin rst_button(&GPIOA, 15, GPIOPin::MODE_INPUT, 0, false);

	//R28 got knocked off the prototype during rework and I didn't feel like putting it back on
	//On die pullups FTW
	pgood_1v2.SetPullMode(GPIOPin::PULL_UP);

	en_12v0 = 0;
	en_1v0 = 0;
	en_gtx_1v0 = 0;
	en_1v2 = 0;
	en_1v8 = 0;
	en_gtx_1v8 = 0;

	mcu_rst_n = 0;
	fpga_rst_n = 0;
	fpga_init = 0;
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

	g_pgood_1v0 = &pgood_1v0;
	g_pgood_gtx_1v0 = &pgood_gtx_1v0;
	g_pgood_gtx_1v8 = &pgood_gtx_1v8;
	g_pgood_1v2 = &pgood_1v2;
	g_pgood_3v3 = &pgood_3v3;

	g_fail_led = &fail_led;
	g_ok_led = &ok_led;

	g_fpga_done = &fpga_done;
	g_mcu_rst_n = &mcu_rst_n;
	g_fpga_rst_n = &fpga_rst_n;
	g_fpga_init_n = &fpga_init;

	g_softPower = &pwr_button;
	g_softReset = &rst_button;
}

void InitSPI()
{
	g_log("Initializing SPI\n");

	//Set up GPIOs
	auto slew = GPIOPin::SLEW_MEDIUM;
	static GPIOPin spi_sck(&GPIOA, 5, GPIOPin::MODE_PERIPHERAL, slew, 0);
	static GPIOPin spi_mosi(&GPIOA, 7, GPIOPin::MODE_PERIPHERAL, slew, 0);
	static GPIOPin spi_miso(&GPIOA, 6, GPIOPin::MODE_PERIPHERAL, slew, 0);
	static GPIOPin spi_cs_n(&GPIOA, 4, GPIOPin::MODE_PERIPHERAL, slew, 0);

	//Save the CS# pin
	g_spiCS = &spi_cs_n;

	//Set up IRQ7 for SPI CS# (PA4) change
	//Use EXTI4 as PA4 interrupt on falling edge
	//TODO: make a wrapper for this?
	volatile uint32_t* NVIC_ISER = (volatile uint32_t*)(0xe000e100);
	NVIC_ISER[0] |= 0x80;
	SYSCFG.EXTICR2 = (SYSCFG.EXTICR2 & 0xfffffff8) | 0x0;
	EXTI.IMR |= 0x10;
	EXTI.FTSR |= 0x10;

	//Set up IRQ25 as SPI1 interrupt
	NVIC_ISER[0] |= 0x2000000;
	SPI1.CR2 |= SPI_RXNEIE;
}
