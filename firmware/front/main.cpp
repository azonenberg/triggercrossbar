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
#include "TCA6424A.h"
#include "Display.h"

//UART console
UART* g_uart = nullptr;
Logger g_log;

Timer* g_logTimer = nullptr;

void InitPower();
void InitClocks();
void InitUART();
void InitLog();
void DetectHardware();
void InitGPIOs();
void InitI2C();
void InitSensors();
void InitExpander();
void InitSPI();
void InitDisplay();

const uint8_t g_tempI2cAddress = 0x90;

I2C* g_i2c = nullptr;
TCA6424A* g_expander = nullptr;

SPI* g_displaySPI = nullptr;

int main()
{
	//Copy .data from flash to SRAM (for some reason the default newlib startup won't do this??)
	memcpy(&__data_start, &__data_romstart, &__data_end - &__data_start + 1);

	//Hardware setup
	InitPower();
	InitClocks();
	InitUART();
	InitLog();
	DetectHardware();
	InitGPIOs();
	InitI2C();
	InitSensors();
	InitExpander();
	InitSPI();
	InitDisplay();

	g_log("Ready\n");

	//Main event loop
	while(1)
	{
		/*
		//Check for overflows on our log message timer
		g_log.UpdateOffset(60000);

		//Check for power button activity
		PollPowerButtons();

		//Blink power LED if rails are up but FPGA isn't
		if( ( (g_powerState == STATE_ON) || (g_powerState == STATE_STARTING) ) && !g_fpgaUp)
		{
			if(g_logTimer->GetCount() >= g_nextBlink)
			{
				*g_ok_led = !*g_ok_led;
				g_nextBlink = g_logTimer->GetCount() + g_blinkDelay;
			}
		}

		//Normal operation
		if(g_powerState == STATE_ON)
		{
			//Check if 12V input power is lost
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

			//Check all rails (other than 1v8 which still has a solder defect on PGOOD that I don't feel like bodging)
			//to see if any of them went out of tolerance (indicating a short)
			MonitorRail(*g_pgood_3v3, "3V3");
			//MonitorRail(pgood_1v8, "1V8");
			MonitorRail(*g_pgood_gtx_1v8, "GTX_1V8");
			MonitorRail(*g_pgood_1v2, "1V2");
			MonitorRail(*g_pgood_1v0, "1V0");
			MonitorRail(*g_pgood_gtx_1v0, "GTX_1V0");

			//See if the FPGA went up or down
			PollFPGA();
		}
		*/
	}

	return 0;
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
	Flash::SetConfiguration(80, RANGE_VOS1);

	RCCHelper::InitializePLLFromHSI16(
		2,	//Pre-divide by 2 (PFD frequency 8 MHz)
		20,	//VCO at 8*20 = 160 MHz
		4,	//Q divider is 40 MHz (nominal 48 but we're not using USB so this is fine)
		2,	//R divider is 80 MHz (fmax for CPU)
		1,	//no further division from SYSCLK to AHB (80 MHz)
		2,	//APB1 at 40 MHz
		2);	//APB2 at 40 MHz
}

void InitUART()
{
	//Initialize the UART for local console: 115.2 Kbps using PD4 for USART2 transmit and PD5 for USART2 receive
	//TODO: nice interface for enabling UART interrupts
	GPIOPin uart_tx(&GPIOD, 5, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_SLOW, 7);
	GPIOPin uart_rx(&GPIOD, 6, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_SLOW, 7);

	//USART2 is on APB1 (40 MHz), so we need a divisor of 347.22, round to 347
	static UART uart(&USART2, 347);
	g_uart = &uart;

	//Enable the UART RX interrupt
	//TODO: Make an RCC method for this
	//volatile uint32_t* NVIC_ISER1 = (volatile uint32_t*)(0xe000e104);
	// *NVIC_ISER1 = 0x100000;
}

void InitLog()
{
	//APB1 is 40 MHz
	//Divide down to get 10 kHz ticks
	static Timer logtim(&TIM2, Timer::FEATURE_GENERAL_PURPOSE_16BIT, 4000);
	g_logTimer = &logtim;

	//Wait 10ms to avoid resets during shutdown from destroying diagnostic output
	g_logTimer->Sleep(100);

	//Clear screen and move cursor to X0Y0
	g_uart->Printf("\x1b[2J\x1b[0;0H");

	//Start the logger
	g_log.Initialize(g_uart, &logtim);
	g_log("UART logging ready\n");
}

void DetectHardware()
{
	g_log("Identifying hardware\n");
	LogIndenter li(g_log);

	uint16_t rev = DBGMCU.IDCODE >> 16;
	uint16_t device = DBGMCU.IDCODE & 0xfff;

	//Look up the stepping number
	const char* srev = nullptr;
	switch(rev)
	{
		case 0x1000:
			srev = "A";
			break;

		case 0x1001:
			srev = "Z";
			break;

		case 0x2001:
			srev = "Y";
			break;

		default:
			srev = "(unknown)";
	}

	const char* part = nullptr;
	switch(device)
	{
		case 0x435:
			part = "L43xxx/44xxx";
			break;

		case 0x462:
			part = "L45xxx/46xxx";
			break;

		case 0x464:
			part = "L41xxx/42xxx";
			break;

		default:
			srev = "(unknown)";
			break;
	}

	g_log("STM32%s stepping %s\n", part, srev);
	g_log("64 kB total SRAM, 1 kB EEPROM, 128 byte backup SRAM\n");
	g_log("%d kB Flash\n", FLASH_SIZE);

	uint16_t waferX = U_ID[0] >> 16;
	uint16_t waferY = U_ID[0] & 0xffff;
	uint8_t waferNum = U_ID[1] & 0xff;
	char waferLot[8] =
	{
		static_cast<char>((U_ID[2] >> 24) & 0xff),
		static_cast<char>((U_ID[2] >> 16) & 0xff),
		static_cast<char>((U_ID[2] >> 8) & 0xff),
		static_cast<char>((U_ID[2] >> 0) & 0xff),
		static_cast<char>((U_ID[1] >> 24) & 0xff),
		static_cast<char>((U_ID[1] >> 16) & 0xff),
		static_cast<char>((U_ID[1] >> 8) & 0xff),
		'\0'
	};
	g_log("Lot %s, wafer %d, die (%d, %d)\n", waferLot, waferNum, waferX, waferY);
}

void InitGPIOs()
{
	g_log("Initializing GPIOs\n");

	//DEBUG: turn on all port direction LEDs
	static GPIOPin dirin_led0(&GPIOD, 10, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
	static GPIOPin dirin_led1(&GPIOC, 9, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
	static GPIOPin dirin_led2(&GPIOA, 11, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
	static GPIOPin dirin_led3(&GPIOA, 12, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);

	static GPIOPin dirout_led0(&GPIOE, 7, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
	static GPIOPin dirout_led1(&GPIOE, 9, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
	static GPIOPin dirout_led2(&GPIOE, 11, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
	static GPIOPin dirout_led3(&GPIOE, 14, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);

	dirin_led0 = 1;
	dirin_led1 = 1;
	dirin_led2 = 1;
	dirin_led3 = 1;
	dirout_led0 = 1;
	dirout_led1 = 1;
	dirout_led2 = 1;
	dirout_led3 = 1;
}

void InitI2C()
{
	g_log("Initializing I2C interface\n");

	static GPIOPin i2c_scl(&GPIOB, 13, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_SLOW, 4, true);
	static GPIOPin i2c_sda(&GPIOB, 11, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_SLOW, 4, true);
	//PB12 is used as a route-through for SDA, so just leave it in the default floating state

	//I2C2 runs off our APB1 clock (40 MHz)
	//Prescale by 4 to get 10 MHz
	//Divide by 100 after that to get 100 kHz
	static I2C i2c(&I2C2, 4, 100);
	g_i2c = &i2c;
}

void InitExpander()
{
	g_log("Initializing IO expander\n");

	//Clear reset
	static GPIOPin tca_rst(&GPIOB, 1, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
	tca_rst = 1;
	g_logTimer->Sleep(20);

	//Initialize the expander
	static TCA6424A expander(g_i2c, 0x44);
	g_expander = &expander;

	//Set all the IOs as output
	for(int i=0; i<24; i++)
	{
		expander.SetDirection(i, false);

		//DEBUG: also turn all the LEDs on
		expander.SetOutputValue(i, true);
	}
}

void InitSPI()
{
	g_log("Initializing SPI\n");

	//Set up GPIOs for display bus
	static GPIOPin display_sck(&GPIOD, 1, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_FAST, 5);
	//PD0 used as route through, leave tristated
	static GPIOPin display_mosi(&GPIOD, 4, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_FAST, 5);
	//MISO not used, bus is bidirectional

	//Divide by 20 to get 2 MHz SPI
	//We can run up to 10 MHz for writes but readback Fmax is much slower
	static SPI displaySPI(&SPI2, true, 20);
	g_displaySPI = &displaySPI;
}

void InitDisplay()
{
	g_log("Initializing display\n");
	LogIndenter li(g_log);

	//Set up GPIOs
	static GPIOPin display_busy_n(&GPIOA, 4, GPIOPin::MODE_INPUT, GPIOPin::SLEW_SLOW);
	static GPIOPin display_bs(&GPIOA, 7, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
	static GPIOPin display_cs_n(&GPIOC, 14, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_FAST);
	static GPIOPin display_dc(&GPIOH, 0, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_FAST);
	static GPIOPin display_rst_n(&GPIOH, 1, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_FAST);

	//BS pin seems to be a strap that just needs to be held low
	display_bs = 0;

	//Set up the display itself
	static Display display(g_displaySPI, &display_busy_n, &display_cs_n, &display_dc, &display_rst_n);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sensor interfacing

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

void InitSensors()
{
	g_log("Initializing sensors\n");
	LogIndenter li(g_log);

	//Set temperature sensor to max resolution
	uint8_t cmd[3] = {0x01, 0x60, 0x00};
	if(!g_i2c->BlockingWrite(g_tempI2cAddress, cmd, sizeof(cmd)))
		g_log(Logger::ERROR, "Failed to initialize I2C temp sensor at 0x%02x\n", g_tempI2cAddress);

	//Print initial values from every sensor
	g_log("Board temperature: %uhk C\n", ReadThermalSensor(g_tempI2cAddress));
}

