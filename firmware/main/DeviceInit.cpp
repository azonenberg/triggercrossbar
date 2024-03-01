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
	@brief	Initialization code only used by real hardware
 */

#include "triggercrossbar.h"
#include <ctype.h>
#include "DeviceFPGAInterface.h"
#include "QSPIEthernetInterface.h"

void TrimSpaces(char* str);

void InitClocks()
{
	//Configure the flash with wait states and prefetching before making any changes to the clock setup.
	//A bit of extra latency is fine, the CPU being faster than flash is not.
	Flash::SetConfiguration(275, RANGE_VOS0);

	//By default out of reset, we're clocked by the HSI clock at 64 MHz

	//Set up PLL1
	RCCHelper::InitializePLL(
		1,		//PLL1
		64,		//input is 64 MHz from the HSE
		4,		//64/4 = 16 MHz at the PFD
		32,		//16 * 32 = 512 MHz at the VCO
		1,		//div P (primary output 512 MHz)
		8,		//div Q (64 MHz kernel clock)
		32,		//div R (not used for now),
		RCCHelper::CLOCK_SOURCE_HSI
	);

	//Set up main system clock tree
	RCCHelper::InitializeSystemClocks(
		1,		//sysclk = 512 MHz
		2,		//AHB = 256 MHz
		4,		//APB1 = 64 MHz
		4,		//APB2 = 64 MHz
		4,		//APB3 = 64 MHz
		4		//APB4 = 64 MHz
	);

	//RNG clock should be >= HCLK/32
	//AHB2 HCLK is 256 MHz so min 8 MHz
	//Select PLL1 Q clock (64 MHz)
	RCC.D2CCIP2R = (RCC.D2CCIP2R & ~0x300) | (0x100);

	//Select PLL1 as system clock source
	RCCHelper::SelectSystemClockFromPLL1();
}

void InitTimer()
{
	//APB1 is 64 MHz
	//Divide down to get 10 kHz ticks
	static Timer logtim(&TIM2, Timer::FEATURE_GENERAL_PURPOSE, 6400);
	g_logTimer = &logtim;
}

void InitUART()
{
	//Initialize the UART for local console: 115.2 Kbps using PA12 for UART4 transmit and PA11 for UART2 receive
	//TODO: nice interface for enabling UART interrupts
	GPIOPin uart_tx(&GPIOA, 12, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_SLOW, 6);
	GPIOPin uart_rx(&GPIOA, 11, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_SLOW, 6);

	//Default after reset is for UART4 to be clocked by PCLK1 (APB1 clock) which is 64 MHz
	//So we need a divisor of 555.55
	static UART uart(&UART4, 556);
	g_cliUART = &uart;

	//Enable the UART RX interrupt
	//TODO: Make an RCC method for this
	volatile uint32_t* NVIC_ISER1 = (volatile uint32_t*)(0xe000e104);
	*NVIC_ISER1 = 0x100000;

	g_logTimer->Sleep(10);	//wait for UART pins to be high long enough to remove any glitches during powerup

	//Clear screen and move cursor to X0Y0
	uart.Printf("\x1b[2J\x1b[0;0H");
}

void DetectHardware()
{
	g_log("Identifying hardware\n");
	LogIndenter li(g_log);

	uint16_t rev = DBGMCU.IDCODE >> 16;
	uint16_t device = DBGMCU.IDCODE & 0xfff;

	if(device == 0x483)
	{
		//Look up the stepping number
		const char* srev = NULL;
		switch(rev)
		{
			case 0x1000:
				srev = "A";
				break;

			case 0x1001:
				srev = "Z";
				break;

			default:
				srev = "(unknown)";
		}

		uint8_t pkg = SYSCFG.PKGR;
		const char* package = "";
		switch(pkg)
		{
			case 0:
				package = "VQFPN68 (industrial)";
				break;
			case 1:
				package = "LQFP100/TFBGA100 (legacy)";
				break;
			case 2:
				package = "LQFP100 (industrial)";
				break;
			case 3:
				package = "TFBGA100 (industrial)";
				break;
			case 4:
				package = "WLCSP115 (industrial)";
				break;
			case 5:
				package = "LQFP144 (legacy)";
				break;
			case 6:
				package = "UFBGA144 (legacy)";
				break;
			case 7:
				package = "LQFP144 (industrial)";
				break;
			case 8:
				package = "UFBGA169 (industrial)";
				break;
			case 9:
				package = "UFBGA176+25 (industrial)";
				break;
			case 10:
				package = "LQFP176 (industrial)";
				break;
			default:
				package = "unknown package";
				break;
		}

		g_log("STM32%c%c%c%c stepping %s, %s\n",
			(L_ID >> 24) & 0xff,
			(L_ID >> 16) & 0xff,
			(L_ID >> 8) & 0xff,
			(L_ID >> 0) & 0xff,
			srev,
			package
			);
		g_log("564 kB total SRAM, 128 kB DTCM, up to 256 kB ITCM, 4 kB backup SRAM\n");
		g_log("%d kB Flash\n", F_ID);

		//U_ID fields documented in 45.1 of STM32 programming manual
		uint16_t waferX = U_ID[0] >> 16;
		uint16_t waferY = U_ID[0] & 0xffff;
		uint8_t waferNum = U_ID[1] & 0xff;
		char waferLot[8] =
		{
			static_cast<char>((U_ID[1] >> 24) & 0xff),
			static_cast<char>((U_ID[1] >> 16) & 0xff),
			static_cast<char>((U_ID[1] >> 8) & 0xff),
			static_cast<char>((U_ID[2] >> 24) & 0xff),
			static_cast<char>((U_ID[2] >> 16) & 0xff),
			static_cast<char>((U_ID[2] >> 8) & 0xff),
			static_cast<char>((U_ID[2] >> 0) & 0xff),
			'\0'
		};
		g_log("Lot %s, wafer %d, die (%d, %d)\n", waferLot, waferNum, waferX, waferY);
	}
	else
		g_log(Logger::WARNING, "Unknown device (0x%06x)\n", device);
}

/**
	@brief Initialize global GPIO LEDs
 */
void InitLEDs()
{
	//Set up the GPIO LEDs and turn them off
	static GPIOPin led0(&GPIOD, 12, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
	static GPIOPin led1(&GPIOD, 13, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
	static GPIOPin led2(&GPIOG, 2, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
	static GPIOPin led3(&GPIOG, 5, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);

	led0 = 0;
	led1 = 0;
	led2 = 0;
	led3 = 0;

	g_leds[0] = &led0;
	g_leds[1] = &led1;
	g_leds[2] = &led2;
	g_leds[3] = &led3;
}

/**
	@brief Initialze the DACs used for the IO subsystem
 */
void InitDACs()
{
	g_log("Initializing DACs\n");
	LogIndenter li(g_log);

	//Set up the GPIOs for chip selects and deselect everything
	static GPIOPin tx_dac_cs_n(&GPIOE, 10, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_FAST);
	static GPIOPin rx_dac0_cs_n(&GPIOA, 3, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_FAST);
	static GPIOPin rx_dac1_cs_n(&GPIOA, 4, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_FAST);
	tx_dac_cs_n = 1;
	rx_dac0_cs_n = 1;
	rx_dac1_cs_n = 1;

	g_leds[0]->Set(1);

	//Initialize the peripherals
	static GPIOPin dac_spi_sck(&GPIOA, 5, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_FAST, 5);
	static GPIOPin dac_spi_miso(&GPIOA, 6, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_FAST, 5);
	static GPIOPin dac_spi_mosi(&GPIOA, 7, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_FAST, 5);

	//SPI1 runs on spi 1/2/3 kernel clock domain
	//default after reset is PLL1 Q which is 64 MHz, divide by 8 to get 8 MHz
	//DACx0508 is weird and wants to work on falling edge of clock
	static SPI dac_spi(&SPI1, true, 8);
	dac_spi.SetClockInvert(true);

	//Wait a while to make sure everything is deselected
	g_logTimer->Sleep(5);

	g_leds[1]->Set(1);

	//Initialize the DACs
	static OctalDAC tx_dac(dac_spi, tx_dac_cs_n);
	static OctalDAC rx_dac0(dac_spi, rx_dac0_cs_n);
	static OctalDAC rx_dac1(dac_spi, rx_dac1_cs_n);

	//Set all output channels to 2.5V
	for(int i=0; i<8; i++)
		tx_dac.SetChannelMillivolts(i, 2500);

	//Set channel 7 (out4) to 3.3V
	tx_dac.SetChannelMillivolts(7, 3300);

	//Set all input channels to 500 mV threshold
	for(int i=0; i<8; i++)
	{
		rx_dac0.SetChannelMillivolts(i, 850);
		rx_dac1.SetChannelMillivolts(i, 850);
	}

	g_log("DAC init complete\n");
}

void InitQSPI()
{
	g_log("Initializing QSPI interface\n");

	//Configure the I/O manager
	OctoSPIManager::ConfigureMux(false);
	OctoSPIManager::ConfigurePort(
		1,							//Configuring port 1
		false,						//DQ[7:4] disabled
		OctoSPIManager::C1_HIGH,
		true,						//DQ[3:0] enabled
		OctoSPIManager::C1_LOW,		//DQ[3:0] from OCTOSPI1 DQ[3:0]
		true,						//CS# enabled
		OctoSPIManager::PORT_1,		//CS# from OCTOSPI1
		false,						//DQS disabled
		OctoSPIManager::PORT_1,
		true,						//Clock enabled
		OctoSPIManager::PORT_1);	//Clock from OCTOSPI1

	//Configure the I/O pins
	static GPIOPin qspi_cs_n(&GPIOE, 11, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_VERYFAST, 11);
	static GPIOPin qspi_sck(&GPIOB, 2, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_VERYFAST, 9);
	static GPIOPin qspi_dq0(&GPIOA, 2, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_VERYFAST, 6);
	static GPIOPin qspi_dq1(&GPIOB, 0, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_VERYFAST, 4);
	static GPIOPin qspi_dq2(&GPIOC, 2, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_VERYFAST, 9);
	static GPIOPin qspi_dq3(&GPIOA, 1, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_VERYFAST, 9);

	//Clock divider value
	//Default is for AHB3 bus clock to be used as kernel clock (256 MHz for us)
	//With 3.3V Vdd, we can go up to 140 MHz.
	//Dividing by 4 gives 64 MHz and a transfer rate of 256 Mbps.
	//FPGA currently requires <= 62.5 MHz due to the RX oversampling used (4x in 250 MHz clock domain)
	//Dividing by 5 gives 51.2 MHz and a transfer rate of 204.8 Mbps
	uint8_t prescale = 5;

	//Configure the OCTOSPI itself
	static OctoSPI qspi(&OCTOSPI1, 0x02000000, prescale);
	qspi.SetDoubleRateMode(false);
	qspi.SetInstructionMode(OctoSPI::MODE_QUAD, 2);
	qspi.SetAddressMode(OctoSPI::MODE_NONE);
	qspi.SetAltBytesMode(OctoSPI::MODE_NONE);
	qspi.SetDataMode(OctoSPI::MODE_QUAD);
	qspi.SetDummyCycleCount(1);
	qspi.SetDQSEnable(false);
	qspi.SetDeselectTime(1);
	qspi.SetSampleDelay(false);

	g_qspi = &qspi;

	static DeviceFPGAInterface fpga;
	g_fpga = &fpga;
}

void InitI2C()
{
	g_log("Initializing I2C interfaces\n");

	static GPIOPin mac_i2c_scl(&GPIOB, 8, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_SLOW, 6, true);
	static GPIOPin mac_i2c_sda(&GPIOB, 9, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_SLOW, 6, true);

	//Default kernel clock for I2C4 is pclk4 (64 MHz for our current config)
	//Prescale by 16 to get 4 MHz
	//Divide by 40 after that to get 100 kHz
	static I2C mac_i2c(&I2C4, 16, 40);
	g_macI2C = &mac_i2c;
	/*
	static GPIOPin temp_i2c_scl(&GPIOB, 6, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_SLOW, 4, true);
	static GPIOPin temp_i2c_sda(&GPIOB, 7, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_SLOW, 4, true);

	//Default kernel clock for I2C1 is APB1 clock (64 MHz for our current config)
	//Prescale by 16 to get 4 MHz
	//Divide by 40 after that to get 100 kHz
	static I2C temp_i2c(&I2C1, 16, 40);
	g_tempI2C = &temp_i2c;
	*/
	static GPIOPin sfp_i2c_scl(&GPIOF, 1, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_SLOW, 4, true);
	static GPIOPin sfp_i2c_sda(&GPIOF, 0, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_SLOW, 4, true);

	//Default kernel clock for I2C2 is APB1 clock (64 MHz for our current config)
	//Prescale by 16 to get 4 MHz
	//Divide by 40 after that to get 100 kHz
	static I2C sfp_i2c(&I2C2, 16, 40);
	g_sfpI2C = &sfp_i2c;
}

void InitEEPROM()
{
	g_log("Initializing MAC address EEPROM\n");

	//Extended memory block for MAC address data isn't in the normal 0xa* memory address space
	//uint8_t main_addr = 0xa0;
	uint8_t ext_addr = 0xb0;

	//Pointers within extended memory block
	uint8_t serial_offset = 0x80;
	uint8_t mac_offset = 0x9a;

	//Read MAC address
	g_macI2C->BlockingWrite8(ext_addr, mac_offset);
	g_macI2C->BlockingRead(ext_addr, &g_macAddress[0], sizeof(g_macAddress));

	//Read serial number
	const int serial_len = 16;
	uint8_t serial[serial_len] = {0};
	g_macI2C->BlockingWrite8(ext_addr, serial_offset);
	g_macI2C->BlockingRead(ext_addr, serial, serial_len);

	{
		LogIndenter li(g_log);
		g_log("MAC address: %02x:%02x:%02x:%02x:%02x:%02x\n",
			g_macAddress[0], g_macAddress[1], g_macAddress[2], g_macAddress[3], g_macAddress[4], g_macAddress[5]);

		g_log("EEPROM serial number: %02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n",
			serial[0], serial[1], serial[2], serial[3], serial[4], serial[5], serial[6], serial[7],
			serial[8], serial[9], serial[10], serial[11], serial[12], serial[13], serial[14], serial[15]);
	}
}

/**
	@brief Initialize sensors and log starting values for each
 */
void InitSensors()
{
	g_log("Initializing sensors\n");
	LogIndenter li(g_log);

	//Read fans
	for(uint8_t i=0; i<2; i++)
	{
		auto rpm = GetFanRPM(i);
		if(rpm == 0)
			g_log(Logger::ERROR, "Fan %d:                                 STOPPED\n", i, rpm);
		else
			g_log("Fan %d:                                 %d RPM\n", i, rpm);
	}

	//Read FPGA temperature
	auto temp = GetFPGATemperature();
	g_log("FPGA die temperature:                  %uhk C\n", temp);

	//Read FPGA voltage sensors
	int volt = GetFPGAVCCINT();
	g_log("FPGA VCCINT:                           %uhk V\n", volt);
	volt = GetFPGAVCCBRAM();
	g_log("FPGA VCCBRAM:                          %uhk V\n", volt);
	volt = GetFPGAVCCAUX();
	g_log("FPGA VCCAUX:                           %uhk V\n", volt);

	InitDTS();
}

/**
	@brief Initialize the I2C EEPROM and GPIOs for the SFP+ interface
 */
void InitSFP()
{
	g_log("Initializing SFP+\n");

	//Set up the IOs
	static GPIOPin modAbs(&GPIOC, 13, GPIOPin::MODE_INPUT, GPIOPin::SLEW_SLOW);
	static GPIOPin txDisable(&GPIOC, 14, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
	static GPIOPin txFault(&GPIOC, 15, GPIOPin::MODE_INPUT, GPIOPin::SLEW_SLOW);
	modAbs.SetPullMode(GPIOPin::PULL_UP);
	txFault.SetPullMode(GPIOPin::PULL_UP);
	g_sfpModAbsPin = &modAbs;
	g_sfpTxDisablePin = &txDisable;
	g_sfpTxFaultPin = &txFault;

	//No GPIOs for rate select, they're hardwired high

	//Default to disabling transmit
	txDisable.Set(true);

	//note that there may not be an optic installed yet, so don't initialize unless we know it's there
	PollSFP();
}
/**
	@brief Check if a SFP+ optic has been inserted or removed
 */
void PollSFP()
{
	//Detect transmitter faults
	if(g_sfpTxFaultPin->Get())
	{
		if(!g_sfpFaulted)
		{
			g_log(Logger::ERROR, "SFP+ laser fault detected\n");
			g_sfpFaulted = true;
			g_sfpTxDisablePin->Set(true);
		}
		return;
	}

	//Fault cleared?
	else if(g_sfpFaulted)
	{
		g_log("SFP+ laser fault cleared\n");
		g_sfpFaulted = false;
		g_sfpTxDisablePin->Set(false);
		return;
	}

	//Optic not present?
	if(g_sfpModAbsPin->Get())
	{
		if(g_sfpPresent)
		{
			g_log("SFP+ optic removed from port xg0\n");
			g_sfpTxDisablePin->Set(true);
			g_sfpPresent = false;
		}
		return;
	}

	//Optic present
	//Did we already know it was here? If so, nothing's changed
	else if(g_sfpPresent)
		return;

	//Nope, optic was just inserted
	g_log("SFP+ optic inserted in port xg0\n");
	g_sfpPresent = true;
	LogIndenter li(g_log);

	//Turn on transmitter
	g_sfpTxDisablePin->Set(false);

	//Wait 300 ms (t_start_up) to make sure it's up
	//TODO: we don't want to block incoming network frame handling during this time!
	//This should be nonblocking
	g_logTimer->Sleep(3000);

	//Read the base EEPROM page from the optic
	uint8_t basePage[128];
	g_sfpI2C->BlockingWrite8(0xa0, 0x00);
	if(!g_sfpI2C->BlockingRead(0xa0, basePage, sizeof(basePage)))
	{
		g_log(Logger::ERROR, "Failed to read base EEPROM page\n");
		return;
	}

	//Print out some minimal information, not full details
	const char* connectorType = "unknown";
	if(basePage[2] == 0x07)
		connectorType = "LC";

	//can have multiple bits set, but for now we only report the highest
	const char* protocol = "unknown";
	if(basePage[3] & 0x80)
		protocol = "10Gbase-ER";
	else if(basePage[3] & 0x40)
		protocol = "10Gbase-LRM";
	else if(basePage[3] & 0x20)
		protocol = "10Gbase-LR";
	else if(basePage[3] & 0x10)
		protocol = "10Gbase-SR";
	else if(basePage[6] & 0x08)
		protocol = "1000base-T";
	else if(basePage[6] & 0x04)
		protocol = "1000base-CX";
	else if(basePage[6] & 0x02)
		protocol = "1000base-LX";
	else if(basePage[6] & 0x01)
		protocol = "1000base-SX";

	g_log("%s optic with %s connector\n", protocol, connectorType);

	char vendorName[17] = {0};
	memcpy(vendorName, basePage + 20, 16);
	TrimSpaces(vendorName);

	char vendorPN[17] = {0};
	memcpy(vendorPN, basePage + 40, 16);
	TrimSpaces(vendorPN);

	char vendorRev[5] = {0};
	memcpy(vendorRev, basePage + 56, 4);
	TrimSpaces(vendorRev);

	char serial[17] = {0};
	memcpy(serial, basePage + 68, 16);
	TrimSpaces(serial);

	char date[9] = {0};
	memcpy(date, basePage + 84, 8);
	TrimSpaces(date);

	g_log("Vendor %s, part %s, rev %s, serial %s, date code %s\n",
		vendorName,
		vendorPN,
		vendorRev[0] ? vendorRev : "(empty)",
		serial,
		date);

	if(basePage[92] & 0x40)
	{
		g_log("Digital diagnostic monitoring available\n");

		bool internalCal = false;
		bool externalCal = false;

		if(basePage[92] & 0x4)
			g_log("Address change sequence required\n");

		if(basePage[92] & 0x20)
		{
			internalCal = true;
			g_log("Internally calibrated\n");
		}
		if(basePage[92] & 0x10)
		{
			externalCal = true;
			g_log("Externally calibrated (not supported)\n");
		}

		//Get temperature
		uint16_t temp = GetSFPTemperature();
		g_log("Temperature:    %2d.%02d C\n", (temp >> 8), static_cast<int>(((temp & 0xff) / 256.0) * 100));

		//Get supply voltage
		g_sfpI2C->BlockingWrite8(0xa2, 98);
		uint16_t volt = 0;
		g_sfpI2C->BlockingRead16(0xa2, volt);
		int voltScaled = volt;
		g_log("Supply voltage: %2d.%04d V\n", (voltScaled / 10000), voltScaled % 10000);

		//Get TX bias current
		g_sfpI2C->BlockingWrite8(0xa2, 100);
		uint16_t bias = 0;
		g_sfpI2C->BlockingRead16(0xa2, bias);
		int biasScaled = bias * 2;		//2 μA per LSB
		g_log("TX bias:        %2d.%3d mA\n", biasScaled / 1000, biasScaled % 1000);

		//Get TX power
		g_sfpI2C->BlockingWrite8(0xa2, 102);
		uint16_t txpower = 0;
		g_sfpI2C->BlockingRead16(0xa2, txpower);
		int txPowerScaled = txpower;	//0.1 μW per LSB
		g_log("TX power:      %3d.%d μW\n", txPowerScaled / 10, txPowerScaled % 10);

		//Get RX power
		g_sfpI2C->BlockingWrite8(0xa2, 104);
		uint16_t rxpower = 0;
		g_sfpI2C->BlockingRead16(0xa2, rxpower);
		int rxPowerScaled = rxpower;	//0.1 μW per LSB
		g_log("RX power:      %3d.%d μW\n", rxPowerScaled / 10, rxPowerScaled % 10);
	}
}

/**
	@brief Remove spaces from trailing edge of a string
 */
void TrimSpaces(char* str)
{
	char* p = str + strlen(str) - 1;

	while(p >= str)
	{
		if(isspace(*p))
			*p = '\0';
		else
			break;

		p --;
	}
}

/**
	@brief Initializes the management PHY
 */
void InitManagementPHY()
{
	g_log("Initializing management PHY\n");
	LogIndenter li(g_log);

	//Read the PHY ID
	auto phyid1 = ManagementPHYRead(REG_PHY_ID_1);
	auto phyid2 = ManagementPHYRead(REG_PHY_ID_2);

	if( (phyid1 == 0x22) && ( (phyid2 >> 4) == 0x162))
	{
		g_log("PHY ID   = %04x %04x (KSZ9031RNX rev %d)\n", phyid1, phyid2, phyid2 & 0xf);

		//Adjust pad skew for RX_CLK register to improve timing FPGA side
		//ManagementPHYExtendedWrite(2, REG_KSZ9031_MMD2_CLKSKEW, 0x01ef);
	}
	else
		g_log("PHY ID   = %04x %04x (unknown)\n", phyid1, phyid2);
}

/**
	@brief Initializes the Ethernet interface
 */
void InitEthernet()
{
	g_log("Initializing Ethernet management\n");

	//Create the Ethernet interface
	static QSPIEthernetInterface iface;
	g_ethIface = &iface;
}

/**
	@brief Initialize the digital temperature sensor
 */
void InitDTS()
{
	//APB4 clock is 64 MHz, so divide by 80 to get 800 kHz ticks
	//(must be <1 MHz)
	//15 cycles integration time = 18.75 us
	static DigitalTempSensor dts(&DTS, 80, 15, 64000000);
	g_dts = &dts;

	auto tempval = dts.GetTemperature();
	g_log("MCU die temperature:                   %d.%02d C\n",
		(tempval >> 8),
		static_cast<int>(((tempval & 0xff) / 256.0) * 100));
}
