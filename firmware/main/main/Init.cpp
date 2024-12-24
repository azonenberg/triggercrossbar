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
	@brief	System initialization
 */

#include "triggercrossbar.h"
#include <ctype.h>
#include <supervisor/SupervisorSPIRegisters.h>

/**
	@brief Set all bidir ports to input so we know what state they're in
 */
void InitRelays()
{
	g_log("Setting all relays to input mode\n");
	for(int chan=0; chan<4; chan++)
	{
		g_relayController->toggle = 0x8000 | chan;

		//Ping-pong poll the two status registers until we're done
		while( (0 != g_relayController->stat) && (0 != g_relayController->stat2) )
		{}
	}
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
	@brief Initialize the DACs used for the IO subsystem
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

	//Initialize the peripherals
	static GPIOPin dac_spi_sck(&GPIOA, 5, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_FAST, 5);
	static GPIOPin dac_spi_miso(&GPIOA, 6, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_FAST, 5);
	static GPIOPin dac_spi_mosi(&GPIOA, 7, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_FAST, 5);

	//SPI1 runs on spi 1/2/3 kernel clock domain
	//default after reset is PLL1 Q which is 68.75 MHz, divide by 8 to get 8.59 MHz
	//DACx0508 is weird and wants to work on falling edge of clock
	static SPI dac_spi(&SPI1, true, 8);
	dac_spi.SetClockInvert(true);

	//Wait a while to make sure everything is deselected
	g_logTimer.Sleep(5);

	//Initialize the DACs
	static OctalDAC tx_dac(dac_spi, tx_dac_cs_n);
	static OctalDAC rx_dac0(dac_spi, rx_dac0_cs_n);
	static OctalDAC rx_dac1(dac_spi, rx_dac1_cs_n);
	g_rxDacs[0] = &rx_dac0;
	g_rxDacs[1] = &rx_dac1;
	g_txDac = &tx_dac;

	//Set all output channels to 2.0V
	for(int i=0; i<8; i++)
		tx_dac.SetChannelMillivolts(i, 2000);

	//Set CDR trigger to 5 mV threshold
	rx_dac1.SetChannelMillivolts(7, 5);

	//Set all input channels to 850 mV threshold
	for(int i=0; i<8; i++)
	{
		rx_dac0.SetChannelMillivolts(i, 850);
		rx_dac1.SetChannelMillivolts(i, 850);
	}

	g_log("DAC init complete\n");
}

/**
	@brief Initialize the SPI bus to the supervisor
 */
void InitSupervisor()
{
	g_log("Initializing supervisor\n");
	LogIndenter li(g_log);

	//Set up the GPIOs for chip selects and deselect everything
	auto slew = GPIOPin::SLEW_MEDIUM;
	static GPIOPin super_cs_n(&GPIOE, 4, GPIOPin::MODE_OUTPUT, slew);
	super_cs_n = 1;
	g_superSPICS = &super_cs_n;
	g_logTimer.Sleep(1);

	//Initialize the rest of our IOs
	static GPIOPin spi_sck(&GPIOE, 2, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_MEDIUM, 5);
	static GPIOPin spi_miso(&GPIOE, 5, GPIOPin::MODE_PERIPHERAL, slew, 5);
	static GPIOPin spi_mosi(&GPIOE, 6, GPIOPin::MODE_PERIPHERAL, slew, 5);

	//Get the supervisor firmware version
	super_cs_n = 0;
	g_superSPI.BlockingWrite(SUPER_REG_VERSION);
	g_superSPI.WaitForWrites();
	g_superSPI.DiscardRxData();
	g_logTimer.Sleep(5);
	g_superSPI.BlockingRead();	//discard dummy byte
	for(size_t i=0; i<sizeof(g_superVersion); i++)
		g_superVersion[i] = g_superSPI.BlockingRead();
	g_superVersion[sizeof(g_superVersion)-1] = '\0';
	super_cs_n = 1;
	g_log("Firmware version:     %s\n", g_superVersion);

	//Get IBC firmware version
	super_cs_n = 0;
	g_superSPI.BlockingWrite(SUPER_REG_IBCVERSION);
	g_superSPI.WaitForWrites();
	g_superSPI.DiscardRxData();
	g_logTimer.Sleep(5);
	g_superSPI.BlockingRead();	//discard dummy byte
	for(size_t i=0; i<sizeof(g_ibcVersion); i++)
		g_ibcVersion[i] = g_superSPI.BlockingRead();
	g_ibcVersion[sizeof(g_ibcVersion)-1] = '\0';
	super_cs_n = 1;
	g_log("IBC firmware version: %s\n", g_ibcVersion);

	//TODO: Print supervisor sensors
}

/**
	@brief Initialize sensors and log starting values for each
 */
void InitSensors()
{
	g_log("Initializing sensors\n");
	LogIndenter li(g_log);

	//Wait 50ms to get accurate readings
	g_logTimer.Sleep(500);

	//Read fans
	for(uint8_t i=0; i<2; i++)
	{
		auto rpm = GetFanRPM(i);
		if(rpm == 0)
			g_log(Logger::ERROR, "Fan %d:                                 STOPPED\n", i, rpm);
		else
			g_log("Fan %d:                                 %d RPM\n", i, rpm);

		//skip reading fan1 as we don't have it connected
		break;
	}

	//Read FPGA temperature
	auto temp = GetFPGATemperature();
	g_log("FPGA die temperature:                  %uhk C\n", temp);

	//Read FPGA voltage sensors
	int volt = GetFPGAVCCINT();
	g_log("FPGA VCCINT:                            %uhk V\n", volt);
	volt = GetFPGAVCCBRAM();
	g_log("FPGA VCCBRAM:                           %uhk V\n", volt);
	volt = GetFPGAVCCAUX();
	g_log("FPGA VCCAUX:                            %uhk V\n", volt);

	InitDTS();
}

void RegisterProtocolHandlers(IPv4Protocol& ipv4)
{
	static ManagementTCPProtocol tcp(&ipv4);
	static ManagementUDPProtocol udp(&ipv4);
	ipv4.UseTCP(&tcp);
	ipv4.UseUDP(&udp);
	g_dhcpClient = &udp.GetDHCP();
	g_ntpClient = &udp.GetNTP();
}
