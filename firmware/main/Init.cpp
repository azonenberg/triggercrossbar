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
#include "../super/superregs.h"

void TrimSpaces(char* str);

/**
	@brief Bring up the control interface to the FPGA
 */
void InitFPGA()
{
	g_log("Initializing FPGA\n");
	LogIndenter li(g_log);

	//Wait for the DONE signal to go high
	g_log("Waiting for FPGA boot\n");
	static GPIOPin fpgaDone(&GPIOF, 6, GPIOPin::MODE_INPUT, GPIOPin::SLEW_SLOW);
	while(!fpgaDone)
	{}

	//Read the FPGA IDCODE and serial number
	//Retry until we get a nonzero result indicating FPGA is up
	while(true)
	{
		uint32_t idcode = g_sysInfo->idcode;
		memcpy(g_fpgaSerial, (const void*)g_sysInfo->serial, 8);

		//If IDCODE is all zeroes, poll again
		if(idcode == 0)
			continue;

		//Print status
		switch(idcode & 0x0fffffff)
		{
			case 0x3647093:
				g_log("IDCODE: %08x (XC7K70T rev %d)\n", idcode, idcode >> 28);
				break;

			case 0x364c093:
				g_log("IDCODE: %08x (XC7K160T rev %d)\n", idcode, idcode >> 28);
				break;

			default:
				g_log("IDCODE: %08x (unknown device, rev %d)\n", idcode, idcode >> 28);
				break;
		}
		g_log("Serial: %02x%02x%02x%02x%02x%02x%02x%02x\n",
			g_fpgaSerial[7], g_fpgaSerial[6], g_fpgaSerial[5], g_fpgaSerial[4],
			g_fpgaSerial[3], g_fpgaSerial[2], g_fpgaSerial[1], g_fpgaSerial[0]);

		break;
	}

	//Read USERCODE
	g_usercode = g_sysInfo->usercode;
	g_log("Usercode: %08x\n", g_usercode);
	{
		LogIndenter li(g_log);

		//Format per XAPP1232:
		//31:27 day
		//26:23 month
		//22:17 year
		//16:12 hr
		//11:6 min
		//5:0 sec
		int day = g_usercode >> 27;
		int mon = (g_usercode >> 23) & 0xf;
		int yr = 2000 + ((g_usercode >> 17) & 0x3f);
		int hr = (g_usercode >> 12) & 0x1f;
		int min = (g_usercode >> 6) & 0x3f;
		int sec = g_usercode & 0x3f;
		g_log("Bitstream timestamp: %04d-%02d-%02d %02d:%02d:%02d\n",
			yr, mon, day, hr, min, sec);
	}

	//Set all bidir ports to input so we know what state they're in
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
	@brief Set our IP address and initialize the IP stack
 */
void InitIP()
{
	g_log("Initializing management IPv4 interface\n");
	LogIndenter li(g_log);

	ConfigureIP();

	g_log("Our IP address is %d.%d.%d.%d\n",
		g_ipConfig.m_address.m_octets[0],
		g_ipConfig.m_address.m_octets[1],
		g_ipConfig.m_address.m_octets[2],
		g_ipConfig.m_address.m_octets[3]);

	//ARP cache (shared by all interfaces)
	static ARPCache cache;

	//Per-interface protocol stacks
	static EthernetProtocol eth(*g_ethIface, g_macAddress);
	g_ethProtocol = &eth;
	static ARPProtocol arp(eth, g_ipConfig.m_address, cache);

	//Global protocol stacks
	static IPv4Protocol ipv4(eth, g_ipConfig, cache);
	static ICMPv4Protocol icmpv4(ipv4);
	static ManagementTCPProtocol tcp(&ipv4);
	static ManagementUDPProtocol udp(&ipv4);

	//Register protocol handlers with the lower layer
	eth.UseARP(&arp);
	eth.UseIPv4(&ipv4);
	ipv4.UseICMPv4(&icmpv4);
	ipv4.UseTCP(&tcp);
	ipv4.UseUDP(&udp);

	//Save a few pointers
	g_dhcpClient = &udp.GetDHCP();
}

/**
	@brief Load our IP configuration from the KVS
 */
void ConfigureIP()
{
	g_ipConfig.m_address = g_kvs->ReadObject<IPv4Address>(g_defaultIP, "ip.address");
	g_ipConfig.m_netmask = g_kvs->ReadObject<IPv4Address>(g_defaultNetmask, "ip.netmask");
	g_ipConfig.m_broadcast = g_kvs->ReadObject<IPv4Address>(g_defaultBroadcast, "ip.broadcast");
	g_ipConfig.m_gateway = g_kvs->ReadObject<IPv4Address>(g_defaultGateway, "ip.gateway");
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
	g_superSPI.BlockingRead();	//discard dummy byte
	for(size_t i=0; i<sizeof(g_superVersion); i++)
		g_superVersion[i] = g_superSPI.BlockingRead();
	g_superVersion[sizeof(g_superVersion)-1] = '\0';
	super_cs_n = 1;
	g_log("Firmware version: %s\n", g_superVersion);

	//Get IBC firmware version
	super_cs_n = 0;
	g_superSPI.BlockingWrite(SUPER_REG_IBCVERSION);
	g_superSPI.WaitForWrites();
	g_superSPI.DiscardRxData();
	g_superSPI.BlockingRead();	//discard dummy byte
	for(size_t i=0; i<sizeof(g_ibcVersion); i++)
		g_ibcVersion[i] = g_superSPI.BlockingRead();
	g_ibcVersion[sizeof(g_ibcVersion)-1] = '\0';
	super_cs_n = 1;
	g_log("IBC firmware version: %s\n", g_ibcVersion);
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
	//Default is for AHB3 bus clock to be used as kernel clock (250 MHz for us)
	//With 3.3V Vdd, we can go up to 140 MHz.
	//FPGA currently requires <= 62.5 MHz due to the RX oversampling used (4x in 250 MHz clock domain)
	//Dividing by 5 gives 50 MHz and a transfer rate of 200 Mbps
	//Dividing by 10, but DDR, gives the same throughput and works around an errata
	uint8_t prescale = 20;

	//Configure the OCTOSPI itself
	//Original code used "instruction", but we want "address" to enable memory mapping
	static OctoSPI qspi(&OCTOSPI1, 0x02000000, prescale);
	qspi.SetDoubleRateMode(false);
	qspi.SetInstructionMode(OctoSPI::MODE_QUAD, 1);
	qspi.SetAddressMode(OctoSPI::MODE_QUAD, 3);
	qspi.SetAltBytesMode(OctoSPI::MODE_NONE);
	qspi.SetDataMode(OctoSPI::MODE_QUAD);
	qspi.SetDummyCycleCount(1);
	qspi.SetDQSEnable(false);
	qspi.SetDeselectTime(1);
	qspi.SetSampleDelay(false, true);
	qspi.SetDoubleRateMode(true);

	//Poke MPU settings to disable caching etc on the QSPI memory range
	MPU::Configure(MPU::KEEP_DEFAULT_MAP, MPU::DISABLE_IN_FAULT_HANDLERS);
	MPU::ConfigureRegion(
		0,
		FPGA_MEM_BASE,
		MPU::SHARED_DEVICE,
		MPU::FULL_ACCESS,
		MPU::EXECUTE_NEVER,
		MPU::SIZE_16M);

	//Configure memory mapping mode
	qspi.SetMemoryMapMode(APBFPGAInterface::OP_APB_READ, APBFPGAInterface::OP_APB_WRITE);
	g_qspi = &qspi;
}

void InitI2C()
{
	g_log("Initializing I2C interfaces\n");

	static GPIOPin mac_i2c_scl(&GPIOB, 8, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_SLOW, 6, true);
	static GPIOPin mac_i2c_sda(&GPIOB, 9, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_SLOW, 6, true);

	//Default kernel clock for I2C4 is pclk4 (68.75 MHz for our current config)
	//Prescale by 16 to get 4.29 MHz
	//Divide by 40 after that to get 107 kHz
	static I2C mac_i2c(&I2C4, 16, 40);
	g_macI2C = &mac_i2c;

	static GPIOPin sfp_i2c_scl(&GPIOF, 1, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_SLOW, 4, true);
	static GPIOPin sfp_i2c_sda(&GPIOF, 0, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_SLOW, 4, true);

	//Default kernel clock for I2C2 is is APB1 clock (68.75 MHz for our current config)
	//Prescale by 16 to get 4.29 MHz
	//Divide by 40 after that to get 107 kHz
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

	//Nope, optic was just inserted
	g_log("SFP+ optic inserted in port xg0\n");
	g_sfpPresent = true;
	LogIndenter li(g_log);

	//Turn on transmitter
	g_sfpTxDisablePin->Set(false);

	//Wait 300 ms (t_start_up) to make sure it's up
	//TODO: we don't want to block incoming network frame handling during this time!
	//This should be nonblocking
	g_logTimer.Sleep(3000);

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

		//bool internalCal = false;
		//bool externalCal = false;

		if(basePage[92] & 0x4)
			g_log("Address change sequence required\n");

		if(basePage[92] & 0x20)
		{
			//internalCal = true;
			g_log("Internally calibrated\n");
		}
		if(basePage[92] & 0x10)
		{
			//externalCal = true;
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
	//APB4 clock is 68.75 MHz, so divide by 80 to get 859 kHz ticks
	//(must be <1 MHz)
	//15 cycles integration time = 18.75 us
	static DigitalTempSensor dts(&DTS, 80, 15, 64000000);
	g_dts = &dts;

	auto tempval = dts.GetTemperature();
	g_log("MCU die temperature:                   %d.%02d C\n",
		(tempval >> 8),
		static_cast<int>(((tempval & 0xff) / 256.0) * 100));
}
