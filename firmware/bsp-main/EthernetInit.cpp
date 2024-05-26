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
	@brief	Common Ethernet initialization used by both bootloader and application
 */
#include <core/platform.h>
#include "hwinit.h"

const IPv4Address g_defaultIP			= { .m_octets{192, 168,   1,   2} };
const IPv4Address g_defaultNetmask		= { .m_octets{255, 255, 255,   0} };
const IPv4Address g_defaultBroadcast	= { .m_octets{192, 168,   1, 255} };
const IPv4Address g_defaultGateway		= { .m_octets{192, 168,   1,   1} };

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

	//Register protocol handlers with the lower layer
	eth.UseARP(&arp);
	eth.UseIPv4(&ipv4);
	ipv4.UseICMPv4(&icmpv4);
	RegisterProtocolHandlers(ipv4);
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
	@brief Gets the temperature of the SFP+
 */
uint16_t GetSFPTemperature()
{
	//FIXME: assumes internally calibrated

	g_sfpI2C->BlockingWrite8(0xa2, 96);
	uint16_t temp = 0;
	g_sfpI2C->BlockingRead16(0xa2, temp);
	return temp;
}
