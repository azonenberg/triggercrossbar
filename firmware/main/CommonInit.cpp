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
	@brief	Common initialization used by both simulation and real hardware
 */

#include "triggercrossbar.h"

const IPv4Address g_defaultIP			= { .m_octets{192, 168,   1,   2} };
const IPv4Address g_defaultNetmask		= { .m_octets{255, 255, 255,   0} };
const IPv4Address g_defaultBroadcast	= { .m_octets{192, 168,   1, 255} };
const IPv4Address g_defaultGateway		= { .m_octets{192, 168,   1,   1} };

/**
	@brief Initialize the logging library
 */
void InitLog(CharacterDevice* logdev, Timer* timer)
{
	g_logTimer = timer;

	static LogSink<MAX_LOG_SINKS> sink(logdev);
	g_logSink = &sink;

	g_log.Initialize(g_logSink, timer);
	g_log("Logging ready\n");
	g_log("trigger-crossbar MCU firmware v0.1 by Andrew D. Zonenberg\n");
	{
		LogIndenter li(g_log);
		g_log("This system is open hardware! Board design files and firmware/gateware source code are at:\n");
		g_log("https://github.com/azonenberg/triggercrossbar\n");
	}
	g_log("Firmware compiled at %s on %s\n", __TIME__, __DATE__);
}

/**
	@brief Set up the microkvs key-value store for persisting our configuration
 */
void InitKVS(StorageBank* left, StorageBank* right, uint32_t logsize)
{
	g_log("Initializing microkvs key-value store\n");
	static KVS kvs(left, right, logsize);
	g_kvs = &kvs;

	LogIndenter li(g_log);
	g_log("Block size:  %d bytes\n", kvs.GetBlockSize());
	g_log("Log:         %d / %d slots free\n", (int)kvs.GetFreeLogEntries(), (int)kvs.GetLogCapacity());
	g_log("Data:        %d / %d bytes free\n", (int)kvs.GetFreeDataSpace(), (int)kvs.GetDataCapacity());
	g_log("Active bank: %s\n", kvs.IsLeftBankActive() ? "left" : "right");
}

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
		uint32_t idcode = g_apbfpga.BlockingRead32(BASE_SYSINFO + REG_FPGA_IDCODE);
		g_apbfpga.BlockingRead(BASE_SYSINFO + REG_FPGA_SERIAL, g_fpgaSerial, 8);

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
	g_usercode = g_apbfpga.BlockingRead32(BASE_SYSINFO + REG_USERCODE);
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
		g_fpga->BlockingWrite16(REG_RELAY_TOGGLE, 0x8000 | chan);
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
