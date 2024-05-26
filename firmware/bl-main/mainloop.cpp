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

#include "bootloader.h"

//Application region of flash runs from the end of the bootloader (0x8020000)
//to the start of the KVS (0x080c0000), so 640 kB
//Firmware version string is put right after vector table by linker script at a constant address
uint32_t* const g_appVector  = reinterpret_cast<uint32_t*>(0x8020000);

//@brief Size of the image
const uint32_t g_appImageSize = 640 * 1024;

//Offset of the version string (size of the vector table plus 32 byte alignment)
const uint32_t g_appVersionOffset = 0x2e0;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Hooks called by bootloader code

void Bootloader_Init()
{
	DoInitKVS();
	InitQSPI();
	InitFPGA();
	InitI2C();
	InitEEPROM();
	InitSFP();
	InitManagementPHY();
	InitEthernet();
	InitIP();

	RTC::Unlock();

	//Initialize the FPGA IRQ pin
	g_irq.SetPullMode(GPIOPin::PULL_DOWN);
}

void Bootloader_ClearRxBuffer()
{
}

void Bootloader_FinalCleanup()
{
	g_cliUART.Flush();
}

void BSP_MainLoop()
{
	Bootloader_MainLoop();
}

void RegisterProtocolHandlers(IPv4Protocol& ipv4)
{
	/*
	static ManagementTCPProtocol tcp(&ipv4);
	static ManagementUDPProtocol udp(&ipv4);
	ipv4.UseTCP(&tcp);
	ipv4.UseUDP(&udp);
	g_dhcpClient = &udp.GetDHCP();
	*/
	g_dhcpClient = nullptr;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Run the firmware updater

void __attribute__((noreturn)) Bootloader_FirmwareUpdateFlow()
{
	g_log("In DFU mode\n");

	while(1)
	{
		/*
		//Main event loop
		static uint32_t secTillNext5MinTick = 0;
		static uint32_t next1HzTick = 0;
		static uint32_t next10HzTick = 0;
		static uint32_t nextPhyPoll = 0;
		const uint32_t logTimerMax = 0xf0000000;

		//Wait for an interrupt
		//asm("wfi");

		//Check if anything happened on the FPGA
		CheckForFPGAEvents();

		//Check if we had a PHY link state change at 20 Hz
		//TODO: add irq bit for this so we don't have to poll nonstop
		if(g_logTimer.GetCount() >= nextPhyPoll)
		{
			PollPHYs();
			nextPhyPoll = g_logTimer.GetCount() + 500;
		}

		//Check if we had an optic inserted or removed
		PollSFP();

		//Poll for UART input
		if(g_cliUART.HasInput())
			g_localConsoleSessionContext.OnKeystroke(g_cliUART.BlockingRead());

		if(g_log.UpdateOffset(logTimerMax))
		{
			next1HzTick -= logTimerMax;
			next10HzTick -= logTimerMax;
		}

		//Refresh of activity LEDs and TCP retransmits at 10 Hz
		if(g_logTimer.GetCount() >= next10HzTick)
		{
			UpdateFrontPanelActivityLEDs();
			g_ethProtocol->OnAgingTick10x();

			next10HzTick = g_logTimer.GetCount() + 1000;
		}

		//1 Hz timer for various aging processes
		if(g_logTimer.GetCount() >= next1HzTick)
		{
			g_ethProtocol->OnAgingTick();
			next1HzTick = g_logTimer.GetCount() + 10000;

			//Push channel config to KVS every 5 mins if it's changed
			//DEBUG: every 10 sec
			if(secTillNext5MinTick == 0)
			{
				secTillNext5MinTick = 300;
				SaveChannelConfig();
			}
			else
				secTillNext5MinTick --;

			//Push new register values to front panel every second (it will refresh the panel whenever it wants to)
			UpdateFrontPanelDisplay();
		}
		*/
	}
}
