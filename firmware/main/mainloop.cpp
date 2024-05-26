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

#include "triggercrossbar.h"
#include "CrossbarCLISessionContext.h"
#include "../front/regids.h"
#include "../super/superregs.h"

void LogTemperatures();
void SendFrontPanelSensor(uint8_t cmd, uint16_t value);
void UpdateFrontPanelActivityLEDs();
void InitFrontPanel();

///@brief Output stream for local serial console
UARTOutputStream g_localConsoleOutputStream;

///@brief Context data structure for local serial console
CrossbarCLISessionContext g_localConsoleSessionContext;

void App_Init()
{
	//Enable interrupts early on since we use them for e.g. debug logging during boot
	EnableInterrupts();

	//Basic hardware setup
	InitLEDs();

	DoInitKVS();

	//Set up the quad SPI and connect to the FPGA
	InitQSPI();
	InitFPGA();
	InitRelays();

	//Get our MAC address
	InitI2C();
	InitEEPROM();

	//Set up the DACs
	InitDACs();

	//Connect to the supervisor
	InitSupervisor();

	//Bring up sensors
	InitSensors();

	//Begin initializing network ports
	InitSFP();
	InitManagementPHY();

	//Initialize our local Ethernet interface and TCP/IP stack
	InitEthernet();
	InitIP();

	//Load instrument channel configuration from the KVS
	LoadChannelConfig();

	//Initialize the local console
	g_localConsoleOutputStream.Initialize(&g_cliUART);
	g_localConsoleSessionContext.Initialize(&g_localConsoleOutputStream, "localadmin");

	//Bring up the front panel
	InitFrontPanel();

	//Show the initial prompt
	g_localConsoleSessionContext.PrintPrompt();

	//Initialize the FPGA IRQ pin
	g_irq.SetPullMode(GPIOPin::PULL_DOWN);
}

void BSP_MainLoopIteration()
{
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
}

/**
	@brief Checks if the FPGA needs any events serviced

	@return true if there are more events to process
 */
bool CheckForFPGAEvents()
{
	if(g_irq)
		PollFPGA();

	return g_irq;
}

/**
	@brief Reads the FPGA status register to see why it sent us an IRQ
 */
void PollFPGA()
{
	uint16_t fpgastat = *g_irqStat;

	//New Ethernet frame ready?
	if(fpgastat & 1)
	{
		auto frame = g_ethIface->GetRxFrame();
		if(frame != nullptr)
			g_ethProtocol->OnRxFrame(frame);
	}
}

void InitFrontPanel()
{
	g_log("Initializing front panel\n");
	LogIndenter li(g_log);

	//Check what mode we're in
	auto mode = GetFrontPanelMode();
	if(mode == FRONT_NORMAL)
		g_log("Front panel MCU is up\n");
	else if(mode == FRONT_BOOTLOADER)
	{
		g_log(Logger::ERROR, "Front panel MCU is in DFU mode (we didn't expect this)\n");
		g_frontPanelDFUInProgress = true;
		return;
	}
	else
	{
		g_log(Logger::ERROR, "Unexpected response 0x%02x to FRONT_GET_STATUS\n", mode);
		return;
	}

	//Update the display and set the direction LEDs to all-input (default state on new FPGA bitstream load)
	SetFrontPanelDirectionLEDs(0xf0);
	UpdateFrontPanelActivityLEDs();
	UpdateFrontPanelDisplay();
}

uint8_t GetFrontPanelMode()
{
	SetFrontPanelCS(0);
	SendFrontPanelByte(FRONT_GET_STATUS);
	g_logTimer.Sleep(1);
	auto ret = ReadFrontPanelByte();
	SetFrontPanelCS(1);
	return ret;
}

void SetFrontPanelCS(bool b)
{
	g_apbfpga.BlockingWrite16(&g_frontPanelSPI->cs_n, b);
}

void SendFrontPanelByte(uint8_t data)
{
	g_apbfpga.BlockingWrite16(&g_frontPanelSPI->data, data);
	StatusRegisterMaskedWait(&g_frontPanelSPI->status, &g_frontPanelSPI->status2, 0x1, 0x0);
}

uint8_t ReadFrontPanelByte()
{
	//Send the data byte
	g_apbfpga.BlockingWrite16(&g_frontPanelSPI->data, 0x00);

	//Return the response once complete
	StatusRegisterMaskedWait(&g_frontPanelSPI->status, &g_frontPanelSPI->status2, 0x1, 0x0);
	return g_frontPanelSPI->data;
}

void SendFrontPanelSensor(uint8_t cmd, uint16_t value)
{
	SetFrontPanelCS(0);
	SendFrontPanelByte(cmd);
	SendFrontPanelByte(value & 0xff);
	SendFrontPanelByte(value >> 8);
	SetFrontPanelCS(1);
}

void SetFrontPanelDirectionLEDs(uint8_t leds)
{
	if(IsFrontPanelDFU())
		return;

	SetFrontPanelCS(0);
	SendFrontPanelByte(FRONT_DIR_LEDS);
	SendFrontPanelByte(leds);
	SetFrontPanelCS(1);
}

void UpdateFrontPanelActivityLEDs()
{
	if(IsFrontPanelDFU())
		return;

	//Read LED state
	uint16_t dinval = g_ledGpioInPortActivity->in;
	uint16_t doutval = g_ledGpioOutPortActivity->in;

	//Convert to bytes and send
	SetFrontPanelCS(0);
	SendFrontPanelByte(FRONT_PORT_LEDS);
	SendFrontPanelByte(doutval & 0xff);
	SendFrontPanelByte( ((dinval & 0xf) << 4) | ( (doutval >> 8) & 0xf) );
	SendFrontPanelByte(dinval >> 4);
	SetFrontPanelCS(1);
}

/**
	@brief Push new content to the front panel display
 */
void UpdateFrontPanelDisplay()
{
	if(IsFrontPanelDFU())
		return;

	static bool firstRefresh = true;

	char tmp[20] = {0};
	StringBuffer buf(tmp, sizeof(tmp));

	//Update IPv4 address
	SetFrontPanelCS(0);
	SendFrontPanelByte(FRONT_IP4_ADDR);
	for(size_t i=0; i<4; i++)
		SendFrontPanelByte(g_ipConfig.m_address.m_octets[i]);
	SetFrontPanelCS(1);

	//IPv4 prefix length
	SendFrontPanelSensor(FRONT_IP4_SUBNET, __builtin_popcount(g_ipConfig.m_netmask.m_word));

	//TODO: set IPv6 address

	//Set Ethernet link state
	SetFrontPanelCS(0);
	SendFrontPanelByte(FRONT_ETH_LINK);
	if(g_sfpLinkUp)
		SendFrontPanelByte(0x03);
	else if(g_basetLinkUp)
		SendFrontPanelByte(g_basetLinkSpeed);
	else
		SendFrontPanelByte(0xff);
	SetFrontPanelCS(1);

	//Set Ethernet DHCP state
	SetFrontPanelCS(0);
	SendFrontPanelByte(FRONT_IPV4_DHCP);
	SendFrontPanelByte(g_dhcpClient->IsEnabled());
	SetFrontPanelCS(1);

	/*
		Only update version number strings once per boot to save SPI bus bandwidth.

		Rationale:
		* If IBC or supervisor is reflashed, we will lose power and reset
		* If we are reflashed, we will obviously reset
		* If FPGA is reprogrammed, we will be reset by the supervisor

		But if front panel is reflashed, it won't show version strings until we reset the main MCU.
		This probably isn't a huge deal?
		TODO: allow forcing a full refresh or something
	 */
	if(firstRefresh)
	{
		g_log("Initial front panel refresh\n");

		//Set serial number
		SetFrontPanelCS(0);
		SendFrontPanelByte(FRONT_SERIAL);
		for(size_t i=0; i<8; i++)
			SendFrontPanelByte(g_fpgaSerial[7-i]);
		SetFrontPanelCS(1);

		//Our firmware version number
		static const char* buildtime = __TIME__;
		buf.Clear();
		buf.Printf("%s %c%c%c%c%c%c",
			__DATE__, buildtime[0], buildtime[1], buildtime[3], buildtime[4], buildtime[6], buildtime[7]);
		buf.Printf("hai");
		SetFrontPanelCS(0);
		SendFrontPanelByte(FRONT_MCU_FW);
		for(size_t i=0; i<sizeof(tmp); i++)
			SendFrontPanelByte(tmp[i]);
		SetFrontPanelCS(1);

		//Supervisor firmware version
		SetFrontPanelCS(0);
		SendFrontPanelByte(FRONT_SUPER_FW);
		for(size_t i=0; i<sizeof(g_superVersion); i++)
			SendFrontPanelByte(g_superVersion[i]);
		SetFrontPanelCS(1);

		//IBC firmware version
		SetFrontPanelCS(0);
		SendFrontPanelByte(FRONT_IBC_FW);
		for(size_t i=0; i<sizeof(g_ibcVersion); i++)
			SendFrontPanelByte(g_ibcVersion[i]);
		SetFrontPanelCS(1);

		//Format FPGA firmware string based on the usercode (see XAPP1232)
		buf.Clear();
		int day = g_usercode >> 27;
		int mon = (g_usercode >> 23) & 0xf;
		int yr = 2000 + ((g_usercode >> 17) & 0x3f);
		int hr = (g_usercode >> 12) & 0x1f;
		int min = (g_usercode >> 6) & 0x3f;
		int sec = g_usercode & 0x3f;
		static const char* months[16] =
		{
			"",		//months in usercode use 1-based indexing
			"Jan",
			"Feb",
			"Mar",
			"Apr",
			"May",
			"Jun",
			"Jul",
			"Aug",
			"Sep",
			"Oct",
			"Nov",
			"Dec",
			"",
			"",
			""
		};
		buf.Printf("%s %2d %04d %02d%02d%02d", months[mon], day, yr, hr, min, sec);
		SetFrontPanelCS(0);
		SendFrontPanelByte(FRONT_FPGA_FW);
		for(size_t i=0; i<sizeof(tmp); i++)
			SendFrontPanelByte(tmp[i]);
		SetFrontPanelCS(1);
	}

	//Temperatures
	SendFrontPanelSensor(FRONT_FPGA_TEMP, GetFPGATemperature());
	SendFrontPanelSensor(FRONT_MCU_TEMP, g_dts->GetTemperature());

	//IBC voltages
	//Report vsense not vout of the IBC because vout seems to be noisier
	//and vsense is the main logic board 12V rail which is what we really care about anyway
	SendFrontPanelSensor(FRONT_IBC_VIN, SupervisorRegRead(SUPER_REG_IBCVIN));
	SendFrontPanelSensor(FRONT_IBC_IIN, SupervisorRegRead(SUPER_REG_IBCIIN));
	SendFrontPanelSensor(FRONT_IBC_VOUT, SupervisorRegRead(SUPER_REG_IBCVSENSE));
	SendFrontPanelSensor(FRONT_IBC_IOUT, SupervisorRegRead(SUPER_REG_IBCIOUT));
	SendFrontPanelSensor(FRONT_IBC_TEMP, SupervisorRegRead(SUPER_REG_IBCTEMP));

	//Fan
	SendFrontPanelSensor(FRONT_FAN_RPM, GetFanRPM(0));

	//Timestamp of data
	tm rtctime;
	uint16_t rtcsubsec;
	RTC::GetTime(rtctime, rtcsubsec);
	buf.Clear();
	buf.Printf("%02d:%02d:%02d",
		rtctime.tm_hour,
		rtctime.tm_min,
		rtctime.tm_sec);

	SetFrontPanelCS(0);
	SendFrontPanelByte(FRONT_TIMESTAMP);
	for(size_t i=0; i<sizeof(tmp); i++)
		SendFrontPanelByte(tmp[i]);
	SetFrontPanelCS(1);

	firstRefresh = false;
}

uint16_t SupervisorRegRead(uint8_t regid)
{
	*g_superSPICS = 0;
	g_superSPI.BlockingWrite(regid);
	g_superSPI.WaitForWrites();
	g_superSPI.DiscardRxData();
	g_superSPI.BlockingRead();	//discard dummy byte
	uint16_t tmp = g_superSPI.BlockingRead();
	tmp |= (g_superSPI.BlockingRead() << 8);
	*g_superSPICS = 1;

	g_logTimer.Sleep(1);

	return tmp;
}

/**
	@brief Debug logging
 */
/*
void LogTemperatures()
{
	//For now, log headers every time
	//This is wasteful but allows reconnecting frequently

	int temps[8] =
	{
		ReadThermalSensor(g_tempSensorAddrs[0]),
		ReadThermalSensor(g_tempSensorAddrs[1]),
		ReadThermalSensor(g_tempSensorAddrs[2]),
		ReadThermalSensor(g_tempSensorAddrs[3]),
		GetSFPTemperature(),
		GetVSC8512Temperature(),
		GetFPGATemperature(),
		g_dts->GetTemperature()
	};

	g_cliUART.Printf("CSV-NAME,Fan0,Fan1,3V3Reg,1V2Reg,SGMIIPhys,QDR,SFP,VSC8512,FPGA,MCU\n");
	g_cliUART.Printf("CSV-UNIT,RPM,RPM,°C,°C,°C,°C,°C,°C,°C,°C\n");
	g_cliUART.Printf("CSV-DATA,%d,%d,%d.%02d,%d.%02d,%d.%02d,%d.%02d,%d.%02d,%d.%02d,%d.%02d,%d.%02d\n",
		GetFanRPM(0),
		GetFanRPM(1),
		(temps[0] >> 8),
		static_cast<int>(((temps[0] & 0xff) / 256.0) * 100),
		(temps[1] >> 8),
		static_cast<int>(((temps[1] & 0xff) / 256.0) * 100),
		(temps[2] >> 8),
		static_cast<int>(((temps[2] & 0xff) / 256.0) * 100),
		(temps[3] >> 8),
		static_cast<int>(((temps[3] & 0xff) / 256.0) * 100),

		(temps[4] >> 8),
		static_cast<int>(((temps[4] & 0xff) / 256.0) * 100),
		(temps[5] >> 8),
		static_cast<int>(((temps[5] & 0xff) / 256.0) * 100),
		(temps[6] >> 8),
		static_cast<int>(((temps[6] & 0xff) / 256.0) * 100),
		(temps[7] >> 8),
		static_cast<int>(((temps[7] & 0xff) / 256.0) * 100)
		);
}*/

void OnEthernetLinkStateChanged()
{
	g_displayRefreshPending = true;
}
