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
#include "../../front/main/regids.h"
#include <supervisor/SupervisorSPIRegisters.h>
#include <peripheral/ITMStream.h>

void LogTemperatures();
void SendFrontPanelSensor(uint8_t cmd, uint16_t value);
void UpdateFrontPanelActivityLEDs();
void InitFrontPanel();
void TraceLogSensors();

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
	static uint32_t next2HzTick = 0;
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

	//Poll sensors at 2 Hz
	if(g_logTimer.GetCount() >= next2HzTick)
	{
		#ifdef _DEBUG
		TraceLogSensors();
		#endif

		next2HzTick = g_logTimer.GetCount() + 5000;
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

#ifdef _DEBUG
void TraceLogSensors()
{
	static ITMStream sensorStream(0);

	sensorStream.Printf(
		"CSV-NAME,"
		"FAN0_RPM,"
		"FPGA_TEMP,FPGA_VCCINT,FPGA_VCCBRAM,FPGA_VCCAUX,"
		"MAINMCU_TEMP,"
		"SFP_TEMP,"
		"IBC_VIN,IBC_IIN,IBC_TEMP,IBC_VOUT,IBC_IOUT,IBC_VSENSE,"
		"SUPER_MCUTEMP,SUPER_3V3,"
		"SFP_3V3"

		//TODO: IBC_MCUTEMP, IBC_3V3

		"\n"
		);

	sensorStream.Printf(
		"CSV-UNIT,"
		"RPM,"
		"°C,V,V,V,"
		"°C,"
		"°C,"
		"V,A,°C,V,A,V,"
		"°C,V,"
		"V"
		"\n"
		);

	auto ibc_vin = SupervisorRegRead(SUPER_REG_IBCVIN);
	auto ibc_iin = SupervisorRegRead(SUPER_REG_IBCIIN);
	auto ibc_temp = SupervisorRegRead(SUPER_REG_IBCTEMP);
	auto ibc_vout = SupervisorRegRead(SUPER_REG_IBCVOUT);
	auto ibc_iout = SupervisorRegRead(SUPER_REG_IBCIOUT);
	auto ibc_vsense = SupervisorRegRead(SUPER_REG_IBCVSENSE);
	auto super_temp = SupervisorRegRead(SUPER_REG_MCUTEMP);
	auto super_3v3 = SupervisorRegRead(SUPER_REG_3V3);
	auto sfp_3v3 = GetSFP3V3();

	sensorStream.Printf(
		"CSV-DATA,"
		"%d,"
		"%uhk,%uhk,%uhk,%uhk,"
		"%uhk,"
		"%uhk,"
		"%d.%03d,%d.%03d,%uhk,%d.%03d,%d.%03d,%d.%03d,"
		"%uhk,%d.%03d,"
		"%d.%03d"
		"\n",

		GetFanRPM(0),
		GetFPGATemperature(), GetFPGAVCCINT(), GetFPGAVCCBRAM(), GetFPGAVCCAUX(),
		g_dts->GetTemperature(),
		GetSFPTemperature(),
		ibc_vin / 1000, ibc_vin % 1000, ibc_iin / 1000, ibc_iin % 1000, ibc_temp, ibc_vout / 1000, ibc_vout % 1000,
			ibc_iout / 1000, ibc_iout % 1000, ibc_vsense / 1000, ibc_vsense % 1000,
		super_temp, super_3v3 / 1000, super_3v3 % 1000,
		sfp_3v3 / 1000, sfp_3v3 % 1000
		);
}
#endif

void InitFrontPanel()
{
	g_log("Initializing front panel\n");
	LogIndenter li(g_log);

	static APB_SPIHostInterfaceDriver frontDriver(g_frontPanelSPI);
	g_frontSPI = &frontDriver;

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
	g_frontSPI->SetCS(0);
	SendFrontPanelByte(FRONT_GET_STATUS);
	g_logTimer.Sleep(1);
	auto ret = ReadFrontPanelByte();
	g_frontSPI->SetCS(1);
	return ret;
}

void SendFrontPanelByte(uint8_t data)
{
	g_apbfpga.BlockingWrite32(&g_frontPanelSPI->data, data);
	StatusRegisterMaskedWait(&g_frontPanelSPI->status, &g_frontPanelSPI->status2, 0x1, 0x0);
}

uint8_t ReadFrontPanelByte()
{
	//Send the data byte
	g_apbfpga.BlockingWrite32(&g_frontPanelSPI->data, 0x00);

	//Return the response once complete
	StatusRegisterMaskedWait(&g_frontPanelSPI->status, &g_frontPanelSPI->status2, 0x1, 0x0);
	return g_frontPanelSPI->data;
}

void SendFrontPanelSensor(uint8_t cmd, uint16_t value)
{
	g_frontSPI->SetCS(0);
	SendFrontPanelByte(cmd);
	SendFrontPanelByte(value & 0xff);
	SendFrontPanelByte(value >> 8);
	g_frontSPI->SetCS(1);
	g_logTimer.Sleep(1);
}

void SetFrontPanelDirectionLEDs(uint8_t leds)
{
	if(IsFrontPanelDFU())
		return;

	g_frontSPI->SetCS(0);
	SendFrontPanelByte(FRONT_DIR_LEDS);
	SendFrontPanelByte(leds);
	g_frontSPI->SetCS(1);
	g_logTimer.Sleep(1);
}

void UpdateFrontPanelActivityLEDs()
{
	if(IsFrontPanelDFU())
		return;

	//Read LED state
	uint16_t dinval = g_ledGpioInPortActivity->in;
	uint16_t doutval = g_ledGpioOutPortActivity->in;

	//Convert to bytes and send
	g_frontSPI->SetCS(0);
	SendFrontPanelByte(FRONT_PORT_LEDS);
	SendFrontPanelByte(doutval & 0xff);
	SendFrontPanelByte( ((dinval & 0xf) << 4) | ( (doutval >> 8) & 0xf) );
	SendFrontPanelByte(dinval >> 4);
	g_frontSPI->SetCS(1);
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
	g_frontSPI->SetCS(0);
	SendFrontPanelByte(FRONT_IP4_ADDR);
	for(size_t i=0; i<4; i++)
		SendFrontPanelByte(g_ipConfig.m_address.m_octets[i]);
	g_frontSPI->SetCS(1);

	//IPv4 prefix length
	SendFrontPanelSensor(FRONT_IP4_SUBNET, __builtin_popcount(g_ipConfig.m_netmask.m_word));

	//TODO: set IPv6 address

	//Set Ethernet link state
	g_frontSPI->SetCS(0);
	SendFrontPanelByte(FRONT_ETH_LINK);
	if(g_sfpLinkUp)
		SendFrontPanelByte(0x03);
	else if(g_basetLinkUp)
		SendFrontPanelByte(g_basetLinkSpeed);
	else
		SendFrontPanelByte(0xff);
	g_frontSPI->SetCS(1);

	g_logTimer.Sleep(2);

	//Set Ethernet DHCP state
	g_frontSPI->SetCS(0);
	SendFrontPanelByte(FRONT_IPV4_DHCP);
	SendFrontPanelByte(g_dhcpClient->IsEnabled());
	g_frontSPI->SetCS(1);

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
		g_frontSPI->SetCS(0);
		SendFrontPanelByte(FRONT_SERIAL);
		for(size_t i=0; i<8; i++)
			SendFrontPanelByte(g_fpgaSerial[7-i]);
		g_frontSPI->SetCS(1);

		//Our firmware version number
		static const char* buildtime = __TIME__;
		buf.Clear();
		buf.Printf("%s %c%c%c%c%c%c",
			__DATE__, buildtime[0], buildtime[1], buildtime[3], buildtime[4], buildtime[6], buildtime[7]);
		g_frontSPI->SetCS(0);
		SendFrontPanelByte(FRONT_MCU_FW);
		for(size_t i=0; i<sizeof(tmp); i++)
			SendFrontPanelByte(tmp[i]);
		g_frontSPI->SetCS(1);

		//Supervisor firmware version
		g_logTimer.Sleep(2);
		g_frontSPI->SetCS(0);
		SendFrontPanelByte(FRONT_SUPER_FW);
		for(size_t i=0; i<sizeof(g_superVersion); i++)
			SendFrontPanelByte(g_superVersion[i]);
		g_frontSPI->SetCS(1);

		//IBC firmware version
		g_logTimer.Sleep(2);
		g_frontSPI->SetCS(0);
		SendFrontPanelByte(FRONT_IBC_FW);
		for(size_t i=0; i<sizeof(g_ibcVersion); i++)
			SendFrontPanelByte(g_ibcVersion[i]);
		g_frontSPI->SetCS(1);

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
		g_frontSPI->SetCS(0);
		SendFrontPanelByte(FRONT_FPGA_FW);
		for(size_t i=0; i<sizeof(tmp); i++)
			SendFrontPanelByte(tmp[i]);
		g_frontSPI->SetCS(1);
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

	g_frontSPI->SetCS(0);
	SendFrontPanelByte(FRONT_TIMESTAMP);
	for(size_t i=0; i<sizeof(tmp); i++)
		SendFrontPanelByte(tmp[i]);
	g_frontSPI->SetCS(1);

	firstRefresh = false;
}

uint16_t SupervisorRegRead(uint8_t regid)
{
	*g_superSPICS = 0;
	g_superSPI.BlockingWrite(regid);
	g_superSPI.WaitForWrites();
	g_superSPI.DiscardRxData();
	g_logTimer.Sleep(5);
	g_superSPI.BlockingRead();	//discard dummy byte
	uint16_t tmp = g_superSPI.BlockingRead();
	tmp |= (g_superSPI.BlockingRead() << 8);
	*g_superSPICS = 1;

	g_logTimer.Sleep(1);

	return tmp;
}

void OnEthernetLinkStateChanged()
{
	g_displayRefreshPending = true;
}
