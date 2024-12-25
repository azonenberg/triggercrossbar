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
#include "LocalConsoleTask.h"
#include "FPGATask.h"
#include <tcpip/PhyPollTask.h>
#include <tcpip/IPAgingTask1Hz.h>
#include <tcpip/IPAgingTask10Hz.h>
#include "OneHzTimerTask.h"
#include "TwentyHzTimerTask.h"
#include "TwoHzTimerTask.h"
#include "TenHzTimerTask.h"

void LogTemperatures();
void SendFrontPanelSensor(uint8_t cmd, uint16_t value);
void InitFrontPanel();

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
	InitMacEEPROM();

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

	//Bring up the front panel
	InitFrontPanel();

	//Initialize tasks
	static OneHzTimerTask timerTask1;
	static TwoHzTimerTask timerTask2;
	static TenHzTimerTask timerTask10;
	static TwentyHzTimerTask timerTask20;
	static IPAgingTask1Hz agingTask1;
	static IPAgingTask10Hz agingTask10;
	static LocalConsoleTask localConsoleTask;
	static PhyPollTask phyTask;
	static FPGATask fpgaTask;

	g_tasks.push_back(&timerTask20);
	g_tasks.push_back(&timerTask10);
	g_tasks.push_back(&timerTask2);
	g_tasks.push_back(&timerTask1);
	g_tasks.push_back(&localConsoleTask);
	g_tasks.push_back(&fpgaTask);
	g_tasks.push_back(&agingTask1);
	g_tasks.push_back(&agingTask10);
	g_tasks.push_back(&phyTask);

	g_timerTasks.push_back(&timerTask20);
	g_timerTasks.push_back(&timerTask10);
	g_timerTasks.push_back(&timerTask2);
	g_timerTasks.push_back(&timerTask1);
	g_timerTasks.push_back(&agingTask1);
	g_timerTasks.push_back(&agingTask10);
	g_timerTasks.push_back(&phyTask);

	//Initialize the FPGA IRQ pin
	g_irq.SetPullMode(GPIOPin::PULL_DOWN);

	//Assert GPIOA8 (MCU_READY) once firmware is fully initialized
	static GPIOPin mcuUp(&GPIOA, 8, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW, 0, true);
	mcuUp = true;
}

void InitFrontPanel()
{
	g_log("Initializing front panel\n");
	LogIndenter li(g_log);

	static APB_SPIHostInterfaceDriver frontDriver(&FFRONTSPI);
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
	FFRONTSPI.data = data;
	StatusRegisterMaskedWait(&FFRONTSPI.status, &FFRONTSPI.status2, 0x1, 0x0);
}

uint8_t ReadFrontPanelByte()
{
	//Send a dummy byte
	FFRONTSPI.data = 0;

	//Return the response once complete
	StatusRegisterMaskedWait(&FFRONTSPI.status, &FFRONTSPI.status2, 0x1, 0x0);
	return FFRONTSPI.data;
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
	SendFrontPanelSensor(FRONT_MCU_TEMP, g_dts.GetTemperature());

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
