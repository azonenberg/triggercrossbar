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
#include <microkvs/driver/STM32StorageBank.h>
#include "../front/regids.h"
#include "../super/superregs.h"

void LogTemperatures();
void SetFrontPanelCS(bool b);
void SendFrontPanelByte(uint8_t data);
void SendFrontPanelSensor(uint8_t cmd, uint16_t value);

int main()
{
	//Initialize power (must be the very first thing done after reset)
	Power::ConfigureSMPSToLDOCascade(Power::VOLTAGE_1V8, RANGE_VOS0);

	//Copy .data from flash to SRAM (for some reason the default newlib startup won't do this??)
	memcpy(&__data_start, &__data_romstart, &__data_end - &__data_start + 1);

	//Enable SYSCFG before changing any settings on it
	RCCHelper::EnableSyscfg();

	//Basic hardware setup
	InitClocks();
	InitLEDs();
	InitTimer();
	InitUART();
	InitLog(g_cliUART, g_logTimer);
	DetectHardware();

	/*
		Use sectors 6 and 7 of main flash (in single bank mode) for a 128 kB microkvs

		Each log entry is 64 bytes, and we want to allocate ~50% of storage to the log since our objects are pretty
		small (SSH keys, IP addresses, etc). A 1024-entry log is a nice round number, and comes out to 64 kB or 50%,
		leaving the remaining 64 kB or 50% for data.
	 */
	static STM32StorageBank left(reinterpret_cast<uint8_t*>(0x080c0000), 0x20000);
	static STM32StorageBank right(reinterpret_cast<uint8_t*>(0x080e0000), 0x20000);
	InitKVS(&left, &right, 1024);

	//Set up the quad SPI and connect to the FPGA
	InitQSPI();
	InitFPGA();

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

	//Create a CLI stream for the UART
	UARTOutputStream uartStream;
	uartStream.Initialize(g_cliUART);

	//Initialize the CLI for the UART
	CrossbarCLISessionContext uartContext;
	uartContext.Initialize(&uartStream, "user");

	//Update the display and set the direction LEDs to all-input (default state on new FPGA bitstream load)
	SetFrontPanelDirectionLEDs(0xf0);
	UpdateFrontPanelDisplay();

	//Enable interrupts only after all setup work is done
	EnableInterrupts();

	//Show the initial prompt
	uartContext.PrintPrompt();

	//Initialize the FPGA IRQ pin
	GPIOPin irq(&GPIOH, 6, GPIOPin::MODE_INPUT, GPIOPin::SLEW_SLOW);
	irq.SetPullMode(GPIOPin::PULL_DOWN);

	//Main event loop
	uint32_t nextAgingTick = 0;
	while(1)
	{
		//Wait for an interrupt
		//asm("wfi");

		//Check if anything happened on the FPGA
		if(irq)
			PollFPGA();

		//Check if we had a PHY link state change
		//TODO: add irq bit for this so we don't have to poll nonstop
		PollPHYs();

		//Check if we had an optic inserted or removed
		PollSFP();

		//Poll for UART input
		if(g_cliUART->HasInput())
			uartContext.OnKeystroke(g_cliUART->BlockingRead());

		//Check for aging on stuff once a second
		if(g_logTimer->GetCount() > nextAgingTick)
		{
			g_ethProtocol->OnAgingTick();
			nextAgingTick = g_logTimer->GetCount() + 10000;
		}
	}
	return 0;
}

/**
	@brief Reads the FPGA status register to see why it sent us an IRQ
 */
void PollFPGA()
{
	uint16_t fpgastat = g_fpga->BlockingRead16(REG_FPGA_IRQSTAT);

	//New Ethernet frame ready?
	if(fpgastat & 1)
	{
		auto frame = g_ethIface->GetRxFrame();
		if(frame != NULL)
			g_ethProtocol->OnRxFrame(frame);
	}
}

void SetFrontPanelCS(bool b)
{
	g_fpga->BlockingWrite8(REG_FRONT_CTRL, b);
}

void SendFrontPanelByte(uint8_t data)
{
	//Send the data byte
	g_fpga->BlockingWrite8(REG_FRONT_DATA, data);

	//Block until not busy
	while(0 != g_fpga->BlockingRead8(REG_FRONT_STAT))
	{}
}

void SendFrontPanelSensor(uint8_t cmd, uint16_t value)
{
	SetFrontPanelCS(0);
	SendFrontPanelByte(cmd);
	SendFrontPanelByte(value & 0xff);
	SendFrontPanelByte(value >> 8);
	SetFrontPanelCS(1);
	g_logTimer->Sleep(2);
}

void SetFrontPanelDirectionLEDs(uint8_t leds)
{
	SetFrontPanelCS(0);
	SendFrontPanelByte(FRONT_DIR_LEDS);
	SendFrontPanelByte(leds);
	SetFrontPanelCS(1);
	g_logTimer->Sleep(2);
}

/**
	@brief Push new content to the front panel display
 */
void UpdateFrontPanelDisplay()
{
	g_log("Updating front panel display\n");

	char tmp[20] = {0};
	StringBuffer buf(tmp, sizeof(tmp));

	//TODO: 50 second timeout between refresh cycles, can't update if already updating
	//(but queue the refresh request and refresh again as soon as we can)

	//Update IPv4 address
	SetFrontPanelCS(0);
	SendFrontPanelByte(FRONT_IP4_ADDR);
	for(size_t i=0; i<4; i++)
		SendFrontPanelByte(g_ipConfig.m_address.m_octets[i]);
	SetFrontPanelCS(1);
	g_logTimer->Sleep(2);

	//TODO: set IPv6 address

	//Set Ethernet link state
	SetFrontPanelCS(0);
	SendFrontPanelByte(FRONT_ETH_LINK);
	if(!g_basetLinkUp)
		SendFrontPanelByte(0xff);
	else
		SendFrontPanelByte(0x03);	//TODO: actual speed
	SetFrontPanelCS(1);
	g_logTimer->Sleep(2);

	//Set serial number
	SetFrontPanelCS(0);
	SendFrontPanelByte(FRONT_SERIAL);
	for(size_t i=0; i<8; i++)
		SendFrontPanelByte(g_fpgaSerial[i]);
	SetFrontPanelCS(1);
	g_logTimer->Sleep(2);

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
	g_logTimer->Sleep(2);

	//Supervisor firmware version
	SetFrontPanelCS(0);
	SendFrontPanelByte(FRONT_SUPER_FW);
	for(size_t i=0; i<sizeof(g_superVersion); i++)
		SendFrontPanelByte(g_superVersion[i]);
	SetFrontPanelCS(1);
	g_logTimer->Sleep(2);

	//IBC firmware version
	SetFrontPanelCS(0);
	SendFrontPanelByte(FRONT_IBC_FW);
	for(size_t i=0; i<sizeof(g_ibcVersion); i++)
		SendFrontPanelByte(g_ibcVersion[i]);
	SetFrontPanelCS(1);
	g_logTimer->Sleep(2);

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
	buf.Printf("%s %02d %04d %02d%02d%02d", months[mon], day, yr, hr, min, sec);
	SetFrontPanelCS(0);
	SendFrontPanelByte(FRONT_FPGA_FW);
	for(size_t i=0; i<sizeof(tmp); i++)
		SendFrontPanelByte(tmp[i]);
	SetFrontPanelCS(1);
	g_logTimer->Sleep(2);

	//Temperatures
	SendFrontPanelSensor(FRONT_FPGA_TEMP, GetFPGATemperature());
	SendFrontPanelSensor(FRONT_MCU_TEMP, g_dts->GetTemperature());

	//IBC voltages
	SendFrontPanelSensor(FRONT_IBC_VIN, SupervisorRegRead(SUPER_REG_IBCVIN));
	SendFrontPanelSensor(FRONT_IBC_IIN, SupervisorRegRead(SUPER_REG_IBCIIN));
	SendFrontPanelSensor(FRONT_IBC_VOUT, SupervisorRegRead(SUPER_REG_IBCVOUT));
	SendFrontPanelSensor(FRONT_IBC_IOUT, SupervisorRegRead(SUPER_REG_IBCIOUT));
	SendFrontPanelSensor(FRONT_IBC_TEMP, SupervisorRegRead(SUPER_REG_IBCTEMP));

	//Refresh the display
	SetFrontPanelCS(0);
	SendFrontPanelByte(FRONT_REFRESH);
	SetFrontPanelCS(1);
	g_logTimer->Sleep(2);

	g_log("Done\n");
}

uint16_t SupervisorRegRead(uint8_t regid)
{
	*g_superSPICS = 0;
	g_superSPI->BlockingWrite(regid);
	g_superSPI->WaitForWrites();
	g_superSPI->DiscardRxData();
	uint16_t tmp = g_superSPI->BlockingRead();
	tmp |= (g_superSPI->BlockingRead() << 8);
	*g_superSPICS = 1;

	g_logTimer->Sleep(1);

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

	g_cliUART->Printf("CSV-NAME,Fan0,Fan1,3V3Reg,1V2Reg,SGMIIPhys,QDR,SFP,VSC8512,FPGA,MCU\n");
	g_cliUART->Printf("CSV-UNIT,RPM,RPM,°C,°C,°C,°C,°C,°C,°C,°C\n");
	g_cliUART->Printf("CSV-DATA,%d,%d,%d.%02d,%d.%02d,%d.%02d,%d.%02d,%d.%02d,%d.%02d,%d.%02d,%d.%02d\n",
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
