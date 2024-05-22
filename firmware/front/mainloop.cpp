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

#include "frontpanel.h"
#include "regids.h"

void RefreshDisplay(bool full);

const uint8_t g_tempI2cAddress = 0x90;

//I2C2 runs off our APB1 clock (40 MHz)
//Prescale by 4 to get 10 MHz
//Divide by 100 after that to get 100 kHz
I2C g_i2c(&I2C2, 4, 100);

TCA6424A* g_expander = nullptr;

//Divide 40 MHz base clock by 128 to get 312.5 kHz SPI
//We can run up to 10 MHz for writes but readback Fmax is ~2 MHz
DisplaySPIType g_displaySPI(&SPI2, false, 128);

Display* g_display = nullptr;

GPIOPin* g_inmodeLED[4] = {nullptr};
GPIOPin* g_outmodeLED[4] = {nullptr};

//Display state
int g_linkSpeed = 0;
uint8_t g_ipv4Addr[4] = {0};
uint16_t g_ipv6Addr[8] = {0};
uint8_t g_serial[8] = {0};
char g_mcuFirmware[20] = {0};
char g_ibcFirmware[20] = {0};
char g_superFirmware[20] = {0};
char g_fpgaFirmware[20] = {0};
uint16_t g_fpgaTemp = 0;
uint16_t g_mcuTemp = 0;
uint16_t g_ibcTemp = 0;
uint16_t g_vin = 0;
uint16_t g_iin = 0;
uint16_t g_vout = 0;
uint16_t g_iout = 0;
uint16_t g_fanspeed = 0;
uint16_t g_ipv4SubnetSize = 0;
uint16_t g_ipv6SubnetSize = 0;
bool g_staticIP = true;
char g_dataTimestamp[20] = {0};

//Indicates the main MCU is alive
bool	g_mainMCUDown = true;

bool RxSPIString(uint8_t nbyte, char* buf, uint8_t size, uint8_t data);

///@brief The battery-backed RAM used to store state across power cycles
volatile BootloaderBBRAM* g_bbram = reinterpret_cast<volatile BootloaderBBRAM*>(&_RTC.BKP[0]);

void BSP_MainLoopIteration()
{
	const int logTimerMax = 60000;
	const uint32_t displayRefreshInterval = 3600;						//1 hour
	const uint32_t fullRefreshInterval = displayRefreshInterval * 24;	//1 day

	static uint32_t nextDisplayRefresh = 30;
	static uint32_t nextFullRefresh = 5;		//Wait 5 sec after power up to do the first full refresh,
										//so main FPGA and MCU have a chance to come up

	static uint32_t next1HzTick = 0;
	static uint8_t nbyte = 0;
	static uint8_t cmd = 0;
	static uint32_t secSinceLastMcuUpdate = 0;

	//Check for overflows on our log message timer
	if(g_log.UpdateOffset(logTimerMax) && (next1HzTick >= logTimerMax) )
		next1HzTick -= logTimerMax;

	//Run the display state machine
	g_display->OnTick();

	//1 Hz timer event for display refreshes
	if(g_logTimer.GetCount() >= next1HzTick)
	{
		//g_log("1 Hz tick nextFull = %d nextDisplay = %d refreshing=%d\n",
		//	nextFullRefresh, nextDisplayRefresh, g_display->IsRefreshInProgress());

		next1HzTick = g_logTimer.GetCount() + 10000;

		//Watchdog timer to detect main MCU acting up
		secSinceLastMcuUpdate ++;
		if(secSinceLastMcuUpdate > 5)
		{
			if(!g_mainMCUDown)
			{
				g_mainMCUDown = true;
				nextDisplayRefresh = 1;
			}
		}

		//Update display if needed
		if(!g_display->IsRefreshInProgress())
		{
			//Full refresh once a day
			if(nextFullRefresh == 0)
			{
				nextFullRefresh = fullRefreshInterval;
				nextDisplayRefresh = displayRefreshInterval;
				RefreshDisplay(true);
			}

			//Default to refreshing the display once an hour
			else if(nextDisplayRefresh == 0)
			{
				nextDisplayRefresh = displayRefreshInterval;
				RefreshDisplay(false);
			}

			//Bump timer counts
			nextDisplayRefresh --;
			nextFullRefresh --;
		}
	}

	//Read and process SPI events
	if(g_fpgaSPI.HasEvents())
	{
		auto event = g_fpgaSPI.GetEvent();

		//Reset byte count on CS# rising or falling edge
		if(event.type == SPIEvent::TYPE_CS)
			nbyte = 0;

		//Process data byte
		else
		{
			auto data = event.data;

			//If main MCU was down, it's now up again
			secSinceLastMcuUpdate = 0;
			if(g_mainMCUDown)
			{
				g_mainMCUDown = false;
				nextDisplayRefresh = 1;
			}

			//First byte is command
			if(nbyte == 0)
			{
				cmd = data;

				switch(cmd)
				{
					//Reboot in bootloader mode
					case FRONT_ENTER_DFU:
						g_bbram->m_state = STATE_DFU;
						Reset();
						break;

					//Commands that produce SPI output
					case FRONT_GET_STATUS:
						{
							if(g_misoIsJtag)
								SetMisoToSPIMode();

							const uint8_t tmp = FRONT_NORMAL;
							g_fpgaSPI.NonblockingWriteFifo(&tmp, sizeof(tmp));
						}
						break;

					//sending any other SPI command returns us to JTAG mode
					default:
						if(!g_misoIsJtag)
							SetMisoToJTAGMode();
						break;
				}
			}

			//Then comes data bytes
			else
			{
				switch(cmd)
				{
					//Readback commands do nothing here
					case FRONT_GET_STATUS:
						break;

					//Schedule a display refresh
					case FRONT_REFRESH_FAST:
						nextDisplayRefresh = 0;
						break;

					case FRONT_REFRESH_FULL:
						nextFullRefresh = 0;
						break;

					//Link speed
					case FRONT_ETH_LINK:

						//If speed changed, trigger a display refresh
						if(g_linkSpeed != data)
							nextDisplayRefresh = 0;

						g_linkSpeed = data;
						break;

					//IPv4 address
					case FRONT_IP4_ADDR:
						if(nbyte <= 4)
						{
							//If IP changed, trigger a display refresh
							if(g_ipv4Addr[nbyte-1] != data)
								nextDisplayRefresh = 0;

							g_ipv4Addr[nbyte-1] = data;
						}
						break;

					//DHCP enable flag
					case FRONT_IPV4_DHCP:
						if(nbyte == 0)
							g_staticIP = nbyte ? true : false;
						break;

					//IPv6 address
					case FRONT_IP6_ADDR:
						if(nbyte <= 16)
						{
							int nword = (nbyte-1)/2;
							int half = (nbyte-1) % 2;

							if(half)
								g_ipv6Addr[nword] |= data;
							else
								g_ipv6Addr[nword] = data << 8;
						}
						break;

					//Serial number
					case FRONT_SERIAL:
						if(nbyte <= 8)
							g_serial[nbyte-1] = data;
						break;

					//Main MCU firmware
					case FRONT_MCU_FW:
						if(RxSPIString(nbyte, g_mcuFirmware, sizeof(g_mcuFirmware), data))
							nextDisplayRefresh = 0;
						break;

					//IBC MCU firmware
					case FRONT_IBC_FW:
						if(RxSPIString(nbyte, g_ibcFirmware, sizeof(g_ibcFirmware), data))
							nextDisplayRefresh = 0;
						break;

					//Supervisor MCU firmware
					case FRONT_SUPER_FW:
						if(RxSPIString(nbyte, g_superFirmware, sizeof(g_superFirmware), data))
							nextDisplayRefresh = 0;
						break;

					//FPGA firmware
					case FRONT_FPGA_FW:
						if(RxSPIString(nbyte, g_fpgaFirmware, sizeof(g_fpgaFirmware), data))
							nextDisplayRefresh = 0;
						break;

					//FPGA die temperature
					case FRONT_FPGA_TEMP:
						if(nbyte == 1)
							g_fpgaTemp = data;
						else if(nbyte == 2)
							g_fpgaTemp |= data << 8;
						break;

					//MCU die temperature
					case FRONT_MCU_TEMP:
						if(nbyte == 1)
							g_mcuTemp = data;
						else if(nbyte == 2)
							g_mcuTemp |= data << 8;
						break;

					//IBC board temperature
					case FRONT_IBC_TEMP:
						if(nbyte == 1)
							g_ibcTemp = data;
						else if(nbyte == 2)
							g_ibcTemp |= data << 8;
						break;

					//IBC input
					case FRONT_IBC_VIN:
						if(nbyte == 1)
							g_vin = data;
						else if(nbyte == 2)
							g_vin |= data << 8;
						break;
					case FRONT_IBC_IIN:
						if(nbyte == 1)
							g_iin = data;
						else if(nbyte == 2)
							g_iin |= data << 8;
						break;

					//IBC output
					case FRONT_IBC_VOUT:
						if(nbyte == 1)
							g_vout = data;
						else if(nbyte == 2)
							g_vout |= data << 8;
						break;
					case FRONT_IBC_IOUT:
						if(nbyte == 1)
							g_iout = data;
						else if(nbyte == 2)
							g_iout |= data << 8;
						break;

					//Fan RPM indicator
					case FRONT_FAN_RPM:
						if(nbyte == 1)
							g_fanspeed = data;
						else if(nbyte == 2)
							g_fanspeed |= data << 8;
						break;

					//Subnet mask
					case FRONT_IP4_SUBNET:
						if(nbyte == 1)
						{
							//If prefix changed, trigger a display refresh
							if(g_ipv4SubnetSize != data)
								nextDisplayRefresh = 0;

							g_ipv4SubnetSize = data;
						}
						break;
					case FRONT_IP6_SUBNET:
						if(nbyte == 1)
							g_ipv6SubnetSize = data;
						else if(nbyte == 2)
							g_ipv6SubnetSize |= data << 8;
						break;

					//Timestamp of sensor values
					case FRONT_TIMESTAMP:
						RxSPIString(nbyte, g_dataTimestamp, sizeof(g_dataTimestamp), data);
						break;

					//Port direction indicator LEDs
					case FRONT_DIR_LEDS:
						*g_outmodeLED[0] = (data & 1) == 1;
						*g_outmodeLED[1] = (data & 2) == 2;
						*g_outmodeLED[2] = (data & 4) == 4;
						*g_outmodeLED[3] = (data & 8) == 8;

						*g_inmodeLED[0] = (data & 0x10) == 0x10;
						*g_inmodeLED[1] = (data & 0x20) == 0x20;
						*g_inmodeLED[2] = (data & 0x40) == 0x40;
						*g_inmodeLED[3] = (data & 0x80) == 0x80;
						break;

					//Port status LEDs
					case FRONT_PORT_LEDS:
						if(nbyte <= 3)
							g_expander->BatchUpdateValue(nbyte-1, data);
						if(nbyte == 3)
							g_expander->BatchCommitValue();
						break;

					default:
						g_log("Unrecognized command %02x\n", cmd);
						break;

				}
			}

			nbyte ++;
		}
	}
}

/**
	@brief Receive a string, return true if value has changed
 */
bool RxSPIString(uint8_t nbyte, char* buf, uint8_t size, uint8_t data)
{
	bool changed = false;
	if(nbyte < size-1)
	{
		//If value changed, trigger a display refresh
		if(g_mcuFirmware[nbyte-1] != data)
			changed = true;

		buf[nbyte-1] = data;
	}
	return changed;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Display interfacing

/**
	@brief Draw stuff on the screen
 */
void RefreshDisplay(bool forceFull)
{
	char tmp[128];
	StringBuffer buf(tmp, sizeof(tmp));

	g_display->Clear();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Main text block

	//Initial constants
	uint8_t xleft = 0;
	uint8_t ytop = 103;
	uint8_t textheight = 8;
	uint8_t textwidth = 6;
	uint8_t xright = 211;

	//Text box extents
	uint8_t textleft = xleft + 2;
	uint8_t textright = textleft + 24*textwidth;
	uint8_t lineright = textright + 2;

	//Line at top of text
	uint8_t texty = ytop;
	g_display->Line(xleft, ytop, lineright, ytop, true);
	texty -= 2;

	//Top row: link speed
	texty -= textheight;
	const char* linkSpeed = "";
	switch(g_linkSpeed)
	{
		case 0:
			linkSpeed = "10M";
			break;

		case 1:
			linkSpeed = "100M";
			break;

		case 2:
			linkSpeed = "1000M";
			break;

		case 3:
			linkSpeed = "10G";
			break;

		default:
			linkSpeed = "DOWN";
			break;
	}
	g_display->Text6x8(textleft, texty, "Link", true);
	uint8_t vleft = 6*textwidth + textleft;
	if(g_linkSpeed > 3)
	{
		g_display->FilledRect(vleft - 1, texty, vleft + 4*textwidth + 1, texty + textheight+1, true);
		g_display->Text6x8(vleft, texty, linkSpeed, false);
	}
	else
		g_display->Text6x8(vleft, texty, linkSpeed, true);

	//DHCP mode
	g_display->Text6x8(textright - 6*textwidth, texty, g_staticIP ? "Static" : "  DHCP", true);

	//IPv4 address and subnet mask
	texty -= textheight;
	buf.Clear();
	buf.Printf("IPv4  %d.%d.%d.%d/%d",
		g_ipv4Addr[0], g_ipv4Addr[1], g_ipv4Addr[2], g_ipv4Addr[3], g_ipv4SubnetSize);
	g_display->Text6x8(textleft, texty, tmp, true);

	//Next rows: IPv6 address
	texty -= textheight;
	buf.Clear();
	buf.Printf("IPv6  %x:%x:%x:%x:", g_ipv6Addr[0], g_ipv6Addr[1], g_ipv6Addr[2], g_ipv6Addr[3]);
	g_display->Text6x8(textleft, texty, tmp, true);

	texty -= textheight;
	buf.Clear();
	buf.Printf("      %x:%x:%x:%x/%d", g_ipv6Addr[4], g_ipv6Addr[5], g_ipv6Addr[6], g_ipv6Addr[7], g_ipv6SubnetSize);
	g_display->Text6x8(textleft, texty, tmp, true);

	//Line between IP info and serial
	g_display->Line(xleft, texty, lineright, texty, true);
	texty -= 2;

	//Serial number
	texty -= textheight;
	buf.Clear();
	buf.Printf("S/N   %02x%02x%02x%02x%02x%02x%02x%02x",
		g_serial[0], g_serial[1], g_serial[2], g_serial[3],
		g_serial[4], g_serial[5], g_serial[6], g_serial[7]);
	g_display->Text6x8(textleft, texty, tmp, true);

	//Line between serial and version
	g_display->Line(xleft, texty, lineright, texty, true);
	texty -= 2;

	//Version info
	texty -= textheight;
	buf.Clear();
	buf.Printf("IBC   %s", g_ibcFirmware);
	g_display->Text6x8(textleft, texty, tmp, true);

	texty -= textheight;
	buf.Clear();
	buf.Printf("Super %s", g_superFirmware);
	g_display->Text6x8(textleft, texty, tmp, true);

	texty -= textheight;
	static const char* buildtime = __TIME__;
	buf.Clear();
	buf.Printf("Panel %s %c%c%c%c%c%c",
		__DATE__, buildtime[0], buildtime[1], buildtime[3], buildtime[4], buildtime[6], buildtime[7]);
	g_display->Text6x8(textleft, texty, tmp, true);

	texty -= textheight;
	buf.Clear();
	buf.Printf("MCU   %s", g_mcuFirmware);
	g_display->Text6x8(textleft, texty, tmp, true);

	texty -= textheight;
	buf.Clear();
	buf.Printf("FPGA  %s", g_fpgaFirmware);
	g_display->Text6x8(textleft, texty, tmp, true);

	//Line below version info
	texty --;
	g_display->Line(xleft, texty, lineright, texty, true);
	texty -= 2;

	//Data timestamp
	texty -= textheight;
	buf.Clear();
	buf.Printf("Updated         %s", g_dataTimestamp);
	g_display->Text6x8(textleft, texty, tmp, true);

	//Final bottom line
	texty --;
	g_display->Line(xleft, texty, lineright, texty, true);

	//Vertical line at left and right of text
	g_display->Line(xleft, ytop, xleft, texty, true);
	g_display->Line(lineright, ytop, lineright, texty, true);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Right area: system health

	texty = ytop;

	//Horizontal line at top
	g_display->Line(lineright, texty, xright, texty, true);
	texty -= 2;

	//IBC input stats
	textleft = 151;
	texty -= textheight;
	buf.Clear();
	buf.Printf("IN %2d.%03dV", g_vin / 1000, g_vin % 1000);
	g_display->Text6x8(textleft, texty, tmp, true);

	texty -= textheight;
	buf.Clear();
	buf.Printf("   %2d.%03dA", g_iin / 1000, g_iin % 1000);
	g_display->Text6x8(textleft, texty, tmp, true);

	texty -= textheight;
	uint32_t pwr = ( (uint32_t)g_vin * (uint32_t)g_iin ) / 1000;	//gives power in mW
	buf.Clear();
	buf.Printf("   %2d.%03dW", pwr / 1000, pwr % 1000);
	g_display->Text6x8(textleft, texty, tmp, true);

	//horizontal line between input and output stats
	g_display->Line(lineright, texty, xright, texty, true);
	texty -= 2;

	//IBC output stats
	texty -= textheight;
	buf.Clear();
	buf.Printf("OP %2d.%03dV", g_vout / 1000, g_vout % 1000);
	g_display->Text6x8(textleft, texty, tmp, true);

	texty -= textheight;
	buf.Clear();
	buf.Printf("   %2d.%03dA", g_iout / 1000, g_iout % 1000);
	g_display->Text6x8(textleft, texty, tmp, true);

	texty -= textheight;
	pwr = ( (uint32_t)g_vout * (uint32_t)g_iout ) / 1000;
	buf.Clear();
	buf.Printf("   %2d.%03dW", pwr / 1000, pwr % 1000);
	g_display->Text6x8(textleft, texty, tmp, true);

	//Horizontal line before temps
	g_display->Line(lineright, texty, xright, texty, true);
	texty -= 2;

	//IBC temperature
	texty -= textheight;
	int degreal = g_ibcTemp >> 8;
	int degfrac = (g_ibcTemp & 0xff) * 10;
	buf.Clear();
	buf.Printf("IBC  %2d.%dC", degreal, degfrac >> 8);
	g_display->Text6x8(textleft, texty, tmp, true);

	texty -= textheight;
	degreal = g_mcuTemp >> 8;
	degfrac = (g_mcuTemp & 0xff) * 10;
	buf.Clear();
	buf.Printf("MCU  %2d.%dC", degreal, degfrac >> 8);
	g_display->Text6x8(textleft, texty, tmp, true);

	texty -= textheight;
	degreal = g_fpgaTemp >> 8;
	degfrac = (g_fpgaTemp & 0xff) * 10;
	buf.Clear();
	buf.Printf("FPGA %2d.%dC", degreal, degfrac >> 8);
	g_display->Text6x8(textleft, texty, tmp, true);

	texty -= textheight;
	auto fronttemp = ReadThermalSensor(g_tempI2cAddress);
	degreal = fronttemp >> 8;
	degfrac = (fronttemp & 0xff) * 10;
	buf.Clear();
	buf.Printf("PANL %2d.%dC", degreal, degfrac >> 8);
	g_display->Text6x8(textleft, texty, tmp, true);

	//Get the fan speed
	texty -= textheight;
	buf.Clear();
	buf.Printf("FAN  %5d", g_fanspeed);
	bool fanSpeedOK = true;
	if( (g_fanspeed < 7000) || (g_fanspeed > 15000) )
		fanSpeedOK = false;
	if(fanSpeedOK)
		g_display->Text6x8(textleft, texty, tmp, true);
	else
	{
		g_display->FilledRect(lineright, texty, xright, texty + textheight + 1, true);
		g_display->Text6x8(textleft, texty, tmp, false);
	}

	//horizontal line below text
	g_display->Line(lineright, texty, xright, texty, true);

	//Vertical line at left and right side
	g_display->Line(lineright, ytop, lineright, texty, true);
	g_display->Line(xright, ytop, xright, texty, true);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Update the display if the main MCU goes down

	if(g_mainMCUDown)
	{
		static const char* err= "MCU or FPGA down";
		g_display->FilledRect(0, 0, strlen(err)*8 + 2, textheight+1, true);
		g_display->Text6x8(1, 0, err, false);
	}

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Done, push the update to the display

	g_display->StartRefresh(forceFull);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sensor interfacing

/**
	@brief Read a temperature sensor at the given I2C address and return the temperature (in 8.8 fixed point format)
 */
uint16_t ReadThermalSensor(uint8_t addr)
{
	if(!g_i2c.BlockingWrite8(addr, 0x00))
		return 0xff;
	uint16_t reply;
	if(!g_i2c.BlockingRead16(addr, reply))
		return 0xff;

	return reply;
}

