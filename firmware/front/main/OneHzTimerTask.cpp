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
#include "OneHzTimerTask.h"

//globals that need to be fixed
uint32_t secSinceLastMcuUpdate = 0;
uint32_t nextDisplayRefresh = 30;
uint32_t nextFullRefresh = 5;		//Wait 5 sec after power up to do the first full refresh,
									//so main FPGA and MCU have a chance to come up

void OneHzTimerTask::OnTimer()
{
	const uint32_t displayRefreshInterval = 3600;						//1 hour
	const uint32_t fullRefreshInterval = displayRefreshInterval * 24;	//1 day

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

/**
	@brief Draw stuff on the screen
 */
void OneHzTimerTask::RefreshDisplay(bool forceFull)
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
