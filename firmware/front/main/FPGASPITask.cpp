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
#include "FPGASPITask.h"
#include "regids.h"

//fixme
extern uint32_t secSinceLastMcuUpdate;
extern uint32_t nextDisplayRefresh;
extern uint32_t nextFullRefresh;

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

void FPGASPITask::OnDataByte(uint8_t data)
{
	//If main MCU was down, it's now up again
	secSinceLastMcuUpdate = 0;
	if(g_mainMCUDown)
	{
		g_mainMCUDown = false;
		nextDisplayRefresh = 1;
	}

	//First byte is command
	if(m_nbyte == 0)
	{
		m_command = data;

		switch(m_command)
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
		switch(m_command)
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
				if(m_nbyte <= 4)
				{
					//If IP changed, trigger a display refresh
					if(g_ipv4Addr[m_nbyte-1] != data)
						nextDisplayRefresh = 0;

					g_ipv4Addr[m_nbyte-1] = data;
				}
				break;

			//DHCP enable flag
			case FRONT_IPV4_DHCP:
				if(m_nbyte == 0)
					g_staticIP = m_nbyte ? true : false;
				break;

			//IPv6 address
			case FRONT_IP6_ADDR:
				if(m_nbyte <= 16)
				{
					int nword = (m_nbyte-1)/2;
					int half = (m_nbyte-1) % 2;

					if(half)
						g_ipv6Addr[nword] |= data;
					else
						g_ipv6Addr[nword] = data << 8;
				}
				break;

			//Serial number
			case FRONT_SERIAL:
				if(m_nbyte <= 8)
					g_serial[m_nbyte-1] = data;
				break;

			//Main MCU firmware
			case FRONT_MCU_FW:
				if(RxSPIString(m_nbyte, g_mcuFirmware, sizeof(g_mcuFirmware), data))
					nextDisplayRefresh = 0;
				break;

			//IBC MCU firmware
			case FRONT_IBC_FW:
				if(RxSPIString(m_nbyte, g_ibcFirmware, sizeof(g_ibcFirmware), data))
					nextDisplayRefresh = 0;
				break;

			//Supervisor MCU firmware
			case FRONT_SUPER_FW:
				if(RxSPIString(m_nbyte, g_superFirmware, sizeof(g_superFirmware), data))
					nextDisplayRefresh = 0;
				break;

			//FPGA firmware
			case FRONT_FPGA_FW:
				if(RxSPIString(m_nbyte, g_fpgaFirmware, sizeof(g_fpgaFirmware), data))
					nextDisplayRefresh = 0;
				break;

			//FPGA die temperature
			case FRONT_FPGA_TEMP:
				if(m_nbyte == 1)
					g_fpgaTemp = data;
				else if(m_nbyte == 2)
					g_fpgaTemp |= data << 8;
				break;

			//MCU die temperature
			case FRONT_MCU_TEMP:
				if(m_nbyte == 1)
					g_mcuTemp = data;
				else if(m_nbyte == 2)
					g_mcuTemp |= data << 8;
				break;

			//IBC board temperature
			case FRONT_IBC_TEMP:
				if(m_nbyte == 1)
					g_ibcTemp = data;
				else if(m_nbyte == 2)
					g_ibcTemp |= data << 8;
				break;

			//IBC input
			case FRONT_IBC_VIN:
				if(m_nbyte == 1)
					g_vin = data;
				else if(m_nbyte == 2)
					g_vin |= data << 8;
				break;
			case FRONT_IBC_IIN:
				if(m_nbyte == 1)
					g_iin = data;
				else if(m_nbyte == 2)
					g_iin |= data << 8;
				break;

			//IBC output
			case FRONT_IBC_VOUT:
				if(m_nbyte == 1)
					g_vout = data;
				else if(m_nbyte == 2)
					g_vout |= data << 8;
				break;
			case FRONT_IBC_IOUT:
				if(m_nbyte == 1)
					g_iout = data;
				else if(m_nbyte == 2)
					g_iout |= data << 8;
				break;

			//Fan RPM indicator
			case FRONT_FAN_RPM:
				if(m_nbyte == 1)
					g_fanspeed = data;
				else if(m_nbyte == 2)
					g_fanspeed |= data << 8;
				break;

			//Subnet mask
			case FRONT_IP4_SUBNET:
				if(m_nbyte == 1)
				{
					//If prefix changed, trigger a display refresh
					if(g_ipv4SubnetSize != data)
						nextDisplayRefresh = 0;

					g_ipv4SubnetSize = data;
				}
				break;
			case FRONT_IP6_SUBNET:
				if(m_nbyte == 1)
					g_ipv6SubnetSize = data;
				else if(m_nbyte == 2)
					g_ipv6SubnetSize |= data << 8;
				break;

			//Timestamp of sensor values
			case FRONT_TIMESTAMP:
				RxSPIString(m_nbyte, g_dataTimestamp, sizeof(g_dataTimestamp), data);
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
				if(m_nbyte <= 3)
					g_expander->BatchUpdateValue(m_nbyte-1, data);
				if(m_nbyte == 3)
					g_expander->BatchCommitValue();
				break;

			default:
				g_log("Unrecognized command %02x\n", m_command);
				break;

		}
	}
}

/**
	@brief Receive a string, return true if value has changed
 */
bool FPGASPITask::RxSPIString(uint8_t nbyte, char* buf, uint8_t size, uint8_t data)
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
