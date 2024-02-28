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
	@brief	PHY control code
 */

#include "triggercrossbar.h"

/**
	@brief Reads a register from the management PHY
 */
uint16_t ManagementPHYRead(uint8_t regid)
{
	//Request the read
	g_fpga->BlockingWrite32(REG_MGMT0_MDIO, (regid << 16) | 0x20000000);

	//Poll until busy flag is cleared
	while(true)
	{
		auto reply = g_fpga->BlockingRead32(REG_MGMT0_MDIO);
		if( (reply & 0x80000000) == 0)
			return reply & 0xffff;
	}
}

void ManagementPHYWrite(uint8_t regid, uint16_t regval)
{
	//Request the write
	g_fpga->BlockingWrite32(REG_MGMT0_MDIO, (regid << 16) | 0x40000000 | regval );

	//Poll until busy flag is cleared
	while(true)
	{
		auto reply = g_fpga->BlockingRead32(REG_MGMT0_MDIO);
		if( (reply & 0x80000000) == 0)
			return;
	}
}

/**
	@brief Reads an extended register from the management PHY
 */
uint16_t ManagementPHYExtendedRead(uint8_t mmd, uint8_t regid)
{
	ManagementPHYWrite(REG_PHY_REGCR, mmd);			//set address
	ManagementPHYWrite(REG_PHY_ADDAR, regid);
	ManagementPHYWrite(REG_PHY_REGCR, 0x4000 | mmd);	//data, no post inc
	return ManagementPHYRead(REG_PHY_ADDAR);
}

/**
	@brief Writes an extended register to the management PHY
 */
void ManagementPHYExtendedWrite(uint8_t mmd, uint8_t regid, uint16_t regval)
{
	ManagementPHYWrite(REG_PHY_REGCR, mmd);				//set address
	ManagementPHYWrite(REG_PHY_ADDAR, regid);
	ManagementPHYWrite(REG_PHY_REGCR, 0x4000 | mmd);	//data, no post inc
	ManagementPHYWrite(REG_PHY_ADDAR, regval);
}

/**
	@brief Poll the PHYs for link state changes

	TODO: use IRQ pin to trigger this vs doing it nonstop?
 */
/*
void PollPHYs()
{
	for(int i=0; i<NUM_PORTS; i++)
	{
		//if port is not up or down, ignore updates
		//(it's administratively down, errdisable, or similar)
		if( (g_linkState[i] != LINK_STATE_UP) && (g_linkState[i] != LINK_STATE_DOWN) )
			continue;

		if(i == UPLINK_PORT)
		{
			uint32_t status = g_fpga->BlockingRead32(REG_XG0_STAT);
			if(status & 1)
			{
				//Link went up?
				if(g_linkState[i] != LINK_STATE_UP)
				{
					g_linkState[i] = LINK_STATE_UP;
					g_linkSpeed[i] = LINK_SPEED_10G;

					g_log("Interface %s (%s): link is up at %s\n",
						g_interfaceNames[i],
						g_interfaceDescriptions[i],
						g_linkSpeedNamesLong[g_linkSpeed[i]]);
				}
			}

			else
			{
				//Link went down?
				if(g_linkState[i] != LINK_STATE_DOWN)
				{
					g_linkState[i] = LINK_STATE_DOWN;
					g_log("Interface %s (%s): link is down\n", g_interfaceNames[i], g_interfaceDescriptions[i]);
				}
			}
		}

		else
		{
			uint16_t bctl = InterfacePHYRead(i, REG_BASIC_CONTROL);
			uint16_t bstat = InterfacePHYRead(i, REG_BASIC_STATUS);
			UpdateLinkState(i, bctl, bstat);
		}
	}
}
*/
