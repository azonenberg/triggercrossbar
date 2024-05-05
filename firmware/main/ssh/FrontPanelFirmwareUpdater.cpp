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
#include "FrontPanelFirmwareUpdater.h"
#include "../../front/regids.h"

bool g_frontPanelDFUInProgress = false;

bool IsFrontPanelDFU()
{ return g_frontPanelDFUInProgress; }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction / destruction

FrontPanelFirmwareUpdater::FrontPanelFirmwareUpdater()
{
}

FrontPanelFirmwareUpdater::~FrontPanelFirmwareUpdater()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Firmware update logic

void FrontPanelFirmwareUpdater::StartUpdate()
{
	g_log("Starting DFU of front panel\n");
	LogIndenter li(g_log);

	g_frontPanelDFUInProgress = true;

	//Put the front panel in DFU mode if it's not already
	auto mode = GetFrontPanelMode();
	switch(mode)
	{
		case FRONT_NORMAL:
		{
			g_log("Front panel is in normal mode, restarting in DFU mode...\n");

			//Go to DFU mode
			SetFrontPanelCS(0);
			SendFrontPanelByte(FRONT_ENTER_DFU);
			SetFrontPanelCS(1);

			//Wait for reset
			g_logTimer->Sleep(500);

			//Make sure we're back up in DFU mode
			mode = GetFrontPanelMode();
			if(mode != FRONT_BOOTLOADER)
			{
				g_log(Logger::ERROR, "Front panel is not in bootloader mode (expected mode 0x%02x, got %02x)\n",
					FRONT_BOOTLOADER,
					mode);

				m_state = STATE_FAILED;
				return;
			}
		}
		break;

		case FRONT_BOOTLOADER:
			g_log("Front panel is already in DFU mode, no action needed at this time\n");
			break;

		default:
			g_log(Logger::ERROR, "Front panel is not in a valid mode (got %02x)\n", mode);
			break;
	}

	//TODO: erase the application flash partition

}

void FrontPanelFirmwareUpdater::OnWriteData(uint32_t physicalAddress, uint8_t* data, uint32_t len)
{
	g_log("OnWriteData phyaddr=0x%08x len=%x\n", physicalAddress, len);
}

void FrontPanelFirmwareUpdater::FinishUpdate()
{
	g_log("DFU complete, rebooting MCU\n");
	LogIndenter li(g_log);

	//Request booting the app
	SetFrontPanelCS(0);
	SendFrontPanelByte(FRONT_BOOT_APP);
	SetFrontPanelCS(1);

	//Wait for reset
	//TODO: we don't want to hang the whole chip for 1 sec, do this in some kind of timer state machine
	g_logTimer->Sleep(10000);

	//Make sure we're back up in application mode
	auto mode = GetFrontPanelMode();
	switch(mode)
	{
		case FRONT_NORMAL:
			g_log("Update successful\n");
			g_frontPanelDFUInProgress = false;
			break;

		case FRONT_BOOTLOADER:
			g_log(Logger::ERROR, "Front panel is still in DFU mode, something went wrong\n");
			break;

		default:
			g_log(Logger::ERROR, "Front panel is not in a valid mode (got %02x)\n", mode);
			break;
	}
}
