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

extern uint32_t secSinceLastMcuUpdate;
extern uint32_t nextDisplayRefresh;
extern uint32_t nextFullRefresh;

void RefreshDisplay(bool forceFull);

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
