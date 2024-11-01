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

#ifndef CrossbarPowerResetSupervisor_h
#define CrossbarPowerResetSupervisor_h

///@brief Project-specific supervisor class with hooks for controlling LEDs on panic
class CrossbarPowerResetSupervisor : public PowerResetSupervisor
{
public:
	CrossbarPowerResetSupervisor(etl::ivector<RailDescriptor*>& rails, etl::ivector<ResetDescriptor*>& resets)
	: PowerResetSupervisor(rails, resets)
	, m_printPending(false)
	{}

	virtual void Iteration() override
	{
		PowerResetSupervisor::Iteration();

		if(PollIBCSensors())
			PrintIfPending();
	}

protected:

	virtual void OnPowerOn() override
	{ m_printPending = true; }

	virtual void OnFault() override
	{
		//Set LEDs to fault state
		g_faultLED = 1;
		g_sysokLED = 0;
		//we have no pgood LED

		//Hang until reset, don't attempt to auto restart
		while(1)
		{}
	}

	void PrintIfPending()
	{
		if(m_printPending)
		{
			m_printPending = false;

			{
				g_log("IBC status\n");
				LogIndenter li(g_log);
				g_log("Temperature = %uhk C\n", g_ibcTemp);
				g_log("vin         = %2d.%03d V\n", g_vin48 / 1000, g_vin48 % 1000);
				g_log("vout        = %2d.%03d V\n", g_vout12 / 1000, g_vout12 % 1000);
				g_log("vsense      = %2d.%03d V\n", g_voutsense / 1000, g_voutsense % 1000);
				g_log("iin         = %2d.%03d A\n", g_iin / 1000, g_iin % 1000);
				g_log("iout        = %2d.%03d A\n", g_iout / 1000, g_iout % 1000);
			}

			auto v = Get12VRailVoltage();
			g_log("Local 12V0      = %2d.%03d V\n", v / 1000, v % 1000);
		}
	}

	bool m_printPending;
};

#endif
