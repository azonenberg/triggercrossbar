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

#ifndef supervisor_h
#define supervisor_h

#include <supervisor/supervisor-common.h>
#include <supervisor/PowerResetSupervisor.h>

//#include <bootloader/BootloaderAPI.h>
#include "../bsp/hwinit.h"

uint16_t Get12VRailVoltage();

///@brief Rail descriptor for the 12V rail using ADC instead of PGOOD
class RailDescriptor12V0 : public RailDescriptorWithEnable
{
public:
	RailDescriptor12V0(const char* name, GPIOPin& enable, Timer& timer, uint16_t timeout)
		: RailDescriptorWithEnable(name, enable, timer, timeout)
	{}

	virtual bool TurnOn() override
	{
		g_log("Turning on %s\n", m_name);

		m_enable = 1;

		for(uint32_t i=0; i<m_delay; i++)
		{
			if(IsPowerGood())
				return true;
			m_timer.Sleep(1);
		}

		if(!IsPowerGood())
		{
			g_log(Logger::ERROR, "Rail %s failed to come up\n", m_name);
			return false;
		}
		return true;
	}

	virtual bool IsPowerGood() override
	{
		auto v12 = Get12VRailVoltage();
		return (v12 <= 13000) && (v12 >= 11000);
	}

	//loss of 12V0 rail should not trigger a panic shutdown
	//(see https://github.com/azonenberg/triggercrossbar/issues/22)
	virtual bool IsCritical() override
	{ return false; }

	//this is the input supply until we add a RailDescriptor for the 48V rail
	virtual bool IsInputSupply() override
	{ return true; }
};

#include "CrossbarPowerResetSupervisor.h"
extern CrossbarPowerResetSupervisor g_super;

#endif
