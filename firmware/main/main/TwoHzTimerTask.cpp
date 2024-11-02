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
#include "TwoHzTimerTask.h"
#include <peripheral/ITMStream.h>
#include <supervisor/SupervisorSPIRegisters.h>

void TwoHzTimerTask::OnTimer()
{
	#ifdef _DEBUG
	static ITMStream sensorStream(0);

	sensorStream.Printf(
		"CSV-NAME,"
		"FAN0_RPM,"
		"FPGA_TEMP,FPGA_VCCINT,FPGA_VCCBRAM,FPGA_VCCAUX,"
		"MAINMCU_TEMP,"
		"SFP_TEMP,"
		"IBC_VIN,IBC_IIN,IBC_TEMP,IBC_VOUT,IBC_IOUT,IBC_VSENSE,"
		"SUPER_MCUTEMP,SUPER_3V3,"
		"SFP_3V3"

		//TODO: IBC_MCUTEMP, IBC_3V3

		"\n"
		);

	sensorStream.Printf(
		"CSV-UNIT,"
		"RPM,"
		"°C,V,V,V,"
		"°C,"
		"°C,"
		"V,A,°C,V,A,V,"
		"°C,V,"
		"V"
		"\n"
		);

	auto ibc_vin = SupervisorRegRead(SUPER_REG_IBCVIN);
	auto ibc_iin = SupervisorRegRead(SUPER_REG_IBCIIN);
	auto ibc_temp = SupervisorRegRead(SUPER_REG_IBCTEMP);
	auto ibc_vout = SupervisorRegRead(SUPER_REG_IBCVOUT);
	auto ibc_iout = SupervisorRegRead(SUPER_REG_IBCIOUT);
	auto ibc_vsense = SupervisorRegRead(SUPER_REG_IBCVSENSE);
	auto super_temp = SupervisorRegRead(SUPER_REG_MCUTEMP);
	auto super_3v3 = SupervisorRegRead(SUPER_REG_3V3);
	auto sfp_3v3 = GetSFP3V3();

	sensorStream.Printf(
		"CSV-DATA,"
		"%d,"
		"%uhk,%uhk,%uhk,%uhk,"
		"%uhk,"
		"%uhk,"
		"%d.%03d,%d.%03d,%uhk,%d.%03d,%d.%03d,%d.%03d,"
		"%uhk,%d.%03d,"
		"%d.%03d"
		"\n",

		GetFanRPM(0),
		GetFPGATemperature(), GetFPGAVCCINT(), GetFPGAVCCBRAM(), GetFPGAVCCAUX(),
		g_dts->GetTemperature(),
		GetSFPTemperature(),
		ibc_vin / 1000, ibc_vin % 1000, ibc_iin / 1000, ibc_iin % 1000, ibc_temp, ibc_vout / 1000, ibc_vout % 1000,
			ibc_iout / 1000, ibc_iout % 1000, ibc_vsense / 1000, ibc_vsense % 1000,
		super_temp, super_3v3 / 1000, super_3v3 % 1000,
		sfp_3v3 / 1000, sfp_3v3 % 1000
		);
	#endif
}
