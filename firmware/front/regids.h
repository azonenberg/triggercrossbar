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
#ifndef front_regids_h
#define front_regids_h

enum front_regid_t
{
	FRONT_ETH_LINK		= 0x00,	//0 = 10M
								//1 = 100M
								//2 = 1G
								//3 = 10G
								//ff = down

	FRONT_IP4_ADDR		= 0x01,	//IPv4 address
	FRONT_IP6_ADDR		= 0x02,	//IPv6 address
	FRONT_SERIAL		= 0x03,	//FPGA serial number (used as system s/n for now... but not 100% reliable as DNA values can have duplicates)
	FRONT_MCU_FW		= 0x04,	//MCU firmware revision
	FRONT_IBC_FW		= 0x05,	//IBC firmware revision
	FRONT_SUPER_FW		= 0x06,	//Supervisor firmware revision
	FRONT_FPGA_FW		= 0x07,	//FPGA firmware revision

	FRONT_FPGA_TEMP		= 0x10,	//FPGA die temperature
	FRONT_MCU_TEMP		= 0x11,	//MCU die temperature
	FRONT_IBC_TEMP		= 0x12,	//IBC board temperature
	FRONT_FAN_RPM		= 0x13,	//Fan RPM

	FRONT_IBC_VIN		= 0x20,	//IBC input voltage
	FRONT_IBC_IIN		= 0x21,	//IBC input power
	FRONT_IBC_VOUT		= 0x22,	//IBC output voltage
	FRONT_IBC_IOUT		= 0x23,	//IBC output power

	FRONT_DIR_LEDS		= 0x30,	//Direction LEDS
								//High nibble = IN11:8
								//Low nibble = OUT11:8
	FRONT_PORT_LEDS		= 0x31,	//All port LED states

	FRONT_REFRESH		= 0xff
};

#endif
