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

#ifndef fpgainterface_h
#define fpgainterface_h

class FPGAInterface
{
public:
	virtual void BlockingWrite(uint32_t addr, const uint8_t* data, uint32_t len) = 0;

	void BlockingWriteN(volatile void* addr, const void* data, uint32_t len)
	{ BlockingWrite(reinterpret_cast<uint32_t>(const_cast<void*>(addr)) & 0xffffff, (const uint8_t*)data, len); }
};

enum baseaddr_t
{
	BASE_CDRTRIG_LANE0	= 0x0000'b600,		//CDRTrigger
	BASE_CDRTRIG_LANE1	= 0x0000'b700		//CDRTrigger
};

struct LogicAnalyzer
{
	uint32_t		trigger;
	uint32_t		field_04[7];
	uint32_t		buf_addr;
	uint32_t		field_24[7];
	uint32_t		buf_size;
	uint32_t		field_44[7];
	uint32_t		trig_offset;
	uint32_t		field_64[7];
	uint32_t		rx_buf[32];
};

struct APB_BERTConfig
{
	uint32_t		tx_config;
	uint32_t		field_04[7];
	uint32_t		tx_clk;
	uint32_t		field_24[7];
	uint32_t		tx_reset;
	uint32_t		field_44[7];
	uint32_t		tx_driver;
	uint32_t		field_64[7];
	uint32_t		rx_config;
	uint32_t		field_84[7];
	uint32_t		rx_clk;
	uint32_t		field_a4[7];
	uint32_t		rx_reset;
};

#include <APB_SPIHostInterface.h>
#include <APB_MDIO.h>
#include <APB_GPIO.h>
#include <APB_SerdesDRP.h>

struct APB_SystemInfo
{
	uint32_t		idcode;
	uint8_t			serial[8];
	uint32_t		fan_rpm[2];
	uint32_t		die_temp;
	uint32_t		voltage_core;
	uint32_t		voltage_ram;
	uint32_t		voltage_aux;
	uint32_t		usercode;
};

struct APB_RelayController
{
	uint32_t		toggle;
	uint32_t		field_04[7];
	uint32_t		stat;
	uint32_t		field_24[7];
	uint32_t		stat2;
};

struct APB_CrossbarChannel
{
	uint32_t		muxsel;
	uint32_t		field_04[7];
};

struct APB_CrossbarMatrix
{
	APB_CrossbarChannel		channels[14];
};

#include <APB_EthernetRxBuffer.h>
#include <APB_EthernetTxBuffer_10G.h>

#define FPGA_MEM_BASE 0x9000'0000

#endif
