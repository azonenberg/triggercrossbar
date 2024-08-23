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
	virtual void Nop()
	{};

	#ifdef SIMULATION
	/**
		@brief Advance simulation time until the crypto engine has finished
	 */
	virtual void CryptoEngineBlock()
	{}
	#endif

	virtual void BlockingRead(uint32_t addr, uint8_t* data, uint32_t len) = 0;
	virtual void BlockingWrite(uint32_t addr, const uint8_t* data, uint32_t len) = 0;

	void BlockingWrite32(uint32_t addr, uint32_t data)
	{ BlockingWrite(addr, reinterpret_cast<uint8_t*>(&data), sizeof(data)); }

	void BlockingWrite32(volatile void* addr, uint32_t data)
	{ BlockingWrite32(reinterpret_cast<uint32_t>(addr) & 0xffffff, data); }

	void BlockingWriteN(volatile void* addr, const void* data, uint32_t len)
	{ BlockingWrite(reinterpret_cast<uint32_t>(const_cast<void*>(addr)) & 0xffffff, (const uint8_t*)data, len); }
};

enum baseaddr_t
{
	//Root bridge, small-address branch (0x400 per node)
	BASE_SYSINFO		= 0x0000'0000,		//APB_SystemInfo
	BASE_IN_LED_GPIO	= 0x0000'0400,		//APB_GPIO
	BASE_OUT_LED_GPIO	= 0x0000'0800,		//APB_GPIO
	BASE_MDIO			= 0x0000'0c00,		//APB_MDIO
	BASE_RELAY			= 0x0000'1000,		//APB_RelayController
	BASE_FRONT_SPI		= 0x0000'1400,		//APB_SPIHostInterface
	BASE_MUXSEL			= 0x0000'1800,		//APB_CrossbarMatrix
	BASE_25519			= 0x0000'1c00,		//APB_Curve25519
	BASE_IRQ_STAT		= 0x0000'2000,		//APB_StatusRegister
	BASE_FLASH_SPI		= 0x0000'2400,		//APB_SPIHostInterface

	//Root bridge, large-address branch (0x1000 per node)
	BASE_XG_TX			= 0x0000'8000,		//Management10GTxFifo
	BASE_1G_TX			= 0x0000'9000,		//ManagementTxFifo
	BASE_ETH_RX			= 0x0000'a000,		//ManagementRxFifo

	//BERT bridge (off large branch, 0x100 per node)
	BASE_BERT_LANE0		= 0x0000'b000,		//APB_BertConfig
	BASE_BERT_LANE1		= 0x0000'b100,		//APB_BertConfig
	BASE_DRP_LANE0		= 0x0000'b200,		//APB_SerdesDRP
	BASE_DRP_LANE1		= 0x0000'b300,		//APB_SerdesDRP
	BASE_LA_LANE0		= 0x0000'b400,		//LogicAnalyzer
	BASE_LA_LANE1		= 0x0000'b500,		//LogicAnalyzer
	BASE_CDRTRIG_LANE0	= 0x0000'b600,		//CDRTrigger
	BASE_CDRTRIG_LANE1	= 0x0000'b700		//CDRTrigger
};

struct __attribute__((packed)) LogicAnalyzer
{
	uint32_t		trigger;
	uint32_t		buf_addr;
	uint32_t		buf_size;
	uint32_t		trig_offset;
	uint32_t		field_10[4];
	uint32_t		rx_buf[16];
};

struct __attribute__((packed)) APB_SerdesDRP
{
	uint32_t		addr;
	uint32_t		data;
	uint32_t		status;
	uint32_t		field_0a[7];
	uint32_t		status2;
};

struct __attribute__((packed)) APB_BERTConfig
{
	uint32_t		tx_config;
	uint32_t		tx_clk;
	uint32_t		tx_reset;
	uint32_t		tx_driver;
	uint32_t		field_10[12];
	uint32_t		rx_config;
	uint32_t		rx_clk;
	uint32_t		rx_reset;
};

#include <APB_SPIHostInterface.h>
#include <APB_MDIO.h>

struct __attribute__((packed)) APB_GPIO
{
	uint32_t		out;
	uint32_t		in;
	uint32_t		tris;
};

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

struct __attribute__((packed)) APB_RelayController
{
	uint32_t		toggle;
	uint32_t		field_04[7];
	uint32_t		stat;
	uint32_t		field_24[7];
	uint32_t		stat2;
};

#include <APB_EthernetRxBuffer.h>
#include <APB_EthernetTxBuffer_10G.h>

struct __attribute__((packed)) APB_Curve25519
{
	uint8_t			e[32];
	uint32_t		status;
	uint32_t		rd_addr;
	uint32_t		cmd;
	uint32_t		field_2a[5];
	uint32_t		status2;
	uint32_t		field_42[7];
	uint8_t			work[32];
	uint8_t			q0[32];
	uint8_t			q1[32];
	uint8_t			field_c0[32];
	uint8_t			field_e0[32];
	uint8_t			base_q0[32];
	uint8_t			field_120[32];
	uint8_t			data_out[32];
};

enum crypt_cmd_t
{
	CMD_CRYPTO_SCALARMULT = 1
};

#define FPGA_MEM_BASE 0x9000'0000

#endif
