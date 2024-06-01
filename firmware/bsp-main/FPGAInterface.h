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
	virtual ~FPGAInterface()
	{}

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

	uint32_t BlockingRead32(uint32_t addr)
	{
		uint32_t data;
		BlockingRead(addr, reinterpret_cast<uint8_t*>(&data), sizeof(data));
		return data;
	}

	uint8_t BlockingRead8(uint32_t addr)
	{
		uint8_t data;
		BlockingRead(addr, reinterpret_cast<uint8_t*>(&data), sizeof(data));
		return data;
	}

	uint16_t BlockingRead16(uint32_t addr)
	{
		uint16_t data;
		BlockingRead(addr, reinterpret_cast<uint8_t*>(&data), sizeof(data));
		return data;
	}

	void BlockingWrite8(uint32_t addr, uint8_t data)
	{ BlockingWrite(addr, &data, sizeof(data)); }

	void BlockingWrite16(uint32_t addr, uint16_t data)
	{ BlockingWrite(addr, reinterpret_cast<uint8_t*>(&data), sizeof(data)); }

	void BlockingWrite32(uint32_t addr, uint32_t data)
	{ BlockingWrite(addr, reinterpret_cast<uint8_t*>(&data), sizeof(data)); }

	void BlockingWrite16(volatile void* addr, uint16_t data)
	{ BlockingWrite16(reinterpret_cast<uint32_t>(addr) & 0xffffff, data); }

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

	//BERT bridge (off large branch, 0x400 per node)
	BASE_BERT_LANE0		= 0x0000'b000,		//APB_BertConfig
	BASE_BERT_LANE1		= 0x0000'b400,		//APB_BertConfig
	BASE_DRP_LANE0		= 0x0000'b800,		//APB_SerdesDRP
	BASE_DRP_LANE1		= 0x0000'bc00		//APB_SerdesDRP
};

struct __attribute__((packed)) APB_SerdesDRP
{
	uint16_t		addr;
	uint16_t		field_02;
	uint16_t		data;
	uint16_t		field_06;
	uint16_t		status;
	uint16_t		field_0a[15];
	uint16_t		status2;
};

struct __attribute__((packed)) APB_BERTConfig
{
	uint16_t		tx_config;
	uint16_t		field_02;
	uint16_t		tx_clk;
	uint16_t		field_06;
	uint16_t		tx_reset;
	uint16_t		field_0a;
	uint16_t		tx_driver;
	uint16_t		field_0e[25];
	uint16_t		rx_config;
	uint16_t		field_42;
	uint16_t		rx_clk;
	uint16_t		field_46;
	uint16_t		rx_reset;
};

struct __attribute__((packed)) APB_SPIHostInterface
{
	uint16_t		clkdiv;
	uint16_t		data;
	uint16_t		cs_n;
	uint16_t		field_06[13];
	uint16_t		status;
	uint16_t		field_22[15];
	uint16_t		status2;
};

struct __attribute__((packed)) APB_GPIO
{
	uint32_t		out;
	uint32_t		in;
	uint32_t		tris;
};

struct __attribute__((packed)) APB_SystemInfo
{
	uint32_t		idcode;
	uint8_t			serial[8];
	uint32_t		field_0c;
	uint16_t		fan_rpm[2];
	uint16_t		die_temp;
	uint16_t		voltage_core;
	uint16_t		voltage_ram;
	uint16_t		voltage_aux;
	uint32_t		usercode;
};

struct __attribute__((packed)) APB_RelayController
{
	uint16_t		toggle;
	uint16_t		field_02[15];
	uint16_t		stat;
	uint16_t		field_22[15];
	uint16_t		stat2;
};

struct __attribute__((packed)) APB_MDIO
{
	uint16_t		cmd_addr;
	uint16_t		field_02[3];
	uint16_t		data;
	uint16_t		field_04[27];
	uint16_t		status;
	uint16_t		field_22[15];
	uint16_t		status2;
};

struct __attribute__((packed)) ManagementRxFifo
{
	uint8_t			rx_buf[4088];
	uint16_t		rx_pop;
	uint16_t		field_ffa;
	uint16_t		rx_len;
};

struct __attribute__((packed)) ManagementTxFifo
{
	uint16_t		tx_stat;
	uint16_t		field_02[3];
	uint64_t		tx_commit;
	uint64_t		tx_len;
	uint16_t		field_14[20];
	uint8_t			tx_buf[4032];
};

struct __attribute__((packed)) APB_Curve25519
{
	uint8_t			e[32];
	uint16_t		status;
	uint16_t		field_22;
	uint16_t		rd_addr;
	uint16_t		field_26;
	uint16_t		cmd;
	uint16_t		field_2a[11];
	uint16_t		status2;
	uint16_t		field_42[15];
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
