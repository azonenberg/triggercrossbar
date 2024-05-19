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

	//BERT bridge (0x100 per node)
	BASE_BERT_LANE0		= 0x0000'2000,		//APB_BertConfig
	BASE_BERT_LANE1		= 0x0000'2100,		//APB_BertConfig
	BASE_DRP_LANE0		= 0x0000'2200,		//APB_SerdesDRP
	BASE_DRP_LANE1		= 0x0000'2300,		//APB_SerdesDRP

	//More small stuff
	BASE_IRQ_STAT		= 0x0000'2400,		//APB_StatusRegister

	//Root bridge, large-address branch (0x1000 per node)
	BASE_XG_TX			= 0x0000'8000,		//Management10GTxFifo
	BASE_1G_TX			= 0x0000'9000,		//ManagementTxFifo
	BASE_ETH_RX			= 0x0000'a000		//ManagementRxFifo
};

enum regid_t
{
	//APB register IDs
	//(must match register IDs in corresponding module)

	//APB_SPIHostInterface
	REG_SPI_CLK_DIV		= 0x0000,
	REG_SPI_DATA		= 0x0002,
	REG_SPI_CS_N		= 0x0004,
	REG_SPI_STATUS		= 0x0020,
	REG_SPI_STATUS2		= 0x0040,

	//APB_GPIO
	REG_GPIO_OUT		= 0x0000,
	REG_GPIO_IN			= 0x0004,
	REG_GPIO_TRIS		= 0x0008,

	//APB_BertConfig
	REG_TX_CONFIG		= 0x0000,
	REG_TX_CLK			= 0x0004,
	REG_TX_RESET		= 0x0008,
	REG_TX_DRIVER		= 0x000c,
	REG_RX_CONFIG		= 0x0040,
	REG_RX_CLK			= 0x0044,
	REG_RX_RESET		= 0x0048,

	//APB_Curve25519
	REG_CRYPT_E			= 0x0000,
	REG_CRYPT_STATUS	= 0x0020,
	REG_CRYPT_RD_ADDR	= 0x0024,
	REG_CRYPT_CMD		= 0x0028,
	REG_CRYPT_STATUS2	= 0x0040,
	REG_CRYPT_WORK		= 0x0060,
	REG_CRYPT_Q_0		= 0x0080,
	REG_CRYPT_Q_1		= 0x00a0,
	REG_CRYPT_BASE_Q_0	= 0x0100,
	REG_CRYPT_DATA_OUT	= 0x0140,

	//APB_SerdesDRP
	REG_DRP_ADDR		= 0x0000,
	REG_DRP_DATA		= 0x0004,
	REG_DRP_STATUS		= 0x0008,
	REG_DRP_STATUS_2	= 0x0028,

	//Management10GTxFifo / ManagementTxFifo
	REG_ETH_TX_BUF		= 0x0040
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
	uint16_t		tx_commit;
	uint16_t		field_0a[27];
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
