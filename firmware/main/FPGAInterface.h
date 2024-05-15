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
	BASE_SYSINFO		= 0x0000'0000,		//APB_SystemInfo
	BASE_MDIO			= 0x0000'0800,		//APB_MDIO
	BASE_RELAY			= 0x0000'1000,		//APB_RelayController
	BASE_FRONT_SPI		= 0x0000'1800,		//APB_SPIHostInterface
	BASE_IN_LED_GPIO	= 0x0000'2000,		//APB_GPIO
	BASE_OUT_LED_GPIO	= 0x0000'2800,		//APB_GPIO
	BASE_MUXSEL			= 0x0000'3000,		//APB_CrossbarMatrix
	BASE_BERT_LANE0		= 0x0000'3800,		//APB_BertConfig
	BASE_BERT_LANE1		= 0x0000'4000,		//APB_BertConfig
	BASE_25519			= 0x0000'4800,		//APB_Curve25519
	BASE_XG_TX			= 0x0000'5000		//Management10GTxFifo
};

enum regid_t
{
	//This section is now on APB
	//(must match register IDs in corresponding module)

	//APB_SystemInfo
	REG_FPGA_IDCODE		= 0x0000,
	REG_FPGA_SERIAL		= 0x0004,
	REG_FAN0_RPM		= 0x0010,
	REG_FAN1_RPM		= 0x0012,
	REG_DIE_TEMP		= 0x0014,
	REG_VOLT_CORE		= 0x0016,
	REG_VOLT_RAM		= 0x0018,
	REG_VOLT_AUX		= 0x001a,
	REG_USERCODE		= 0x001c,

	//APB_MDIO
	REG_MDIO_CMD_ADDR	= 0x0000,
	REG_MDIO_DATA		= 0x0002,
	REG_MDIO_STATUS		= 0x0020,
	REG_MDIO_STATUS2	= 0x0040,

	//APB_RelayController
	REG_RELAY_TOGGLE	= 0x0000,
	REG_RELAY_STAT		= 0x0020,
	REG_RELAY_STAT2		= 0x0040,

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
	REG_CRYPT_STATUS2	= 0x0040,
	REG_CRYPT_WORK		= 0x0060,
	REG_CRYPT_Q_0		= 0x0080,
	REG_CRYPT_Q_1		= 0x00a0,
	REG_CRYPT_BASE_Q_0	= 0x0100,
	REG_CRYPT_DATA_OUT	= 0x0140,

	//Management10GTxFifo
	REG_XG_TX_STAT		= 0x0000,
	REG_XG_TX_COMMIT	= 0x0004,
	REG_XG_TX_BUF		= 0x0008,

	//everything below here is still on the legacy bus
	//must match regid_t in ManagementRegisterInterface.sv
	REG_FPGA_IRQSTAT	= 0x0020,
	REG_EMAC_RXLEN		= 0x0024,
	REG_EMAC_COMMIT		= 0x0028,

	REG_BERT_LANE0_WD	= 0x0084,
	REG_BERT_LANE0_AD	= 0x0086,
	REG_BERT_LANE0_RD	= 0x0088,
	REG_BERT_LANE0_STAT	= 0x008a,

	REG_BERT_LANE1_WD	= 0x00a4,
	REG_BERT_LANE1_AD	= 0x00a6,
	REG_BERT_LANE1_RD	= 0x00a8,
	REG_BERT_LANE1_STAT	= 0x00aa,

	REG_EMAC_BUFFER		= 0x1000
};

#define BERT_LANE_STRIDE 0x20

#endif
