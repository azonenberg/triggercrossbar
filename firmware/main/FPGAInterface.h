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

	virtual void BlockingRead(uint32_t insn, uint8_t* data, uint32_t len) = 0;
	virtual void BlockingWrite(uint32_t insn, const uint8_t* data, uint32_t len) = 0;

	uint32_t BlockingRead32(uint32_t insn)
	{
		uint32_t data;
		BlockingRead(insn, reinterpret_cast<uint8_t*>(&data), sizeof(data));
		return data;
	}

	uint8_t BlockingRead8(uint32_t insn)
	{
		uint8_t data;
		BlockingRead(insn, reinterpret_cast<uint8_t*>(&data), sizeof(data));
		return data;
	}

	uint16_t BlockingRead16(uint32_t insn)
	{
		uint16_t data;
		BlockingRead(insn, reinterpret_cast<uint8_t*>(&data), sizeof(data));
		return data;
	}

	void BlockingWrite8(uint32_t insn, uint8_t data)
	{ BlockingWrite(insn, &data, sizeof(data)); }

	void BlockingWrite16(uint32_t insn, uint16_t data)
	{ BlockingWrite(insn, reinterpret_cast<uint8_t*>(&data), sizeof(data)); }

	void BlockingWrite32(uint32_t insn, uint32_t data)
	{ BlockingWrite(insn, reinterpret_cast<uint8_t*>(&data), sizeof(data)); }

};

//must match regid_t in ManagementRegisterInterface.sv
enum regid_t
{
	REG_FPGA_IDCODE		= 0x0000,
	REG_FPGA_SERIAL		= 0x0004,
	REG_FAN0_RPM		= 0x0010,
	REG_FAN1_RPM		= 0x0012,
	REG_DIE_TEMP		= 0x0014,
	REG_VOLT_CORE		= 0x0016,
	REG_VOLT_RAM		= 0x0018,
	REG_VOLT_AUX		= 0x001a,

	REG_USERCODE		= 0x001c,

	REG_FPGA_IRQSTAT	= 0x0020,
	REG_EMAC_RXLEN		= 0x0024,
	REG_EMAC_COMMIT		= 0x0028,
	REG_XG_COMMIT		= 0x002c,

	REG_MGMT0_MDIO		= 0x0048,

	REG_FRONT_CTRL		= 0x0050,
	REG_FRONT_DATA		= 0x0051,
	REG_FRONT_STAT		= 0x0052,
	REG_FRONT_LED_0		= 0x0053,
	REG_FRONT_LED_1		= 0x0054,
	REG_FRONT_LED_2		= 0x0055,

	REG_XG0_STAT		= 0x0060,

	REG_RELAY_TOGGLE	= 0x0070,
	REG_RELAY_STAT		= 0x0072,

	REG_BERT_LANE0_PRBS	= 0x0080,
	REG_BERT_LANE0_TX	= 0x0082,
	REG_BERT_LANE0_WD	= 0x0084,
	REG_BERT_LANE0_AD	= 0x0086,
	REG_BERT_LANE0_RD	= 0x0088,
	REG_BERT_LANE0_STAT	= 0x008a,
	REG_BERT_LANE0_CLK	= 0x008c,
	REG_BERT_LANE0_RST	= 0x008e,
	REG_BERT_LANE0_RX	= 0x0090,

	REG_BERT_LANE1_PRBS	= 0x00a0,
	REG_BERT_LANE1_TX	= 0x00a2,
	REG_BERT_LANE1_WD	= 0x00a4,
	REG_BERT_LANE1_AD	= 0x00a6,
	REG_BERT_LANE1_RD	= 0x00a8,
	REG_BERT_LANE1_STAT	= 0x00aa,
	REG_BERT_LANE1_CLK	= 0x00ac,
	REG_BERT_LANE1_RST	= 0x00ae,
	REG_BERT_LANE1_RX	= 0x00b0,

	REG_MUXSEL_BASE		= 0x00f0,

	REG_EMAC_BUFFER		= 0x1000,

	REG_XG_TX_BUFFER	= 0x2000,

	REG_CRYPT_BASE		= 0x3800,
};

#define BERT_LANE_STRIDE 0x20

enum cryptreg_t
{
	REG_WORK			= 0x0000,
	REG_E				= 0x0020,
	REG_CRYPT_STATUS	= 0x0040,
	REG_WORK_OUT		= 0x0060,
	REG_DSA_IN			= 0x0080,
	REG_DSA_BASE		= 0x0100
};

#endif
