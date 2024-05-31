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

#ifndef APBSpiFlashInterface_h
#define APBSpiFlashInterface_h

/**
	@file
	@brief Declaration of APBSpiFlashInterface
 */
class APBSpiFlashInterface
{
public:
	APBSpiFlashInterface(volatile APB_SPIHostInterface* device);

	void WriteEnable();
	void WriteDisable();

	bool EraseSector(uint32_t start);

	uint8_t GetStatusRegister1();
	uint8_t GetStatusRegister2();
	uint8_t GetConfigRegister();

	uint32_t GetEraseBlockSize()
	{ return m_sectorSize; }

	void ReadData(uint32_t addr, uint8_t* data, uint32_t len);
	bool WriteData(uint32_t addr, const uint8_t* data, uint32_t len);

	uint32_t GetMinWriteBlockSize()
	{ return 16; }

	uint32_t GetMaxWriteBlockSize()
	{ return m_maxWriteBlock; }

protected:

	void SetCS(bool b);
	void SendByte(uint8_t data);
	uint8_t ReadByte();

	volatile APB_SPIHostInterface*	m_device;

	uint32_t m_capacityBytes;
	uint32_t m_maxWriteBlock;
	uint32_t m_sectorSize;
};

#endif
