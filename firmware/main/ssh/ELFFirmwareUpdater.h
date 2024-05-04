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

#ifndef ELFFirmwareUpdater_h
#define ELFFirmwareUpdater_h

#define ELF_RX_BUFFER_SIZE 2048

//max number of segments we're willing to process
#define MAX_SEGMENTS_PER_ELF 4

#include "ELFStructs.h"

/**
	@brief Important attributes of a program header specifying how to flash it
 */
class ProgramHeaderData
{
public:
	uint32_t	m_fileoff;
	uint32_t	m_phyaddr;
	uint32_t	m_size;
};

/**
	@brief Base class for flashing an external device with an image in ELF format
 */
class ELFFirmwareUpdater
{
public:
	ELFFirmwareUpdater();
	virtual ~ELFFirmwareUpdater();

	///@brief Called when the device file is opened
	void OnDeviceOpened();

	///@brief Called when new data arrives
	void OnRxData(const uint8_t* data, uint32_t len);

	///@brief Called when the device file is closed (update complete)
	void OnDeviceClosed();

protected:

	void MarkDataProcessed(uint32_t bytes)
	{
		m_offset += bytes;
		m_rxBuffer.Pop(bytes);
	}

	bool ProcessDataFromBuffer();

	void ParseEhdr();
	void DiscardPhdrPadding();
	void ParsePhdr();
	void DiscardSegmentPadding();
	void ProcessSegmentData();

	//Handlers for derived class
	virtual void StartUpdate();
	virtual void OnWriteData(uint32_t physicalAddress, uint8_t* data, uint32_t len);
	virtual void FinishUpdate();

	///@brief Parser state machine
	enum
	{
		STATE_READ_EHDR,
		STATE_PHDR_PADDING,
		STATE_READ_PHDRS,
		STATE_SEGMENT_PADDING,
		STATE_SEGMENT_DATA,
		STATE_DONE,

		STATE_FAILED
	} m_state;

	///@brief Buffer for handling incoming data
	CircularFIFO<ELF_RX_BUFFER_SIZE> m_rxBuffer;

	//Current offset into the file
	uint32_t m_offset;

	//ELF header values
	uint32_t m_phoff;
	uint32_t m_phnum;
	uint32_t m_phIndex;

	///@brief Number of PT_LOAD segments we have to flash
	uint32_t m_numLoadableSegments;

	///@brief Metadata for the loadable segments
	ProgramHeaderData m_loadableSegments[MAX_SEGMENTS_PER_ELF];

	///@brief Index of the current segment being processed
	uint32_t m_currentSegment;
};

#endif
