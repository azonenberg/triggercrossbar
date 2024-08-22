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

#include <core/platform.h>
#include "hwinit.h"
#include "ELFFirmwareUpdater.h"
#include <algorithm>

//TODO move to derived class
#include "../../front/main/regids.h"

int ComparePhdrAddress(const void* a, const void* b);

int ComparePhdrAddress(const void* a, const void* b)
{
	auto pa = reinterpret_cast<const ProgramHeaderData*>(a);
	auto pb = reinterpret_cast<const ProgramHeaderData*>(b);

	if(pa->m_fileoff > pb->m_fileoff)
		return 1;
	else if(pa->m_fileoff < pb->m_fileoff)
		return -1;
	else
		return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction / destruction

ELFFirmwareUpdater::ELFFirmwareUpdater()
	: m_state(STATE_READ_EHDR)
	, m_offset(0)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Incoming data handlers

void ELFFirmwareUpdater::OnDeviceOpened()
{
	m_rxBuffer.Reset();
	m_state = STATE_READ_EHDR;
	m_offset = 0;
	m_numLoadableSegments = 0;

	StartUpdate();
}

void ELFFirmwareUpdater::OnRxData(const uint8_t* data, uint32_t len)
{
	//Push the incoming data into our buffer
	if(!m_rxBuffer.Push(data, len))
	{
		g_log(Logger::ERROR, "RX buffer overflow\n");
		return;
	}

	//Process data until we run out of things to do with the current buffer contents
	while(ProcessDataFromBuffer())
	{}
}

void ELFFirmwareUpdater::OnDeviceClosed()
{
	FinishUpdate();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ELF header parsing

/**
	@brief Process data in the buffer and parse on it

	@return True if we successfully moved to the next state or otherwise are ready to continue
			False if we can't do anything until we get more data
 */
bool ELFFirmwareUpdater::ProcessDataFromBuffer()
{
	switch(m_state)
	{
		//Try to read the ELF header. See if we have enough data for that
		case STATE_READ_EHDR:
			if(m_rxBuffer.ReadSize() >= sizeof(Elf32_Ehdr))
			{
				ParseEhdr();
				return true;
			}
			return false;

		//Discard padding before the program header table
		case STATE_PHDR_PADDING:
			DiscardPhdrPadding();
			return (m_rxBuffer.ReadSize() > 0);

		//Try to read the next program header
		case STATE_READ_PHDRS:
			if(m_rxBuffer.ReadSize() >= sizeof(Elf32_Phdr))
			{
				ParsePhdr();
				return true;
			}
			return false;

		//Throw away anything before the start of the next segment of interest
		case STATE_SEGMENT_PADDING:
			DiscardSegmentPadding();
			return (m_rxBuffer.ReadSize() > 0);

		//Actually process flash data
		case STATE_SEGMENT_DATA:
			ProcessSegmentData();
			return (m_rxBuffer.ReadSize() > 0);

		//Throw away anything after the last loadable segment
		case STATE_DONE:
			m_rxBuffer.Reset();
			return false;

		//If parsing or flashing failed at any point, just throw away all subsequent data
		case STATE_FAILED:
			m_rxBuffer.Reset();
			return false;

		//Don't know what to do
		default:
			g_log(Logger::ERROR, "Unknown state\n");
			return false;
	}

	return false;
}

/**
	@brief Parse the top level header
 */
void ELFFirmwareUpdater::ParseEhdr()
{
	g_log("Parsing ELF header\n");
	LogIndenter li(g_log);

	//Grab the ELF header and validate it
	auto ehdr = reinterpret_cast<Elf32_Ehdr*>(m_rxBuffer.Rewind());
	if( (ehdr->e_ident[0] != ELFMAG0) || (ehdr->e_ident[1] != ELFMAG1) ||
		(ehdr->e_ident[2] != ELFMAG2) || (ehdr->e_ident[3] != ELFMAG3) )
	{
		g_log(Logger::ERROR, "Invalid ELF magic number, aborting\n");
		m_state = STATE_FAILED;
		return;
	}
	if(ehdr->e_ident[EI_CLASS] != ELFCLASS32)
	{
		g_log(Logger::ERROR, "File is not a 32-bit ELF\n");
		m_state = STATE_FAILED;
		return;
	}
	if(ehdr->e_ident[EI_DATA] != ELFDATA2LSB)
	{
		g_log(Logger::ERROR, "File is not a little endian ELF\n");
		m_state = STATE_FAILED;
		return;
	}
	if( (ehdr->e_ident[EI_VERSION] != EV_CURRENT) || (ehdr->e_version != EV_CURRENT) )
	{
		g_log(Logger::ERROR, "File is not a version %d ELF\n", EV_CURRENT);
		m_state = STATE_FAILED;
		return;
	}

	//Ok, it looks like a valid ELF that's at least plausible (32 bit little endian) if we get here.
	//Make sure it's an ARM32 executable
	if(ehdr->e_type != ET_EXEC)
	{
		g_log(Logger::ERROR, "File is not an executable\n");
		m_state = STATE_FAILED;
	}
	if(ehdr->e_machine != EM_ARM)
	{
		g_log(Logger::ERROR, "File is not an ARM binary\n");
		m_state = STATE_FAILED;
	}

	//Ignore entry point address, that points to our vector table not the reset vector

	//Validate program header struct size
	if(ehdr->e_phentsize != sizeof(Elf32_Phdr))
	{
		g_log(Logger::ERROR, "Invalid program header entry size\n");
		m_state = STATE_FAILED;
	}

	//Save program header info
	m_phnum = ehdr->e_phnum;
	m_phoff = ehdr->e_phoff;

	//Done, pop the header
	MarkDataProcessed(sizeof(Elf32_Ehdr));

	//Ready to read program headers
	m_phIndex = 0;
	m_state = STATE_PHDR_PADDING;
}

/**
	@brief Discard padding before the program header table
 */
void ELFFirmwareUpdater::DiscardPhdrPadding()
{
	LogIndenter li(g_log);

	//We want to be at m_phoff
	//If we already are, stop
	if(m_offset == m_phoff)
	{
		m_state = STATE_READ_PHDRS;
		return;
	}

	//If not, calculate how much data we need to discard and see if we already have it in the buffer
	uint32_t remainingPaddingBytes = m_phoff - m_offset;
	auto readsize = m_rxBuffer.ReadSize();
	if(readsize >= remainingPaddingBytes)
	{
		MarkDataProcessed(remainingPaddingBytes);
		m_state = STATE_READ_PHDRS;
	}
	else
	{
		MarkDataProcessed(readsize);
		//stay in this state until we've got rid of all padding
	}
}

/**
	@brief Discard padding before the next segment
 */
void ELFFirmwareUpdater::DiscardSegmentPadding()
{
	LogIndenter li(g_log);

	//We want to be at the file offset of the next section
	//If we already are, stop
	uint32_t targetOffset = m_loadableSegments[m_currentSegment].m_fileoff;
	if(m_offset == targetOffset)
	{
		m_state = STATE_SEGMENT_DATA;
		return;
	}

	//If not, calculate how much data we need to discard and see if we already have it in the buffer
	uint32_t remainingPaddingBytes = targetOffset - m_offset;
	auto readsize = m_rxBuffer.ReadSize();
	if(readsize >= remainingPaddingBytes)
	{
		MarkDataProcessed(remainingPaddingBytes);
		m_state = STATE_SEGMENT_DATA;
	}
	else
	{
		MarkDataProcessed(readsize);
		//stay in this state until we've got rid of all padding
	}
}

/**
	@brief Parse our program headers
 */
void ELFFirmwareUpdater::ParsePhdr()
{
	//Get the phdr
	auto phdr = reinterpret_cast<Elf32_Phdr*>(m_rxBuffer.Rewind());

	//If it's not a PT_LOAD segment, ignore it
	if(phdr->p_type != PT_LOAD)
	{}

	//If the length is zero, even if flagged as loadable it makes no sense to lod it
	else if(phdr->p_filesz == 0)
	{}

	//It's loadable, figure out the details
	//We don't care about vaddr, the startup code takes care of that.
	//We just want physical address to flash it to.
	else
	{
		//Make sure we don't overrun the phdr buffer
		if(m_numLoadableSegments >= MAX_SEGMENTS_PER_ELF)
		{
			g_log(Logger::ERROR, "Too many loadable segments (we only support %d)\n", MAX_SEGMENTS_PER_ELF);
			m_state = STATE_FAILED;
			return;
		}

		//Save it
		m_loadableSegments[m_numLoadableSegments].m_fileoff = phdr->p_offset;
		m_loadableSegments[m_numLoadableSegments].m_phyaddr = phdr->p_paddr;
		m_loadableSegments[m_numLoadableSegments].m_size = phdr->p_filesz;
		m_numLoadableSegments ++;
	}

	//Done
	MarkDataProcessed(sizeof(Elf32_Phdr));

	//Was this the last phdr? if so, get ready to process segment content
	m_phIndex ++;
	if(m_phIndex >= m_phnum)
	{
		//We've processed the last phdr. Sort the list of loadable segments
		g_log("Found a total of %u loadable segments\n", m_numLoadableSegments);
		qsort(m_loadableSegments, m_numLoadableSegments, sizeof(ProgramHeaderData), ComparePhdrAddress);

		//Print them out
		LogIndenter li(g_log);
		for(uint32_t i=0; i<m_numLoadableSegments; i++)
		{
			g_log("[%d] Found a loadable segment (file offset %08x, flash address %08x, %d bytes)\n",
				i,
				m_loadableSegments[i].m_fileoff,
				m_loadableSegments[i].m_phyaddr,
				m_loadableSegments[i].m_size);
		}

		//We're now in padding before the first segment of interest
		m_state = STATE_SEGMENT_PADDING;
		m_currentSegment = 0;
	}
}

void ELFFirmwareUpdater::ProcessSegmentData()
{
	//Offset within the current segment
	uint32_t segoff = m_offset - m_loadableSegments[m_currentSegment].m_fileoff;

	//Target physical address
	uint32_t phydest = m_loadableSegments[m_currentSegment].m_phyaddr + segoff;

	//Figure out how much data to process
	uint32_t bytesLeft = m_loadableSegments[m_currentSegment].m_size - segoff;
	uint32_t blocklen = std::min((uint32_t)m_rxBuffer.ReadSize(), bytesLeft);

	//Actually write to flash and mark it as consumed
	OnWriteData(phydest, m_rxBuffer.Rewind(), blocklen);
	MarkDataProcessed(blocklen);

	//Are we at the end of the segment?
	if(m_offset >= (m_loadableSegments[m_currentSegment].m_fileoff + m_loadableSegments[m_currentSegment].m_size) )
	{
		FinishSegment();

		//Was this the last loadable segment?
		if(m_currentSegment >= m_numLoadableSegments)
			m_state = STATE_DONE;

		//No, move on to the next
		else
		{
			m_state = STATE_SEGMENT_PADDING;
			m_currentSegment ++;
		}
	}
}

