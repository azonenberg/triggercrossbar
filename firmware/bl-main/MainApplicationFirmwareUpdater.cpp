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

#include "bootloader.h"
#include "MainApplicationFirmwareUpdater.h"

#define ETHERNET_CRC_POLY 0x04c11db7

extern const char* g_imageVersionKey;
extern const char* g_imageCRCKey;

///@brief Number of sectors in the image
const uint32_t g_flashSectorCount = 5;

///@brief Size of a sector
const uint32_t g_flashSectorSize = 128 * 1024;

///@brief Size of the image
const uint32_t g_appImageSize = g_flashSectorCount * g_flashSectorSize;

//true if we want to boot the app soon
bool g_bootAppPending = false;

//100ms steps until we try booting the application
uint32_t g_bootAppTimer = 0;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction / destruction

MainApplicationFirmwareUpdater::MainApplicationFirmwareUpdater()
	: m_runningLength(0)
	, m_pendingPhysicalAddress(0)
{
	RCCHelper::Enable(&_CRC);
}

MainApplicationFirmwareUpdater::~MainApplicationFirmwareUpdater()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Firmware update logic

void MainApplicationFirmwareUpdater::StartUpdate()
{
	g_log("Starting DFU of main MCU\n");
	LogIndenter li(g_log);

	//Erase the application flash partition
	//TODO: can we optimize to only erase sectors that have content?
	g_log("Erasing application flash partition...\n");
	LogIndenter li2(g_log);
	auto start = g_logTimer.GetCount();

	//Main application partition is five sectors of 128 kB, for now always erase all of it
	for(uint32_t i=0; i<g_flashSectorCount; i++)
	{
		auto ptr = reinterpret_cast<uint8_t*>(g_appVector) + g_flashSectorSize*i;
		g_log("Block %d of %d (%08x)...\n", i+1, g_flashSectorCount, reinterpret_cast<uint32_t>(ptr));
		Flash::BlockErase(ptr);
	}

	//Verify it's blank
	g_log("Blank check...\n");
	auto sr = SCB_DisableDataFaults();
	Flash::ClearECCFaults();
	for(uint32_t i=0; i<g_appImageSize; i+=4)
	{
		auto rdata = g_appVector[i/4];
		if(rdata != 0xffffffff)
		{
			g_log(Logger::ERROR, "Flash is not blank (at 0x%08x, expected 0xffffffff, read 0x%08x\n",
				g_appVector + (i/4),
				rdata);

			SCB_EnableDataFaults(sr);
			m_state = STATE_FAILED;
			return;
		}
	}
	if(Flash::CheckForECCFaults())
	{
		g_log(Logger::ERROR, "Uncorrectable ECC error during blank check (at %08x)\n", Flash::GetFaultAddress());
		Flash::ClearECCFaults();
		SCB_EnableDataFaults(sr);
		m_state = STATE_FAILED;
		return;
	}
	SCB_EnableDataFaults(sr);

	CRC::ChecksumInit();
	m_runningLength = 0;
	m_pendingPhysicalAddress = 0;

	auto delta = g_logTimer.GetCount() - start;
	g_log("Flash erase complete (in %d.%d ms)\n", delta / 10, delta % 10);
}

void MainApplicationFirmwareUpdater::OnWriteData(uint32_t physicalAddress, uint8_t* data, uint32_t len)
{
	//If the block is outside our application area, abort
	uint32_t flashStart = reinterpret_cast<uint32_t>(g_appVector);
	uint32_t flashEnd = flashStart + g_appImageSize;
	if( (physicalAddress < flashStart) || ( (physicalAddress + len) > flashEnd) )
	{
		g_log(Logger::ERROR,
			"Physical address range (%08x, len %d) is not completely within application flash (0x%08x - 0x%08x)\n",
			physicalAddress,
			len,
			flashStart,
			flashEnd);
		m_state = STATE_FAILED;
		return;
	}

	//For now, require the first block to be at least $vectortable bytes (openssh always will do this)
	if(physicalAddress == flashStart)
	{
		if(len < (g_appVersionOffset + 32))
		{
			g_log(Logger::ERROR,
				"We don't know how to parse initial blocks that are too small (version strings split)\n");
			m_state = STATE_FAILED;
			return;
		}

		if(data[g_appVersionOffset + 31] != '\0')
		{
			g_log(Logger::ERROR, "Missing null terminator on firmware build string\n");
			m_state = STATE_FAILED;
			return;
		}

		if(data[g_appVersionOffset + 63] != '\0')
		{
			g_log(Logger::ERROR, "Missing null terminator on firmware ID string\n");
			m_state = STATE_FAILED;
			return;
		}

		auto fwBuild = reinterpret_cast<const char*>(data + g_appVersionOffset);
		auto fwID = fwBuild + 0x20;

		g_log("Firmware build date: %s\n", fwBuild);
		g_log("Firmware ID:         %s\n", fwID);

		const char* expectedID = "trigger-crossbar-main";
		if(0 != strcmp(fwID, expectedID))
		{
			g_log(
				Logger::ERROR,
				"Aborting flash (image is probably for the wrong device, expected ID %s)\n",
				expectedID);
			m_state = STATE_FAILED;
			return;
		}

		//TODO: don't erase flash until we've confirmed the new image is actually valid for us?

		//Write this image version to the bootloader now as the expected image ID string
		if(!g_kvs->StoreObject(g_imageVersionKey, (const uint8_t*)fwBuild, strlen(fwBuild)))
			g_log(Logger::ERROR, "KVS write error\n");
	}

	//Record run time
	auto start = g_logTimer.GetCount();

	//Save the write data in our buffer
	m_pendingWriteData.Push(data, len);

	//If this is the first write of the segment, update our address
	if(m_pendingPhysicalAddress == 0)
		m_pendingPhysicalAddress = physicalAddress;

	//Write data until we run out of full write blocks to store (32 bytes)
	auto wdata = m_pendingWriteData.Rewind();
	auto wlen = m_pendingWriteData.ReadSize() & ~0x1f;
	if(!Flash::Write(reinterpret_cast<uint8_t*>(m_pendingPhysicalAddress), wdata, wlen))
	{
		g_log(Logger::ERROR, "Flash write failed\n");
		m_state = STATE_FAILED;
	}

	//Calculate the CRC32 of the data we just flashed
	CRC::ChecksumUpdate(wdata, wlen);
	auto runningCRC = CRC::ChecksumFinal();
	m_runningLength += wlen;

	m_pendingWriteData.Pop(wlen);
	m_pendingWriteData.Rewind();

	//1K bits / 1 sec =
	//1 bits / 1 ms =
	//0.1 bits / tick
	auto delta = g_logTimer.GetCount() - start;
	uint32_t kbps = 10 * len / delta;
	g_log("Wrote %u bytes to 0x%08x in %d.%d ms (%u Kbps), running crc = %08x, length = %d\n",
		wlen, m_pendingPhysicalAddress, delta / 10, delta % 10, kbps, runningCRC, m_runningLength);

	//Record new start point for next chunk of data
	m_pendingPhysicalAddress += wlen;
}

void MainApplicationFirmwareUpdater::FinishSegment()
{
	g_log("Segment complete\n");
	LogIndenter li(g_log);
	g_log("Flushing remaining data to flash\n");

	//Record run time
	auto start = g_logTimer.GetCount();

	//Flush any remaining data to flash
	auto wdata = m_pendingWriteData.Rewind();
	auto wlen = m_pendingWriteData.ReadSize();
	if(!Flash::Write(reinterpret_cast<uint8_t*>(m_pendingPhysicalAddress), wdata, wlen))
	{
		g_log(Logger::ERROR, "Flash write failed\n");
		m_state = STATE_FAILED;
	}
	CRC::ChecksumUpdate(wdata, wlen);
	m_pendingWriteData.Reset();
	m_runningLength += wlen;

	//If the segment did not end on a flash write block (32 byte) boundary, append the necessary number of 0x00
	//padding bytes and CRC them
	static const uint8_t zero[32] = { 0 };
	if(m_runningLength % 32)
	{
		auto zpad = 32 - (m_runningLength % 32);
		CRC::ChecksumUpdate(zero, zpad);
		m_runningLength += zpad;
	}
	auto runningCRC = CRC::ChecksumFinal();

	//1K bits / 1 sec =
	//1 bits / 1 ms =
	//0.1 bits / tick
	auto delta = g_logTimer.GetCount() - start;
	uint32_t kbps = 10 * wlen / delta;
	g_log("Wrote %u bytes to 0x%08x in %d.%d ms (%u Kbps), running crc = %08x, length = %d\n",
		wlen, m_pendingPhysicalAddress, delta / 10, delta % 10, kbps, runningCRC, m_runningLength);

	//Done with this segment, no longer have a dest address
	m_pendingPhysicalAddress = 0;
}

void MainApplicationFirmwareUpdater::FinishUpdate()
{
	g_log("DFU complete\n");
	LogIndenter li(g_log);

	//CRC of the actual firmware file
	g_log("Calculated image CRC32: 0x%08x\n", CRC::ChecksumFinal());

	//Calculate CRC of the entire flash partition
	//(this is horribly inefficient!)
	static const uint8_t one[32] =
	{
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
	};
	uint32_t blocksToAdd = (g_appImageSize - m_runningLength) / 32;
	for(uint32_t i=0; i<blocksToAdd; i++)
		CRC::ChecksumUpdate(one, 32);
	uint32_t crc = CRC::ChecksumFinal();
	g_log("Calculated full-partition CRC32: 0x%08x\n", crc);

	//Send the expected CRC to the bootloader
	if(!g_kvs->StoreObject(g_imageCRCKey, (const uint8_t*)&crc, 4))
		g_log(Logger::ERROR, "KVS write error\n");

	//Request booting the app but give the SSH session a chance to close down
	g_bootAppTimer = 2;
	g_bootAppPending = true;
}
