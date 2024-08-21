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

#include "triggercrossbar.h"
#include "FrontPanelFirmwareUpdater.h"
#include "../../front/regids.h"

bool g_frontPanelDFUInProgress = false;

bool IsFrontPanelDFU()
{ return g_frontPanelDFUInProgress; }

#define ETHERNET_CRC_POLY 0x04c11db7

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction / destruction

FrontPanelFirmwareUpdater::FrontPanelFirmwareUpdater()
	: m_runningLength(0)
{
	RCCHelper::Enable(&_CRC);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Firmware update logic

void FrontPanelFirmwareUpdater::StartUpdate()
{
	g_log("Starting DFU of front panel\n");
	LogIndenter li(g_log);

	g_frontPanelDFUInProgress = true;

	//Put the front panel in DFU mode if it's not already
	auto mode = GetFrontPanelMode();
	switch(mode)
	{
		case FRONT_NORMAL:
		{
			g_log("Front panel is in normal mode, restarting in DFU mode...\n");

			//Go to DFU mode
			SetFrontPanelCS(0);
			SendFrontPanelByte(FRONT_ENTER_DFU);
			SetFrontPanelCS(1);

			//Wait for reset
			g_logTimer.Sleep(500);

			//Make sure we're back up in DFU mode
			mode = GetFrontPanelMode();
			if(mode != FRONT_BOOTLOADER)
			{
				g_log(Logger::ERROR, "Front panel is not in bootloader mode (expected mode 0x%02x, got %02x)\n",
					FRONT_BOOTLOADER,
					mode);

				m_state = STATE_FAILED;
				return;
			}
		}
		break;

		case FRONT_BOOTLOADER:
			g_log("Front panel is already in DFU mode, no action needed at this time\n");
			break;

		default:
			g_log(Logger::ERROR, "Front panel is not in a valid mode (got %02x)\n", mode);
			break;
	}

	//Erase the application flash partition
	//TODO: can we optimize to only erase sectors that have content?
	g_log("Erasing application flash partition...\n");
	auto start = g_logTimer.GetCount();
	SetFrontPanelCS(0);
	SendFrontPanelByte(FRONT_ERASE_APP);
	g_logTimer.Sleep(1);
	while(ReadFrontPanelByte() == 0)
	{
		//time out after 5 sec
		auto delta = g_logTimer.GetCount() - start;
		if(delta > 50000)
		{
			g_log(Logger::ERROR, "Timed out waiting for flash erase\n");
			m_state = STATE_FAILED;
			break;
		}

		g_logTimer.Sleep(1);
	}
	SetFrontPanelCS(1);

	CRC::ChecksumInit();
	m_runningLength = 0;

	LogIndenter li2(g_log);
	auto delta = g_logTimer.GetCount() - start;
	g_log("Flash erase complete (in %d.%d ms)\n", delta / 10, delta % 10);
}

void FrontPanelFirmwareUpdater::OnWriteData(uint32_t physicalAddress, uint8_t* data, uint32_t len)
{
	//If the block is outside our application area, abort
	uint32_t flashStart = 0x8008000;
	uint32_t flashEnd = 0x803f000;
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

	//For now, require the first block to be at least 0x1e0 bytes (openssh always will do this)
	if(physicalAddress == flashStart)
	{
		if(len < 0x1e0)
		{
			g_log(Logger::ERROR,
				"We don't know how to parse initial blocks that are too small (version strings split)\n");
			m_state = STATE_FAILED;
			return;
		}

		if(data[0x1bf] != '\0')
		{
			g_log(Logger::ERROR, "Missing null terminator on firmware build string\n");
			m_state = STATE_FAILED;
			return;
		}

		if(data[0x1df] != '\0')
		{
			g_log(Logger::ERROR, "Missing null terminator on firmware ID string\n");
			m_state = STATE_FAILED;
			return;
		}

		auto fwBuild = reinterpret_cast<const char*>(data + 0x1a0);
		auto fwID = reinterpret_cast<const char*>(data + 0x1c0);

		g_log("Firmware build date: %s\n", fwBuild);
		g_log("Firmware ID:         %s\n", fwID);

		const char* expectedID = "trigger-crossbar-frontpanel";
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
		//TODO: push build date to bootloader or just let it parse?
	}

	//Send the address to the bootloader
	SetFrontPanelCS(0);
	SendFrontPanelByte(FRONT_FLASH_ADDR);
	SendFrontPanelByte(physicalAddress & 0xff);
	SendFrontPanelByte((physicalAddress >> 8) & 0xff);
	SendFrontPanelByte((physicalAddress >> 16) & 0xff);
	SendFrontPanelByte((physicalAddress >> 24) & 0xff);
	SetFrontPanelCS(1);

	g_logTimer.Sleep(5);

	//Send the data
	//TODO: if more than 1-2 kB, chunk it
	auto start = g_logTimer.GetCount();
	SetFrontPanelCS(0);
	SendFrontPanelByte(FRONT_FLASH_WRITE);
	for(uint32_t i=0; i<len; i++)
		SendFrontPanelByte(data[i]);
	SetFrontPanelCS(1);

	g_logTimer.Sleep(50);

	//Calculate the CRC32 of the data while we wait for the write to complete
	CRC::ChecksumUpdate(data, len);
	auto runningCRC = CRC::ChecksumFinal();
	m_runningLength += len;

	//Poll until the write has completed
	SetFrontPanelCS(0);
	SendFrontPanelByte(FRONT_FLASH_STATUS);
	g_logTimer.Sleep(20);
	while(ReadFrontPanelByte() != 1)
		g_logTimer.Sleep(50);
	SetFrontPanelCS(1);

	//Flush the SPI buffer to make sure we don't get false "done" values
	SetFrontPanelCS(0);
	SendFrontPanelByte(FRONT_FLASH_SYNC);
	g_logTimer.Sleep(20);
	while(ReadFrontPanelByte() != 0xcc)
		g_logTimer.Sleep(50);
	SetFrontPanelCS(1);

	//1K bits / 1 sec =
	//1 bits / 1 ms =
	//0.1 bits / tick
	auto delta = g_logTimer.GetCount() - start;
	uint32_t kbps = 10 * len / delta;
	g_log("Wrote %u bytes to 0x%08x in %d.%d ms (%u Kbps), running crc = %08x, length = %d\n",
		len, physicalAddress, delta / 10, delta % 10, kbps, runningCRC, m_runningLength);
}

void FrontPanelFirmwareUpdater::FinishSegment()
{
	g_log("Segment complete\n");
	LogIndenter li(g_log);

	//Flush any remaining data to flash
	g_log("Flushing remaining data to flash\n");
	SetFrontPanelCS(0);
	SendFrontPanelByte(FRONT_FLASH_FLUSH);
	g_logTimer.Sleep(1);
	while(ReadFrontPanelByte() == 0)
	{}
	SetFrontPanelCS(1);

	//If the segment did not end on a flash write block (64 bit) boundary, append the necessary number of 0x00
	//padding bytes and CRC them
	static const uint8_t zero[8] = { 0 };
	if(m_runningLength % 8)
	{
		auto zpad = 8 - (m_runningLength % 8);
		CRC::ChecksumUpdate(zero, zpad);
		m_runningLength += zpad;
	}
}

void FrontPanelFirmwareUpdater::FinishUpdate()
{
	g_log("DFU complete\n");
	LogIndenter li(g_log);

	//CRC of the actual firmware file
	g_log("Calculated image CRC32: 0x%08x\n", CRC::ChecksumFinal());

	//Calculate CRC of the entire flash partition
	//Pad with zeroes up to an 8-byte boundary, then ones
	//(this is horribly inefficient!)
	static const uint8_t zero[8] = { 0 };
	static const uint8_t one[8] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
	if(m_runningLength % 8)
	{
		auto zpad = 8 - (m_runningLength % 8);
		CRC::ChecksumUpdate(zero, zpad);
		m_runningLength += zpad;
	}
	const uint32_t flashSectorLength = 2048;
	const uint32_t flashSectorCount = 110;
	const uint32_t flashByteSize = flashSectorLength * flashSectorCount;
	uint32_t blocksToAdd = (flashByteSize - m_runningLength) / 8;
	for(uint32_t i=0; i<blocksToAdd; i++)
		CRC::ChecksumUpdate(one, 8);
	auto crc = CRC::ChecksumFinal();
	g_log("Calculated full-partition CRC32: 0x%08x\n", crc);

	//Send the expected CRC to the bootloader
	SetFrontPanelCS(0);
	SendFrontPanelByte(FRONT_EXPECTED_CRC);
	SendFrontPanelByte(crc & 0xff);
	SendFrontPanelByte((crc >> 8) & 0xff);
	SendFrontPanelByte((crc >> 16) & 0xff);
	SendFrontPanelByte((crc >> 24) & 0xff);
	SetFrontPanelCS(1);

	//Request booting the app
	g_log("Booting application\n");
	SetFrontPanelCS(0);
	SendFrontPanelByte(FRONT_BOOT_APP);
	SetFrontPanelCS(1);

	//Wait for reset
	//TODO: we don't want to hang the whole chip for 1 sec, do this in some kind of timer state machine
	g_logTimer.Sleep(15000);

	//Make sure we're back up in application mode
	auto mode = GetFrontPanelMode();
	switch(mode)
	{
		case FRONT_NORMAL:
			g_log("Update successful\n");
			g_frontPanelDFUInProgress = false;
			break;

		case FRONT_BOOTLOADER:
			g_log(Logger::ERROR, "Front panel is still in DFU mode, something went wrong\n");
			break;

		default:
			g_log(Logger::ERROR, "Front panel is not in a valid mode (got %02x)\n", mode);
			break;
	}

	//Send another command to return to jtag mode
	SendFrontPanelByte(FRONT_REFRESH_FAST);
}
