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
#include "BootloaderAPI.h"
#include <microkvs/driver/STM32StorageBank.h>
#include <algorithm>
#include "../front/regids.h"
#include "../../staticnet/util/CircularFIFO.h"

GPIOPin* g_inmodeLED[4] = {nullptr};
GPIOPin* g_outmodeLED[4] = {nullptr};

///@brief Key-value store used for storing firmware version config etc
KVS* g_kvs = nullptr;

bool ValidateAppPartition(const uint32_t* appVector);
void FirmwareUpdateFlow();
void BootApplication(const uint32_t* appVector);
bool IsAppUpdated(const char*& firmwareVer);
extern "C" void __attribute__((noreturn)) DoBootApplication(const uint32_t* appVector);

///@brief The battery-backed RAM used to store state across power cycles
volatile BootloaderBBRAM* g_bbram = reinterpret_cast<volatile BootloaderBBRAM*>(&_RTC.BKP[0]);

const char* g_imageVersionKey = "firmware.imageVersion";
const char* g_imageCRCKey = "firmware.crc";

void EraseFlash();
const char* GetImageVersion();

void App_Init()
{
	g_log("Front panel bootloader (%s %s)\n", __DATE__, __TIME__);

	RCCHelper::Enable(&_CRC);
	RCCHelper::Enable(&_RTC);

	/**
		@brief Use sectors 126 and 127 of flash for a for a 2 kB microkvs

		TODO: support multiple sectors since 2 kB is a little small?
	 */
	static STM32StorageBank left(reinterpret_cast<uint8_t*>(0x0803f000), 0x800);
	static STM32StorageBank right(reinterpret_cast<uint8_t*>(0x0803f800), 0x800);
	InitKVS(&left, &right, 32);
}

void BSP_MainLoop()
{
	//Check bbram state
	bool goStraightToDFU = false;
	bool crashed = false;
	g_log("Checking reason for last reset...\n");
	{
		LogIndenter li(g_log);
		switch(g_bbram->m_state)
		{
			case STATE_POR:
				g_log("Power cycle\n");
				break;

			case STATE_APP:
				g_log("Application was running, probably requested warm reboot\n");
				break;

			case STATE_DFU:
				g_log("Application requested DFU entry\n");
				goStraightToDFU = true;
				break;

			case STATE_CRASH:
				crashed = true;
				switch(g_bbram->m_crashReason)
				{
					case CRASH_UNUSED_ISR:
						g_log(Logger::ERROR, "Unused ISR called\n");
						break;

					case CRASH_NMI:
						g_log(Logger::ERROR, "NMI\n");
						break;

					case CRASH_HARD_FAULT:
						g_log(Logger::ERROR, "Hard fault\n");
						break;

					case CRASH_BUS_FAULT:
						g_log(Logger::ERROR, "Bus fault\n");
						break;

					case CRASH_USAGE_FAULT:
						g_log(Logger::ERROR, "Usage fault\n");
						break;

					case CRASH_MMU_FAULT:
						g_log(Logger::ERROR, "MMU fault\n");
						break;

					default:
						g_log(Logger::ERROR, "Unknown crash code\n");
						break;
				}
				break;

			default:
				g_log("Last reset from unknown cause\n");
				break;
		}
	}

	//Application region of flash runs from the end of the bootloader (0x08008000)
	//to the start of the KVS (0x0803f000), so 220 kB
	//Firmware version string is put right after vector table by linker script at a constant address
	const uint32_t* appVector  = reinterpret_cast<const uint32_t*>(0x8008000);

	//Skip all other processing if a DFU was requested
	if(goStraightToDFU)
		FirmwareUpdateFlow();

	//Application crashed? Don't try to run the crashy app again to avoid bootlooping
	//TODO: give it a couple of tries first?
	else if(crashed)
	{
		const char* firmwareVer = nullptr;
		if(IsAppUpdated(firmwareVer))
		{
			g_log("Application was updated since last flash, attempting to boot new image\n");
			if(ValidateAppPartition(appVector))
				BootApplication(appVector);
			else
				FirmwareUpdateFlow();
		}

		else
		{
			g_log("Still running same crashy binary, going to DFU flow\n");
			FirmwareUpdateFlow();
		}
	}

	//Corrupted image? Go to DFU, it's our only hope
	else if(!ValidateAppPartition(appVector))
		FirmwareUpdateFlow();

	//If we get to this point, the image is valid, boot it
	else
		BootApplication(appVector);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Run the firmware updater

void __attribute__((noreturn)) FirmwareUpdateFlow()
{
	g_log("In DFU mode\n");

	uint32_t nbyte = 0;
	uint8_t cmd = 0;

	const uint32_t* appVector  = reinterpret_cast<const uint32_t*>(0x8008000);

	CircularFIFO<2048> writebuf;
	uint32_t wraddr = 0;
	uint32_t bytesSoFar = 0;
	uint32_t bytesLastPrinted = 0;
	uint32_t expectedCRC = 0;

	while(1)
	{
		g_log.UpdateOffset(60000);

		//If we have at least a full 8-byte block of data, push it to flash
		if(cmd != FRONT_FLASH_WRITE)
		{
			while(writebuf.ReadSize() >= 8)
			{
				Flash::Write((uint8_t*)wraddr, writebuf.Rewind(), 8);
				writebuf.Pop(8);
				wraddr += 8;
				bytesSoFar += 8;

				if(bytesSoFar > (bytesLastPrinted + 4096))
				{
					g_log("[%08x] Wrote %d bytes, %u still in buffer, %u overflows, %u pending\n",
						wraddr, bytesSoFar, writebuf.ReadSize(), g_spiRxFifoOverflows, g_fpgaSPI.GetEventCount());
					bytesLastPrinted = bytesSoFar;
				}

				//If there's <8 bytes left and we're not still pushing new data, we're done
				if( (writebuf.ReadSize() < 8) && (cmd != FRONT_FLASH_WRITE) )
				{
					g_fpgaSPI.NonblockingWriteDevice(0x01);
					break;
				}
			}
		}

		//If CS# deasserted, go to JTAG mode
		if(*g_fpgaSPICS)
		{
			if(!g_misoIsJtag)
				SetMisoToJTAGMode();
		}

		//Read and process SPI events
		if(g_fpgaSPI.HasEvents())
		{
			auto event = g_fpgaSPI.GetEvent();

			//Reset byte count on CS# rising or falling edge
			if(event.type == SPIEvent::TYPE_CS)
				nbyte = 0;

			//Process data byte
			else
			{
				auto data = event.data;

				//First byte is command
				if(nbyte == 0)
				{
					cmd = data;

					switch(cmd)
					{
						//padding sent during flash writes etc, discard it
						case 0x00:
							break;

						//Update write address but account for pending data not yet written
						case FRONT_FLASH_WRITE:
							wraddr -= writebuf.ReadSize();
							break;

						//Launch the application
						case FRONT_BOOT_APP:
							{
								g_log("Preparing to boot application\n");

								auto sver = GetImageVersion();
								if(!sver)
								{
									g_log(Logger::ERROR, "No image version string found\n");
									continue;
								}

								//Write image version and expected CRC to KVS
								g_kvs->StoreObject(g_imageVersionKey, (const uint8_t*)sver, strlen(sver));
								g_kvs->StoreObject(g_imageCRCKey, (const uint8_t*)&expectedCRC, sizeof(uint32_t));

								//Try booting it
								if(ValidateAppPartition(appVector))
									BootApplication(appVector);
								g_log("Image is invalid, remaining in DFU mode\n");
							}
							break;

						//Start erasing flash
						case FRONT_ERASE_APP:

							//Tell client we're busy
							if(g_misoIsJtag)
								SetMisoToSPIMode();
							g_fpgaSPI.NonblockingWriteDevice(0x00);

							//Do the erase cycle
							bytesSoFar = 0;
							bytesLastPrinted = 0;
							expectedCRC = 0;
							EraseFlash();

							//Discard any SPI data events that showed up during erase
							g_fpgaSPI.ClearEvents();

							//We're now done
							g_fpgaSPI.NonblockingWriteDevice(0x01);

							break;

						//Get the operating mode
						case FRONT_GET_STATUS:
							{
								if(g_misoIsJtag)
									SetMisoToSPIMode();

								g_fpgaSPI.NonblockingWriteDevice(FRONT_BOOTLOADER);
							}
							break;

						//Get write status
						case FRONT_FLASH_STATUS:

							//Tell client we're busy
							if(g_misoIsJtag)
								SetMisoToSPIMode();
							g_fpgaSPI.NonblockingWriteDevice(0x00);

							break;

						//Synchronize to make sure we're not reading stale data
						case FRONT_FLASH_SYNC:

							//Tell client we're busy
							if(g_misoIsJtag)
								SetMisoToSPIMode();
							g_fpgaSPI.NonblockingWriteDevice(0xcc);

							break;

						//Flush any remaining flash data (end of the block)
						case FRONT_FLASH_FLUSH:
							{
								//Tell client we're busy
								if(g_misoIsJtag)
									SetMisoToSPIMode();
								g_fpgaSPI.NonblockingWriteDevice(0x00);

								//Flush any remaining data to flash
								auto nbytes = writebuf.ReadSize();
								if(nbytes > 0)
								{
									Flash::Write((uint8_t*)wraddr, writebuf.Rewind(), nbytes);
									writebuf.Pop(nbytes);
									wraddr += nbytes;
								}

								g_log("Flushed write buffer, total %u bytes so far\n", bytesSoFar);

								//We're now done
								g_fpgaSPI.NonblockingWriteDevice(0x01);
							}
							break;

						default:
							break;
					}
				}

				//Then comes data bytes
				else
				{
					switch(cmd)
					{
						//Readback commands do nothing here
						case 0x00:
						case FRONT_GET_STATUS:
						case FRONT_ERASE_APP:
							break;

						//Flash address (little endian)
						case FRONT_FLASH_ADDR:
							wraddr = (wraddr >> 8) | (data << 24);
							break;

						//Expected CRC32 (little endian)
						case FRONT_EXPECTED_CRC:
							expectedCRC = (expectedCRC >> 8) | (data << 24);
							break;

						//Flash data
						case FRONT_FLASH_WRITE:
							writebuf.Push(data);
							break;

						default:
							//g_log("Unrecognized command %02x\n", cmd);
							break;

					}
				}

				nbyte ++;
			}
		}
	}
}

void EraseFlash()
{
	//TODO: it's much more efficient to only erase the stuff we are overwriting, instead of the whole chip
	g_log("Erasing application flash partition\n");
	LogIndenter li(g_log);

	uint8_t* appStart				= reinterpret_cast<uint8_t*>(0x08008000);
	uint8_t* appEnd 				= reinterpret_cast<uint8_t*>(0x0803f000);
	const uint32_t delta			= appEnd - appStart;
	const uint32_t eraseBlockSize	= 2048;
	const uint32_t nblocks 			= delta / eraseBlockSize;

	auto start = g_logTimer.GetCount();
	for(uint32_t i=0; i<nblocks; i++)
	{
		if( (i % 10) == 0)
		{
			auto dt = g_logTimer.GetCount() - start;
			g_log("Block %u / %u (elapsed %d.%d ms, %u overflows)\n", i, nblocks, dt / 10, dt % 10, g_spiRxFifoOverflows);
		}

		Flash::BlockErase(appStart + i*eraseBlockSize);

		//Clear any SPI RX buffer content from status polling
		g_fpgaSPI.ClearEvents();
	}

	auto dt = g_logTimer.GetCount() - start;
	g_log("Done (in %d.%d ms)\n", dt / 10, dt % 10);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Boot the application

void __attribute__((noreturn)) BootApplication(const uint32_t* appVector)
{
	//Debug delay in case we bork something
	g_logTimer.Sleep(10000);

	//Print our final log message and flush the transmit FIFO before transferring control to the application
	g_log("Booting application...\n\n");
	g_uart.Flush();

	g_bbram->m_state = STATE_APP;
	DoBootApplication(appVector);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Application booting

const char* GetImageVersion()
{
	auto firmwareVer = reinterpret_cast<const char*>(0x80081a0);
	for(size_t i=0; i<32; i++)
	{
		if(firmwareVer[i] == '\0')
			return firmwareVer;
	}

	//no null terminator
	return nullptr;
}

/**
	@brief Checks if the application partition contains a different firmware version than we last booted
 */
bool IsAppUpdated(const char*& firmwareVer)
{
	//Image is present, see if we have a good version string
	firmwareVer = reinterpret_cast<const char*>(0x80081a0);
	bool validVersion = false;
	for(size_t i=0; i<32; i++)
	{
		if(firmwareVer[i] == '\0')
		{
			validVersion = true;
			break;
		}
	}
	if(!validVersion)
	{
		g_log(Logger::ERROR, "No version string found in application partition!\n");
		g_log("Expected <32 byte null terminated string at 0x%08x\n",
			reinterpret_cast<uint32_t>(firmwareVer));

		return false;
	}
	g_log("Found firmware version:       %s\n", firmwareVer);

	//See if we're booting a previously booted image
	auto hlog = g_kvs->FindObject(g_imageVersionKey);
	if(hlog)
	{
		char knownVersion[33] = {0};
		strncpy(knownVersion, (const char*)g_kvs->MapObject(hlog), std::min(hlog->m_len, (uint32_t)32));
		g_log("Previous image version:       %s\n", knownVersion);

		//Is this the image we are now booting?
		if(0 != strcmp(knownVersion, firmwareVer))
			return true;
	}

	//Valid image but nothing in KVS, we must have just jtagged the first firmware
	else
	{
		g_log("No previous image version information in KVS\n");
		return true;
	}

	//If we get here, we're using the same firmware
	return false;
}

/**
	@brief Check if the app partition contains what looks like a valid image
 */
bool ValidateAppPartition(const uint32_t* appVector)
{
	g_log("Checking application partition at 0x%08x\n",
		reinterpret_cast<uint32_t>(appVector));
	LogIndenter li(g_log);

	//Vector table is blank? No app present
	if(appVector[0] == 0xffffffff)
	{
		g_log(Logger::ERROR, "Application partition appears to be blank\n");
		return false;
	}

	//See if we have a saved CRC in flash
	uint32_t expectedCRC = 0;
	const char* firmwareVer = nullptr;
	bool updatedViaJtag = IsAppUpdated(firmwareVer);
	if(!updatedViaJtag)
	{
		auto hlog = g_kvs->FindObject(g_imageCRCKey);
		if(!hlog)
		{
			g_log(Logger::WARNING, "Image version found in KVS, but not a CRC. Can't verify integrity\n");
			updatedViaJtag = true;
		}

		else
		{
			expectedCRC = *reinterpret_cast<uint32_t*>(g_kvs->MapObject(hlog));
			g_log("Expected image CRC:           %08x\n", expectedCRC);
		}
	}

	//CRC the entire application partition (including blank space)
	uint32_t appImageSize = 220 * 1024;
	auto start = g_logTimer.GetCount();
	auto crc = CRC::Checksum((const uint8_t*)appVector, appImageSize);
	auto dt = g_logTimer.GetCount() - start;
	g_log("CRC of application partition: %08x (took %d.%d ms)\n", crc, dt/10, dt%10);

	//If we detected a JTAG update, we need to update the saved version and checksum info
	if(updatedViaJtag)
	{
		g_log("New image present (JTAG flash?) but no corresponding saved CRC, updating CRC and version\n");

		g_kvs->StoreObject(g_imageVersionKey, (const uint8_t*)firmwareVer, strlen(firmwareVer));
		g_kvs->StoreObject(g_imageCRCKey, (const uint8_t*)&crc, sizeof(crc));
		return true;
	}

	//We are booting the same image we have in flash. Need to check integrity
	else if(crc == expectedCRC)
	{
		g_log("CRC verification passed\n");
		return true;
	}

	else
	{
		g_log(Logger::ERROR, "CRC mismatch, application partition flash corruption?\n");
		return false;
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Minimal hardware init

/**
	@brief Set up the microkvs key-value store for persisting our configuration
 */
void InitKVS(StorageBank* left, StorageBank* right, uint32_t logsize)
{
	g_log("Initializing microkvs key-value store\n");
	static KVS kvs(left, right, logsize);
	g_kvs = &kvs;

	LogIndenter li(g_log);
	g_log("Block size:  %d bytes\n", kvs.GetBlockSize());
	g_log("Log:         %d / %d slots free\n", (int)kvs.GetFreeLogEntries(), (int)kvs.GetLogCapacity());
	g_log("Data:        %d / %d bytes free\n", (int)kvs.GetFreeDataSpace(), (int)kvs.GetDataCapacity());
	g_log("Active bank: %s\n", kvs.IsLeftBankActive() ? "left" : "right");
}
