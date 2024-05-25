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
#include <bootloader/bootloader-common.h>
#include <bootloader/BootloaderAPI.h>
#include <microkvs/driver/STM32StorageBank.h>
#include <algorithm>
#include "../bsp-front/hwinit.h"
#include "../front/regids.h"
#include "../../staticnet/util/CircularFIFO.h"

//Application region of flash runs from the end of the bootloader (0x08008000)
//to the start of the KVS (0x0803f000), so 220 kB
//Firmware version string is put right after vector table by linker script at a constant address
uint32_t* const g_appVector  = reinterpret_cast<uint32_t*>(0x8008000);

//@brief Size of the image
const uint32_t g_appImageSize = 220 * 1024;

//Offset of the version string
const uint32_t g_appVersionOffset = 0x1a0;

///@brief The battery-backed RAM used to store state across power cycles
volatile BootloaderBBRAM* g_bbram = reinterpret_cast<volatile BootloaderBBRAM*>(&_RTC.BKP[0]);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Hooks called by bootloader code

void Bootloader_Init()
{
	/**
		@brief Use sectors 126 and 127 of flash for a for a 2 kB microkvs

		TODO: support multiple sectors since 2 kB is a little small?
	 */
	static STM32StorageBank left(reinterpret_cast<uint8_t*>(0x0803f000), 0x800);
	static STM32StorageBank right(reinterpret_cast<uint8_t*>(0x0803f800), 0x800);
	InitKVS(&left, &right, 32);
}

void Bootloader_ClearRxBuffer()
{
	//Clear any SPI RX buffer content from status polling
	g_fpgaSPI.ClearEvents();
}

void Bootloader_FinalCleanup()
{
	g_uart.Flush();
}

void BSP_MainLoop()
{
	Bootloader_MainLoop();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Run the firmware updater

void __attribute__((noreturn)) Bootloader_FirmwareUpdateFlow()
{
	g_log("In DFU mode\n");

	uint32_t nbyte = 0;
	uint8_t cmd = 0;

	CircularFIFO<2048> writebuf;
	uint32_t wraddr = 0;
	uint32_t wraddrNext = 0;
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
							wraddr = wraddrNext - writebuf.ReadSize();
							break;

						//Launch the application
						case FRONT_BOOT_APP:
							{
								g_log("Preparing to boot application\n");

								auto sver = GetImageVersion(g_appVector);
								if(!sver)
								{
									g_log(Logger::ERROR, "No image version string found\n");
									continue;
								}

								//Write image version and expected CRC to KVS
								g_kvs->StoreObject(g_imageVersionKey, (const uint8_t*)sver, strlen(sver));
								g_kvs->StoreObject(g_imageCRCKey, (const uint8_t*)&expectedCRC, sizeof(uint32_t));

								//Try booting it
								if(ValidateAppPartition(g_appVector))
									BootApplication(g_appVector);
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
							EraseFlash(g_appVector);

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
							wraddrNext = (wraddrNext >> 8) | (data << 24);
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
