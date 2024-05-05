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

///@brief Logging UART
///USART2 is on APB1 (40 MHz), so we need a divisor of 347.22, round to 347
UART<16, 256> g_uart(&USART2, 347);

///@brief Logging block
Logger g_log;

//APB1 is 40 MHz
//Divide down to get 10 kHz ticks (note TIM2 is double rate)
Timer g_logTimer(&TIM2, Timer::FEATURE_ADVANCED, 8000);

GPIOPin* g_inmodeLED[4] = {nullptr};
GPIOPin* g_outmodeLED[4] = {nullptr};

SPI<1024, 64> g_fpgaSPI(&SPI1, true, 2, false);
GPIOPin* g_fpgaSPICS = nullptr;

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

//Default MISO to be using alt mode 0 (NJTRST) so we can use JTAG for debug
GPIOPin g_fpgaMiso(&GPIOB, 4, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_FAST, 0);

bool g_misoIsJtag = true;

void SetMisoToSPIMode();
void SetMisoToJTAGMode();

void EraseFlash();

int main()
{
	//Copy .data from flash to SRAM (for some reason the default newlib startup won't do this??)
	memcpy(&__data_start, &__data_romstart, &__data_end - &__data_start + 1);

	//Hardware setup
	InitPower();
	InitClocks();
	InitUART();
	InitLog();
	RCCHelper::Enable(&_CRC);
	RCCHelper::Enable(&_RTC);
	InitSPI();

	/**
		@brief Use sectors 126 and 127 of flash for a for a 2 kB microkvs

		TODO: support multiple sectors since 2 kB is a little small?
	 */
	static STM32StorageBank left(reinterpret_cast<uint8_t*>(0x0803f000), 0x800);
	static STM32StorageBank right(reinterpret_cast<uint8_t*>(0x0803f800), 0x800);
	InitKVS(&left, &right, 32);

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

void InitSPI()
{
	g_log("Initializing SPI\n");

	//Set up GPIOs for display bus
	static GPIOPin display_sck(&GPIOD, 1, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_FAST, 5);
	//PD0 used as route through, leave tristated
	static GPIOPin display_mosi(&GPIOD, 4, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_FAST, 5);

	//Set up GPIOs for FPGA bus
	static GPIOPin fpga_sck(&GPIOE, 13, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_FAST, 5);
	static GPIOPin fpga_mosi(&GPIOE, 15, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_FAST, 5);
	static GPIOPin fpga_cs_n(&GPIOB, 0, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_FAST, 5);
	//DO NOT configure MISO initially since this pin doubles as JTRST
	//If we enable it, JTAG will stop working!

	//Save the CS# pin
	g_fpgaSPICS = &fpga_cs_n;

	//Set up IRQ6 for SPI CS# (PB0) change
	//Use EXTI0 as PB0 interrupt on falling edge
	//TODO: make a wrapper for this?
	RCCHelper::EnableSyscfg();
	NVIC_EnableIRQ(6);
	SYSCFG.EXTICR1 = (SYSCFG.EXTICR1 & 0xfffffff8) | 0x1;
	EXTI.IMR1 |= 1;
	EXTI.FTSR1 |= 1;

	//Set up IRQ35 as SPI1 interrupt
	NVIC_EnableIRQ(35);
	g_fpgaSPI.EnableRxInterrupt();
}

/**
	@brief Puts the SPI MISO pin in SPI mode (disconnecting JTAG)
 */
void SetMisoToSPIMode()
{
	g_fpgaMiso.SetMode(GPIOPin::MODE_PERIPHERAL, 5);
	g_misoIsJtag = false;
}

/**
	@brief Puts the SPI MISO pin in JTAG mode (reconnecting JTAG)
 */
void SetMisoToJTAGMode()
{
	g_fpgaMiso.SetMode(GPIOPin::MODE_PERIPHERAL, 0);
	g_misoIsJtag = true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Run the firmware updater

void __attribute__((noreturn)) FirmwareUpdateFlow()
{
	g_log("In DFU mode\n");

	uint8_t nbyte = 0;
	uint8_t cmd = 0;

	const uint32_t* appVector  = reinterpret_cast<const uint32_t*>(0x8008000);

	while(1)
	{
		g_log.UpdateOffset(60000);

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

						//Launch the application
						case FRONT_BOOT_APP:
							g_log("Attempting to boot application\n");
							if(ValidateAppPartition(appVector))
								BootApplication(appVector);
							g_log("Image is invalid, remaining in DFU mode\n");
							break;

						//Start erasing flash
						case FRONT_ERASE_APP:

							if(g_misoIsJtag)
								SetMisoToSPIMode();

							//Tell client we're busy
							g_fpgaSPI.NonblockingWriteDevice(0x00);

							//Do the erase cycle
							EraseFlash();

							//We're now done
							g_fpgaSPI.NonblockingWriteDevice(0x01);
							break;

						//Commands that produce SPI output
						case FRONT_GET_STATUS:
							{
								if(g_misoIsJtag)
									SetMisoToSPIMode();

								const uint8_t tmp = FRONT_BOOTLOADER;
								g_fpgaSPI.NonblockingWriteFifo(&tmp, sizeof(tmp));
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

						default:
							g_log("Unrecognized command %02x\n", cmd);
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

	uint8_t* appStart				= reinterpret_cast<uint8_t*>(0x8008000);
	uint8_t* appEnd 				= reinterpret_cast<uint8_t*>(0x0803f000);
	const uint32_t delta			= appEnd - appStart;
	const uint32_t eraseBlockSize	= 2048;
	const uint32_t nblocks 			= delta / eraseBlockSize;

	auto start = g_logTimer.GetCount();
	for(uint32_t i=0; i<nblocks; i++)
	{
		//Discard any SPI data events sent during this time
		while(g_fpgaSPI.HasEvents())
		{ g_fpgaSPI.GetEvent(); }

		if( (i % 10) == 0)
		{
			auto dt = g_logTimer.GetCount() - start;
			g_log("Block %u / %u (elapsed %d.%d ms)\n", i, nblocks, dt / 10, dt % 10);
		}

		Flash::BlockErase(appStart + i*nblocks);
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
	const char* crcKey = "firmware.crc";
	const char* firmwareVer = nullptr;
	bool updatedViaJtag = IsAppUpdated(firmwareVer);
	if(!updatedViaJtag)
	{
		auto hlog = g_kvs->FindObject(crcKey);
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
		g_kvs->StoreObject(crcKey, (const uint8_t*)&crc, sizeof(crc));
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
		g_log(Logger::ERROR, "CRC mismatch, application partition flash corruption?");
		return false;
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Minimal hardware init

void InitPower()
{
	RCCHelper::Enable(&PWR);
	Power::ConfigureLDO(RANGE_VOS1);
}

void InitClocks()
{
	//Configure the flash with wait states and prefetching before making any changes to the clock setup.
	//A bit of extra latency is fine, the CPU being faster than flash is not.
	Flash::SetConfiguration(80, RANGE_VOS1);

	RCCHelper::InitializePLLFromHSI16(
		2,	//Pre-divide by 2 (PFD frequency 8 MHz)
		20,	//VCO at 8*20 = 160 MHz
		4,	//Q divider is 40 MHz (nominal 48 but we're not using USB so this is fine)
		2,	//R divider is 80 MHz (fmax for CPU)
		1,	//no further division from SYSCLK to AHB (80 MHz)
		2,	//APB1 at 40 MHz
		2);	//APB2 at 40 MHz
}

void InitUART()
{
	//Initialize the UART for local console: 115.2 Kbps using PD4 for USART2 transmit and PD5 for USART2 receive
	//TODO: nice interface for enabling UART interrupts
	GPIOPin uart_tx(&GPIOD, 5, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_SLOW, 7);
	GPIOPin uart_rx(&GPIOD, 6, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_SLOW, 7);

	//Enable the UART interrupt
	NVIC_EnableIRQ(38);
}

void InitLog()
{
	//Wait 10ms to avoid resets during shutdown from destroying diagnostic output
	g_logTimer.Sleep(100);

	//Clear screen and move cursor to X0Y0
	g_uart.Printf("\x1b[2J\x1b[0;0H");

	//Start the logger
	g_log.Initialize(&g_uart, &g_logTimer);
	g_log("Front panel bootloader (%s %s)\n", __DATE__, __TIME__);
}

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
