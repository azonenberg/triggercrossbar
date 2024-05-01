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
#include <microkvs/driver/STM32StorageBank.h>
#include <algorithm>

///@brief Logging UART
///USART2 is on APB1 (40 MHz), so we need a divisor of 347.22, round to 347
UART<16, 256> g_uart(&USART2, 347);

///@brief Logging block
Logger g_log;

//APB1 is 40 MHz
//Divide down to get 10 kHz ticks
Timer g_logTimer(&TIM15, Timer::FEATURE_GENERAL_PURPOSE_16BIT, 4000);

GPIOPin* g_inmodeLED[4] = {nullptr};
GPIOPin* g_outmodeLED[4] = {nullptr};

SPI<1024, 64> g_fpgaSPI(&SPI1, true, 2, false);
GPIOPin* g_fpgaSPICS = nullptr;

///@brief Key-value store used for storing firmware version config etc
KVS* g_kvs = nullptr;

bool ValidateAppPartition(const uint32_t* appVector);
void FirmwareUpdateFlow();
void BootApplication(const uint32_t* appVector);
extern "C" void __attribute__((noreturn)) DoBootApplication(const uint32_t* appVector);

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

	/**
		@brief Use sectors 126 and 127 of flash for a for a 2 kB microkvs

		TODO: support multiple sectors since 2 kB is a little small?
	 */
	static STM32StorageBank left(reinterpret_cast<uint8_t*>(0x0803f000), 0x800);
	static STM32StorageBank right(reinterpret_cast<uint8_t*>(0x0803f800), 0x800);
	InitKVS(&left, &right, 32);

	//Application region of flash runs from the end of the bootloader (0x08008000)
	//to the start of the KVS (0x0803f000), so 220 kB
	//Firmware version string is put right after vector table by linker script at a constant address
	const uint32_t* appVector  = reinterpret_cast<const uint32_t*>(0x8008000);
	if(!ValidateAppPartition(appVector))
		FirmwareUpdateFlow();

	//TODO: somehow detect (via bbram) boot failures/segfaults in the app image and stay in bootloader mode
	//TODO: have some config for the app to explicitly request us to stay in bootloader mode

	//If we get to this point, the image is valid, boot it
	BootApplication(appVector);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Run the firmware updater

void __attribute__((noreturn)) FirmwareUpdateFlow()
{
	g_log("Hanging until we implement a reflash flow\n");
	while(1)
	{}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Boot the application

void __attribute__((noreturn)) BootApplication(const uint32_t* appVector)
{
	//Print our final log message and flush the transmit FIFO before transferring control to the application
	g_log("Booting application...\n\n");
	g_uart.Flush();

	DoBootApplication(appVector);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Application booting

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

	//Image is present, see if we have a good version string
	const char* firmwareVer = reinterpret_cast<const char*>(0x80081a0);
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
	const char* imageVersionKey = "firmware.imageVersion";
	bool updatedViaJtag = false;
	auto hlog = g_kvs->FindObject(imageVersionKey);
	if(hlog)
	{
		char knownVersion[33] = {0};
		strncpy(knownVersion, (const char*)g_kvs->MapObject(hlog), std::min(hlog->m_len, (uint32_t)32));
		g_log("Previous image version:       %s\n", knownVersion);

		//Is this the image we are now booting?
		if(0 != strcmp(knownVersion, firmwareVer))
			updatedViaJtag = true;
	}

	//Valid image but nothing in KVS, we must have just jtagged the first firmware
	else
	{
		g_log("No previous image version information in KVS\n");
		updatedViaJtag = true;
	}

	//See if we have a saved CRC in flash
	uint32_t expectedCRC = 0;
	const char* crcKey = "firmware.crc";
	if(!updatedViaJtag)
	{
		hlog = g_kvs->FindObject(crcKey);
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

		g_kvs->StoreObject(imageVersionKey, (const uint8_t*)firmwareVer, strlen(firmwareVer));
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
