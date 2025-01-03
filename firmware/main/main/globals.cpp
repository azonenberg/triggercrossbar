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

///@brief GPIO LEDs
GPIOPin* g_leds[4] = {0};

///@brief DACs for RX channels
OctalDAC* g_rxDacs[2] = {nullptr, nullptr};

///@brief DACs for TX channels
OctalDAC* g_txDac = nullptr;

/**
	@brief SPI bus to supervisor

	SPI4 runs on spi 4/5 kernel clock domain
	default after reset is APB2 clock which is 62.5 MHz, divide by 128 to get 488 kHz
 */
SPI<64, 64> g_superSPI(&SPI4, true, 128);

///@brief Chip select for supervisor CS
GPIOPin* g_superSPICS = nullptr;

///@brief Version string for supervisor MCU
char g_superVersion[20] = {0};

///@brief Version string for IBC MCU
char g_ibcVersion[20] = {0};

///@brief Request refresh of the display if link state changes
bool g_displayRefreshPending = false;

///@brief The NTP client
STM32NTPClient* g_ntpClient = nullptr;

///@brief The SSH server
ManagementSSHTransportServer* g_sshd = nullptr;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Memory mapped SFRs on the FPGA

//TODO: use linker script to locate these rather than this ugly pointer code?

APB_SPIHostInterfaceDriver* g_frontSPI = nullptr;

///@brief Low speed configuration for BERT channels
volatile APB_BERTConfig* g_bertConfig[2] = { &FBERT0, &FBERT1 };

///@brief DRP access for BERT channels
volatile APB_SerdesDRP* g_bertDRP[2] = { &FDRP0, &FDRP1 };

///@brief Logic analyzers
volatile LogicAnalyzer* g_logicAnalyzer[2] = { &FLA0, &FLA1 };
