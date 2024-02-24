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

/**
	@brief Global Ethernet interface
 */
EthernetInterface* g_ethIface = nullptr;

/**
	@brief Global key-value store for persistent configuration
 */
KVS* g_kvs = nullptr;

/**
	@brief Character device for logging
 */
LogSink<MAX_LOG_SINKS>* g_logSink = nullptr;

/**
	@brief Logger for output stuff
 */
Logger g_log;

/**
	@brief Timer used by logger
 */
Timer* g_logTimer = nullptr;

/**
	@brief Interface to the FPGA
 */
FPGAInterface* g_fpga = nullptr;

/**
	@brief Our MAC address
 */
MACAddress g_macAddress;

/**
	@brief Our IPv4 address
 */
IPv4Config g_ipConfig;

/**
	@brief Ethernet protocol stack
 */
EthernetProtocol* g_ethProtocol = nullptr;

/**
	@brief QSPI interface to FPGA
 */
OctoSPI* g_qspi = nullptr;

/**
	@brief UART console
 */
UART* g_cliUART = nullptr;

/**
	@brief Digital temperature sensor
 */
DigitalTempSensor* g_dts = nullptr;

/**
	@brief MAC address I2C EEPROM
 */
I2C* g_macI2C = nullptr;
