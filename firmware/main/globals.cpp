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
	@brief Interface to the FPGA via legacy bus protocol
 */
FPGAInterface* g_fpga = nullptr;

/**
	@brief Interface to the FPGA via APB
 */
APBFPGAInterface g_apbfpga;

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

	Default after reset is for UART4 to be clocked by PCLK1 (APB1 clock) which is 62.5 MHz
	So we need a divisor of 542.53
 */
UART<32, 256> g_cliUART(&UART4, 543);

/**
	@brief Digital temperature sensor
 */
DigitalTempSensor* g_dts = nullptr;

///@brief MAC address I2C EEPROM
I2C* g_macI2C = nullptr;

///@brief SFP+ DOM / ID EEPROM
I2C* g_sfpI2C = nullptr;

///@brief GPIO LEDs
GPIOPin* g_leds[4] = {0};

///@brief SFP mod_abs
GPIOPin* g_sfpModAbsPin = nullptr;

///@brief SFP tx_disable
GPIOPin* g_sfpTxDisablePin = nullptr;

///@brief SFP tx_fault
GPIOPin* g_sfpTxFaultPin = nullptr;

///@brief SFP laser fault detected
bool g_sfpFaulted = false;

///@brief SFP module inserted (does not imply link is up)
bool g_sfpPresent = false;

///@brief FPGA die serial number
uint8_t g_fpgaSerial[8] = {0};

///@brief DACs for RX channels
OctalDAC* g_rxDacs[2] = {nullptr, nullptr};

///@brief DACs for TX channels
OctalDAC* g_txDac = nullptr;

///@brief BaseT link status
bool g_basetLinkUp = false;

//Ethernet link speed
uint8_t g_basetLinkSpeed = 0;

//TODO: SFP+ link status

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

///@brief USERCODE of the FPGA (build timestamp)
uint32_t g_usercode = 0;

///@brief Request refresh of the display if link state changes
bool g_displayRefreshPending = false;

///@brief SFP+ link state
bool g_sfpLinkUp;

///@brief Key manager
CrossbarSSHKeyManager g_keyMgr;

///@brief The single supported SSH username
char g_sshUsername[CLI_USERNAME_MAX] = "";

///@brief KVS key for the SSH username
const char* g_usernameObjectID = "ssh.username";

///@brief The SSH server
ManagementSSHTransportServer* g_sshd = nullptr;

///@brief Default SSH username if not configured
const char* g_defaultSshUsername = "admin";

///@brief Selects whether the DHCP client is active or not
bool g_usingDHCP = false;

///@brief The DHCP client
ManagementDHCPClient* g_dhcpClient = nullptr;

///@brief The NTP client
ManagementNTPClient* g_ntpClient = nullptr;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Memory mapped SFRs on the FPGA

//TODO: use linker script to locate these rather than this ugly pointer code?

///@brief System information
volatile APB_SystemInfo* g_sysInfo =
	reinterpret_cast<volatile APB_SystemInfo*>(FPGA_MEM_BASE + BASE_SYSINFO);

///@brief Relay controller
volatile APB_RelayController* g_relayController =
	reinterpret_cast<volatile APB_RelayController*>(FPGA_MEM_BASE + BASE_RELAY);

///@brief Ethernet RX buffer
volatile ManagementRxFifo* g_ethRxFifo =
	reinterpret_cast<volatile ManagementRxFifo*>(FPGA_MEM_BASE + BASE_ETH_RX);

///@brief Ethernet TX buffers
volatile ManagementTxFifo* g_eth1GTxFifo =
	reinterpret_cast<volatile ManagementTxFifo*>(FPGA_MEM_BASE + BASE_1G_TX);
volatile ManagementTxFifo* g_eth10GTxFifo =
	reinterpret_cast<volatile ManagementTxFifo*>(FPGA_MEM_BASE + BASE_XG_TX);
