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

/**
	@file
	@author	Andrew D. Zonenberg
	@brief	Boot-time hardware initialization
 */
#include <core/platform.h>
#include "hwinit.h"
#include "LogSink.h"
#include <peripheral/Power.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Common peripherals used by application and bootloader

//APB1 is 62.5 MHz but default is for timer clock to be 2x the bus clock (see table 53 of RM0468)
//Divide down to get 10 kHz ticks
Timer g_logTimer(&TIM2, Timer::FEATURE_GENERAL_PURPOSE, 12500);

///@brief Character device for logging
LogSink<MAX_LOG_SINKS>* g_logSink = nullptr;

/**
	@brief UART console

	Default after reset is for UART4 to be clocked by PCLK1 (APB1 clock) which is 62.5 MHz
	So we need a divisor of 542.53
 */
UART<32, 256> g_cliUART(&UART4, 543);

///@brief Global Ethernet interface
EthernetInterface* g_ethIface = nullptr;

///@brief Interface to the FPGA via APB
APBFPGAInterface g_apbfpga;

///@brief Our MAC address
MACAddress g_macAddress;

///@brief Our IPv4 address
IPv4Config g_ipConfig;

///@brief Ethernet protocol stack
EthernetProtocol* g_ethProtocol = nullptr;

///@brief QSPI interface to FPGA
OctoSPI* g_qspi = nullptr;

///@brief MAC address I2C EEPROM
I2C* g_macI2C = nullptr;

///@brief SFP+ DOM / ID EEPROM
I2C* g_sfpI2C = nullptr;

///@brief BaseT link status
bool g_basetLinkUp = false;

//Ethernet link speed
uint8_t g_basetLinkSpeed = 0;

///@brief SFP+ link state
bool g_sfpLinkUp;

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

///@brief Key manager
CrossbarSSHKeyManager g_keyMgr;

///@brief The single supported SSH username
char g_sshUsername[CLI_USERNAME_MAX] = "";

///@brief KVS key for the SSH username
const char* g_usernameObjectID = "ssh.username";

///@brief Default SSH username if not configured
const char* g_defaultSshUsername = "admin";

///@brief Selects whether the DHCP client is active or not
bool g_usingDHCP = false;

///@brief The DHCP client
ManagementDHCPClient* g_dhcpClient = nullptr;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Memory mapped SFRs on the FPGA

//TODO: use linker script to locate these rather than this ugly pointer code?

///@brief MDIO interface
volatile APB_MDIO* g_mdio =
	reinterpret_cast<volatile APB_MDIO*>(FPGA_MEM_BASE + BASE_MDIO);

///@brief Curve25519 controller
volatile APB_Curve25519* g_curve25519 =
	reinterpret_cast<volatile APB_Curve25519*>(FPGA_MEM_BASE + BASE_25519);

///@brief Interrupt status
volatile uint16_t* g_irqStat =
	reinterpret_cast<volatile uint16_t*>(FPGA_MEM_BASE + BASE_IRQ_STAT);

///@brief Ethernet RX buffer
volatile ManagementRxFifo* g_ethRxFifo =
	reinterpret_cast<volatile ManagementRxFifo*>(FPGA_MEM_BASE + BASE_ETH_RX);

///@brief Ethernet TX buffers
volatile ManagementTxFifo* g_eth1GTxFifo =
	reinterpret_cast<volatile ManagementTxFifo*>(FPGA_MEM_BASE + BASE_1G_TX);
volatile ManagementTxFifo* g_eth10GTxFifo =
	reinterpret_cast<volatile ManagementTxFifo*>(FPGA_MEM_BASE + BASE_XG_TX);


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Do other initialization

void BSP_Init()
{
	InitRTC();
	App_Init();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// BSP overrides for low level init

void BSP_InitPower()
{
	//Initialize power (must be the very first thing done after reset)
	Power::ConfigureSMPSToLDOCascade(Power::VOLTAGE_1V8, RANGE_VOS0);
}

void BSP_InitClocks()
{
	//With CPU_FREQ_BOOST not set, max frequency is 520 MHz

	//Configure the flash with wait states and prefetching before making any changes to the clock setup.
	//A bit of extra latency is fine, the CPU being faster than flash is not.
	Flash::SetConfiguration(513, RANGE_VOS0);

	//By default out of reset, we're clocked by the HSI clock at 64 MHz
	//Initialize the external clock source at 25 MHz
	RCCHelper::EnableHighSpeedExternalClock();

	//Set up PLL1 to run off the external oscillator
	RCCHelper::InitializePLL(
		1,		//PLL1
		25,		//input is 25 MHz from the HSE
		2,		//25/2 = 12.5 MHz at the PFD
		40,		//12.5 * 40 = 500 MHz at the VCO
		1,		//div P (primary output 500 MHz)
		10,		//div Q (50 MHz kernel clock)
		32,		//div R (not used for now),
		RCCHelper::CLOCK_SOURCE_HSE
	);

	//Set up main system clock tree
	RCCHelper::InitializeSystemClocks(
		1,		//sysclk = 500 MHz
		2,		//AHB = 250 MHz
		4,		//APB1 = 62.5 MHz
		4,		//APB2 = 62.5 MHz
		4,		//APB3 = 62.5 MHz
		4		//APB4 = 62.5 MHz
	);

	//RNG clock should be >= HCLK/32
	//AHB2 HCLK is 250 MHz so min 7.8125 MHz
	//Select PLL1 Q clock (50 MHz)
	RCC.D2CCIP2R = (RCC.D2CCIP2R & ~0x300) | (0x100);

	//Select PLL1 as system clock source
	RCCHelper::SelectSystemClockFromPLL1();
}

void BSP_InitUART()
{
	//Initialize the UART for local console: 115.2 Kbps using PA12 for UART4 transmit and PA11 for UART2 receive
	//TODO: nice interface for enabling UART interrupts
	static GPIOPin uart_tx(&GPIOA, 12, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_SLOW, 6);
	static GPIOPin uart_rx(&GPIOA, 11, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_SLOW, 6);

	//Enable the UART interrupt
	NVIC_EnableIRQ(52);

	g_logTimer.Sleep(10);	//wait for UART pins to be high long enough to remove any glitches during powerup

	//Clear screen and move cursor to X0Y0
	g_cliUART.Printf("\x1b[2J\x1b[0;0H");
}

void BSP_InitLog()
{
	static LogSink<MAX_LOG_SINKS> sink(&g_cliUART);
	g_logSink = &sink;

	g_log.Initialize(g_logSink, &g_logTimer);
	g_log("trigger-crossbar by Andrew D. Zonenberg\n");
	{
		LogIndenter li(g_log);
		g_log("This system is open hardware! Board design files and firmware/gateware source code are at:\n");
		g_log("https://github.com/azonenberg/triggercrossbar\n");
	}
	g_log("Firmware compiled at %s on %s\n", __TIME__, __DATE__);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Higher level initialization we used for a lot of stuff

void InitRTC()
{
	g_log("Initializing RTC...\n");
	LogIndenter li(g_log);
	g_log("Using external clock divided by 50 (500 kHz)\n");

	//Turn on the RTC APB clock so we can configure it, then set the clock source for it in the RCC
	RCCHelper::Enable(&_RTC);
	RTC::SetClockFromHSE(50);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Placeholders for bootloader or app to override

void __attribute__((weak)) OnEthernetLinkStateChanged()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SFR access

/**
	@brief Block until the status register (mapped at two locations) matches the target
 */
void StatusRegisterMaskedWait(volatile uint16_t* a, volatile uint16_t* b, uint16_t mask, uint16_t target)
{
	asm("dmb st");

	while(true)
	{
		uint16_t va = *a;
		uint16_t vb = *b;
		asm("dmb");

		if( ( (va & mask) == target) && ( (vb & mask) == target) )
			return;
	}
}

/**
	@brief Memcpy-like that does 64-bit copies as much as possible and always copies from LSB to MSB

	Assumes dst is 64-bit aligned and src is at least 32-bit aligned
 */
void SfrMemcpy(volatile void* dst, void* src, uint32_t len)
{
	volatile uint64_t* dst64 = reinterpret_cast<volatile uint64_t*>(dst);
	uint64_t* src64 = reinterpret_cast<uint64_t*>(src);

	uint32_t leftover = (len % 8);
	uint32_t blocklen = len - leftover;

	//64-bit block copy
	for(uint32_t i=0; i<blocklen; i++)
		dst64[i] = src64[i];

	//32-bit block copy
	volatile uint32_t* dst32 = reinterpret_cast<volatile uint32_t*>(dst64 + blocklen);
	uint32_t* src32 = reinterpret_cast<uint32_t*>(src64 + blocklen);
	if(leftover >= 4)
	{
		*dst32 = *src32;
		dst32 ++;
		src32 ++;
		leftover -= 4;
	}

	//16-bit block copy
	volatile uint16_t* dst16 = reinterpret_cast<volatile uint16_t*>(dst32);
	uint16_t* src16 = reinterpret_cast<uint16_t*>(src32);
	if(leftover >= 2)
	{
		*dst16 = *src16;
		dst16 ++;
		src16 ++;
		leftover -= 2;
	}

	volatile uint8_t* dst8 = reinterpret_cast<volatile uint8_t*>(dst16);
	uint8_t* src8 = reinterpret_cast<uint8_t*>(src16);
	if(leftover)
		*dst8 = *src8;
}
