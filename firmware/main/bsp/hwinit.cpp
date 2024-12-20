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
#include <peripheral/DWT.h>
#include <peripheral/ITM.h>
#include <peripheral/Power.h>
#include <ctype.h>

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

///@brief Interface to the FPGA via APB
APBFPGAInterface g_apbfpga;

///@brief Our MAC address
MACAddress g_macAddress;

///@brief Our IPv4 address
IPv4Config g_ipConfig;

///@brief Our IPv6 address
IPv6Config g_ipv6Config;

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

///@brief USERCODE of the FPGA (build timestamp)
uint32_t g_usercode = 0;

///@brief FPGA die serial number
uint8_t g_fpgaSerial[8] = {0};

///@brief IRQ line to the FPGA
GPIOPin g_irq(&GPIOH, 6, GPIOPin::MODE_INPUT, GPIOPin::SLEW_SLOW);

///@brief The battery-backed RAM used to store state across power cycles
volatile BootloaderBBRAM* g_bbram = reinterpret_cast<volatile BootloaderBBRAM*>(&_RTC.BKP[0]);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Memory mapped SFRs on the FPGA

//TODO: use linker script to locate these rather than this ugly pointer code?

///@brief System information
volatile APB_SystemInfo* g_sysInfo =
	reinterpret_cast<volatile APB_SystemInfo*>(FPGA_MEM_BASE + BASE_SYSINFO);

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
volatile APB_EthernetRxBuffer* g_ethRxFifo =
	reinterpret_cast<volatile APB_EthernetRxBuffer*>(FPGA_MEM_BASE + BASE_ETH_RX);

///@brief Ethernet TX buffers
volatile APB_EthernetTxBuffer_10G* g_eth1GTxFifo =
	reinterpret_cast<volatile APB_EthernetTxBuffer_10G*>(FPGA_MEM_BASE + BASE_1G_TX);
volatile APB_EthernetTxBuffer_10G* g_eth10GTxFifo =
	reinterpret_cast<volatile APB_EthernetTxBuffer_10G*>(FPGA_MEM_BASE + BASE_XG_TX);

///@brief SPI flash controller
volatile APB_SPIHostInterface* g_flashSpi =
	reinterpret_cast<volatile APB_SPIHostInterface*>(FPGA_MEM_BASE + BASE_FLASH_SPI);

///@brief Controller for the MDIO interface
MDIODevice g_mgmtPhy(g_mdio, 0);

APB_SpiFlashInterface* g_fpgaFlash = nullptr;

APBEthernetInterface g_ethIface(g_ethRxFifo, g_eth10GTxFifo);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Task tables

etl::vector<Task*, MAX_TASKS>  g_tasks;
etl::vector<TimerTask*, MAX_TIMER_TASKS>  g_timerTasks;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Do other initialization

#ifdef _DEBUG
void InitTrace();
#endif

void BSP_Init()
{
	InitRTC();

	#ifdef _DEBUG
	InitTrace();
	#endif

	App_Init();
}

#ifdef _DEBUG
void InitTrace()
{
	//Enable ITM, enable PC sampling, and turn on forwarding to the TPIU
	ITM::Enable();
	DWT::EnablePCSampling(DWT::PC_SAMPLE_SLOW);
	ITM::EnableDwtForwarding();

	//Turn on ITM stimulus channel 0 for temperature logging
	ITM::EnableChannel(0);
}
#endif

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

	//Switch back to the HSI clock (in case we're already running on the PLL from the bootloader)
	RCCHelper::SelectSystemClockFromHSI();

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
		6,		//div R (83.33 MHz SWO Manchester bit clock, 41.66 Mbps data rate)
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
	if(IsBootloader())
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

void DoInitKVS()
{
	/*
		Use sectors 6 and 7 of main flash (in single bank mode) for a 128 kB microkvs

		Each log entry is 64 bytes, and we want to allocate ~50% of storage to the log since our objects are pretty
		small (SSH keys, IP addresses, etc). A 1024-entry log is a nice round number, and comes out to 64 kB or 50%,
		leaving the remaining 64 kB or 50% for data.
	 */
	static STM32StorageBank left(reinterpret_cast<uint8_t*>(0x080c0000), 0x20000);
	static STM32StorageBank right(reinterpret_cast<uint8_t*>(0x080e0000), 0x20000);
	InitKVS(&left, &right, 1024);
}

void InitI2C()
{
	g_log("Initializing I2C interfaces\n");

	static GPIOPin mac_i2c_scl(&GPIOB, 8, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_SLOW, 6, true);
	static GPIOPin mac_i2c_sda(&GPIOB, 9, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_SLOW, 6, true);

	//Default kernel clock for I2C4 is pclk4 (68.75 MHz for our current config)
	//Prescale by 16 to get 4.29 MHz
	//Divide by 40 after that to get 107 kHz
	static I2C mac_i2c(&I2C4, 16, 40);
	g_macI2C = &mac_i2c;

	static GPIOPin sfp_i2c_scl(&GPIOF, 1, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_SLOW, 4, true);
	static GPIOPin sfp_i2c_sda(&GPIOF, 0, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_SLOW, 4, true);

	//Default kernel clock for I2C2 is is APB1 clock (68.75 MHz for our current config)
	//Prescale by 16 to get 4.29 MHz
	//Divide by 40 after that to get 107 kHz
	static I2C sfp_i2c(&I2C2, 16, 40);
	g_sfpI2C = &sfp_i2c;
}

void InitQSPI()
{
	g_log("Initializing QSPI interface\n");

	//Configure the I/O manager
	OctoSPIManager::ConfigureMux(false);
	OctoSPIManager::ConfigurePort(
		1,							//Configuring port 1
		false,						//DQ[7:4] disabled
		OctoSPIManager::C1_HIGH,
		true,						//DQ[3:0] enabled
		OctoSPIManager::C1_LOW,		//DQ[3:0] from OCTOSPI1 DQ[3:0]
		true,						//CS# enabled
		OctoSPIManager::PORT_1,		//CS# from OCTOSPI1
		false,						//DQS disabled
		OctoSPIManager::PORT_1,
		true,						//Clock enabled
		OctoSPIManager::PORT_1);	//Clock from OCTOSPI1

	//Configure the I/O pins
	static GPIOPin qspi_cs_n(&GPIOE, 11, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_VERYFAST, 11);
	static GPIOPin qspi_sck(&GPIOB, 2, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_VERYFAST, 9);
	static GPIOPin qspi_dq0(&GPIOA, 2, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_VERYFAST, 6);
	static GPIOPin qspi_dq1(&GPIOB, 0, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_VERYFAST, 4);
	static GPIOPin qspi_dq2(&GPIOC, 2, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_VERYFAST, 9);
	static GPIOPin qspi_dq3(&GPIOA, 1, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_VERYFAST, 9);

	//Clock divider value
	//Default is for AHB3 bus clock to be used as kernel clock (250 MHz for us)
	//With 3.3V Vdd, we can go up to 140 MHz.
	//FPGA currently requires <= 62.5 MHz due to the RX oversampling used (4x in 250 MHz clock domain)
	//Dividing by 5 gives 50 MHz and a transfer rate of 200 Mbps
	//Dividing by 10, but DDR, gives the same throughput and works around an errata (which one??)
	uint8_t prescale = 15;

	//Configure the OCTOSPI itself
	//Original code used "instruction", but we want "address" to enable memory mapping
	static OctoSPI qspi(&OCTOSPI1, 0x02000000, prescale);
	qspi.SetDoubleRateMode(true);
	qspi.SetInstructionMode(OctoSPI::MODE_QUAD, 1);
	qspi.SetAddressMode(OctoSPI::MODE_QUAD, 3);
	qspi.SetAltBytesMode(OctoSPI::MODE_NONE);
	qspi.SetDataMode(OctoSPI::MODE_QUAD);
	qspi.SetDummyCycleCount(1);
	qspi.SetDQSEnable(false);
	qspi.SetDeselectTime(1);
	qspi.SetSampleDelay(false, true);

	//Poke MPU settings to disable caching etc on the QSPI memory range
	MPU::Configure(MPU::KEEP_DEFAULT_MAP, MPU::DISABLE_IN_FAULT_HANDLERS);
	MPU::ConfigureRegion(
		0,
		FPGA_MEM_BASE,
		MPU::SHARED_DEVICE,
		MPU::FULL_ACCESS,
		MPU::EXECUTE_NEVER,
		MPU::SIZE_16M);

	//Configure memory mapping mode
	qspi.SetMemoryMapMode(APBFPGAInterface::OP_APB_READ, APBFPGAInterface::OP_APB_WRITE);
	g_qspi = &qspi;
}

/**
	@brief Bring up the control interface to the FPGA
 */
void InitFPGA()
{
	g_log("Initializing FPGA\n");
	LogIndenter li(g_log);

	//Wait for the DONE signal to go high
	g_log("Waiting for FPGA boot\n");
	static GPIOPin fpgaDone(&GPIOF, 6, GPIOPin::MODE_INPUT, GPIOPin::SLEW_SLOW);
	while(!fpgaDone)
	{}

	//Read the FPGA IDCODE and serial number
	//Retry until we get a nonzero result indicating FPGA is up
	while(true)
	{
		uint32_t idcode = g_sysInfo->idcode;
		memcpy(g_fpgaSerial, (const void*)g_sysInfo->serial, 8);

		//If IDCODE is all zeroes, poll again
		if(idcode == 0)
			continue;

		//Print status
		switch(idcode & 0x0fffffff)
		{
			case 0x3647093:
				g_log("IDCODE: %08x (XC7K70T rev %d)\n", idcode, idcode >> 28);
				break;

			case 0x364c093:
				g_log("IDCODE: %08x (XC7K160T rev %d)\n", idcode, idcode >> 28);
				break;

			default:
				g_log("IDCODE: %08x (unknown device, rev %d)\n", idcode, idcode >> 28);
				break;
		}
		g_log("Serial: %02x%02x%02x%02x%02x%02x%02x%02x\n",
			g_fpgaSerial[7], g_fpgaSerial[6], g_fpgaSerial[5], g_fpgaSerial[4],
			g_fpgaSerial[3], g_fpgaSerial[2], g_fpgaSerial[1], g_fpgaSerial[0]);

		break;
	}

	//Read USERCODE
	g_usercode = g_sysInfo->usercode;
	g_log("Usercode: %08x\n", g_usercode);
	{
		LogIndenter li2(g_log);

		//Format per XAPP1232:
		//31:27 day
		//26:23 month
		//22:17 year
		//16:12 hr
		//11:6 min
		//5:0 sec
		int day = g_usercode >> 27;
		int mon = (g_usercode >> 23) & 0xf;
		int yr = 2000 + ((g_usercode >> 17) & 0x3f);
		int hr = (g_usercode >> 12) & 0x1f;
		int min = (g_usercode >> 6) & 0x3f;
		int sec = g_usercode & 0x3f;
		g_log("Bitstream timestamp: %04d-%02d-%02d %02d:%02d:%02d\n",
			yr, mon, day, hr, min, sec);
	}

	InitFPGAFlash();
}

void InitFPGAFlash()
{
	g_log("Initializing FPGA flash\n");
	LogIndenter li(g_log);

	static APB_SpiFlashInterface flash(g_flashSpi, 64);
	g_fpgaFlash = &flash;
}

/**
	@brief Remove spaces from trailing edge of a string
 */
void TrimSpaces(char* str)
{
	char* p = str + strlen(str) - 1;

	while(p >= str)
	{
		if(isspace(*p))
			*p = '\0';
		else
			break;

		p --;
	}
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
void StatusRegisterMaskedWait(volatile uint32_t* a, volatile uint32_t* b, uint32_t mask, uint32_t target)
{
	asm("dmb st");

	while(true)
	{
		uint32_t va = *a;
		uint32_t vb = *b;
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
