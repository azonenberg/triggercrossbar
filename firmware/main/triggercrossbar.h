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

#ifndef triggercrossbar_h
#define triggercrossbar_h

#include "stm32.h"
#include <peripheral/CRC.h>
#include <peripheral/DTS.h>
#include <peripheral/Flash.h>
#include <peripheral/GPIO.h>
#include <peripheral/I2C.h>
#include <peripheral/MPU.h>
#include <peripheral/OctoSPI.h>
#include <peripheral/OctoSPIManager.h>
#include <peripheral/Power.h>
#include <peripheral/RCC.h>
#include <peripheral/RTC.h>
#include <peripheral/SPI.h>
#include <peripheral/Timer.h>
#include <peripheral/UART.h>
#include <cli/UARTOutputStream.h>

#include "LogSink.h"

#include <microkvs/kvs/KVS.h>

#include <util/Logger.h>
#include <util/StringBuffer.h>

#include <staticnet-config.h>
#include <staticnet/stack/staticnet.h>
#include <staticnet/ssh/SSHTransportServer.h>

//SFR poll (ignore warnings related to this, we know the alignment of the SFRs are correct)
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
void StatusRegisterMaskedWait(volatile uint16_t* a, volatile uint16_t* b, uint16_t mask, uint16_t target);

#include "net/ManagementTCPProtocol.h"
#include "net/ManagementUDPProtocol.h"
#include "FPGAInterface.h"
#include "APBFPGAInterface.h"
#include "OctalDAC.h"

#include "ssh/CrossbarSSHKeyManager.h"

#define MAX_LOG_SINKS SSH_TABLE_SIZE

extern KVS* g_kvs;
extern LogSink<MAX_LOG_SINKS>* g_logSink;
extern Logger g_log;
extern APBFPGAInterface g_apbfpga;
extern Timer* g_logTimer;
extern EthernetInterface* g_ethIface;
extern MACAddress g_macAddress;
extern IPv4Config g_ipConfig;
extern bool g_usingDHCP;
extern ManagementDHCPClient* g_dhcpClient;
extern ManagementNTPClient* g_ntpClient;
extern EthernetProtocol* g_ethProtocol;
extern I2C* g_macI2C;
extern I2C* g_sfpI2C;

extern UART<32, 256> g_cliUART;
extern OctoSPI* g_qspi;

extern DigitalTempSensor* g_dts;

extern GPIOPin* g_leds[4];

extern GPIOPin* g_sfpModAbsPin;
extern GPIOPin* g_sfpTxDisablePin;
extern GPIOPin* g_sfpTxFaultPin;
extern bool g_sfpFaulted;
extern bool g_sfpPresent;

extern const IPv4Address g_defaultIP;
extern const IPv4Address g_defaultNetmask;
extern const IPv4Address g_defaultBroadcast;
extern const IPv4Address g_defaultGateway;

void InitClocks();
void InitLEDs();
void InitTimer();
void InitUART();
void InitLog(CharacterDevice* logdev, Timer* timer);

void InitDTS();
void InitQSPI();
void InitFPGA();
void InitRTC();

void InitI2C();
void InitEEPROM();
void InitDACs();
void InitSupervisor();

void InitSensors();

void InitSFP();
void PollSFP();
void InitManagementPHY();
void PollFPGA();
void PollPHYs();

uint16_t GetFanRPM(uint8_t channel);
uint16_t GetFPGATemperature();
uint16_t GetFPGAVCCINT();
uint16_t GetFPGAVCCAUX();
uint16_t GetFPGAVCCBRAM();
uint16_t GetSFPTemperature();

void UpdateFrontPanelDisplay();

void InitKVS(StorageBank* left, StorageBank* right, uint32_t logsize);

void InitEthernet();
void InitIP();
void ConfigureIP();

void DetectHardware();

uint16_t ManagementPHYRead(uint8_t regid);
uint16_t ManagementPHYExtendedRead(uint8_t mmd, uint8_t regid);
void ManagementPHYWrite(uint8_t regid, uint16_t regval);
void ManagementPHYExtendedWrite(uint8_t regid, uint8_t mmd, uint16_t regval);

uint16_t SupervisorRegRead(uint8_t regid);

enum mdioreg_t
{
	//IEEE defined registers
	REG_BASIC_CONTROL			= 0x0000,
	REG_BASIC_STATUS			= 0x0001,
	REG_PHY_ID_1				= 0x0002,
	REG_PHY_ID_2				= 0x0003,
	REG_AN_ADVERT				= 0x0004,
	REG_GIG_CONTROL				= 0x0009,

	//Extended register access
	REG_PHY_REGCR				= 0x000d,
	REG_PHY_ADDAR				= 0x000e,

	//KSZ9031 specific
	REG_KSZ9031_MDIX			= 0x001c,

	//KSZ9031 MMD 2
	REG_KSZ9031_MMD2_CLKSKEW	= 0x0008
};

extern uint8_t g_fpgaSerial[8];
extern OctalDAC* g_rxDacs[2];
extern OctalDAC* g_txDac;

extern bool g_basetLinkUp;
extern uint8_t g_basetLinkSpeed;
extern bool g_sfpLinkUp;

extern SPI<64, 64> g_superSPI;
extern GPIOPin* g_superSPICS;

extern bool g_displayRefreshPending;

extern char g_superVersion[20];
extern char g_ibcVersion[20];
extern uint32_t g_usercode;

void SetFrontPanelDirectionLEDs(uint8_t leds);
void SetFrontPanelCS(bool b);
void SendFrontPanelByte(uint8_t data);
uint8_t ReadFrontPanelByte();
uint8_t GetFrontPanelMode();

bool CheckForFPGAEvents();

extern CrossbarSSHKeyManager g_keyMgr;

extern const char* g_usernameObjectID;
extern char g_sshUsername[CLI_USERNAME_MAX];
extern ManagementSSHTransportServer* g_sshd;
extern const char* g_defaultSshUsername;

void LoadChannelConfig();
void SaveChannelConfig();

extern bool g_frontPanelDFUInProgress;
bool IsFrontPanelDFU();

#define DISPLAY_NAME_MAX 32
extern char g_inputDisplayNames[8][DISPLAY_NAME_MAX];
extern char g_outputDisplayNames[8][DISPLAY_NAME_MAX];
extern char g_bidirDisplayNames[4][DISPLAY_NAME_MAX];

//SFRs on the FPGA
extern volatile APB_SystemInfo* g_sysInfo;
extern volatile APB_RelayController* g_relayController;
extern volatile APB_MDIO* g_mdio;
extern volatile ManagementRxFifo* g_ethRxFifo;
extern volatile ManagementTxFifo* g_eth1GTxFifo;
extern volatile ManagementTxFifo* g_eth10GTxFifo;
extern volatile APB_Curve25519* g_curve25519;
extern volatile uint16_t* g_irqStat;
extern volatile APB_GPIO* g_ledGpioInPortActivity;
extern volatile APB_GPIO* g_ledGpioOutPortActivity;
extern volatile APB_SPIHostInterface* g_frontPanelSPI;
extern volatile APB_BERTConfig*	g_bertConfig[2];
extern volatile APB_SerdesDRP* g_bertDRP[2];

void SfrMemcpy(volatile void* dst, void* src, uint32_t len);

#endif
