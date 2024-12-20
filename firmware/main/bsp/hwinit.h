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

#ifndef hwinit_h
#define hwinit_h

#include <cli/UARTOutputStream.h>

#include <peripheral/CRC.h>
#include <peripheral/Flash.h>
#include <peripheral/GPIO.h>
#include <peripheral/I2C.h>
#include <peripheral/MPU.h>
#include <peripheral/OctoSPI.h>
#include <peripheral/OctoSPIManager.h>
#include <peripheral/RTC.h>
#include <peripheral/UART.h>

#include <microkvs/driver/STM32StorageBank.h>

#include <staticnet-config.h>

#include <staticnet/stack/staticnet.h>
#include <staticnet/ssh/SSHTransportServer.h>

#include "ManagementDHCPClient.h"
#include <staticnet/drivers/apb/APBEthernetInterface.h>

//SFR poll (ignore warnings related to this, we know the alignment of the SFRs are correct)
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
void StatusRegisterMaskedWait(volatile uint32_t* a, volatile uint32_t* b, uint32_t mask, uint32_t target);

//Memcpy-like function that uses indirect OCTOSPI register access
void SfrMemcpy(volatile void* dst, void* src, uint32_t len);

#include "FPGAInterface.h"
#include "APBFPGAInterface.h"
#include <embedded-utils/APB_SpiFlashInterface.h>
#include "CrossbarSSHKeyManager.h"

#include <bootloader/BootloaderAPI.h>

#define MAX_LOG_SINKS SSH_TABLE_SIZE
#include <LogSink.h>
extern LogSink<MAX_LOG_SINKS>* g_logSink;

void App_Init();
void InitRTC();
void InitQSPI();
void InitFPGA();
void InitFPGAFlash();
void InitI2C();
void InitEEPROM();
void InitEthernet();
void InitIP();
void InitSFP();
void InitManagementPHY();
void PollSFP();
void PollFPGA();
void PollPHYs();
void ConfigureIP();
void DoInitKVS();

uint16_t GetSFPTemperature();
uint16_t GetSFP3V3();

//Common hardware interface stuff (mostly Ethernet related)
extern UART<32, 256> g_cliUART;
extern APBFPGAInterface g_apbfpga;
extern APBEthernetInterface g_ethIface;
extern MACAddress g_macAddress;
extern IPv4Config g_ipConfig;
extern IPv6Config g_ipv6Config;
extern bool g_usingDHCP;
extern ManagementDHCPClient* g_dhcpClient;
extern OctoSPI* g_qspi;
extern I2C* g_macI2C;
extern I2C* g_sfpI2C;
extern EthernetProtocol* g_ethProtocol;
extern bool g_basetLinkUp;
extern uint8_t g_basetLinkSpeed;
extern bool g_sfpLinkUp;
extern CrossbarSSHKeyManager g_keyMgr;
extern GPIOPin* g_sfpModAbsPin;
extern GPIOPin* g_sfpTxDisablePin;
extern GPIOPin* g_sfpTxFaultPin;
extern bool g_sfpFaulted;
extern bool g_sfpPresent;
extern GPIOPin g_irq;

extern uint8_t g_fpgaSerial[8];
extern uint32_t g_usercode;

extern const char* g_defaultSshUsername;
extern const char* g_usernameObjectID;
extern char g_sshUsername[CLI_USERNAME_MAX];

//IP address configuration
extern const IPv4Address g_defaultIP;
extern const IPv4Address g_defaultNetmask;
extern const IPv4Address g_defaultBroadcast;
extern const IPv4Address g_defaultGateway;

//SFRs on the FPGA used by both bootloader and application
extern volatile APB_SystemInfo* g_sysInfo;
extern volatile APB_MDIO* g_mdio;
extern volatile APB_EthernetRxBuffer* g_ethRxFifo;
extern volatile APB_EthernetTxBuffer_10G* g_eth1GTxFifo;
extern volatile APB_EthernetTxBuffer_10G* g_eth10GTxFifo;
extern volatile APB_Curve25519* g_curve25519;
extern volatile uint16_t* g_irqStat;
extern volatile APB_SPIHostInterface* g_flashSpi;

//Backup SRAM used for communication with bootloader
extern volatile BootloaderBBRAM* g_bbram;

void UART4_Handler();

void OnEthernetLinkStateChanged();
bool CheckForFPGAEvents();
void TrimSpaces(char* str);
void RegisterProtocolHandlers(IPv4Protocol& ipv4);

//Global device controllers
extern APB_SpiFlashInterface* g_fpgaFlash;
extern MDIODevice g_mgmtPhy;

#endif
