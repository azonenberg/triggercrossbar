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

#include <core/platform.h>
#include <hwinit.h>

#include <peripheral/DTS.h>
#include <peripheral/Power.h>
#include <peripheral/SPI.h>

#include <embedded-utils/StringBuffer.h>

#include "net/ManagementTCPProtocol.h"
#include "net/ManagementUDPProtocol.h"
#include "OctalDAC.h"

extern ManagementNTPClient* g_ntpClient;

extern DigitalTempSensor* g_dts;

extern GPIOPin* g_leds[4];

void InitLEDs();
void InitDTS();
void InitDACs();
void InitSupervisor();
void InitSensors();
void InitRelays();

uint16_t GetFanRPM(uint8_t channel);
uint16_t GetFPGATemperature();
uint16_t GetFPGAVCCINT();
uint16_t GetFPGAVCCAUX();
uint16_t GetFPGAVCCBRAM();

void UpdateFrontPanelDisplay();

uint16_t SupervisorRegRead(uint8_t regid);

extern OctalDAC* g_rxDacs[2];
extern OctalDAC* g_txDac;

extern SPI<64, 64> g_superSPI;
extern GPIOPin* g_superSPICS;

extern bool g_displayRefreshPending;

extern char g_superVersion[20];
extern char g_ibcVersion[20];

void SetFrontPanelDirectionLEDs(uint8_t leds);
void SendFrontPanelByte(uint8_t data);
uint8_t ReadFrontPanelByte();
uint8_t GetFrontPanelMode();

extern ManagementSSHTransportServer* g_sshd;

void LoadChannelConfig();
void SaveChannelConfig();

extern bool g_frontPanelDFUInProgress;
bool IsFrontPanelDFU();

#define DISPLAY_NAME_MAX 32
extern char g_inputDisplayNames[8][DISPLAY_NAME_MAX];
extern char g_outputDisplayNames[8][DISPLAY_NAME_MAX];
extern char g_bidirDisplayNames[4][DISPLAY_NAME_MAX];

//SFRs on the FPGA
extern volatile APB_RelayController* g_relayController;
extern volatile APB_GPIO* g_ledGpioInPortActivity;
extern volatile APB_GPIO* g_ledGpioOutPortActivity;
extern volatile APB_SPIHostInterface* g_frontPanelSPI;
extern APB_SPIHostInterfaceDriver* g_frontSPI;
extern volatile APB_BERTConfig*	g_bertConfig[2];
extern volatile APB_SerdesDRP* g_bertDRP[2];
extern volatile LogicAnalyzer* g_logicAnalyzer[2];

#endif
