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
#include <peripheral/DTS.h>
#include <peripheral/Flash.h>
#include <peripheral/GPIO.h>
#include <peripheral/I2C.h>
#include <peripheral/OctoSPI.h>
#include <peripheral/OctoSPIManager.h>
#include <peripheral/Power.h>
#include <peripheral/RCC.h>
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

//#include "net/ManagementTCPProtocol.h"
#include "FPGAInterface.h"
#include "OctalDAC.h"

#define MAX_LOG_SINKS SSH_TABLE_SIZE

extern KVS* g_kvs;
extern LogSink<MAX_LOG_SINKS>* g_logSink;
extern Logger g_log;
extern FPGAInterface* g_fpga;
extern Timer* g_logTimer;
extern EthernetInterface* g_ethIface;
extern MACAddress g_macAddress;
extern IPv4Config g_ipConfig;
extern EthernetProtocol* g_ethProtocol;
extern I2C* g_macI2C;
extern I2C* g_sfpI2C;

extern UART* g_cliUART;
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

void InitI2C();
void InitEEPROM();
void InitDACs();

void InitSensors();

void InitSFP();
void PollSFP();
/*
void InitManagementPHY();
void PollFPGA();
void PollPHYs();

uint16_t ReadThermalSensor(uint8_t addr);*/
uint16_t GetFanRPM(uint8_t channel);
uint16_t GetFPGATemperature();
uint16_t GetFPGAVCCINT();
uint16_t GetFPGAVCCAUX();
uint16_t GetFPGAVCCBRAM();
uint16_t GetSFPTemperature();

void InitKVS(StorageBank* left, StorageBank* right, uint32_t logsize);
/*
void InitFPGAInterface();
void InitFPGA();
void InitInterfaces();
void ConfigureInterfaces();
void InitEthernet();
void InitIP();
void ConfigureIP();
void InitSSH();
*/
void DetectHardware();

#endif
