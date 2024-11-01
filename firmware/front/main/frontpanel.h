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

#ifndef frontpanel_h
#define frontpanel_h

#include <core/platform.h>

#include <peripheral/ADC.h>
#include <peripheral/I2C.h>

#include <embedded-utils/FIFO.h>
#include <embedded-utils/StringBuffer.h>

#include <bootloader/BootloaderAPI.h>
#include "../bsp/hwinit.h"
#include "TCA6424A.h"
#include "Display.h"

#include <etl/vector.h>

void InitGPIOs();
void InitI2C();
void InitSensors();
void InitExpander();
void InitSPI();
void InitDisplay();

extern UART<16, 256> g_uart;
extern I2C g_i2c;
extern TCA6424A* g_expander;
extern DisplaySPIType g_displaySPI;
extern Display* g_display;

uint16_t ReadThermalSensor(uint8_t addr);
extern const uint8_t g_tempI2cAddress;

extern GPIOPin* g_inmodeLED[4];
extern GPIOPin* g_outmodeLED[4];

//Display state
extern int g_linkSpeed;
extern uint8_t g_ipv4Addr[4];
extern uint16_t g_ipv6Addr[8];
extern uint8_t g_serial[8];
extern uint16_t g_fpgaTemp;
extern uint16_t g_mcuTemp;
extern uint16_t g_ibcTemp;
extern uint16_t g_vin;
extern uint16_t g_iin;
extern uint16_t g_vout;
extern uint16_t g_iout;
extern uint16_t g_fanspeed;
extern uint16_t g_ipv4SubnetSize;
extern uint16_t g_ipv6SubnetSize;
extern char g_mcuFirmware[20];
extern char g_ibcFirmware[20];
extern char g_superFirmware[20];
extern char g_fpgaFirmware[20];
extern bool	g_mainMCUDown;
extern char g_dataTimestamp[20];
extern bool g_staticIP;

void SetMisoToSPIMode();
void SetMisoToJTAGMode();

#define MAX_TASKS 4
extern etl::vector<Task*, MAX_TASKS>  g_tasks;

//Timer tasks are a strict subset of total tasks
#define MAX_TIMER_TASKS 2
extern etl::vector<TimerTask*, MAX_TIMER_TASKS>  g_timerTasks;

#endif
