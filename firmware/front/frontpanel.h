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

#include <platform.h>

#include <peripheral/ADC.h>
#include <peripheral/I2C.h>

#include <util/FIFO.h>
#include <util/StringBuffer.h>

#include "../bl-front/BootloaderAPI.h"
#include "../bsp-front/hwinit.h"
#include "TCA6424A.h"
#include "Display.h"

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

void SetMisoToSPIMode();
void SetMisoToJTAGMode();

#endif
