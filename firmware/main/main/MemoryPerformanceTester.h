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
	@brief Declaration of MemoryPerformanceTester
 */
#ifndef MemoryPerformanceTester_h
#define MemoryPerformanceTester_h

#include <peripheral/DWT.h>

#pragma GCC push_options
#pragma GCC optimize "-O3"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"

extern "C" void MemReadTestX8(volatile void* ptr, uint32_t len_words);
extern "C" void MemReadTestX16(volatile void* ptr, uint32_t len_words);
extern "C" void MemReadTestX32(volatile void* ptr, uint32_t len_words);
extern "C" void MemReadTestX64(volatile void* ptr, uint32_t len_words);

extern "C" void MemWriteTestX8(volatile void* ptr, uint32_t len_words);
extern "C" void MemWriteTestX16(volatile void* ptr, uint32_t len_words);
extern "C" void MemWriteTestX32(volatile void* ptr, uint32_t len_words);
extern "C" void MemWriteTestX64(volatile void* ptr, uint32_t len_words);

class MemoryPerformanceTester
{
public:

	static void DoTestX8(volatile uint8_t* buf, size_t size)
	{
		//Write test
		asm("dmb");
		uint32_t start = _DWT.CYCCNT;
		MemWriteTestX8(buf, size);
		asm("dmb");
		uint32_t end = _DWT.CYCCNT;
		uint32_t dt = end-start;
		g_log("Fill:      %d\n", dt);

		asm("dmb");
		start = _DWT.CYCCNT;
		MemReadTestX8(buf, size);
		asm("dmb");
		end = _DWT.CYCCNT;
		dt = end-start;
		g_log("Readback:  %d\n", dt);
	}

	static void DoTestX16(volatile uint16_t* buf, size_t size)
	{
		//Write test
		asm("dmb");
		uint32_t start = _DWT.CYCCNT;
		MemWriteTestX16(buf, size);
		asm("dmb");
		uint32_t end = _DWT.CYCCNT;
		uint32_t dt = end-start;
		g_log("Fill:      %d\n", dt);

		asm("dmb");
		start = _DWT.CYCCNT;
		MemReadTestX16(buf, size);
		asm("dmb");
		end = _DWT.CYCCNT;
		dt = end-start;
		g_log("Readback:  %d\n", dt);
	}

	static void DoTestX32(volatile uint32_t* buf, size_t size)
	{
		//Write test
		asm("dmb");
		uint32_t start = _DWT.CYCCNT;
		MemWriteTestX32(buf, size);
		asm("dmb");
		uint32_t end = _DWT.CYCCNT;
		uint32_t dt = end-start;
		g_log("Fill:      %d\n", dt);

		asm("dmb");
		start = _DWT.CYCCNT;
		MemReadTestX32(buf, size);
		asm("dmb");
		end = _DWT.CYCCNT;
		dt = end-start;
		g_log("Readback:  %d\n", dt);
	}

	static void DoTestX64(volatile uint64_t* buf, size_t size)
	{
		//Write test
		asm("dmb");
		uint32_t start = _DWT.CYCCNT;
		MemWriteTestX64(buf, size);
		asm("dmb");
		uint32_t end = _DWT.CYCCNT;
		uint32_t dt = end-start;
		g_log("Fill:      %d\n", dt);

		asm("dmb");
		start = _DWT.CYCCNT;
		MemReadTestX64(buf, size);
		asm("dmb");
		end = _DWT.CYCCNT;
		dt = end-start;
		g_log("Readback:  %d\n", dt);
	}
};

#pragma GCC diagnostic pop
#pragma GCC pop_options

#endif
