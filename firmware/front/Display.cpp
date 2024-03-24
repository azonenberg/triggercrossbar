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

#include "frontpanel.h"
#include "Display.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction / destruction

Display::Display(SPI* spi, GPIOPin* busy_n, GPIOPin* cs_n, GPIOPin* dc, GPIOPin* rst)
	: m_spi(spi)
	, m_busy_n(busy_n)
	, m_cs_n(cs_n)
	, m_dc(dc)
	, m_rst_n(rst)
	, m_width(212)
	, m_height(104)
{
	//Deassert SPI CS#
	*m_cs_n = 1;

	g_log("Resetting display\n");

	//Reset the display, need 5ms between each cycle
	*m_rst_n = 1;
	g_logTimer->Sleep(500);
	*m_rst_n = 0;
	g_logTimer->Sleep(500);
	*m_rst_n = 1;
	g_logTimer->Sleep(500);

	//Soft reset
	SendCommand(0x00);
	SendData(0x0e);
	g_logTimer->Sleep(500);

	//Clear both bitplanes to blank
	memset(m_blackFramebuffer, 0, sizeof(m_blackFramebuffer));
	memset(m_redFramebuffer, 0, sizeof(m_redFramebuffer));

	//Fill the image with a checkerboard
	/*for(uint8_t x=0; x<m_width; x++)
	{
		for(uint8_t y=0; y<m_height; y++)
		{
			int xb = x / 8;
			int yb = y / 8;

			//BLACK bit plane: 8x8 pixel grid the whole image
			bool black = ((xb ^ yb) & 1) ? true : false;

			//RED bit plane: TBD
			bool red = false;

			SetPixel(x, y, red, black);
		}
	}*/

	//diagonal red line from bottom left up and right
	for(uint8_t x=0; x<32; x++)
		SetPixel(x, x, true, false);

	//Diagonal black line to the right of it
	for(uint8_t x=0; x<32; x++)
		SetPixel(x+32, x, false, true);

	//Refresh the image
	Refresh();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Mid level interface

/**
	@brief Sets a single pixel in the framebuffer
 */
void Display::SetPixel(uint8_t x, uint8_t y, bool red, bool black)
{
	//Image scan order: bottom left to top left is first scanline
	//bottom right to top right is last scanline
	//Total format is 104 pixels / 13 bytes per scanline, 212 scanlines
	//MSB of a byte is the lowest pixel

	//Start of the column
	uint16_t coloff = x * (m_height/8);

	//Bit and byte positions
	uint8_t nbyte = y / 8;
	uint8_t nbit = 7 - (y % 8);
	uint8_t bitmask = (1 << nbit);

	//Do the actual write operations
	if(red)
		m_redFramebuffer[coloff + nbyte] |= bitmask;
	else
		m_redFramebuffer[coloff + nbyte] &= ~bitmask;

	if(black)
		m_blackFramebuffer[coloff + nbyte] |= bitmask;
	else
		m_blackFramebuffer[coloff + nbyte] &= ~bitmask;
}

/**
	@brief Push the framebuffer to the display

	TODO: make this nonblocking
 */
void Display::Refresh()
{
	//Send temperature
	auto temp = ReadThermalSensor(g_tempI2cAddress);
	SendCommand(0xe5);
	g_log("Updating temperature (display is %d C)\n", (temp >> 8));
	SendData(temp >> 8);

	//Activate temperature
	SendCommand(0xe0);
	SendData(0x02);

	//Send panel settings
	SendCommand(0x00);
	SendData(0xcf);
	SendData(0x89);

	g_log("Sending image data\n");

	//TEST: Fill the entire image with red
	//Resolution is 212 x 104 pixels (22048 total pixels, 2756 bytes)
	//First bitplane is 1=black, second is 1=red. 0 for both means white.
	SendCommand(0x10);
	for(int32_t i=0; i<2756; i++)
		SendData(m_blackFramebuffer[i]);
	SendCommand(0x13);
	for(int32_t i=0; i<2756; i++)
		SendData(m_redFramebuffer[i]);
	while(*m_busy_n == 0)
	{}

	g_log("Sending power on\n");

	//Send power-on command
	SendCommand(0x04);
	while(*m_busy_n == 0)
	{}

	g_log("Sending display refresh\n");

	//Send display refresh command
	SendCommand(0x12);
	while(*m_busy_n == 0)
	{}

	g_log("Shutting down\n");

	//Turn off DC-DC (TODO is this required?)
	SendCommand(0x02);
	while(*m_busy_n == 0)
	{}

	//TODO: we need to refresh every 24 hours to avoid ghosting
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Low level interface

void Display::SendCommand(uint8_t cmd)
{
	*m_dc = 0;
	*m_cs_n = 0;

	m_spi->BlockingWrite(cmd);
	m_spi->WaitForWrites();
	m_spi->DiscardRxData();

	*m_cs_n = 1;
}

void Display::SendData(uint8_t data)
{
	*m_dc = 1;
	*m_cs_n = 0;

	m_spi->BlockingWrite(data);
	m_spi->WaitForWrites();
	m_spi->DiscardRxData();

	*m_cs_n = 1;
}
