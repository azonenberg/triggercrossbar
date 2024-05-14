`timescale 1ns/1ps
`default_nettype none
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
	@author Andrew D. Zonenberg
	@brief Clock input buffers and PLLs
 */
module ClockGeneration(

	//Main system clock input
	input wire			clk_200mhz_p,
	input wire			clk_200mhz_n,

	//PLL A: 1000 MHz VCO (x5)
	output wire			clk_50mhz,			//for DNA
	output wire			clk_125mhz,			//1x RGMII
	output wire			clk_250mhz,			//2x RGMII, plus main system clock
	output wire			pll_rgmii_lock
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Differential clock input buffer

	wire	sysclk_in;

	IBUFDS #(
		.DIFF_TERM("TRUE")
	) ibuf (
		.I(clk_200mhz_p),
		.IB(clk_200mhz_n),
		.O(sysclk_in)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Main PLL and clock buffers for it

	wire	rgmii_fbclk;

	wire	clk_50mhz_raw;
	wire	clk_125mhz_raw;
	wire	clk_250mhz_raw;

	MMCME2_BASE #(
		.BANDWIDTH("OPTIMIZED"),
		.CLKOUT0_DIVIDE_F(10),
		.CLKOUT1_DIVIDE(20),		//1000 MHz / 20 = 50 MHz
		.CLKOUT2_DIVIDE(10),
		.CLKOUT3_DIVIDE(10),
		.CLKOUT4_DIVIDE(4),			//1000 MHz / 4 = 250 MHz
		.CLKOUT5_DIVIDE(8),			//1000 MHz / 8 = 125 MHz
		.CLKOUT6_DIVIDE(4),

		.CLKOUT0_PHASE(0),
		.CLKOUT1_PHASE(0),
		.CLKOUT2_PHASE(90),
		.CLKOUT3_PHASE(0),
		.CLKOUT4_PHASE(0),
		.CLKOUT5_PHASE(0),
		.CLKOUT6_PHASE(0),

		.CLKOUT0_DUTY_CYCLE(0.5),
		.CLKOUT1_DUTY_CYCLE(0.5),
		.CLKOUT2_DUTY_CYCLE(0.5),
		.CLKOUT3_DUTY_CYCLE(0.5),
		.CLKOUT4_DUTY_CYCLE(0.5),
		.CLKOUT5_DUTY_CYCLE(0.5),
		.CLKOUT6_DUTY_CYCLE(0.5),

		.CLKFBOUT_MULT_F(5),		//200 MHz * 5 = 1000 MHz VCO
		.DIVCLK_DIVIDE(1),
		.CLKFBOUT_PHASE(0),

		.CLKIN1_PERIOD(4),			//200 MHz input
		.STARTUP_WAIT("FALSE"),
		.CLKOUT4_CASCADE("FALSE")

	) rgmii_mmcm (
		.CLKIN1(sysclk_in),
		.CLKFBIN(rgmii_fbclk),
		.RST(1'b0),
		.PWRDWN(1'b0),
		.CLKOUT0(),
		.CLKOUT0B(),
		.CLKOUT1(clk_50mhz_raw),
		.CLKOUT1B(),
		.CLKOUT2(),
		.CLKOUT2B(),
		.CLKOUT3(),
		.CLKOUT3B(),
		.CLKOUT4(clk_250mhz_raw),
		.CLKOUT5(clk_125mhz_raw),
		.CLKOUT6(),
		.CLKFBOUT(rgmii_fbclk),
		.CLKFBOUTB(),
		.LOCKED(pll_rgmii_lock)
	);

	BUFGCE buf_50mhz(
		.I(clk_50mhz_raw),
		.O(clk_50mhz),
		.CE(pll_rgmii_lock));

	BUFGCE buf_125mhz(
		.I(clk_125mhz_raw),
		.O(clk_125mhz),
		.CE(pll_rgmii_lock));

	BUFGCE bufg_250mhz(
		.I(clk_250mhz_raw),
		.O(clk_250mhz),
		.CE(pll_rgmii_lock));

endmodule
