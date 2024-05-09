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

import BERTConfig::*;

/**
	@file
	@author Andrew D. Zonenberg
	@brief The actual switch crossbar

	Completely combinatorial
 */
module CrossbarMatrix(
	input wire					clk_250mhz,

	input wire[11:0]			trig_in,
	output logic[11:0]			trig_out,

	input wire muxsel_t[11:0]	muxsel,

	output wire[11:0]			trig_in_led,
	output wire[11:0]			trig_out_led
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The actual muxes

	always_comb begin

		for(integer i=0; i<12; i=i+1) begin
			trig_out[i]	= trig_in[muxsel[i]];
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Pulse stretching for LEDs

	for(genvar g=0; g<12; g=g+1) begin : pstretch

		//Synchronize into local clock domain
		wire	trig_in_sync;
		ThreeStageSynchronizer #(
			.IN_REG(0)
		) sync_trig_in(
			.clk_in(),
			.din(trig_in[g]),
			.clk_out(clk_250mhz),
			.dout(trig_in_sync));

		wire	trig_out_sync;
		ThreeStageSynchronizer #(
			.IN_REG(0)
		) sync_trig_out(
			.clk_in(),
			.din(trig_out[g]),
			.clk_out(clk_250mhz),
			.dout(trig_out_sync));


		PulseStretcher stretch_in(
			.clk(clk_250mhz),
			.pulse(trig_in_sync),
			.stretched(trig_in_led[g]));

		PulseStretcher stretch_out(
			.clk(clk_250mhz),
			.pulse(trig_out_sync),
			.stretched(trig_out_led[g]));

	end

endmodule
