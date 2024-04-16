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
	@brief Gearboxes a 32-bit data stream from the transceiver to a 40-bit data stream suitable for 8b/10b decoding
 */
module Gearbox32To40(
	input wire			clk,
	input wire[31:0]	data_in,
	input wire			valid_in,
	input wire			bitslip,

	output logic[39:0]	data_out	= 0,
	output logic		valid_out	= 0
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Rearrange the data to MSB-first bit ordering because serdes IPs are dumb, and pipeline to improve latency

	logic[31:0] data_in_flip		= 0;
	logic		data_in_flip_valid	= 0;

	always_ff @(posedge clk) begin
		for(integer i=0; i<32; i=i+1)
			data_in_flip[i] 	<= data_in[31-i];
		data_in_flip_valid		<= valid_in;
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The gearbox

	//Append new data to the existing buffer
	logic[5:0]	work_valid	= 0;
	logic[38:0]	workbuf		= 0;
	logic[71:0] combined;
	logic[6:0]	combined_valid;

	always_comb begin

		//Clear all unset bits to zero
		combined		= 0;

		//Copy working buffer
		combined[71:33]	= workbuf;
		combined_valid	= work_valid;

		//Add new data (left justified)
		if(data_in_flip_valid) begin
			combined[(40-work_valid) +: 32]	= data_in_flip;
			combined_valid					= work_valid + 32;
		end

		//Drop a bit if we're bitslipping
		if(bitslip)
			combined_valid	= combined_valid - 1;

	end

	always_ff @(posedge clk) begin

		valid_out	<= 0;

		//New data? If not, nothing to do
		if(data_in_flip_valid) begin

			//If we have a full output word, push it out and save the remaining data
			if(combined_valid >= 40) begin
				data_out	<= combined[71:32];
				valid_out	<= 1;
				workbuf		<= { combined[31:0], 7'h0 };
				work_valid	<= combined_valid - 40;
			end

			//No, just register it as-is
			else begin
				workbuf		<= combined[71:33];
				work_valid	<= combined_valid;
			end

		end

	end

endmodule
