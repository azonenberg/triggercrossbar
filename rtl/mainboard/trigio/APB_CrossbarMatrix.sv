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

import CrossbarTypes::*;

/**
	@file
	@author Andrew D. Zonenberg
	@brief	APB wrapper around the crossbar switch matrix

	Register map:
		0000	muxsel0
		0020	muxsel1
		..
		0160	muxsel11
 */
module APB_CrossbarMatrix #(
	parameter NUM_PORTS = 12
)(

	//The APB bus
	APB.completer 				apb,

	//Trigger signals
	input wire[NUM_PORTS-1:0]	trig_in,
	output wire[NUM_PORTS-1:0]	trig_out,

	//Indicator LEDs
	output wire[NUM_PORTS-1:0]	trig_in_led,
	output wire[NUM_PORTS-1:0]	trig_out_led
);
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// We only support 32-bit APB, throw synthesis error for anything else

	if(apb.DATA_WIDTH != 32)
		apb_bus_width_is_invalid();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Register logic

	muxsel_t[NUM_PORTS-1:0] muxsel;

	logic[apb.ADDR_WIDTH-1:0]	portidx;

	initial begin
		for(integer i=0; i<NUM_PORTS; i++)
			muxsel[i] <= 0;
	end

	//Combinatorial readback
	always_comb begin

		apb.pready	= apb.psel && apb.penable;
		apb.prdata	= 0;
		apb.pslverr	= 0;

		portidx		= apb.paddr[apb.ADDR_WIDTH-1:5];

		if(apb.pready) begin

			//Throw error on unaligned address for either read or write
			if(apb.paddr[0])
				apb.pslverr = 1;

			//Throw error if port number is out of range
			else if(portidx >= NUM_PORTS)
				apb.pslverr = 1;

			//read
			else if(!apb.pwrite)
				apb.prdata	= muxsel[portidx];

			//write fails if we write an out-of-range value
			else begin
				if(apb.pwdata > NUM_PORTS)
					apb.pslverr	 = 1;
			end

		end
	end

	always_ff @(posedge apb.pclk or negedge apb.preset_n) begin

		//Reset
		if(!apb.preset_n) begin
			for(integer i=0; i<NUM_PORTS; i++)
				muxsel[i] <= 0;
		end

		//Normal path
		else begin

			if(apb.pready && apb.pwrite) begin

				//only actually write if we're going to the low word of the range
				if(apb.paddr[4:0] == 0)
					muxsel[portidx]	<= apb.pwdata;
			end

		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The actual crossbar block

	(* keep_hierarchy = "yes" *)
	CrossbarMatrix crossbar(
		.clk(apb.pclk),

		.muxsel(muxsel),

		.trig_in(trig_in),
		.trig_out(trig_out),

		.trig_in_led(trig_in_led),
		.trig_out_led(trig_out_led)
	);

endmodule
