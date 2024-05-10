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

`include "../../../antikernel-ipcores/amba/apb/APBTypes.sv"

/**
	@file
	@author	Andrew D. Zonenberg
	@brief	APB register access wrapper around the relay controller
 */
module APB_RelayController(
	//The APB bus
	APB.completer 		apb,

	output logic[3:0]	relay_state	= 0,

	//Outputs to relay driver
	output wire[3:0]	relay_a,
	output wire[3:0]	relay_b
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Native relay controller

	logic			toggle_en;
	logic			toggle_dir;
	logic[1:0]		toggle_channel;
	wire			toggle_done;

	RelayController ctrl(
		.clk_250mhz(apb.pclk),

		.toggle_en(toggle_en),
		.toggle_dir(toggle_dir),
		.toggle_channel(toggle_channel),
		.toggle_done(toggle_done),

		.relay_a(relay_a),
		.relay_b(relay_b)
		);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// We only support 16-bit APB, throw synthesis error for anything else

	if(apb.DATA_WIDTH != 16)
		apb_bus_width_is_invalid();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Register map

	typedef enum logic[apb.ADDR_WIDTH-1:0]
	{
		REG_RELAY_TOGGLE	= 16'h0000,		//15	= direction (1 = in, 0 = out)
											//1:0	= channel number
											//Writes are self timed

		REG_RELAY_STAT		= 16'h0020,		//0 = busy flag
		REG_RELAY_STAT2		= 16'h0040		//duplicate of RELAY_STAT
	} regid_t;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Keep track of what position each relay is in

	//(TODO: allow readback?)

	logic	relay_busy	= 0;

	always_ff @(posedge apb.pclk or negedge apb.preset_n) begin

		//Reset
		if(!apb.preset_n) begin
			relay_state						<= 0;
			relay_busy						<= 0;
		end

		//Normal path
		else begin

			//Manage relay states
			if(toggle_done)
				relay_busy					<= 0;
			if(toggle_en) begin
				relay_busy					<= 1;
				relay_state[toggle_channel]	<= toggle_dir;
			end

		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// APB interface logic

	always_comb begin

		//Combinatorially assert PREADY when selected
		apb.pready		= apb.psel && apb.penable;

		//Default to no errors and no read data
		apb.prdata		= 0;
		apb.pslverr		= 0;

		//Clear control signals
		toggle_en		= 0;
		toggle_channel	= 0;
		toggle_dir		= 0;

		if(apb.pready) begin

			if(apb.pwrite) begin

				case(apb.paddr)

					REG_RELAY_TOGGLE: begin
						toggle_channel	= apb.pwdata[1:0];
						toggle_dir		= apb.pwdata[15];
						toggle_en		= 1;
					end

					//unmapped address
					default:	apb.pslverr		= 1;

				endcase

			end

			else begin

				case(apb.paddr)

					REG_RELAY_STAT:		apb.prdata	= { 15'h0, relay_busy };

					//unmapped address
					default:	apb.pslverr		= 1;

				endcase

			end

		end

	end

endmodule
