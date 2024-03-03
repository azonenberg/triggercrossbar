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
	@brief Controller for the relays on the bidirectional ports
 */
module RelayController(
	input wire			clk_250mhz,

	input wire			toggle_en,
	input wire			toggle_dir,			//1 = in, 0 = out
	input wire[1:0]		toggle_channel,
	output logic		toggle_done	= 0,

	output logic[3:0]	relay_a		= 0,
	output logic[3:0]	relay_b		= 0
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Cycle all of the relays once during startup and place them in input mode

	//Relays spec max 5ms operating time
	//2M cycles at 250 MHz is 8 ns which should be plenty of margin

	logic[20:0] relayCount = 0;

	always_ff @(posedge clk_250mhz) begin

		toggle_done	<= 0;

		if(toggle_en) begin

			//A high is input, B high is output
			if(toggle_dir) begin
				relay_a[toggle_channel]	<= 1;
				relay_b[toggle_channel]	<= 0;
			end
			else begin
				relay_a[toggle_channel]	<= 0;
				relay_b[toggle_channel]	<= 1;
			end

			relayCount				<= 1;

		end

		//Timeout to de-energize coil
		if(relay_a || relay_b) begin

			relayCount		<= relayCount + 1;

			//Done with this cycle
			if(relayCount == 0) begin

				//De-energize coil
				relay_a		<= 0;
				relay_b		<= 0;

				//Strobe done flag
				toggle_done	<= 1;

			end

		end

	end

endmodule
