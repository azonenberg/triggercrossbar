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
	input wire			clk_125mhz,

	output logic[3:0]	relay_a		= 0,
	output logic[3:0]	relay_b		= 0
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Cycle all of the relays once during startup and place them in output mode

	//Relays spec max 5ms operating time
	//1M cycles at 125 MHz is 8 ns which should be plenty of margin

	logic[1:0]	relayIndex		= 0;
	logic[19:0]	relayCount		= 0;

	enum logic[1:0]
	{
		RELAY_NEXT	= 2'h0,
		RELAY_ON	= 2'h1,
		RELAY_WAIT	= 2'h2,
		RELAY_DONE	= 2'h3
	} relayState	= RELAY_NEXT;

	always_ff @(posedge clk_125mhz) begin

		case(relayState)

			//Start doing the next relay
			RELAY_NEXT: begin

				//not sure which direction this selects but i guess we'll find out
				relay_a[relayIndex]	<= 0;
				relay_b[relayIndex]	<= 1;

				relayCount			<= 1;
				relayState			<= RELAY_ON;

			end

			//Coil energized
			RELAY_ON: begin
				relayCount	<= relayCount + 1;

				//Done with this cycle
				if(relayCount == 0) begin

					//De-energize coil
					relay_a	<= 0;
					relay_b	<= 0;

					//Finished last one?
					if(relayIndex == 3)
						relayState <= RELAY_DONE;

					//Nope, move on
					else begin
						relayIndex	<= relayIndex + 1;
						relayState	<= RELAY_WAIT;
						relayCount	<= 1;
					end

				end

			end

			//Wait a short time before moving to the next one
			RELAY_WAIT: begin
				relayCount	<= relayCount + 1;

				if(relayCount == 0)
					relayState	<= RELAY_NEXT;
			end

			//Done updating, don't touch again
			RELAY_DONE: begin
			end

		endcase

	end


endmodule
