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

module top(
	input wire			clk_200mhz_p,
	input wire			clk_200mhz_n,

	//GPIO LEDs
	output logic[3:0]	led = 0,

	//H-bridge control for relays
	output logic[3:0]	relay_a		= 0,
	output logic[3:0]	relay_b		= 0,

	//Trigger outputs
	output logic[11:0]	trig_out	= 0
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Clock synthesis

	wire	clk_200mhz_raw;

	wire	clk_200mhz;

	DifferentialInputBuffer #(
		.WIDTH(1),
		.IOSTANDARD("LVDS"),
		.ODT(1)
	) ibuf_clk (
		.pad_in_p(clk_200mhz_p),
		.pad_in_n(clk_200mhz_n),
		.fabric_out(clk_200mhz_raw)
	);

	ClockBuffer #(
		.TYPE("LOCAL"),
		.CE("NO")
	) buf_clk (
		.clkin(clk_200mhz_raw),
		.clkout(clk_200mhz),
		.ce(1'b1)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Cycle all of the relays once during startup and place them in output mode

	//Relays spec max 5ms operating time
	//This is 1M cycles @ 5 ns, round up to 2^20 cycles

	logic[1:0]	relayIndex		= 0;
	logic[19:0]	relayCount		= 0;

	enum logic[1:0]
	{
		RELAY_NEXT	= 2'h0,
		RELAY_ON	= 2'h1,
		RELAY_WAIT	= 2'h2,
		RELAY_DONE	= 2'h3
	} relayState	= RELAY_NEXT;

	always_ff @(posedge clk_200mhz) begin

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

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debug LEDs etc

	logic[22:0] count = 0;
	always_ff @(posedge clk_200mhz) begin
		count <= count + 1;

		if(count == 0)
			led			<= led + 1;

		//toggle trigger outputs at 100 MHz
		trig_out		<= ~trig_out;
	end

endmodule
