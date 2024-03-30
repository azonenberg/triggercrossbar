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
	@brief Pulse stretcher with blink functionality during heavy activity
 */
module PulseStretcher(
	input wire		clk,
	input wire		pulse,
	output logic	stretched = 0
);

	//assume 250 MHz clock for now
	//~4 Hz pulse rate so ~125ms between toggles
	//so 62500000 cycles. 2^26 is close enough for this purpose and is a nice round number

	logic[25:0] count 			= 0;

	enum logic[1:0]
	{
		STATE_IDLE		= 0,
		STATE_BLINK_ON	= 1,
		STATE_BLINK_OFF	= 2,
		STATE_ARM_OFF	= 3
	} state = STATE_IDLE;

	always_ff @(posedge clk) begin

		case(state)

			//Nothing to do, LED off
			STATE_IDLE: begin
				if(pulse) begin
					count		<= 1;
					stretched	<= 1;
					state		<= STATE_BLINK_ON;
				end
			end

			//Turn on the LED, ignore further pulses during the on time
			STATE_BLINK_ON: begin
				count <= count + 1;
				if(count == 0) begin
					stretched	<= 0;
					state		<= STATE_BLINK_OFF;
				end
			end

			//LED off, but watch for more pulses
			STATE_BLINK_OFF: begin
				count <= count + 1;

				//Go back to idle when timeout expires
				if(count == 0)
					state	<= STATE_IDLE;

				//Get ready to turn on again if we get another pulse
				if(pulse)
					state	<= STATE_ARM_OFF;

			end

			//LED off, but will turn on after timeout
			STATE_ARM_OFF: begin
				count <= count + 1;

				//Go back to idle when timeout expires
				if(count == 0) begin
					count		<= 1;
					stretched	<= 1;
					state		<= STATE_BLINK_ON;
				end
			end


		endcase

	end

endmodule
