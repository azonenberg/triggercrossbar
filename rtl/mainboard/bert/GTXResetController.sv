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
	@brief Reset controller for 7 series GTX

	This module replaces the TX_STARTUP_FSM block generated by the 7 series transceivers wizard.
 */
module GTXResetController #(
	parameter SYSCLK_PERIOD				= 8,	//sysclk period, in ns
	parameter MAX_UI					= 2		//longest unit interval (lowest data rate)
												//that we might potentially reconfigure ourselves for
												//Default is 500 Mbps (lowest possible per DS183)
)(
	//Always-on clock for reset controller
	input wire			sysclk,

	//Transmit side control ports
	input wire			tx_reset,
	input wire			tx_clk_from_qpll,
	output logic		tx_reset_done = 0,

	input wire			rx_reset,
	input wire			rx_clk_from_qpll,
	output logic		rx_reset_done = 0,

	//Ports to the GTX
	output logic		GTTXRESET = 0,
	input wire			TXRESETDONE,
	output logic		TXUSERRDY = 0,

	output logic		GTRXRESET = 0,
	input wire			RXRESETDONE,
	output logic		RXUSERRDY = 0,

	output logic		CPLLRESET = 0,
	input wire			CPLLLOCK,
	input wire			QPLLLOCK
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Synchronize async flags into the local clock domain

	wire	cpll_lock;
	wire	qpll_lock;
	wire	tx_reset_done_int;
	wire	rx_reset_done_int;

	ThreeStageSynchronizer #(
		.IN_REG(0)
	) sync_cpll_lock (
		.clk_in(sysclk),
		.din(CPLLLOCK),
		.clk_out(sysclk),
		.dout(cpll_lock));

	ThreeStageSynchronizer #(
		.IN_REG(0)
	) sync_qpll_lock (
		.clk_in(sysclk),
		.din(QPLLLOCK),
		.clk_out(sysclk),
		.dout(qpll_lock));

	ThreeStageSynchronizer #(
		.IN_REG(0)
	) sync_tx_rst_done (
		.clk_in(sysclk),
		.din(TXRESETDONE),
		.clk_out(sysclk),
		.dout(tx_reset_done_int));


	ThreeStageSynchronizer #(
		.IN_REG(0)
	) sync_rx_rst_done (
		.clk_in(sysclk),
		.din(RXRESETDONE),
		.clk_out(sysclk),
		.dout(rx_reset_done_int));

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Main transceiver reset

	//Fixed 500ns delay
	localparam CYCLES_FOR_500NS = 500 / SYSCLK_PERIOD;

	//CDR lock time is 50K UIs per DS183
	localparam UIS_FOR_CDRLOCK = 50000;
	localparam CYCLES_FOR_CDRLOCK = UIS_FOR_CDRLOCK * MAX_UI / SYSCLK_PERIOD;

	enum logic[2:0]
	{
		RST_STATE_POWERUP_DELAY 	= 0,
		RST_STATE_POWERUP_DELAY_1	= 1,
		RST_STATE_PLL_RESET			= 2,
		RST_STATE_PLL_LOCK			= 3,
		RST_STATE_USERCLK_WAIT		= 4,
		RST_STATE_RESET_WAIT		= 5,
		RST_STATE_IDLE				= 6
	} rst_state = RST_STATE_POWERUP_DELAY;

	logic[15:0] count = 0;

	//Mux lock state for both PLLs
	logic	tx_pll_lock;
	logic	rx_pll_lock;
	always_comb begin

		if(tx_clk_from_qpll)
			tx_pll_lock = qpll_lock;
		else
			tx_pll_lock = cpll_lock;

		if(rx_clk_from_qpll)
			rx_pll_lock = qpll_lock;
		else
			rx_pll_lock = cpll_lock;

	end

	always_ff @(posedge sysclk) begin

		case(rst_state)

			//Reset in Response to Completion of Configuration (UG476 v1.12.1 page 67)
			//1) Wait 500ns after configuration
			RST_STATE_POWERUP_DELAY: begin
				count <= count + 1;
				if(count > CYCLES_FOR_500NS) begin
					count 		<= 0;
					//2) can we just leave reset in sequential the whole time or do we ahve to change this?
					rst_state	<= RST_STATE_POWERUP_DELAY_1;
				end
			end	//RST_STATE_POWERUP_DELAY

			//3) Wait another 300-500ns
			//4) Assert GTTXRESET
			RST_STATE_POWERUP_DELAY_1: begin

				count <= count + 1;
				if(count > CYCLES_FOR_500NS) begin
					GTTXRESET	<= 1;
					GTRXRESET	<= 1;

					//reset CPLL if either side is using it
					if(!tx_clk_from_qpll || !rx_clk_from_qpll)
						CPLLRESET	<= 1;

					rst_state	<= RST_STATE_PLL_RESET;
				end

			end	//RST_STATE_POWERUP_DELAY_1

			//Wait for PLL to lose lock before deasserting reset
			RST_STATE_PLL_RESET: begin

				//If using CPLL, wait for it to unlock first
				if( (!tx_clk_from_qpll || !rx_clk_from_qpll) ) begin
					if(!cpll_lock) begin
						CPLLRESET	<= 0;
						rst_state	<= RST_STATE_PLL_LOCK;
					end
				end

				//We don't manage QPLL reset, that's done separately
				else
					rst_state	<= RST_STATE_PLL_LOCK;

			end	//RST_STATE_PLL_RESET

			//Wait for PLL to regain lock
			RST_STATE_PLL_LOCK: begin

				//Wait for both TX and RX PLLs to lock
				if(tx_pll_lock && rx_pll_lock) begin

					GTTXRESET	<= 0;
					GTRXRESET	<= 0;
					rst_state	<= RST_STATE_USERCLK_WAIT;
					count		<= 0;

				end
			end	//RST_STATE_PLL_LOCK

			//Wait a while then set TXUSERRDY
			RST_STATE_USERCLK_WAIT: begin
				count		<= count + 1;

				if(count > CYCLES_FOR_CDRLOCK) begin
					TXUSERRDY	<= 1;
					RXUSERRDY	<= 1;
					rst_state	<= RST_STATE_RESET_WAIT;
				end

			end	//RST_STATE_USERCLK_WAIT

			//Wait for GTX to finish resetting
			RST_STATE_RESET_WAIT: begin
				if(tx_reset_done_int) begin
					rst_state		<= RST_STATE_IDLE;
					tx_reset_done	<= 1;
					rx_reset_done	<= 1;
				end
			end	//RST_STATE_RESET_WAIT

			//Wait for soft reset request
			//NOTE: concurrent soft resets allowed, but can't reset TX if RX is in the middle of a reset
			//or vice versa
			RST_STATE_IDLE: begin

				if(tx_reset) begin
					GTTXRESET		<= 1;
					tx_reset_done	<= 0;
					rst_state		<= RST_STATE_PLL_LOCK;
				end

				if(rx_reset) begin
					GTRXRESET		<= 1;
					rx_reset_done	<= 0;
					rst_state		<= RST_STATE_PLL_LOCK;
				end

			end	//RST_STATE_IDLE

		endcase

	end

endmodule