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

import EthernetBus::*;

/**
	@file
	@author Andrew D. Zonenberg
	@brief FIFO for shifting Ethernet frames from the QSPI clock domain to the management PHY clock domain

	No overflow checks since the incoming QSPI/APB data stream is far too slow to have any chance of overrunning
	the buffer of the 10G MAC
 */
module Management10GTxFifo(
	APB.completer 			apb,

	input wire				tx_clk,
	input wire				link_up,
	output EthernetTxBus	tx_bus
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// We only support 16-bit APB, throw synthesis error for anything else

	if(apb.DATA_WIDTH != 16)
		apb_bus_width_is_invalid();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Register map

	typedef enum logic[apb.ADDR_WIDTH-1:0]
	{
		REG_STAT	= 'h0000,		//[0] = link up flag
		REG_COMMIT	= 'h0008,		//Write any value to send the current frame
		REG_LENGTH	= 'h0010,		//Write expected frame length (in bytes) here before writing to TX buffer
		REG_TX_BUF	= 'h0040		//Write any address >= here to write to transmit buffer
	} regid_t;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Synchronizers for status signals

	wire	link_up_sync;

	ThreeStageSynchronizer sync_xg0_link_up(
		.clk_in(tx_clk),
		.din(link_up),
		.clk_out(apb.pclk),
		.dout(link_up_sync)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Deserialize the 16-bit data coming in from APB to a 32-bit stream

	logic		fifo_wr_en 			= 0;
	logic[31:0]	fifo_wr_data 		= 0;
	logic[2:0]	fifo_wr_valid		= 0;
	logic		fifo_wr_commit		= 0;
	logic		fifo_wr_commit_adv	= 0;

	logic[10:0]	expected_len_bytes	= 0;
	logic[10:0]	expected_len_words	= 0;
	logic[10:0]	running_len_bytes	= 0;
	logic		tx_wr_phase			= 0;
	logic[10:0]	bytes_left			= 0;
	logic[10:0]	wr_offset			= 0;

	//Combinatorial readback
	always_comb begin

		apb.pready	= apb.psel && apb.penable;
		apb.prdata	= 0;
		apb.pslverr	= 0;

		wr_offset 	= (apb.paddr - REG_TX_BUF);
		bytes_left	= expected_len_bytes - wr_offset;

		if(apb.pready) begin

			//write
			if(apb.pwrite) begin

				//Can't write to status register
				if(apb.paddr == REG_STAT)
					apb.pslverr		= 1;

				//everything else is in sequential block

			end

			//read
			else begin

				//Status register readback
				if(apb.paddr == REG_STAT)
					apb.prdata	= { 15'h0, link_up_sync };

				//No other readback allowed (FIFO is write only)
				else
					apb.pslverr	= 1;

			end

		end
	end

	always_ff @(posedge apb.pclk or negedge apb.preset_n) begin

		//Reset
		if(!apb.preset_n) begin
			fifo_wr_en			<= 0;
			fifo_wr_data		<= 0;
			fifo_wr_valid		<= 0;
			fifo_wr_commit		<= 0;
			fifo_wr_commit_adv	<= 0;
			tx_wr_phase			<= 0;
			expected_len_words	<= 0;
			running_len_bytes	<= 0;
			expected_len_bytes	<= 0;
		end

		//Normal path
		else begin

			fifo_wr_en			<= 0;
			fifo_wr_commit_adv	<= 0;
			fifo_wr_commit		<= fifo_wr_commit_adv;

			//Increment word count as we push
			if(fifo_wr_en)
				running_len_bytes	<= running_len_bytes + fifo_wr_valid;

			//Reset state after a push completes
			if(fifo_wr_commit) begin
				running_len_bytes	<= 0;
				fifo_wr_valid		<= 0;
				tx_wr_phase			<= 0;
				expected_len_words	<= 0;
			end

			if(apb.pready && apb.pwrite) begin

				//Commit an in-progress packet
				if(apb.paddr == REG_COMMIT) begin

					fifo_wr_commit_adv	<= 1;

					//If committing a half word, push it as-is
					//(bump packet length here a cycle early since we're committing next cycle)
					if(tx_wr_phase) begin
						fifo_wr_en			<= 1;
						fifo_wr_valid		<= 2;
						fifo_wr_data[15:0]	<= 0;
					end

				end

				else if(apb.paddr == REG_LENGTH) begin
					expected_len_bytes		<= apb.pwdata[10:0];
					if(apb.pwdata[1:0])
						expected_len_words	<= apb.pwdata[10:2] + 1;
					else
						expected_len_words	<= apb.pwdata[10:2];
				end

				//Write to the transmit buffer
				//Expect writes to be rounded up to multiples of 64-byte AXI transactions
				else if(apb.paddr >= REG_TX_BUF) begin

					//Even numbered address: high half of word
					if(apb.paddr[1:0] == 0) begin

						//Normal - write the half-word into the TX buffer
						fifo_wr_data	<= { apb.pwdata[7:0], apb.pwdata[15:8], 16'h0 };
						tx_wr_phase		<= 1;

						//If this is the end of a frame, push it
						if(bytes_left <= 2) begin
							fifo_wr_valid	<= bytes_left;
							fifo_wr_en		<= 1;
							tx_wr_phase		<= 0;
						end

					end

					//Odd numbered address: low half of word
					else begin

						//Either way, we're pushing the data into the FIFO
						tx_wr_phase		<= 0;
						fifo_wr_en		<= 1;

						//Off the end of the frame? Don't write
						if(wr_offset >= expected_len_bytes) begin
							fifo_wr_valid	<= 0;
							fifo_wr_en		<= 0;
						end

						//Enough space for the whole word? Write it
						else if(bytes_left >= 2)
							fifo_wr_valid	<= 4;

						//Partial word
						else
							fifo_wr_valid	<= bytes_left + 2;	//account for what's in the buffer now

						//Full write
						fifo_wr_data	<= { fifo_wr_data[31:16], apb.pwdata[7:0], apb.pwdata[15:8] };

					end

				end

			end

		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The actual FIFOs

	//For now, no checks for overflow
	//Assume we're popping (at 10G speed) faster than we can possibly push from STM32 over QSPI

	logic		rd_reset = 0;
	always_ff @(posedge tx_clk) begin
		rd_reset	<= !link_up;
	end

	wire		wr_reset;

	logic		txfifo_rd_en			= 0;
	wire[31:0]	txfifo_rd_data;

	EthernetTxBus	tx_bus_adv;

	ThreeStageSynchronizer sync_fifo_rst(
		.clk_in(apb.pclk),
		.din(rd_reset),
		.clk_out(tx_clk),
		.dout(wr_reset)
	);
	CrossClockFifo #(
		.WIDTH($bits(fifo_wr_data) + $bits(fifo_wr_valid)),
		.DEPTH(1024),
		.USE_BLOCK(1),
		.OUT_REG(1)
	) tx_cdc_fifo (
		.wr_clk(apb.pclk),
		.wr_en(fifo_wr_en),
		.wr_data({fifo_wr_data, fifo_wr_valid}),
		.wr_size(),
		.wr_full(),
		.wr_overflow(),
		.wr_reset(wr_reset),

		.rd_clk(tx_clk),
		.rd_en(txfifo_rd_en),
		.rd_data({tx_bus_adv.data, tx_bus_adv.bytes_valid}),
		.rd_size(),
		.rd_empty(),
		.rd_underflow(),
		.rd_reset(rd_reset)
	);

	logic		txheader_rd_en				= 0;
	wire[10:0]	txheader_rd_data;
	wire		txheader_rd_empty;

	CrossClockFifo #(
		.WIDTH(11),
		.DEPTH(32),
		.USE_BLOCK(0),
		.OUT_REG(1)
	) tx_framelen_fifo (
		.wr_clk(apb.pclk),
		.wr_en(fifo_wr_commit),
		.wr_data(expected_len_words),
		.wr_size(),
		.wr_full(),
		.wr_overflow(),
		.wr_reset(wr_reset),

		.rd_clk(tx_clk),
		.rd_en(txheader_rd_en),
		.rd_data(txheader_rd_data),
		.rd_size(),
		.rd_empty(txheader_rd_empty),
		.rd_underflow(),
		.rd_reset(rd_reset)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Pop logic

	enum logic[1:0]
	{
		TX_STATE_IDLE 		= 0,
		TX_STATE_POP		= 1,
		TX_STATE_SENDING	= 2
	} tx_state = TX_STATE_IDLE;

	logic[10:0] tx_count = 0;
	always_ff @(posedge tx_clk) begin

		tx_bus_adv.start		<= 0;
		tx_bus_adv.data_valid	<= txfifo_rd_en;
		txheader_rd_en			<= 0;
		txfifo_rd_en			<= 0;

		//Pipeline transmit bus to improve performance
		tx_bus					<= tx_bus_adv;

		case(tx_state)

			TX_STATE_IDLE: begin

				if(!txheader_rd_empty && !txheader_rd_en) begin
					txheader_rd_en	<= 1;
					tx_state		<= TX_STATE_POP;
				end

			end

			TX_STATE_POP: begin
				tx_bus_adv.start	<= 1;
				tx_count		<= 1;
				txfifo_rd_en	<= 1;
				tx_state		<= TX_STATE_SENDING;
			end

			TX_STATE_SENDING: begin

				if(tx_count >= txheader_rd_data) begin
					tx_state		<= TX_STATE_IDLE;
				end
				else begin
					txfifo_rd_en	<= 1;
					tx_count		<= tx_count + 1;
				end

			end

		endcase

	end

endmodule
