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
 */
module Management10GTxFifo(
	input wire				sys_clk,

	input wire				wr_en,
	input wire[7:0]			wr_data,
	input wire				wr_commit,

	input wire				tx_clk,
	input wire				link_up,
	output EthernetTxBus	tx_bus
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Deserialize the 8-bit data coming in from the MCU to a 32-bit stream

	logic		fifo_wr_en 		= 0;
	logic[31:0]	fifo_wr_data 	= 0;
	logic[2:0]	fifo_wr_valid	= 0;
	logic		fifo_wr_commit	= 0;

	logic[10:0]	tx_wr_packetlen = 0;
	logic[1:0]	bytecount		= 0;

	always_ff @(posedge sys_clk) begin

		fifo_wr_en		<= 0;

		fifo_wr_commit	<= wr_commit;

		//Count number of *words* in the FIFO
		if(fifo_wr_en)
			tx_wr_packetlen	<= tx_wr_packetlen + 1;

		//Push frame data 32 bits at a time
		if(wr_en) begin

			bytecount			<= bytecount + 1;

			//Full word?
			if(bytecount == 3) begin

				fifo_wr_valid		<= 4;
				fifo_wr_en			<= 1;
				fifo_wr_data[7:0]	<= wr_data;
			end

			//Partial word
			else begin
				fifo_wr_valid		<= fifo_wr_valid + 1;

				case(bytecount)
					0:	fifo_wr_data[31:24]	<= wr_data;
					1:	fifo_wr_data[23:16]	<= wr_data;
					2:	fifo_wr_data[15:8]	<= wr_data;
					default: begin
					end
				endcase
			end

		end

		//If committing a partial word, push it as-is
		//(bump packet length here a cycle early since we're committing next cycle)
		else if(wr_commit && (bytecount > 0) ) begin
			fifo_wr_en		<= 1;
			fifo_wr_valid	<= bytecount;
			tx_wr_packetlen	<= tx_wr_packetlen + 1;

			//Clear low bytes of the word to zero
			case(bytecount)
				1:	fifo_wr_data[23:0]	<= 0;
				2:	fifo_wr_data[15:0]	<= 0;
				3:	fifo_wr_data[7:0]	<= 0;
			endcase
		end

		//Reset length when pushing packet
		if(fifo_wr_commit) begin
			tx_wr_packetlen	<= 0;
			fifo_wr_valid	<= 0;
			bytecount		<= 0;
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The actual FIFOs

	//For now, no checks for overflow
	//Assume we're popping (at 10G speed) faster than we can possibly push from STM32 over SPI

	logic		rd_reset = 0;
	always_ff @(posedge tx_clk) begin
		rd_reset	<= !link_up;
	end

	wire		wr_reset;

	logic		txfifo_rd_en			= 0;
	wire[31:0]	txfifo_rd_data;

	EthernetTxBus	tx_bus_adv;

	ThreeStageSynchronizer sync_fifo_rst(
		.clk_in(sys_clk),
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
		.wr_clk(sys_clk),
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
		.wr_clk(sys_clk),
		.wr_en(fifo_wr_commit),
		.wr_data(tx_wr_packetlen),
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
