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

import BERTConfig::*;
import CrossbarTypes::*;
import EthernetBus::*;

/**
	@file
	@author Andrew D. Zonenberg
	@brief Container for management logic using legacy bus

	Management registers have 16-bit addresses and are 8 bits in size.
 */
module ManagementRegisterInterface(

	//Core clock for the management domain
	input wire						clk,

	output logic					irq			= 0,

	//Data bus from QSPI interface or simulation bridge
	input wire						rd_en,
	input wire[15:0]				rd_addr,
	output logic					rd_valid	= 0,
	output logic[7:0]				rd_data		= 0,

	input wire						wr_en,
	input wire[15:0]				wr_addr,
	input wire[7:0]					wr_data,

	//Configuration registers in port RX clock domains
	input wire						xg0_rx_clk,
	input wire						xg0_link_up,

	//Configuration registers in core clock domain
	output logic					rxfifo_rd_en = 0,
	output logic					rxfifo_rd_pop_single = 0,
	input wire[31:0]				rxfifo_rd_data,
	output logic					rxheader_rd_en = 0,
	input wire						rxheader_rd_empty,
	input wire[10:0]				rxheader_rd_data,
	output logic 					txfifo_wr_en = 0,
	output logic[7:0] 				txfifo_wr_data = 0,
	output logic	 				txfifo_wr_commit = 0
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Synchronizers for status signals

	wire	xg0_link_up_sync;

	ThreeStageSynchronizer sync_xg0_link_up(
		.clk_in(xg0_rx_clk),
		.din(xg0_link_up),
		.clk_out(clk),
		.dout(xg0_link_up_sync)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// List of named registers

	//Note that ManagementBridge uses MSB of address as read/write flag
	//so we actually have only 15 bits available for addressing

	//must match ManagementRegisterInterface in FPGAInterface.h
	typedef enum logic[15:0]
	{
		//Reasons for an IRQ
		REG_FPGA_IRQSTAT	= 16'h0020,		//
		REG_FPGA_IRQSTAT_1	= 16'h0021,		//
											// 0 = RX Ethernet frame ready

		//Ethernet MAC
		REG_EMAC_RXLEN		= 16'h0024,
		REG_EMAC_RXLEN_1	= 16'h0025,
		REG_EMAC_COMMIT		= 16'h0028,		//write any value to end the active packet

		//Ethernet MAC frame buffer
		//Any address in this range will be treated as reading from the top of the buffer
		REG_EMAC_BUFFER_LO	= 16'h1000,
		REG_EMAC_BUFFER_HI	= 16'h1fff,

		//helper just so we can use commas to separate list items
		REG_LAST

	} regid_t;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Address decoding and muxing logic

	logic 					reading					= 0;

	logic[8:0]				packetWordsRead = 0;
	logic[8:0]				packetWordLength = 0;
	logic					rxheader_rd_en_ff = 0;

	always_ff @(posedge clk) begin

		//Clear single cycle flags
		rd_valid				<= 0;
		rxfifo_rd_en			<= 0;
		rxheader_rd_en			<= 0;
		rxfifo_rd_pop_single	<= 0;
		txfifo_wr_en			<= 0;
		txfifo_wr_commit		<= 0;

		//Start a new read
		if(rd_en)
			reading	<= 1;

		//Set interrupt line if something's changed
		if(!rxheader_rd_empty)
			irq					<= 1;

		//Track expected length of a packet being read
		rxheader_rd_en_ff	<= rxheader_rd_en;
		if(rxheader_rd_en_ff) begin
			if(rxheader_rd_data[1:0])
				packetWordLength	<= rxheader_rd_data[10:2] + 1;
			else
				packetWordLength	<= rxheader_rd_data[10:2];
		end
		if(rxheader_rd_en)
			packetWordsRead 		<= 0;
		if(rxfifo_rd_pop_single)
			packetWordsRead 		<= packetWordsRead + 1;

		//Continue a read
		if(rd_en || reading) begin
			rd_valid	<= 1;
			reading		<= 0;

			//Ethernet MAC
			//Read data without any endianness swapping, since it's logically an array of bytes
			if(rd_addr >= REG_EMAC_BUFFER_LO) begin

				case(rd_addr[1:0])
					0: rd_data					<= rxfifo_rd_data[31:24];
					1: begin
						rd_data					<= rxfifo_rd_data[23:16];
						//pop the buffer since we've got the read data in the working register
						//if(packetWordsRead < packetWordLength)
						rxfifo_rd_pop_single	<= 1;
					end
					2:	rd_data					<= rxfifo_rd_data[15:8];
					3: begin
						rxfifo_rd_en			<= 1;
						rd_data					<= rxfifo_rd_data[7:0];
					end
				endcase

			end

			//Main register decoder
			else begin

				case(rd_addr)

					REG_FPGA_IRQSTAT: begin
						rd_data		<= {7'b0, !rxheader_rd_empty };
						irq			<= 0;
					end
					REG_FPGA_IRQSTAT_1: rd_data <= 8'b0;

					REG_EMAC_RXLEN:		rd_data <= rxheader_rd_data[7:0];
					REG_EMAC_RXLEN_1: begin
						rd_data 		<= {5'b0, rxheader_rd_data[10:8]};
						rxheader_rd_en	<= 1;

						//read (but don't pop) first data word
						//so it's ready by the time we need it
						if(rxheader_rd_data != 0)
							rxfifo_rd_en	<= 1;
					end

					default: begin
						rd_data	<= 0;
					end

				endcase

			end

		end

		//Execute a write
		if(wr_en) begin

			//Ethernet MAC
			if(wr_addr >= REG_EMAC_BUFFER_LO) begin
				txfifo_wr_en	<= 1;
				txfifo_wr_data	<= wr_data;
			end

			else begin

				case(wr_addr[7:0])

					REG_EMAC_COMMIT:	txfifo_wr_commit <= 1;

				endcase

			end

		end

	end

endmodule
