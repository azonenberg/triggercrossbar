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
	@brief Container for management logic

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
	output logic	 				txfifo_wr_commit = 0,
	output logic 					xg_txfifo_wr_en = 0,
	output logic[7:0] 				xg_txfifo_wr_data = 0,
	output logic	 				xg_txfifo_wr_commit = 0,

	//still in core clock domain, synchronizer in serdes module
	output logic					mgmt_lane0_en = 0,
	output logic					mgmt_lane1_en = 0,
	output logic					mgmt_we = 0,
	output logic[8:0]				mgmt_addr = 0,
	output logic[15:0]				mgmt_wdata = 0,
	input wire[15:0]				mgmt_lane0_rdata,
	input wire[15:0]				mgmt_lane1_rdata,
	input wire						mgmt_lane0_done,
	input wire						mgmt_lane1_done,
	input wire						mgmt_lane0_rx_rstdone,
	input wire						mgmt_lane1_rx_rstdone
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
		REG_XG_COMMIT		= 16'h002c,		//write any value to end the active packet

		//10G interface
		REG_XG0_STAT		= 16'h0060,		//0 = link up

		//BERT configuration

		REG_BERT_LANE0_WD	= 16'h0084,		//DRP write data (must write before address)
		REG_BERT_LANE0_WD_1	= 16'h0085,
		REG_BERT_LANE0_AD	= 16'h0086,		//DRP address
		REG_BERT_LANE0_AD_1 = 16'h0087,		//15 = write enable
											//14:9 = reserved
											//8:0 = address
		REG_BERT_LANE0_RD	= 16'h0088,
		REG_BERT_LANE0_RD_1	= 16'h0089,
		REG_BERT_LANE0_STAT	= 16'h008a,		//0 = DRP busy
											//1 = rx reset done

		REG_BERT_LANE1_WD	= 16'h00a4,
		REG_BERT_LANE1_WD_1	= 16'h00a5,
		REG_BERT_LANE1_AD	= 16'h00a6,
		REG_BERT_LANE1_AD_1	= 16'h00a7,
		REG_BERT_LANE1_RD	= 16'h00a8,
		REG_BERT_LANE1_RD_1	= 16'h00a9,
		REG_BERT_LANE1_STAT	= 16'h00aa,

		//Ethernet MAC frame buffer
		//Any address in this range will be treated as reading from the top of the buffer
		REG_EMAC_BUFFER_LO	= 16'h1000,
		REG_EMAC_BUFFER_HI	= 16'h1fff,

		//Ethernet MAC TX frame buffer
		//Any address in this range will be treated as writing to the end of the buffer
		REG_XG_TX_BUFFER_LO	= 16'h2000,
		REG_XG_TX_BUFFER_HI	= 16'h2fff,

		//helper just so we can use commas to separate list items
		REG_LAST

	} regid_t;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Address decoding and muxing logic

	logic 					reading					= 0;

	logic[1:0]				drp_busy	= 0;


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
		xg_txfifo_wr_en			<= 0;
		xg_txfifo_wr_commit		<= 0;
		mgmt_lane0_en			<= 0;
		mgmt_lane1_en			<= 0;

		//Start a new read
		if(rd_en)
			reading	<= 1;

		//Set interrupt line if something's changed
		if(!rxheader_rd_empty)
			irq					<= 1;

		//Manage DRP states
		if(mgmt_lane0_en)
			drp_busy[0]			<= 1;
		if(mgmt_lane1_en)
			drp_busy[1]			<= 1;
		if(mgmt_lane0_done)
			drp_busy[0]			<= 0;
		if(mgmt_lane1_done)
			drp_busy[1]			<= 0;

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

					REG_XG0_STAT:		rd_data <= {7'b0, xg0_link_up_sync };

					REG_BERT_LANE0_RD:		rd_data	<= mgmt_lane0_rdata[7:0];
					REG_BERT_LANE0_RD_1:	rd_data	<= mgmt_lane0_rdata[15:8];
					REG_BERT_LANE0_STAT:	rd_data	<= {6'b0, mgmt_lane0_rx_rstdone, drp_busy[0]};

					REG_BERT_LANE1_RD:		rd_data	<= mgmt_lane1_rdata[7:0];
					REG_BERT_LANE1_RD_1:	rd_data	<= mgmt_lane1_rdata[15:8];
					REG_BERT_LANE1_STAT:	rd_data	<= {6'b0, mgmt_lane1_rx_rstdone, drp_busy[1]};

					default: begin
						rd_data	<= 0;
					end

				endcase

			end

		end

		//Execute a write
		if(wr_en) begin

			//Ethernet MAC
			if(wr_addr >= REG_XG_TX_BUFFER_LO) begin
				xg_txfifo_wr_en	<= 1;
				xg_txfifo_wr_data	<= wr_data;
			end

			//Ethernet MAC
			else if(wr_addr >= REG_EMAC_BUFFER_LO) begin
				txfifo_wr_en	<= 1;
				txfifo_wr_data	<= wr_data;
			end

			else begin

				case(wr_addr[7:0])

					REG_BERT_LANE0_WD:		mgmt_wdata[7:0]		<= wr_data;
					REG_BERT_LANE0_WD_1:	mgmt_wdata[15:8]	<= wr_data;
					REG_BERT_LANE0_AD: 		mgmt_addr[7:0]		<= wr_data;
					REG_BERT_LANE0_AD_1: begin
						mgmt_lane0_en				<= 1;
						mgmt_we						<= wr_data[7];
						mgmt_addr[8]				<= wr_data[0];

					end

					REG_BERT_LANE1_WD:		mgmt_wdata[7:0]		<= wr_data;
					REG_BERT_LANE1_WD_1:	mgmt_wdata[15:8]	<= wr_data;
					REG_BERT_LANE1_AD:		mgmt_addr[7:0]		<= wr_data;
					REG_BERT_LANE1_AD_1: begin
						mgmt_lane1_en				<= 1;
						mgmt_we						<= wr_data[7];
						mgmt_addr[8]				<= wr_data[0];
					end

					REG_EMAC_COMMIT:	txfifo_wr_commit <= 1;
					REG_XG_COMMIT:		xg_txfifo_wr_commit <= 1;

				endcase

			end

		end

	end

endmodule
