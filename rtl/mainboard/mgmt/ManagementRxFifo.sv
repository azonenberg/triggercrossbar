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
	@brief FIFO for buffering incoming Ethernet frames (already in the management clock domain) to the QSPI
 */
module ManagementRxFifo(

	//The APB bus
	APB.completer 				apb,

	//RX link from MAC
	input wire EthernetRxBus	eth_rx_bus,
	input wire					eth_link_up,

	//Status flag to IRQ line
	output logic				rx_frame_ready
);
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// We only support 16-bit APB, throw synthesis error for anything else

	if(apb.DATA_WIDTH != 16)
		apb_bus_width_is_invalid();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Register IDs

	typedef enum logic[11:0]
	{
		REG_RX_BUF			= 12'h0000,	//start of receive buffer
										//(must be mapped at zero so we can use paddr directly as FIFO read index)

		REG_RX_POP			= 12'h0ff8,	//write any value to pop the current frame
		REG_RX_LEN			= 12'h0ffc	//frame length, in bytes
	} regid_t;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Register logic

	wire[31:0]			rxfifo_rd_data;
	logic				rxheader_rd_en;
	wire				rxheader_rd_empty;
	wire[10:0]			rxheader_rd_data;
	logic				rxfifo_pop_packet;
	logic[10:0]			rxfifo_packet_size;

	logic				apb_pready_next;

	//Combinatorial readback, but with one cycle of latency because of registered reads

	always_comb begin

		apb_pready_next	= apb.psel && apb.penable && !apb.pready;

		rx_frame_ready = !rxheader_rd_empty;

		apb.prdata	= 0;
		apb.pslverr	= 0;

		//this is one cycle after the request is valid due to pipeline latency on reads
		if(apb.pready) begin

			//read
			if(!apb.pwrite) begin

				if(apb.paddr == REG_RX_LEN)
					apb.prdata	= { 5'h0, rxheader_rd_data[10:0] };

				//can't read from pop register
				else if(apb.paddr == REG_RX_POP)
					apb.pslverr = 1;

				//anything else is reading from the rx buffer
				else begin

					//low half?
					if(apb.paddr[1:0] == 0)
						apb.prdata = { rxfifo_rd_data[23:16], rxfifo_rd_data[31:24] };

					//high half
					else
						apb.prdata = { rxfifo_rd_data[7:0], rxfifo_rd_data[15:8] };

				end
			end

			//write
			else begin
				if(apb.paddr != REG_RX_POP)
					apb.pslverr	 = 1;
			end

		end
	end

	always_ff @(posedge apb.pclk or negedge apb.preset_n) begin

		//Reset
		if(!apb.preset_n) begin
			apb.pready			<= 0;
			rxheader_rd_en		<= 0;
			rxfifo_pop_packet	<= 0;
		end

		//Normal path
		else begin

			rxheader_rd_en		<= 0;
			rxfifo_pop_packet	<= 0;

			//Register request flags
			//address/write data don't need to be registered, they'll be kept stable
			apb.pready		<= apb_pready_next;

			//Pop at the end of a packet
			if(apb.pready && apb.pwrite && (apb.paddr == REG_RX_POP) ) begin

				//Header is no longer valid
				rxheader_rd_en		<= 1;

				//Pop it (convert length from bytes to words, rounding up)
				rxfifo_pop_packet	<= 1;
				if(rxheader_rd_data[1:0])
					rxfifo_packet_size	<= rxheader_rd_data[10:2] + 1;
				else
					rxfifo_packet_size	<= rxheader_rd_data[10:2];
			end

		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Main FIFO and push logic

	//No need to handle drop requests because we're on the other side of an EthernetRxClockCrossing
	//which will drop anything with a bad FCS itself

	logic		rxfifo_wr_en		= 0;
	logic		rxfifo_wr_commit	= 0;
	logic[31:0]	rxfifo_wr_data		= 0;

	wire[12:0]	rxfifo_wr_size;
	wire[12:0]	rxfifo_rd_size;
	logic		rxfifo_wr_drop;

	CrossClockPacketFifo #(
		.WIDTH(32),
		.DEPTH(4096)	//at least 8 packets worth
	) rx_cdc_fifo (
		.wr_clk(apb.pclk),
		.wr_en(rxfifo_wr_en),
		.wr_data(rxfifo_wr_data),
		.wr_reset(!eth_link_up),
		.wr_size(rxfifo_wr_size),
		.wr_commit(rxfifo_wr_commit),
		.wr_rollback(rxfifo_wr_drop),

		.rd_clk(apb.pclk),
		.rd_en(apb_pready_next),				//read all the time even if actually reading a status register
												//(reading too much is harmless)
		.rd_offset({2'b0, apb.paddr[11:2]}),	//reading 32 bit words so use word address index
		.rd_pop_single(1'b0),
		.rd_pop_packet(rxfifo_pop_packet),
		.rd_packet_size(rxfifo_packet_size),
		.rd_data(rxfifo_rd_data),
		.rd_size(rxfifo_rd_size),
		.rd_reset(!eth_link_up)
	);

	//PUSH SIDE
	logic dropping = 0;
	logic[10:0] framelen = 0;

	wire		header_wfull;
	wire[5:0]	rxheader_rd_size;
	wire[5:0]	rxheader_wr_size;

	SingleClockFifo #(
		.WIDTH(11),
		.DEPTH(32),
		.USE_BLOCK(0),
		.OUT_REG(0)
	) rx_framelen_fifo (
		.clk(apb.pclk),

		.wr(eth_rx_bus.commit && !dropping && (framelen != 0) ),
		.din(framelen),
		.wsize(rxheader_wr_size),
		.full(header_wfull),
		.overflow(),
		.reset(!eth_link_up),

		.rd(rxheader_rd_en),
		.dout(rxheader_rd_data),
		.rsize(rxheader_rd_size),
		.empty(rxheader_rd_empty),
		.underflow()
	);

	always_ff @(posedge apb.pclk) begin

		rxfifo_wr_drop		<= 0;
		rxfifo_wr_commit	<= eth_rx_bus.commit;

		//Frame delimiter
		if(eth_rx_bus.start) begin

			//Not enough space for a full sized frame? Give up
			if( (rxfifo_wr_size < 375) || header_wfull )
				dropping	<= 1;

			//Nope, start a new frame
			else begin
				framelen	<= 0;
				dropping	<= 0;
			end

		end

		//If we hit max length frame or run out of space, drop the frame
		else if( (rxfifo_wr_size < 2) || (framelen > 1500) ) begin
			rxfifo_wr_drop	<= 1;
			dropping		<= 1;
		end

		//Nope, push data as needed
		else if(!dropping) begin
			rxfifo_wr_en	<= eth_rx_bus.data_valid;
			rxfifo_wr_data	<= eth_rx_bus.data;
			framelen		<= framelen + eth_rx_bus.bytes_valid;
		end

	end

	ila_2 ila(
		.clk(apb.pclk),
		.probe0(eth_rx_bus.start),
		.probe1(framelen),
		.probe2(apb.penable),
		.probe3(apb.psel),
		.probe4(apb.pready),
		.probe5(eth_rx_bus.commit),
		.probe6(apb.prdata),
		.probe7(apb.paddr),
		.probe8(rxfifo_pop_packet),
		.probe9(rxfifo_packet_size),
		.probe10(rxfifo_rd_data),
		.probe11(rxheader_rd_size),
		.probe12(rxfifo_rd_size),
		.probe13(apb.pwrite)
	);

endmodule
