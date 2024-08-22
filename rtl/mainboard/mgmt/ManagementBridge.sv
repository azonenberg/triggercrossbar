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

`include "../../../antikernel-ipcores/amba/apb/APBTypes.sv"

/**
	@file
	@author	Andrew D. Zonenberg
	@brief	Quad SPI to APB bridge
 */
module ManagementBridge(

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// System clocks

	input wire			clk,

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Bus to MCU

	input wire			qspi_sck,
	input wire			qspi_cs_n,
	inout wire[3:0]		qspi_dq,

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Top level APB bus to the rest of the chip

	APB.requester		apb
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Mux readback data

	logic[23:0]	rd_addr	= 0;
	logic[23:0]	wr_addr	= 0;

	logic		rd_valid_muxed;
	logic[7:0]	rd_data_muxed;

	wire		start;
	wire		insn_valid;
	wire[7:0]	opcode;
	wire[23:0]	addr;
	logic		rd_mode		= 0;
	wire		rd_ready;

	wire		wr_en_raw;
	wire		rd_en_raw;

	logic[31:0]	rd_data_ff = 0;

	always_ff @(posedge clk) begin

		rd_valid_muxed	<= 0;
		rd_data_muxed	<= 0;

		//First word
		if(apb.pready) begin
			rd_valid_muxed	<= 1;
			rd_data_muxed	<= apb.prdata[7:0];
		end

		//Subsequent words
		else if(rd_en_raw) begin
			rd_valid_muxed	<= 1;
			case(rd_addr[1:0])
				0:	rd_data_muxed	<= rd_data_ff[7:0];
				1:	rd_data_muxed	<= rd_data_ff[15:8];
				2:	rd_data_muxed	<= rd_data_ff[23:16];
				3:	rd_data_muxed	<= rd_data_ff[31:24];
			endcase
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// QSPI interface

	wire		stop;
	wire[7:0]	wr_data;

	typedef enum logic[7:0]
	{
		OP_APB_READ			= 8'h40,
		OP_APB_WRITE		= 8'h41
	} opcode_t;

	(* keep_hierarchy = "yes" *)
	QSPIDeviceInterface #(
		.INSN_BYTES(4),
		.DDR_MODE(1)
	) qspi (
		.clk(clk),
		.sck(qspi_sck),
		.cs_n(qspi_cs_n),
		.dq(qspi_dq),

		.start(start),
		.stop(stop),
		.insn_valid(insn_valid),
		.insn({opcode, addr}),
		.wr_valid(wr_en_raw),
		.wr_data(wr_data),

		.rd_mode(rd_mode),
		.rd_ready(rd_en_raw),
		.rd_valid(rd_valid_muxed),
		.rd_data(rd_data_muxed)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Address counting

	logic[23:0]	rd_addr_ff	= 0;
	logic[23:0]	wr_addr_ff	= 0;
	logic		first		= 0;

	always_comb begin

		//Default to passthrough
		rd_addr			= rd_addr_ff;
		wr_addr			= wr_addr_ff;

		//Increment if reading/writing
		if(rd_en_raw)
			rd_addr		= rd_addr_ff + 1;
		if(wr_en_raw && !first)
			wr_addr		= wr_addr_ff + 1;

		//Start a new transaction
		if(insn_valid) begin
			rd_addr		= addr;
			wr_addr		= addr;
		end

	end

	always_ff @(posedge clk) begin

		wr_addr_ff		<= wr_addr;
		rd_addr_ff		<= rd_addr;

		//Process instruction
		if(insn_valid)
			rd_mode		<= (opcode == OP_APB_READ);

		//Reset anything we need on CS# falling edge
		if(start) begin
			rd_mode		<= 0;
			first		<= 1;
		end

		if(wr_en_raw)
			first		<= 0;

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// APB interfacing

	//Hook up clock
	assign apb.pclk = clk;

	//Tie off unused signals
	//TODO: enable soft reset?
	assign apb.preset_n = 1;
	assign apb.pwakeup = 0;
	assign apb.pprot = 0;

	logic						apb_penable_ff				= 0;
	logic						apb_pwrite_ff			= 0;
	logic[23:0]					apb_pwdata_lo			= 0;
	logic[31:0]					apb_pwdata_ff			= 0;
	logic[apb.DATA_WIDTH-1:0]	apb_paddr_ff			= 0;
	logic						apb_write_half_valid	= 0;

	always_comb begin

		//Default to all write data being valid
		apb.pstrb	= 2'b11;

		//Default to not writing and pushing registered state
		apb.penable	= apb_penable_ff;
		apb.psel	= apb_penable_ff;

		if(opcode == OP_APB_READ)
			apb.paddr	= rd_addr;
		else
			apb.paddr	= apb_paddr_ff;

		apb.pwrite	= apb_pwrite_ff;
		apb.pwdata	= apb_pwdata_ff;

		//Dispatch APB reads on first byte of a word
		if(rd_en_raw && (opcode == OP_APB_READ) && (rd_addr[1:0] == 0) ) begin
			apb.penable	= 1;
			apb.pwrite	= 0;
		end

		//Dispatch APB writes on last byte of a word
		if(wr_en_raw && (opcode == OP_APB_WRITE) && (wr_addr[1:0] == 3) ) begin
			apb.psel	= 1;
			apb.penable	= 1;
			apb.pwrite	= 1;
			apb.pwdata	= { wr_data, apb_pwdata_lo };
		end

		//If a write is in progress (even alignment) send a partial-width write
		//TODO: do we want to allow this or just force 32-bit transactions
		/*
		if(stop && apb_write_half_valid) begin
			apb.psel	= 1;
			apb.pwrite	= 1;
			apb.pwdata	= { 8'h00, apb_pwdata_lo };
			apb.pstrb	= 2'b01;
		end
		*/

	end

	always_ff @(posedge clk) begin

		//Register combinatorially generated flags
		apb_penable_ff		<= apb.penable;
		apb_pwrite_ff		<= apb.pwrite;
		apb_pwdata_ff		<= apb.pwdata;

		//clear pending request when it completes
		if(apb.pready)
			apb_penable_ff	<= 0;

		//clear write-pending flag when write commits
		if(apb.pready && apb.pwrite)
			apb_write_half_valid	<= 0;

		//Save low half of write data
		if(wr_en_raw)  begin
			if(!apb_write_half_valid)
				apb_paddr_ff				<= wr_addr;
			apb_write_half_valid			<= 1;

			case(wr_addr[1:0])
				0:	apb_pwdata_lo[7:0]		<= wr_data;
				1:	apb_pwdata_lo[15:8]		<= wr_data;
				2:	apb_pwdata_lo[23:16]	<= wr_data;
				3: begin
				end
			endcase
		end

		//Save high bytes of read data
		if(apb.pready && !apb.pwrite)
			rd_data_ff	<= apb.prdata;

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debug ILA
	/*
	ila_1 ila(
		.clk(clk),

		.probe0(start),
		.probe1(stop),
		.probe2(insn_valid),
		.probe3(opcode),
		.probe4(addr),
		.probe5(wr_en_raw),
		.probe6(wr_data),
		.probe7(rd_mode),
		.probe8(rd_en_raw),
		.probe9(rd_valid_muxed),
		.probe10(rd_data_muxed),

		.probe11(apb.psel),
		.probe12(apb.penable),
		.probe13(apb.pwdata),
		.probe14(apb.prdata),
		.probe15(apb.paddr),
		.probe16(apb.pready),

		.probe17(apb_pwdata_lo),
		.probe18(rd_data_ff),
		.probe19(apb.pwrite),

		.probe20(qspi_cs_n),
		.probe21(qspi.dq_in),
		.probe22(qspi.dq_out),
		.probe23(qspi.dq_oe),
		.probe24(qspi.sck_sync),
		.probe25(qspi.sck_rising),
		.probe26(qspi.sck_falling),

		.probe27(apb_penable_ff),
		.probe28(wr_addr)
	);
	*/

endmodule
