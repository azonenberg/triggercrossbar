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
	@brief	Quad SPI interface
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
	// Bus to ManagementRegisterInterface

	output logic		rd_en,
	output logic[15:0]	rd_addr	= 0,

	input wire			rd_valid,
	input wire[7:0]		rd_data,

	output logic		wr_en,
	output logic[15:0]	wr_addr	= 0,
	output wire[7:0]	wr_data,

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Top level APB bus to the rest of the chip

	APB.requester		apb
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Mux readback data

	logic		rd_valid_muxed;
	logic[7:0]	rd_data_muxed;
	logic		rd_is_apb = 0;

	wire		start;
	wire		insn_valid;
	wire[7:0]	opcode;
	wire[23:0]	addr;
	logic		rd_mode		= 0;
	wire		rd_ready;

	wire		wr_en_raw;
	wire		rd_en_raw;

	logic[7:0]	rd_data_hi = 0;

	always_ff @(posedge clk) begin

		rd_valid_muxed	<= 0;
		rd_data_muxed	<= 0;

		//APB reads
		if(rd_is_apb) begin

			//Even half
			if(apb.pready) begin
				rd_valid_muxed	<= 1;
				rd_data_muxed	<= apb.prdata[7:0];
			end

			//Odd half
			else if(rd_en_raw && rd_addr[0]) begin
				rd_valid_muxed	<= 1;
				rd_data_muxed	<= rd_data_hi;
			end

		end

		//legacy bus reads
		else begin
			rd_valid_muxed	<= rd_valid;
			rd_data_muxed	<= rd_data;
		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// QSPI interface

	typedef enum logic[7:0]
	{
		OP_LEGACY_READ		= 8'h20,
		OP_LEGACY_WRITE		= 8'h21,

		OP_APB_READ			= 8'h40,
		OP_APB_WRITE		= 8'h41
	} opcode_t;

	(* keep_hierarchy = "yes" *)
	QSPIDeviceInterface #(
		.INSN_BYTES(4)
	) qspi (
		.clk(clk),
		.sck(qspi_sck),
		.cs_n(qspi_cs_n),
		.dq(qspi_dq),

		.start(start),
		.insn_valid(insn_valid),
		.insn({opcode, addr}),
		.wr_valid(wr_en_raw),
		.wr_data(wr_data),

		.rd_mode(rd_mode),
		.rd_ready(rd_en_raw),
		.rd_valid(rd_valid_muxed),
		.rd_data(rd_data_muxed)
	);

	always_comb begin
		rd_en	= rd_en_raw && (opcode == OP_LEGACY_READ);
		wr_en	= wr_en_raw && (opcode == OP_LEGACY_WRITE);
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Address counting

	logic[15:0]	rd_addr_ff	= 0;
	logic[15:0]	wr_addr_ff	= 0;
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
			rd_addr		= {1'b0, addr[14:0]};
			wr_addr		= {1'b0, addr[14:0]};
		end

	end

	always_ff @(posedge clk) begin

		wr_addr_ff		<= wr_addr;
		rd_addr_ff		<= rd_addr;

		//Process instruction
		if(insn_valid) begin
			rd_mode		<= (opcode == OP_LEGACY_READ) || (opcode == OP_APB_READ);
			rd_is_apb	<= (opcode == OP_APB_READ);
		end

		//Reset anything we need on CS# falling edge
		if(start) begin
			rd_mode		<= 0;
			first		<= 1;
		end

		if(wr_en)
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
	assign apb.pstrb = 2'b11;

	logic	apb_psel_ff	= 0;

	always_comb begin

		//Default to not writing and pushing registered state
		apb.psel	= apb_psel_ff;
		apb.paddr	= rd_addr;
		apb.pwrite	= 0;

		//Dispatch APB reads on even address alignment
		if(rd_en_raw && (opcode == OP_APB_READ) && (rd_addr[0] == 0) )
			apb.psel	= 1;

		//enable one cycle after select
		apb.penable = apb_psel_ff;

	end

	always_ff @(posedge clk) begin

		//Register combinatorially generated flags
		apb_psel_ff	<= apb.psel;

		//clear pending request when it completes
		if(apb.pready)
			apb_psel_ff	<= 0;

		//TODO: APB writes need to wait until we have two bytes of data to send?

		//Save high half of read data
		if(apb.pready && !apb.pwrite)
			rd_data_hi	<= apb.prdata[15:8];

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debug ILA

	ila_1 ila(
		.clk(clk),
		.probe0(start),
		.probe1(insn_valid),
		.probe2(opcode),
		.probe3(addr),
		.probe4(wr_en_raw),
		.probe5(wr_en),
		.probe6(wr_data),
		.probe7(rd_en_raw),
		.probe8(rd_en),
		.probe9(rd_data),
		.probe10(rd_valid),
		.probe11(qspi_cs_n),
		.probe12(qspi.dq_in_sync),
		.probe13(qspi_sck),
		.probe14(qspi.dq_out),
		.probe15(apb.psel),
		.probe16(apb.pwrite),
		.probe17(apb.paddr),
		.probe18(apb.penable),
		.probe19(apb.pready),
		.probe20(apb.prdata),
		.probe21(first),
		.probe22(rd_mode),
		.probe23(rd_addr),
		.probe24(rd_valid_muxed),
		.probe25(rd_data_muxed),
		.probe26(rd_is_apb),
		.probe27(rd_data_hi),

		.probe28(qspi.state),
		.probe29(qspi.rd_data_next),
		.probe30(qspi.sck_rising),
		.probe31(qspi.sck_sync)
		);

endmodule
