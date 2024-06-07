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

module CDRTrigger #(
	parameter DEBUG_LA = 0
)(

	//Control bus
	APB.completer 				apb,

	//SERDES interface
	input wire					rx_clk,
	input wire[31:0]			rx_data,

	//Trigger output (rx_clk domain)
	output wire					trig_out
);
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// We only support 16-bit APB, throw synthesis error for anything else

	if(apb.DATA_WIDTH != 16)
		apb_bus_width_is_invalid();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Register map

	typedef enum logic[apb.ADDR_WIDTH-1:0]
	{
		//Global configuration
		REG_TRIG_MODE			= 'h00,			//00 = 8B/10B pattern match
												//01 = 8B/10B disparity error

												//80 = 64B/66B pattern match
												//81 = 64B/66B invalid symbol

		//8B/10B trigger config
		REG_TRIG_8B10B_DATA0	= 'h10,			//Byte values for 8B/10B trigger pattern
												//Symbols 1:0
		REG_TRIG_8B10B_DATA1	= 'h12,			//Symbols 3:2
		REG_TRIG_8B10B_DATA2	= 'h14,			//Symbols 5:4
		REG_TRIG_8B10B_DATA3	= 'h16,			//Symbols 7:6
		REG_TRIG_8B10B_DATA4	= 'h18,			//Symbols 9:8
		REG_TRIG_8B10B_TYPE		= 'h1a,			//Control/data flag for 8B/10B trigger pattern (1 bit per symbol)
												//0=data, 1=control
		REG_TRIG_8B10B_MASK		= 'h1c,			//Mask for 8B/10B trigger pattern (1 bit per symbol)
												//0=ignored, 1=checked
		REG_TRIG_8B10B_DISP		= 'h1e,			//Target for 8B/10B disparity check (1 bit per symbol)
												//1=negative, 0=positive
		REG_TRIG_8B10B_DISPMASK	= 'h20,			//Mask for 8B/10B disparity check (1 bit per symbol)
												//0=ignored, 1=checked

		//64B/66B trigger config
		REG_TRIG_64B66B_DATA0	= 'h40,			//Byte values for 64B/66B trigger pattern
												//Bits 15:0
		REG_TRIG_64B66B_DATA1	= 'h42,			//Bits 31:16
		REG_TRIG_64B66B_DATA2	= 'h44,			//Bits 47:32
		REG_TRIG_64B66B_DATA3	= 'h46,			//Bits 63:48
		REG_TRIG_64B66B_TYPE	= 'h48,			//2-bit block type field to match
		REG_TRIG_64B66B_MASK	= 'h4a,			//Byte mask for packet payload
												//0=ignored, 1=checked

		//Status registers
		REG_TRIG_STAT			= 'h60,			//0 = 8B10B symbol lock
												//1 = 64B66B symbol lock
		REG_TRIG_STAT2			= 'h80			//duplicate of TRIG_STAT

	} regid_t;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// 64b/66b gearbox

	//32-to-66 is a really annoying path with a lot of muxes! Difficult to do in one clock cycle
	//To get better performance, do 32-to-33 then 33-to-66 (simple 1:2 demux) which adds latency but is much cleaner

	logic		rx_64b66b_bitslip;

	logic[65:0]	rx_64b66b_gearbox_dout		= 0;
	logic		rx_64b66b_gearbox_dvalid		= 0;

	wire[32:0]	rx_64b66b_gearbox_dout_internal;
	wire		rx_64b66b_gearbox_dvalid_internal;

	Gearbox32ToN #(
		.IN_WIDTH(32),
		.OUT_WIDTH(33)
	) gearbox66(
		.clk(rx_clk),
		.data_in(rx_data),
		.valid_in(1'b1),
		.bitslip(rx_64b66b_bitslip),

		.data_out(rx_64b66b_gearbox_dout_internal),
		.valid_out(rx_64b66b_gearbox_dvalid_internal)
	);

	//Demux logic
	logic	rx_66b_phase = 0;
	always_ff @(posedge rx_clk) begin
		rx_64b66b_gearbox_dvalid	<= 0;

		if(rx_64b66b_gearbox_dvalid_internal) begin

			rx_66b_phase	<= !rx_66b_phase;

			//Second half
			if(rx_66b_phase) begin
				rx_64b66b_gearbox_dout[32:0]	<= rx_64b66b_gearbox_dout_internal;
				rx_64b66b_gearbox_dvalid	<= 1;
			end

			else
				rx_64b66b_gearbox_dout[65:33]	<= rx_64b66b_gearbox_dout_internal;

		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// 64b/66b decode

	wire		rx_64b66b_locked;

	SymbolAligner64b66b rx_64b66b_aligner(
		.clk(rx_clk),
		.header_valid(rx_64b66b_gearbox_dvalid),
		.header(rx_64b66b_gearbox_dout[65:64]),
		.bitslip(rx_64b66b_bitslip),
		.block_sync_good(rx_64b66b_locked));

	wire		rx_64b66b_symbol_valid;
	wire[1:0]	rx_64b66b_header;
	wire[63:0]	rx_64b66b_symbol;

	Descrambler64b66b rx_64b66b_descrambler(
		.clk(rx_clk),
		.valid_in(rx_64b66b_gearbox_dvalid),
		.header_in(rx_64b66b_gearbox_dout[65:64]),
		.data_in(rx_64b66b_gearbox_dout[63:0]),

		.valid_out(rx_64b66b_symbol_valid),
		.header_out(rx_64b66b_header),
		.data_out(rx_64b66b_symbol));

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// 8b/10b gearbox

	logic		rx_8b10b_bitslip;

	wire[39:0]	rx_8b10b_gearbox_dout;
	wire		rx_8b10b_gearbox_dvalid;

	Gearbox32ToN #(
		.IN_WIDTH(32),
		.OUT_WIDTH(40)
	) rx_gearbox40(
		.clk(rx_clk),
		.data_in(rx_data),
		.valid_in(1'b1),
		.bitslip(rx_8b10b_bitslip),

		.data_out(rx_8b10b_gearbox_dout),
		.valid_out(rx_8b10b_gearbox_dvalid)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// 8b/10b decode

	//Per word decode output
	wire[3:0]	rx_8b10b_data_valid;
	wire[7:0]	rx_8b10b_data[3:0];
	wire[3:0]	rx_8b10b_data_is_ctl;
	wire[3:0]	rx_8b10b_disparity_err;
	wire[3:0]	rx_8b10b_symbol_err;

	//Look for commas in each pair of adjacent lanes
	wire[3:0]	rx_8b10b_locked;
	wire[3:0]	rx_8b10b_no_commas;
	wire[3:0]	rx_8b10b_lane_bitslip;

	//Sliding window of last 50 bits
	logic[9:0]	old_lsb_ff = 0;
	always_ff @(posedge rx_clk) begin
		if(rx_8b10b_gearbox_dvalid)
			old_lsb_ff	<= rx_8b10b_gearbox_dout[9:0];
	end
	wire[49:0]	rx_window = {old_lsb_ff, rx_8b10b_gearbox_dout};

	for(genvar g=0; g<4; g=g+1) begin : decoders

		//The actual line decoder block
		Decode8b10b rx_8b10b_decode(
			.clk(rx_clk),
			.codeword_valid(rx_8b10b_gearbox_dvalid),
			.codeword_in(rx_8b10b_gearbox_dout[10*g +: 10]),

			.data_valid(rx_8b10b_data_valid[g]),
			.data(rx_8b10b_data[g]),
			.data_is_ctl(rx_8b10b_data_is_ctl[g]),
			.disparity_err(rx_8b10b_disparity_err[g]),
			.symbol_err(rx_8b10b_symbol_err[g]),
			.locked(),
			.bitslip()
		);

		//Symbol aligner looking at a 20-bit sliding window around our symbol
		SymbolAligner8b10b rx_aligner(
			.clk(rx_clk),
			.codeword_valid(rx_8b10b_gearbox_dvalid),
			.comma_window(rx_window[g*10 +: 20]),

			.locked(rx_8b10b_locked[g]),
			.no_commas(rx_8b10b_no_commas[g]),
			.bitslip(rx_8b10b_lane_bitslip[g])
			);

	end

	logic		rx_8b10b_locked_all;
	logic[3:0]	rx_8b10b_locked_or_no_commas;
	always_comb begin
		rx_8b10b_bitslip 			= |rx_8b10b_lane_bitslip;
		rx_8b10b_locked_or_no_commas	= rx_8b10b_locked | rx_8b10b_no_commas;
		rx_8b10b_locked_all			= &rx_8b10b_locked_or_no_commas;
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// 8b/10b trigger TODO

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// 64b/66b trigger

	assign trig_out = rx_64b66b_symbol_valid && (rx_64b66b_header == 2'b01);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// APB interface logic

	//Combinatorial readback
	always_comb begin

		apb.pready	= apb.psel && apb.penable;
		apb.prdata	= 0;
		apb.pslverr	= 0;

		if(apb.pready) begin
			/*
			//write
			if(apb.pwrite) begin

				case(apb.paddr)

					//illegal address
					default:	apb.pslverr	= 1;

				endcase

			end

			//read
			else begin

				case(apb.paddr)

					REG_TX_CONFIG:	apb.prdata	= { tx_config.enable, tx_config.invert, 11'h0, tx_config.prbsmode };
					REG_TX_CLK:		apb.prdata	= { tx_config.clk_from_qpll, 12'h0, tx_config.clkdiv };
					REG_TX_RESET:	apb.prdata	= { 15'h0, tx_config.tx_reset };
					REG_TX_DRIVER:	apb.prdata	= { 2'h0, tx_config.precursor, tx_config.postcursor, tx_config.swing };

					REG_RX_CONFIG:	apb.prdata	= { 1'b0, rx_config.invert, 11'h0, rx_config.prbsmode };
					REG_RX_CLK:		apb.prdata	= { rx_config.clk_from_qpll, 12'h0, rx_config.clkdiv };
					REG_RX_RESET:	apb.prdata	= { 14'h0, rx_config.pmareset, rx_config.rx_reset };

					//illegal address
					default:	apb.pslverr	= 1;

				endcase

			end
			*/
		end
	end

	always_ff @(posedge apb.pclk or negedge apb.preset_n) begin

		/*
		//Reset
		if(!apb.preset_n) begin
			tx_config	<= 0;
			rx_config	<= 0;
		end

		//Normal path
		else begin

			config_updated		<= 0;

			if(apb.pready && apb.pwrite) begin

				config_updated	<= 1;

				case(apb.paddr)
					REG_TX_CONFIG: begin
						tx_config.prbsmode		<= apb.pwdata[2:0];
						tx_config.invert		<= apb.pwdata[14];
						tx_config.enable		<= apb.pwdata[15];
					end

					REG_TX_CLK: begin
						tx_config.clkdiv		<= apb.pwdata[2:0];
						tx_config.clk_from_qpll	<= apb.pwdata[15];
					end

					REG_TX_RESET: begin
						tx_config.tx_reset		<= apb.pwdata[0];
					end

					REG_TX_DRIVER: begin
						tx_config.swing			<= apb.pwdata[3:0];
						tx_config.postcursor	<= apb.pwdata[8:4];
						tx_config.precursor		<= apb.pwdata[13:9];
					end

					REG_RX_CONFIG: begin
						rx_config.prbsmode		<= apb.pwdata[2:0];
						rx_config.invert		<= apb.pwdata[14];
					end

					REG_RX_CLK: begin
						rx_config.clkdiv		<= apb.pwdata[2:0];
						rx_config.clk_from_qpll	<= apb.pwdata[15];
					end

					REG_RX_RESET: begin
						rx_config.rx_reset		<= apb.pwdata[0];
						rx_config.pmareset		<= apb.pwdata[1];
					end

					//illegal address
					default: begin
					end

				endcase

			end

		end*/

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debug ILA

	if(DEBUG_LA) begin
		ila_0 ila(
			.clk(rx_clk),
			.probe0(rx_8b10b_locked),
			.probe1(rx_8b10b_symbol_err),
			.probe2(rx_8b10b_disparity_err),
			.probe3(rx_8b10b_data[0]),
			.probe4(rx_8b10b_data[1]),
			.probe5(rx_8b10b_data[2]),
			.probe6(rx_8b10b_data[3]),
			.probe7(rx_8b10b_data_is_ctl),
			.probe8(rx_8b10b_locked_all),
			.probe9(rx_8b10b_no_commas),
			.probe10(rx_64b66b_gearbox_dvalid),
			.probe11(rx_64b66b_gearbox_dout),
			.probe12(rx_64b66b_locked),
			.probe13(rx_64b66b_symbol_valid),
			.probe14(rx_64b66b_symbol),
			.probe15(rx_64b66b_header)
			);
	end

endmodule
