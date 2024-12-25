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

/**
	@file
	@author Andrew D. Zonenberg
	@brief	APB wrapper around SERDES control pin configuration for the BERT (DRP is separate and unrelated)
 */
module APB_BertConfig(

	//The APB bus
	APB.completer 				apb,

	//TX configuration state
	output bert_txconfig_t		tx_config		= 0,
	output bert_rxconfig_t		rx_config		= 0,
	output logic				config_updated	= 0
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// We only support 32-bit APB, throw synthesis error for anything else

	if(apb.DATA_WIDTH != 32)
		apb_bus_width_is_invalid();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Tie off unused APB signals

	assign apb.pruser = 0;
	assign apb.pbuser = 0;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Register map

	//All resets are set/clear flags, not self clearing
	//All registers on 0x20 alignment for octospi workaround
	typedef enum logic[apb.ADDR_WIDTH-1:0]
	{
		REG_TX_CONFIG	= 'h00,			//15 = enable
										//14 = invert
										//2:0 = PRBS mode

		REG_TX_CLK		= 'h20,			//15 = clock mux (1=QPLL, 0=CPLL)
										//2:0 = clock divide

		REG_TX_RESET	= 'h40,			//0 = full TX reset

		REG_TX_DRIVER	= 'h60,			//3:0 = swing
										//8:4 = postcursor
										//13:9 = precursor
										//TODO: pre/post cursor inversion in 15/14

		REG_RX_CONFIG	= 'h80,			//14 = invert
										//2:0 = PRBS mode

		REG_RX_CLK		= 'ha0,			//15 = clock mux (1=QPLL, 0=CPLL)
										//2:0 = clock divide

		REG_RX_RESET	= 'hc0			//0 = full RX reset
										//1 = PMA reset

	} regid_t;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Register logic

	//Combinatorial readback
	always_comb begin

		apb.pready	= apb.psel && apb.penable;
		apb.prdata	= 0;
		apb.pslverr	= 0;

		if(apb.pready) begin

			//write
			if(apb.pwrite) begin

				case(apb.paddr)
					REG_TX_CONFIG: begin
					end

					REG_TX_CLK: begin
					end

					REG_TX_RESET: begin
					end

					REG_TX_DRIVER: begin
					end

					REG_RX_CONFIG: begin
					end

					REG_RX_CLK: begin
					end

					REG_RX_RESET: begin
					end

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

		end
	end

	always_ff @(posedge apb.pclk or negedge apb.preset_n) begin

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

		end

	end

endmodule
