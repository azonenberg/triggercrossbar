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

module BERTSubsystem(

	//Top level clocks
	input wire					clk_125mhz,
	input wire					pll_rgmii_lock,

	//QPLL signals
	input wire					qpll_lock,
	input wire					qpll_refclk,
	input wire					qpll_refclk_lost,
	input wire					qpll_clkout_10g3125,

	//Refclks to CPLLs
	input wire					serdes_refclk_156m25,
	input wire					serdes_refclk_200m,

	//Front panel differential BERT/pattern generator port
	output wire					tx0_p,
	output wire					tx0_n,

	input wire					rx0_p,
	input wire					rx0_n,

	output wire					tx1_p,
	output wire					tx1_n,

	input wire					rx1_p,
	input wire					rx1_n,

	//Control buses
	APB.completer 				apb,

	//Status outputs
	output wire[1:0]			cpll_lock
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Clock buffers

	wire lane0_rxclk;
	wire lane0_txclk;

	wire lane0_rxclk_raw;
	wire lane0_txclk_raw;

	BUFG bufg_lane0_rx(.I(lane0_rxclk_raw), .O(lane0_rxclk));
	BUFH bufh_lane0_tx(.I(lane0_txclk_raw), .O(lane0_txclk));

	wire lane1_rxclk;
	wire lane1_txclk;

	wire lane1_rxclk_raw;
	wire lane1_txclk_raw;

	BUFH bufh_lane1_rx(.I(lane1_rxclk_raw), .O(lane1_rxclk));
	BUFH bufh_lane1_tx(.I(lane1_txclk_raw), .O(lane1_txclk));

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// APB bridge (base 0x00_b000) for SFRs

	localparam ADDR_WIDTH			= 12;
	localparam NUM_DEVS				= 4;

	APB #(.DATA_WIDTH(16), .ADDR_WIDTH(ADDR_WIDTH), .USER_WIDTH(0)) downstreamBus[NUM_DEVS-1:0]();

	APBBridge #(
		.BASE_ADDR(24'h000_0000),
		.BLOCK_SIZE(32'h400),
		.NUM_PORTS(NUM_DEVS)
	) apb_bridge (
		.upstream(apb),
		.downstream(downstreamBus)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Low speed GPIOs on lane 0 (0x0000_b000)

	//TODO: refactor synchronizers into these blocks to reduce duplicated code
	wire			lane0_serdes_config_updated;
	bert_txconfig_t	tx0_config;
	bert_rxconfig_t	rx0_config;

	APB_BertConfig lane0_config(
		.apb(downstreamBus[0]),

		.tx_config(tx0_config),
		.rx_config(rx0_config),
		.config_updated(lane0_serdes_config_updated));

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Low speed GPIOs on lane 1 (0x0000_b400)

	//TODO: refactor synchronizers into these blocks to reduce duplicated code
	wire			lane1_serdes_config_updated;
	bert_txconfig_t	tx1_config;
	bert_rxconfig_t	rx1_config;

	APB_BertConfig lane1_config(
		.apb(downstreamBus[1]),

		.tx_config(tx1_config),
		.rx_config(rx1_config),
		.config_updated(lane1_serdes_config_updated));

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// DRP on lane 0 (0x0000_b800)

	wire		lane0_drp_en;
	wire		lane0_drp_we;
	wire[8:0]	lane0_drp_addr;
	wire[15:0]	lane0_drp_di;
	wire[15:0]	lane0_drp_do;
	wire		lane0_drp_rdy;

	wire		mgmt_lane0_rx_rstdone;

	APB_SerdesDRP lane0_drp(
		.apb(downstreamBus[2]),

		.drp_clk(clk_125mhz),

		.drp_en(lane0_drp_en),
		.drp_we(lane0_drp_we),
		.drp_addr(lane0_drp_addr),
		.drp_wdata(lane0_drp_di),
		.drp_rdata(lane0_drp_do),
		.drp_rdy(lane0_drp_rdy),

		.rx_rst_done(mgmt_lane0_rx_rstdone)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// DRP on lane 1 (0x0000_bc00)

	wire		lane1_drp_en;
	wire		lane1_drp_we;
	wire[8:0]	lane1_drp_addr;
	wire[15:0]	lane1_drp_di;
	wire[15:0]	lane1_drp_do;
	wire		lane1_drp_rdy;

	wire		mgmt_lane1_rx_rstdone;

	APB_SerdesDRP lane1_drp(
		.apb(downstreamBus[3]),

		.drp_clk(clk_125mhz),

		.drp_en(lane1_drp_en),
		.drp_we(lane1_drp_we),
		.drp_addr(lane1_drp_addr),
		.drp_wdata(lane1_drp_di),
		.drp_rdata(lane1_drp_do),
		.drp_rdy(lane1_drp_rdy),

		.rx_rst_done(mgmt_lane1_rx_rstdone)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Synchronizers for control registers

	bert_rxconfig_t		rx0_config_sync;
	bert_rxconfig_t		rx1_config_sync;
	bert_txconfig_t		tx0_config_sync;
	bert_txconfig_t		tx1_config_sync;

	wire	rx0_rxreset_done;
	wire	rx1_rxreset_done;

	ThreeStageSynchronizer sync_lane0_rx_rst_done(
		.clk_in(lane0_rxclk),
		.din(rx0_rxreset_done),
		.clk_out(apb.pclk),
		.dout(mgmt_lane0_rx_rstdone));

	ThreeStageSynchronizer sync_lane1_rx_rst_done(
		.clk_in(lane1_rxclk),
		.din(rx1_rxreset_done),
		.clk_out(apb.pclk),
		.dout(mgmt_lane1_rx_rstdone));

	RegisterSynchronizer #(
		.WIDTH($bits(bert_rxconfig_t)),
		.INIT(0),
		.IN_REG(1)
	) sync_rx0_config (
		.clk_a(apb.pclk),
		.en_a(lane0_serdes_config_updated),
		.ack_a(),
		.reg_a(rx0_config),
		.clk_b(lane0_rxclk),
		.updated_b(),
		.reset_b(1'b0),
		.reg_b(rx0_config_sync));

	RegisterSynchronizer #(
		.WIDTH($bits(bert_txconfig_t)),
		.INIT(0),
		.IN_REG(1)
	) sync_tx0_config (
		.clk_a(apb.pclk),
		.en_a(lane0_serdes_config_updated),
		.ack_a(),
		.reg_a(tx0_config),
		.clk_b(lane0_txclk),
		.updated_b(),
		.reset_b(1'b0),
		.reg_b(tx0_config_sync));

	RegisterSynchronizer #(
		.WIDTH($bits(bert_rxconfig_t)),
		.INIT(0),
		.IN_REG(1)
	) sync_rx1_config (
		.clk_a(apb.pclk),
		.en_a(lane1_serdes_config_updated),
		.ack_a(),
		.reg_a(rx1_config),
		.clk_b(lane1_rxclk),
		.updated_b(),
		.reset_b(1'b0),
		.reg_b(rx1_config_sync));

	RegisterSynchronizer #(
		.WIDTH($bits(bert_txconfig_t)),
		.INIT(0),
		.IN_REG(1)
	) sync_tx1_config (
		.clk_a(apb.pclk),
		.en_a(lane1_serdes_config_updated),
		.ack_a(),
		.reg_a(tx1_config),
		.clk_b(lane1_txclk),
		.updated_b(),
		.reset_b(1'b0),
		.reg_b(tx1_config_sync));

	wire	tx0_rst_sync;
	wire	tx1_rst_sync;
	wire	rx0_rst_sync;
	wire	rx1_rst_sync;

	ThreeStageSynchronizer sync_lane0_tx_reset(
		.clk_in(apb.pclk),
		.din(tx0_config.tx_reset),
		.clk_out(clk_125mhz),
		.dout(tx0_rst_sync));

	ThreeStageSynchronizer sync_lane1_tx_reset(
		.clk_in(apb.pclk),
		.din(tx1_config.tx_reset),
		.clk_out(clk_125mhz),
		.dout(tx1_rst_sync));

	ThreeStageSynchronizer sync_lane0_rx_reset(
		.clk_in(apb.pclk),
		.din(rx0_config.rx_reset),
		.clk_out(clk_125mhz),
		.dout(rx0_rst_sync));

	ThreeStageSynchronizer sync_lane1_rx_reset(
		.clk_in(apb.pclk),
		.din(rx1_config.rx_reset),
		.clk_out(clk_125mhz),
		.dout(rx1_rst_sync));

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Output PRBS generation on TX0 port

	wire		lane0_prbs_err;
	wire[31:0]	lane0_rx_data;

	GTXWrapper lane0_transceiver(
		.sysclk_in(clk_125mhz),

		//Resets
		.soft_reset_tx_in(tx0_rst_sync),
		.soft_reset_rx_in(rx0_rst_sync),
		.dont_reset_on_data_error_in(1'b0),
		.tx_fsm_reset_done_out(),
		.rx_fsm_reset_done_out(),

		//Register access
		.drpclk_in(clk_125mhz),
		.drpaddr_in(lane0_drp_addr),
		.drpdi_in(lane0_drp_di),
		.drpdo_out(lane0_drp_do),
		.drpen_in(lane0_drp_en),
		.drprdy_out(lane0_drp_rdy),
		.drpwe_in(lane0_drp_we),

		//Tie off unused ports
		.eyescanreset_in(1'b0),
		.eyescandataerror_out(),
		.eyescantrigger_in(1'b0),
		//.rxphmonitor_out(),
		//.rxphslipmonitor_out(),
		.rxmonitorout_out(),
		.rxmonitorsel_in(2'b0),

		//Subsystem resets
		.rxpmareset_in(rx0_config.pmareset),
		.rxresetdone_out(rx0_rxreset_done),

		//Transmit interface
		.txusrclk_in(lane0_txclk),
		.txusrclk2_in(lane0_txclk),
		.data_valid_in(1'b1),
		.txdata_in(32'h5555aaaa),
		.txoutclk_out(lane0_txclk_raw),
		.txoutclkfabric_out(),
		.txoutclkpcs_out(),
		.txresetdone_out(),

		//Fabric RX interface
		.rxusrclk_in(lane0_rxclk),
		.rxusrclk2_in(lane0_rxclk),
		.rxdata_out(lane0_rx_data),
		.rxoutclk_out(lane0_rxclk_raw),
		.rxoutclkfabric_out(),

		//Output pattern selection
		.txprbssel_in(tx0_config_sync.prbsmode),

		//Input PRBS detector
		.rxprbssel_in(rx0_config_sync.prbsmode),
		.rxprbserr_out(lane0_prbs_err),

		//Top level diff pairs
		.gtxtxn_out(tx0_p),
		.gtxtxp_out(tx0_n),
		.gtxrxn_in(rx0_p),
		.gtxrxp_in(rx0_n),

		//Input buffer config
		.rxpolarity_in(rx0_config_sync.invert),

		//Clock configuration
		.txrate_in(tx0_config_sync.clkdiv),
		.txratedone_out(),
		.rxrate_in(rx0_config_sync.clkdiv),
		.rxratedone_out(),

		//Output swing control and equalizer taps
		.txinhibit_in(!tx0_config_sync.enable),
		.txpolarity_in(tx0_config_sync.invert),
		.txdiffctrl_in(tx0_config_sync.swing),
		.txprecursor_in(tx0_config_sync.precursor),
		.txpostcursor_in(tx0_config_sync.postcursor),

		//Clock to/from CPLL
		.cplllock_out(cpll_lock[0]),
		.cplllockdetclk_in(clk_125mhz),
		.gtrefclk0_in(serdes_refclk_156m25),
		.gtrefclk1_in(serdes_refclk_200m),

		//Clock from QPLL
		.qplllock_in(qpll_lock),
		.qpllrefclklost_in(qpll_refclk_lost),
		.qplloutclk_in(qpll_clkout_10g3125),
		.qplloutrefclk_in(qpll_refclk),

		.rx_clk_from_qpll(rx0_config_sync.clk_from_qpll),
		.tx_clk_from_qpll(tx0_config_sync.clk_from_qpll)
		);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Output PRBS generation on TX1 port

	wire[31:0]	lane1_rx_data;
	wire		lane1_prbs_err;

	GTXWrapper lane1_transceiver(
		.sysclk_in(clk_125mhz),

		//Resets
		.soft_reset_tx_in(tx1_rst_sync),
		.soft_reset_rx_in(rx1_rst_sync),
		.dont_reset_on_data_error_in(1'b0),
		.tx_fsm_reset_done_out(),
		.rx_fsm_reset_done_out(),

		//Register access
		.drpclk_in(clk_125mhz),
		.drpaddr_in(lane1_drp_addr),
		.drpdi_in(lane1_drp_di),
		.drpdo_out(lane1_drp_do),
		.drpen_in(lane1_drp_en),
		.drprdy_out(lane1_drp_rdy),
		.drpwe_in(lane1_drp_we),

		//Tie off unused ports
		.eyescanreset_in(1'b0),
		.eyescandataerror_out(),
		.eyescantrigger_in(1'b0),
		//.rxphmonitor_out(),
		//.rxphslipmonitor_out(),
		.rxmonitorout_out(),
		.rxmonitorsel_in(2'b0),

		//Subsystem resets
		.rxpmareset_in(rx1_config.pmareset),
		.rxresetdone_out(rx1_rxreset_done),

		//Transmit interface
		.txusrclk_in(lane1_txclk),
		.txusrclk2_in(lane1_txclk),
		.data_valid_in(1'b1),
		.txdata_in(32'h5555aaaa),
		.txoutclk_out(lane1_txclk_raw),
		.txoutclkfabric_out(),
		.txoutclkpcs_out(),
		.txresetdone_out(),

		//Fabric RX interface
		.rxusrclk_in(lane1_rxclk),
		.rxusrclk2_in(lane1_rxclk),
		.rxdata_out(lane1_rx_data),
		.rxoutclk_out(lane1_rxclk_raw),
		.rxoutclkfabric_out(),

		//Output pattern selection
		.txprbssel_in(tx1_config_sync.prbsmode),

		//Input PRBS detector
		.rxprbssel_in(rx1_config_sync.prbsmode),
		.rxprbserr_out(lane1_prbs_err),

		//Top level diff pairs
		.gtxtxn_out(tx1_p),
		.gtxtxp_out(tx1_n),
		.gtxrxn_in(rx1_p),
		.gtxrxp_in(rx1_n),

		//Input buffer config
		.rxpolarity_in(rx1_config_sync.invert),

		//Clock configuration
		.txrate_in(tx1_config_sync.clkdiv),
		.txratedone_out(),
		.rxrate_in(rx1_config_sync.clkdiv),
		.rxratedone_out(),

		//Output swing control and equalizer taps
		.txinhibit_in(!tx1_config_sync.enable),
		.txpolarity_in(tx1_config_sync.invert),
		.txdiffctrl_in(tx1_config_sync.swing),
		.txprecursor_in(tx1_config_sync.precursor),
		.txpostcursor_in(tx1_config_sync.postcursor),

		//Clock to/from CPLL
		.cplllock_out(cpll_lock[1]),
		.cplllockdetclk_in(clk_125mhz),
		.gtrefclk0_in(serdes_refclk_156m25),
		.gtrefclk1_in(serdes_refclk_200m),

		//Clock from QPLL
		.qplllock_in(qpll_lock),
		.qpllrefclklost_in(qpll_refclk_lost),
		.qplloutclk_in(qpll_clkout_10g3125),
		.qplloutrefclk_in(qpll_refclk),

		.rx_clk_from_qpll(rx1_config_sync.clk_from_qpll),
		.tx_clk_from_qpll(tx1_config_sync.clk_from_qpll)
		);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// 64b66b gearboxing for CDR triggers

	//32-to-66 is a really annoying path with a lot of muxes! Difficult to do in one clock cycle
	//To get better performance, do 32-to-33 then 33-to-66 (simple 1:2 demux) which adds latency but is much cleaner

	logic		lane0_64b66b_bitslip;

	logic[65:0]	lane0_64b66b_gearbox_dout		= 0;
	logic		lane0_64b66b_gearbox_dvalid		= 0;

	wire[32:0]	lane0_64b66b_gearbox_dout_internal;
	wire		lane0_64b66b_gearbox_dvalid_internal;

	Gearbox32ToN #(
		.IN_WIDTH(32),
		.OUT_WIDTH(33)
	) lane0_gearbox66(
		.clk(lane0_rxclk),
		.data_in(lane0_rx_data),
		.valid_in(1'b1),
		.bitslip(lane0_64b66b_bitslip),

		.data_out(lane0_64b66b_gearbox_dout_internal),
		.valid_out(lane0_64b66b_gearbox_dvalid_internal)
	);

	//Demux logic
	logic	lane0_66b_phase = 0;
	always_ff @(posedge lane0_rxclk) begin
		lane0_64b66b_gearbox_dvalid	<= 0;

		if(lane0_64b66b_gearbox_dvalid_internal) begin

			lane0_66b_phase	<= !lane0_66b_phase;

			//Second half
			if(lane0_66b_phase) begin
				lane0_64b66b_gearbox_dout[32:0]	<= lane0_64b66b_gearbox_dout_internal;
				lane0_64b66b_gearbox_dvalid	<= 1;
			end

			else
				lane0_64b66b_gearbox_dout[65:33]	<= lane0_64b66b_gearbox_dout_internal;

		end

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// 64/66b decode subsystem

	wire		lane0_64b66b_locked;

	SymbolAligner64b66b lane0_64b66b_aligner(
		.clk(lane0_rxclk),
		.header_valid(lane0_64b66b_gearbox_dvalid),
		.header(lane0_64b66b_gearbox_dout[65:64]),
		.bitslip(lane0_64b66b_bitslip),
		.block_sync_good(lane0_64b66b_locked));

	wire		lane0_64b66b_symbol_valid;
	wire[1:0]	lane0_64b66b_header;
	wire[63:0]	lane0_64b66b_symbol;

	Descrambler64b66b lane0_64b66b_descrambler(
		.clk(lane0_rxclk),
		.valid_in(lane0_64b66b_gearbox_dvalid),
		.header_in(lane0_64b66b_gearbox_dout[65:64]),
		.data_in(lane0_64b66b_gearbox_dout[63:0]),

		.valid_out(lane0_64b66b_symbol_valid),
		.header_out(lane0_64b66b_header),
		.data_out(lane0_64b66b_symbol));

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// 8b0b gearboxing for CDR triggers

	logic		lane0_8b10b_bitslip;

	wire[39:0]	lane0_8b10b_gearbox_dout;
	wire		lane0_8b10b_gearbox_dvalid;

	Gearbox32ToN #(
		.IN_WIDTH(32),
		.OUT_WIDTH(40)
	) lane0_gearbox40(
		.clk(lane0_rxclk),
		.data_in(lane0_rx_data),
		.valid_in(1'b1),
		.bitslip(lane0_8b10b_bitslip),

		.data_out(lane0_8b10b_gearbox_dout),
		.valid_out(lane0_8b10b_gearbox_dvalid)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// 8B/10B decode subsystem

	//Per word decode output
	wire[3:0]	lane0_8b10b_data_valid;
	wire[7:0]	lane0_8b10b_data[3:0];
	wire[3:0]	lane0_8b10b_data_is_ctl;
	wire[3:0]	lane0_8b10b_disparity_err;
	wire[3:0]	lane0_8b10b_symbol_err;

	//Look for commas in each pair of adjacent lanes
	wire[3:0]	lane0_8b10b_locked;
	wire[3:0]	lane0_8b10b_no_commas;
	wire[3:0]	lane0_8b10b_lane_bitslip;

	//Sliding window of last 50 bits
	logic[9:0]	old_lsb_ff = 0;
	always_ff @(posedge lane0_rxclk) begin
		if(lane0_8b10b_gearbox_dvalid)
			old_lsb_ff	<= lane0_8b10b_gearbox_dout[9:0];
	end
	wire[49:0]	rx_window = {old_lsb_ff, lane0_8b10b_gearbox_dout};

	for(genvar g=0; g<4; g=g+1) begin : decoders

		//The actual line decoder block
		Decode8b10b lane0_8b10b_decode(
			.clk(lane0_rxclk),
			.codeword_valid(lane0_8b10b_gearbox_dvalid),
			.codeword_in(lane0_8b10b_gearbox_dout[10*g +: 10]),

			.data_valid(lane0_8b10b_data_valid[g]),
			.data(lane0_8b10b_data[g]),
			.data_is_ctl(lane0_8b10b_data_is_ctl[g]),
			.disparity_err(lane0_8b10b_disparity_err[g]),
			.symbol_err(lane0_8b10b_symbol_err[g]),
			.locked(),
			.bitslip()
		);

		//Symbol aligner looking at a 20-bit sliding window around our symbol
		SymbolAligner8b10b lane0_aligner(
			.clk(lane0_rxclk),
			.codeword_valid(lane0_8b10b_gearbox_dvalid),
			.comma_window(rx_window[g*10 +: 20]),

			.locked(lane0_8b10b_locked[g]),
			.no_commas(lane0_8b10b_no_commas[g]),
			.bitslip(lane0_8b10b_lane_bitslip[g])
			);

	end

	logic		lane0_8b10b_locked_all;
	logic[3:0]	lane0_8b10b_locked_or_no_commas;
	always_comb begin
		lane0_8b10b_bitslip 			= |lane0_8b10b_lane_bitslip;
		lane0_8b10b_locked_or_no_commas	= lane0_8b10b_locked | lane0_8b10b_no_commas;
		lane0_8b10b_locked_all			= &lane0_8b10b_locked_or_no_commas;
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debug ILA

	ila_0 ila(
		.clk(lane0_rxclk),
		.probe0(lane0_8b10b_locked),
		.probe1(lane0_8b10b_symbol_err),
		.probe2(lane0_8b10b_disparity_err),
		.probe3(lane0_8b10b_data[0]),
		.probe4(lane0_8b10b_data[1]),
		.probe5(lane0_8b10b_data[2]),
		.probe6(lane0_8b10b_data[3]),
		.probe7(lane0_8b10b_data_is_ctl),
		.probe8(lane0_8b10b_locked_all),
		.probe9(lane0_8b10b_no_commas),
		.probe10(lane0_64b66b_gearbox_dvalid),
		.probe11(lane0_64b66b_gearbox_dout),
		.probe12(lane0_64b66b_locked),
		.probe13(lane0_64b66b_symbol_valid),
		.probe14(lane0_64b66b_symbol),
		.probe15(lane0_64b66b_header)
		);

endmodule
