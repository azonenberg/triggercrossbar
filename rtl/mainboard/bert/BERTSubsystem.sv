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
	output wire[1:0]			cpll_lock,

	//Trigger inputs from the crossbar
	input wire[1:0]				la_trig
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

	BUFG bufg_lane1_rx(.I(lane1_rxclk_raw), .O(lane1_rxclk));
	BUFH bufh_lane1_tx(.I(lane1_txclk_raw), .O(lane1_txclk));

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// APB bridge (base 0x00_c000) for SFRs

	localparam ADDR_WIDTH			= 12;
	localparam NUM_DEVS				= 8;

	APB #(.DATA_WIDTH(32), .ADDR_WIDTH(ADDR_WIDTH), .USER_WIDTH(0)) downstreamBus[NUM_DEVS-1:0]();

	APBBridge #(
		.BASE_ADDR(24'h000_0000),
		.BLOCK_SIZE(32'h100),
		.NUM_PORTS(NUM_DEVS)
	) apb_bridge (
		.upstream(apb),
		.downstream(downstreamBus)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Low speed GPIOs on lane 0 (0x0000_c000)

	//TODO: refactor synchronizers into these blocks to reduce duplicated code
	wire			lane0_serdes_config_updated;
	bert_txconfig_t	tx0_config;
	bert_rxconfig_t	rx0_config;

	APB #(.DATA_WIDTH(32), .ADDR_WIDTH(ADDR_WIDTH), .USER_WIDTH(0)) lane0_config_apb();
	APB_CDC lane0_apb_config_cdc(
		.upstream(downstreamBus[0]),
		.downstream_pclk(clk_125mhz),
		.downstream(lane0_config_apb));

	APB_BertConfig lane0_config(
		.apb(lane0_config_apb),

		.tx_config(tx0_config),
		.rx_config(rx0_config),
		.config_updated(lane0_serdes_config_updated));

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Low speed GPIOs on lane 1 (0x0000_c100)

	//TODO: refactor synchronizers into these blocks to reduce duplicated code
	wire			lane1_serdes_config_updated;
	bert_txconfig_t	tx1_config;
	bert_rxconfig_t	rx1_config;

	APB #(.DATA_WIDTH(32), .ADDR_WIDTH(ADDR_WIDTH), .USER_WIDTH(0)) lane1_config_apb();
	APB_CDC lane1_apb_config_cdc(
		.upstream(downstreamBus[1]),
		.downstream_pclk(clk_125mhz),
		.downstream(lane1_config_apb));

	APB_BertConfig lane1_config(
		.apb(lane1_config_apb),

		.tx_config(tx1_config),
		.rx_config(rx1_config),
		.config_updated(lane1_serdes_config_updated));

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// DRP on lane 0 (0x0000_c200)

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
	// DRP on lane 1 (0x0000_c300)

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
		.clk_a(clk_125mhz),
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
		.clk_a(clk_125mhz),
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
		.clk_a(clk_125mhz),
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
		.clk_a(clk_125mhz),
		.en_a(lane1_serdes_config_updated),
		.ack_a(),
		.reg_a(tx1_config),
		.clk_b(lane1_txclk),
		.updated_b(),
		.reset_b(1'b0),
		.reg_b(tx1_config_sync));

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Output PRBS generation on TX0 port

	wire		lane0_prbs_err;
	wire[31:0]	lane0_rx_data;

	GTXWrapper lane0_transceiver(
		.sysclk_in(clk_125mhz),

		//Resets
		.soft_reset_tx_in(tx0_config.tx_reset),
		.soft_reset_rx_in(rx0_config.rx_reset),
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

		.rx_clk_from_qpll(rx0_config.clk_from_qpll),
		.tx_clk_from_qpll(tx0_config.clk_from_qpll)
		);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Output PRBS generation on TX1 port

	wire[31:0]	lane1_rx_data;
	wire		lane1_prbs_err;

	GTXWrapper lane1_transceiver(
		.sysclk_in(clk_125mhz),

		//Resets
		.soft_reset_tx_in(tx1_config.tx_reset),
		.soft_reset_rx_in(rx1_config.rx_reset),
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

		.rx_clk_from_qpll(rx1_config.clk_from_qpll),
		.tx_clk_from_qpll(tx1_config.clk_from_qpll)
		);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// CDR triggers TODO

	wire	lane0_cdrtrig;
	wire	lane1_cdrtrig;

	//0x0000_c600
	CDRTrigger #(
		.DEBUG_LA(1)
	) lane0_cdr_trig (
		.apb(downstreamBus[6]),

		.rx_clk(lane0_rxclk),
		.rx_data(lane0_rx_data),

		.trig_out(lane0_cdrtrig)
	);

	//0x0000_c700
	CDRTrigger #(
		.DEBUG_LA(0)
	) lane1_cdr_trig (
		.apb(downstreamBus[7]),

		.rx_clk(lane1_rxclk),
		.rx_data(lane1_rx_data),

		.trig_out(lane1_cdrtrig)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Logic analyzer modules

	//Shift the APB to the PHY clock domain
	APB #(.DATA_WIDTH(32), .ADDR_WIDTH(ADDR_WIDTH), .USER_WIDTH(0)) lane0_la_apb_rxclk();
	APB_CDC lane0_apb_cdc(
		.upstream(downstreamBus[4]),
		.downstream_pclk(lane0_rxclk),
		.downstream(lane0_la_apb_rxclk));

	APB #(.DATA_WIDTH(32), .ADDR_WIDTH(ADDR_WIDTH), .USER_WIDTH(0)) lane1_la_apb_rxclk();
	APB_CDC lane1_apb_cdc(
		.upstream(downstreamBus[5]),
		.downstream_pclk(lane1_rxclk),
		.downstream(lane1_la_apb_rxclk));

	//TODO: shared, synchronized trigger

	//0x0000_c400
	wire	la0_trig;
	LogicAnalyzer lane0_la(
		.apb(lane0_la_apb_rxclk),

		.rx_clk(lane0_rxclk),
		.rx_data(lane0_rx_data),
		.rx_trigger(la0_trig)
	);

	//0x0000_c500
	wire	la1_trig;
	LogicAnalyzer lane1_la(
		.apb(lane1_la_apb_rxclk),

		.rx_clk(lane1_rxclk),
		.rx_data(lane1_rx_data),
		.rx_trigger(la1_trig)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Synchronize trigger source from wherever it came from down to each LA's clock domain

	ThreeStageSynchronizer #(
		.IN_REG(0)
	) sync_la0_trig (
		.clk_in(),
		.din(la_trig[0]),
		.clk_out(lane0_rxclk),
		.dout(la0_trig)
	);

	ThreeStageSynchronizer #(
		.IN_REG(0)
	) sync_la1_trig (
		.clk_in(),
		.din(la_trig[1]),
		.clk_out(lane1_rxclk),
		.dout(la1_trig)
	);

endmodule
