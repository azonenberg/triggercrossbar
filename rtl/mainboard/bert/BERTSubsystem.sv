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

`include "BERTConfig.svh"

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

	//Control registers (clk_250mhz domain, synchronized internally)
	input wire					clk_250mhz,
	input wire					config_updated,			//update strobe

	input wire bert_txconfig_t	tx0_config,
	input wire bert_txconfig_t	tx1_config,

	input wire bert_rxconfig_t	rx0_config,
	input wire bert_rxconfig_t	rx1_config,

	input wire					mgmt_lane0_en,
	input wire					mgmt_lane1_en,
	input wire					mgmt_we,
	input wire[8:0]				mgmt_addr,
	input wire[15:0]			mgmt_wdata,
	output wire[15:0]			mgmt_lane0_rdata,
	output wire[15:0]			mgmt_lane1_rdata,
	output wire					mgmt_lane0_done,
	output wire					mgmt_lane1_done,
	output wire					mgmt_lane0_rx_rstdone,
	output wire					mgmt_lane1_rx_rstdone,

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
		.clk_out(clk_250mhz),
		.dout(mgmt_lane0_rx_rstdone));

	ThreeStageSynchronizer sync_lane1_rx_rst_done(
		.clk_in(lane1_rxclk),
		.din(rx1_rxreset_done),
		.clk_out(clk_250mhz),
		.dout(mgmt_lane1_rx_rstdone));

	RegisterSynchronizer #(
		.WIDTH($bits(bert_rxconfig_t)),
		.INIT(0),
		.IN_REG(1)
	) sync_rx0_config (
		.clk_a(clk_250mhz),
		.en_a(config_updated),
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
		.clk_a(clk_250mhz),
		.en_a(config_updated),
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
		.clk_a(clk_250mhz),
		.en_a(config_updated),
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
		.clk_a(clk_250mhz),
		.en_a(config_updated),
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
		.clk_in(clk_250mhz),
		.din(tx0_config.tx_reset),
		.clk_out(clk_125mhz),
		.dout(tx0_rst_sync));

	ThreeStageSynchronizer sync_lane1_tx_reset(
		.clk_in(clk_250mhz),
		.din(tx1_config.tx_reset),
		.clk_out(clk_125mhz),
		.dout(tx1_rst_sync));

	ThreeStageSynchronizer sync_lane0_rx_reset(
		.clk_in(clk_250mhz),
		.din(rx0_config.rx_reset),
		.clk_out(clk_125mhz),
		.dout(rx0_rst_sync));

	ThreeStageSynchronizer sync_lane1_rx_reset(
		.clk_in(clk_250mhz),
		.din(rx1_config.rx_reset),
		.clk_out(clk_125mhz),
		.dout(rx1_rst_sync));


	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Output PRBS generation on TX0 port

	wire		lane0_prbs_err;
	wire[31:0]	lane0_rx_data;

	wire		lane0_drp_en;
	wire		lane0_drp_we;
	wire[8:0]	lane0_drp_addr;
	wire[15:0]	lane0_drp_di;
	wire[15:0]	lane0_drp_do;
	wire		lane0_drp_rdy;

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

	wire		lane1_drp_en;
	wire		lane1_drp_we;
	wire[8:0]	lane1_drp_addr;
	wire[15:0]	lane1_drp_di;
	wire[15:0]	lane1_drp_do;
	wire		lane1_drp_rdy;

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
	// DRP arbitration and clock domain shifting

	DRPClockDomainShifting lane0_drp_cdc(
		.mgmt_clk(clk_250mhz),
		.mgmt_en(mgmt_lane0_en),
		.mgmt_wr(mgmt_we),
		.mgmt_addr(mgmt_addr),
		.mgmt_wdata(mgmt_wdata),
		.mgmt_rdata(mgmt_lane0_rdata),
		.mgmt_done(mgmt_lane0_done),

		.drp_clk(clk_125mhz),
		.drp_en(lane0_drp_en),
		.drp_we(lane0_drp_we),
		.drp_addr(lane0_drp_addr),
		.drp_di(lane0_drp_di),
		.drp_do(lane0_drp_do),
		.drp_rdy(lane0_drp_rdy)
		);

	DRPClockDomainShifting lane1_drp_cdc(
		.mgmt_clk(clk_250mhz),
		.mgmt_en(mgmt_lane1_en),
		.mgmt_wr(mgmt_we),
		.mgmt_addr(mgmt_addr),
		.mgmt_wdata(mgmt_wdata),
		.mgmt_rdata(mgmt_lane1_rdata),
		.mgmt_done(mgmt_lane1_done),

		.drp_clk(clk_125mhz),
		.drp_en(lane1_drp_en),
		.drp_we(lane1_drp_we),
		.drp_addr(lane1_drp_addr),
		.drp_di(lane1_drp_di),
		.drp_do(lane1_drp_do),
		.drp_rdy(lane1_drp_rdy)
		);

endmodule
