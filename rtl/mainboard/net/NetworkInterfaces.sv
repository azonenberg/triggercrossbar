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
	@brief Contains all Ethernet interfaces
 */
module NetworkInterfaces(

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Global clocks

	input wire					clk_125mhz,
	input wire					clk_250mhz,

	input wire					pll_rgmii_lock,

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// SERDES clocking

	input wire					qpll_lock,
	input wire					qpll_clkout_10g3125,

	input wire					qpll_refclk,
	input wire					qpll_refclk_lost,

	input wire					serdes_refclk_156m25,
	input wire					serdes_refclk_200m,

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// 10G SFP+

	//SFP+ interface (polarity inverted on both legs)
	output wire					sfp_tx_p,
	output wire					sfp_tx_n,
	input wire					sfp_rx_p,
	input wire					sfp_rx_n,

	input wire					sfp_rx_los,

	output logic[1:0]			sfp_led,

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// RGMII PHY

	input wire					rgmii_rx_clk,
	input wire					rgmii_rx_dv,
	input wire[3:0]				rgmii_rxd,

	output wire					rgmii_tx_clk,
	output wire					rgmii_tx_en,
	output wire[3:0]			rgmii_txd,

	output logic				rgmii_rst_n = 0,

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Output MAC buses

	output wire					xg0_mac_rx_clk,
	output wire EthernetRxBus	xg0_mac_rx_bus,

	output wire					xg0_mac_tx_clk,
	input wire EthernetTxBus	xg0_mac_tx_bus,

	output wire					xg0_link_up,

	output wire					mgmt0_rx_clk_buf,
	output EthernetRxBus		mgmt0_rx_bus,
	input EthernetTxBus			mgmt0_tx_bus,
	output wire					mgmt0_tx_ready,
	output wire					mgmt0_link_up,
	output lspeed_t				mgmt0_link_speed,

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Register interfaces for controlling stuff

	APB.completer				mdioBus,

	inout wire					mgmt0_mdio,
	output wire					mgmt0_mdc
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// PHY reset counter

	//Bring up the PHYs after a little while
	logic[18:0] eth_rst_count = 1;
	always_ff @(posedge clk_125mhz) begin
		if(eth_rst_count == 0)
			rgmii_rst_n		<= 1;
		else
			eth_rst_count	<= eth_rst_count + 1'h1;
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// 10G SFP+ uplink (xg0)

	wire			xg0_rx_clk;
	wire			xg0_tx_clk;

	wire			xg0_rx_clk_raw;
	wire			xg0_tx_clk_raw;

	wire			xg0_rx_data_valid;
	wire			xg0_rx_header_valid;
	wire[1:0]		xg0_rx_header;
	wire[31:0]		xg0_rx_data;
	wire			xg0_rx_bitslip;

	wire[5:0]		xg0_tx_sequence;
	wire[1:0]		xg0_tx_header;
	wire[31:0]		xg0_tx_data;

	wire			xg0_remote_fault;

	//TODO: performance counters

	//Clock buffers
	ClockBuffer #(
		.TYPE("GLOBAL"),
		.CE("NO")
	) clk_buf_xg0_tx_clk (
		.clkin(xg0_tx_clk_raw),
		.ce(1'b1),
		.clkout(xg0_tx_clk)
		);

	ClockBuffer #(
		.TYPE("GLOBAL"),
		.CE("NO")
	) clk_buf_xg0_rx_clk (
		.clkin(xg0_rx_clk_raw),
		.ce(1'b1),
		.clkout(xg0_rx_clk)
		);

	sfp_transceiver xg_transceiver(

		.sysclk_in(clk_125mhz),
		.soft_reset_tx_in(1'b0),
		.soft_reset_rx_in(1'b0),
		.dont_reset_on_data_error_in(1'b1),
		.gt0_tx_fsm_reset_done_out(),
		.gt0_rx_fsm_reset_done_out(),
		.gt0_data_valid_in(1'b1),

		//QPLL clocks
		.gt0_qplllock_in(qpll_lock),
		.gt0_qpllrefclklost_in(qpll_refclk_lost),
		.gt0_qpllreset_out(),
		.gt0_qplloutclk_in(qpll_clkout_10g3125),
		.gt0_qplloutrefclk_in(qpll_refclk),

		//DRP - not used, tie off
		.gt0_drpaddr_in(9'b0),
		.gt0_drpclk_in(clk_125mhz),
		.gt0_drpdi_in(16'b0),
		.gt0_drpdo_out(),
		.gt0_drpen_in(1'b0),
		.gt0_drprdy_out(),
		.gt0_drpwe_in(1'b0),

		//Polarity control
		.gt0_rxpolarity_in(1'b0),	//no inversion
		.gt0_txpolarity_in(1'b1),	//Invert TX to compensate for PCB pair swap

		//Tie off other miscellaneous ports for unused interfaces
		.gt0_dmonitorout_out(),
		.gt0_eyescanreset_in(1'b0),
		.gt0_eyescandataerror_out(),
		.gt0_eyescantrigger_in(1'b0),
		.gt0_rxdfelpmreset_in(1'b0),
		.gt0_rxmonitorout_out(),
		.gt0_rxmonitorsel_in(2'b0),

		//Fabric RX interface
		.gt0_rxusrclk_in(xg0_rx_clk),
		.gt0_rxusrclk2_in(xg0_rx_clk),
		.gt0_rxdata_out(xg0_rx_data),
		.gt0_rxoutclk_out(xg0_rx_clk_raw),
		.gt0_rxoutclkfabric_out(),
		.gt0_rxdatavalid_out(xg0_rx_data_valid),
		.gt0_rxheader_out(xg0_rx_header),
		.gt0_rxheadervalid_out(xg0_rx_header_valid),
		.gt0_rxgearboxslip_in(xg0_rx_bitslip),

		//Reset controls
		.gt0_gtrxreset_in(1'b0),
		.gt0_rxpmareset_in(1'b0),
		.gt0_rxresetdone_out(),
		.gt0_gttxreset_in(1'b0),
		.gt0_rxuserrdy_in(1'b1),
		.gt0_txuserrdy_in(1'b1),
		.gt0_txresetdone_out(),

		//TX driver control
		.gt0_txpostcursor_in(5'h06),		//values determined from front panel TX1 port assuming same pcb loss to sfp
		.gt0_txprecursor_in(5'h00),
		.gt0_txmaincursor_in(6'b0),
		.gt0_txdiffctrl_in(5'h06),

		//Fabric TX interface
		.gt0_txusrclk_in(xg0_tx_clk),
		.gt0_txusrclk2_in(xg0_tx_clk),
		.gt0_txdata_in(xg0_tx_data),
		.gt0_txoutclk_out(xg0_tx_clk_raw),
		.gt0_txoutclkfabric_out(),
		.gt0_txoutclkpcs_out(),
		.gt0_txheader_in(xg0_tx_header),
		.gt0_txsequence_in({1'b0, xg0_tx_sequence}),

		//Top level I/Os
		.gt0_gtxrxp_in(sfp_rx_p),
		.gt0_gtxrxn_in(sfp_rx_n),
		.gt0_gtxtxp_out(sfp_tx_p),
		.gt0_gtxtxn_out(sfp_tx_n)
	);

	XGMACWrapper port_xg0(
		.rx_clk(xg0_rx_clk),
		.tx_clk(xg0_tx_clk),

		.rx_data_valid(xg0_rx_data_valid),
		.rx_header_valid(xg0_rx_header_valid),
		.rx_header(xg0_rx_header),
		.rx_data(xg0_rx_data),
		.rx_bitslip(xg0_rx_bitslip),

		.tx_sequence(xg0_tx_sequence),
		.tx_header(xg0_tx_header),
		.tx_data(xg0_tx_data),

		.sfp_los(sfp_rx_los),

		.mac_rx_clk(xg0_mac_rx_clk),
		.mac_rx_bus(xg0_mac_rx_bus),

		.mac_tx_clk(xg0_mac_tx_clk),
		.mac_tx_bus(xg0_mac_tx_bus),

		.link_up(xg0_link_up),
		.remote_fault(xg0_remote_fault)
	);

	//Debug: LEDs for link status
	//TODO: activity indicator
	always_comb begin
		sfp_led[0]	= xg0_link_up;
		sfp_led[1]	= xg0_remote_fault;
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// RGMII PHY for mgmt0

	RGMIIMACWrapper #(
		.CLK_BUF_TYPE("LOCAL"),
		.PHY_INTERNAL_DELAY_RX(1)
	) port_mgmt0 (
		.clk_125mhz(clk_125mhz),
		.clk_250mhz(clk_250mhz),

		.rgmii_rxc(rgmii_rx_clk),
		.rgmii_rxd(rgmii_rxd),
		.rgmii_rx_ctl(rgmii_rx_dv),

		.rgmii_txc(rgmii_tx_clk),
		.rgmii_txd(rgmii_txd),
		.rgmii_tx_ctl(rgmii_tx_en),

		.mac_rx_clk(mgmt0_rx_clk_buf),
		.mac_rx_bus(mgmt0_rx_bus),

		.mac_tx_bus(mgmt0_tx_bus),
		.mac_tx_ready(mgmt0_tx_ready),

		.link_up(mgmt0_link_up),
		.link_speed(mgmt0_link_speed)
		);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// MDIO interface to mgmt0

	APB_MDIO #(
		.CLK_DIV(75)
	) mdio (
		.apb(mdioBus),

		.mdio(mgmt0_mdio),
		.mdc(mgmt0_mdc)
	);

endmodule
