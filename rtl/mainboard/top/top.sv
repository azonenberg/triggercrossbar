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

module top(
	input wire			clk_200mhz_p,
	input wire			clk_200mhz_n,

	//Tachometers from cooling fans
	input wire[1:0]		fan_tach,

	//Quad SPI interface to MCU
	input wire			qspi_sck,
	input wire			qspi_cs_n,
	inout wire[3:0]		qspi_dq,
	output wire			irq,

	//SPI interface to front panel
	output wire			frontpanel_sck,
	output wire			frontpanel_miso,
	output wire			frontpanel_mosi,
	output wire			frontpanel_cs_n,

	//RGMII interface
	output wire			rgmii_rst_n,

	inout wire			rgmii_mdio,
	output wire			rgmii_mdc,

	input wire			rgmii_rxc,
	input wire			rgmii_rx_dv,
	input wire[3:0]		rgmii_rxd,

	output wire			rgmii_tx_clk,
	output wire			rgmii_tx_en,
	output wire[3:0]	rgmii_txd,

	//SFP+ interface
	input wire			sfp_rx_p,
	input wire			sfp_rx_n,

	output wire			sfp_tx_p,
	output wire			sfp_tx_n,

	input wire			sfp_rx_los,

	output wire[1:0]	sfp_led,

	//GPIO LEDs
	output logic[3:0]	led,

	//H-bridge control for relays
	output wire[3:0]	relay_a,
	output wire[3:0]	relay_b,

	//Trigger outputs
	output wire[11:0]	trig_out,

	//Trigger inputs
	input wire[11:0]	trig_in_p,
	input wire[11:0]	trig_in_n,

	//GTX refclks
	input wire			gtx_refclk_156m25_p,
	input wire			gtx_refclk_156m25_n,
	input wire			gtx_refclk_200m_p,
	input wire			gtx_refclk_200m_n,

	//Front panel "sync" GTX port
	output wire			sync_p,
	output wire			sync_n,

	//Front panel single ended CDR trigger input
	input wire			cdrtrig_p,
	input wire			cdrtrig_n,

	//Front panel differential BERT/pattern generator port
	output wire			tx0_p,
	output wire			tx0_n,

	input wire			rx0_p,
	input wire			rx0_n,

	output wire			tx1_p,
	output wire			tx1_n,

	input wire			rx1_p,
	input wire			rx1_n
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// tie off because silicon errata in front panel STM32, this has to be used as TRST#

	assign frontpanel_miso = 1;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Clock synthesis

	wire	clk_50mhz;
	wire	clk_125mhz;
	wire	clk_250mhz;

	wire	pll_rgmii_lock;

	wire	serdes_refclk_156m25;
	wire	serdes_refclk_200m;

	wire	qpll_lock;
	wire	qpll_refclk;
	wire	qpll_refclk_lost;
	wire	qpll_clkout_10g3125;

	ClockGeneration clk_main(
		.clk_200mhz_p(clk_200mhz_p),
		.clk_200mhz_n(clk_200mhz_n),

		.clk_50mhz(clk_50mhz),
		.clk_125mhz(clk_125mhz),
		.clk_250mhz(clk_250mhz),
		.pll_rgmii_lock(pll_rgmii_lock)
	);

	SerdesClocking clk_serdes(
		.gtx_refclk_156m25_p(gtx_refclk_156m25_p),
		.gtx_refclk_156m25_n(gtx_refclk_156m25_n),
		.gtx_refclk_200m_p(gtx_refclk_200m_p),
		.gtx_refclk_200m_n(gtx_refclk_200m_n),

		.serdes_refclk_156m25(serdes_refclk_156m25),
		.serdes_refclk_200m(serdes_refclk_200m),

		.clk_125mhz(clk_125mhz),

		.qpll_lock(qpll_lock),
		.qpll_refclk(qpll_refclk),
		.qpll_refclk_lost(qpll_refclk_lost),
		.qpll_clkout_10g3125(qpll_clkout_10g3125)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Differential input buffers for LVDS trigger inputs

	wire[11:0]	trig_in_raw;

	DifferentialInputBuffer #(
		.WIDTH(12)
	) ibuf_trigin (
		.pad_in_p(trig_in_p),
		.pad_in_n(trig_in_n),
		.fabric_out(trig_in_raw));

	//Flip a few trigger inputs that had P/N swapped on the PCB for layout reasons
	wire[11:0]	trig_in;
	assign trig_in = trig_in_raw ^ 12'h65;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Output PRBS generation on sync port (also runs CDR trigger input)
	// TODO: Figure out what to do with this

	//Dummy GTX clocking
	wire	cdrtrig_rx_clk;
	wire	cdrtrig_rx_clk_raw;
	wire	prbs_tx_clk;
	wire	prbs_tx_clk_raw;

	BUFG buf_cdrtrig_rx_clk(
		.I(cdrtrig_rx_clk_raw),
		.O(cdrtrig_rx_clk));

	BUFG buf_prbs_tx_clk(
		.I(prbs_tx_clk_raw),
		.O(prbs_tx_clk));

	GTXWrapper prbs_transceiver(
		.sysclk_in(clk_125mhz),

		//TODO: do we need any of this
		.soft_reset_tx_in(1'b0),
		.soft_reset_rx_in(1'b0),
		.dont_reset_on_data_error_in(1'b0),
		.tx_fsm_reset_done_out(),
		.rx_fsm_reset_done_out(),

		//Register access
		.drpclk_in(clk_125mhz),
		.drpaddr_in(9'h0),
		.drpdi_in(16'h0),
		.drpdo_out(),
		.drpen_in(1'b0),
		.drprdy_out(),
		.drpwe_in(1'b0),

		//Tie off unused ports
		.eyescanreset_in(1'b0),
		.eyescandataerror_out(),
		.eyescantrigger_in(1'b0),
		//.rxphmonitor_out(),
		//.rxphslipmonitor_out(),
		.rxmonitorout_out(),
		.rxmonitorsel_in(2'b0),

		//Subsystem resets
		.rxpmareset_in(1'b0),
		.rxresetdone_out(),

		//Transmit interface
		.txusrclk_in(prbs_tx_clk),
		.txusrclk2_in(prbs_tx_clk),
		.data_valid_in(1'b1),
		.txdata_in(32'h5555aaaa),
		.txoutclk_out(prbs_tx_clk_raw),
		.txoutclkfabric_out(),
		.txoutclkpcs_out(),
		.txresetdone_out(),

		//Fabric RX interface
		.rxusrclk_in(cdrtrig_rx_clk),
		.rxusrclk2_in(cdrtrig_rx_clk),
		.rxdata_out(),
		.rxoutclk_out(cdrtrig_rx_clk_raw),
		.rxoutclkfabric_out(),

		//Output pattern selection
		.txprbssel_in(3'b010),	//PRBS15

		//Input PRBS detector
		.rxprbssel_in(3'b010),
		.rxprbserr_out(),

		//Top level diff pairs
		.gtxtxn_out(sync_p),
		.gtxtxp_out(sync_n),
		.gtxrxn_in(cdrtrig_p),
		.gtxrxp_in(cdrtrig_n),

		//Input buffer config
		.rxpolarity_in(1'b0),

		//TX clock configuration
		.txrate_in(0),
		.txratedone_out(),

		//Output swing control and equalizer taps
		.txinhibit_in(1'b0),
		.txpolarity_in(1'b0),
		.txdiffctrl_in(4'h5),
		.txprecursor_in(5'h7),
		.txpostcursor_in(5'h8),

		//Clock to/from CPLL
		//.cpllfbclklost_out(),
		//.cplllock_out(cpll_lock[0]),
		//.cplllockdetclk_in(clk_125mhz),
		//.gtrefclk0_in(serdes_refclk_156m25),
		//.gtrefclk1_in(serdes_refclk_200m),

		//Clock from QPLL
		.qplllock_in(qpll_lock),
		.qpllrefclklost_in(qpll_refclk_lost),
		.qplloutclk_in(qpll_clkout_10g3125),
		.qplloutrefclk_in(qpll_refclk),

		.rx_clk_from_qpll(1),
		.tx_clk_from_qpll(1)
		);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// BERT subsystem (2x GTX)

	wire[1:0]	cpll_lock;

	wire		serdes_config_updated;

	`include "BERTConfig.svh"

	bert_txconfig_t	tx0_config;
	bert_txconfig_t	tx1_config;

	bert_rxconfig_t	rx0_config;
	bert_rxconfig_t	rx1_config;

	wire		mgmt_lane0_en;
	wire		mgmt_lane1_en;
	wire		mgmt_we;
	wire[8:0]	mgmt_addr;
	wire[15:0]	mgmt_wdata;
	wire[15:0]	mgmt_lane0_rdata;
	wire[15:0]	mgmt_lane1_rdata;
	wire		mgmt_lane0_done;
	wire		mgmt_lane1_done;

	wire		mgmt_lane0_rx_rstdone;
	wire		mgmt_lane1_rx_rstdone;

	BERTSubsystem bert(

		.clk_125mhz(clk_125mhz),
		.pll_rgmii_lock(pll_rgmii_lock),

		.qpll_lock(qpll_lock),
		.qpll_refclk(qpll_refclk),
		.qpll_refclk_lost(qpll_refclk_lost),
		.qpll_clkout_10g3125(qpll_clkout_10g3125),

		.serdes_refclk_156m25(serdes_refclk_156m25),
		.serdes_refclk_200m(serdes_refclk_200m),

		.tx0_p(tx0_p),
		.tx0_n(tx0_n),

		.rx0_p(rx0_p),
		.rx0_n(rx0_n),

		.tx1_p(tx1_p),
		.tx1_n(tx1_n),

		.rx1_p(rx1_p),
		.rx1_n(rx1_n),

		.cpll_lock(cpll_lock),

		.clk_250mhz(clk_250mhz),
		.config_updated(serdes_config_updated),
		.tx0_config(tx0_config),
		.tx1_config(tx1_config),
		.rx0_config(rx0_config),
		.rx1_config(rx1_config),
		.mgmt_lane0_en(mgmt_lane0_en),
		.mgmt_lane1_en(mgmt_lane1_en),
		.mgmt_we(mgmt_we),
		.mgmt_addr(mgmt_addr),
		.mgmt_wdata(mgmt_wdata),
		.mgmt_lane0_rdata(mgmt_lane0_rdata),
		.mgmt_lane1_rdata(mgmt_lane1_rdata),
		.mgmt_lane0_done(mgmt_lane0_done),
		.mgmt_lane1_done(mgmt_lane1_done),
		.mgmt_lane0_rx_rstdone(mgmt_lane0_rx_rstdone),
		.mgmt_lane1_rx_rstdone(mgmt_lane1_rx_rstdone)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Network interfaces

	`include "GmiiBus.svh"
	`include "EthernetBus.svh"

	wire					mgmt0_rx_clk_buf;
	EthernetRxBus			mgmt0_rx_bus;
	EthernetTxBus			mgmt0_tx_bus;
	wire					mgmt0_tx_ready;
	wire					mgmt0_link_up;
	lspeed_t				mgmt0_link_speed;

	wire					xg0_mac_rx_clk;
	wire					xg0_mac_tx_clk;
	EthernetRxBus			xg0_mac_rx_bus;
	EthernetTxBus			xg0_mac_tx_bus;
	wire					xg0_link_up;

	NetworkInterfaces network(
		.clk_125mhz(clk_125mhz),
		.clk_250mhz(clk_250mhz),
		.pll_rgmii_lock(pll_rgmii_lock),

		.qpll_lock(qpll_lock),
		.qpll_clkout_10g3125(qpll_clkout_10g3125),
		.qpll_refclk(qpll_refclk),
		.qpll_refclk_lost(qpll_refclk_lost),
		.serdes_refclk_156m25(serdes_refclk_156m25),
		.serdes_refclk_200m(serdes_refclk_200m),

		.sfp_tx_p(sfp_tx_p),
		.sfp_tx_n(sfp_tx_n),
		.sfp_rx_p(sfp_rx_p),
		.sfp_rx_n(sfp_rx_n),
		.sfp_rx_los(sfp_rx_los),
		.sfp_led(sfp_led),

		.rgmii_rst_n(rgmii_rst_n),

		.rgmii_rx_clk(rgmii_rxc),
		.rgmii_rx_dv(rgmii_rx_dv),
		.rgmii_rxd(rgmii_rxd),

		.rgmii_tx_clk(rgmii_tx_clk),
		.rgmii_tx_en(rgmii_tx_en),
		.rgmii_txd(rgmii_txd),

		.xg0_mac_rx_clk(xg0_mac_rx_clk),
		.xg0_mac_rx_bus(xg0_mac_rx_bus),
		.xg0_mac_tx_clk(xg0_mac_tx_clk),
		.xg0_mac_tx_bus(xg0_mac_tx_bus),
		.xg0_link_up(xg0_link_up),

		.mgmt0_rx_clk_buf(mgmt0_rx_clk_buf),
		.mgmt0_rx_bus(mgmt0_rx_bus),
		.mgmt0_tx_bus(mgmt0_tx_bus),
		.mgmt0_tx_ready(mgmt0_tx_ready),
		.mgmt0_link_up(mgmt0_link_up),
		.mgmt0_link_speed(mgmt0_link_speed)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// RX side muxing for SFP vs RGMII PHY to allow management from either

	wire					eth_link_up;
	wire EthernetRxBus		eth_rx_bus;

	NetworkRxMuxing rx_mux(
		.clk_250mhz(clk_250mhz),

		.mgmt0_link_up(mgmt0_link_up),
		.mgmt0_rx_clk_buf(mgmt0_rx_clk_buf),
		.mgmt0_rx_bus(mgmt0_rx_bus),

		.xg0_link_up(xg0_link_up),
		.xg0_mac_rx_clk(xg0_mac_rx_clk),
		.xg0_mac_rx_bus(xg0_mac_rx_bus),

		.eth_link_up(eth_link_up),
		.eth_rx_bus(eth_rx_bus));

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Curve25519 crypto_scalarmult accelerator (for speeding up SSH key exchange)

	wire		crypt_en;
	wire[255:0]	crypt_work_in;
	wire[255:0]	crypt_e;
	wire		crypt_out_valid;
	wire[255:0]	crypt_work_out;

	wire		crypt_dsa_en;
	wire		crypt_dsa_load;
	wire		crypt_dsa_rd;
	wire		crypt_dsa_done;
	wire[1:0]	crypt_dsa_addr;

	X25519_ScalarMult crypt25519(
		.clk(clk_250mhz),

		//Common inputs
		.e(crypt_e),
		.work_in(crypt_work_in),

		//ECDH signals
		.dh_en(crypt_en),

		//ECDSA signals
		.dsa_en(crypt_dsa_en),
		.dsa_load(crypt_dsa_load),
		.dsa_rd(crypt_dsa_rd),
		.dsa_done(crypt_dsa_done),
		.dsa_addr(crypt_dsa_addr),

		//Common outputs
		.out_valid(crypt_out_valid),
		.work_out(crypt_work_out)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Relays

	wire			toggle_en;
	wire			toggle_dir;
	wire[1:0]		toggle_channel;
	wire			toggle_done;

	RelayController relays(
		.clk_250mhz(clk_250mhz),

		.toggle_en(toggle_en),
		.toggle_dir(toggle_dir),
		.toggle_channel(toggle_channel),
		.toggle_done(toggle_done),

		.relay_a(relay_a),
		.relay_b(relay_b)
		);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The actual crossbar itself

	`include "CrossbarTypes.svh"

	muxsel_t[11:0]	muxsel;
	wire[11:0]		trig_in_led;
	wire[11:0]		trig_out_led;

	(* keep_hierarchy = "yes" *)
	CrossbarMatrix matrix(
		.trig_in(trig_in),
		.trig_out(trig_out),

		.muxsel(muxsel),

		.clk_250mhz(clk_250mhz),
		.trig_in_led(trig_in_led),
		.trig_out_led(trig_out_led)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Management register interface

	ManagementSubsystem mgmt(
		.sys_clk(clk_250mhz),
		.clk_sysinfo(clk_50mhz),

		.qspi_sck(qspi_sck),
		.qspi_cs_n(qspi_cs_n),
		.qspi_dq(qspi_dq),
		.irq(irq),

		.eth_rx_clk(clk_250mhz),
		.eth_rx_bus(eth_rx_bus),
		.eth_link_up(eth_link_up),

		.mgmt0_tx_clk(clk_125mhz),
		.mgmt0_tx_bus(mgmt0_tx_bus),
		.mgmt0_tx_ready(mgmt0_tx_ready),

		.mgmt0_mdio(rgmii_mdio),
		.mgmt0_mdc(rgmii_mdc),

		.xg0_rx_clk(xg0_mac_rx_clk),
		.xg0_link_up(xg0_link_up),
		.xg0_tx_clk(xg0_mac_tx_clk),
		.xg0_tx_bus(xg0_mac_tx_bus),

		.fan_tach(fan_tach),

		.relay_en(toggle_en),
		.relay_dir(toggle_dir),
		.relay_channel(toggle_channel),
		.relay_done(toggle_done),

		.frontpanel_sck(frontpanel_sck),
		.frontpanel_mosi(frontpanel_mosi),
		.frontpanel_cs_n(frontpanel_cs_n),
		.trig_in_led(trig_in_led),
		.trig_out_led(trig_out_led),

		.muxsel(muxsel),

		.serdes_config_updated(serdes_config_updated),
		.rx0_config(rx0_config),
		.rx1_config(rx1_config),
		.tx0_config(tx0_config),
		.tx1_config(tx1_config),
		.mgmt_lane0_en(mgmt_lane0_en),
		.mgmt_lane1_en(mgmt_lane1_en),
		.mgmt_we(mgmt_we),
		.mgmt_addr(mgmt_addr),
		.mgmt_wdata(mgmt_wdata),
		.mgmt_lane0_rdata(mgmt_lane0_rdata),
		.mgmt_lane1_rdata(mgmt_lane1_rdata),
		.mgmt_lane0_done(mgmt_lane0_done),
		.mgmt_lane1_done(mgmt_lane1_done),
		.mgmt_lane0_rx_rstdone(mgmt_lane0_rx_rstdone),
		.mgmt_lane1_rx_rstdone(mgmt_lane1_rx_rstdone),

		.clk_crypt(clk_250mhz),
		.crypt_en(crypt_en),
		.crypt_work_in(crypt_work_in),
		.crypt_work_out(crypt_work_out),
		.crypt_e(crypt_e),
		.crypt_out_valid(crypt_out_valid),
		.crypt_dsa_en(crypt_dsa_en),
		.crypt_dsa_load(crypt_dsa_load),
		.crypt_dsa_rd(crypt_dsa_rd),
		.crypt_dsa_done(crypt_dsa_done),
		.crypt_dsa_addr(crypt_dsa_addr)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debug LEDs etc

	always_comb begin
		//led[0]		= xg0_link_up;
		//led[1]		= mgmt0_link_up;
		led[0]		= 0;
		led[1]		= qpll_lock;
		led[3:2]	= cpll_lock;
	end

endmodule
