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

	//Dummy GTX clocking
	wire	cdrtrig_rxclk;
	wire	cdrtrig_rxclk_raw;
	wire	prbs_tx_clk;
	wire	prbs_tx_clk_raw;

	BUFG buf_cdrtrig_rxclk(
		.I(cdrtrig_rxclk_raw),
		.O(cdrtrig_rxclk));

	BUFG buf_prbs_tx_clk(
		.I(prbs_tx_clk_raw),
		.O(prbs_tx_clk));

	wire[3:0]	tx_swing;			//best results for 10gbase-R 'h05
	wire[4:0]	tx_precursor;		//best results for 10gbase-R 'h07
	wire[4:0]	tx_postcursor;		//best results for 10Gbase-R 'h08
	wire[6:0]	tx_maincursor;		//best results for 10Gbase-R 'h00

	vio_0 vio(
		.clk(clk_125mhz),
		.probe_out0(tx_swing),
		.probe_out1(tx_precursor),
		.probe_out2(tx_postcursor),
		.probe_out3(tx_maincursor));

	gtx_syncout_cdrtrig prbs_transceiver(
		.sysclk_in(clk_125mhz),

		//TODO: do we need any of this
		.soft_reset_tx_in(1'b0),
		.soft_reset_rx_in(1'b0),
		.dont_reset_on_data_error_in(1'b0),
		.gt0_tx_fsm_reset_done_out(),
		.gt0_rx_fsm_reset_done_out(),

		//Tie off unused ports
		.gt0_drpaddr_in(9'b0),
		.gt0_drpclk_in(clk_125mhz),
		.gt0_drpdi_in(16'b0),
		.gt0_drpdo_out(),
		.gt0_drpen_in(1'b0),
		.gt0_drprdy_out(),
		.gt0_drpwe_in(1'b0),
		.gt0_dmonitorout_out(),
		.gt0_eyescanreset_in(1'b0),
		.gt0_eyescandataerror_out(),
		.gt0_eyescantrigger_in(1'b0),
		//.gt0_rxphmonitor_out(),
		//.gt0_rxphslipmonitor_out(),
		.gt0_rxmonitorout_out(),
		.gt0_rxmonitorsel_in(2'b0),
		.gt0_gtrxreset_in(1'b0),
		.gt0_gttxreset_in(1'b0),

		//Transmit interface
		.gt0_txuserrdy_in(pll_rgmii_lock),
		.gt0_txusrclk_in(prbs_tx_clk),
		.gt0_txusrclk2_in(prbs_tx_clk),
		.gt0_data_valid_in(1'b1),
		.gt0_txdata_in(32'h00000000),
		.gt0_txoutclk_out(prbs_tx_clk_raw),
		.gt0_txoutclkfabric_out(),
		.gt0_txoutclkpcs_out(),
		.gt0_txresetdone_out(),

		//Fabric RX interface
		.gt0_rxusrclk_in(cdrtrig_rxclk),
		.gt0_rxusrclk2_in(cdrtrig_rxclk),
		.gt0_rxdata_out(),
		.gt0_rxoutclk_out(cdrtrig_rxclk_raw),
		.gt0_rxoutclkfabric_out(),
		.gt0_rxdatavalid_out(),
		.gt0_rxheader_out(),
		.gt0_rxheadervalid_out(),
		.gt0_rxgearboxslip_in(1'b0),

		//Output pattern selection
		.gt0_txprbssel_in(3'b010),	//PRBS-15

		//Top level diff pairs
		.gt0_gtxtxn_out(sync_p),
		.gt0_gtxtxp_out(sync_n),
		.gt0_gtxrxn_in(cdrtrig_p),
		.gt0_gtxrxp_in(cdrtrig_n),

		//Output swing control and equalizer taps
		.gt0_txdiffctrl_in(/*4'b0100*/tx_swing),	//543 mV p-p differential
		.gt0_txprecursor_in(tx_precursor),
		.gt0_txpostcursor_in(tx_postcursor),
		.gt0_txmaincursor_in(tx_maincursor),

		//Clock from QPLL
		.gt0_qplllock_in(qpll_lock),
		.gt0_qpllrefclklost_in(qpll_refclk_lost),
		.gt0_qpllreset_out(),
		.gt0_qplloutclk_in(qpll_clkout_10g3125),
		.gt0_qplloutrefclk_in(qpll_refclk)
		);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// BERT subsystem (2x GTX)

	wire[1:0]	cpll_lock;

	wire		serdes_config_updated;

	wire[2:0]	rx0_prbs_mode;
	wire[2:0]	rx1_prbs_mode;

	wire[2:0]	tx0_prbs_mode;
	wire[2:0]	tx1_prbs_mode;

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
		.rx0_prbs_mode(rx0_prbs_mode),
		.rx1_prbs_mode(rx1_prbs_mode),
		.tx0_prbs_mode(tx0_prbs_mode),
		.tx1_prbs_mode(tx1_prbs_mode)
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
	// TX side muxing for SFP vs RGMII PHY to allow management from either

	//TODO: actually make this work, we need to handle different bus widths which will mean changes to ManagementTxFifo

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Curve25519 crypto_scalarmult accelerator (for speeding up SSH key exchange)

	wire		crypt_en;
	wire[255:0]	crypt_work_in;
	wire[255:0]	crypt_e;
	wire		crypt_out_valid;
	wire[255:0]	crypt_work_out;

	X25519_ScalarMult crypt25519(
		.clk(clk_250mhz),
		.en(crypt_en),
		.work_in(crypt_work_in),
		.e(crypt_e),
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

	(* keep_hierarchy = "yes" *)
	CrossbarMatrix matrix(
		.trig_in(trig_in),
		.trig_out(trig_out),

		.muxsel(muxsel)
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

		.fan_tach(fan_tach),

		.relay_en(toggle_en),
		.relay_dir(toggle_dir),
		.relay_channel(toggle_channel),
		.relay_done(toggle_done),

		.muxsel(muxsel),

		.serdes_config_updated(serdes_config_updated),
		.rx0_prbs_mode(rx0_prbs_mode),
		.rx1_prbs_mode(rx1_prbs_mode),
		.tx0_prbs_mode(tx0_prbs_mode),
		.tx1_prbs_mode(tx1_prbs_mode),

		.clk_crypt(clk_250mhz),
		.crypt_en(crypt_en),
		.crypt_work_in(crypt_work_in),
		.crypt_work_out(crypt_work_out),
		.crypt_e(crypt_e),
		.crypt_out_valid(crypt_out_valid)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debug LEDs etc

	always_comb begin
		led[0]		= xg0_link_up;
		led[1]		= mgmt0_link_up;
		led[3:2]	= cpll_lock;
	end

endmodule
