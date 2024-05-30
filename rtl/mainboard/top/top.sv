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
import EthernetBus::*;
import CrossbarTypes::*;

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
	input wire			frontpanel_miso,
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
	input wire			rx1_n,

	//PMOD connector
	inout wire[7:0]		pmod_dq,

	//QSPI flash lines
	inout wire[3:0]		flash_dq,
	output wire			flash_cs_n
	//flash SCK is CCLK pin from STARTUPE2
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
	// SPI flash controller for storing bitstream

	wire	cclk;

	//DQ2 / WP and DQ3 / HOLD aren't used for now, tie high
	assign flash_dq[3:2] = 2'b11;

	//Drive DQ1 / SO to high-Z
	assign flash_dq[1] = 1'bz;

	//STARTUP block
	wire	ring_clk;
	STARTUPE2 startup(
		.CLK(ring_clk),
		.GSR(1'b0),
		.GTS(1'b0),
		.KEYCLEARB(1'b1),
		.PACK(1'b0),
		.PREQ(),
		.USRCCLKO(cclk),
		.USRCCLKTS(1'b0),
		.USRDONEO(1'b1),
		.USRDONETS(1'b0),
		.CFGCLK(),
		.CFGMCLK(ring_clk),
		.EOS()
		);

	//SPI bus controller
	APB #(.DATA_WIDTH(16), .ADDR_WIDTH(SMOL_ADDR_WIDTH), .USER_WIDTH(0)) flashBus();
	APB_SPIHostInterface flash_spi(
		.apb(flashBus),

		.spi_sck(cclk),
		.spi_mosi(flash_dq[0]),
		.spi_miso(flash_dq[1]),
		.spi_cs_n(flash_cs_n)
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

	localparam SMOL_ADDR_WIDTH 		= 10;
	localparam BIG_ADDR_WIDTH		= 12;

	APB #(.DATA_WIDTH(16), .ADDR_WIDTH(SMOL_ADDR_WIDTH), .USER_WIDTH(0)) bertBus();

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

		.apb(bertBus)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Network interfaces

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

	APB #(.DATA_WIDTH(16), .ADDR_WIDTH(SMOL_ADDR_WIDTH), .USER_WIDTH(0)) mdioBus();

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
		.mgmt0_link_speed(mgmt0_link_speed),

		.mgmt0_mdio(rgmii_mdio),
		.mgmt0_mdc(rgmii_mdc),

		//APB connections
		.mdioBus(mdioBus)
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

	APB #(.DATA_WIDTH(16), .ADDR_WIDTH(SMOL_ADDR_WIDTH), .USER_WIDTH(0)) cryptBus();

	APB_Curve25519 crypt25519(
		.apb(cryptBus)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Relays for bidirectional IOs

	APB #(.DATA_WIDTH(16), .ADDR_WIDTH(SMOL_ADDR_WIDTH), .USER_WIDTH(0)) relayBus();

	wire[3:0]	relay_state;

	APB_RelayController relays(
		.apb(relayBus),

		.relay_state(relay_state),

		.relay_a(relay_a),
		.relay_b(relay_b)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The actual crossbar itself

	wire[11:0]		trig_in_led;
	wire[11:0]		trig_out_led;

	APB #(.DATA_WIDTH(16), .ADDR_WIDTH(SMOL_ADDR_WIDTH), .USER_WIDTH(0)) crossbarBus();

	APB_CrossbarMatrix matrix(
		.apb(crossbarBus),

		.trig_in(trig_in),
		.trig_out(trig_out),

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

		.relayBus(relayBus),
		.mdioBus(mdioBus),
		.crossbarBus(crossbarBus),
		.bertBus(bertBus),
		.cryptBus(cryptBus),
		.flashBus(flashBus),

		.relay_state(relay_state),

		.xg0_rx_clk(xg0_mac_rx_clk),
		.xg0_link_up(xg0_link_up),
		.xg0_tx_clk(xg0_mac_tx_clk),
		.xg0_tx_bus(xg0_mac_tx_bus),

		.fan_tach(fan_tach),

		.frontpanel_sck(frontpanel_sck),
		.frontpanel_mosi(frontpanel_mosi),
		.frontpanel_miso(frontpanel_miso),
		.frontpanel_cs_n(frontpanel_cs_n),
		.trig_in_led(trig_in_led),
		.trig_out_led(trig_out_led)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debug LEDs etc

	always_comb begin
		led[3:2] = 0;
		led[1] = bert.lane0_64b66b_locked;
		led[0] = bert.lane0_8b10b_locked_all;
	end

	assign pmod_dq[7:0] = 8'd0;

endmodule
