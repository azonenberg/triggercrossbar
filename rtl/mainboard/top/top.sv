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

	//RGMII interface
	output logic		rgmii_rst_n	= 0,
	input wire			rgmii_rxc,

	//GPIO LEDs
	output logic[3:0]	led,

	//H-bridge control for relays
	output logic[3:0]	relay_a		= 0,
	output logic[3:0]	relay_b		= 0,

	//Trigger outputs
	output logic[11:0]	trig_out	= 0,

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
	input wire			cdrtrig_n
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Clock synthesis

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

	//Dummy GTX for PRBS generation
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

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Output PRBS generation

	wire[4:0]	tx_swing;			//best results for 10gbase-R 'h05
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
	// Reset the Ethernet PHY

	//Bring up the PHYs after a little while
	logic[18:0] eth_rst_count = 1;
	always_ff @(posedge clk_125mhz) begin
		if(eth_rst_count == 0) begin
			rgmii_rst_n		<= 1;
		end
		else
			eth_rst_count	<= eth_rst_count + 1'h1;
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Cycle all of the relays once during startup and place them in output mode

	//Relays spec max 5ms operating time
	//1M cycles at 125 MHz is 8 ns which should be plenty of margin

	logic[1:0]	relayIndex		= 0;
	logic[19:0]	relayCount		= 0;

	enum logic[1:0]
	{
		RELAY_NEXT	= 2'h0,
		RELAY_ON	= 2'h1,
		RELAY_WAIT	= 2'h2,
		RELAY_DONE	= 2'h3
	} relayState	= RELAY_NEXT;

	always_ff @(posedge clk_125mhz) begin

		case(relayState)

			//Start doing the next relay
			RELAY_NEXT: begin

				//not sure which direction this selects but i guess we'll find out
				relay_a[relayIndex]	<= 0;
				relay_b[relayIndex]	<= 1;

				relayCount			<= 1;
				relayState			<= RELAY_ON;

			end

			//Coil energized
			RELAY_ON: begin
				relayCount	<= relayCount + 1;

				//Done with this cycle
				if(relayCount == 0) begin

					//De-energize coil
					relay_a	<= 0;
					relay_b	<= 0;

					//Finished last one?
					if(relayIndex == 3)
						relayState <= RELAY_DONE;

					//Nope, move on
					else begin
						relayIndex	<= relayIndex + 1;
						relayState	<= RELAY_WAIT;
						relayCount	<= 1;
					end

				end

			end

			//Wait a short time before moving to the next one
			RELAY_WAIT: begin
				relayCount	<= relayCount + 1;

				if(relayCount == 0)
					relayState	<= RELAY_NEXT;
			end

			//Done updating, don't touch again
			RELAY_DONE: begin
			end

		endcase

	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debug LEDs etc

	/*logic[22:0] count = 0;
	always_ff @(posedge rgmii_rxc) begin
		count <= count + 1;

		if(count == 0)
			led			<= led + 1;
	end*/

	always_comb begin
		led[0]		= pll_rgmii_lock;
		led[1]		= qpll_lock;
		led[3:2]	= 2'b11;
	end

	always_ff @(posedge clk_250mhz) begin
		//toggle trigger outputs at 125 MHz
		trig_out		<= ~trig_out;
	end

endmodule
