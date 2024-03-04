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

module BERTSubsystem(

	//Top level clocks
	input wire			clk_125mhz,
	input wire			pll_rgmii_lock,

	//QPLL signals
	input wire			qpll_lock,
	input wire			qpll_refclk,
	input wire			qpll_refclk_lost,
	input wire			qpll_clkout_10g3125,

	//Refclks to CPLLs
	input wire			serdes_refclk_156m25,
	input wire			serdes_refclk_200m,

	//Front panel differential BERT/pattern generator port
	output wire			tx0_p,
	output wire			tx0_n,

	input wire			rx0_p,
	input wire			rx0_n,

	output wire			tx1_p,
	output wire			tx1_n,

	input wire			rx1_p,
	input wire			rx1_n,

	//Status outputs
	output wire[1:0]	cpll_lock
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Controls for drive strength and FFT

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

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Output PRBS generation on TX0 port

	wire lane0_rxclk;
	wire lane0_txclk;

	wire lane0_rxclk_raw;
	wire lane0_txclk_raw;

	BUFG bufg_lane0_rx(.I(lane0_rxclk_raw), .O(lane0_rxclk));
	BUFH bufh_lane0_tx(.I(lane0_txclk_raw), .O(lane0_txclk));

	gtx_frontlane0 lane0_transceiver(
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
		.gt0_txusrclk_in(lane0_txclk),
		.gt0_txusrclk2_in(lane0_txclk),
		.gt0_data_valid_in(1'b1),
		.gt0_txdata_in(32'h00000000),
		.gt0_txoutclk_out(lane0_txclk_raw),
		.gt0_txoutclkfabric_out(),
		.gt0_txoutclkpcs_out(),
		.gt0_txresetdone_out(),

		//Fabric RX interface
		.gt0_rxusrclk_in(lane0_rxclk),
		.gt0_rxusrclk2_in(lane0_rxclk),
		.gt0_rxdata_out(),
		//.gt0_rxoutclk_out(lane0_rxclk_raw),
		.gt0_rxoutclkfabric_out(),

		//Output pattern selection
		.gt0_txprbssel_in(3'b010),	//PRBS-15

		//Top level diff pairs
		.gt0_gtxtxn_out(tx0_p),
		.gt0_gtxtxp_out(tx0_n),
		.gt0_gtxrxn_in(rx0_p),
		.gt0_gtxrxp_in(rx0_n),

		//Output swing control and equalizer taps
		.gt0_txdiffctrl_in(/*4'b0100*/tx_swing),	//04 works well
		.gt0_txprecursor_in(tx_precursor),			//00 works well
		.gt0_txpostcursor_in(tx_postcursor),		//03 works well
		.gt0_txmaincursor_in(tx_maincursor),		//00 works well

		//Clock to/from CPLL
		.gt0_cpllfbclklost_out(),
		.gt0_cplllock_out(cpll_lock[0]),
		.gt0_cplllockdetclk_in(clk_125mhz),
		.gt0_cpllreset_in(1'b0),
		.gt0_gtrefclk0_in(serdes_refclk_156m25),
		.gt0_gtrefclk1_in(serdes_refclk_200m),

		//Clock from QPLL
		//.gt0_qplllock_in(qpll_lock),
		//.gt0_qpllrefclklost_in(qpll_refclk_lost),
		//.gt0_qpllreset_out(),
		.gt0_qplloutclk_in(qpll_clkout_10g3125),
		.gt0_qplloutrefclk_in(qpll_refclk)
		);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Output PRBS generation on TX1 port

	wire lane1_rxclk;
	wire lane1_txclk;

	wire lane1_rxclk_raw;
	wire lane1_txclk_raw;

	BUFH bufh_lane1_rx(.I(lane1_rxclk_raw), .O(lane1_rxclk));
	BUFH bufh_lane1_tx(.I(lane1_txclk_raw), .O(lane1_txclk));

	wire[31:0]	lane1_rx_data;
	wire		lane1_prbs_err;

	gtx_frontlane1 lane1_transceiver(
		.sysclk_in(clk_125mhz),

		//TODO: do we need any of this
		.soft_reset_tx_in(1'b0),
		.soft_reset_rx_in(1'b0),
		.dont_reset_on_data_error_in(1'b0),
		.gt0_tx_fsm_reset_done_out(),
		.gt0_rx_fsm_reset_done_out(),

		.gt0_tx_mmcm_lock_in(pll_rgmii_lock),
		.gt0_rx_mmcm_lock_in(pll_rgmii_lock),

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
		.gt0_txusrclk_in(lane1_txclk),
		.gt0_txusrclk2_in(lane1_txclk),
		.gt0_data_valid_in(1'b1),
		.gt0_txdata_in(32'h00000000),
		.gt0_txoutclk_out(lane1_txclk_raw),
		.gt0_txoutclkfabric_out(),
		.gt0_txoutclkpcs_out(),
		.gt0_txresetdone_out(),

		//Fabric RX interface
		.gt0_rxusrclk_in(lane1_rxclk),
		.gt0_rxusrclk2_in(lane1_rxclk),
		.gt0_rxdata_out(lane1_rx_data),
		.gt0_rxoutclk_out(lane1_rxclk_raw),
		.gt0_rxoutclkfabric_out(),

		//Output pattern selection
		.gt0_txprbssel_in(3'b010),	//PRBS-15

		//Input PRBS detector
		.gt0_rxprbssel_in(3'b010),	//PRBS-15
		.gt0_rxprbserr_out(lane1_prbs_err),

		//Top level diff pairs
		.gt0_gtxtxn_out(tx1_p),
		.gt0_gtxtxp_out(tx1_n),
		.gt0_gtxrxn_in(rx1_p),
		.gt0_gtxrxp_in(rx1_n),

		//Output swing control and equalizer taps
		.gt0_txdiffctrl_in(/*4'b0100*/tx_swing),	//04 works well
		.gt0_txprecursor_in(tx_precursor),			//00 works well
		.gt0_txpostcursor_in(tx_postcursor),		//03 works well
		.gt0_txmaincursor_in(tx_maincursor),		//00 works well

		//Clock to/from CPLL
		/*.gt0_cpllfbclklost_out(),
		.gt0_cplllock_out(cpll_lock[1]),
		.gt0_cplllockdetclk_in(clk_125mhz),
		.gt0_cpllreset_in(1'b0),
		.gt0_gtrefclk0_in(serdes_refclk_156m25),
		.gt0_gtrefclk1_in(serdes_refclk_200m),*/

		//Clock from QPLL
		.gt0_qplllock_in(qpll_lock),
		.gt0_qpllrefclklost_in(qpll_refclk_lost),
		.gt0_qpllreset_out(),
		.gt0_qplloutclk_in(qpll_clkout_10g3125),
		.gt0_qplloutrefclk_in(qpll_refclk)
		);

	assign cpll_lock[1] = 0;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debug ILAs

	ila_1 ila(
		.clk(lane1_rxclk),
		.probe0(lane1_rx_data),
		.probe1(lane1_prbs_err)
	);

endmodule
