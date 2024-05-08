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

module NetworkRxMuxing(
	input wire					clk_250mhz,

	input wire					mgmt0_link_up,
	input wire					mgmt0_rx_clk_buf,
	input wire EthernetRxBus	mgmt0_rx_bus,

	input wire					xg0_link_up,
	input wire					xg0_mac_rx_clk,
	input wire EthernetRxBus	xg0_mac_rx_bus,

	output logic				eth_link_up = 0,
	output EthernetRxBus		eth_rx_bus
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Pipeline registers

	EthernetRxBus	xg0_mac_rx_bus_ff = 0;

	always_ff @(posedge xg0_mac_rx_clk) begin
		xg0_mac_rx_bus_ff	<= xg0_mac_rx_bus;
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The actual muxes

	EthernetRxBus	baset_cdc_rx_bus;
	EthernetRxBus	baser_cdc_rx_bus;

	//Synchronize link state into core clock domain
	wire			sfp_link_up_core;
	wire			rgmii_link_up_core;
	ThreeStageSynchronizer sync_sfp_link_up(
		.clk_in(xg0_mac_rx_clk), .din(xg0_link_up), .clk_out(clk_250mhz), .dout(sfp_link_up_core));
	ThreeStageSynchronizer sync_rgmii_link_up(
		.clk_in(mgmt0_rx_clk_buf), .din(mgmt0_link_up), .clk_out(clk_250mhz), .dout(rgmii_link_up_core));

	EthernetRxClockCrossing baset_rx_cdc(
		.gmii_rxc(mgmt0_rx_clk_buf),
		.mac_rx_bus(mgmt0_rx_bus),

		.sys_clk(clk_250mhz),
		.cdc_rx_bus(baset_cdc_rx_bus),

		.perf_rx_cdc_frames()
	);

	EthernetRxClockCrossing baser_rx_cdc(
		.gmii_rxc(xg0_mac_rx_clk),
		.mac_rx_bus(xg0_mac_rx_bus_ff),

		.sys_clk(clk_250mhz),
		.cdc_rx_bus(baser_cdc_rx_bus),

		.perf_rx_cdc_frames()
	);

	always_ff @(posedge clk_250mhz) begin

		eth_link_up	<= sfp_link_up_core || rgmii_link_up_core;

		//If SFP is up, use that bus
		if(sfp_link_up_core)
			eth_rx_bus	<= baser_cdc_rx_bus;

		//otherwise use baseT if that's up
		else if(rgmii_link_up_core)
			eth_rx_bus	<= baset_cdc_rx_bus;

		//neither
		else
			eth_rx_bus	<= 0;

	end

endmodule
