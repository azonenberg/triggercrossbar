`timescale 1ns/1ps
`default_nettype none
/***********************************************************************************************************************
*                                                                                                                      *
* trigger-crossbar                                                                                                     *
*                                                                                                                      *
* Copyright (c) 2023-2025 Andrew D. Zonenberg and contributors                                                         *
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
import CrossbarTypes::*;
import EthernetBus::*;

/**
	@file
	@author Andrew D. Zonenberg
	@brief Container for management logic
 */
module ManagementSubsystem(
	input wire						sys_clk,
	input wire						clk_sysinfo,

	input wire						qspi_sck,
	input wire						qspi_cs_n,
	inout wire[3:0]					qspi_dq,
	output logic					irq,

	//Management network buses
	input wire						xg0_tx_clk,
	AXIStream.receiver				xg0_axi_rx,
	AXIStream.transmitter			xg0_axi_tx,
	input wire						xg0_link_up,

	input wire						mgmt0_tx_clk,
	AXIStream.receiver				mgmt0_axi_rx,
	AXIStream.transmitter			mgmt0_axi_tx,
	input wire						mgmt0_link_up,

	//APB to external components
	APB.requester					mdioBus,
	APB.requester					relayBus,
	APB.requester					crossbarBus,
	APB.requester					bertBus,
	APB.requester					cryptBus,
	APB.requester					flashBus,

	//Tachometers for fans
	input wire[1:0]					fan_tach,

	//SPI interface to front panel
	output wire						frontpanel_sck,
	output wire						frontpanel_mosi,
	input wire						frontpanel_miso,
	output wire						frontpanel_cs_n,

	//Configuration registers in core clock domain
	input wire[11:0]				trig_in_led,
	input wire[11:0]				trig_out_led,
	input wire[3:0]					relay_state
);

	localparam SMOL_ADDR_WIDTH 		= 10;
	localparam BIG_ADDR_WIDTH		= 12;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Tachometer

	wire[15:0] fan0_rpm;
	wire[15:0] fan1_rpm;

	Tachometer #(
		.REFCLK_HZ(250000000)
	) tach0 (
		.clk(sys_clk),
		.tach(fan_tach[0]),
		.rpm(fan0_rpm));

	Tachometer #(
		.REFCLK_HZ(250000000)
	) tach1 (
		.clk(sys_clk),
		.tach(fan_tach[1]),
		.rpm(fan1_rpm));

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// QSPI device bridge to internal legacy bus plus APB

	//The top level APB bus from the QSPI bridge to everything else
	APB #(.DATA_WIDTH(32), .ADDR_WIDTH(24), .USER_WIDTH(0)) processorBus();

	//Prevent any logic from the rest of this module from being optimized into the bridge
	(* keep_hierarchy = "yes" *)
	ManagementBridge bridge(
		.clk(sys_clk),

		.qspi_sck(qspi_sck),
		.qspi_cs_n(qspi_cs_n),
		.qspi_dq(qspi_dq),

		.apb(processorBus)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Optional pipeline register on APB before the bridge

	APB #(.DATA_WIDTH(32), .ADDR_WIDTH(24), .USER_WIDTH(0)) bridgeUpstreamBus();
	APBRegisterSlice #(.UP_REG(0), .DOWN_REG(0))
		apb_regslice_root( .upstream(processorBus), .downstream(bridgeUpstreamBus) );

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Top level APB interconnect bridge

	//Allocate 0x8000 bytes of address space to each half of the root
	APB #(.DATA_WIDTH(32), .ADDR_WIDTH(15), .USER_WIDTH(0)) rootDownstreamBus[1:0]();

	APBBridge #(
		.BASE_ADDR(24'h00_0000),
		.BLOCK_SIZE(24'h00_8000),
		.NUM_PORTS(2)
	) apb_bridge_root (
		.upstream(bridgeUpstreamBus),
		.downstream(rootDownstreamBus)
	);

	APB #(.DATA_WIDTH(32), .ADDR_WIDTH(15), .USER_WIDTH(0)) rootDownstreamBusReg[1:0]();

	APBRegisterSlice #(.UP_REG(1), .DOWN_REG(1))
		apb_regslice_root2smol( .upstream(rootDownstreamBus[0]), .downstream(rootDownstreamBusReg[0]) );

	APBRegisterSlice #(.UP_REG(0), .DOWN_REG(0))
		apb_regslice_root2big( .upstream(rootDownstreamBus[1]), .downstream(rootDownstreamBusReg[1]) );

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Second level bridge for devices with smaller amounts of address space (starts at 0x00_0000)

	localparam NUM_SMOL_DEVS		= 10;

	APB #(.DATA_WIDTH(32), .ADDR_WIDTH(SMOL_ADDR_WIDTH), .USER_WIDTH(0)) smolDownstreamBus[NUM_SMOL_DEVS-1:0]();

	APBBridge #(
		.BASE_ADDR(24'h000_0000),
		.BLOCK_SIZE(32'h400),
		.NUM_PORTS(NUM_SMOL_DEVS)
	) apb_bridge_smol (
		.upstream(rootDownstreamBusReg[0]),
		.downstream(smolDownstreamBus)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Second level bridge for devices with larger amounts of address space (starts at 0x00_8000)

	localparam NUM_BIG_DEVS			= 5;

	APB #(.DATA_WIDTH(32), .ADDR_WIDTH(BIG_ADDR_WIDTH), .USER_WIDTH(0)) bigDownstreamBus[NUM_BIG_DEVS-1:0]();

	APBBridge #(
		.BASE_ADDR(24'h000_0000),
		.BLOCK_SIZE(32'h1000),
		.NUM_PORTS(NUM_BIG_DEVS)
	) apb_bridge_big (
		.upstream(rootDownstreamBusReg[1]),
		.downstream(bigDownstreamBus)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// System health + information (0x00_0000)

	APB #(.DATA_WIDTH(32), .ADDR_WIDTH(SMOL_ADDR_WIDTH), .USER_WIDTH(0)) sysinfoBus();

	APBRegisterSlice #(.UP_REG(1), .DOWN_REG(0))
		apb_regslice_sysinfo( .upstream(smolDownstreamBus[0]), .downstream(sysinfoBus) );

	APB_SystemInfo sysinfo(
		.apb(sysinfoBus),

		.clk_sysinfo(clk_sysinfo),

		.fan0_rpm(fan0_rpm),
		.fan1_rpm(fan1_rpm)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//Front panel LED indicator state

	//Virtual GPIO bank 0 (input LEDs, 0x00_0400)
	APB #(.DATA_WIDTH(32), .ADDR_WIDTH(SMOL_ADDR_WIDTH), .USER_WIDTH(0)) ledBus0();
	APBRegisterSlice #(.UP_REG(1), .DOWN_REG(0))
		apb_regslice_gpio_0( .upstream(smolDownstreamBus[1]), .downstream(ledBus0) );

	logic[11:0]	 trig_in_led_flipped;
	APB_GPIO #(
		.OUT_INIT(0),
		.TRIS_INIT(16'h0fff)
	) gpio_led0 (
		.apb(ledBus0),

		.gpio_out(),
		.gpio_tris(),
		.gpio_in({4'h0, trig_in_led_flipped})
	);

	//flip MSB to LSB to match expander pinout
	always_comb begin
		for(integer i=0; i<12; i=i+1)
			trig_in_led_flipped[i]	= trig_in_led[11-i];
	end

	//Virtual GPIO bank 1 (output LEDs, 0x000_800)
	APB #(.DATA_WIDTH(32), .ADDR_WIDTH(SMOL_ADDR_WIDTH), .USER_WIDTH(0)) ledBus1();
	APBRegisterSlice #(.UP_REG(1), .DOWN_REG(0))
		apb_regslice_gpio_1( .upstream(smolDownstreamBus[2]), .downstream(ledBus1) );

	logic[11:0]	trig_out_led_gated;
	APB_GPIO #(
		.OUT_INIT(0),
		.TRIS_INIT(16'h0fff)
	) gpio_led1 (
		.apb(ledBus1),

		.gpio_out(),
		.gpio_tris(),
		.gpio_in({4'h0, trig_out_led_gated})
	);

	always_comb begin
		trig_out_led_gated[7:0]		= trig_out_led[7:0];
		trig_out_led_gated[11:8]	= trig_out_led[11:8] & ~relay_state;
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Front panel SPI interface

	APB #(.DATA_WIDTH(32), .ADDR_WIDTH(SMOL_ADDR_WIDTH), .USER_WIDTH(0)) frontSpiBus();
	APB_SPIHostInterface iface(
		.apb(frontSpiBus),

		.spi_sck(frontpanel_sck),
		.spi_mosi(frontpanel_mosi),
		.spi_miso(frontpanel_miso),
		.spi_cs_n(frontpanel_cs_n)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//Interrupt status register (0x00_2000)

	APB #(.DATA_WIDTH(32), .ADDR_WIDTH(SMOL_ADDR_WIDTH), .USER_WIDTH(0)) irqStatusBus();

	APBRegisterSlice #(.UP_REG(1), .DOWN_REG(0))
		apb_regslice_irq_status( .upstream(smolDownstreamBus[8]), .downstream(irqStatusBus) );

	wire	mgmt0_rx_frame_ready;
	wire	xg0_rx_frame_ready;
	APB_StatusRegister irqstat (
		.apb(irqStatusBus),
		.status({30'h0, mgmt0_rx_frame_ready, xg0_rx_frame_ready})
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Pipeline registers for external APB endpoints with small address space chunks

	//MDIO (0x00_0c00)
	APBRegisterSlice #(.UP_REG(1), .DOWN_REG(0))
		apb_regslice_mdio( .upstream(smolDownstreamBus[3]), .downstream(mdioBus) );

	//Relay controller (0x00_1000)
	APBRegisterSlice #(.UP_REG(1), .DOWN_REG(0))
		apb_regslice_relay( .upstream(smolDownstreamBus[4]), .downstream(relayBus) );

	//SPI controller (0x00_1400)
	APBRegisterSlice #(.UP_REG(1), .DOWN_REG(0))
		apb_regslice_frontspi( .upstream(smolDownstreamBus[5]), .downstream(frontSpiBus) );

	//Crossbar mux selectors (0x00_1800)
	APBRegisterSlice #(.UP_REG(1), .DOWN_REG(0))
		apb_regslice_crossbar( .upstream(smolDownstreamBus[6]), .downstream(crossbarBus) );

	//Curve25519 accelerator (0x00_1c00)
	APBRegisterSlice #(.UP_REG(1), .DOWN_REG(0))
		apb_regslice_crypt( .upstream(smolDownstreamBus[7]), .downstream(cryptBus) );

	//FPGA boot flash controller (0x00_2400)
	APBRegisterSlice #(.UP_REG(1), .DOWN_REG(0))
		apb_regslice_flash( .upstream(smolDownstreamBus[9]), .downstream(flashBus) );

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Pipeline registers for external APB endpoints with large address space chunks

	//RGMII RX FIFO (0x00_8000)
	APBRegisterSlice #(.UP_REG(1), .DOWN_REG(1))
		apb_regslice_1g_rx( .upstream(bigDownstreamBus[0]), .downstream(mgmt0_apb_rx) );

	//SFP+ RX FIFO (0x00_9000)
	APBRegisterSlice #(.UP_REG(1), .DOWN_REG(1))
		apb_regslice_xg_rx( .upstream(bigDownstreamBus[1]), .downstream(xg0_apb_rx) );

	//RGMII TX FIFO (0x00_a000)
	APBRegisterSlice #(.UP_REG(1), .DOWN_REG(1))
		apb_regslice_1g_tx( .upstream(bigDownstreamBus[2]), .downstream(mgmt0_apb_tx) );

	//SFP+ RX FIFO (0x00_b000)
	APBRegisterSlice #(.UP_REG(1), .DOWN_REG(1))
		apb_regslice_xg_tx( .upstream(bigDownstreamBus[3]), .downstream(xg0_apb_tx) );

	//BERT configuration (00_c000)
	APBRegisterSlice #(.UP_REG(1), .DOWN_REG(1))
		apb_regslice_bert_config( .upstream(bigDownstreamBus[4]), .downstream(bertBus) );

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// FIFO for storing inbound/outbound Ethernet frames

	APB #(.DATA_WIDTH(32), .ADDR_WIDTH(BIG_ADDR_WIDTH), .USER_WIDTH(0)) mgmt0_apb_rx();
	APB_AXIS_EthernetRxBuffer mgmt0_rx_fifo(
		.apb(mgmt0_apb_rx),
		.axi_rx(mgmt0_axi_rx),
		.eth_link_up(mgmt0_link_up),
		.rx_frame_ready(mgmt0_rx_frame_ready)
	);

	APB #(.DATA_WIDTH(32), .ADDR_WIDTH(BIG_ADDR_WIDTH), .USER_WIDTH(0)) xg0_apb_rx();
	APB_AXIS_EthernetRxBuffer xg0_rx_fifo(
		.apb(xg0_apb_rx),
		.axi_rx(xg0_axi_rx),
		.eth_link_up(xg0_link_up),
		.rx_frame_ready(xg0_rx_frame_ready)
	);

	APB #(.DATA_WIDTH(32), .ADDR_WIDTH(BIG_ADDR_WIDTH), .USER_WIDTH(0)) mgmt0_apb_tx();
	APB_AXIS_EthernetTxBuffer mgmt0_tx_fifo(
		.apb(mgmt0_apb_tx),

		.tx_clk(mgmt0_tx_clk),
		.link_up_pclk(mgmt0_link_up),
		.axi_tx(mgmt0_axi_tx)
	);

	APB #(.DATA_WIDTH(32), .ADDR_WIDTH(BIG_ADDR_WIDTH), .USER_WIDTH(0)) xg0_apb_tx();
	APB_AXIS_EthernetTxBuffer xg0_tx_fifo(
		.apb(xg0_apb_tx),

		.tx_clk(xg0_tx_clk),
		.link_up_pclk(xg0_link_up),
		.axi_tx(xg0_axi_tx)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Interrupt pin generation

	always_comb begin
		irq	= mgmt0_rx_frame_ready || xg0_rx_frame_ready;	//TODO other sources OR'd together
	end

endmodule
