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

`include "CrossbarTypes.svh"
`include "BERTConfig.svh"

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
	output wire						irq,

	//Management network bus
	input wire						eth_rx_clk,
	input wire						mgmt0_tx_clk,

	input wire EthernetRxBus		eth_rx_bus,
	output EthernetTxBus			mgmt0_tx_bus,
	input wire						mgmt0_tx_ready,
	input wire						eth_link_up,

	inout wire						mgmt0_mdio,
	output wire						mgmt0_mdc,

	input wire						xg0_rx_clk,
	input wire						xg0_link_up,
	input wire						xg0_tx_clk,
	output EthernetTxBus			xg0_tx_bus,

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
	output wire						relay_en,
	output wire						relay_dir,
	output wire[1:0]				relay_channel,
	input wire						relay_done,
	output muxsel_t[11:0]			muxsel,
	output wire						serdes_config_updated,
	output bert_txconfig_t			tx0_config,
	output bert_txconfig_t			tx1_config,
	output bert_rxconfig_t			rx0_config,
	output bert_rxconfig_t			rx1_config,
	output wire						mgmt_lane0_en,
	output wire						mgmt_lane1_en,
	output wire						mgmt_we,
	output wire[8:0]				mgmt_addr,
	output wire[15:0]				mgmt_wdata,
	input wire[15:0]				mgmt_lane0_rdata,
	input wire[15:0]				mgmt_lane1_rdata,
	input wire						mgmt_lane0_done,
	input wire						mgmt_lane1_done,
	input wire						mgmt_lane0_rx_rstdone,
	input wire						mgmt_lane1_rx_rstdone,

	//Configuration registers in crypto clock domain
	input wire						clk_crypt,
	output wire						crypt_en,
	output wire[255:0]				crypt_work_in,
	output wire[255:0]				crypt_e,
	input wire						crypt_out_valid,
	input wire[255:0]				crypt_work_out,
	output wire						crypt_dsa_en,
	output wire						crypt_dsa_base_en,
	output wire						crypt_dsa_load,
	output wire						crypt_dsa_rd,
	input wire						crypt_dsa_done,
	output wire[1:0]				crypt_dsa_addr
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// MDIO transceivers

	wire		mgmt0_mdio_tx_data;
	wire		mgmt0_mdio_tx_en;
	wire		mgmt0_mdio_rx_data;

	BidirectionalBuffer mgmt0_mdio_obuf(
		.fabric_in(mgmt0_mdio_rx_data),
		.fabric_out(mgmt0_mdio_tx_data),
		.pad(mgmt0_mdio),
		.oe(mgmt0_mdio_tx_en)
	);

	wire		mgmt0_mdio_busy;
	wire[4:0]	mgmt0_phy_reg_addr;
	wire[15:0]	mgmt0_phy_wr_data;
	wire[15:0]	mgmt0_phy_rd_data;
	wire		mgmt0_phy_reg_wr;
	wire		mgmt0_phy_reg_rd;
	wire[4:0]	mgmt0_phy_md_addr;

	//Prevent any logic from the rest of this module from being optimized into the bridge
	(* keep_hierarchy = "yes" *)
	EthernetMDIOTransceiver #(
		.CLK_DIV(75)
	)  mgmt0_mdio_txvr (
		.clk(sys_clk),
		.phy_md_addr(mgmt0_phy_md_addr),

		.mdio_tx_data(mgmt0_mdio_tx_data),
		.mdio_tx_en(mgmt0_mdio_tx_en),
		.mdio_rx_data(mgmt0_mdio_rx_data),
		.mdc(mgmt0_mdc),

		.mgmt_busy_fwd(mgmt0_mdio_busy),
		.phy_reg_addr(mgmt0_phy_reg_addr),
		.phy_wr_data(mgmt0_phy_wr_data),
		.phy_rd_data(mgmt0_phy_rd_data),
		.phy_reg_wr(mgmt0_phy_reg_wr),
		.phy_reg_rd(mgmt0_phy_reg_rd)
	);

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
	// FIFO for storing inbound/outbound Ethernet frames

	wire		rxfifo_rd_en;
	wire		rxfifo_rd_pop_single;
	wire[31:0]	rxfifo_rd_data;
	wire		rxheader_rd_en;
	wire		rxheader_rd_empty;
	wire[10:0]	rxheader_rd_data;

	ManagementRxFifo rx_fifo(
		.sys_clk(sys_clk),

		.eth_rx_clk(eth_rx_clk),
		.eth_rx_bus(eth_rx_bus),
		.eth_link_up(eth_link_up),

		.rxfifo_rd_en(rxfifo_rd_en),
		.rxfifo_rd_pop_single(rxfifo_rd_pop_single),
		.rxfifo_rd_data(rxfifo_rd_data),
		.rxheader_rd_en(rxheader_rd_en),
		.rxheader_rd_empty(rxheader_rd_empty),
		.rxheader_rd_data(rxheader_rd_data)
	);

	wire		txfifo_wr_en;
	wire[7:0]	txfifo_wr_data;
	wire		txfifo_wr_commit;

	wire		eth_link_up_txclk;
	ThreeStageSynchronizer sync_link_up_txclk(
		.clk_in(eth_rx_clk),
		.din(eth_link_up),
		.clk_out(mgmt0_tx_clk),
		.dout(eth_link_up_txclk)
	);

	wire		xg0_link_up_txclk;
	ThreeStageSynchronizer sync_xg0_link_up_txclk(
		.clk_in(xg0_rx_clk),
		.din(xg0_link_up),
		.clk_out(xg0_tx_clk),
		.dout(xg0_link_up_txclk)
	);

	ManagementTxFifo tx_fifo(
		.sys_clk(sys_clk),

		.wr_en(txfifo_wr_en),
		.wr_data(txfifo_wr_data),
		.wr_commit(txfifo_wr_commit),

		.tx_clk(mgmt0_tx_clk),
		.link_up(eth_link_up_txclk),
		.tx_ready(mgmt0_tx_ready),
		.tx_bus(mgmt0_tx_bus)
	);

	wire		xg_txfifo_wr_en;
	wire[7:0]	xg_txfifo_wr_data;
	wire		xg_txfifo_wr_commit;

	Management10GTxFifo xg_tx_fifo(
		.sys_clk(sys_clk),

		.wr_en(xg_txfifo_wr_en),
		.wr_data(xg_txfifo_wr_data),
		.wr_commit(xg_txfifo_wr_commit),

		.tx_clk(xg0_tx_clk),
		.link_up(xg0_link_up_txclk),
		.tx_bus(xg0_tx_bus)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// QSPI device bridge

	wire		mgmt_rd_en;
	wire[15:0]	mgmt_rd_addr;
	wire		mgmt_rd_valid;
	wire[7:0]	mgmt_rd_data;

	wire		mgmt_wr_en;
	wire[15:0]	mgmt_wr_addr;
	wire[7:0]	mgmt_wr_data;

	//(* retiming_backward = 1 *)
	logic		mgmt_rd_valid_out	= 0;

	//(* retiming_backward = 1 *)
	logic[7:0]	mgmt_rd_data_out	= 0;

	//Prevent any logic from the rest of this module from being optimized into the bridge
	(* keep_hierarchy = "yes" *)
	ManagementBridge bridge(
		.clk(sys_clk),

		.qspi_sck(qspi_sck),
		.qspi_cs_n(qspi_cs_n),
		.qspi_dq(qspi_dq),

		.rd_en(mgmt_rd_en),
		.rd_addr(mgmt_rd_addr),
		.rd_valid(mgmt_rd_valid_out),
		.rd_data(mgmt_rd_data_out),

		.wr_en(mgmt_wr_en),
		.wr_addr(mgmt_wr_addr),
		.wr_data(mgmt_wr_data)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Optionally pipeline read data by one cycle

	always_comb begin
	//always_ff @(posedge sys_clk) begin
		mgmt_rd_valid_out	= mgmt_rd_valid;
		mgmt_rd_data_out	= mgmt_rd_data;
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Pipeline register on write data plus read address bus

	logic		mgmt_rd_en_ff	= 0;
	logic[15:0]	mgmt_rd_addr_ff	= 0;

	logic		mgmt_wr_en_ff	= 0;
	logic[15:0]	mgmt_wr_addr_ff	= 0;
	logic[7:0]	mgmt_wr_data_ff	= 0;

	always_ff @(posedge sys_clk) begin
		mgmt_wr_en_ff	<= mgmt_wr_en;
		mgmt_wr_addr_ff	<= mgmt_wr_addr;
		mgmt_wr_data_ff	<= mgmt_wr_data;

		mgmt_rd_en_ff	<= mgmt_rd_en;
		mgmt_rd_addr_ff	<= mgmt_rd_addr;
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Device information

	wire[63:0]	die_serial;
	wire		die_serial_valid;

	wire[31:0]	idcode;
	wire		idcode_valid;

	DeviceInfo_7series info(
		.clk(clk_sysinfo),

		.die_serial(die_serial),
		.die_serial_valid(die_serial_valid),
		.idcode(idcode),
		.idcode_valid(idcode_valid)
	);

	wire[15:0]	die_temp;
	wire[15:0]	volt_core;
	wire[15:0]	volt_ram;
	wire[15:0]	volt_aux;

	OnDieSensors_7series #(
		.EXT_IN_ENABLE(16'h0)
	) sensors (
		.clk(sys_clk),
		.vin_p(),
		.vin_n(),
		.die_temp(die_temp),
		.volt_core(volt_core),
		.volt_ram(volt_ram),
		.volt_aux(volt_aux),
		.sensors_update(),

		.ext_in(),
		.ext_update(),
		.die_temp_native()
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Front panel SPI interface

	wire		front_shift_en;
	wire[7:0]	front_shift_data;
	wire		front_shift_done;
	wire[7:0]	front_rx_data;

	SPIHostInterface spi(
		.clk(sys_clk),
		.clkdiv(100),		//2.5 MHz

		.spi_sck(frontpanel_sck),
		.spi_mosi(frontpanel_mosi),
		.spi_miso(frontpanel_miso),

		.shift_en(front_shift_en),
		.shift_done(front_shift_done),
		.tx_data(front_shift_data),
		.rx_data(front_rx_data));

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Register interface

	ManagementRegisterInterface regs (
		.clk(sys_clk),

		.irq(irq),

		//Memory bus
		.rd_en(mgmt_rd_en_ff),
		.rd_addr(mgmt_rd_addr_ff),
		.rd_valid(mgmt_rd_valid),
		.rd_data(mgmt_rd_data),

		.wr_en(mgmt_wr_en_ff),
		.wr_addr(mgmt_wr_addr_ff),
		.wr_data(mgmt_wr_data_ff),

		//Control registers (device info clock domain)
		.die_serial_valid(die_serial_valid),
		.die_serial(die_serial),
		.idcode_valid(idcode_valid),
		.idcode(idcode),

		//Control registers (core clock domain)
		.fan0_rpm(fan0_rpm),
		.fan1_rpm(fan1_rpm),
		.die_temp(die_temp),
		.volt_core(volt_core),
		.volt_ram(volt_ram),
		.volt_aux(volt_aux),
		.mgmt0_mdio_busy(mgmt0_mdio_busy),
		.mgmt0_phy_reg_addr(mgmt0_phy_reg_addr),
		.mgmt0_phy_wr_data(mgmt0_phy_wr_data),
		.mgmt0_phy_rd_data(mgmt0_phy_rd_data),
		.mgmt0_phy_reg_wr(mgmt0_phy_reg_wr),
		.mgmt0_phy_reg_rd(mgmt0_phy_reg_rd),
		.mgmt0_phy_md_addr(mgmt0_phy_md_addr),
		.rxfifo_rd_en(rxfifo_rd_en),
		.rxfifo_rd_pop_single(rxfifo_rd_pop_single),
		.rxfifo_rd_data(rxfifo_rd_data),
		.rxheader_rd_en(rxheader_rd_en),
		.rxheader_rd_empty(rxheader_rd_empty),
		.rxheader_rd_data(rxheader_rd_data),
		.txfifo_wr_en(txfifo_wr_en),
		.txfifo_wr_data(txfifo_wr_data),
		.txfifo_wr_commit(txfifo_wr_commit),
		.xg_txfifo_wr_en(xg_txfifo_wr_en),
		.xg_txfifo_wr_data(xg_txfifo_wr_data),
		.xg_txfifo_wr_commit(xg_txfifo_wr_commit),
		.relay_en(relay_en),
		.relay_dir(relay_dir),
		.relay_channel(relay_channel),
		.relay_done(relay_done),
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
		.front_shift_en(front_shift_en),
		.front_shift_done(front_shift_done),
		.front_shift_data(front_shift_data),
		.front_rx_data(front_rx_data),
		.front_cs_n(frontpanel_cs_n),
		.trig_in_led(trig_in_led),
		.trig_out_led(trig_out_led),

		//Control registers (port RX clock domain)
		.xg0_rx_clk(xg0_rx_clk),
		.xg0_link_up(xg0_link_up),

		//Control registers (crypto clock domain)
		.clk_crypt(clk_crypt),
		.crypt_en(crypt_en),
		.crypt_work_in(crypt_work_in),
		.crypt_e(crypt_e),
		.crypt_out_valid(crypt_out_valid),
		.crypt_work_out(crypt_work_out),
		.crypt_dsa_en(crypt_dsa_en),
		.crypt_dsa_base_en(crypt_dsa_base_en),
		.crypt_dsa_load(crypt_dsa_load),
		.crypt_dsa_rd(crypt_dsa_rd),
		.crypt_dsa_done(crypt_dsa_done),
		.crypt_dsa_addr(crypt_dsa_addr)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debug ILA

	ila_1 ila(
		.clk(sys_clk),
		.probe0(front_shift_en),
		.probe1(front_shift_done),
		.probe2(front_shift_data),
		.probe3(front_rx_data),
		.probe4(frontpanel_sck),
		.probe5(frontpanel_mosi),
		.probe6(frontpanel_miso),
		.probe7(frontpanel_cs_n),

		.probe8(mgmt_wr_en_ff),
		.probe9(mgmt_wr_addr_ff),
		.probe10(mgmt_wr_data_ff),

		.probe11(mgmt_rd_en_ff),
		.probe12(mgmt_rd_addr_ff)
		);

endmodule
