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

`include "EthernetBus.svh"
`include "GmiiBus.svh"
`include "CrossbarTypes.svh"
`include "BERTConfig.svh"

/**
	@file
	@author Andrew D. Zonenberg
	@brief Container for management logic

	Management registers have 16-bit addresses and are 8 bits in size.
 */
module ManagementRegisterInterface(

	//Core clock for the management domain
	input wire						clk,

	output logic					irq			= 0,

	//Data bus from QSPI interface or simulation bridge
	input wire						rd_en,
	input wire[15:0]				rd_addr,
	output logic					rd_valid	= 0,
	output logic[7:0]				rd_data		= 0,

	input wire						wr_en,
	input wire[15:0]				wr_addr,
	input wire[7:0]					wr_data,

	//Device information bus
	//Must be divided down from core clock, but phase aligned
	input wire						die_serial_valid,
	input wire[63:0]				die_serial,
	input wire						idcode_valid,
	input wire[31:0]				idcode,

	//Configuration registers in port RX clock domains
	input wire						xg0_rx_clk,
	input wire						xg0_link_up,

	//Configuration registers in core clock domain
	input wire[15:0]				fan0_rpm,
	input wire[15:0]				fan1_rpm,
	input wire[15:0]				die_temp,
	input wire[15:0]				volt_core,
	input wire[15:0]				volt_ram,
	input wire[15:0]				volt_aux,
	input wire						mgmt0_mdio_busy,
	output logic[4:0]				mgmt0_phy_reg_addr = 0,
	output logic[15:0]				mgmt0_phy_wr_data = 0,
	input wire[15:0]				mgmt0_phy_rd_data,
	output logic					mgmt0_phy_reg_wr = 0,
	output logic					mgmt0_phy_reg_rd = 0,
	output logic[4:0]				mgmt0_phy_md_addr = 0,

	output logic					relay_en = 0,
	output logic					relay_dir = 0,
	output logic[1:0]				relay_channel = 0,
	input wire						relay_done,

	output muxsel_t[11:0]			muxsel = 0,

	output logic					rxfifo_rd_en = 0,
	output logic					rxfifo_rd_pop_single = 0,
	input wire[31:0]				rxfifo_rd_data,
	output logic					rxheader_rd_en = 0,
	input wire						rxheader_rd_empty,
	input wire[10:0]				rxheader_rd_data,
	output logic 					txfifo_wr_en = 0,
	output logic[7:0] 				txfifo_wr_data = 0,
	output logic	 				txfifo_wr_commit = 0,

	//still in core clock domain, synchronizer in serdes module
	output logic					serdes_config_updated = 0,
	output bert_txconfig_t			tx0_config = 0,
	output bert_txconfig_t			tx1_config = 0,
	output bert_rxconfig_t			rx0_config = 0,
	output bert_rxconfig_t			rx1_config = 0,
	output logic					mgmt_lane0_en = 0,
	output logic					mgmt_lane1_en = 0,
	output logic					mgmt_we = 0,
	output logic[8:0]				mgmt_addr = 0,
	output logic[15:0]				mgmt_wdata = 0,
	input wire[15:0]				mgmt_lane0_rdata,
	input wire[15:0]				mgmt_lane1_rdata,
	input wire						mgmt_lane0_done,
	input wire						mgmt_lane1_done,
	input wire						mgmt_lane0_rx_rstdone,
	input wire						mgmt_lane1_rx_rstdone,

	//Configuration registers in crypto clock domain
	input wire						clk_crypt,
	output logic					crypt_en = 0,
	output wire[255:0]				crypt_work_in,
	output wire[255:0]				crypt_e,
	input wire						crypt_out_valid,
	input wire[255:0]				crypt_work_out
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Pipeline registers for some external flags

	logic	mgmt0_mdio_busy_ff = 0;
	always_ff @(posedge clk) begin
		mgmt0_mdio_busy_ff	<= mgmt0_mdio_busy;
	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Synchronizers for crypto stuff

	logic			crypt_in_updated	= 0;
	logic[255:0]	crypt_work_in_mgmt	= 0;
	logic[255:0]	crypt_e_mgmt		= 0;

	//Ignore toggles on updated_b for first few clocks after reset
	//seems sync glitches, at least in sim?
	wire			crypt_en_sync;
	logic[3:0]		rst_count 			= 1;
	always_ff @(posedge clk_crypt) begin
		if(rst_count)
			rst_count	<= rst_count + 1;
		else
			crypt_en	<= crypt_en_sync;
	end

	RegisterSynchronizer #(
		.WIDTH(512)
	) sync_crypt_inputs (
		.clk_a(clk),
		.en_a(crypt_in_updated),
		.ack_a(),
		.reg_a({crypt_work_in_mgmt, crypt_e_mgmt}),

		.clk_b(clk_crypt),
		.updated_b(crypt_en_sync),
		.reset_b(1'b0),
		.reg_b({crypt_work_in, crypt_e})
	);

	wire			crypt_out_updated;
	wire[255:0]		crypt_work_out_mgmt;

	RegisterSynchronizer #(
		.WIDTH(256)
	) sync_crypt_outputs (
		.clk_a(clk_crypt),
		.en_a(crypt_out_valid),
		.ack_a(),
		.reg_a(crypt_work_out),

		.clk_b(clk),
		.updated_b(crypt_out_updated),
		.reset_b(1'b0),
		.reg_b(crypt_work_out_mgmt)
	);

	wire	xg0_link_up_sync;

	ThreeStageSynchronizer sync_xg0_link_up(
		.clk_in(xg0_rx_clk),
		.din(xg0_link_up),
		.clk_out(clk),
		.dout(xg0_link_up_sync)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// List of named registers

	//Note that ManagementBridge uses MSB of address as read/write flag
	//so we actually have only 15 bits available for addressing

	//must match ManagementRegisterInterface in FPGAInterface.h
	typedef enum logic[15:0]
	{
		//FPGA die information
		REG_FPGA_IDCODE		= 16'h0000,		//4 bytes of IDCODE
		REG_FPGA_IDCODE_1	= 16'h0001,
		REG_FPGA_IDCODE_2	= 16'h0002,
		REG_FPGA_IDCODE_3	= 16'h0003,
		REG_FPGA_SERIAL		= 16'h0004,		//8 bytes of die serial
		REG_FPGA_SERIAL_1	= 16'h0005,
		REG_FPGA_SERIAL_2	= 16'h0006,
		REG_FPGA_SERIAL_3	= 16'h0007,
		REG_FPGA_SERIAL_4	= 16'h0008,
		REG_FPGA_SERIAL_5	= 16'h0009,
		REG_FPGA_SERIAL_6	= 16'h000a,
		REG_FPGA_SERIAL_7	= 16'h000b,

		//Sensors
		REG_FAN0_RPM		= 16'h0010,
		REG_FAN0_RPM_1		= 16'h0011,
		REG_FAN1_RPM		= 16'h0012,
		REG_FAN1_RPM_1		= 16'h0013,
		REG_DIE_TEMP		= 16'h0014,
		REG_DIE_TEMP_1		= 16'h0015,
		REG_VOLT_CORE		= 16'h0016,
		REG_VOLT_CORE_1		= 16'h0017,
		REG_VOLT_RAM		= 16'h0018,
		REG_VOLT_RAM_1		= 16'h0019,
		REG_VOLT_AUX		= 16'h001a,
		REG_VOLT_AUX_1		= 16'h001b,

		//Reasons for an IRQ
		REG_FPGA_IRQSTAT	= 16'h0020,		//
		REG_FPGA_IRQSTAT_1	= 16'h0021,		//
											// 0 = RX Ethernet frame ready

		//Ethernet MAC
		REG_EMAC_RXLEN		= 16'h0024,
		REG_EMAC_RXLEN_1	= 16'h0025,
		REG_EMAC_COMMIT		= 16'h0028,		//write any value to end the active packet

		//MDIO controllers
		REG_MGMT0_MDIO		= 16'h0048,		//31    = busy flag (R)
		REG_MGMT0_MDIO_1	= 16'h0049,		//30    = write enable (W)
		REG_MGMT0_MDIO_2	= 16'h004a,		//29    = read enable (W)
		REG_MGMT0_MDIO_3	= 16'h004b,		//25:21 = phy addr (W)
											//20:16 = register addr (W)
											//15:0	= register data (RW)

		//10G interface
		REG_XG0_STAT		= 16'h0060,		//0 = link up

		//Relay controller
		REG_RELAY_TOGGLE	= 16'h0070,		//15	= direction (1 = in, 0 = out)
		REG_RELAY_TOGGLE_1	= 16'h0071,		//1:0	= channel number
											//Writes are self timed

		REG_RELAY_STAT		= 16'h0072,		//0		= relay busy flag
		REG_RELAY_STAT_1	= 16'h0073,
											//If set, no new relay commands are allowed

		//BERT configuration
		REG_BERT_LANE0_PRBS	= 16'h0080,		//6:4  = TX PRBS mode (GTX TXPRBSSEL see table 3-22 of UG476)
											//2:0  = RX PRBS mode (GTX RXPRBSSEL see table 4-30 of UG476)

		REG_BERT_LANE0_TX	= 16'h0082,		//15   = invert
											//14   = tx enable
											//13:9 = postcursor
											//8:4  = precursor
											//3:0  = swing
		REG_BERT_LANE0_TX_1 = 16'h0083,
		REG_BERT_LANE0_WD	= 16'h0084,		//DRP write data (must write before address)
		REG_BERT_LANE0_WD_1	= 16'h0085,
		REG_BERT_LANE0_AD	= 16'h0086,		//DRP address
		REG_BERT_LANE0_AD_1 = 16'h0087,		//15 = write enable
											//14:9 = reserved
											//8:0 = address
		REG_BERT_LANE0_RD	= 16'h0088,
		REG_BERT_LANE0_RD_1	= 16'h0089,
		REG_BERT_LANE0_STAT	= 16'h008a,		//0 = DRP busy
											//1 = rx reset done
		REG_BERT_LANE0_CLK	= 16'h008c,		//2:0 = TX clock divider
											//3 = TX clock from QPLL (0 = CPLL)
		REG_BERT_LANE0_RST	= 16'h008e,		//0 = RX PMA reset
											//1 = TX soft reset

		REG_BERT_LANE0_RX	= 16'h0090,		//0 = invert

		REG_BERT_LANE1_PRBS = 16'h00a0,		//same as LANE0
		REG_BERT_LANE1_TX	= 16'h00a2,
		REG_BERT_LANE1_TX_1 = 16'h00a3,
		REG_BERT_LANE1_WD	= 16'h00a4,
		REG_BERT_LANE1_WD_1	= 16'h00a5,
		REG_BERT_LANE1_AD	= 16'h00a6,
		REG_BERT_LANE1_AD_1	= 16'h00a7,
		REG_BERT_LANE1_RD	= 16'h00a8,
		REG_BERT_LANE1_RD_1	= 16'h00a9,
		REG_BERT_LANE1_STAT	= 16'h00aa,
		REG_BERT_LANE1_CLK	= 16'h00ac,
		REG_BERT_LANE1_RST	= 16'h00ae,
		REG_BERT_LANE1_RX	= 16'h00b0,

		//Mux selectors
		REG_MUXSEL_BASE		= 16'h00f0,		//3:0 = mux selector
		//next 11 addresses are subsequent channels

		//Ethernet MAC frame buffer
		//Any address in this range will be treated as reading from the top of the buffer
		REG_EMAC_BUFFER_LO	= 16'h1000,
		REG_EMAC_BUFFER_HI	= 16'h1fff,

		//Crypto accelerator
		REG_CRYPT_BASE		= 16'h3800,

		//helper just so we can use commas to separate list items
		REG_LAST

	} regid_t;

	//Register offsets within crypto block
	typedef enum logic[15:0]
	{
		REG_WORK			= 16'h0000,
		REG_E				= 16'h0020,
		REG_CRYPT_STATUS	= 16'h0040,
		REG_WORK_OUT		= 16'h0060
	} cryptoff_t;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Address decoding and muxing logic

	logic 					reading					= 0;
	logic					crypto_active			= 0;

	logic					mgmt0_mdio_busy_latched = 0;
	logic					dp_mdio_busy_latched	= 0;
	logic					vsc_mdio_busy_latched	= 0;

	logic					relay_busy	= 0;

	logic[1:0]				drp_busy	= 0;

	always_ff @(posedge clk) begin

		//Clear single cycle flags
		rd_valid				<= 0;
		crypt_in_updated		<= 0;
		mgmt0_phy_reg_wr		<= 0;
		mgmt0_phy_reg_rd		<= 0;
		rxfifo_rd_en			<= 0;
		rxheader_rd_en			<= 0;
		rxfifo_rd_pop_single	<= 0;
		txfifo_wr_en			<= 0;
		txfifo_wr_commit		<= 0;
		relay_en				<= 0;
		serdes_config_updated	<= 0;
		mgmt_lane0_en			<= 0;
		mgmt_lane1_en			<= 0;

		//Start a new read
		if(rd_en)
			reading	<= 1;

		//Finish a crypto operation
		if(crypt_out_updated)
			crypto_active		<= 0;

		//Set interrupt line if something's changed
		if(!rxheader_rd_empty)
			irq					<= 1;

		//Manage relay states
		if(relay_done)
			relay_busy			<= 0;
		if(relay_en)
			relay_busy			<= 1;

		//Manage DRP states
		if(mgmt_lane0_en)
			drp_busy[0]			<= 1;
		if(mgmt_lane1_en)
			drp_busy[1]			<= 1;
		if(mgmt_lane0_done)
			drp_busy[0]			<= 0;
		if(mgmt_lane1_done)
			drp_busy[1]			<= 0;

		//Continue a read
		if(rd_en || reading) begin

			//Data not ready? Wait
			if( (rd_addr >= REG_FPGA_IDCODE) && (rd_addr <= REG_FPGA_IDCODE_3) && !idcode_valid) begin
			end
			else if( (rd_addr >= REG_FPGA_SERIAL) && (rd_addr <= REG_FPGA_SERIAL_7) && !die_serial_valid) begin
			end

			//Data is ready
			else begin

				rd_valid	<= 1;
				reading		<= 0;

			end

			//Crypto registers are decoded separately
			if(rd_addr >= REG_CRYPT_BASE) begin

				if(rd_addr[7:0] == REG_CRYPT_STATUS)
					rd_data	<= {7'b0, crypto_active};
				else if(rd_addr[7:0] >= REG_WORK_OUT)
					rd_data <= crypt_work_out_mgmt[rd_addr[4:0]*8 +: 8];

				//unmapped address
				else
					rd_data	<= 0;

			end

			//Ethernet MAC
			//Read data without any endianness swapping, since it's logically an array of bytes
			else if(rd_addr >= REG_EMAC_BUFFER_LO) begin

				case(rd_addr[1:0])
					0: begin

						//pop the buffer since we've got the read data in the working register
						rxfifo_rd_pop_single	<= 1;

						rd_data					<= rxfifo_rd_data[31:24];
					end
					1:	rd_data					<= rxfifo_rd_data[23:16];
					2:	rd_data					<= rxfifo_rd_data[15:8];
					3: begin
						//prepare to read the next
						rxfifo_rd_en			<= 1;

						rd_data					<= rxfifo_rd_data[7:0];
					end
				endcase

			end

			//Main register decoder
			else begin

				case(rd_addr)

					REG_FPGA_IDCODE:	rd_data <= idcode[3*8 +: 8];
					REG_FPGA_IDCODE_1:	rd_data <= idcode[2*8 +: 8];
					REG_FPGA_IDCODE_2:	rd_data <= idcode[1*8 +: 8];
					REG_FPGA_IDCODE_3:	rd_data <= idcode[0*8 +: 8];

					REG_FPGA_SERIAL:	rd_data <= die_serial[7*8 +: 8];
					REG_FPGA_SERIAL_1:	rd_data <= die_serial[6*8 +: 8];
					REG_FPGA_SERIAL_2:	rd_data <= die_serial[5*8 +: 8];
					REG_FPGA_SERIAL_3:	rd_data <= die_serial[4*8 +: 8];
					REG_FPGA_SERIAL_4:	rd_data <= die_serial[3*8 +: 8];
					REG_FPGA_SERIAL_5:	rd_data <= die_serial[2*8 +: 8];
					REG_FPGA_SERIAL_6:	rd_data <= die_serial[1*8 +: 8];
					REG_FPGA_SERIAL_7:	rd_data <= die_serial[0*8 +: 8];

					REG_FAN0_RPM:		rd_data	<= fan0_rpm[7:0];
					REG_FAN0_RPM_1:		rd_data	<= fan0_rpm[15:8];
					REG_FAN1_RPM:		rd_data	<= fan1_rpm[7:0];
					REG_FAN1_RPM_1:		rd_data	<= fan1_rpm[15:8];

					REG_DIE_TEMP:		rd_data	<= die_temp[7:0];
					REG_DIE_TEMP_1:		rd_data	<= die_temp[15:8];
					REG_VOLT_CORE:		rd_data	<= volt_core[7:0];
					REG_VOLT_CORE_1:	rd_data	<= volt_core[15:8];
					REG_VOLT_RAM:		rd_data	<= volt_ram[7:0];
					REG_VOLT_RAM_1:		rd_data	<= volt_ram[15:8];
					REG_VOLT_AUX:		rd_data	<= volt_aux[7:0];
					REG_VOLT_AUX_1:		rd_data	<= volt_aux[15:8];

					REG_FPGA_IRQSTAT: begin
						rd_data		<= {7'b0, !rxheader_rd_empty };
						irq			<= 0;
					end
					REG_FPGA_IRQSTAT_1: rd_data <= 8'b0;

					REG_EMAC_RXLEN:		rd_data <= rxheader_rd_data[7:0];
					REG_EMAC_RXLEN_1: begin
						rd_data 		<= {5'b0, rxheader_rd_data[10:8]};
						rxheader_rd_en	<= 1;

						//read (but don't pop) first data word
						//so it's ready by the time we need it
						rxfifo_rd_en	<= 1;
					end

					REG_MGMT0_MDIO: begin
						rd_data					<= mgmt0_phy_rd_data[7:0];
						mgmt0_mdio_busy_latched	<= mgmt0_mdio_busy_ff;
					end
					REG_MGMT0_MDIO_1:	rd_data	<= mgmt0_phy_rd_data[15:8];
					REG_MGMT0_MDIO_2:	rd_data	<= 0;
					REG_MGMT0_MDIO_3:	rd_data <= {mgmt0_mdio_busy_latched, 7'b0};

					REG_XG0_STAT:		rd_data <= {7'b0, xg0_link_up_sync };

					REG_RELAY_STAT:		rd_data	<= 8'h0;
					REG_RELAY_STAT_1:	rd_data	<= {7'b0, relay_busy};

					REG_BERT_LANE0_RD:		rd_data	<= mgmt_lane0_rdata[7:0];
					REG_BERT_LANE0_RD_1:	rd_data	<= mgmt_lane0_rdata[15:8];
					REG_BERT_LANE0_STAT:	rd_data	<= {6'b0, mgmt_lane0_rx_rstdone, drp_busy[0]};

					REG_BERT_LANE1_RD:		rd_data	<= mgmt_lane1_rdata[7:0];
					REG_BERT_LANE1_RD_1:	rd_data	<= mgmt_lane1_rdata[15:8];
					REG_BERT_LANE1_STAT:	rd_data	<= {6'b0, mgmt_lane1_rx_rstdone, drp_busy[1]};

					default: begin
						rd_data	<= 0;
					end

				endcase

			end

		end

		//Execute a write
		if(wr_en) begin

			//Crypto accelerator registers are decoded separately
			if(wr_addr >= REG_CRYPT_BASE) begin

				//E register
				if(wr_addr[7:0] >= REG_E) begin
					crypt_e_mgmt[wr_addr[4:0]*8 +: 8]	<= wr_data;

					if(wr_addr[4:0] == 5'h1f) begin
						crypt_in_updated				<= 1;
						crypto_active					<= 1;
					end

				end

				//work_in register
				else if(wr_addr[7:0] >= REG_WORK) begin
					crypt_work_in_mgmt[wr_addr[4:0]*8 +: 8]	<= wr_data;
				end

			end

			//Ethernet MAC
			else if(wr_addr >= REG_EMAC_BUFFER_LO) begin
				txfifo_wr_en	<= 1;
				txfifo_wr_data	<= wr_data;
			end

			//Mux selectors are decoded separately
			else if(wr_addr >= REG_MUXSEL_BASE) begin
				muxsel[wr_addr[3:0]]	<= wr_data[3:0];
			end

			else begin

				case(wr_addr[7:0])

					REG_MGMT0_MDIO:		mgmt0_phy_wr_data[7:0]	<= wr_data;
					REG_MGMT0_MDIO_1:	mgmt0_phy_wr_data[15:8]	<= wr_data;
					REG_MGMT0_MDIO_2: begin
						mgmt0_phy_reg_addr			<= wr_data[4:0];
						mgmt0_phy_md_addr[2:0]		<= wr_data[7:5];
					end
					REG_MGMT0_MDIO_3: begin
						mgmt0_phy_md_addr[4:3]		<= wr_data[1:0];
						mgmt0_phy_reg_rd			<= wr_data[5];
						mgmt0_phy_reg_wr			<= wr_data[6];
					end

					REG_RELAY_TOGGLE: begin
						relay_channel				<= wr_data[1:0];
					end
					REG_RELAY_TOGGLE_1: begin
						relay_dir					<= wr_data[7];
						relay_en					<= 1;
					end

					REG_BERT_LANE0_PRBS: begin
						serdes_config_updated		<= 1;
						rx0_config.prbsmode			<= wr_data[2:0];
						tx0_config.prbsmode			<= wr_data[6:4];
					end

					REG_BERT_LANE0_TX: begin
						tx0_config.swing			<= wr_data[3:0];
						tx0_config.precursor[3:0]	<= wr_data[7:4];
					end
					REG_BERT_LANE0_TX_1: begin
						tx0_config.precursor[4]		<= wr_data[0];
						tx0_config.postcursor		<= wr_data[5:1];
						tx0_config.enable			<= wr_data[6];
						tx0_config.invert			<= wr_data[7];
						serdes_config_updated		<= 1;
					end

					REG_BERT_LANE0_WD:		mgmt_wdata[7:0]		<= wr_data;
					REG_BERT_LANE0_WD_1:	mgmt_wdata[15:8]	<= wr_data;
					REG_BERT_LANE0_AD: 		mgmt_addr[7:0]		<= wr_data;
					REG_BERT_LANE0_AD_1: begin
						mgmt_lane0_en				<= 1;
						mgmt_we						<= wr_data[7];
						mgmt_addr[8]				<= wr_data[0];

					end

					REG_BERT_LANE0_CLK: begin
						tx0_config.clkdiv			<= wr_data[2:0];
						tx0_config.clk_from_qpll	<= wr_data[3];
						serdes_config_updated		<= 1;
					end

					REG_BERT_LANE0_RST: begin
						rx0_config.pmareset			<= wr_data[0];
						tx0_config.tx_reset			<= wr_data[1];
						serdes_config_updated		<= 1;
					end

					REG_BERT_LANE0_RX: begin
						rx0_config.invert			<= wr_data[0];
						serdes_config_updated		<= 1;
					end

					REG_BERT_LANE1_PRBS: begin
						rx1_config.prbsmode			<= wr_data[2:0];
						tx1_config.prbsmode			<= wr_data[6:4];
						serdes_config_updated		<= 1;
					end

					REG_BERT_LANE1_TX: begin
						tx1_config.swing			<= wr_data[3:0];
						tx1_config.precursor[3:0]	<= wr_data[7:4];
					end
					REG_BERT_LANE1_TX_1: begin
						tx1_config.precursor[4]		<= wr_data[0];
						tx1_config.postcursor		<= wr_data[5:1];
						tx1_config.enable			<= wr_data[6];
						tx1_config.invert			<= wr_data[7];
						serdes_config_updated		<= 1;
					end

					REG_BERT_LANE1_WD:		mgmt_wdata[7:0]		<= wr_data;
					REG_BERT_LANE1_WD_1:	mgmt_wdata[15:8]	<= wr_data;
					REG_BERT_LANE1_AD:		mgmt_addr[7:0]		<= wr_data;
					REG_BERT_LANE1_AD_1: begin
						mgmt_lane1_en				<= 1;
						mgmt_we						<= wr_data[7];
						mgmt_addr[8]				<= wr_data[0];
					end

					REG_BERT_LANE1_CLK: begin
						tx1_config.clkdiv			<= wr_data[2:0];
						tx1_config.clk_from_qpll	<= wr_data[3];
						serdes_config_updated		<= 1;
					end

					REG_BERT_LANE1_RST: begin
						rx1_config.pmareset			<= wr_data[0];
						tx1_config.tx_reset			<= wr_data[1];
						serdes_config_updated		<= 1;
					end

					REG_BERT_LANE1_RX: begin
						rx1_config.invert			<= wr_data[0];
						serdes_config_updated		<= 1;
					end

					REG_EMAC_COMMIT:	txfifo_wr_commit <= 1;

				endcase

			end

		end

	end

endmodule
