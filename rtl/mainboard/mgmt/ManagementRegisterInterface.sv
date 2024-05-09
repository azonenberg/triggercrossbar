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
import CrossbarTypes::*;
import EthernetBus::*;

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

	//Configuration registers in port RX clock domains
	input wire						xg0_rx_clk,
	input wire						xg0_link_up,

	//Configuration registers in core clock domain
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

	input wire[11:0]				trig_in_led,
	input wire[11:0]				trig_out_led,

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
	output logic 					xg_txfifo_wr_en = 0,
	output logic[7:0] 				xg_txfifo_wr_data = 0,
	output logic	 				xg_txfifo_wr_commit = 0,

	output logic					front_shift_en = 0,
	output logic[7:0]				front_shift_data = 0,
	input wire						front_shift_done,
	input wire[7:0]					front_rx_data,
	output logic					front_cs_n = 1,

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
	input wire[255:0]				crypt_work_out,
	output logic					crypt_dsa_en = 0,
	output logic					crypt_dsa_base_en = 0,
	output logic					crypt_dsa_load = 0,
	output logic					crypt_dsa_rd = 0,
	input wire						crypt_dsa_done,
	output wire[1:0]				crypt_dsa_addr
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
	logic[1:0]		crypt_addr			= 0;

	typedef enum logic[2:0]
	{
		CRYPT_DH,			//writing work/e for DH
		CRYPT_DSA_LOWREG,	//writing dsa_in0-3
		CRYPT_DSA_FINAL,	//writing e for DSA
		CRYPT_DSA_RD,		//reading output
		CRYPT_DSA_BASE,		//writing dsa_in0-1 for scalarbase
		CRYPT_DSA_BASEFIN	//writing e for DSA scalarbase
	} crypt_mode_t;

	crypt_mode_t crypt_mode = CRYPT_DH;

	//Ignore toggles on updated_b for first few clocks after reset
	//seems sync glitches, at least in sim?
	wire			crypt_updated_sync;
	crypt_mode_t 	crypt_mode_sync;
	logic[3:0]		rst_count 			= 1;
	always_ff @(posedge clk_crypt) begin

		crypt_en			<= 0;
		crypt_dsa_en		<= 0;
		crypt_dsa_load		<= 0;
		crypt_dsa_rd		<= 0;
		crypt_dsa_base_en	<= 0;

		if(rst_count)
			rst_count		<= rst_count + 1;
		else begin

			if(crypt_updated_sync) begin

				case(crypt_mode_sync)
					CRYPT_DH:			crypt_en 			<= 1;
					CRYPT_DSA_LOWREG:	crypt_dsa_load		<= 1;
					CRYPT_DSA_BASE:		crypt_dsa_load		<= 1;
					CRYPT_DSA_FINAL: 	crypt_dsa_en		<= 1;
					CRYPT_DSA_RD:		crypt_dsa_rd		<= 1;
					CRYPT_DSA_BASEFIN: 	crypt_dsa_base_en	<= 1;
				endcase

			end

		end
	end

	RegisterSynchronizer #(
		.WIDTH(517)
	) sync_crypt_inputs (
		.clk_a(clk),
		.en_a(crypt_in_updated),
		.ack_a(),
		.reg_a({crypt_work_in_mgmt, crypt_e_mgmt, crypt_mode, crypt_addr}),

		.clk_b(clk_crypt),
		.updated_b(crypt_updated_sync),
		.reset_b(1'b0),
		.reg_b({crypt_work_in, crypt_e, crypt_mode_sync, crypt_dsa_addr})
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

	wire	crypt_dsa_done_sync;
	PulseSynchronizer sync_dsa_done(
		.clk_a(clk_crypt),
		.pulse_a(crypt_dsa_done),
		.clk_b(clk),
		.pulse_b(crypt_dsa_done_sync)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// List of named registers

	//Note that ManagementBridge uses MSB of address as read/write flag
	//so we actually have only 15 bits available for addressing

	//must match ManagementRegisterInterface in FPGAInterface.h
	typedef enum logic[15:0]
	{
		//Reasons for an IRQ
		REG_FPGA_IRQSTAT	= 16'h0020,		//
		REG_FPGA_IRQSTAT_1	= 16'h0021,		//
											// 0 = RX Ethernet frame ready

		//Ethernet MAC
		REG_EMAC_RXLEN		= 16'h0024,
		REG_EMAC_RXLEN_1	= 16'h0025,
		REG_EMAC_COMMIT		= 16'h0028,		//write any value to end the active packet
		REG_XG_COMMIT		= 16'h002c,		//write any value to end the active packet

		//MDIO controllers
		REG_MGMT0_MDIO		= 16'h0048,		//31    = busy flag (R)
		REG_MGMT0_MDIO_1	= 16'h0049,		//30    = write enable (W)
		REG_MGMT0_MDIO_2	= 16'h004a,		//29    = read enable (W)
		REG_MGMT0_MDIO_3	= 16'h004b,		//25:21 = phy addr (W)
											//20:16 = register addr (W)
											//15:0	= register data (RW)

		//Front panel SPI interface
		REG_FRONT_CTRL		= 16'h0050,		//0 = CS# value
		REG_FRONT_DATA		= 16'h0051,		//byte of data to send
		REG_FRONT_STAT		= 16'h0052,		//0 = transmitter busy
		REG_FRONT_LED_0		= 16'h0053,
		REG_FRONT_LED_1		= 16'h0054,
		REG_FRONT_LED_2		= 16'h0055,

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
											//6:4 = RX clock divider
											//7 = RX clock from QPLL (0 = CPLL)
		REG_BERT_LANE0_RST	= 16'h008e,		//0 = RX PMA reset
											//1 = TX soft reset
											//2 = RX soft reset

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

		//Ethernet MAC TX frame buffer
		//Any address in this range will be treated as writing to the end of the buffer
		REG_XG_TX_BUFFER_LO	= 16'h2000,
		REG_XG_TX_BUFFER_HI	= 16'h2fff,

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
		REG_WORK_OUT		= 16'h0060,
		REG_DSA_IN			= 16'h0080,
		REG_DSA_BASE		= 16'h0100
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
	logic					front_busy	= 0;


	logic[8:0]				packetWordsRead = 0;
	logic[8:0]				packetWordLength = 0;
	logic					rxheader_rd_en_ff = 0;

	//Front panel LED indicator state
	logic[3:0]				relay_state = 0;
	logic[23:0]				front_led_state;
	always_comb begin
		for(integer i=0; i<12; i=i+1)
			front_led_state[i+12]	<= trig_in_led[11-i];
		front_led_state[11:0]	<= trig_out_led[11:0];

		//If relays are in input mode, never light up the output port indicator
		for(integer i=0; i<12; i=i+1) begin
			if(i >= 8) begin
				if(relay_state[i-8])
					front_led_state[i]	= 0;
			end
		end

	end

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
		xg_txfifo_wr_en			<= 0;
		xg_txfifo_wr_commit		<= 0;
		relay_en				<= 0;
		serdes_config_updated	<= 0;
		mgmt_lane0_en			<= 0;
		mgmt_lane1_en			<= 0;
		front_shift_en			<= 0;

		//Track relay state
		if(relay_en)
			relay_state[relay_channel]	<= relay_dir;

		//Start a new read
		if(rd_en)
			reading	<= 1;

		//Finish a crypto operation
		if(crypt_out_updated || crypt_dsa_done_sync)
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

		//Manage SPI bus state
		if(front_shift_en)
			front_busy			<= 1;
		if(front_shift_done)
			front_busy			<= 0;

		//Track expected length of a packet being read
		rxheader_rd_en_ff	<= rxheader_rd_en;
		if(rxheader_rd_en_ff) begin
			if(rxheader_rd_data[1:0])
				packetWordLength	<= rxheader_rd_data[10:2] + 1;
			else
				packetWordLength	<= rxheader_rd_data[10:2];
		end
		if(rxheader_rd_en)
			packetWordsRead 		<= 0;
		if(rxfifo_rd_pop_single)
			packetWordsRead 		<= packetWordsRead + 1;

		//Continue a read
		if(rd_en || reading) begin
			rd_valid	<= 1;
			reading		<= 0;

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
					0: rd_data					<= rxfifo_rd_data[31:24];
					1: begin
						rd_data					<= rxfifo_rd_data[23:16];
						//pop the buffer since we've got the read data in the working register
						//if(packetWordsRead < packetWordLength)
						rxfifo_rd_pop_single	<= 1;
					end
					2:	rd_data					<= rxfifo_rd_data[15:8];
					3: begin
						rxfifo_rd_en			<= 1;
						rd_data					<= rxfifo_rd_data[7:0];
					end
				endcase

			end

			//Main register decoder
			else begin

				case(rd_addr)

					REG_FRONT_STAT:		rd_data <= {7'b0, front_busy };
					REG_FRONT_DATA:		rd_data	<= front_rx_data;

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
						if(rxheader_rd_data != 0)
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

				if(wr_addr[8:0] >= REG_DSA_BASE) begin
					crypt_mode								<= CRYPT_DSA_BASE;
					crypt_addr								<= wr_addr[6:5];
					crypt_work_in_mgmt[wr_addr[4:0]*8 +: 8]	<= wr_data;

					//Push the write onto the bus every 32 bytes
					if(wr_addr[4:0] == 5'h1f) begin
						crypt_in_updated				<= 1;
						//if(wr_addr[6:5] == 1)
							crypto_active				<= 1;
					end

				end

				//DSA input
				else if(wr_addr[8:0] >= REG_DSA_IN) begin
					crypt_mode								<= CRYPT_DSA_LOWREG;
					crypt_addr								<= wr_addr[6:5];
					crypt_work_in_mgmt[wr_addr[4:0]*8 +: 8]	<= wr_data;

					//Push the write onto the bus every 32 bytes
					if(wr_addr[4:0] == 5'h1f) begin
						crypt_in_updated				<= 1;
						if(wr_addr[6:5] == 1)
							crypto_active					<= 1;
					end

				end

				//write to status to read from the output buffer
				else if(wr_addr[8:0] == REG_CRYPT_STATUS) begin
					crypt_mode			<= CRYPT_DSA_RD;
					crypt_addr			<= wr_data[1:0];
					crypt_in_updated	<= 1;
				end

				//E register
				else if(wr_addr[8:0] >= REG_E) begin

					if(crypt_mode == CRYPT_DSA_LOWREG)
						crypt_mode						<= CRYPT_DSA_FINAL;
					else if(crypt_mode == CRYPT_DSA_BASE)
						crypt_mode						<= CRYPT_DSA_BASEFIN;

					crypt_e_mgmt[wr_addr[4:0]*8 +: 8]	<= wr_data;

					if(wr_addr[4:0] == 5'h1f) begin
						crypt_in_updated				<= 1;
						crypto_active					<= 1;
					end

				end

				//work_in register
				else if(wr_addr[8:0] >= REG_WORK) begin
					crypt_mode								<= CRYPT_DH;
					crypt_work_in_mgmt[wr_addr[4:0]*8 +: 8]	<= wr_data;
				end

			end

			//Ethernet MAC
			else if(wr_addr >= REG_XG_TX_BUFFER_LO) begin
				xg_txfifo_wr_en	<= 1;
				xg_txfifo_wr_data	<= wr_data;
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

					REG_FRONT_CTRL: front_cs_n		<= wr_data[0];
					REG_FRONT_DATA: begin
						front_shift_en				<= 1;
						front_shift_data			<= wr_data;
					end

					//Special registers for saying "push front panel LED state
					REG_FRONT_LED_0: begin
						front_shift_en				<= 1;
						front_shift_data			<= front_led_state[7:0];
					end
					REG_FRONT_LED_1: begin
						front_shift_en				<= 1;
						front_shift_data			<= front_led_state[15:8];
					end
					REG_FRONT_LED_2: begin
						front_shift_en				<= 1;
						front_shift_data			<= front_led_state[23:16];
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
						rx0_config.clkdiv			<= wr_data[6:4];
						rx0_config.clk_from_qpll	<= wr_data[7];
						serdes_config_updated		<= 1;
					end

					REG_BERT_LANE0_RST: begin
						rx0_config.pmareset			<= wr_data[0];
						tx0_config.tx_reset			<= wr_data[1];
						rx0_config.rx_reset			<= wr_data[2];
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
						rx1_config.clkdiv			<= wr_data[6:4];
						rx1_config.clk_from_qpll	<= wr_data[7];
						serdes_config_updated		<= 1;
					end

					REG_BERT_LANE1_RST: begin
						rx1_config.pmareset			<= wr_data[0];
						tx1_config.tx_reset			<= wr_data[1];
						rx1_config.rx_reset			<= wr_data[2];
						serdes_config_updated		<= 1;
					end

					REG_BERT_LANE1_RX: begin
						rx1_config.invert			<= wr_data[0];
						serdes_config_updated		<= 1;
					end

					REG_EMAC_COMMIT:	txfifo_wr_commit <= 1;
					REG_XG_COMMIT:		xg_txfifo_wr_commit <= 1;

				endcase

			end

		end

	end

endmodule
