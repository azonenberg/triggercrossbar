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

module LogicAnalyzer #(

	//Capture buffer size, in 32-bit words
	//Default is 32K words so 1M bits
	parameter		CAPTURE_BUF_SIZE = 32'h8000
)(
	APB.completer			apb,

	input wire				rx_clk,
	input wire[31:0]		rx_data,

	//rx clock domain
	input wire				rx_trigger
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// We only support 16-bit APB, throw synthesis error for anything else

	if(apb.DATA_WIDTH != 16)
		apb_bus_width_is_invalid();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Buffer memory

	localparam ADDR_BITS = $clog2(CAPTURE_BUF_SIZE);

	//Write side
	logic					mem_wr_en	= 0;
	logic[ADDR_BITS-1:0]	mem_wr_addr	= 0;
	logic[31:0]				mem_wr_data	= 0;

	logic					mem_rd_en;
	logic[ADDR_BITS-1:0]	mem_rd_addr;
	wire[31:0]				mem_rd_data;

	MemoryMacro #(
		.WIDTH(32),
		.DEPTH(CAPTURE_BUF_SIZE),
		.TRUE_DUAL(0),
		.PORTA_WRONLY(1),
		.USE_BLOCK(1),
		.OUT_REG(1)
	) rxbuf (
		.porta_clk(rx_clk),
		.porta_en(mem_wr_en),
		.porta_addr(mem_wr_addr),
		.porta_we(mem_wr_en),
		.porta_din(mem_wr_data),
		.porta_dout(),

		.portb_clk(apb.pclk),
		.portb_en(mem_rd_en),
		.portb_addr(mem_rd_addr),
		.portb_we(1'b0),
		.portb_din(32'h0),
		.portb_dout(mem_rd_data)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Waveform capture state machine

	logic[ADDR_BITS-1:0]	trig_addr		= 0;

	wire					arm_sync;
	logic					capture_done = 0;

	//desired trigger position within the buffer
	logic[ADDR_BITS-1:0]	trigger_pos	= CAPTURE_BUF_SIZE / 2;

	enum logic[1:0]
	{
		LA_STATE_DISARMED	= 0,
		LA_STATE_PRE_TRIG	= 1,
		LA_STATE_ARMED		= 2,
		LA_STATE_CAPTURE	= 3
	} la_state = LA_STATE_DISARMED;

	logic[ADDR_BITS-1:0]	mem_wr_addr_next;
	logic					trigger_idle;
	logic[ADDR_BITS-1:0]	end_addr;
	always_comb begin
		mem_wr_addr_next	= mem_wr_addr + 1;
		trigger_idle		= (la_state == LA_STATE_DISARMED);
	end

	always_ff @(posedge rx_clk) begin

		capture_done	<= 0;

		case(la_state)

			//idle, not yet armed
			LA_STATE_DISARMED: begin

				if(arm_sync) begin
					la_state	<= LA_STATE_PRE_TRIG;

					mem_wr_en	<= 1;
					mem_wr_addr	<= 0;
					mem_wr_data	<= rx_data;
				end

			end	//LA_STATE_DISARMED

			//collecting pre trigger samples
			LA_STATE_PRE_TRIG: begin

				mem_wr_en		<= 1;
				mem_wr_addr		<= mem_wr_addr_next;
				mem_wr_data		<= rx_data;

				if(mem_wr_addr >= trigger_pos)
					la_state	<= LA_STATE_ARMED;

			end	//LA_STATE_PRE_TRIG

			//armed, collecting data until trigger event shows up
			LA_STATE_ARMED: begin

				mem_wr_en		<= 1;
				mem_wr_addr		<= mem_wr_addr_next;
				mem_wr_data		<= rx_data;

				if(rx_trigger) begin
					la_state	<= LA_STATE_CAPTURE;
					trig_addr	<= mem_wr_addr;
					end_addr	<= mem_wr_addr + trigger_pos;
				end

			end	//LA_STATE_ARMED

			//capturing, wait for buffer to fill
			LA_STATE_CAPTURE: begin
				mem_wr_en		<= 1;
				mem_wr_addr		<= mem_wr_addr_next;
				mem_wr_data		<= rx_data;

				if(mem_wr_addr_next == end_addr) begin
					mem_wr_en		<= 0;
					capture_done	<= 1;
					la_state		<= LA_STATE_DISARMED;
				end

			end	//LA_STATE_CAPTURE

		endcase


	end

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Register map

	typedef enum logic[apb.ADDR_WIDTH-1:0]
	{
		REG_TRIGGER		= 'h00,		//0		Trigger arm request (write only)
									//7 	Trigger idle flag (indicates we're in STATE_DISARMED)

		REG_BUF_ADDR	= 'h04,		//Offset of RX_BUF within the capture buffer
									//For example BUF_ADDR='h42 means APB address 10 maps to 'h42,
									//address 11 maps to 'h43, etc
		REG_BUF_ADDR_HI	= 'h06,		//high half of REG_BUF_ADDR

		REG_BUF_SIZE	= 'h08,		//size of the capture memory buffer
									//(app is free to not read it all, but hardware always runs full depth)
		REG_BUF_SIZE_HI	= 'h0a,


		REG_RX_BUF		= 'h20		//Capture buffer window (memory mapped)
	} regid_t;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// CDC synchronizers

	logic				apb_pready_next;

	logic				arm;
	wire				trigger_idle_sync;
	wire[ADDR_BITS-1:0]	trig_addr_sync;

	PulseSynchronizer sync_arm(
		.clk_a(apb.pclk),
		.pulse_a(arm),
		.clk_b(rx_clk),
		.pulse_b(arm_sync));

	ThreeStageSynchronizer #(
		.IN_REG(1)
	) sync_idle (
		.clk_in(rx_clk),
		.din(trigger_idle),
		.clk_out(apb.pclk),
		.dout(trigger_idle_sync)
	);

	RegisterSynchronizer #(
		.WIDTH(ADDR_BITS)
	) sync_trigaddr (
		.clk_a(rx_clk),
		.en_a(capture_done),
		.ack_a(),
		.reg_a(trig_addr),

		.clk_b(apb.pclk),
		.updated_b(),
		.reset_b(1'b0),
		.reg_b(trig_addr_sync)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Prefetch buffer

	logic		prefetch_start;
	logic[4:0]	prefetch_count = 0;
	logic		prefetch_valid = 0;
	logic[4:0]	prefetch_count_ff	= 0;
	logic[4:0]	prefetch_count_ff2	= 0;

	wire[31:0]	prefetch_rdata;

	logic[6:0]	prefetch_raddr;
	assign prefetch_raddr = apb.paddr - REG_RX_BUF;

	MemoryMacro #(
		.WIDTH(32),
		.DEPTH(32),
		.TRUE_DUAL(0),
		.PORTA_WRONLY(1),
		.USE_BLOCK(0),
		.OUT_REG(1)
	) prefetchbuf (
		.porta_clk(apb.pclk),
		.porta_en(prefetch_valid),
		.porta_addr(prefetch_count_ff2),
		.porta_we(prefetch_valid),
		.porta_din(mem_rd_data),
		.porta_dout(),

		.portb_clk(apb.pclk),
		.portb_en(apb_pready_next),
		.portb_addr(prefetch_raddr[6:2]),
		.portb_we(1'b0),
		.portb_din(32'h0),
		.portb_dout(prefetch_rdata)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// APB interface logic

	logic[ADDR_BITS-1:0]	rd_base_addr;
	logic[ADDR_BITS-1:0]	rd_base_addr_ff;

	//base address for upcoming reads after applying offsets
	logic[ADDR_BITS-1:0]	rd_real_base_addr = 0;

	//Combinatorial readback, but with one cycle of latency
	always_comb begin

		apb_pready_next	= apb.psel && apb.penable && !apb.pready;

		apb.prdata		= 0;
		apb.pslverr		= 0;

		arm				= 0;
		prefetch_start	= 0;

		//Base address of reads
		rd_base_addr		= rd_base_addr_ff;
		rd_real_base_addr	= rd_base_addr + trig_addr_sync - trigger_pos;

		//this is one cycle after the request is valid due to pipeline latency on reads
		if(apb.pready) begin

			//read
			if(!apb.pwrite) begin

				case(apb.paddr)
					REG_TRIGGER:		apb.prdata = { trigger_idle_sync, 7'h0 };
					REG_BUF_ADDR: begin
						if(ADDR_BITS > 16)
							apb.prdata = rd_base_addr[15:0];
						else
							apb.prdata = rd_base_addr[ADDR_BITS-1:0];
					end
					REG_BUF_ADDR_HI: begin
						if(ADDR_BITS > 16)
							apb.prdata = rd_base_addr[ADDR_BITS-1:16];
						else
							apb.prdata = 0;
					end
					REG_BUF_SIZE:		apb.prdata = CAPTURE_BUF_SIZE[15:0];
					REG_BUF_SIZE_HI:	apb.prdata = CAPTURE_BUF_SIZE[31:16];

					//assume it's a read from RX_BUF
					default: begin
						if(apb.paddr[1])
							apb.prdata = prefetch_rdata[31:16];
						else
							apb.prdata = prefetch_rdata[15:0];
					end
				endcase

			end

			//write
			else begin

				case(apb.paddr)
					REG_TRIGGER:		arm = 1;

					REG_BUF_ADDR: begin
						if(ADDR_BITS > 16)
							rd_base_addr[15:0]	= apb.pwdata;
						else
							rd_base_addr[ADDR_BITS-1:0] = apb.pwdata[ADDR_BITS-1:0];
					end
					REG_BUF_ADDR_HI: begin
						if(ADDR_BITS > 16)
							rd_base_addr[ADDR_BITS-1:16]	= apb.pwdata[ADDR_BITS-17:0];
						prefetch_start			= 1;
					end

					//nothing else is writable
					default:			apb.pslverr	= 1;
				endcase

			end

		end
	end

	always_ff @(posedge apb.pclk or negedge apb.preset_n) begin

		//Reset
		if(!apb.preset_n) begin
			apb.pready			<= 0;
			rd_base_addr_ff		<= 0;
		end

		//Normal path
		else begin

			mem_rd_en			<= 0;
			mem_rd_addr			<= 0;

			//Register request flags
			//address/write data don't need to be registered, they'll be kept stable
			apb.pready		<= apb_pready_next;

			rd_base_addr_ff	<= rd_base_addr;

			if(prefetch_start) begin
				mem_rd_en		<= 1;
				mem_rd_addr		<= rd_real_base_addr;
				prefetch_count	<= 1;
			end

			if(prefetch_count) begin
				mem_rd_en		<= 1;
				mem_rd_addr		<= rd_real_base_addr + prefetch_count;
				prefetch_count	<= prefetch_count + 1;
			end

			prefetch_count_ff	<= prefetch_count;
			prefetch_count_ff2	<= prefetch_count_ff;
			prefetch_valid		<= mem_rd_en;

		end

	end

endmodule
