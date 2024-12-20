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

`include "../../../antikernel-ipcores/amba/apb/APBTypes.sv"

/**
	@file
	@author	Andrew D. Zonenberg
	@brief	APB register access to system health sensors

	Register map:
		0x0000		idcode[31:0]
		0x0004		serial[63:0]
		0x000c		fan0_rpm
		0x0010		fan1_rpm
		0x0014		die_temp
		0x0018		volt_core
		0x001c		volt_ram
		0x0020		volt_aux
		0x0024		usercode[31:0]
 */
module APB_SystemInfo(

	//The APB bus
	APB.completer 					apb,

	//Divided clock for device ID stuff
	input wire						clk_sysinfo,

	//Sensor values
	input wire[15:0]				fan0_rpm,
	input wire[15:0]				fan1_rpm
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// We only support 32-bit APB, throw synthesis error for anything else

	if(apb.DATA_WIDTH != 32)
		apb_bus_width_is_invalid();

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// USERCODE register

	wire[31:0] usercode;

	USR_ACCESSE2 user(
		.DATA(usercode),
		.CFGCLK(),
		.DATAVALID()
		);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Device ID

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

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// XADC sensors

	wire[15:0]	die_temp;
	wire[15:0]	volt_core;
	wire[15:0]	volt_ram;
	wire[15:0]	volt_aux;

	OnDieSensors_7series #(
		.EXT_IN_ENABLE(16'h0)
	) sensors (
		.clk(apb.pclk),
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
	// Readback logic (all combinatorial)

	always_comb begin

		//Always ready immediately when selected
		apb.pready	= apb.psel && apb.penable;

		//By default: reads OK with all-zero data, writes give error since we're read only
		apb.pslverr	= apb.pwrite;
		apb.prdata	= 0;

		//Mux read data
		case(apb.paddr)

			8'h00:		apb.prdata = idcode;
			8'h04:		apb.prdata = die_serial[31:0];
			8'h08:		apb.prdata = die_serial[63:32];
			8'h0c:		apb.prdata = fan0_rpm;
			8'h10:		apb.prdata = fan1_rpm;
			8'h14:		apb.prdata = die_temp;
			8'h18:		apb.prdata = volt_core;
			8'h1c:		apb.prdata = volt_ram;
			8'h20:		apb.prdata = volt_aux;
			8'h24:		apb.prdata = usercode[31:0];

			//Invalid / out of range
			default:	apb.pslverr = 1;

		endcase

		//Throw error if serial or idcode aren't valid
		if( (apb.paddr < 8'h04) && !idcode_valid)
			apb.pslverr = 1;
		if( (apb.paddr < 8'h10) && (apb.paddr >= 8'h04) && !die_serial_valid)
			apb.pslverr = 1;

		//Throw error on unaligned register access
		if(apb.paddr[1:0] != 0)
			apb.pslverr	= 1;

	end

endmodule
