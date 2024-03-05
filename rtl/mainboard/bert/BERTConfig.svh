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

`ifndef BERTConfig_svh
`define BERTConfig_svh

typedef struct packed
{
	logic[2:0]	prbsmode;	//GTX TXPRBSSEL
							//0 = normal
							//1 = PRBS-7
							//2 = PRBS-15
							//3 = PRBS-23
							//4 = PRBS-31
							//5 = reserved (PCIe compliance pattern but only in 20/40 bit bus width)
							//6 = 2 UI period squarewave
							//7 = 32 UI period squarewave

	logic		invert;		//1 = invert, 0 = normal

	logic[3:0]	swing;		//GTX TXDIFFCTRL
							//See table 3-30 of UG476, all values assume FFE taps at zero
							//0 =  269 mV
							//1 =  336
							//2 =  407
							//3 =  474
							//4 =  543
							//5 =  609
							//6 =  677
							//7 =  741
							//8 =  807
							//9 =  866
							//a =  924
							//b =  973
							//c = 1018
							//d = 1056
							//e = 1092
							//f = 1119

	logic[4:0]	postcursor;	//GTX TXPOSTCURSOR
							//See table 3-30 of UG476
							//00 =  0.00 dB
							//01 =  0.22
							//02 =  0.45
							//03 =  0.68
							//04 =  0.92
							//05 =  1.16
							//06 =  1.41
							//07 =  1.67
							//08 =  1.94
							//09 =  2.21
							//0a =  2.50
							//0b =  2.79
							//0c =  3.10
							//0d =  3.41
							//0e =  3.74
							//0f =  4.08
							//10 =  4.44
							//11 =  4.81
							//12 =  5.19
							//13 =  5.60
							//14 =  6.02
							//15 =  6.47
							//16 =  6.94
							//17 =  7.43
							//18 =  7.96
							//19 =  8.52
							//1a =  9.12
							//1b =  9.76
							//1c = 10.46
							//1d = 11.21
							//1e = 12.04
							//1f = 12.96

	logic[4:0] precursor;	//GTX TXPRECURSOR
							//See table 3-30 of UG476
							//00 =  0.00 dB
							//01 =  0.22
							//02 =  0.45
							//03 =  0.68
							//04 =  0.92
							//05 =  1.16
							//06 =  1.41
							//07 =  1.67
							//08 =  1.94
							//09 =  2.21
							//0a =  2.50
							//0b =  2.79
							//0c =  3.10
							//0d =  3.41
							//0e =  3.74
							//0f =  4.08
							//10 =  4.44
							//11 =  4.81
							//12 =  5.19
							//13 =  5.60
							//14 =  6.02
							//Saturates, codes 15 to 1f are all 6.02 dB


} bert_txconfig_t;

typedef struct packed
{
	logic[2:0]	prbsmode;	//GTX RXPRBSSEL
	logic		invert;
} bert_rxconfig_t;

`endif
