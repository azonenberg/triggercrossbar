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

/**
	@file
	@brief Implementation of CrossbarSCPIServer

	Crossbar channel names: IN0...7, OUT0...7, IO8...11

	BERT channel names: RX0-1, TX0-1

	Common commands:
		*IDN?

	Crossbar commands:
		[chan]:DIR [IN|OUT] (only for IO8..11)
		Set bidirectional channel direction

		[chan]:LEVEL [mV] (only for OUT4...7, IO8...11)
		Set logic-1 level for the channel

		[chan]:MUX [index]
		Set mux selector for given output or bidirectional channel

		[chan]:THRESH [mV]
		Set input threshold

	BERT commands:
		[chan]:PATTERN USER, PRBS7, PRBS15, PRBS23, PRBS31, FASTSQUARE, SLOWSQUARE

		[chan]:SWING [step]

		[chan]:PRECURSOR [step]

		[chan]:POSTCURSOR [step]

		[chan]:INVERT [0|1]

		[chan]:ENABLE [0|1]

		[chan]:CLKDIV [step]

		[chan]:CLKSEL [QPLL | CPLL]

		[chan]:EYESCAN?

		[chan]:HBATHTUB?

		[chan]:PRESCALE?
 */
#include "triggercrossbar.h"
#include <ctype.h>
#include <math.h>
#include "SocketReplyBuffer.h"

///@brief Transmit pattern IDs
uint8_t g_bertTxPattern[2] = {0};

///@brief Receive pattern IDs
uint8_t g_bertRxPattern[2] = {0};

///@brief Transmit swing steps
uint8_t g_bertTxSwing[2] = {0};

///@brief Transmit precursor steps
uint8_t g_bertTxPreCursor[2] = {0};

///@brief Transmit postcursor steps
uint8_t g_bertTxPostCursor[2] = {0};

///@brief Transmit invert flag
bool g_bertTxInvert[2] = {0};

///@brief Receive invert flag
bool g_bertRxInvert[2] = {0};

///@brief Transmit enable flag
bool g_bertTxEnable[2] = {false, false};

///@brief Receive eye scan prescaler
uint8_t g_bertRxPrescale[2] = {5, 5};

///@brief Mux selectors
uint8_t g_muxsel[12] = {0};

///@brief Bidir port directions
bool g_bidirOut[4] = {0};

///@brief Transmit sub-rate control
uint8_t g_bertTxClkDiv[2] = {1, 1};

///@brief Receive sub-rate control
uint8_t g_bertRxClkDiv[2] = {1, 1};

///@brief Clock selectors for BERT TX channels
bool g_bertTxClkSelIsQpll[2] = {false, false};

///@brief Clock selectors for BERT RX channels
bool g_bertRxClkSelIsQpll[2] = {false, false};

///Names of PRBS patterns
static const char* g_patternNames[8] =
{
	"USER",
	"PRBS7",
	"PRBS15",
	"PRBS23",
	"PRBS31",
	"RESERVED",
	"FASTSQUARE",
	"SLOWSQUARE"
};

//Note, there's some duplicates (multiple logical registers are different bitfields in one physical register)
enum gtxregids
{
	REG_ES_QUAL_MASK		= 0x031,
	REG_ES_SDATA_MASK		= 0x036,
	REG_ES_VERT_OFFSET		= 0x03b,
	REG_ES_HORZ_OFFSET		= 0x03c,
	REG_ES_CONTROL 			= 0x03d,
	REG_PMA_RSV2			= 0x082,
	REG_ES_CONTROL_STATUS	= 0x151,
	REG_ES_ERROR_COUNT		= 0x14f,
	REG_ES_SAMPLE_COUNT		= 0x150

	/*
	ES_QUALIFIER		= 02c - 030
	ES_QUAL_MASK		= 031 - 035
	ES_SDATA_MASK		= 036 - 03a
	ES_PRESCALE			= 03b 15:11
	ES_VERT_OFFSET		= 03b 7:0
	ES_HORZ_OFFSET		= 03c 11:0
	ES_EYE_SCAN_EN		= 03d bit 8
	ES_CONTROL			= 03d bit 5:0
	PMA_RSV2			= 082
	ES_ERROR_COUNT		= 14f
	ES_SAMPLE_COUNT		= 150
	ES_CONTROL_STATUS	= 151
	ES_RDATA			= 152 - 156
	ES_SDATA			= 157 - 15b
	*/
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction / destruction

CrossbarSCPIServer::CrossbarSCPIServer(TCPProtocol& tcp)
	: SCPIServer(tcp)
{
}

CrossbarSCPIServer::~CrossbarSCPIServer()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Socket event handlers

void CrossbarSCPIServer::OnConnectionAccepted(TCPTableEntry* socket)
{
	//Make a new entry in the socket state table
	int id = AllocateConnectionID(socket);
	if(id < 0)
		return;

	//TODO: do stuff
}

void CrossbarSCPIServer::OnConnectionClosed(TCPTableEntry* socket)
{
	//Connection was terminated by the other end, close our state so we can reuse it
	auto id = GetConnectionID(socket);
	if(id >= 0)
		m_state[id].Clear();
}

void CrossbarSCPIServer::GracefulDisconnect(int id, TCPTableEntry* socket)
{
	m_state[id].Clear();
	m_tcp.CloseSocket(socket);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Command handling

void CrossbarSCPIServer::UpdateDirectionLEDs()
{
	uint8_t dir = 0;

	if(g_bidirOut[0])
		dir |= 0x1;
	else
		dir |= 0x10;

	if(g_bidirOut[1])
		dir |= 0x2;
	else
		dir |= 0x20;

	if(g_bidirOut[2])
		dir |= 0x4;
	else
		dir |= 0x40;

	if(g_bidirOut[3])
		dir |= 0x8;
	else
		dir |= 0x80;

	SetFrontPanelDirectionLEDs(dir);
}

void CrossbarSCPIServer::OnCommand(char* line, TCPTableEntry* socket)
{
	/*g_log("Got SCPI command: %s\n", line);
	LogIndenter li(g_log);*/

	//Chunk the SCPI command up into subject, command, query flag, and arguments
	//At first, assume we have a command with no subject
	char* subject = nullptr;
	char* command = line;
	char* args = nullptr;
	bool query = false;

	//Then loop over the string and figure out what it actually is
	for(int i=0; line[i] != '\0'; i++)
	{
		//If we get a colon, everything left of it is the subject, then the command
		if(line[i] == ':')
		{
			line[i] = '\0';
			subject = line;
			command = line + i + 1;
			continue;
		}

		//If we get a space and don't have args yet, everything right of it is the args
		if(isspace(line[i]) && !args)
		{
			line[i] = '\0';
			args = line + i + 1;
		}

		//If it's a query, stop
		if(line[i] == '?')
		{
			line[i] = '\0';
			query = true;
			break;
		}
	}

	//Process queries
	if(query)
		DoQuery(subject, command, socket);

	//Not a query, just a regular command
	else
		DoCommand(subject, command, args);
}

void CrossbarSCPIServer::UpdateClocks(int lane)
{
	uint8_t regval = g_bertTxClkDiv[lane] | (g_bertRxClkDiv[lane] << 4);
	if(g_bertTxClkSelIsQpll[lane])
		regval |= 0x8;
	if(g_bertRxClkSelIsQpll[lane])
		regval |= 0x80;

	//Push to FPGA (need to set/release resets as we do this)
	//TODO: only reset if we changed clock source
	if(lane == 0)
	{
		g_fpga->BlockingWrite8(REG_BERT_LANE0_RST, 0x06);
		g_fpga->BlockingWrite8(REG_BERT_LANE0_CLK, regval);
		g_fpga->BlockingWrite8(REG_BERT_LANE0_RST, 0x00);
	}
	else
	{
		g_fpga->BlockingWrite8(REG_BERT_LANE1_RST, 0x06);
		g_fpga->BlockingWrite8(REG_BERT_LANE1_CLK, regval);
		g_fpga->BlockingWrite8(REG_BERT_LANE1_RST, 0x00);
	}
}

void CrossbarSCPIServer::DoCommand(const char* subject, const char* command, const char* args)
{
	//Threshold for input
	if(!strcmp(command, "THRESH"))
	{
		//need to have a channel and value
		if(!subject || !args)
			return;

		int chan = GetChannelID(subject);
		int mv = atoi(args);

		//Figure out which DAC channel to use for each channel
		//No rhyme or reason here, depends on PCB layout
		static const int channels[12] = {7, 6, 1, 0, 5, 4, 3, 2, 4, 5, 3, 6};
		static const int dacs[12] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1 };
		g_rxDacs[dacs[chan]]->SetChannelMillivolts(channels[chan], mv);
	}

	//Level for output ports
	else if(!strcmp(command, "LEV"))
	{
		//need to have a channel and value
		if(!subject || !args)
			return;

		int chan = GetChannelID(subject);
		int mv = atoi(args);

		//Figure out which DAC channel to use for each channel
		//No rhyme or reason here, depends on PCB layout
		static const int channels[12] = {0, 0, 0, 0, 7, 6, 5, 4, 3, 1, 2, 0};
		g_txDac->SetChannelMillivolts(channels[chan], mv);
	}

	//Mux selector
	else if(!strcmp(command, "MUX"))
	{
		//need to have a channel and value
		if(!subject || !args)
			return;

		int chan = GetChannelID(subject);
		int muxsel = atoi(args);

		g_muxsel[chan] = muxsel;
		g_fpga->BlockingWrite8(REG_MUXSEL_BASE + chan, muxsel);
	}

	//Direction for bidir ports
	else if(!strcmp(command, "DIR"))
	{
		//need to have a channel and value
		if(!subject || !args)
			return;

		//need valid channel ID (IO port)
		int chan = GetChannelID(subject);
		if( (chan < 8) || (chan > 11) )
			return;
		chan -= 8;

		if(!strcmp(args, "IN"))
		{
			g_fpga->BlockingWrite16(REG_RELAY_TOGGLE, 0x8000 | chan);
			g_bidirOut[chan] = false;
		}
		else
		{
			g_fpga->BlockingWrite16(REG_RELAY_TOGGLE, 0x0000 | chan);
			g_bidirOut[chan] = true;
		}

		//Poll until not busy
		while(g_fpga->BlockingRead16(REG_RELAY_STAT) != 0)
		{}

		//Update direction LEDs
		UpdateDirectionLEDs();
	}

	//RX prescaler
	else if(!strcmp(command, "PRESCALE"))
	{
		//need to have a channel and value
		if(!subject || !args)
			return;

		//need valid channel ID (BERT port)
		int chan = GetChannelID(subject);
		if( (chan < 0) || (chan > 1) )
			return;
		if(subject[0] != 'R')
			return;

		//Set prescaler
		g_bertRxPrescale[chan] = atoi(args) & 0x1f;
	}

	//Output amplitude
	else if(!strcmp(command, "SWING"))
	{
		//need to have a channel and value
		if(!subject || !args)
			return;

		//need valid channel ID (BERT port)
		int chan = GetChannelID(subject);
		if( (chan < 0) || (chan > 1) )
			return;
		if(subject[0] != 'T')
			return;

		//TX amplitude
		g_bertTxSwing[chan] = atoi(args) & 0x0f;

		UpdateTxLane(chan);
	}

	//Output FFE taps
	else if(!strcmp(command, "PRECURSOR"))
	{
		//need to have a channel and value
		if(!subject || !args)
			return;

		//need valid channel ID (BERT port)
		int chan = GetChannelID(subject);
		if( (chan < 0) || (chan > 1) )
			return;
		if(subject[0] != 'T')
			return;

		//TX amplitude
		g_bertTxPreCursor[chan] = atoi(args) & 0x1f;

		UpdateTxLane(chan);
	}
	else if(!strcmp(command, "POSTCURSOR"))
	{
		//need to have a channel and value
		if(!subject || !args)
			return;

		//need valid channel ID (BERT port)
		int chan = GetChannelID(subject);
		if( (chan < 0) || (chan > 1) )
			return;
		if(subject[0] != 'T')
			return;

		//TX amplitude
		g_bertTxPostCursor[chan] = atoi(args) & 0x1f;

		UpdateTxLane(chan);
	}

	//Output inversion
	else if(!strcmp(command, "INVERT"))
	{
		//need to have a channel and value
		if(!subject || !args)
			return;

		//need valid channel ID (BERT port)
		int chan = GetChannelID(subject);
		if( (chan < 0) || (chan > 1) )
			return;

		if(subject[0] == 'T')
		{
			if(!strcmp(args, "1"))
				g_bertTxInvert[chan] = true;
			else
				g_bertTxInvert[chan] = false;

			UpdateTxLane(chan);
		}
		else
		{
			if(!strcmp(args, "1"))
				g_bertRxInvert[chan] = true;
			else
				g_bertRxInvert[chan] = false;

			UpdateRxLane(chan);
		}
	}

	//Output inversion
	else if(!strcmp(command, "ENABLE"))
	{
		//need to have a channel and value
		if(!subject || !args)
			return;

		//need valid channel ID (BERT port)
		int chan = GetChannelID(subject);
		if( (chan < 0) || (chan > 1) )
			return;
		if(subject[0] != 'T')
			return;

		//TX amplitude
		if(!strcmp(args, "1"))
			g_bertTxEnable[chan] = true;
		else
			g_bertTxEnable[chan] = false;

		UpdateTxLane(chan);
	}

	//PRBS pattern
	else if(!strcmp(command, "PATTERN"))
	{
		//need to have a channel and value
		if(!subject || !args)
			return;

		//need valid channel ID (BERT port)
		int chan = GetChannelID(subject);
		if( (chan < 0) || (chan > 1) )
			return;

		//Decode the pattern (unrecognized = "user")
		int pattern = 0;
		for(int i=0; i<8; i++)
		{
			if(!strcmp(args, g_patternNames[i]))
			{
				pattern = i;
				break;
			}
		}

		//Figure out if this is the TX or RX lane and update accordingly
		if(subject[0] == 'T')
			g_bertTxPattern[chan] = pattern;
		else
			g_bertRxPattern[chan] = pattern;

		//Push to hardware
		uint8_t regval = (g_bertTxPattern[chan] << 4) | g_bertRxPattern[chan];
		if(chan == 0)
			g_fpga->BlockingWrite8(REG_BERT_LANE0_PRBS, regval);
		else
			g_fpga->BlockingWrite8(REG_BERT_LANE1_PRBS, regval);
	}

	else if(!strcmp(command, "CLKDIV"))
	{
		//need to have a channel and value
		if(!subject || !args)
			return;

		//need valid channel ID (BERT port)
		int chan = GetChannelID(subject);
		if( (chan < 0) || (chan > 1) )
			return;

		int step = atoi(args);
		if(step < 1)
			step = 1;
		if(step > 5)
			step = 5;

		//Update the divider
		if(subject[0] == 'T')
			g_bertTxClkDiv[chan] = step;
		else
			g_bertRxClkDiv[chan] = step;

		//Push updates to hardware
		UpdateClocks(chan);
	}

	else if(!strcmp(command, "CLKSEL"))
	{
		//need to have a channel and value
		if(!subject || !args)
			return;

		//need valid channel ID (BERT port)
		int chan = GetChannelID(subject);
		if( (chan < 0) || (chan > 1) )
			return;

		bool value = !strcmp(args, "QPLL");

		if(subject[0] == 'T')
			g_bertTxClkSelIsQpll[chan] = value;
		else
			g_bertRxClkSelIsQpll[chan] = value;

		UpdateClocks(chan);


	}
}

uint16_t CrossbarSCPIServer::SerdesDRPRead(uint8_t lane, uint16_t regid)
{
	uint16_t offset = BERT_LANE_STRIDE * lane;

	//Send the command
	g_fpga->BlockingWrite16(REG_BERT_LANE0_AD + offset, regid);

	//Make sure we're ready
	//while(0 != g_fpga->BlockingRead8(REG_BERT_LANE0_STAT + offset))
	//{}

	//Read the result
	//(blocking poll shouldn't be needed after all of the CDC delays etc?
	return g_fpga->BlockingRead16(REG_BERT_LANE0_RD + offset);
}

void CrossbarSCPIServer::SerdesDRPWrite(uint8_t lane, uint16_t regid, uint16_t regval)
{
	uint16_t offset = BERT_LANE_STRIDE * lane;

	g_fpga->BlockingWrite16(REG_BERT_LANE0_WD + offset, regval);
	g_fpga->BlockingWrite16(REG_BERT_LANE0_AD + offset, regid | 0x8000);
}

void CrossbarSCPIServer::PrintFloat(CharacterDevice& buf, float f)
{
	if(fabs(f) < 1e-20)
	{
		buf.Printf("0.000e-0");
		return;
	}

	int base = floor(log10(f));

	float rescaled = f * pow(10, -base);
	int ipart = floor(rescaled);
	float fpart = rescaled - ipart;
	int ifpart = fpart * 1000;

	buf.Printf("%d.%03de%d", ipart, ifpart, base);
}

void CrossbarSCPIServer::SerdesPMAReset(uint8_t lane)
{
	uint16_t offset = BERT_LANE_STRIDE*lane;

	//Reset the PMA
	g_fpga->BlockingWrite8(REG_BERT_LANE0_RST + offset, 1);
	g_fpga->BlockingWrite8(REG_BERT_LANE0_RST + offset, 0);

	//Read and throw away a few status values until reset takes effect
	for(int i=0; i<3; i++)
		g_fpga->BlockingRead8(REG_BERT_LANE0_STAT + offset);

	//Poll until reset completes
	while(1)
	{
		auto stat = g_fpga->BlockingRead8(REG_BERT_LANE0_STAT + offset);
		if((stat & 0x2) == 2)
			break;
	}
}

/**
	@brief Make sure the requested SERDES lane is ready for eye scanning
 */
void CrossbarSCPIServer::PrepareForEyeScan(uint8_t chan)
{
	//TODO: this should happen during init to avoid glitching links?
	//Set PMA_RSV2 bit 5 to power up the eye scan
	//If we do this, we also have to reset the PMA
	auto rsv2 = SerdesDRPRead(chan, REG_PMA_RSV2);
	if( (rsv2 & 0x20) == 0)
	{
		g_log("Powering up eye scan (PMA_RSV2 bit 5)\n");
		SerdesDRPWrite(chan, REG_PMA_RSV2, rsv2 | 0x20);
		SerdesPMAReset(chan);
	}
	auto ctl = SerdesDRPRead(chan, REG_ES_CONTROL);
	if((ctl & 0x100) == 0)
	{
		g_log("Enabling eye scan\n");
		SerdesDRPWrite(chan, REG_ES_CONTROL, ctl | 0x100);
		SerdesPMAReset(chan);
	}
}

/**
	@brief Begin an eye BER measurement (but don't read the results)
 */
void CrossbarSCPIServer::StartEyeBERMeasurement(uint8_t chan, int prescale, int xoff, int yoff)
{
	int yoffSigned;
	if(yoff >= 0)
		yoffSigned = (yoff & 0x7f);
	else
		yoffSigned = 0x80 | (-yoff & 0x7f);

	//Need to read-modify-write since ES_CONTROL shares same register as ES_EYE_SCAN_EN and other stuff
	//Reset eye scan
	auto regval = (SerdesDRPRead(chan, REG_ES_CONTROL) & 0xffe0) | 0x100;
	SerdesDRPWrite(chan, REG_ES_CONTROL, regval);

	//Set prescale and horizontal offset
	SerdesDRPWrite(chan, REG_ES_VERT_OFFSET, (prescale << 11) | yoffSigned);

	//Set horizontal offset
	//DEBUG: See if phase unification is wrong, try ultrascale version (always 1)
	//regval |= 0x800;
	//g_log("regval=%04x for xoff=%d\n", regval, xoff);
	SerdesDRPWrite(chan, REG_ES_HORZ_OFFSET, (xoff & 0xfff));

	//Start the BER measurement
	SerdesDRPWrite(chan, REG_ES_CONTROL, regval | 0x1);

	//Poll ES_CONTROL_STATUS until the DONE bit goes high
	while( (SerdesDRPRead(chan, REG_ES_CONTROL_STATUS) & 1) == 0)
	{}
}

/**
	@brief Measure eye BER at a single x/y point
 */
float CrossbarSCPIServer::DoEyeBERMeasurement(uint8_t chan, int prescaleMax, int xoff, int yoff)
{
	float ber = 0;
	int prescale;
	uint16_t errcount;
	uint16_t sampcount;
	uint64_t realSamples;

	//Loop through prescale values and iterate until we stop saturating the sample counter
	for(prescale=0; prescale<=prescaleMax; prescale++)
	{
		//Initiate the measurement and block until it finishes
		StartEyeBERMeasurement(chan, prescale, xoff, yoff);

		//Read count values
		errcount = SerdesDRPRead(chan, REG_ES_ERROR_COUNT);
		sampcount = SerdesDRPRead(chan, REG_ES_SAMPLE_COUNT);

		//Correct for prescaler 2^(1+regval)
		//(but for some reason we have to prescale by another few bits to get expected 0.5 BER outside eye)
		uint64_t sampleScale = (1 << (prescale+5));
		realSamples = sampcount * sampleScale;

		//Calculate error rate
		if(errcount == 0)
			ber = 0;
		else
			ber = errcount * 1.0 / realSamples;

		//If sample counter is not saturated, break out of the loop
		if(sampcount < 65535)
			break;
	}

	//Pretty-print BER since we don't have this integrated in our printf yet
	//buf.Printf("%3d,%2d,%9d,%9d,", xoff, prescale, errcount, (int)realSamples);
	//PrintFloat(buf, ber);
	//buf.Printf("\n");

	return ber;
}

void CrossbarSCPIServer::DoQuery(const char* subject, const char* command, TCPTableEntry* socket)
{
	if(!strcmp(command, "*IDN"))
	{
		SocketReplyBuffer buf(m_tcp, socket);
		buf.Printf("AntikernelLabs,AKL-TXB1,%02x%02x%02x%02x%02x%02x%02x%02x,0.1\n",
			g_fpgaSerial[0], g_fpgaSerial[1], g_fpgaSerial[2], g_fpgaSerial[3],
			g_fpgaSerial[4], g_fpgaSerial[5], g_fpgaSerial[6], g_fpgaSerial[7]);
	}

	//Eye scan
	else if(!strcmp(command, "EYESCAN"))
	{
		//need valid channel ID (BERT port)
		if(!subject)
			return;
		int chan = GetChannelID(subject);
		if( (chan < 0) || (chan > 1) )
			return;
		if(subject[0] != 'R')
			return;

		SocketReplyBuffer buf(m_tcp, socket);

		PrepareForEyeScan(chan);

		//No qualifier
		for(int i=0; i<5; i++)
			SerdesDRPWrite(chan, REG_ES_QUAL_MASK + i, 0xffff);

		//SDATA for 32 bit bus width: 40 1s, 32 0s, 8 1s
		SerdesDRPWrite(chan, REG_ES_SDATA_MASK + 0, 0x00ff);
		SerdesDRPWrite(chan, REG_ES_SDATA_MASK + 1, 0x0000);
		SerdesDRPWrite(chan, REG_ES_SDATA_MASK + 2, 0xff00);
		SerdesDRPWrite(chan, REG_ES_SDATA_MASK + 3, 0xffff);
		SerdesDRPWrite(chan, REG_ES_SDATA_MASK + 4, 0xffff);

		//Larger sweep range for sub-rate modes
		int16_t xrange = 16 << g_bertRxClkDiv[chan];

		//Sweep vertical offset
		//GTX step size is 1.89 mV/code
		//https://support.xilinx.com/s/question/0D52E00006hphfdSAA/gtx-transceiver-margin-analysis-verctical-voltage-range?language=en_US
		for(int yoff=-127; yoff <= 127; yoff += 4)
		{
			//Sweep horizontal offset
			for(int16_t xoff = -xrange; xoff <= xrange; xoff ++)
			{
				auto ber = DoEyeBERMeasurement(chan, g_bertRxPrescale[chan], xoff, yoff);

				//this outer loop takes a while, process any FPGA events that may have appeared each iteration
				while(CheckForFPGAEvents())
				{}

				PrintFloat(buf, ber);
				buf.Printf(",");
			}
			buf.Printf("\n");
		}
	}

	//Horizontal bathtub curve at the center of the eye
	else if(!strcmp(command, "HBATHTUB"))
	{
		//need valid channel ID (BERT port)
		if(!subject)
			return;
		int chan = GetChannelID(subject);
		if( (chan < 0) || (chan > 1) )
			return;
		if(subject[0] != 'R')
			return;

		SocketReplyBuffer buf(m_tcp, socket);

		PrepareForEyeScan(chan);

		//No qualifier
		for(int i=0; i<5; i++)
			SerdesDRPWrite(chan, REG_ES_QUAL_MASK + i, 0xffff);

		//SDATA for 32 bit bus width: 40 1s, 32 0s, 8 1s
		SerdesDRPWrite(chan, REG_ES_SDATA_MASK + 0, 0x00ff);
		SerdesDRPWrite(chan, REG_ES_SDATA_MASK + 1, 0x0000);
		SerdesDRPWrite(chan, REG_ES_SDATA_MASK + 2, 0xff00);
		SerdesDRPWrite(chan, REG_ES_SDATA_MASK + 3, 0xffff);
		SerdesDRPWrite(chan, REG_ES_SDATA_MASK + 4, 0xffff);

		//Larger sweep range for sub-rate modes
		int16_t xrange = 16 << g_bertRxClkDiv[chan];

		//Keep vertical offset at zero
		//Sweep horizontal offset
		for(int16_t xoff = -xrange; xoff <= xrange; xoff ++)
		{
			auto ber = DoEyeBERMeasurement(chan, g_bertRxPrescale[chan], xoff, 0);

			//this outer loop takes a while, process any FPGA events that may have appeared each iteration
			while(CheckForFPGAEvents())
			{}

			PrintFloat(buf, ber);
			buf.Printf(",");
			buf.Flush();
		}
		buf.Printf("\n");
	}

	//Mux selector
	else if(!strcmp(command, "MUX"))
	{
		//need valid channel ID
		if(!subject)
			return;
		int chan = GetChannelID(subject);
		if( (chan < 0) || (chan >= 12) )
			return;

		SocketReplyBuffer buf(m_tcp, socket);
		buf.Printf("%d\n", g_muxsel[chan]);
	}

	//Bidir port direction
	else if(!strcmp(command, "DIR"))
	{
		//need valid channel ID
		if(!subject)
			return;
		int chan = GetChannelID(subject);
		if( (chan < 8) || (chan >= 12) )
			return;

		SocketReplyBuffer buf(m_tcp, socket);
		if(g_bidirOut[chan-8])
			buf.Printf("OUT\n");
		else
			buf.Printf("IN\n");
	}

	//Clock sources
	else if(!strcmp(command, "CLKSEL"))
	{
		//need valid channel ID (BERT port)
		if(!subject)
			return;
		int chan = GetChannelID(subject);
		if( (chan < 0) || (chan > 1) )
			return;

		SocketReplyBuffer buf(m_tcp, socket);
		if(subject[0] == 'T')
			buf.Printf("%cPLL\n", g_bertTxClkSelIsQpll[chan] ? 'Q' : 'C');
		else
			buf.Printf("%cPLL\n", g_bertRxClkSelIsQpll[chan] ? 'Q' : 'C');
	}

	//ES_PRESCALE register
	else if(!strcmp(command, "PRESCALE"))
	{
		//need valid channel ID (BERT port)
		if(!subject)
			return;
		int chan = GetChannelID(subject);
		if( (chan < 0) || (chan > 1) )
			return;
		if(subject[0] != 'R')
			return;

		SocketReplyBuffer buf(m_tcp, socket);
		buf.Printf("%d\n", g_bertRxPrescale[chan]);
	}

	//Output amplitude
	else if(!strcmp(command, "SWING"))
	{
		//need valid channel ID (BERT port)
		if(!subject)
			return;
		int chan = GetChannelID(subject);
		if( (chan < 0) || (chan > 1) )
			return;
		if(subject[0] != 'T')
			return;

		SocketReplyBuffer buf(m_tcp, socket);
		buf.Printf("%d\n", g_bertTxSwing[chan]);
	}

	//Output FFE taps
	else if(!strcmp(command, "PRECURSOR"))
	{
		//need valid channel ID (BERT port)
		if(!subject)
			return;
		int chan = GetChannelID(subject);
		if( (chan < 0) || (chan > 1) )
			return;
		if(subject[0] != 'T')
			return;

		SocketReplyBuffer buf(m_tcp, socket);
		buf.Printf("%d\n", g_bertTxPreCursor[chan]);
	}

	else if(!strcmp(command, "POSTCURSOR"))
	{
		//need valid channel ID (BERT port)
		if(!subject)
			return;
		int chan = GetChannelID(subject);
		if( (chan < 0) || (chan > 1) )
			return;
		if(subject[0] != 'T')
			return;

		SocketReplyBuffer buf(m_tcp, socket);
		buf.Printf("%d\n", g_bertTxPostCursor[chan]);
	}

	//Output inversion
	else if(!strcmp(command, "INVERT"))
	{
		//need valid channel ID (BERT port)
		if(!subject)
			return;
		int chan = GetChannelID(subject);
		if( (chan < 0) || (chan > 1) )
			return;

		SocketReplyBuffer buf(m_tcp, socket);
		if(subject[0] == 'T')
			buf.Printf("%d\n", g_bertTxInvert[chan]);
		else
			buf.Printf("%d\n", g_bertRxInvert[chan]);
	}

	//Output enable
	else if(!strcmp(command, "ENABLE"))
	{
		//need valid channel ID (BERT port)
		if(!subject)
			return;
		int chan = GetChannelID(subject);
		if( (chan < 0) || (chan > 1) || (subject[0] != 'T') )
			return;

		SocketReplyBuffer buf(m_tcp, socket);
		buf.Printf("%d\n", g_bertTxEnable[chan]);
	}

	//PRBS pattern
	else if(!strcmp(command, "PATTERN"))
	{
		//need valid channel ID (BERT port)
		if(!subject)
			return;
		int chan = GetChannelID(subject);
		if( (chan < 0) || (chan > 1) )
			return;

		//Build the string into the outbound TCP buffer
		auto segment = m_tcp.GetTxSegment(socket);
		auto payload = reinterpret_cast<char*>(segment->Payload());

		//Figure out what pattern we're using
		if(subject[0] == 'T')
			strncpy(payload, g_patternNames[g_bertTxPattern[chan]], TCP_IPV4_PAYLOAD_MTU);
		else
			strncpy(payload, g_patternNames[g_bertRxPattern[chan]], TCP_IPV4_PAYLOAD_MTU);
		strncat(payload, "\n", TCP_IPV4_PAYLOAD_MTU);

		//And send the reply
		m_tcp.SendTxSegment(socket, segment, strlen(payload));
	}

	//Output enable
	else if(!strcmp(command, "CLKDIV"))
	{
		//need valid channel ID (BERT port)
		if(!subject)
			return;
		int chan = GetChannelID(subject);
		if( (chan < 0) || (chan > 1) )
			return;

		SocketReplyBuffer buf(m_tcp, socket);
		if(subject[0] == 'T')
			buf.Printf("%d\n", g_bertTxClkDiv[chan]);
		else
			buf.Printf("%d\n", g_bertRxClkDiv[chan]);
	}
}

void CrossbarSCPIServer::UpdateTxLane(int lane)
{
	uint16_t regval = g_bertTxSwing[lane];
	if(g_bertTxInvert[lane])
		regval |= 0x8000;
	if(g_bertTxEnable[lane])
		regval |= 0x4000;
	regval |= g_bertTxPostCursor[lane] << 9;
	regval |= g_bertTxPreCursor[lane] << 4;

	if(lane == 0)
		g_fpga->BlockingWrite16(REG_BERT_LANE0_TX, regval);
	else
		g_fpga->BlockingWrite16(REG_BERT_LANE1_TX, regval);
}

void CrossbarSCPIServer::UpdateRxLane(int lane)
{
	uint8_t regval = 0;
	if(g_bertRxInvert[lane])
		regval |= 1;

	if(lane == 0)
		g_fpga->BlockingWrite16(REG_BERT_LANE0_RX, regval);
	else
		g_fpga->BlockingWrite16(REG_BERT_LANE1_RX, regval);
}

/**
	@brief Gets the port number given the hwname
 */
int CrossbarSCPIServer::GetChannelID(const char* name)
{
	//Input channels
	if( (name[0] == 'I') && (name[1] == 'N') )
	{
		int chnum = name[2] - '0';
		if(chnum > 7)
			chnum = 7;
		if(chnum < 0)
			chnum = 0;
		return chnum;
	}

	//Output channels
	if( (name[0] == 'O') && (name[1] == 'U') && (name[2] == 'T') )
	{
		int chnum = name[3] - '0';
		if(chnum > 7)
			chnum = 7;
		if(chnum < 0)
			chnum = 0;
		return chnum;
	}

	//Bidir channels
	if( (name[0] == 'I') && (name[1] == 'O') )
	{
		int chnum = atoi(name+2);
		if(chnum > 11)
			chnum = 11;
		if(chnum < 8)
			chnum = 8;

		return chnum;
	}

	//BERT channels
	if( ( (name[0] == 'T') || (name[0] == 'R') )&& (name[1] == 'X') )
	{
		int chnum = name[2] - '0';
		if(chnum > 1)
			chnum = 1;
		if(chnum < 0)
			chnum = 0;
		return chnum;
	}


	//invalid
	else
		return 0;
}
