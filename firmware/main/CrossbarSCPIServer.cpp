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

		[chan]:EYESCAN?
 */
#include "triggercrossbar.h"
#include <ctype.h>
#include <math.h>

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

///@brief Transmit enable flag
bool g_bertTxEnable[2] = {false, false};

///@brief Transmit sub-rate control
uint8_t g_bertTxClkDiv[2] = {1, 1};

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

		if(!strcmp(args, "IN"))
			g_fpga->BlockingWrite16(REG_RELAY_TOGGLE, 0x8000 | (chan - 8));
		else
			g_fpga->BlockingWrite16(REG_RELAY_TOGGLE, 0x0000 | (chan - 8));

		//Poll until not busy
		while(g_fpga->BlockingRead16(REG_RELAY_STAT) != 0)
		{}
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
		if(subject[0] != 'T')
			return;

		//TX amplitude
		if(!strcmp(args, "1"))
			g_bertTxInvert[chan] = true;
		else
			g_bertTxInvert[chan] = false;

		UpdateTxLane(chan);
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
		if(subject[0] != 'T')
			return;

		//TX amplitude
		int step = atoi(args);
		if(step < 1)
			step = 1;
		if(step > 5)
			step = 5;
		g_bertTxClkDiv[chan] = step;

		//Push to FPGA
		if(chan == 0)
			g_fpga->BlockingWrite8(REG_BERT_LANE0_CLK, step);
		else
			g_fpga->BlockingWrite8(REG_BERT_LANE1_CLK, step);
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

void CrossbarSCPIServer::PrintFloat(StringBuffer& buf, float f)
{
	if(fabs(f) < 1e-20)
	{
		buf.Printf("0.0");
		return;
	}

	int base = floor(log10(f));

	float rescaled = f * pow(10, -base);
	int ipart = floor(rescaled);
	float fpart = rescaled - ipart;
	int ifpart = fpart * 1000;

	buf.Printf("%d.%03de%d", ipart, ifpart, base);
}

void CrossbarSCPIServer::DoQuery(const char* subject, const char* command, TCPTableEntry* socket)
{
	if(!strcmp(command, "*IDN"))
	{
		//Build the string into the outbound TCP buffer
		auto segment = m_tcp.GetTxSegment(socket);
		auto payload = reinterpret_cast<char*>(segment->Payload());
		StringBuffer buf(payload, TCP_IPV4_PAYLOAD_MTU);
		buf.Printf("AntikernelLabs,AKL-TXB1,%02x%02x%02x%02x%02x%02x%02x%02x,0.1\n",
			g_fpgaSerial[0], g_fpgaSerial[1], g_fpgaSerial[2], g_fpgaSerial[3],
			g_fpgaSerial[4], g_fpgaSerial[5], g_fpgaSerial[6], g_fpgaSerial[7]);

		//And send the reply
		m_tcp.SendTxSegment(socket, segment, strlen(payload));
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

		//Prepare to send a reply
		auto segment = m_tcp.GetTxSegment(socket);
		auto payload = reinterpret_cast<char*>(segment->Payload());
		StringBuffer buf(payload, TCP_IPV4_PAYLOAD_MTU);

		//Use every bit position so don't touch ES_SDATA_MASK for now

		//for now leave Y position and prescale at whatever the default is
		//ES_VERT_OFFSET + ES_PRESCALE share register 0x03b

		//note, there's some duplicates
		enum regids
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

		//TODO: this should happen during init to avoid glitching links?
		//Set PMA_RSV2 bit 5 to power up the eye scan
		//If we do this, we also have to reset the PMA
		auto rsv2 = SerdesDRPRead(chan, REG_PMA_RSV2);
		if( (rsv2 & 0x20) == 0)
		{
			//Power up the eye scan block
			rsv2 |= 0x20;
			SerdesDRPWrite(chan, REG_PMA_RSV2, rsv2);

			//Reset the PMA
			g_fpga->BlockingWrite8(REG_BERT_LANE0_RST + BERT_LANE_STRIDE*chan, 1);
			g_fpga->BlockingWrite8(REG_BERT_LANE0_RST + BERT_LANE_STRIDE*chan, 0);

			//Read and throw away a few status values until reset takes effect
			for(int i=0; i<3; i++)
				g_fpga->BlockingRead8(REG_BERT_LANE0_STAT + BERT_LANE_STRIDE*chan);

			//Poll until reset completes
			while(1)
			{
				auto stat = g_fpga->BlockingRead8(REG_BERT_LANE0_STAT + BERT_LANE_STRIDE*chan);
				if((stat & 0x2) == 2)
					break;
			}
		}

		//No qualifier
		for(int i=0; i<5; i++)
			SerdesDRPWrite(chan, REG_ES_QUAL_MASK + i, 0xffff);

		//SDATA for 32 bit bus width: 40 1s, 32 0s, 8 1s
		SerdesDRPWrite(chan, REG_ES_SDATA_MASK + 0, 0x00ff);
		SerdesDRPWrite(chan, REG_ES_SDATA_MASK + 1, 0x0000);
		SerdesDRPWrite(chan, REG_ES_SDATA_MASK + 2, 0xff00);
		SerdesDRPWrite(chan, REG_ES_SDATA_MASK + 3, 0xffff);
		SerdesDRPWrite(chan, REG_ES_SDATA_MASK + 4, 0xffff);

		//Sweep vertical offset
		//GTX step size is 1.89 mV/code
		//https://support.xilinx.com/s/question/0D52E00006hphfdSAA/gtx-transceiver-margin-analysis-verctical-voltage-range?language=en_US
		for(int yoff=-127; yoff <= 127; yoff += 4)
		{
			//Sweep horizontal offset
			for(int16_t xoff = -32; xoff <= 32; xoff ++)
			{
				float ber = 0;
				int prescale;
				uint16_t errcount;
				uint16_t sampcount;
				uint64_t realSamples;

				//Loop through prescale values and iterate until we stop saturating the sample counter
				for(prescale=0; prescale<6; prescale++)
				{
					int yoffSigned;
					if(yoff >= 0)
						yoffSigned = (yoff & 0x7f);
					else
						yoffSigned = 0x80 | (-yoff & 0x7f);

					//Set prescale and horizontal offset
					SerdesDRPWrite(chan, REG_ES_VERT_OFFSET, (prescale << 11) | yoffSigned);

					//Set horizontal offset
					uint16_t regval = (xoff & 0xfff);

					//DEBUG: See if phase unification is wrong, try ultrascale version (always 1)
					//regval &= ~0x800;
					//g_log("regval=%04x for xoff=%d\n", regval, xoff);

					SerdesDRPWrite(chan, REG_ES_HORZ_OFFSET, regval);

					//Need to read-modify-write since ES_CONTROL shares same register as ES_EYE_SCAN_EN and other stuff
					//Reset eye scan
					regval = SerdesDRPRead(chan, REG_ES_CONTROL) & 0xffe0;

					//Go to BER measurement path
					regval |= 0x01;
					SerdesDRPWrite(chan, REG_ES_CONTROL, regval);

					//Poll ES_CONTROL_STATUS until the DONE bit goes high
					while( (SerdesDRPRead(chan, REG_ES_CONTROL_STATUS) & 1) == 0)
					{}

					//Read count values
					errcount = SerdesDRPRead(chan, REG_ES_ERROR_COUNT);
					sampcount = SerdesDRPRead(chan, REG_ES_SAMPLE_COUNT);

					//Correct for prescaler 2^(1+regval)
					//Also, we count every cycle but have a 32-bit datapath
					//Does that mean we have to multiply the sample count by 32?
					uint64_t sampleScale = (1 << (prescale+1)) * 32;
					realSamples = sampcount * sampleScale;

					//Calculate error rate
					if(errcount == 0)
						ber = 0;
					else
					{
						ber = errcount * 1.0 / realSamples;

						//ber = sampcount * 1.0 / errcount;
					}

					//If sample counter is not saturated, break out of the loop
					if(sampcount < 65535)
						break;
				}

				//Pretty-print BER since we don't have this integrated in our printf yet
				//buf.Printf("%3d,%2d,%5d,%8d,", xoff, prescale, errcount, (int)realSamples);
				//PrintFloat(buf, ber);
				//buf.Printf("\n");

				PrintFloat(buf, ber);
				buf.Printf(",");

				//If we have more than 1 kB of data in the segment, start a new one
				//TODO: Make wrapper class for reply buffer that will take care of this
				if(buf.length() > 1024)
				{
					m_tcp.SendTxSegment(socket, segment, buf.length());

					segment = m_tcp.GetTxSegment(socket);
					payload = reinterpret_cast<char*>(segment->Payload());
					buf = StringBuffer(payload, TCP_IPV4_PAYLOAD_MTU);
				}
			}
			buf.Printf("\n");
		}

		//And send the reply
		m_tcp.SendTxSegment(socket, segment, buf.length());
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

		//Build the string into the outbound TCP buffer
		auto segment = m_tcp.GetTxSegment(socket);
		auto payload = reinterpret_cast<char*>(segment->Payload());
		StringBuffer buf(payload, TCP_IPV4_PAYLOAD_MTU);
		buf.Printf("%d\n", g_bertTxSwing[chan]);

		//And send the reply
		m_tcp.SendTxSegment(socket, segment, strlen(payload));
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

		//Build the string into the outbound TCP buffer
		auto segment = m_tcp.GetTxSegment(socket);
		auto payload = reinterpret_cast<char*>(segment->Payload());
		StringBuffer buf(payload, TCP_IPV4_PAYLOAD_MTU);
		buf.Printf("%d\n", g_bertTxPreCursor[chan]);

		//And send the reply
		m_tcp.SendTxSegment(socket, segment, strlen(payload));
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

		//Build the string into the outbound TCP buffer
		auto segment = m_tcp.GetTxSegment(socket);
		auto payload = reinterpret_cast<char*>(segment->Payload());
		StringBuffer buf(payload, TCP_IPV4_PAYLOAD_MTU);
		buf.Printf("%d\n", g_bertTxPostCursor[chan]);

		//And send the reply
		m_tcp.SendTxSegment(socket, segment, strlen(payload));
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

		//Build the string into the outbound TCP buffer
		auto segment = m_tcp.GetTxSegment(socket);
		auto payload = reinterpret_cast<char*>(segment->Payload());
		StringBuffer buf(payload, TCP_IPV4_PAYLOAD_MTU);
		if(subject[0] == 'T')
			buf.Printf("%d\n", g_bertTxInvert[chan]);
		//else
		//	buf.Printf("%d\n", g_bertRxInvert[chan]);

		//And send the reply
		m_tcp.SendTxSegment(socket, segment, strlen(payload));
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

		//Build the string into the outbound TCP buffer
		auto segment = m_tcp.GetTxSegment(socket);
		auto payload = reinterpret_cast<char*>(segment->Payload());
		StringBuffer buf(payload, TCP_IPV4_PAYLOAD_MTU);
		buf.Printf("%d\n", g_bertTxEnable[chan]);

		//And send the reply
		m_tcp.SendTxSegment(socket, segment, strlen(payload));
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
		if( (chan < 0) || (chan > 1) || (subject[0] != 'T') )
			return;

		//Build the string into the outbound TCP buffer
		auto segment = m_tcp.GetTxSegment(socket);
		auto payload = reinterpret_cast<char*>(segment->Payload());
		StringBuffer buf(payload, TCP_IPV4_PAYLOAD_MTU);
		buf.Printf("%d\n", g_bertTxClkDiv[chan]);

		//And send the reply
		m_tcp.SendTxSegment(socket, segment, strlen(payload));
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
