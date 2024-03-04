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

	Channel names: IN0...7, OUT0...7, IO8...11

	Commands:
		[chan]:DIR [IN|OUT] (only for IO8..11)
		Set bidirectional channel direction

		[chan]:LEVEL [mV] (only for OUT4...7, IO8...11)
		Set logic-1 level for the channel

		[chan]:MUX [index]
		Set mux selector for given output or bidirectional channel

		[chan]:THRESH [mV]
		Set input threshold
 */
#include "triggercrossbar.h"
#include <ctype.h>

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

	//DEBUG: log the parsed command
	/*if(query)
		g_log("Query\n");
	if(subject)
		g_log("Subject: %s\n", subject);
	else
		g_log("(no subject)\n");
	g_log("Command: %s\n", command);
	if(args)
		g_log("Args: %s\n", args);
	else
		g_log("(no args)\n");
	*/

	//Process queries
	if(query)
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
	}

	//Not a query, just a regular command
	else
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
	}
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

	//invalid
	else
		return 0;
}
