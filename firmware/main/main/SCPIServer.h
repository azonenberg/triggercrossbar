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
	@brief Declaration of SCPIServer
 */
#ifndef SCPIServer_h
#define SCPIServer_h

#include "../../staticnet/net/tcp/TCPServer.h"

/**
	@brief Base class for a generic SCPI server
 */
template<int MAXCONNS, class ContextType>
class SCPIServer
	: public TCPServer<MAXCONNS, ContextType>
{
public:
	SCPIServer(TCPProtocol& tcp)
		: TCPServer<MAXCONNS, ContextType>(tcp)
	{}

	virtual ~SCPIServer()
	{}

	__attribute__((noinline))
	virtual bool OnRxData(TCPTableEntry* socket, uint8_t* payload, uint16_t payloadLen) override
	{
		//Look up the connection ID for the incoming session
		auto id = TCPServer<MAXCONNS, ContextType>::GetConnectionID(socket);
		if(id < 0)
			return true;

		//Push the segment data into our RX FIFO
		if(!TCPServer<MAXCONNS, ContextType>::m_state[id].m_rxBuffer.Push(payload, payloadLen))
			return false;

		//Process commands (may be >1 in a single segment)
		while(1)
		{
			//Rewind the FIFO then see if there's a newline in the buffer
			auto& fifo = TCPServer<MAXCONNS, ContextType>::m_state[id].m_rxBuffer;
			auto line = fifo.Rewind();
			auto len = fifo.ReadSize();
			if(len == 0)
				break;

			//Search for a \n
			bool newlineFound = false;
			size_t lineLen = 0;
			for(size_t i=0; i<len; i++)
			{
				if(line[i] == '\n')
				{
					//Convert the newline to a nul so we can use c string functions on it
					line[i] = '\0';

					//Done, we found it
					lineLen = i+1;
					newlineFound = true;
					break;
				}
			}

			//If NO newline was found, look at the overall line length.
			if(!newlineFound)
			{
				//If line was more than 512 bytes long and still no newline, assume something is screwy
				//and drop the connection
				if(len > 512)
				{
					TCPServer<MAXCONNS, ContextType>::m_state[id].Clear();
					TCPServer<MAXCONNS, ContextType>::m_tcp.CloseSocket(socket);
				}
				return false;
			}

			//We found a newline, process it
			OnCommand(reinterpret_cast<char*>(line), socket);

			//Pop the line data so we have space for more commands
			fifo.Pop(lineLen);
		}

		return true;
	}


protected:
	virtual void OnCommand(char* line, TCPTableEntry* socket) =0;
};

#endif
