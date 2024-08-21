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

#ifndef SocketReplyBuffer_h
#define SocketReplyBuffer_h

/**
	@brief Character device for conveniently outputting formatted ASCII text content of arbitrary length to a socket

	New TCP segments are created and sent as needed as data is written.

	The final segment (if any) is flushed when the buffer goes out of scope.
 */
class SocketReplyBuffer : public CharacterDevice
{
public:

	//Create the reply buffer
	SocketReplyBuffer(TCPProtocol& tcp, TCPTableEntry* socket)
		: m_tcp(tcp)
		, m_socket(socket)
		, m_segment(m_tcp.GetTxSegment(socket))
		, m_buf(reinterpret_cast<char*>(m_segment->Payload()), TCP_IPV4_PAYLOAD_MTU)
	{
	}

	virtual ~SocketReplyBuffer()
	{
		//If we have anything to send, send it
		if(m_buf.length())
			m_tcp.SendTxSegment(m_socket, m_segment, m_buf.length());

		//Nope, discard the unsent segment
		else
			m_tcp.CancelTxSegment(m_segment, m_socket);
	}

	virtual void PrintBinary(char ch) override
	{
		//Write the byte
		m_buf.PrintBinary(ch);

		//See if we have to send and make a new segment
		if( (m_buf.length() + 1) >= TCP_IPV4_PAYLOAD_MTU)
			Flush();
	}

	///@brief unimplemented but the base class doesn't know that
	virtual char BlockingRead() override
	{ return 0; }

	void Flush()
	{
		if(m_buf.length() == 0)
			return;

		m_tcp.SendTxSegment(m_socket, m_segment, m_buf.length());

		m_segment = m_tcp.GetTxSegment(m_socket);
		if(m_segment == nullptr)
		{
			g_log("failed to allocate buffer\n");
			while(1)
			{}
		}
		m_buf = StringBuffer(reinterpret_cast<char*>(m_segment->Payload()), TCP_IPV4_PAYLOAD_MTU);
	}

protected:

	///@brief TCP protocol stack we're using
	TCPProtocol& m_tcp;

	///@brief Socket we're replying to
	TCPTableEntry* m_socket;

	///@brief The current (not yet sent) TCP segment
	TCPSegment* m_segment;

	///@brief String buffer for our current segment
	StringBuffer m_buf;
};

#endif
