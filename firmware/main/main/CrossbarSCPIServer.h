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
	@brief Declaration of CrossbarSCPIServer
 */
#ifndef CrossbarSCPIServer_h
#define CrossbarSCPIServer_h

#include "SCPIServer.h"

class CrossbarSCPIConnectionState
{
public:
	CrossbarSCPIConnectionState()
	{ Clear(); }

	///@brief Clears connection state
	void Clear()
	{
		m_valid = false;
		m_socket = nullptr;
		m_rxBuffer.Reset();
	}

	///@brief True if the connection is valid
	bool	m_valid;

	///@brief Socket state handle
	TCPTableEntry* m_socket;

	///@brief Packet reassembly buffer (may span multiple TCP segments)
	CircularFIFO<SCPI_RX_BUFFER_SIZE> m_rxBuffer;
};

/**
	@brief SCPI server for the crossbar
 */
class CrossbarSCPIServer : public SCPIServer<MAX_SCPI_CONNS, CrossbarSCPIConnectionState>
{
public:
	CrossbarSCPIServer(TCPProtocol& tcp);

	virtual void OnConnectionAccepted(TCPTableEntry* socket);
	virtual void OnConnectionClosed(TCPTableEntry* socket);
	virtual void GracefulDisconnect(int id, TCPTableEntry* socket);

protected:
	virtual void OnCommand(char* line, TCPTableEntry* socket) override;

	void DoQuery(const char* subject, const char* command, TCPTableEntry* socket);
	void DoCommand(const char* subject, const char* command, const char* args);

	int GetChannelID(const char* name);

	void UpdateTxLane(int lane);
	void UpdateRxLane(int lane);
	void UpdateClocks(int lane);

	uint16_t SerdesDRPRead(uint8_t lane, uint16_t regid);
	void SerdesDRPWrite(uint8_t lane, uint16_t regid, uint16_t regval);

	void PrepareForEyeScan(uint8_t chan);
	void StartEyeBERMeasurement(uint8_t chan, int prescale, int xoff, int yoff);
	float DoEyeBERMeasurement(uint8_t chan, int prescaleMax, int xoff, int yoff);

	void SerdesPMAReset(uint8_t lane);

	void PrintFloat(CharacterDevice& buf, float f);

	void UpdateDirectionLEDs();
};

#endif
