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
	@brief Implementation of ManagementSFTPServer
 */
#include "triggercrossbar.h"
#include "ManagementSFTPServer.h"
#include <staticnet/sftp/SFTPOpenPacket.h>

const char* g_frontPanelDfuPath = "/dfu/frontpanel";

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Filesystem wrapper APIs

bool ManagementSFTPServer::DoesFileExist(const char* path)
{
	if(!strcmp(path, g_frontPanelDfuPath))
		return true;

	//no other files to match
	return false;
}

bool ManagementSFTPServer::CanOpenFile(const char* path, uint32_t accessMask, uint32_t flags)
{
	//If we already have an open file, abort
	//(we don't support concurrent file operations)
	if(m_openFile != FILE_ID_NONE)
		return false;

	//Check if this is a DFU file path
	bool isDFU = false;
	if(!strcmp(path, g_frontPanelDfuPath))
		isDFU = true;

	//DFU files must be opened in overwrite/truncate mode
	if(isDFU)
	{
		switch(flags & SFTPOpenPacket::SSH_FXF_ACCESS_DISPOSITION)
		{
			//valid modes
			case SFTPOpenPacket::SSH_FXF_CREATE_NEW:
			case SFTPOpenPacket::SSH_FXF_CREATE_TRUNCATE:
			case SFTPOpenPacket::SSH_FXF_TRUNCATE_EXISTING:
				break;

			//anything else isn't allowed
			default:
				return false;
		}

		//access mask must request write data
		if( (accessMask & SFTPPacket::ACE4_WRITE_DATA) == 0)
			return false;

		//no readback allowed for the ELF binaries
		//(since they're not actually stored as ELF and we don't want to synthesize one on the fly!)
		if( (accessMask & SFTPPacket::ACE4_READ_DATA) != 0)
			return false;

		//otherwise we're good
		return true;
	}

	//If we get here, no go
	return false;
}

uint32_t ManagementSFTPServer::OpenFile(
	const char* path,
	[[maybe_unused]] uint32_t accessMask,
	[[maybe_unused]] uint32_t flags)
{
	g_cliUART.Printf("OpenFile(%s, access=%x, flags=%x)\n", path, accessMask, flags);

	//For now, all of our files are stored in a single handle
	//See which one to use
	if(!strcmp(path, g_frontPanelDfuPath))
	{
		m_openFile = FILE_ID_FRONT_DFU;
		g_cliUART.Printf("Front panel DFU\n");

		//TODO: reboot the MCU in DFU mode and wait for it to acknowledge
	}

	//Return the constant handle zero for all open requests
	return 0;
}

void ManagementSFTPServer::WriteFile(
	uint32_t handle,
	uint64_t offset,
	const uint8_t* data,
	uint32_t len)
{
	//TODO: validate offsets are sequential
	g_cliUART.Printf("WriteFile: %u bytes at offset=%d\n", len, (uint32_t)offset);
}

bool ManagementSFTPServer::CloseFile([[maybe_unused]] uint32_t handle)
{
	//always allowed
	return true;
}
