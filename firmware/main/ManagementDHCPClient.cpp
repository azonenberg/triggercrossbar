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

#include "triggercrossbar.h"
#include "ManagementDHCPClient.h"

///@brief KVS key for DHCP enable state
static const char* g_dhcpEnableObjectID = "dhcp.enable";

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Notification handlers from DHCP stack

void ManagementDHCPClient::OnIPAddressChanged(IPv4Address addr)
{
	g_ipConfig.m_address = addr;

	//recalculate broadcast address
	for(int i=0; i<4; i++)
		g_ipConfig.m_broadcast.m_octets[i] = g_ipConfig.m_address.m_octets[i] | ~g_ipConfig.m_netmask.m_octets[i];
}

void ManagementDHCPClient::OnDefaultGatewayChanged(IPv4Address addr)
{
	g_ipConfig.m_gateway = addr;
}

void ManagementDHCPClient::OnSubnetMaskChanged(IPv4Address addr)
{
	g_ipConfig.m_netmask = addr;

	//recalculate broadcast address
	for(int i=0; i<4; i++)
		g_ipConfig.m_broadcast.m_octets[i] = g_ipConfig.m_address.m_octets[i] | ~g_ipConfig.m_netmask.m_octets[i];
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Serialization

void ManagementDHCPClient::LoadConfigFromKVS()
{
	//Check if we're using DHCP and, if so, enable it
	bool enable = g_kvs->ReadObject(g_dhcpEnableObjectID, false);
	if(enable)
		Enable();
	else
		Disable();
}

void ManagementDHCPClient::SaveConfigToKVS()
{
	g_kvs->StoreObjectIfNecessary(g_dhcpEnableObjectID, IsEnabled(), false);
}
