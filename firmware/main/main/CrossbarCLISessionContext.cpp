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
#include <algorithm>
#include "CrossbarCLISessionContext.h"
#include <ctype.h>
#include "../../front/main/regids.h"
#include <bootloader/BootloaderAPI.h>

static const char* hostname_objid = "hostname";

//List of all valid commands
enum cmdid_t
{
	CMD_ADDRESS,
	CMD_ALL,
	CMD_ARP,
	CMD_AUTHORIZED,
	CMD_CACHE,
	//CMD_CLEAR,
	CMD_COMMIT,
	CMD_COMPACT,
	//CMD_COUNTERS,
	//CMD_DESCRIPTION,
	CMD_DETAIL,
	//CMD_DEBUG,
	CMD_DFU,
	CMD_DHCP,
	CMD_EXIT,
	CMD_FINGERPRINT,
	CMD_FULL,
	CMD_FLASH,
	CMD_GATEWAY,
	CMD_HARDWARE,
	CMD_HOSTNAME,
	CMD_IP,
	CMD_KEY,
	CMD_KEYS,
	CMD_MMD,
	CMD_NO,
	CMD_NTP,
	CMD_REFRESH,
	CMD_RELOAD,
	CMD_REGISTER,
	CMD_ROLLBACK,
	CMD_ROUTE,
	CMD_SERVER,
	CMD_SET,
	CMD_SHOW,
	CMD_SSH,
	CMD_SSH_ED25519,
	//CMD_STATUS,
	//CMD_TEMPERATURE,
	//CMD_TEST,
	CMD_USERNAME,
	CMD_VERSION,
	CMD_ZEROIZE
};
/*
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// "clear"

static const clikeyword_t g_clearCountersCommands[] =
{
	{"interface",	CMD_INTERFACE,		g_interfaceCommands,	"Clear performance counters for an interface"},
	{nullptr,		INVALID_COMMAND,	nullptr,				nullptr}
};

static const clikeyword_t g_clearCommands[] =
{
	{"counters",	CMD_COUNTERS,		g_clearCountersCommands,	"Clear performance counters"},
	{nullptr,		INVALID_COMMAND,	nullptr,					nullptr}
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// "debug"

static const clikeyword_t g_debugCommands[] =
{
	{"temperature",	CMD_TEMPERATURE,	nullptr,	"Enable temperature logging"},
	{nullptr,		INVALID_COMMAND,	nullptr,	nullptr}
};

*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// "flash"

static const clikeyword_t g_flashCommands[] =
{
	{"compact",		CMD_COMPACT,		nullptr,	"Force a compact operation on the key-value store, even if not full"},
	{nullptr,		INVALID_COMMAND,	nullptr,	nullptr}
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// "hostname"

static const clikeyword_t g_hostnameCommands[] =
{
	{"<string>",	FREEFORM_TOKEN,		nullptr,	"New host name"},
	{nullptr,		INVALID_COMMAND,	nullptr,	nullptr}
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// "ip"

static const clikeyword_t g_ipAddressCommands[] =
{
	{"dhcp",		CMD_DHCP,			nullptr,				"Use DHCP for IP configuration"},
	{"<address>",	FREEFORM_TOKEN,		nullptr,				"New IPv4 address and subnet mask in x.x.x.x/yy format"},
	{nullptr,		INVALID_COMMAND,	nullptr,				nullptr}
};

static const clikeyword_t g_ipGatewayCommands[] =
{
	{"<address>",	FREEFORM_TOKEN,		nullptr,				"New IPv4 default gateway"},
	{nullptr,		INVALID_COMMAND,	nullptr,				nullptr}
};

static const clikeyword_t g_ipCommands[] =
{
	{"address",		CMD_ADDRESS,		g_ipAddressCommands,	"Set the IPv4 address of the device"},
	{"gateway",		CMD_GATEWAY,		g_ipGatewayCommands,	"Set the IPv4 default gateway of the device"},

	{nullptr,		INVALID_COMMAND,	nullptr,				nullptr}
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// "no"

static const clikeyword_t g_noSshKeyCommands[] =
{
	{"<slot>",			FREEFORM_TOKEN,			nullptr,					"Slot number of the authorized SSH key to delete"},
	{nullptr,			INVALID_COMMAND,		nullptr,					nullptr}
};

static const clikeyword_t g_noSshCommands[] =
{
	{"key",				CMD_KEY,				g_noSshKeyCommands,			"Remove authorized SSH keys"},

	{nullptr,			INVALID_COMMAND,		nullptr,					nullptr}
};

static const clikeyword_t g_noFlashCommands[] =
{
	{"<key>",			FREEFORM_TOKEN,			nullptr,					"Key of the flash object to delete"},
	{nullptr,			INVALID_COMMAND,		nullptr,					nullptr}
};

static const clikeyword_t g_noCommands[] =
{
	{"flash",			CMD_FLASH,				g_noFlashCommands,			"Deletes objects from flash"},
	{"ntp",				CMD_NTP,				nullptr,					"Disables the NTP client"},
	{"ssh",				CMD_SSH,				g_noSshCommands,			"Remove authorized SSH keys"},

	{nullptr,			INVALID_COMMAND,		nullptr,					nullptr}
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// "refresh"

static const clikeyword_t g_refreshCommands[] =
{
	{"<cr>",			OPTIONAL_TOKEN,		nullptr,						"With no arguments, perform a fast refresh"},
	{"full",			CMD_FULL,			nullptr,						"Perform a full refresh"},
	{nullptr,			INVALID_COMMAND,	nullptr,						nullptr}
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// "set"

static const clikeyword_t g_setRegisterValues[] =
{
	{"<value>",			FREEFORM_TOKEN,			nullptr,					"Hexadecimal register value"},

	{nullptr,			INVALID_COMMAND,		nullptr,					nullptr}
};

static const clikeyword_t g_setRegisterCommands[] =
{
	{"<regid>",			FREEFORM_TOKEN,			g_setRegisterValues,		"Hexadecimal register address"},

	{nullptr,			INVALID_COMMAND,		nullptr,					nullptr}
};

static const clikeyword_t g_setMmdRegisterCommands[] =
{
	{"register",		CMD_REGISTER,			g_setRegisterCommands,		"Register within the MMD"},

	{nullptr,			INVALID_COMMAND,		nullptr,					nullptr}
};

static const clikeyword_t g_setMmdCommands[] =
{
	{"<mmdid>",			FREEFORM_TOKEN,			g_setMmdRegisterCommands,	"Hexadecimal MMD index"},

	{nullptr,			INVALID_COMMAND,		nullptr,					nullptr}
};

static const clikeyword_t g_setCommands[] =
{
	{"mmd",				CMD_MMD,				g_setMmdCommands,			"Set MMD registers"},
	{"register",		CMD_REGISTER,			g_setRegisterCommands,		"Set PHY registers"},

	{nullptr,			INVALID_COMMAND,		nullptr,					nullptr}
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// "show"

static const clikeyword_t g_showRegisterCommands[] =
{
	{"<regid>",			FREEFORM_TOKEN,			nullptr,					"Hexadecimal register address"},

	{nullptr,			INVALID_COMMAND,		nullptr,					nullptr}
};

static const clikeyword_t g_showMmdRegisterCommands[] =
{
	{"register",		CMD_REGISTER,			g_showRegisterCommands,		"Register within the MMD"},

	{nullptr,			INVALID_COMMAND,		nullptr,					nullptr}
};

static const clikeyword_t g_showMmdCommands[] =
{
	{"<mmdid>",			FREEFORM_TOKEN,			g_showMmdRegisterCommands,	"Hexadecimal MMD index"},

	{nullptr,			INVALID_COMMAND,		nullptr,					nullptr}
};

static const clikeyword_t g_showArpCommands[] =
{
	{"cache",			CMD_CACHE,				nullptr,				"Show contents of the ARP cache"},
	{nullptr,			INVALID_COMMAND,		nullptr,				nullptr}
};

static const clikeyword_t g_showIpCommands[] =
{
	{"address",			CMD_ADDRESS,			nullptr,				"Show the IPv4 address and subnet mask"},
	{"route",			CMD_ROUTE,				nullptr,				"Show the IPv4 routing table"},
	{nullptr,			INVALID_COMMAND,		nullptr,				nullptr}
};

static const clikeyword_t g_showSshAuthorized[] =
{
	{"keys",			CMD_KEYS,				nullptr,				"Show authorized keys"},
	{nullptr,			INVALID_COMMAND,		nullptr,				nullptr}
};

static const clikeyword_t g_showSshCommands[] =
{
	{"authorized",		CMD_AUTHORIZED,			g_showSshAuthorized,	"Show authorized keys"},
	{"fingerprint",		CMD_FINGERPRINT,		nullptr,				"Show the SSH host key fingerprint (in OpenSSH base64 SHA256 format)"},
	{nullptr,			INVALID_COMMAND,		nullptr,				nullptr}
};

static const clikeyword_t g_showFlashDetailCommands[] =
{
	{"<objname>",		FREEFORM_TOKEN,		nullptr,					"Name of the flash object to display"},
	{nullptr,			INVALID_COMMAND,	nullptr,					nullptr}
};

static const clikeyword_t g_showFlashCommands[] =
{
	{"<cr>",			OPTIONAL_TOKEN,		nullptr,					""},
	{"detail",			CMD_DETAIL,			g_showFlashDetailCommands,	"Show detailed flash object contents"},
	{nullptr,			INVALID_COMMAND,	nullptr,					nullptr}
};

static const clikeyword_t g_showCommands[] =
{
	{"arp",				CMD_ARP,			g_showArpCommands,			"Print ARP information"},
	{"flash",			CMD_FLASH,			g_showFlashCommands,		"Display flash usage and log data"},
	{"hardware",		CMD_HARDWARE,		nullptr,					"Print hardware information"},
	//{"interface",		CMD_INTERFACE,		g_showInterfaceCommands,	"Display interface properties and stats"},*/
	{"ip",				CMD_IP,				g_showIpCommands,			"Print IPv4 information"},
	{"ntp",				CMD_NTP,			nullptr,					"Print NTP information"},
	{"ssh",				CMD_SSH,			g_showSshCommands,			"Print SSH information"},
	//{"temperature",		CMD_TEMPERATURE,	nullptr,					"Display temperature sensor values"},
	{"version",			CMD_VERSION,		nullptr,					"Show firmware / FPGA version"},
	{"mmd",				CMD_MMD,			g_showMmdCommands,			"Read MMD registers"},
	{"register",		CMD_REGISTER,		g_showRegisterCommands,		"Read PHY registers"},
	{nullptr,			INVALID_COMMAND,	nullptr,					nullptr}
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// "ssh"

static const clikeyword_t g_sshCommandsDescription[] =
{
	{"<description>",	FREEFORM_TOKEN,			nullptr,					"Description of key (typically user@host)"},
	{nullptr,			INVALID_COMMAND,		nullptr,					nullptr}
};

static const clikeyword_t g_sshCommandsBlob[] =
{
	{"<blob>",			FREEFORM_TOKEN,			g_sshCommandsDescription,	"Base64 encoded public key blob"},
	{nullptr,			INVALID_COMMAND,		nullptr,					nullptr}
};

static const clikeyword_t g_sshCommandsType[] =
{
	{"ssh-ed25519",		CMD_SSH_ED25519,		g_sshCommandsBlob,			"Ed25519 public key"},
	{nullptr,			INVALID_COMMAND,		nullptr,					nullptr}
};

static const clikeyword_t g_sshCommandsUsername[] =
{
	{"<username>",		FREEFORM_TOKEN,			nullptr,					"SSH username"},
	{nullptr,			INVALID_COMMAND,		nullptr,					nullptr}
};

static const clikeyword_t g_sshCommands[] =
{
	{"key",				CMD_KEY,				g_sshCommandsType,			"Authorize a new SSH public key"},
	{"username",		CMD_USERNAME,			g_sshCommandsUsername,		"Sets the SSH username"},
	{nullptr,			INVALID_COMMAND,		nullptr,					nullptr}
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// "ntp"

static const clikeyword_t g_ntpServerCommands[] =
{
	{"<ip>",			FREEFORM_TOKEN,			nullptr,					"IP address of NTP server to use"},
	{nullptr,			INVALID_COMMAND,		nullptr,					nullptr}
};

static const clikeyword_t g_ntpCommands[] =
{
	{"server",			CMD_SERVER,				g_ntpServerCommands,		"Sets the NTP server to use"},
	{nullptr,			INVALID_COMMAND,		nullptr,					nullptr}
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// zeroize

static const clikeyword_t g_zeroizeCommands[] =
{
	{"all",				FREEFORM_TOKEN,			nullptr,				"Confirm erasing all flash data and return to default state"},
	{nullptr,			INVALID_COMMAND,		nullptr,				nullptr}
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Top level command lists

//Top level commands in root mode
static const clikeyword_t g_rootCommands[] =
{
	//{"clear",		CMD_CLEAR,			g_clearCommands,		"Clear performance counters and other debugging state"},
	{"commit",		CMD_COMMIT,			nullptr,				"Commit volatile config changes to flash memory"},
	//{"debug",		CMD_DEBUG,			g_debugCommands,		"Enable debug output"},
	{"dfu",			CMD_DFU,			nullptr,				"Reboot in DFU mode for firmware updating the main CPU"},
	{"exit",		CMD_EXIT,			nullptr,				"Log out"},
	{"flash",		CMD_FLASH,			g_flashCommands,		"Maintenance operations on flash"},
	{"hostname",	CMD_HOSTNAME,		g_hostnameCommands,		"Change the host name"},
	{"ip",			CMD_IP,				g_ipCommands,			"Configure IP addresses"},
	{"no",			CMD_NO,				g_noCommands,			"Remove or disable features"},
	{"ntp",			CMD_NTP,			g_ntpCommands,			"Configure NTP client"},
	{"refresh",		CMD_REFRESH,		g_refreshCommands,		"Refresh front panel display"},
	{"reload",		CMD_RELOAD,			nullptr,				"Restart the system"},
	{"rollback",	CMD_ROLLBACK,		nullptr,				"Revert changes made since last commit"},
	{"set",			CMD_SET,			g_setCommands,			"Set raw hardware registers"},
	{"show",		CMD_SHOW,			g_showCommands,			"Print information"},
	{"ssh",			CMD_SSH,			g_sshCommands,			"Configure SSH protocol"},
	//{"test",		CMD_TEST,			g_interfaceCommands,	"Run a cable test"},
	{"zeroize",		CMD_ZEROIZE,		g_zeroizeCommands,		"Erase all configuration data and reload"},

	{nullptr,		INVALID_COMMAND,	nullptr,				nullptr}
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Main CLI code

CrossbarCLISessionContext::CrossbarCLISessionContext()
	: CLISessionContext(g_rootCommands)
{
	//cannot LoadHostname or do anything else touching the KVS here because the local console session is a global,
	//and is initialized before App_Init() calls InitKVS()
}

void CrossbarCLISessionContext::PrintPrompt()
{
	if(m_rootCommands == g_rootCommands)
		m_stream->Printf("%s@%s# ", m_username, m_hostname);
	m_stream->Flush();
}

void CrossbarCLISessionContext::LoadHostname()
{
	memset(m_hostname, 0, sizeof(m_hostname));

	//Read hostname, set to default value if not found
	auto hlog = g_kvs->FindObject(hostname_objid);
	if(hlog)
		strncpy(m_hostname, (const char*)g_kvs->MapObject(hlog), std::min((size_t)hlog->m_len, sizeof(m_hostname)-1));
	else
		strncpy(m_hostname, "crossbar", sizeof(m_hostname)-1);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Top level command dispatcher

void CrossbarCLISessionContext::OnExecute()
{
	if(m_rootCommands == g_rootCommands)
		OnExecuteRoot();

	//TODO other modes

	m_stream->Flush();
}

/**
	@brief Execute a command in config mode
 */
void CrossbarCLISessionContext::OnExecuteRoot()
{
	switch(m_command[0].m_commandID)
	{
		/*
		case CMD_CLEAR:
			//for now, 1-2 are always "counters" and "interface" so don't bother checking
			//then 3 will be the interface ID
			OnClearCounters(m_command[3].m_commandID - CMD_G0);
			break;
		*/
		case CMD_COMMIT:
			OnCommit();
			break;
		/*
		case CMD_DEBUG:
			OnDebug();
			break;
		*/

		case CMD_DFU:
			{
				//TODO: require confirmation or something
				RTC::Unlock();
				g_bbram->m_state = STATE_DFU;
				asm("dmb st");
				RTC::Lock();
				Reset();
			}
			break;

		case CMD_EXIT:
			m_stream->Flush();

			//SSH session? Close the socket
			if(m_stream == &m_sshstream)
				m_sshstream.GetServer()->GracefulDisconnect(m_sshstream.GetSessionID(), m_sshstream.GetSocket());

			//Local console? Nothing needed on real hardware (TODO logout)
			else
			{
			}
			break;

		case CMD_FLASH:
			if(m_command[1].m_commandID == CMD_COMPACT)
			{
				if(!g_kvs->Compact())
					g_log(Logger::ERROR, "Compaction failed\n");
			}
			break;

		case CMD_HOSTNAME:
			memcpy(m_hostname, m_command[1].m_text, sizeof(m_hostname)-1);
			m_hostname[sizeof(m_hostname)-1] = '\0';
			break;

		case CMD_IP:
			OnIPCommand();
			break;

		case CMD_NO:
			OnNoCommand();
			break;

		case CMD_NTP:
			if(m_command[1].m_commandID == CMD_SERVER)
				OnNtpServer(m_command[2].m_text);
			break;

		case CMD_RELOAD:
			OnReload();
			break;

		case CMD_REFRESH:
			g_frontSPI->SetCS(0);
			if(m_command[1].m_commandID == CMD_FULL)
				SendFrontPanelByte(FRONT_REFRESH_FULL);
			else
				SendFrontPanelByte(FRONT_REFRESH_FAST);
			SendFrontPanelByte(0x00);	//dummy byte
			g_frontSPI->SetCS(1);
			break;

		case CMD_ROLLBACK:
			OnRollback();
			break;

		case CMD_SET:
			OnSetCommand();
			break;

		case CMD_SHOW:
			OnShowCommand();
			break;

		case CMD_SSH:
			OnSSHCommand();
			break;

		case CMD_ZEROIZE:
			if(!strcmp(m_command[1].m_text, "all"))
				OnZeroize();
			break;

		default:
			m_stream->Printf("Unrecognized command\n");
			break;
	}
}

/*
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// "clear"

void CrossbarCLISessionContext::OnClearCounters(uint8_t interface)
{
	g_fpga->BlockingWrite8(REG_PERF_CLEAR, interface);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// "commit"
*/
void CrossbarCLISessionContext::OnCommit()
{
	//Save hostname and SSH username
	if(!g_kvs->StoreStringObjectIfNecessary(hostname_objid, m_hostname, "crossbar"))
		m_stream->Printf("KVS write error\n");
	if(!g_kvs->StoreStringObjectIfNecessary(g_usernameObjectID, g_sshUsername, g_defaultSshUsername))
		m_stream->Printf("KVS write error\n");

	//Check if we already have the same hostname stored
	auto hlog = g_kvs->FindObject(hostname_objid);
	bool needToStoreHostname = true;
	if(hlog)
	{
		auto oldhost = (const char*)g_kvs->MapObject(hlog);
		if( (strlen(m_hostname) == hlog->m_len) && (!strncmp(m_hostname, oldhost, hlog->m_len)) )
			needToStoreHostname = false;

	}

	//if not found, store it
	if(needToStoreHostname)
	{
		if(!g_kvs->StoreObject(hostname_objid, (uint8_t*)m_hostname, strlen(m_hostname)))
			m_stream->Printf("KVS write error\n");
	}

	//Save SSH authorized key list
	g_keyMgr.CommitToKVS();

	//Save DHCP configuration
	g_dhcpClient->SaveConfigToKVS();
	g_ntpClient->SaveConfigToKVS();

	//Save IP configuration
	if(!g_kvs->StoreObjectIfNecessary<IPv4Address>(g_ipConfig.m_address, g_defaultIP, "ip.address"))
		m_stream->Printf("KVS write error\n");
	if(!g_kvs->StoreObjectIfNecessary<IPv4Address>(g_ipConfig.m_netmask, g_defaultNetmask, "ip.netmask"))
		m_stream->Printf("KVS write error\n");
	if(!g_kvs->StoreObjectIfNecessary<IPv4Address>(g_ipConfig.m_broadcast, g_defaultBroadcast, "ip.broadcast"))
		m_stream->Printf("KVS write error\n");
	if(!g_kvs->StoreObjectIfNecessary<IPv4Address>(g_ipConfig.m_gateway, g_defaultGateway, "ip.gateway"))
		m_stream->Printf("KVS write error\n");
}

/*
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// "debug"

void CrossbarCLISessionContext::OnDebug()
{
	switch(m_command[1].m_commandID)
	{
		case CMD_TEMPERATURE:
			g_debugEnv = true;
			break;

		default:
			m_stream->Printf("Unrecognized command\n");
			break;
	}
}
*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// "ip"

bool CrossbarCLISessionContext::ParseIPAddress(const char* addr, IPv4Address& ip)
{
	int len = strlen(addr);

	int nfield = 0;
	unsigned int fields[4] = {0};

	//Parse
	for(int i=0; i<len; i++)
	{
		//Dot = move to next field
		if( (addr[i] == '.') && (nfield < 3) )
			nfield ++;

		//Digit = update current field
		else if(isdigit(addr[i]))
			fields[nfield] = (fields[nfield] * 10) + (addr[i] - '0');

		else
			return false;
	}

	//Validate
	if(nfield != 3)
		return false;
	for(int i=0; i<4; i++)
	{
		if(fields[i] > 255)
			return false;
	}

	//Set the IP
	for(int i=0; i<4; i++)
		ip.m_octets[i] = fields[i];
	return true;
}

bool CrossbarCLISessionContext::ParseIPAddressWithSubnet(const char* addr, IPv4Address& ip, uint32_t& mask)
{
	int len = strlen(addr);

	int nfield = 0;	//0-3 = IP, 4 = netmask
	unsigned int fields[5] = {0};

	//Parse
	for(int i=0; i<len; i++)
	{
		//Dot = move to next field
		if( (addr[i] == '.') && (nfield < 3) )
			nfield ++;

		//Slash = move to netmask
		else if( (addr[i] == '/') && (nfield == 3) )
			nfield ++;

		//Digit = update current field
		else if(isdigit(addr[i]))
			fields[nfield] = (fields[nfield] * 10) + (addr[i] - '0');

		else
			return false;
	}

	//Validate
	if(nfield != 4)
		return false;
	for(int i=0; i<4; i++)
	{
		if(fields[i] > 255)
			return false;
	}
	if( (fields[4] > 32) || (fields[4] == 0) )
		return false;

	//Set the IP
	for(int i=0; i<4; i++)
		ip.m_octets[i] = fields[i];

	mask = 0xffffffff << (32 - fields[4]);
	return true;
}

void CrossbarCLISessionContext::OnIPAddress(const char* addr)
{
	//Parse the base IP address
	uint32_t mask = 0;
	if(!ParseIPAddressWithSubnet(addr, g_ipConfig.m_address, mask))
	{
		m_stream->Printf("Usage: ip address x.x.x.x/yy\n");
		return;
	}

	//Calculate the netmask
	g_ipConfig.m_netmask.m_octets[0] = (mask >> 24) & 0xff;
	g_ipConfig.m_netmask.m_octets[1] = (mask >> 16) & 0xff;
	g_ipConfig.m_netmask.m_octets[2] = (mask >> 8) & 0xff;
	g_ipConfig.m_netmask.m_octets[3] = (mask >> 0) & 0xff;

	//Calculate the broadcast address
	for(int i=0; i<4; i++)
		g_ipConfig.m_broadcast.m_octets[i] = g_ipConfig.m_address.m_octets[i] | ~g_ipConfig.m_netmask.m_octets[i];
}

void CrossbarCLISessionContext::OnIPGateway(const char* gw)
{
	if(!ParseIPAddress(gw, g_ipConfig.m_gateway))
		m_stream->Printf("Usage: ip gateway x.x.x.x\n");
}

void CrossbarCLISessionContext::OnIPCommand()
{
	switch(m_command[1].m_commandID)
	{
		case CMD_ADDRESS:
			if(m_command[2].m_commandID == CMD_DHCP)
			{
				g_usingDHCP = true;
				g_dhcpClient->Enable();
			}
			else
			{
				g_usingDHCP = false;
				g_dhcpClient->Disable();
				OnIPAddress(m_command[2].m_text);
			}
			break;

		case CMD_GATEWAY:
			OnIPGateway(m_command[2].m_text);
			break;

		default:
			m_stream->Printf("Unrecognized command\n");
			break;
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// "ntp"

void CrossbarCLISessionContext::OnNtpServer(const char* addr)
{
	//Parse the base IP address
	IPv4Address iaddr;
	if(!ParseIPAddress(addr, iaddr))
	{
		m_stream->Printf("Usage: ntp server x.x.x.x\n");
		return;
	}

	g_ntpClient->Enable();
	g_ntpClient->SetServerAddress(iaddr);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// "no"

void CrossbarCLISessionContext::OnNoSSHKeyCommand()
{
	g_keyMgr.RemovePublicKey(atoi(m_command[3].m_text));
}

void CrossbarCLISessionContext::OnNoSSHCommand()
{
	switch(m_command[2].m_commandID)
	{
		case CMD_KEY:
			OnNoSSHKeyCommand();
			break;

		default:
			break;
	}
}

/**
	@brief "no flash key" - deletes a key from flash
 */
void CrossbarCLISessionContext::OnNoFlashCommand()
{
	const char* key = m_command[2].m_text;

	auto hlog = g_kvs->FindObject(key);
	if(hlog)
	{
		if(!g_kvs->StoreObject(key, nullptr, 0))
			m_stream->Printf("KVS write error\n");
		else
			m_stream->Printf("Object \"%s\" deleted\n", key);
	}
	else
		m_stream->Printf("Object \"%s\" not found, could not delete\n", key);
}

void CrossbarCLISessionContext::OnNoCommand()
{
	switch(m_command[1].m_commandID)
	{
		case CMD_FLASH:
			OnNoFlashCommand();
			break;

		case CMD_NTP:
			g_ntpClient->Disable();
			break;

		case CMD_SSH:
			OnNoSSHCommand();
			break;

		default:
			break;
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// "reload"

void CrossbarCLISessionContext::OnReload()
{
	//TODO: do this through the supervisor to do a whole-system reset instead of just rebooting the MCU

	g_log("Reload requested\n");
	Reset();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// "rollback"

/**
	@brief Load all of our configuration from the KVS, discarding any recent changes made in the CLI
 */
void CrossbarCLISessionContext::OnRollback()
{
	g_keyMgr.LoadFromKVS(false);

	g_dhcpClient->LoadConfigFromKVS();
	g_ntpClient->LoadConfigFromKVS();
	ConfigureIP();
	LoadHostname();
	g_sshd->LoadUsername();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// "set"

void CrossbarCLISessionContext::OnSetCommand()
{
	switch(m_command[1].m_commandID)
	{
		case CMD_MMD:
			OnSetMmdRegister();
			break;

		case CMD_REGISTER:
			OnSetRegister();
			break;
	}
}

void CrossbarCLISessionContext::OnSetRegister()
{
	int regid = strtol(m_command[2].m_text, nullptr, 16);
	int value = strtol(m_command[3].m_text, nullptr, 16);
	g_mgmtPhy.WriteRegister(regid, value);
	m_stream->Printf("Set register 0x%02x to 0x%04x\n", regid, value);
}

void CrossbarCLISessionContext::OnSetMmdRegister()
{
	int mmd = strtol(m_command[2].m_text, nullptr, 16);
	int regid = strtol(m_command[4].m_text, nullptr, 16);
	auto value = strtol(m_command[5].m_text, nullptr, 16);
	g_mgmtPhy.WriteExtendedRegister(mmd, regid, value);
	m_stream->Printf("Set MMD %02x register 0x%04x to 0x%04x\n", mmd, regid, value);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// "show"

void CrossbarCLISessionContext::OnShowCommand()
{
	switch(m_command[1].m_commandID)
	{

		case CMD_ARP:
			switch(m_command[2].m_commandID)
			{
				case CMD_CACHE:
					OnShowARPCache();
					break;

				default:
					break;
			}
			break;

		case CMD_FLASH:
			OnShowFlash();
			break;
		/*
		case CMD_INTERFACE:
			OnShowInterfaceCommand();
			break;
		*/
		case CMD_IP:
			switch(m_command[2].m_commandID)
			{
				case CMD_ADDRESS:
					OnShowIPAddress();
					break;

				case CMD_ROUTE:
					OnShowIPRoute();
					break;

				default:
					break;
			}
			break;

		case CMD_MMD:
			OnShowMMDRegister();
			break;

		case CMD_REGISTER:
			OnShowRegister();
			break;

		case CMD_NTP:
			OnShowNtp();
			break;

		case CMD_SSH:
			switch(m_command[2].m_commandID)
			{
				case CMD_AUTHORIZED:
					OnShowSSHKeys();
					break;

				case CMD_FINGERPRINT:
					OnShowSSHFingerprint();
					break;

				default:
					break;
			}
			break;

		/*case CMD_TEMPERATURE:
			OnShowTemperature();
			break;*/

		case CMD_VERSION:
			OnShowVersion();
			break;

		default:
			m_stream->Printf("Unrecognized command\n");
			break;
	}
}

void CrossbarCLISessionContext::OnShowARPCache()
{
	auto cache = g_ethProtocol->GetARP()->GetCache();

	uint32_t ways = cache->GetWays();
	uint32_t lines = cache->GetLines();
	m_stream->Printf("ARP cache is %d ways of %d lines, %d spaces total\n", ways, lines, ways*lines);

	m_stream->Printf("Expiration  HWaddress           Address\n");

	for(uint32_t i=0; i<ways; i++)
	{
		auto way = cache->GetWay(i);
		for(uint32_t j=0; j<lines; j++)
		{
			auto& line = way->m_lines[j];
			if(line.m_valid)
			{
				m_stream->Printf("%10d  %02x:%02x:%02x:%02x:%02x:%02x   %d.%d.%d.%d\n",
					line.m_lifetime,
					line.m_mac[0], line.m_mac[1], line.m_mac[2], line.m_mac[3], line.m_mac[4], line.m_mac[5],
					line.m_ip.m_octets[0], line.m_ip.m_octets[1], line.m_ip.m_octets[2], line.m_ip.m_octets[3]
				);
			}
		}
	}
}

void CrossbarCLISessionContext::OnShowFlash()
{
	//No details requested? Show root dir listing
	if(m_command[2].m_commandID == OPTIONAL_TOKEN)
	{
		//Print info about the flash memory in general
		m_stream->Printf("Flash configuration storage is 2 banks of %d kB\n", g_kvs->GetBlockSize() / 1024);
		if(g_kvs->IsLeftBankActive())
			m_stream->Printf("    Active bank: Left\n");
		else
			m_stream->Printf("    Active bank: Right\n");
		m_stream->Printf("    Header version: %d\n", g_kvs->GetBankHeaderVersion());
		m_stream->Printf("    Log area:    %6d / %6d entries free (%d %%)\n",
			g_kvs->GetFreeLogEntries(),
			g_kvs->GetLogCapacity(),
			g_kvs->GetFreeLogEntries()*100 / g_kvs->GetLogCapacity());
		m_stream->Printf("    Data area:   %6d / %6d kB free      (%d %%)\n",
			g_kvs->GetFreeDataSpace() / 1024,
			g_kvs->GetDataCapacity() / 1024,
			g_kvs->GetFreeDataSpace() * 100 / g_kvs->GetDataCapacity());

		//Dump directory listing
		const uint32_t nmax = 256;
		KVSListEntry list[nmax];
		uint32_t nfound = g_kvs->EnumObjects(list, nmax);
		m_stream->Printf("    Objects:\n");
		m_stream->Printf("        Key                               Size  Revisions\n");
		int size = 0;
		for(uint32_t i=0; i<nfound; i++)
		{
			//If the object has no content, don't show it (it's been deleted)
			if(list[i].size == 0)
				continue;

			//Is this a group?
			auto dotpos = strchr(list[i].key, '.');
			if(dotpos != nullptr)
			{
				//Get the name of the group
				char groupname[KVS_NAMELEN+1] = {0};
				auto grouplen = dotpos + 1 - list[i].key;
				memcpy(groupname, list[i].key, grouplen);

				//If we have a previous key with the same group, we're not the first
				bool first = true;
				if(i > 0)
				{
					if(memcmp(list[i-1].key, groupname, grouplen) == 0)
						first = false;
				}

				//Do we have a subsequent key with the same group?
				bool next = true;
				if(i+1 < nfound)
				{
					if(memcmp(list[i+1].key, groupname, grouplen) != 0)
						next = false;
				}
				else
					next = false;

				//Trim off the leading dot in the group
				groupname[grouplen-1] = '\0';

				//Beginning of a group (with more than one key)? Add the heading
				if(first && next)
					m_stream->Printf("        %-32s\n", groupname);

				//If in a group with >1 item, print the actual entry
				if(next || !first)
				{
					//Print the tree node
					if(next)
						m_stream->Printf("        ├── %-28s %5d  %d\n", list[i].key + grouplen, list[i].size, list[i].revs);
					else
						m_stream->Printf("        └── %-28s %5d  %d\n", list[i].key + grouplen, list[i].size, list[i].revs);
				}

				//Single entry group, normal print
				else
					m_stream->Printf("        %-32s %5d  %d\n", list[i].key, list[i].size, list[i].revs);
			}

			//No, not in a group
			else
				m_stream->Printf("        %-32s %5d  %d\n", list[i].key, list[i].size, list[i].revs);

			//Record total data size
			size += list[i].size;
		}
		m_stream->Printf("    %d objects total (%d.%02d kB)\n",
			nfound,
			size/1024, (size % 1024) * 100 / 1024);
	}

	//Showing details of a single object
	else
	{
		auto hlog = g_kvs->FindObject(m_command[3].m_text);
		if(!hlog)
		{
			m_stream->Printf("Object not found\n");
			return;
		}

		//TODO: show previous versions too?
		m_stream->Printf("Object \"%s\":\n", m_command[3].m_text);
		{
			m_stream->Printf("    Start:  0x%08x\n", hlog->m_start);
			m_stream->Printf("    Length: 0x%08x\n", hlog->m_len);
			m_stream->Printf("    CRC32:  0x%08x\n", hlog->m_crc);
		}

		auto pdata = g_kvs->MapObject(hlog);

		//TODO: make this a dedicated hexdump routine
		const uint32_t linelen = 16;
		for(uint32_t i=0; i<hlog->m_len; i += linelen)
		{
			m_stream->Printf("%04x   ", i);

			//Print hex
			for(uint32_t j=0; j<linelen; j++)
			{
				//Pad with spaces so we get good alignment on the end of the block
				if(i+j >= hlog->m_len)
					m_stream->Printf("   ");

				else
					m_stream->Printf("%02x ", pdata[i+j]);
			}

			m_stream->Printf("  ");

			//Print ASCII
			for(uint32_t j=0; j<linelen; j++)
			{
				//No padding needed here
				if(i+j >= hlog->m_len)
					break;

				else if(isprint(pdata[i+j]))
					m_stream->Printf("%c", pdata[i+j]);
				else
					m_stream->Printf(".");
			}

			m_stream->Printf("\n");
		}
	}
}

void CrossbarCLISessionContext::OnShowHardware()
{
	uint16_t rev = DBGMCU.IDCODE >> 16;
	uint16_t device = DBGMCU.IDCODE & 0xfff;

	if(device == 0x483)
	{
		//Look up the stepping number
		const char* srev = nullptr;
		switch(rev)
		{
			case 0x1000:
				srev = "A";
				break;

			case 0x1001:
				srev = "Z";
				break;

			default:
				srev = "(unknown)";
		}

		uint8_t pkg = SYSCFG.PKGR;
		const char* package = "";
		switch(pkg)
		{
			case 0:
				package = "VQFPN68 (industrial)";
				break;
			case 1:
				package = "LQFP100/TFBGA100 (legacy)";
				break;
			case 2:
				package = "LQFP100 (industrial)";
				break;
			case 3:
				package = "TFBGA100 (industrial)";
				break;
			case 4:
				package = "WLCSP115 (industrial)";
				break;
			case 5:
				package = "LQFP144 (legacy)";
				break;
			case 6:
				package = "UFBGA144 (legacy)";
				break;
			case 7:
				package = "LQFP144 (industrial)";
				break;
			case 8:
				package = "UFBGA169 (industrial)";
				break;
			case 9:
				package = "UFBGA176+25 (industrial)";
				break;
			case 10:
				package = "LQFP176 (industrial)";
				break;
			default:
				package = "unknown package";
				break;
		}

		m_stream->Printf("STM32%c%c%c%c stepping %s, %s\n",
			(L_ID >> 24) & 0xff,
			(L_ID >> 16) & 0xff,
			(L_ID >> 8) & 0xff,
			(L_ID >> 0) & 0xff,
			srev,
			package
			);
		m_stream->Printf("564 kB total SRAM, 128 kB DTCM, up to 256 kB ITCM, 4 kB backup SRAM\n");
		m_stream->Printf("%d kB Flash\n", F_ID);

		//U_ID fields documented in 45.1 of STM32 programming manual
		uint16_t waferX = U_ID[0] >> 16;
		uint16_t waferY = U_ID[0] & 0xffff;
		uint8_t waferNum = U_ID[1] & 0xff;
		char waferLot[8] =
		{
			static_cast<char>((U_ID[1] >> 24) & 0xff),
			static_cast<char>((U_ID[1] >> 16) & 0xff),
			static_cast<char>((U_ID[1] >> 8) & 0xff),
			static_cast<char>((U_ID[2] >> 24) & 0xff),
			static_cast<char>((U_ID[2] >> 16) & 0xff),
			static_cast<char>((U_ID[2] >> 8) & 0xff),
			static_cast<char>((U_ID[2] >> 0) & 0xff),
			'\0'
		};
		m_stream->Printf("Lot %s, wafer %d, die (%d, %d)\n", waferLot, waferNum, waferX, waferY);
	}
	else
		m_stream->Printf("Unknown device (0x%06x)\n", device);

	//Print CPU info
	if( (SCB.CPUID & 0xff00fff0) == 0x4100c270 )
	{
		m_stream->Printf("ARM Cortex-M7 r%dp%d\n", (SCB.CPUID >> 20) & 0xf, (SCB.CPUID & 0xf));
		if(SCB.CLIDR & 2)
		{
			m_stream->Printf("    L1 data cache present\n");
			SCB.CCSELR = 0;

			int sets = ((SCB.CCSIDR >> 13) & 0x7fff) + 1;
			int ways = ((SCB.CCSIDR >> 3) & 0x3ff) + 1;
			int words = 1 << ((SCB.CCSIDR & 3) + 2);
			int total = (sets * ways * words * 4) / 1024;
			m_stream->Printf("        %d sets, %d ways, %d words per line, %d kB total\n",
				sets, ways, words, total);
		}
		if(SCB.CLIDR & 1)
		{
			m_stream->Printf("    L1 instruction cache present\n");
			SCB.CCSELR = 1;

			int sets = ((SCB.CCSIDR >> 13) & 0x7fff) + 1;
			int ways = ((SCB.CCSIDR >> 3) & 0x3ff) + 1;
			int words = 1 << ((SCB.CCSIDR & 3) + 2);
			int total = (sets * ways * words * 4) / 1024;
			m_stream->Printf("        %d sets, %d ways, %d words per line, %d kB total\n",
				sets, ways, words, total);
		}
	}
	else
		m_stream->Printf("Unknown CPU (0x%08x)\n", SCB.CPUID);

	m_stream->Printf("Ethernet MAC address is %02x:%02x:%02x:%02x:%02x:%02x\n",
		g_macAddress[0], g_macAddress[1], g_macAddress[2], g_macAddress[3], g_macAddress[4], g_macAddress[5]);

}
/*
void CrossbarCLISessionContext::OnShowInterfaceCommand()
{
	//Interface number?
	if(m_command[2].m_commandID >= CMD_G0)
	{
		//Look at the next argument
		//Generally something like "show int g3 count"
		switch(m_command[3].m_commandID)
		{
			case CMD_COUNTERS:
				OnShowInterfaceCounters(m_command[2].m_commandID - CMD_G0);
				break;

			default:
				m_stream->Printf("Unrecognized command\n");
				break;
		}

		return;
	}

	switch(m_command[2].m_commandID)
	{
		case CMD_STATUS:
			OnShowInterfaceStatus();
			break;

		default:
			m_stream->Printf("Unrecognized command\n");
			break;
	}
}


void CrossbarCLISessionContext::OnShowInterfaceStatus()
{
	m_stream->Printf("---------------------------------------------------------------------------------------------------------\n");
	m_stream->Printf("Port     Name                             Status            Vlan     Duplex    Speed                 Type\n");
	m_stream->Printf("---------------------------------------------------------------------------------------------------------\n");

	//TODO: refresh interface status from hardware or something
	for(int i=0; i<NUM_PORTS; i++)
	{
		const char* portType = "10/100/1000baseT";
		if(i == UPLINK_PORT)
			portType = "10Gbase-SR";

		if(i == MGMT_PORT)
		{
			m_stream->Printf("%-5s    %-32s %-15s %6s %10s  %7s %20s\n",
				g_interfaceNames[i],
				g_interfaceDescriptions[i],
				g_linkStateNames[g_linkState[i]],
				"(none)",
				"full",
				g_linkSpeedNames[g_linkSpeed[i]],
				portType);
		}
		else
		{
			m_stream->Printf("%-5s    %-32s %-15s %6d %10s  %7s %20s\n",
				g_interfaceNames[i],
				g_interfaceDescriptions[i],
				g_linkStateNames[g_linkState[i]],
				g_portVlans[i],
				"full",
				g_linkSpeedNames[g_linkSpeed[i]],
				portType);
		}
	}
}
*/
void CrossbarCLISessionContext::OnShowIPAddress()
{
	m_stream->Printf("IPv4 address: %d.%d.%d.%d\n",
		g_ipConfig.m_address.m_octets[0],
		g_ipConfig.m_address.m_octets[1],
		g_ipConfig.m_address.m_octets[2],
		g_ipConfig.m_address.m_octets[3]
	);

	m_stream->Printf("Subnet mask:  %d.%d.%d.%d\n",
		g_ipConfig.m_netmask.m_octets[0],
		g_ipConfig.m_netmask.m_octets[1],
		g_ipConfig.m_netmask.m_octets[2],
		g_ipConfig.m_netmask.m_octets[3]
	);

	m_stream->Printf("Broadcast:    %d.%d.%d.%d\n",
		g_ipConfig.m_broadcast.m_octets[0],
		g_ipConfig.m_broadcast.m_octets[1],
		g_ipConfig.m_broadcast.m_octets[2],
		g_ipConfig.m_broadcast.m_octets[3]
	);
}

void CrossbarCLISessionContext::OnShowIPRoute()
{
	m_stream->Printf("IPv4 routing table\n");
	m_stream->Printf("Destination     Gateway\n");
	m_stream->Printf("0.0.0.0         %d.%d.%d.%d\n",
		g_ipConfig.m_gateway.m_octets[0],
		g_ipConfig.m_gateway.m_octets[1],
		g_ipConfig.m_gateway.m_octets[2],
		g_ipConfig.m_gateway.m_octets[3]);
}

void CrossbarCLISessionContext::OnShowMMDRegister()
{
	int mmd = strtol(m_command[2].m_text, nullptr, 16);
	int regid = strtol(m_command[4].m_text, nullptr, 16);
	auto value = g_mgmtPhy.ReadExtendedRegister(mmd, regid);

	m_stream->Printf("MMD %02x register 0x%04x = 0x%04x\n", mmd, regid, value);
}

void CrossbarCLISessionContext::OnShowNtp()
{
	if(g_ntpClient->IsEnabled())
	{
		m_stream->Printf("NTP client enabled\n");
		auto ip = g_ntpClient->GetServerAddress();

		if(g_ntpClient->IsSynchronized())
		{
			tm synctime;
			uint16_t syncsub;
			g_ntpClient->GetLastSync(synctime, syncsub);

			m_stream->Printf("Last synchronized to server %d.%d.%d.%d at %04d-%02d-%02dT%02d:%02d:%02d.%04d\n",
				ip.m_octets[0], ip.m_octets[1], ip.m_octets[2], ip.m_octets[3],
				synctime.tm_year + 1900,
				synctime.tm_mon+1,
				synctime.tm_mday,
				synctime.tm_hour,
				synctime.tm_min,
				synctime.tm_sec,
				syncsub);
		}
		else
		{
			m_stream->Printf("Using server %d.%d.%d.%d (not currently synchronized)\n",
				ip.m_octets[0], ip.m_octets[1], ip.m_octets[2], ip.m_octets[3] );
		}

	}
	else
		m_stream->Printf("NTP client disabled\n");
}

void CrossbarCLISessionContext::OnShowRegister()
{
	int regid = strtol(m_command[2].m_text, nullptr, 16);
	auto value = g_mgmtPhy.ReadRegister(regid);

	m_stream->Printf("Register 0x%02x = 0x%04x\n", regid, value);
}

void CrossbarCLISessionContext::OnShowSSHKeys()
{
	m_stream->Printf("Authorized keys:\n");
	m_stream->Printf("Slot  Nickname                        Fingerprint\n");

	AcceleratedCryptoEngine tmp;
	char fingerprint[64];

	for(int i=0; i<MAX_SSH_KEYS; i++)
	{
		if(g_keyMgr.m_authorizedKeys[i].m_nickname[0] != '\0')
		{
			tmp.GetKeyFingerprint(fingerprint, sizeof(fingerprint), g_keyMgr.m_authorizedKeys[i].m_pubkey);
			m_stream->Printf("%2d    %-30s  SHA256:%s\n",
				i,
				g_keyMgr.m_authorizedKeys[i].m_nickname,
				fingerprint);
		}
	}
}

void CrossbarCLISessionContext::OnShowSSHFingerprint()
{
	char buf[64] = {0};
	AcceleratedCryptoEngine tmp;
	tmp.GetHostKeyFingerprint(buf, sizeof(buf));
	m_stream->Printf("ED25519 key fingerprint is SHA256:%s.\n", buf);
}
/*
void CrossbarCLISessionContext::OnShowTemperature()
{
	//Read fans
	for(uint8_t i=0; i<2; i++)
	{
		auto rpm = GetFanRPM(i);
		if(rpm == 0)
			m_stream->Printf("Fan %d:                                 STOPPED\n", i, rpm);
		else
			m_stream->Printf("Fan %d:                                 %d RPM\n", i, rpm);
	}

	//Read I2C temp sensors
	for(uint8_t i=0; i<4; i++)
	{
		auto addr = g_tempSensorAddrs[i];
		auto temp = ReadThermalSensor(addr);
		m_stream->Printf("Temp 0x%02x (%25s): %d.%02d C\n",
			addr,
			g_tempSensorNames[i],
			(temp >> 8),
			static_cast<int>(((temp & 0xff) / 256.0) * 100));
	}

	//Read SFP+ temp sensor (TODO: only if optic is inserted)
	auto temp = GetSFPTemperature();
	m_stream->Printf("SFP+ optic:                            %2d.%02d C\n",
		(temp >> 8), static_cast<int>(((temp & 0xff) / 256.0) * 100));

	//Read VSC8512 PHY temperature
	temp = GetVSC8512Temperature();
	m_stream->Printf("VSC8512:                               %2d.%02d C\n",
		(temp >> 8), static_cast<int>(((temp & 0xff) / 256.0) * 100));

	//Read FPGA XADC temperature
	temp = GetFPGATemperature();
	m_stream->Printf("FPGA:                                  %2d.%02d C\n",
		(temp >> 8), static_cast<int>(((temp & 0xff) / 256.0) * 100));

	//Read MCU DTS temperature
	temp = g_dts->GetTemperature();
	m_stream->Printf("MCU:                                   %2d.%02d C\n",
		(temp >> 8), static_cast<int>(((temp & 0xff) / 256.0) * 100));
}*/

void CrossbarCLISessionContext::OnShowVersion()
{
	m_stream->Printf("Trigger crossbar v0.1\n");
	m_stream->Printf("by Andrew D. Zonenberg\n");
	m_stream->Printf("\n");
	m_stream->Printf("This system is open hardware! Board design files and firmware/gateware source code are at:\n");
	m_stream->Printf("https://github.com/azonenberg/triggercrossbar\n");
	m_stream->Printf("\n");
	m_stream->Printf("Firmware compiled at %s on %s\n", __TIME__, __DATE__);
	#ifdef __GNUC__
	m_stream->Printf("Compiler: g++ %s\n", __VERSION__);
	m_stream->Printf("CLI source code last modified: %s\n", __TIMESTAMP__);
	#endif
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// "ssh"

void CrossbarCLISessionContext::OnSSHCommand()
{
	switch(m_command[1].m_commandID)
	{
		case CMD_KEY:
			OnSSHKey();
			break;

		case CMD_USERNAME:
			//yes this can truncate, we accept that
			#pragma GCC diagnostic push
			#pragma GCC diagnostic ignored "-Wstringop-truncation"
			strncpy(g_sshUsername, m_command[2].m_text, sizeof(g_sshUsername)-1);
			#pragma GCC diagnostic pop
			break;

		default:
			m_stream->Printf("Unrecognized command\n");
			break;
	}
}

void CrossbarCLISessionContext::OnSSHKey()
{
	g_keyMgr.AddPublicKey(m_command[2].m_text, m_command[3].m_text, m_command[4].m_text);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// "zeroize"

void CrossbarCLISessionContext::OnZeroize()
{
	g_kvs->WipeAll();
	OnReload();
}
