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

static const char* hostname_objid = "hostname";

//List of all valid commands
enum cmdid_t
{
	/*CMD_10,
	CMD_100,
	CMD_1000,*/
	CMD_ADDRESS,
	/*CMD_AUTO,
	CMD_AUTONEGOTIATION,*/
	CMD_ARP,
	CMD_CACHE,
	/*CMD_CLEAR,
	CMD_COMMIT,
	CMD_COUNTERS,
	CMD_CROSSOVER,
	CMD_DESCRIPTION,*/
	CMD_DETAIL,
	/*CMD_DEBUG,
	CMD_DISTORTION,
	CMD_END,*/
	CMD_EXIT,
	CMD_FINGERPRINT,
	CMD_FLASH,
	//CMD_GATEWAY,
	CMD_HARDWARE,
	/*CMD_HOSTNAME,
	CMD_INTERFACE,*/
	CMD_IP,
	/*CMD_JITTER,
	CMD_MASTER,
	CMD_MDI,
	CMD_MLT3,*/
	CMD_MMD,
	/*CMD_MODE,
	CMD_NO,
	CMD_PREFER,
	CMD_PULSE4,
	CMD_PULSE64,
	CMD_RELOAD,*/
	CMD_REGISTER,
	//CMD_ROLLBACK,
	CMD_ROUTE,
	CMD_SET,
	CMD_SHOW,
	/*CMD_SLAVE,
	CMD_SPEED,*/
	CMD_SSH,/*
	CMD_STRAIGHT,
	CMD_STATUS,
	CMD_TEMPERATURE,
	CMD_TEST,
	CMD_TESTPATTERN,*/
	CMD_VERSION,/*
	CMD_VLAN,
	CMD_WAVEFORM_TEST,
	CMD_ZEROIZE,

	CMD_XG0,
	CMD_MGMT0
	*/
};
/*
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// "interface"

// Out of alphabetical order so it can be referred to by other commands operating on interfaces to avoid duplication
static const clikeyword_t g_interfaceCommands[] =
{
	{"mgmt0",		CMD_MGMT0,			nullptr,	"Management0"},
	{"xg0",			CMD_XG0,			nullptr,	"10Gigabit0"},

	{nullptr,		INVALID_COMMAND,	nullptr,	nullptr}
};

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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// "description"

static const clikeyword_t g_descriptionCommands[] =
{
	{"<string>",	TEXT_TOKEN,			nullptr,	"New description for the port"},
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
// "mdi"

static const clikeyword_t g_mdiCommands[] =
{
	{"auto",			CMD_AUTO,				nullptr,					"Auto MDI-X"},
	{"crossover",		CMD_CROSSOVER,			nullptr,					"MDI-X mode"},
	{"straight",		CMD_STRAIGHT,			nullptr,					"MDI mode"},
	{nullptr,			INVALID_COMMAND,		nullptr,					nullptr}
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// "mode"

static const clikeyword_t g_modeCommands[] =
{
	{"master",			CMD_MASTER,				nullptr,					"Master mode"},
	{"slave",			CMD_SLAVE,				nullptr,					"Slave mode"},
	{nullptr,			INVALID_COMMAND,		nullptr,					nullptr}
};

static const clikeyword_t g_preferModeCommands[] =
{
	{"prefer",			CMD_PREFER,				g_modeCommands,				"Specify preference"},
	{nullptr,			INVALID_COMMAND,		nullptr,					nullptr}
};

static const clikeyword_t g_allModeCommands[] =
{
	{"auto",			CMD_AUTO,				g_preferModeCommands,		"Negotiate master or slave mode"},
	{"master",			CMD_MASTER,				nullptr,					"Master mode"},
	{"slave",			CMD_SLAVE,				nullptr,					"Slave mode"},
	{nullptr,			INVALID_COMMAND,		nullptr,					nullptr}
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// "speed"

static const clikeyword_t g_interfaceSpeedCommands[] =
{
	{"10",				CMD_10,					nullptr,					"10baseT"},
	{"100",				CMD_100,				nullptr,					"100baseTX"},
	{"1000",			CMD_1000,				nullptr,					"1000baseT"},
	{nullptr,			INVALID_COMMAND,		nullptr,					nullptr}
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// "no" (interface mode)

static const clikeyword_t g_interfaceNoCommands[] =
{
	{"autonegotiation",	CMD_AUTONEGOTIATION,	nullptr,					"Disable autonegotiation"},
	{"speed",			CMD_SPEED,				g_interfaceSpeedCommands,	"Turn off advertisement of a specific speed"},
	{"testpattern",		CMD_TESTPATTERN,		nullptr,					"Stop sending test patterns"},

	{nullptr,			INVALID_COMMAND,		nullptr,					nullptr}
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// "set"
*/
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

static const clikeyword_t g_showSshCommands[] =
{
	{"fingerprint",		CMD_FINGERPRINT,		nullptr,				"Show the SSH host key fingerprint (in OpenSSH base64 SHA256 format)"},
	{nullptr,			INVALID_COMMAND,		nullptr,				nullptr}
};
/*
static const clikeyword_t g_showIntSuffixCommands[] =
{
	//{"<cr>",			OPTIONAL_TOKEN,		nullptr,	""},
	{"counters",		CMD_COUNTERS,		nullptr,	"Show interface performance counters"},
	{nullptr,			INVALID_COMMAND,	nullptr,	nullptr}
};

static const clikeyword_t g_showInterfaceCommands[] =
{
	{"mgmt0",			CMD_MGMT0,			g_showIntSuffixCommands,	"Management0"},
	{"xg0",				CMD_XG0,			g_showIntSuffixCommands,	"10Gigabit0"},

	{"status",			CMD_STATUS,			nullptr,					"Display summary of all network interfaces"},
	{nullptr,			INVALID_COMMAND,	nullptr,					nullptr}
};
*/
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
	{"ssh",				CMD_SSH,			g_showSshCommands,			"Print SSH information"},
	//{"temperature",		CMD_TEMPERATURE,	nullptr,					"Display temperature sensor values"},
	{"version",			CMD_VERSION,		nullptr,					"Show firmware / FPGA version"},
	{"mmd",				CMD_MMD,			g_showMmdCommands,			"Read MMD registers"},
	{"register",		CMD_REGISTER,		g_showRegisterCommands,		"Read PHY registers"},
	{nullptr,			INVALID_COMMAND,	nullptr,					nullptr}
};
/*
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// "show" (interface mode)

static const clikeyword_t g_interfaceShowCommands[] =
{
	{"arp",				CMD_ARP,			g_showArpCommands,			"Print ARP information"},
	{"flash",			CMD_FLASH,			g_showFlashCommands,		"Display flash usage and log data"},
	{"hardware",		CMD_HARDWARE,		nullptr,					"Print hardware information"},
	{"interface",		CMD_INTERFACE,		g_showInterfaceCommands,	"Display interface properties and stats"},
	{"ip",				CMD_IP,				g_showIpCommands,			"Print IPv4 information"},
		{"ssh",				CMD_SSH,			g_showSshCommands,			"Print SSH information"},
	{"version",			CMD_VERSION,		nullptr,					"Show firmware / FPGA version"},
	{nullptr,			INVALID_COMMAND,	nullptr,					nullptr}
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// "testpattern"

static const clikeyword_t g_testpatternCommands[] =
{
	{"distortion",		CMD_DISTORTION,			nullptr,					"Distortion test (mode 4)"},
	{"jitter",			CMD_JITTER,				g_modeCommands,				"Jitter test (modes 2/3)"},
	{"mlt3",			CMD_MLT3,				nullptr,					"MLT-3 idles (DP83867 only)"},
	{"pulse4",			CMD_PULSE4,				nullptr,					"Pulse, 3 zeroes (DP83867 only)"},
	{"pulse64",			CMD_PULSE64,			nullptr,					"Pulse, 63 zeroes (DP83867 only)"},
	{"waveform",		CMD_WAVEFORM_TEST,		nullptr,					"Waveform test (mode 1)"},
	{nullptr,			INVALID_COMMAND,		nullptr,					nullptr}
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// "vlan"

static const clikeyword_t g_vlanCommands[] =
{
	{"<1-4095>",	FREEFORM_TOKEN,		nullptr,	"VLAN number to assign"},
	{nullptr,		INVALID_COMMAND,	nullptr,	nullptr}
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// zeroize

static const clikeyword_t g_zeroizeCommands[] =
{
	{"all",				FREEFORM_TOKEN,			NULL,				"Confirm erasing all flash data and return to default state"},
	{NULL,				INVALID_COMMAND,		NULL,				NULL}
};
*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Top level command lists

//Top level commands in root mode
static const clikeyword_t g_rootCommands[] =
{
	/*{"clear",		CMD_CLEAR,			g_clearCommands,		"Clear performance counters and other debugging state"},
	{"commit",		CMD_COMMIT,			nullptr,				"Commit volatile config changes to flash memory"},
	{"debug",		CMD_DEBUG,			g_debugCommands,		"Enable debug output"},*/
	{"exit",		CMD_EXIT,			nullptr,				"Log out"},
	/*{"hostname",	CMD_HOSTNAME,		g_hostnameCommands,		"Change the host name"},
	{"interface",	CMD_INTERFACE,		g_interfaceCommands,	"Configure interface properties"},
	{"ip",			CMD_IP,				g_ipCommands,			"Configure IP addresses"},
	{"reload",		CMD_RELOAD,			nullptr,				"Restart the system"},
	{"rollback",	CMD_ROLLBACK,		nullptr,				"Revert all changes made since last commit"},
	*/
	{"set",			CMD_SET,			g_setCommands,			"Set raw hardware registers"},
	{"show",		CMD_SHOW,			g_showCommands,			"Print information"},/*
	{"test",		CMD_TEST,			g_interfaceCommands,	"Run a cable test"},
	{"zeroize",		CMD_ZEROIZE,		g_zeroizeCommands,		"Erase all configuration data and reload"},*/

	{nullptr,		INVALID_COMMAND,	nullptr,				nullptr}
};
/*
//Top level commands in interface mode
static const clikeyword_t g_copperInterfaceRootCommands[] =
{
	{"autonegotiation",	CMD_AUTONEGOTIATION,	nullptr,					"Enable autonegotiation"},
	{"description",		CMD_DESCRIPTION,		g_descriptionCommands,		"Set interface description"},
	{"end",				CMD_END,				nullptr,					"Return to normal mode"},
	{"exit",			CMD_EXIT,				nullptr,					"Return to normal mode"},
	{"interface",		CMD_INTERFACE,			g_interfaceCommands,		"Configure another interface"},
	{"mdi",				CMD_MDI,				g_mdiCommands,				"Specify auto or manual MDI/MDI-X mode"},
	{"mode",			CMD_MODE,				g_allModeCommands,			"Specify 1000base-T master/slave mode"},
	{"no",				CMD_NO,					g_interfaceNoCommands,		"Turn settings off"},
	{"show",			CMD_SHOW,				g_interfaceShowCommands,	"Print information"},
	{"speed",			CMD_SPEED,				g_interfaceSpeedCommands,	"Set port operating speed"},
	{"testpattern",		CMD_TESTPATTERN,		g_testpatternCommands,		"Send a test pattern"},
	{"vlan",			CMD_VLAN,				g_vlanCommands,				"Configure interface VLAN"},
	{nullptr,			INVALID_COMMAND,		nullptr,					nullptr}
};

static const clikeyword_t g_fiberInterfaceRootCommands[] =
{
	{"description",		CMD_DESCRIPTION,		g_descriptionCommands,		"Set interface description"},
	{"end",				CMD_END,				nullptr,					"Return to normal mode"},
	{"exit",			CMD_EXIT,				nullptr,					"Return to normal mode"},
	{"interface",		CMD_INTERFACE,			g_interfaceCommands,		"Configure another interface"},
	{"no",				CMD_NO,					g_interfaceNoCommands,		"Turn settings off"},
	{"set",				CMD_SET,				g_interfaceSetCommands,		"Set raw hardware registers"},
	{"show",			CMD_SHOW,				g_interfaceShowCommands,	"Print information"},
	{"vlan",			CMD_VLAN,				g_vlanCommands,				"Configure interface VLAN"},
	{nullptr,			INVALID_COMMAND,		nullptr,					nullptr}
};
*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Main CLI code

CrossbarCLISessionContext::CrossbarCLISessionContext()
	: CLISessionContext(g_rootCommands)
{
	LoadHostname();
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

		case CMD_COMMIT:
			OnCommit();
			break;

		case CMD_DEBUG:
			OnDebug();
			break;
		*/
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
		/*
		case CMD_HOSTNAME:
			strncpy(m_hostname, m_command[1].m_text, sizeof(m_hostname)-1);
			break;

		case CMD_INTERFACE:
			OnInterfaceCommand();
			break;

		case CMD_IP:
			OnIPCommand();
			break;

		case CMD_RELOAD:
			OnReload();
			break;

		case CMD_ROLLBACK:
			OnRollback();
			break;
		*/

		case CMD_SET:
			OnSetCommand();
			break;

		case CMD_SHOW:
			OnShowCommand();
			break;
		/*
		case CMD_ZEROIZE:
			if(!strcmp(m_command[1].m_text, "all"))
				OnZeroize();
			break;
		*/
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

void CrossbarCLISessionContext::OnCommit()
{
	//Save interface configuration
	for(int i=0; i<NUM_PORTS; i=i+1)
	{
		//VLAN number for everything but management
		if(i != MGMT_PORT)
		{
			if(!g_kvs->StoreObjectIfNecessary<uint16_t>(g_portVlans[i], 1, "%s.vlan", g_interfaceNames[i]))
				m_stream->Printf("KVS write error\n");
		}

		//Description
		//TODO: make string wrapper for StoreObjectIfNecessary
		if(strcmp(g_defaultInterfaceDescriptions[i], g_interfaceDescriptions[i]) != 0)
		{
			bool nameChanged = true;

			//See if the previously stored name is the same and only store if different
			auto plog = g_kvs->FindObjectF("%s.desc", g_interfaceNames[i]);
			if(plog)
			{
				auto olddesc = (const char*)g_kvs->MapObject(plog);

				char tmp[DESCRIPTION_LEN] = {0};
				auto len = plog->m_len;
				if(len >= DESCRIPTION_LEN)
					len = DESCRIPTION_LEN - 1;
				memcpy(tmp, olddesc, len);

				if(strcmp(tmp, g_interfaceDescriptions[i]) == 0)
					nameChanged = false;
			}

			if(nameChanged)
			{
				if(!g_kvs->StoreObject(
					(uint8_t*)g_interfaceDescriptions[i],
					strlen(g_interfaceDescriptions[i]),
					"%s.desc",
					g_interfaceNames[i]))
				{
					m_stream->Printf("KVS write error\n");
				}
			}
		}
	}

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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// "description"

void CrossbarCLISessionContext::OnDescription()
{
	strncpy(g_interfaceDescriptions[m_activeInterface], m_command[1].m_text, DESCRIPTION_LEN-1);
	g_interfaceDescriptions[m_activeInterface][DESCRIPTION_LEN-1] = '\0';
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// "interface"

void CrossbarCLISessionContext::OnInterfaceCommand()
{
	m_activeInterface = (m_command[1].m_commandID - CMD_G0);

	if(m_activeInterface >= NUM_PORTS)
		m_activeInterface = NUM_PORTS-1;

	if(m_activeInterface == UPLINK_PORT)
		m_rootCommands = g_fiberInterfaceRootCommands;
	else
		m_rootCommands = g_copperInterfaceRootCommands;
}

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
		m_stream->Printf("Usage: ip default-gateway x.x.x.x\n");
}

void CrossbarCLISessionContext::OnIPCommand()
{
	switch(m_command[1].m_commandID)
	{
		case CMD_ADDRESS:
			OnIPAddress(m_command[2].m_text);
			break;

		case CMD_GATEWAY:
			OnIPGateway(m_command[2].m_text);
			break;

		default:
			m_stream->Printf("Unrecognized command\n");
			break;
	}
}
*/
/*
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// "no"

void CrossbarCLISessionContext::OnNoCommand()
{
	switch(m_command[1].m_commandID)
	{
		case CMD_AUTONEGOTIATION:
			OnNoAutonegotiation();
			break;

		case CMD_SPEED:
			OnNoSpeed();
			break;

		case CMD_TESTPATTERN:
			OnNoTestPattern();
			break;

		default:
			break;
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// "reload"
*/
void CrossbarCLISessionContext::OnReload()
{
	g_log("Reload requested\n");
	SCB.AIRCR = 0x05fa0004;
	while(1)
	{}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// "rollback"

/**
	@brief Load all of our configuration from the KVS, discarding any recent changes made in the CLI
 */
/*
void CrossbarCLISessionContext::OnRollback()
{
	ConfigureInterfaces();
	ConfigureIP();
	LoadHostname();
}
*/
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
	ManagementPHYWrite(regid, value);
	m_stream->Printf("Set register 0x%02x to 0x%04x\n", regid, value);
}

void CrossbarCLISessionContext::OnSetMmdRegister()
{
	int mmd = strtol(m_command[2].m_text, nullptr, 16);
	int regid = strtol(m_command[4].m_text, nullptr, 16);
	auto value = strtol(m_command[5].m_text, nullptr, 16);
	ManagementPHYExtendedWrite(mmd, regid, value);
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

		case CMD_SSH:
			switch(m_command[2].m_commandID)
			{
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
		m_stream->Printf("        Key               Size  Revisions\n");
		int size = 0;
		for(uint32_t i=0; i<nfound; i++)
		{
			m_stream->Printf("        %-16s %5d  %d\n", list[i].key, list[i].size, list[i].revs);
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
		const char* srev = NULL;
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
		if(CPUID.CLIDR & 2)
		{
			m_stream->Printf("    L1 data cache present\n");
			CPUID.CCSELR = 0;

			int sets = ((CPUID.CCSIDR >> 13) & 0x7fff) + 1;
			int ways = ((CPUID.CCSIDR >> 3) & 0x3ff) + 1;
			int words = 1 << ((CPUID.CCSIDR & 3) + 2);
			int total = (sets * ways * words * 4) / 1024;
			m_stream->Printf("        %d sets, %d ways, %d words per line, %d kB total\n",
				sets, ways, words, total);
		}
		if(CPUID.CLIDR & 1)
		{
			m_stream->Printf("    L1 instruction cache present\n");
			CPUID.CCSELR = 1;

			int sets = ((CPUID.CCSIDR >> 13) & 0x7fff) + 1;
			int ways = ((CPUID.CCSIDR >> 3) & 0x3ff) + 1;
			int words = 1 << ((CPUID.CCSIDR & 3) + 2);
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
	auto value = ManagementPHYExtendedRead(mmd, regid);

	m_stream->Printf("MMD %02x register 0x%04x = 0x%04x\n", mmd, regid, value);
}

void CrossbarCLISessionContext::OnShowRegister()
{
	int regid = strtol(m_command[2].m_text, nullptr, 16);
	auto value = ManagementPHYRead(regid);

	m_stream->Printf("Register 0x%02x = 0x%04x\n", regid, value);
}

void CrossbarCLISessionContext::OnShowSSHFingerprint()
{
	char buf[64] = {0};
	STM32CryptoEngine tmp;
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
/*
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// "testpattern"

void CrossbarCLISessionContext::OnTestPattern()
{
	int mode = 0;
	switch(m_command[1].m_commandID)
	{
		case CMD_WAVEFORM_TEST:
			mode = 1;
			break;

		case CMD_JITTER:
			if(m_command[2].m_commandID == CMD_MASTER)
				mode = 2;
			else
				mode = 3;
			break;

		case CMD_DISTORTION:
			mode = 4;
			break;

		case CMD_PULSE4:
			if(IsActiveInterfaceDP83867())
				mode = 6;
			break;

		case CMD_PULSE64:
			if(IsActiveInterfaceDP83867())
				mode = 7;
			break;

		case CMD_MLT3:
			if(IsActiveInterfaceDP83867())
				mode = 5;
			break;
	}

	//Save previous state if we're not already in test mode
	auto oldGig = InterfacePHYRead(m_activeInterface, REG_GIG_CONTROL);
	if( (oldGig & 0xe000) == 0)
	{
		m_testModeSavedRegisters[0] = InterfacePHYRead(m_activeInterface, REG_BASIC_CONTROL);

		//KSZ9031 also needs preserving the MDIX register
		if(IsActiveInterfaceKSZ9031())
			m_testModeSavedRegisters[1] = InterfacePHYRead(m_activeInterface, REG_KSZ9031_MDIX);

		m_testModeSavedRegisters[2] = oldGig;
	}

	//Force link up, no negotiation, 1000baseT, and enable the test mode on all pairs
	InterfacePHYWrite(m_activeInterface, REG_BASIC_CONTROL, 0x0140);
	if(IsActiveInterfaceKSZ9031())
		InterfacePHYWrite(m_activeInterface, REG_KSZ9031_MDIX, 0x0040);
	if(IsActiveInterfaceDP83867())
		InterfacePHYExtendedWrite(m_activeInterface, 0x1f, REG_DP83867_TMCH_CTRL, 0x0480);
	InterfacePHYWrite(m_activeInterface, REG_GIG_CONTROL, 0x1000 | (mode << 13));

	//Update link state
	g_log("Interface %s (%s): link is sending test pattern\n",
		g_interfaceNames[m_activeInterface], g_interfaceDescriptions[m_activeInterface]);
	g_linkState[m_activeInterface] = LINK_STATE_TESTPATTERN;
}

void CrossbarCLISessionContext::OnNoTestPattern()
{
	//Restore old register values in reverse order
	InterfacePHYWrite(m_activeInterface, REG_GIG_CONTROL, m_testModeSavedRegisters[2]);
	if(IsActiveInterfaceKSZ9031())
		InterfacePHYWrite(m_activeInterface, REG_KSZ9031_MDIX, m_testModeSavedRegisters[1]);
	InterfacePHYWrite(m_activeInterface, REG_BASIC_CONTROL, m_testModeSavedRegisters[0]);

	//Restart negotiation so the link comes back up
	RestartNegotiation(m_activeInterface);

	g_log("Interface %s (%s): link is down\n",
		g_interfaceNames[m_activeInterface], g_interfaceDescriptions[m_activeInterface]);
	g_linkState[m_activeInterface] = LINK_STATE_DOWN;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// "vlan"

void CrossbarCLISessionContext::OnVlan()
{
	//Validate
	uint16_t vlanNum = atoi(m_command[1].m_text);
	if( (vlanNum < 1) || (vlanNum > 4095) )
	{
		m_stream->Printf("Invalid VLAN number\n");
		return;
	}

	//Update our local config and push to hardware
	g_portVlans[m_activeInterface] = vlanNum;
	g_fpga->BlockingWrite16(GetInterfaceBase() + REG_VLAN_NUM, vlanNum);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// "zeroize"

void CrossbarCLISessionContext::OnZeroize()
{
	g_kvs->WipeAll();
	OnReload();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Miscellaneous helpers

void CrossbarCLISessionContext::RestartNegotiation(int nport)
{
	auto base = InterfacePHYRead(nport, REG_BASIC_CONTROL);
	InterfacePHYWrite(nport, REG_BASIC_CONTROL, base | 0x0200);
}
*/
